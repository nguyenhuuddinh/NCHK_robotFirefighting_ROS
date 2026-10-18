"""Bounded local IPC; the ROS process never owns the serial file descriptor.

No ROS imports here. A separate interpreter owns the existing transport state
machine. A lost parent lease latches active commands OFF until session restart.
"""
import json
import logging
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time

from fire_robot_bringup.serial_transport_core import SerialTransportCore

PACKET_LIMIT = 32768
COMMAND_TTL = 0.2
PARENT_LEASE = 0.5
SLOTS = ('latest_state', 'latest_env', 'latest_ir', 'latest_actuator',
         'latest_usb_diagnostic')


def channel_pair():
    pair = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    for endpoint in pair:
        endpoint.setblocking(False)
        endpoint.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, PACKET_LIMIT * 2)
    return pair


def send_packet(endpoint, payload):
    data = json.dumps(payload, allow_nan=False, separators=(',', ':')).encode()
    if len(data) > PACKET_LIMIT:
        raise ValueError('IPC packet exceeds bounded size')
    try:
        count = endpoint.send(data)
    except BlockingIOError:
        return False
    if count != len(data):
        raise OSError('Partial IPC packet')
    return True


def receive_packets(endpoint):
    """Bound work per tick. EOF/truncation/malformed packets are faults."""
    result = []
    for _ in range(64):
        try:
            data, _, flags, _ = endpoint.recvmsg(PACKET_LIMIT)
        except BlockingIOError:
            break
        if not data:
            raise EOFError('IPC peer closed')
        if flags & socket.MSG_TRUNC:
            raise ValueError('Truncated IPC packet')
        value = json.loads(data)
        if not isinstance(value, dict):
            raise ValueError('IPC object required')
        result.append(value)
    return result


def fresh(stamp, now, limit=COMMAND_TTL):
    return (isinstance(stamp, (int, float)) and not isinstance(stamp, bool)
            and math.isfinite(stamp) and 0 <= now - stamp <= limit)


def safe_command(kind, values):
    return ((kind == 'CMD' and values == [0.0, 0.0])
            or (kind == 'PUMP' and values == [0]))


class ProcessTransport(SerialTransportCore):
    def __init__(self, config, serial_cls):
        self.logger = logging.getLogger('serial_transport')
        self.init_transport(config['port'], config['baudrate'],
                            config['first_state_timeout_s'],
                            config['pump_command_timeout_s'], serial_cls,
                            diagnostics_enabled=config.get(
                                'enable_diagnostics', True),
                            fault_trace_events=config.get(
                                'usb_fault_trace_events', 0))
        self.ipc_fault = ''
        self.last_heartbeat = time.monotonic()
        self.deadlines = {}
        self.last_command_id = 0
        self.last_safe_enqueue = 0.0

    def get_logger(self):
        return self.logger

    def _latch_locked(self, reason):
        if not self.ipc_fault:
            self.ipc_fault = reason
        self.deadlines.clear()
        self.pending_fire = None
        # Discard pending active values immediately, even when USB is closing.
        self.cmd_version += 1
        self.pending_cmd = ((0.0, 0.0), self.session_epoch, self.cmd_version)
        self.pump_version += 1
        self.pending_pump = (0, self.session_epoch, self.pump_version)
        self.last_pump_command_time = None
        self.last_safe_enqueue = time.monotonic()

    def fail_closed(self, reason):
        with self.state_lock:
            self._latch_locked(reason)

    def check_lease(self):
        with self.state_lock:
            if time.monotonic() - self.last_heartbeat > PARENT_LEASE:
                if not self.ipc_fault:
                    self._latch_locked('parent_lease_expired')
            return self.ipc_fault

    def heartbeat(self, stamp):
        now = time.monotonic()
        with self.state_lock:
            # Check before updating: late queued heartbeats cannot heal a fault.
            if now - self.last_heartbeat > PARENT_LEASE and not self.ipc_fault:
                self._latch_locked('parent_lease_expired')
            if fresh(stamp, now):
                self.last_heartbeat = max(self.last_heartbeat, stamp)

    def accept_command(self, packet):
        now = time.monotonic()
        kind, values = packet.get('kind'), packet.get('values')
        if kind not in ('CMD', 'PUMP', 'FIRE') or not isinstance(values, list):
            raise ValueError('Invalid command packet')
        if len(values) != (1 if kind == 'PUMP' else 2):
            raise ValueError('Invalid command dimensions')
        if any(not isinstance(v, (float, int)) or isinstance(v, bool)
               or not math.isfinite(v) for v in values):
            raise ValueError('Nonfinite command')
        if kind == 'PUMP' and values[0] not in (0, 1):
            raise ValueError('Invalid pump command')
        command_id = packet.get('id')
        if not isinstance(command_id, int) or isinstance(command_id, bool):
            raise ValueError('Invalid command id')
        with self.state_lock:
            if now - self.last_heartbeat > PARENT_LEASE and not self.ipc_fault:
                self._latch_locked('parent_lease_expired')
            if command_id <= self.last_command_id:
                return False
            self.last_command_id = command_id
            if (not fresh(packet.get('mono'), now)
                    or packet.get('epoch') != self.session_epoch
                    or not self.session_ready
                    or ((self.ipc_fault or not self.telemetry_healthy)
                        and not safe_command(kind, values))):
                self.cmd_drop_count += 1
                return False
            field, version = {'CMD': ('pending_cmd', 'cmd_version'),
                              'PUMP': ('pending_pump', 'pump_version'),
                              'FIRE': ('pending_fire', 'fire_version')}[kind]
            ver = getattr(self, version) + 1
            setattr(self, version, ver)
            value = values[0] if kind == 'PUMP' else tuple(values)
            setattr(self, field, (value, self.session_epoch, ver))
            self.deadlines[kind] = (self.session_epoch, ver, packet['mono'] + COMMAND_TTL)
            if kind == 'PUMP':
                self.last_pump_command_time = packet['mono']
            return True

    def _pending_snapshot(self, gen=None):
        self.check_lease()
        with self.state_lock:
            if self.ipc_fault and time.monotonic() - self.last_safe_enqueue >= 0.1:
                self._latch_locked(self.ipc_fault)
        items = super()._pending_snapshot(gen)
        with self.state_lock:
            # Existing bootstrap path only returns CMD while unhealthy. OFF is
            # also required during an IPC fault, without permitting any ON.
            if self.ipc_fault and self.session_ready and not any(i[0] == 'PUMP' for i in items):
                p = self.pending_pump
                if p is not None:
                    items.append(('PUMP', p[0], p[1], p[2], p))
        return items

    def _transport_command_allowed_locked(self, item):
        kind, value, epoch, version, _ = item
        now = time.monotonic()
        if now - self.last_heartbeat > PARENT_LEASE and not self.ipc_fault:
            self._latch_locked('parent_lease_expired')
        if (kind == 'CMD' and value == (0.0, 0.0)) or (kind in ('PUMP', 'PUMP_REFRESH') and value == 0):
            return True
        if self.ipc_fault:
            return False
        if kind == 'PUMP_REFRESH':
            return (self.last_pump_command_time is not None and
                    now - self.last_pump_command_time <= self.pump_command_timeout_s)
        return self.deadlines.get(kind, ())[:2] == (epoch, version) and now <= self.deadlines[kind][2]

    def snapshot(self, sequence):
        with self.state_lock:
            return dict(type='status', sequence=sequence, mono=time.monotonic(),
                        epoch=self.session_epoch, ready=self.session_ready,
                        healthy=self.telemetry_healthy, fault=self.ipc_fault,
                        state=self._session_state,
                        slots={name: getattr(self, name) for name in SLOTS},
                        read_max_ms=self.read_max_ms,
                        worker_gap_max_ms=self.worker_gap_max_ms)


def worker_main(endpoint, config, serial_cls=None):
    endpoint.setblocking(False)
    if serial_cls is None:
        import serial
        serial_cls = serial.Serial
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s [%(name)s] %(levelname)s %(message)s')
    transport = ProcessTransport(config, serial_cls)
    parent_pid = os.getppid()
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    transport.worker_thread = threading.Thread(target=transport.serial_worker, daemon=True)
    sequence, awaiting_ack, next_status = 0, None, 0.0
    next_log = (time.monotonic() + 10
                if config.get('enable_diagnostics', True) else None)
    transport.worker_thread.start()
    try:
        while not stopping:
            if os.getppid() != parent_pid:
                transport.fail_closed('parent_exited')
                break
            transport.check_lease()
            commands = []
            for packet in receive_packets(endpoint):
                kind = packet.get('type')
                if kind == 'stop':
                    stopping = True
                elif kind == 'heartbeat':
                    transport.heartbeat(packet.get('mono'))
                elif kind == 'ack':
                    if packet.get('sequence') == awaiting_ack:
                        awaiting_ack = None
                elif kind == 'command':
                    commands.append(packet)
                else:
                    raise ValueError('Unknown IPC packet')
            if stopping:
                transport.fail_closed('stop_requested')
            # Safe values dominate active values for their kind in this batch.
            safe_kinds = {c.get('kind') for c in commands if safe_command(c.get('kind'), c.get('values'))
                          and fresh(c.get('mono'), time.monotonic())}
            for command in commands:
                if command.get('kind') in safe_kinds and not safe_command(command.get('kind'), command.get('values')):
                    continue
                transport.accept_command(command)
            now = time.monotonic()
            if awaiting_ack is None and now >= next_status:
                sequence += 1
                snapshot = transport.snapshot(sequence)
                if send_packet(endpoint, snapshot):
                    awaiting_ack = sequence
                    with transport.state_lock:
                        for name, item in snapshot['slots'].items():
                            if getattr(transport, name) is item:
                                setattr(transport, name, None)
                next_status = now + 0.02
            if next_log is not None and now >= next_log:
                transport.log_timer_callback()
                next_log = now + 10
            time.sleep(0.005)
    except (EOFError, OSError, ValueError) as exc:
        transport.fail_closed('ipc_error:' + type(exc).__name__)
        transport.get_logger().error(str(exc))
    finally:
        # Best effort explicit safe writes before bounded close. Firmware
        # watchdog remains the fallback if USB itself cannot accept writes.
        transport.fail_closed('shutdown')
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            time.sleep(0.01)
        transport.shutdown_transport()
        endpoint.close()


class SerialProcessClient:
    def __init__(self, config):
        self.endpoint, child = channel_pair()
        self.process = None
        self.closed = False
        self.fault = ''
        self.command_id = 0
        self.status = None
        try:
            self.process = subprocess.Popen(
                [sys.executable, '-m', 'fire_robot_bringup.serial_process',
                 str(child.fileno()), json.dumps(config)],
                pass_fds=(child.fileno(),), stdin=subprocess.DEVNULL,
                start_new_session=True)
        except BaseException:
            self.endpoint.close()
            raise
        finally:
            child.close()

    def _send(self, value):
        if self.closed or self.fault:
            return False
        try:
            if self.process.poll() is not None:
                raise EOFError('serial worker exited')
            if not send_packet(self.endpoint, value):
                raise OSError('command IPC congested')
            return True
        except (OSError, EOFError, ValueError) as exc:
            self.fault = str(exc)
            self.endpoint.close()  # EOF forces worker safe shutdown.
            return False

    def heartbeat(self):
        return self._send(dict(type='heartbeat', mono=time.monotonic()))

    def command(self, kind, values, epoch):
        self.command_id += 1
        return self._send(dict(type='command', id=self.command_id, mono=time.monotonic(),
                               kind=kind, values=values, epoch=epoch))

    def poll(self):
        if self.closed or self.fault:
            return None
        try:
            if self.process.poll() is not None:
                raise EOFError('serial worker exited')
            for value in receive_packets(self.endpoint):
                if value.get('type') != 'status':
                    raise ValueError('Invalid worker status')
                self.status = value
                self._send(dict(type='ack', sequence=value['sequence']))
            return self.status
        except (EOFError, OSError, ValueError, KeyError) as exc:
            self.fault = str(exc)
            self.endpoint.close()
            return None

    def close(self):
        if self.closed:
            return
        self._send(dict(type='stop'))
        self.closed = True
        self.endpoint.close()
        if self.process is not None:
            try:
                self.process.wait(timeout=3.7)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=0.5)


if __name__ == '__main__':
    worker_main(socket.socket(fileno=int(sys.argv[1])), json.loads(sys.argv[2]))
