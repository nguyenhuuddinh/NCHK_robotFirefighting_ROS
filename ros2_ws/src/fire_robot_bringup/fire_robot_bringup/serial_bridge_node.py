"""Bridge ROS 2 and raw serial V2."""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Bool
import threading
import time

from fire_robot_bringup.serial_protocol import SerialProtocolV2


class SerialBridgeNode(Node):
    """Bridge Node between ROS 2 topics and raw serial."""

    def __init__(self, serial_cls=None):
        """Initialize SerialBridgeNode."""
        super().__init__('serial_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baudrate', 115200)

        self.port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('serial_baudrate').value

        self.serial_cls = serial_cls
        if self.serial_cls is None:
            import serial
            self.serial_cls = serial.Serial

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', best_effort_qos)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', best_effort_qos)
        self.env_pub = self.create_publisher(String, '/env_status', best_effort_qos)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, best_effort_qos)
        self.create_subscription(Point, '/fire_target', self.fire_target_callback, reliable_qos)
        self.create_subscription(Bool, '/pump_cmd', self.pump_cmd_callback, reliable_qos)

        self.protocol = SerialProtocolV2()
        self.ser = None

        self.state_lock = threading.Lock()
        self.tx_lock = threading.Lock()
        self.publish_lock = threading.Lock()
        self.node_destroyed = False
        self.stop_request = False
        self._session_state = 'BACKOFF'
        self.open_generation = -1
        self.close_ownership = {
            'task': None,
            'handle': None,
            'result': None,
            'retry_deadline': 0.0,
            'backoff': 0.05,
            'generation': -1,
            'cancel_called': False,
            'deadline': None,
            'pending': [],
        }

        self.session_ready = False
        self.session_epoch = 0
        self.telemetry_healthy = False
        self.state_stale_timeout_ms = 500

        self.pending_cmd = None
        self.pending_pump = None
        self.pending_fire = None

        self.pump_state = 0
        self.cmd_version = 0
        self.pump_version = 0
        self.fire_version = 0
        self.pump_written_version = 0
        self.last_pump_time = 0.0
        self.session_started_mono = 0.0
        self.last_state_time_mono = 0.0
        self.last_env_time_mono = 0.0

        self.reconnect_backoff = 0.0
        self.next_reconnect_mono = 0.0
        self.latest_state = None
        self.latest_env = None

        self.tx_fail_count = 0
        self.tx_partial_count = 0
        self.close_fail_count = 0
        self.reconnect_count = 0
        self.reconnect_total = 0
        self.first_state_timeout_count = 0
        self.read_fail_count = 0
        self.bootstrap_fail_count = 0
        self.open_fail_count = 0
        self.open_attempt_count = 0
        self.open_success_count = 0
        self.cmd_drop_count = 0
        self.telemetry_drop_count = 0
        self.telemetry_publish_count = 0
        self.telemetry_publish_fail_count = 0
        self.last_failure_reason = 'none'

        self.state_publish_max_age = 0.0
        self.env_publish_max_age = 0.0

        self.log_timer = self.create_timer(10.0, self.log_timer_callback)
        self.telemetry_timer = self.create_timer(0.05, self.telemetry_publish_callback)

        self.running = True
        self.worker_thread = threading.Thread(target=self.serial_worker, daemon=True)
        self.worker_thread.start()

    def _transition_state(self, new_state):
        """Transition the transport state while ``state_lock`` is held."""
        valid = {
            'BACKOFF': ['OPENING', 'SHUTTING_DOWN'],
            'OPENING': ['WAIT_FIRST_STATE', 'CLOSING', 'SHUTTING_DOWN'],
            'WAIT_FIRST_STATE': ['HEALTHY', 'CLOSING', 'SHUTTING_DOWN'],
            'HEALTHY': ['CLOSING', 'SHUTTING_DOWN'],
            'CLOSING': ['BACKOFF', 'SHUTTING_DOWN'],
            'SHUTTING_DOWN': []
        }
        if new_state in valid.get(getattr(self, '_session_state', ''), []) or \
                (getattr(self, '_session_state', '') == 'CLOSING' and new_state == 'CLOSING'):
            self._session_state = new_state

    def _increment_counter(self, name, amount=1):
        """Increment a diagnostic counter atomically."""
        with self.state_lock:
            setattr(self, name, getattr(self, name, 0) + amount)

    def _record_publish_result(self, *, published=0, failed=0, dropped=0,
                               state_age=None, env_age=None):
        """Update windowed publish diagnostics atomically."""
        with self.state_lock:
            self.telemetry_publish_count += published
            self.telemetry_publish_fail_count += failed
            self.telemetry_drop_count += dropped
            if state_age is not None:
                self.state_publish_max_age = max(
                    self.state_publish_max_age, state_age)
            if env_age is not None:
                self.env_publish_max_age = max(
                    self.env_publish_max_age, env_age)

    def cmd_vel_callback(self, msg):
        """Handle /cmd_vel."""
        with self.state_lock:
            if not getattr(self, 'session_ready', False) or \
                    (not getattr(self, 'telemetry_healthy', False) and
                     (msg.linear.x != 0.0 or msg.angular.z != 0.0)):
                self.cmd_drop_count += 1
                return
            self.cmd_version += 1
            self.pending_cmd = ((msg.linear.x, msg.angular.z),
                                self.session_epoch, self.cmd_version)

    def fire_target_callback(self, msg):
        """Handle /fire_target."""
        with self.state_lock:
            if not getattr(self, 'session_ready', False) or \
                    not getattr(self, 'telemetry_healthy', False):
                self.cmd_drop_count += 1
                return
            self.fire_version += 1
            self.pending_fire = ((msg.x, msg.y), self.session_epoch, self.fire_version)

    def pump_cmd_callback(self, msg):
        """Handle /pump_cmd."""
        pump_req = 1 if msg.data else 0
        with self.state_lock:
            if not getattr(self, 'session_ready', False) or \
                    (not getattr(self, 'telemetry_healthy', False) and pump_req != 0):
                self.cmd_drop_count += 1
                return
            self.pump_version += 1
            self.pending_pump = (pump_req, self.session_epoch, self.pump_version)

    def close_serial(self):
        """Initiate or reap a serial close task."""
        with self.state_lock:
            self.session_ready = False
            self.telemetry_healthy = False
            self.pending_cmd = None
            self.pending_pump = None
            self.pending_fire = None

            if getattr(self, '_session_state', '') not in ('CLOSING', 'SHUTTING_DOWN'):
                self._transition_state('CLOSING')
                if self.reconnect_backoff <= 0.0:
                    self.reconnect_backoff = 0.1
                else:
                    self.reconnect_backoff = min(1.0, self.reconnect_backoff * 1.5)
                self.next_reconnect_mono = time.monotonic() + self.reconnect_backoff

            handle = self.ser
            self.ser = None

            if handle is not None:
                self._enqueue_close_handle_locked(
                    handle, self.session_epoch, 'close:Requested')

            self._promote_pending_close_locked()

            if self.close_ownership.get('handle') is None:
                return 'CLOSED'

            if self.close_ownership.get('result') is not None:
                res = self.close_ownership['result']
                if isinstance(res, Exception):
                    now = time.monotonic()
                    if (not getattr(self, '_shutdown_started', False)
                            and now < self.close_ownership.get('retry_deadline', 0.0)):
                        return 'IN_PROGRESS'
                    self.close_ownership['result'] = None
                    self.close_ownership['task'] = None
                    backoff = self.close_ownership.get(
                        'backoff', 0.05) * 1.5
                    self.close_ownership['backoff'] = min(
                        2.0, backoff)
                else:
                    self._clear_close_owner_locked()
                    self._promote_pending_close_locked()
                    if self.close_ownership.get('handle') is not None:
                        return 'IN_PROGRESS'
                    return 'CLOSED'

            if self.close_ownership.get('task') is None:
                shutdown_deadline = self.close_ownership.get('deadline')
                if (shutdown_deadline is not None
                        and time.monotonic() >= shutdown_deadline):
                    return 'IN_PROGRESS'

                current_handle = self.close_ownership['handle']
                current_gen = self.close_ownership['generation']

                def close_task():
                    result = None
                    with self.state_lock:
                        cancel_called = self.close_ownership.get('cancel_called', False)
                        if not cancel_called and self.close_ownership.get(
                                'generation') == current_gen:
                            self.close_ownership['cancel_called'] = True
                        else:
                            cancel_called = True

                    deadline_expired = (
                        shutdown_deadline is not None
                        and time.monotonic() >= shutdown_deadline)
                    if deadline_expired:
                        result = TimeoutError('Shutdown deadline exhausted')

                    if not cancel_called and result is None:
                        if getattr(current_handle, 'cancel_read', None):
                            try:
                                current_handle.cancel_read()
                            except Exception as e:
                                with self.state_lock:
                                    self.last_failure_reason = (
                                        f"cancel_read:{type(e).__name__}:{e}")
                        if getattr(current_handle, 'cancel_write', None):
                            try:
                                current_handle.cancel_write()
                            except Exception as e:
                                with self.state_lock:
                                    r = getattr(self, 'last_failure_reason', '')
                                    self.last_failure_reason = (
                                        f"{r} cancel_write:{type(e).__name__}:{e}")

                    tx_acq = False
                    try:
                        if result is None and hasattr(self, 'tx_lock'):
                            timeout = 0.5
                            if shutdown_deadline is not None:
                                timeout = min(
                                    timeout,
                                    max(0.0, shutdown_deadline - time.monotonic()))
                            tx_acq = self.tx_lock.acquire(timeout=timeout)
                            if not tx_acq:
                                raise TimeoutError(
                                    'Could not acquire tx_lock before deadline')
                        if (result is None and shutdown_deadline is not None
                                and time.monotonic() >= shutdown_deadline):
                            raise TimeoutError('Shutdown deadline exhausted')
                        if result is None:
                            current_handle.close()
                            result = True
                    except TimeoutError as e:
                        result = e
                    except Exception as e:
                        result = e
                    finally:
                        if tx_acq:
                            self.tx_lock.release()

                    with self.state_lock:
                        if (self.close_ownership.get('handle') is current_handle
                                and self.close_ownership.get(
                                    'generation') == current_gen):
                            self.close_ownership['result'] = result
                            if isinstance(result, Exception):
                                if not isinstance(result, TimeoutError):
                                    self.close_fail_count += 1
                                self.last_failure_reason = (
                                    f"close:{type(result).__name__}:{result}")
                                self.close_ownership['retry_deadline'] = (
                                    time.monotonic()
                                    + self.close_ownership.get(
                                        'backoff', 0.05))
                        else:
                            self.get_logger().error(
                                'Close ownership changed while close task ran')

                task_to_start = threading.Thread(
                    target=close_task, daemon=True)
                self.close_ownership['task'] = task_to_start
                task_to_start.start()

        return 'IN_PROGRESS'

    def _clear_close_owner_locked(self):
        """Clear the current close owner while ``state_lock`` is held."""
        self.close_ownership['task'] = None
        self.close_ownership['handle'] = None
        self.close_ownership['result'] = None
        self.close_ownership['retry_deadline'] = 0.0
        self.close_ownership['backoff'] = 0.05
        self.close_ownership['generation'] = -1
        self.close_ownership['cancel_called'] = False
        self.close_ownership['deadline'] = None

    def _install_close_owner_locked(self, handle, generation):
        """Install one handle as close owner while ``state_lock`` is held."""
        self.close_ownership['handle'] = handle
        self.close_ownership['generation'] = generation
        self.close_ownership['retry_deadline'] = 0.0
        self.close_ownership['backoff'] = 0.05
        self.close_ownership['result'] = None
        self.close_ownership['task'] = None
        self.close_ownership['cancel_called'] = False
        self.close_ownership['deadline'] = (
            getattr(self, '_shutdown_deadline_mono', None)
            if getattr(self, '_shutdown_started', False) else None)

    def _enqueue_close_handle_locked(self, handle, generation, reason):
        """Queue a unique handle without replacing existing ownership."""
        if reason:
            self.last_failure_reason = reason
        if handle is None or getattr(handle, 'closed', False):
            return

        owner = self.close_ownership.get('handle')
        if owner is handle:
            return
        pending = self.close_ownership['pending']
        if any(entry[0] is handle for entry in pending):
            return
        if owner is None:
            self._install_close_owner_locked(handle, generation)
        else:
            pending.append((handle, generation))

    def _promote_pending_close_locked(self):
        """Promote the next still-open quarantined handle."""
        if self.close_ownership.get('handle') is not None:
            return
        pending = self.close_ownership['pending']
        while pending:
            handle, generation = pending.pop(0)
            if not getattr(handle, 'closed', False):
                self._install_close_owner_locked(handle, generation)
                return

    def _quarantine_handle(self, handle, reason=""):
        """Quarantine a serial handle into close_ownership."""
        if handle is None:
            return

        with self.state_lock:
            if reason:
                self.last_failure_reason = reason

            self.session_ready = False
            self.telemetry_healthy = False
            self.pending_cmd = None
            self.pending_pump = None
            self.pending_fire = None

            if self.ser is handle:
                self.ser = None

            self._enqueue_close_handle_locked(
                handle, self.session_epoch, reason)

            if getattr(self, '_session_state', '') not in ('CLOSING', 'SHUTTING_DOWN'):
                self._transition_state('CLOSING')

            if self.reconnect_backoff <= 0.0:
                self.reconnect_backoff = 0.1
            else:
                self.reconnect_backoff = min(1.0, self.reconnect_backoff * 1.5)
            self.next_reconnect_mono = time.monotonic() + self.reconnect_backoff

        self.close_serial()

    def _pending_snapshot(self, gen=None):
        """Take a snapshot of pending TX items without clearing."""
        snapshot = []
        now_mono = time.monotonic()
        with self.state_lock:
            if not getattr(self, 'session_ready', False):
                return snapshot

            # Special bootstrap rule: ALLOW one CMD write even if not healthy
            is_healthy = getattr(self, 'telemetry_healthy', False)
            if not is_healthy:
                c_cmd = getattr(self, 'pending_cmd', None)
                if c_cmd:
                    args, ep, ver = c_cmd
                    if ep == getattr(self, 'session_epoch', -1):
                        snapshot.append(('CMD', args, ep, ver, c_cmd))
                return snapshot

            c_cmd = getattr(self, 'pending_cmd', None)
            if c_cmd:
                args, ep, ver = c_cmd
                if ep == getattr(self, 'session_epoch', -1):
                    snapshot.append(('CMD', args, ep, ver, c_cmd))

            c_pump = getattr(self, 'pending_pump', None)
            if c_pump:
                req, ep, ver = c_pump
                if ep == getattr(self, 'session_epoch', -1):
                    snapshot.append(('PUMP', req, ep, ver, c_pump))
            elif now_mono - getattr(self, 'last_pump_time', 0.0) >= 0.5:
                req = getattr(self, 'pump_state', 0)
                ver = getattr(self, 'pump_written_version', 0)
                ep = getattr(self, 'session_epoch', -1)
                snapshot.append(('PUMP_REFRESH', req, ep, ver, None))

            c_fire = getattr(self, 'pending_fire', None)
            if c_fire:
                args, ep, ver = c_fire
                if ep == getattr(self, 'session_epoch', -1):
                    snapshot.append(('FIRE', args, ep, ver, c_fire))

        return snapshot

    def _write_pending_item(self, item, current_ser):
        """Format and write pending item to serial."""
        itype, data, ep, ver, original_tuple = item
        gen = ep
        frame = None
        with self.state_lock:
            if getattr(self, 'ser', None) is not current_ser or self.session_epoch != gen:
                if itype == 'CMD':
                    if getattr(self, 'pending_cmd', None) is original_tuple:
                        self.pending_cmd = None
                        self.cmd_drop_count += 1
                    return True
                if itype == 'PUMP' and data != 0:
                    if getattr(self, 'pending_pump', None) is original_tuple:
                        self.pending_pump = None
                    return True
                if itype == 'FIRE' or itype == 'PUMP_REFRESH':
                    if itype == 'FIRE' and getattr(self, 'pending_fire', None) is original_tuple:
                        self.pending_fire = None
                    return True

            if not getattr(self, 'telemetry_healthy', False):
                if itype == 'CMD' and data != (0.0, 0.0):
                    if getattr(self, 'pending_cmd', None) is original_tuple:
                        self.pending_cmd = None
                        self.cmd_drop_count += 1
                    return True
                if itype == 'PUMP' and data != 0:
                    if getattr(self, 'pending_pump', None) is original_tuple:
                        self.pending_pump = None
                    return True
                if itype == 'FIRE':
                    if getattr(self, 'pending_fire', None) is original_tuple:
                        self.pending_fire = None
                    return True

            if itype == 'CMD':
                current_item = getattr(self, 'pending_cmd', None)
                if current_item is not original_tuple and current_item is not None:
                    c_args, c_ep, c_ver = current_item
                    if c_ver > ver:
                        return True
                frame = self.protocol.generate_cmd(data[0], data[1])
            elif itype == 'PUMP':
                current_item = getattr(self, 'pending_pump', None)
                if current_item is not original_tuple and current_item is not None:
                    c_req, c_ep, c_ver = current_item
                    if c_ver > ver:
                        return True
                frame = self.protocol.generate_pump(data)
            elif itype == 'PUMP_REFRESH':
                current_item = getattr(self, 'pending_pump', None)
                if current_item is not None:
                    c_req, c_ep, c_ver = current_item
                    if c_ver > ver:
                        return True
                frame = self.protocol.generate_pump(data)
            elif itype == 'FIRE':
                current_item = getattr(self, 'pending_fire', None)
                if current_item is not original_tuple and current_item is not None:
                    c_args, c_ep, c_ver = current_item
                    if c_ver > ver:
                        return True
                frame = self.protocol.generate_fire(data[0], data[1])

        if not frame:
            return True

        if getattr(self, 'stop_request', False):
            return True

        try:
            written = current_ser.write(frame)
            if written < len(frame):
                self._increment_counter('tx_partial_count')
                self._quarantine_handle(current_ser, f"tx:{itype}:Exception:Partial write")
                return False
        except Exception as e:
            with self.state_lock:
                if self.ser is current_ser:
                    self.tx_fail_count += 1
            self._quarantine_handle(current_ser, f"tx:{itype}:{type(e).__name__}:{e}")
            return False

        with self.state_lock:
            if itype == 'CMD':
                if getattr(self, 'pending_cmd', None) is original_tuple:
                    self.pending_cmd = None
            elif itype == 'PUMP':
                if getattr(self, 'pending_pump', None) is original_tuple:
                    self.pending_pump = None
                self.pump_state = data
                self.pump_written_version = ver
                self.last_pump_time = time.monotonic()
            elif itype == 'PUMP_REFRESH':
                self.last_pump_time = time.monotonic()
            elif itype == 'FIRE':
                if getattr(self, 'pending_fire', None) is original_tuple:
                    self.pending_fire = None

        return True

    def serial_worker(self):
        """Worker thread for reading and writing."""
        self.get_logger().info("Serial worker thread started.")
        while not getattr(self, 'stop_request', False):
            with self.state_lock:
                state = getattr(self, '_session_state', 'BACKOFF')
                exact_handle = self.ser
                gen = self.session_epoch
                has_close_task = self.close_ownership.get('task') is not None

            if state == 'SHUTTING_DOWN':
                break

            if state == 'CLOSING' or has_close_task:
                res = self.close_serial()
                if res != 'CLOSED':
                    time.sleep(0.01)
                else:
                    with self.state_lock:
                        if getattr(self, '_session_state', '') == 'CLOSING':
                            self._transition_state('BACKOFF')
                continue

            if state == 'BACKOFF':
                now = time.monotonic()
                if now < getattr(self, 'next_reconnect_mono', 0.0):
                    time.sleep(0.01)
                    continue
                with self.state_lock:
                    if (getattr(self, 'stop_request', False)
                            or getattr(self, '_session_state', '')
                            == 'SHUTTING_DOWN'):
                        break
                    self.session_epoch += 1
                    self.open_generation = self.session_epoch
                    self._transition_state('OPENING')
                continue

            if state == 'OPENING':
                self._increment_counter('open_attempt_count')
                try:
                    new_ser = self.serial_cls(
                        self.port, self.baudrate,
                        timeout=0.01, write_timeout=0.1)
                except Exception as e:
                    self.get_logger().warn(
                        f"Failed to open port {self.port}: {e}")
                    with self.state_lock:
                        self.open_fail_count += 1
                        self.last_failure_reason = (
                            f"open:{type(e).__name__}:{e}")
                        self.reconnect_backoff = min(
                            1.0, max(0.1, self.reconnect_backoff * 1.5))
                        self.next_reconnect_mono = (
                            time.monotonic() + self.reconnect_backoff)
                        self._transition_state('BACKOFF')
                    continue

                try:
                    cmd = self.protocol.generate_cmd(0.0, 0.0)
                    pump = self.protocol.generate_pump(0)

                    if cmd:
                        w1 = new_ser.write(cmd)
                        if w1 < len(cmd):
                            raise Exception("Partial write during bootstrap (CMD)")
                    if getattr(self, 'stop_request', False):
                        raise Exception("Bootstrap interrupted by shutdown")
                    if pump:
                        w2 = new_ser.write(pump)
                        if w2 < len(pump):
                            raise Exception("Partial write during bootstrap (PUMP)")
                    if getattr(self, 'stop_request', False):
                        raise Exception("Bootstrap interrupted by shutdown")
                except Exception as e:
                    self._increment_counter('bootstrap_fail_count')
                    self._quarantine_handle(
                        new_ser,
                        f"bootstrap:{type(e).__name__}:{e}")
                    continue

                reject_reason = None
                with self.state_lock:
                    if (getattr(self, 'stop_request', False)
                            or getattr(self, '_session_state', '')
                            in ('CLOSING', 'SHUTTING_DOWN')
                            or self.session_epoch
                            != getattr(self, 'open_generation', -1)
                            or getattr(self, 'ser', None) is not None
                            or self.close_ownership.get('handle')
                            is not None
                            or self.close_ownership['pending']):
                        reject_reason = 'bootstrap:Abort:State changed'
                    else:
                        now = time.monotonic()
                        self.open_success_count += 1
                        self.reconnect_total += 1
                        self.reconnect_count += 1
                        self.ser = new_ser
                        self.session_ready = True
                        self.telemetry_healthy = False
                        self.session_started_mono = now
                        self.last_state_time_mono = 0.0
                        self.last_pump_time = now
                        self.pump_state = 0
                        self._transition_state('WAIT_FIRST_STATE')

                if reject_reason is not None:
                    self._quarantine_handle(new_ser, reject_reason)
                    continue

                self.protocol.reset_parser()
                self.get_logger().info(f"Opened serial port {self.port}")

                continue

            if exact_handle is None:
                time.sleep(0.01)
                continue

            try:
                data = exact_handle.read(1024)
                if data:
                    for frame in self.protocol.parse_chunk(data):
                        self.handle_frame(frame)
            except Exception as e:
                self._increment_counter('read_fail_count')
                self._quarantine_handle(exact_handle, f"read:{type(e).__name__}:{e}")
                continue

            items = self._pending_snapshot(gen)
            if items:
                tx_acquired = self.tx_lock.acquire(timeout=0.01)
                if tx_acquired:
                    try:
                        for item in items:
                            res = self._write_pending_item(item, exact_handle)
                            if not res:
                                # exact_handle is already quarantined by _write_pending_item!
                                break
                    finally:
                        self.tx_lock.release()
            else:
                time.sleep(0.005)

            timeout_reason = None
            now = time.monotonic()
            with self.state_lock:
                same_session = (
                    self.ser is exact_handle and self.session_epoch == gen)
                current_state = self._session_state
                if (same_session and current_state == 'WAIT_FIRST_STATE'
                        and now - self.session_started_mono > 2.0):
                    self.first_state_timeout_count += 1
                    timeout_reason = 'first_state:Timeout'
                elif (same_session and current_state == 'HEALTHY'
                      and now - self.last_state_time_mono > 0.5):
                    timeout_reason = 'healthy:Timeout'
            if timeout_reason is not None:
                self._quarantine_handle(exact_handle, timeout_reason)

        self.get_logger().info("Serial worker thread exited.")

    def handle_frame(self, frame):
        """Handle parsed telemetry frame."""
        now_mono = time.monotonic()
        with self.state_lock:
            if not self.session_ready:
                return
            epoch = self.session_epoch

            if frame['type'] == 'STATE':
                self.last_state_time_mono = now_mono
                self.telemetry_healthy = True
                self.reconnect_backoff = 0.0
                self.next_reconnect_mono = 0.0
                self.latest_state = (frame, epoch, now_mono)
                self._transition_state('HEALTHY')
            elif frame['type'] == 'ENV':
                self.last_env_time_mono = now_mono
                self.latest_env = (frame, epoch, now_mono)

    def telemetry_publish_callback(self):
        """Publish telemetry from latest slots."""
        with self.publish_lock:
            if getattr(self, 'node_destroyed', False):
                return

        with self.state_lock:
            state_item = self.latest_state
            self.latest_state = None
            env_item = self.latest_env
            self.latest_env = None

        if state_item:
            frame, epoch, frame_mono = state_item
            age = time.monotonic() - frame_mono

            with self.state_lock:
                valid_odom = (self.running and self.session_ready and
                              self.telemetry_healthy and self.session_epoch == epoch)

            if valid_odom and age < 0.2:
                try:
                    now = self.get_clock().now().to_msg()
                    odom = Odometry()
                    odom.header.stamp = now
                    odom.header.frame_id = 'odom'
                    odom.child_frame_id = 'base_link'
                    odom.pose.pose.position.x = frame['x']
                    odom.pose.pose.position.y = frame['y']
                    odom.pose.pose.position.z = 0.0

                    cy = math.cos(frame['yaw'] * 0.5)
                    sy = math.sin(frame['yaw'] * 0.5)
                    odom.pose.pose.orientation.x = 0.0
                    odom.pose.pose.orientation.y = 0.0
                    odom.pose.pose.orientation.z = sy
                    odom.pose.pose.orientation.w = cy

                    odom.twist.twist.linear.x = frame['vx']
                    odom.twist.twist.angular.z = frame['wz']

                    with self.publish_lock:
                        if not getattr(self, 'node_destroyed', False):
                            self.odom_pub.publish(odom)
                    self._record_publish_result(
                        published=1, state_age=age)
                except Exception as e:
                    self.get_logger().warn(f"Odom publish exception: {e}")
                    self._record_publish_result(failed=1)
            else:
                self._record_publish_result(dropped=1)

            with self.state_lock:
                valid_imu = (self.running and self.session_ready and
                             self.telemetry_healthy and self.session_epoch == epoch)

            if valid_imu and age < 0.2:
                try:
                    # Reuse 'now', but don't overwrite it since they should match
                    imu = Imu()
                    imu.header.stamp = now
                    imu.header.frame_id = 'imu_frame'
                    imu.orientation.x = 0.0
                    imu.orientation.y = 0.0
                    imu.orientation.z = sy
                    imu.orientation.w = cy
                    imu.angular_velocity.z = frame['gyro_z']
                    with self.publish_lock:
                        if not getattr(self, 'node_destroyed', False):
                            self.imu_pub.publish(imu)
                    self._record_publish_result(published=1)
                except Exception as e:
                    self.get_logger().warn(f"Imu publish exception: {e}")
                    self._record_publish_result(failed=1)
            else:
                self._record_publish_result(dropped=1)

        if env_item:
            frame, epoch, frame_mono = env_item
            age = time.monotonic() - frame_mono

            with self.state_lock:
                valid_env = (self.running and self.session_ready and
                             self.telemetry_healthy and self.session_epoch == epoch)

            if valid_env and age < 1.0:
                try:
                    msg = String()
                    if frame['valid'] == 0:
                        msg.data = json.dumps({"status": "WROOM_OFFLINE"})
                    else:
                        fire_str = f"{frame['fire_flags']:03d}"
                        msg.data = json.dumps({
                            "fire": fire_str,
                            "gas": frame['gas_ppm'],
                            "temp": frame['temp_c'],
                            "batt": frame['batt_v']
                        })
                    with self.publish_lock:
                        if not getattr(self, 'node_destroyed', False):
                            self.env_pub.publish(msg)
                    self._record_publish_result(
                        published=1, env_age=age)
                except Exception as e:
                    self.get_logger().warn(f"Env publish exception: {e}")
                    self._record_publish_result(failed=1)
            else:
                self._record_publish_result(dropped=1)

    def log_timer_callback(self):
        """Log diagnostics."""
        with getattr(self, 'state_lock', threading.Lock()):
            state = getattr(self, '_session_state', 'BACKOFF')
            drop = getattr(self, 'cmd_drop_count', 0)
            self.cmd_drop_count = 0
            t_drop = getattr(self, 'telemetry_drop_count', 0)
            self.telemetry_drop_count = 0
            t_pub = getattr(self, 'telemetry_publish_count', 0)
            self.telemetry_publish_count = 0
            t_fail = getattr(self, 'telemetry_publish_fail_count', 0)
            self.telemetry_publish_fail_count = 0
            tx_p = getattr(self, 'tx_partial_count', 0)
            self.tx_partial_count = 0
            tx_f = getattr(self, 'tx_fail_count', 0)
            self.tx_fail_count = 0
            s_max = getattr(self, 'state_publish_max_age', 0.0)
            self.state_publish_max_age = 0.0
            e_max = getattr(self, 'env_publish_max_age', 0.0)
            self.env_publish_max_age = 0.0

            open_attempt = getattr(self, 'open_attempt_count', 0)
            open_succ = getattr(self, 'open_success_count', 0)
            open_fail = getattr(self, 'open_fail_count', 0)
            first_state_timeout = getattr(self, 'first_state_timeout_count', 0)
            recon = getattr(self, 'reconnect_count', 0)
            self.reconnect_count = 0
            recon_total = getattr(self, 'reconnect_total', 0)
            read_f = getattr(self, 'read_fail_count', 0)
            boot_f = getattr(self, 'bootstrap_fail_count', 0)
            close_f = getattr(self, 'close_fail_count', 0)

        self.get_logger().info(
            f"state={state} "
            f"open_attempt_count={open_attempt} "
            f"open_success_count={open_succ} "
            f"open_fail_count={open_fail} "
            f"first_state_timeout_count={first_state_timeout} "
            f"reconnect_count={recon} "
            f"reconnect_total={recon_total} "
            f"read_fail_count={read_f} "
            f"bootstrap_fail_count={boot_f} "
            f"close_fail_count={close_f} "
            f"cmd_drop_count={drop} "
            f"telemetry_drop_count={t_drop} "
            f"telemetry_publish_count={t_pub} "
            f"telemetry_publish_fail_count={t_fail} "
            f"tx_partial_count={tx_p} "
            f"tx_fail_count={tx_f} "
            f"state_publish_max_age={s_max:.3f} "
            f"env_publish_max_age={e_max:.3f}"
        )

    def destroy_node(self):
        """Shutdown the node safely."""
        if getattr(self, 'node_destroyed', False):
            return True

        timers_to_cancel = []
        with getattr(self, 'state_lock', threading.Lock()):
            if not getattr(self, '_shutdown_started', False):
                self._shutdown_started = True
                self._shutdown_deadline_mono = time.monotonic() + 3.5
                self._transition_state('SHUTTING_DOWN')
                self.stop_request = True
                self.running = False
                self.session_ready = False
                self.telemetry_healthy = False
                self.session_epoch += 1
                self.pending_cmd = None
                self.pending_pump = None
                self.pending_fire = None
                self.latest_state = None
                self.latest_env = None
                self.session_started_mono = 0.0
                self.last_state_time_mono = 0.0
                self.close_ownership['deadline'] = (
                    self._shutdown_deadline_mono)
                timers_to_cancel = [
                    timer for timer in (
                        getattr(self, 'telemetry_timer', None),
                        getattr(self, 'log_timer', None))
                    if timer is not None]

        for timer in timers_to_cancel:
            try:
                timer.cancel()
            except Exception as e:
                self.get_logger().warn(f"Failed to cancel timer: {e}")

        deadline = getattr(self, '_shutdown_deadline_mono', time.monotonic())

        needs_close = False
        with self.state_lock:
            if (getattr(self, 'ser', None) is not None
                    or self.close_ownership.get('handle') is not None
                    or self.close_ownership.get('pending')):
                needs_close = True

        res = 'CLOSED'
        if needs_close:
            res = self.close_serial()
            if res != 'CLOSED':
                task = self.close_ownership.get('task')
                if task is not None and task is not threading.current_thread():
                    remaining = max(0.0, deadline - time.monotonic())
                    task.join(timeout=min(0.05, remaining))
                res = self.close_serial()
                if res != 'CLOSED':
                    return False

        if hasattr(self, 'worker_thread') and self.worker_thread.is_alive():
            if self.worker_thread is not threading.current_thread():
                remaining = max(0.0, deadline - time.monotonic())
                self.worker_thread.join(timeout=remaining)
                if self.worker_thread.is_alive():
                    if time.monotonic() >= deadline:
                        self.get_logger().error(
                            "Teardown budget exhausted, failing closed (worker thread).")
                    return False

        pl = getattr(self, 'publish_lock', None)
        if pl:
            remaining = max(0.0, deadline - time.monotonic())
            if not pl.acquire(timeout=remaining):
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        "Teardown budget exhausted, failing closed (publish_lock).")
                return False
            pl.release()

        tl = getattr(self, 'tx_lock', None)
        if tl:
            remaining = max(0.0, deadline - time.monotonic())
            if not tl.acquire(timeout=remaining):
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        "Teardown budget exhausted, failing closed (tx_lock).")
                return False
            tl.release()

        try:
            ret = super().destroy_node()
        except Exception:
            return False
        success = ret is None or ret is True
        if success:
            self.node_destroyed = True
        return success


def main(args=None):
    """Run main entry point."""
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        deadline = time.monotonic() + 3.7
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if node.destroy_node():
                break
            time.sleep(min(0.1, remaining))

        # Final nonblocking reap attempt
        if not getattr(node, 'node_destroyed', False):
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
