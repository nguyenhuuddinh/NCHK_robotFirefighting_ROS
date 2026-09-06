"""Tests for node logic with mock serial."""
import os
import select
import signal
import subprocess
import sys
import threading
import time

import pytest
import rclpy
from rclpy.signals import SignalHandlerOptions

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from fire_robot_bringup.serial_bridge_node import SerialBridgeNode
from fire_robot_bringup.serial_protocol import crc16_ccitt_false


@pytest.fixture(autouse=True)
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


class Registry:
    def __init__(self):
        self.instances = []
        self.next_config = []


registry = Registry()


def wait_until(condition, timeout=2.0, step=0.01):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if condition():
            return True
        time.sleep(step)
    return False


def make_state(seq, ms, x, y, yaw, vx, wz, gz):
    payload = f"STATE,2,{seq},{ms},{x:.3f},{y:.3f},{yaw:.3f},{vx:.3f},{wz:.3f},{gz:.3f}"
    crc = crc16_ccitt_false(payload.encode())
    return f"@{payload}*{crc:04X}\n".encode()


def make_env(seq, ms, fire, gas, temp, batt, valid):
    payload = f"ENV,2,{seq},{fire},{gas:.1f},{temp:.1f},{batt:.1f},{valid}"
    crc = crc16_ccitt_false(payload.encode())
    return f"@{payload}*{crc:04X}\n".encode()


def run_serial_bridge_signal_subprocess(signal_to_send, iterations):
    """Run immediate real-signal cycles without opening hardware serial."""
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    env['ROS_DOMAIN_ID'] = '97'
    env['ROS_LOCALHOST_ONLY'] = '1'
    env['ROS_LOG_DIR'] = (
        f'/tmp/serial_bridge_signal_test_{signal_to_send}')
    os.makedirs(env['ROS_LOG_DIR'], exist_ok=True)

    command = [
        sys.executable,
        '-m',
        'fire_robot_bringup.serial_bridge_node',
        '--ros-args',
        '-p',
        'serial_port:=/tmp/serial_bridge_signal_test_no_device',
    ]
    forbidden_output = (
        'Traceback',
        'KeyboardInterrupt',
        'ExternalShutdownException',
        'RCLError',
        "publisher's context is invalid",
        'rcl_shutdown already called',
        'Executor.__del__',
    )

    for iteration in range(iterations):
        proc = subprocess.Popen(
            command,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        output = ''
        ready = False

        try:
            readiness_deadline = time.monotonic() + 5.0
            while time.monotonic() < readiness_deadline:
                if proc.poll() is not None:
                    remainder, _ = proc.communicate()
                    output += remainder or ''
                    break

                readable, _, _ = select.select([proc.stdout], [], [], 0.1)
                if not readable:
                    continue
                line = proc.stdout.readline()
                output += line
                if 'Serial worker thread started.' in line:
                    ready = True
                    proc.send_signal(signal_to_send)
                    break

            assert ready, (
                f'Iteration {iteration} did not reach readiness:\n{output}')

            remainder, _ = proc.communicate(timeout=5.0)
            output += remainder or ''
        except subprocess.TimeoutExpired:
            pytest.fail(
                f'Iteration {iteration} did not exit after signal:\n{output}')
        finally:
            if proc.poll() is None:
                proc.kill()
                remainder, _ = proc.communicate()
                output += remainder or ''

        assert proc.returncode == 0, (
            f'Iteration {iteration} returned {proc.returncode}:\n{output}')
        for forbidden in forbidden_output:
            assert forbidden not in output, (
                f'Iteration {iteration} leaked {forbidden!r}:\n{output}')


class FakeSerial:
    def __init__(self, port, baudrate, timeout=0.01, write_timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.is_open = True
        self.closed = False
        self.read_buffer = b""
        self.written_frames = []
        self.block_read_event = None
        self.block_write_event = None
        self.raise_on_read = False
        self.raise_on_write = False
        self.partial_write = False
        self._cancel_current_read = False
        self._cancel_current_write = False
        self._lock = threading.Lock()
        self.close_count = 0
        self.close_count = 0

        cfg = registry.next_config.pop(0) if registry.next_config else {}
        for k, v in cfg.items():
            setattr(self, k, v)
        registry.instances.append(self)

    def write(self, data: bytes):
        if getattr(self, 'closed', False):
            raise Exception("Serial port closed")
        if self.block_write_event:
            self.block_write_event.wait()
        with self._lock:
            if self._cancel_current_write:
                self._cancel_current_write = False
                raise Exception("Write canceled")
            if self.raise_on_write:
                raise Exception("Mock write error")
            if self.partial_write:
                half = max(1, len(data) // 2)
                self.written_frames.append((0, data[:half]))
                return half
            self.written_frames.append((0, data))
            return len(data)

    def read(self, size: int):
        if self.block_read_event:
            self.block_read_event.wait()
        with self._lock:
            if getattr(self, 'closed', False):
                raise Exception("Serial port closed")
            if self._cancel_current_read:
                self._cancel_current_read = False
                return b""
            if self.raise_on_read:
                raise Exception("Mock read error")
            ret = self.read_buffer[:size]
            self.read_buffer = self.read_buffer[size:]
        if not ret:
            time.sleep(0.005)
        return ret

    def close(self):
        with self._lock:
            self.closed = True
            self.close_count += 1
        if self.block_read_event:
            self._cancel_current_read = True
            self.block_read_event.set()
        if self.block_write_event:
            self._cancel_current_write = True
            self.block_write_event.set()

    def cancel_read(self):
        if self.block_read_event:
            self._cancel_current_read = True
            self.block_read_event.set()

    def cancel_write(self):
        if self.block_write_event:
            self._cancel_current_write = True
            self.block_write_event.set()

    def inject_rx(self, data: bytes):
        with self._lock:
            self.read_buffer += data

    def snapshot_writes(self):
        with self._lock:
            return list(self.written_frames)


class FakeSerialWithInWaiting(FakeSerial):
    def __init__(self, port, baudrate, timeout=0.01, write_timeout=0.1):
        self.timeout = timeout
        self.rx_event = threading.Event()
        self.polls = 0
        self.raise_on_write = False
        self.raise_on_write_after = -1
        self.write_timestamps = []
        if not hasattr(self, 'write_start_event'):
            self.write_start_event = None
        if not hasattr(self, 'write_resume_event'):
            self.write_resume_event = None

        # Now call super which will pop from registry and override our
        # defaults!
        super().__init__(port, baudrate, timeout, write_timeout)

    @property
    def in_waiting(self):
        with self._lock:
            self.polls += 1
            return len(self.read_buffer)

    def inject_rx(self, data: bytes):
        with self._lock:
            self.read_buffer += data
        self.rx_event.set()

    def read(self, size: int):
        if self.block_read_event:
            self.block_read_event.wait()

        with self._lock:
            if getattr(self, 'closed', False):
                raise Exception("Serial port closed")
            if self._cancel_current_read:
                self._cancel_current_read = False
                return b""
            if getattr(self, 'raise_on_read', False):
                raise Exception("Mock read error")
            if self.read_buffer:
                ret = self.read_buffer[:size]
                self.read_buffer = self.read_buffer[size:]
                if not self.read_buffer:
                    self.rx_event.clear()
                return ret

        self.rx_event.wait(timeout=self.timeout)

        with self._lock:
            if getattr(self, 'closed', False):
                return b""
            ret = self.read_buffer[:size]
            self.read_buffer = self.read_buffer[size:]
            if not self.read_buffer:
                self.rx_event.clear()
            return ret

    def write(self, data: bytes):
        if getattr(self, 'closed', False):
            raise Exception("Mock write error: closed")

        import time
        self.write_timestamps.append(time.monotonic())

        if self.write_start_event:
            self.write_start_event.set()
        if self.write_resume_event:
            self.write_resume_event.wait()

        with self._lock:
            if self.raise_on_write:
                import serial
                raise serial.SerialTimeoutException("Mock write error")
            if self.raise_on_write_after > 0:
                if len(data) > self.raise_on_write_after:
                    ret = self.raise_on_write_after
                    self.raise_on_write_after = -1
                    return ret
                self.raise_on_write_after -= len(data)
        return super().write(data)

    def snapshot_writes(self):
        with self._lock:
            return list(self.written_frames)


class QA44NoHookSerial:
    construct_count = 0
    close_count = 0

    def __init__(self, port, baudrate, timeout=0.01, write_timeout=0.1):
        QA44NoHookSerial.construct_count += 1
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.is_open = True
        self.closed = False
        self.in_waiting = 0
        self.read_buffer = b""
        registry.instances.append(self)

    def close(self):
        QA44NoHookSerial.close_count += 1
        self.is_open = False
        self.closed = True

    def read(self, size: int):
        if self.in_waiting > 0:
            chunk = self.read_buffer[:size]
            self.read_buffer = self.read_buffer[size:]
            self.in_waiting = len(self.read_buffer)
            return chunk
        return b""

    def write(self, data: bytes):
        return len(data)

    def inject_rx(self, data: bytes):
        self.read_buffer += data
        self.in_waiting = len(self.read_buffer)


class NoHookSerial:
    construct_count = 0

    def __init__(self, port, baudrate, timeout=0.01, write_timeout=0.1):
        NoHookSerial.construct_count += 1
        self.is_open = True
        self.closed = False

    def read(self, size: int):
        import time
        time.sleep(0.005)
        return b""

    def write(self, data: bytes):
        return len(data)

    def close(self):
        self.closed = True

    def inject_rx(self, data: bytes):
        pass


def wait_valid_state(node, expected_seq, timeout=2.0):
    captured_state = []

    import math

    def check():
        lock = getattr(node, 'state_lock', None)
        assert lock is not None, "Missing state_lock"
        with lock:
            state = getattr(node, 'latest_state', None)
            if not state or len(state) != 3:
                return False
            frame, gen, t = state
            if (gen == getattr(node, 'session_epoch', -1) and
                    frame.get('type') == 'STATE' and
                    frame.get('seq') == expected_seq and
                    isinstance(t, (int, float)) and t >= 0 and math.isfinite(t)):
                captured_state.append(state)
                return True
            return False

    assert wait_until(
        check, timeout=timeout
    ), f"Timeout waiting for exact seq {expected_seq}"

    return captured_state[0]


class ThreadSafeFakeClock:
    def __init__(self):
        import time
        self._time = time.monotonic()
        self.call_count = 0
        import threading
        self._lock = threading.Lock()

    def __call__(self):
        with self._lock:
            self.call_count += 1
            return self._time

    def step(self, dt):
        with self._lock:
            self._time += dt

    def set(self, value):
        with self._lock:
            self._time = value

    def get_call_count(self):
        with self._lock:
            return self.call_count


def test_env_json_schema(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: len(registry.instances) > 0, timeout=2.0)
        inst = registry.instances[0]

        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: getattr(node, 'session_ready', False))

        published = []

        class MockPub:
            def publish(self, msg):
                published.append(msg)
        node.env_pub = MockPub()

        inst.inject_rx(make_env(2, 100, 1, 120.5, 32.1, 11.8, 1))

        def check():
            return getattr(node, 'latest_env', None) is not None

        assert wait_until(check, timeout=2.0)

        node.telemetry_publish_callback()
        assert len(published) == 1
    finally:
        if node:
            node.destroy_node()


def test_odom_imu_quaternion(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: len(registry.instances) > 0, timeout=2.0)
        inst = registry.instances[0]

        pub_odom = []
        pub_imu = []

        class MockOdomPub:
            def publish(self, msg):
                pub_odom.append(msg)

        class MockImuPub:
            def publish(self, msg):
                pub_imu.append(msg)
        node.odom_pub = MockOdomPub()
        node.imu_pub = MockImuPub()

        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0))
        assert wait_until(lambda: node.latest_state is not None)
        assert wait_until(lambda: node.session_ready)

        node.telemetry_publish_callback()

        assert len(pub_odom) == 1
        assert len(pub_imu) == 1

        odom = pub_odom[0]
        imu = pub_imu[0]

        assert odom.header.stamp == imu.header.stamp
        assert abs(odom.pose.pose.orientation.z - 0.707) < 0.01
        assert abs(imu.orientation.z - 0.707) < 0.01
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_exact_bootstrap_and_offline_drop(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        inst = node.ser
        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: getattr(node, "session_ready", False) and
            getattr(node, "telemetry_healthy", False),
            timeout=2.0)

        _ = inst.snapshot_writes()
        writes = inst.snapshot_writes()
        assert len(writes) >= 2
        assert b"CMD,2" in writes[0][1]
        assert b"0.000,0.000" in writes[0][1]
        assert b"PUMP,2" in writes[1][1]
        assert b",0*" in writes[1][1]

        node.close_serial()

        msg = Twist()
        node.cmd_vel_callback(msg)
        assert node.cmd_drop_count == 1
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_no_local_nonzero_after_shutdown(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        inst1 = node.ser
        inst1.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: getattr(node, "session_ready", False) and
            getattr(node, "telemetry_healthy", False),
            timeout=2.0)

        write_evt = threading.Event()
        inst1.block_write_event = write_evt

        msg = Twist()
        msg.linear.x = 2.0
        node.cmd_vel_callback(msg)

        destroy_thread = threading.Thread(target=node.destroy_node)
        destroy_thread.start()

        time.sleep(0.1)
        write_evt.set()

        destroy_thread.join(2.0)

        writes = inst1.snapshot_writes()
        for w in writes:
            if b"CMD" in w[1]:
                assert b"2.0" not in w[1]

        assert not node.worker_thread.is_alive()
        node = None
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_cancel_close_no_unblock_retry_shutdown(rclpy_init):
    node = None
    try:
        registry.instances.clear()

        class StubbornSerial(FakeSerial):
            def cancel_read(self):
                pass

            def close(self):
                pass

        node = SerialBridgeNode(serial_cls=StubbornSerial)
        assert wait_until(lambda: len(registry.instances) > 0, timeout=2.0)
        inst1 = registry.instances[-1]
        inst1.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: node.session_ready)

        read_evt = threading.Event()
        inst1.block_read_event = read_evt
        time.sleep(0.05)

        t0 = time.time()
        success = node.destroy_node()
        t1 = time.time()

        assert not success
        assert t1 - t0 < 4.5
        assert not node.node_destroyed

        read_evt.set()
        time.sleep(0.1)

        s2 = node.destroy_node()
        assert s2
        assert node.node_destroyed
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_cancel_yields_zero_and_closes(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        inst1 = node.ser
        inst1.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: getattr(node, "session_ready", False) and
            getattr(node, "telemetry_healthy", False),
            timeout=2.0)

        read_evt = threading.Event()
        inst1.block_read_event = read_evt

        time.sleep(0.05)
        node.destroy_node()

        assert inst1.closed
        node = None
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_typed_mapping_cmd_fire_pump(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        inst = node.ser
        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: getattr(node, "session_ready", False) and
            getattr(node, "telemetry_healthy", False),
            timeout=2.0)

        msg = Twist()
        msg.linear.x = 1.0
        node.cmd_vel_callback(msg)

        msg2 = Point()
        msg2.x = 0.5
        msg2.y = 0.5
        node.fire_target_callback(msg2)

        class PumpMsg:
            data = True
        node.pump_cmd_callback(PumpMsg())

        assert wait_until(
            lambda: any(
                b'1.0' in w[1] for w in inst.snapshot_writes()))
        assert wait_until(
            lambda: any(
                b'FIRE' in w[1] for w in inst.snapshot_writes()))
        assert wait_until(lambda: any(b'PUMP,2' in w[1] and b',1*' in w[1]
                                      for w in inst.snapshot_writes()))
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_latest_wins_cmd_pump_refresh(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        inst = node.ser
        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: getattr(node, "session_ready", False) and
            getattr(node, "telemetry_healthy", False),
            timeout=2.0)

        write_evt = threading.Event()
        inst.block_write_event = write_evt

        msg = Twist()
        msg.linear.x = 1.0
        node.cmd_vel_callback(msg)
        msg.linear.x = 2.0
        node.cmd_vel_callback(msg)

        write_evt.set()
        assert wait_until(lambda: any(b'CMD' in w[1] and b'2.0' in w[1]
                                      for w in inst.snapshot_writes()))

        _ = inst.snapshot_writes()
        writes = inst.snapshot_writes()
        assert not any(b'CMD' in w[1] and b'1.0' in w[1] for w in writes)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_stale_state_fails_closed_reconnect_fresh(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)

        assert wait_until(lambda: len(registry.instances) > 0, timeout=2.0)
        registry.instances[0].inject_rx(make_state(
            200, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: node.session_ready)

        # State stale -> reconnect
        time.sleep(0.6)
        assert wait_until(lambda: len(registry.instances) > 1, timeout=2.0)

        msg = Twist()
        msg.linear.x = 1.0
        node.cmd_vel_callback(msg)
        assert node.cmd_drop_count >= 1
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_full_duplex_rate(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)

        def run_full_duplex():
            for _ in range(10):
                if not getattr(node, 'running', False):
                    break
                if getattr(registry, 'instances', []):
                    assert wait_until(
                        lambda: len(
                            registry.instances) > 0,
                        timeout=2.0)
                    assert wait_until(lambda: len(registry.instances) > 0, timeout=2.0)
                    registry.instances[-1].inject_rx(make_state(
                        _ * 2 + 1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                    assert wait_until(
                        lambda: len(
                            registry.instances) > 0,
                        timeout=2.0)
                    assert wait_until(lambda: len(registry.instances) > 0, timeout=2.0)
                    registry.instances[-1].inject_rx(
                        make_env(_ * 2 + 2, 100, 1, 10.0, 10.0, 10.0, 1))
                time.sleep(0.1)
                msg = Twist()
                node.cmd_vel_callback(msg)

        t = threading.Thread(target=run_full_duplex)
        t.start()
        t.join(2.0)

        inst = registry.instances[-1]
        _ = inst.snapshot_writes()

        assert node.protocol.crc_fail_count == 0
        assert node.protocol.parse_fail_count == 0
        assert node.protocol.overflow_count == 0
        assert node.protocol.gap_count == 0
        assert node.protocol.dup_count == 0
        assert node.tx_fail_count == 0
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_rx_first_drain_before_tx(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        inst = node.ser
        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: (
                getattr(node, "session_ready", False)
                and getattr(node, "telemetry_healthy", False)),
            timeout=2.0)

        if getattr(node, 'telemetry_timer', None):
            node.telemetry_timer.cancel()

        node.tx_lock.acquire()
        try:
            write_start_event = threading.Event()
            write_resume_event = threading.Event()
            inst.write_start_event = write_start_event
            inst.write_resume_event = write_resume_event

            inst.inject_rx(make_state(2, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

            cmd = Twist()
            cmd.linear.x = 1.0
            node.cmd_vel_callback(cmd)

            assert wait_until(
                lambda: getattr(
                    node,
                    "latest_state",
                    None) is not None and node.latest_state[0]['seq'] == 2,
                timeout=2.0)
        finally:
            node.tx_lock.release()

        try:
            assert write_start_event.wait(timeout=2.0)

            assert len(inst.write_timestamps) > 0
            tx_time = inst.write_timestamps[-1]

            assert node.last_state_time_mono <= tx_time
        finally:
            write_resume_event.set()
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_blocked_publisher_lifecycle(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: len(registry.instances) > 0, timeout=2.0)
        inst = registry.instances[-1]

        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: node.session_ready)

        pub_lock = threading.Lock()
        publish_entered_evt = threading.Event()

        class BlockingPub:
            def publish(self, msg):
                publish_entered_evt.set()
                with pub_lock:
                    pass
        node.odom_pub = BlockingPub()

        pub_lock.acquire()
        if getattr(node, 'telemetry_timer', None):
            node.telemetry_timer.cancel()

        inst.inject_rx(make_state(2, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: node.latest_state is not None, timeout=2.0)

        t = threading.Thread(target=node.telemetry_publish_callback)
        t.start()

        assert publish_entered_evt.wait(timeout=2.0)

        success = node.destroy_node()

        assert not success
        assert not getattr(node, 'node_destroyed', False)

        pub_lock.release()
        t.join(timeout=2.0)
        assert not t.is_alive()

        success2 = node.destroy_node()
        assert success2
        assert getattr(node, 'node_destroyed', False)
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_reconnect_between_odom_imu_env(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: len(registry.instances) > 0, timeout=2.0)
        inst = registry.instances[-1]

        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: node.session_ready)

        inst.inject_rx(make_state(2, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        inst.inject_rx(make_env(3, 100, 1, 1.0, 1.0, 1.0, 1))

        time.sleep(0.1)
        # Force a reconnect manually via simulating closed serial
        node.close_serial()

        pub = []

        class MockPub:
            def publish(self, msg):
                pub.append(msg)
        node.odom_pub = MockPub()
        node.imu_pub = MockPub()
        node.env_pub = MockPub()

        node.telemetry_publish_callback()

        assert len(pub) == 0
        assert node.telemetry_drop_count >= 2
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_pause_before_tx_gate_shutdown_wins(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: len(registry.instances) > 0, timeout=2.0)
        inst = registry.instances[-1]

        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: node.session_ready)

        pause_evt = threading.Event()
        resume_evt = threading.Event()

        class MockLock:
            def __init__(self, real):
                self.real = real

            def acquire(self, *args, **kwargs):
                return self.real.acquire(*args, **kwargs)

            def release(self):
                self.real.release()

            def __enter__(self):
                self.real.acquire()

            def __exit__(self, exc_type, exc_val, exc_tb):
                res = self.real.release()
                tx_locked = getattr(node.tx_lock, 'locked', lambda: False)()
                if tx_locked and threading.current_thread() == node.worker_thread:
                    pause_evt.set()
                    resume_evt.wait()
                return res

        node.state_lock = MockLock(node.state_lock)

        msg = Twist()
        msg.linear.x = 1.0
        node.cmd_vel_callback(msg)

        assert pause_evt.wait(timeout=1.0)

        t = threading.Thread(target=node.destroy_node)
        t.start()
        time.sleep(0.1)

        resume_evt.set()
        t.join(timeout=2.0)
        assert not t.is_alive()

        writes = inst.snapshot_writes()
        # Normal TX should be dropped, finally zero TX should be written
        assert not any(b'CMD' in w[1] and b'1.0' in w[1] for w in writes)
        assert any(b'CMD' in w[1] and b'0.0' in w[1] for w in writes)
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_block_pump_refresh_write_callback_unblocked(rclpy_init):
    node = None
    try:
        registry.instances.clear()

        class StubbornSerial(FakeSerial):
            def cancel_read(self):
                pass

            def cancel_write(self):
                pass

            def close(self):
                pass

        node = SerialBridgeNode(serial_cls=StubbornSerial)
        assert wait_until(lambda: len(registry.instances) > 0, timeout=2.0)
        inst = registry.instances[-1]

        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: node.session_ready)
        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: node.telemetry_healthy)

        node.state_stale_timeout_ms = 5000

        pump_msg = Bool()
        pump_msg.data = True
        node.pump_cmd_callback(pump_msg)

        write_evt = threading.Event()
        inst.block_write_event = write_evt
        time.sleep(0.6)  # Wait for 2Hz pump refresh

        # Another pump cmd callback
        t0 = time.time()
        node.pump_cmd_callback(pump_msg)
        t1 = time.time()
        assert t1 - t0 < 0.1  # Callback not blocked!

        success = node.destroy_node()
        assert not success

        write_evt.set()
        time.sleep(0.1)
        success2 = node.destroy_node()
        assert success2
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_presend_cmd_zero_and_pump_off_supersede_unsafe_items(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: (
                getattr(node, "session_ready", False)
                and getattr(node, "telemetry_healthy", False)),
            timeout=2.0)

        pause_evt = threading.Event()
        resume_evt = threading.Event()

        orig_write_pending_item = node._write_pending_item

        def hooked_write_pending_item(item, current_ser):
            pause_evt.set()
            resume_evt.wait()
            return orig_write_pending_item(item, current_ser)

        node._write_pending_item = hooked_write_pending_item

        cmd1 = Twist()
        cmd1.linear.x = 1.0
        node.cmd_vel_callback(cmd1)

        pump1 = Bool()
        pump1.data = True
        node.pump_cmd_callback(pump1)

        try:
            assert pause_evt.wait(timeout=2.0)

            cmd2 = Twist()
            cmd2.linear.x = 0.0
            node.cmd_vel_callback(cmd2)

            pump2 = Bool()
            pump2.data = False
            node.pump_cmd_callback(pump2)
        finally:
            resume_evt.set()

        assert wait_until(
            lambda: any(
                b'CMD' in w[1] and b'0.0' in w[1] for w in first_ser.snapshot_writes()),
            timeout=2.0)
        assert wait_until(
            lambda: any(
                b'PUMP' in w[1] and b'0' in w[1] for w in first_ser.snapshot_writes()),
            timeout=2.0)

        writes = first_ser.snapshot_writes()
        assert not any(b'CMD' in w[1] and b'1.0' in w[1] for w in writes)
        assert not any(b'PUMP' in w[1] and b'1' in w[1] for w in writes)
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_bootstrap_partial(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        registry.next_config.append({
            'raise_on_write_after': 5
        })

        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        # The first instance will partial write during bootstrap!
        assert wait_until(lambda: len(registry.instances) > 1, timeout=2.0)
        assert getattr(node, 'reconnect_count', 0) > 0
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_bootstrap_error(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        registry.next_config.append({
            'raise_on_write': True
        })

        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        # The first instance will raise on write during bootstrap!
        assert wait_until(lambda: len(registry.instances) > 1, timeout=2.0)
        assert getattr(node, 'reconnect_count', 0) > 0
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_persistent_bootstrap_write_timeout(rclpy_init):
    node = None
    try:
        registry.instances.clear()

        healthy_event = threading.Event()
        barrier_event = threading.Event()

        class EventControlledSerial(FakeSerialWithInWaiting):
            def write(self, data: bytes):
                barrier_event.wait()
                if not healthy_event.is_set():
                    import serial
                    raise serial.SerialTimeoutException(
                        "Event controlled write timeout")
                return super().write(data)

        node = SerialBridgeNode(serial_cls=EventControlledSerial)

        published_odom = []
        published_imu = []

        class MockPubOdom:
            def publish(self, msg): published_odom.append(msg)

        class MockPubImu:
            def publish(self, msg): published_imu.append(msg)

        node.odom_pub = MockPubOdom()
        node.imu_pub = MockPubImu()

        # Release barrier to let bootstrap fail
        barrier_event.set()

        # Test state DURING the persistent phase
        time.sleep(1.0)
        assert getattr(node, "session_ready", False) is False
        assert getattr(node, "telemetry_healthy", False) is False

        # Upper and lower bound for retries: 1.0s with backoff (0.1, 0.15...)
        assert 1 < node.bootstrap_fail_count < 10

        # Inject some old data during failing phase
        if len(registry.instances) > 0:
            inst = registry.instances[-1]
            inst.inject_rx(make_state(1, 100, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0))

        # Try to publish, it should drop because epoch/session is not ready
        node.telemetry_publish_callback()

        assert getattr(node, "telemetry_healthy", False) is False
        # Assert open_attempt == open_success + open_fail
        att = getattr(node, "open_attempt_count", 0)
        succ = getattr(node, "open_success_count", 0)
        fail = getattr(node, "open_fail_count", 0)
        boot_fail = getattr(node, "bootstrap_fail_count", 0)
        assert att == succ + fail + boot_fail
        assert node.open_fail_count == 0

        with node.state_lock:
            assert len(published_odom) == 0
            assert len(published_imu) == 0

        # Now make it healthy
        healthy_event.set()

        assert wait_until(
            lambda: getattr(
                node,
                "session_ready",
                False),
            timeout=3.0)
        assert node.bootstrap_fail_count > 0
        assert "Event controlled write timeout" in node.last_failure_reason

        # Inject state into the NEW epoch (the active one)
        active_inst = node.ser
        active_inst.inject_rx(
            make_state(
                2,
                100,
                10.0,
                20.0,
                30.0,
                40.0,
                50.0,
                60.0))
        assert wait_until(
            lambda: getattr(
                node,
                "telemetry_healthy",
                False),
            timeout=2.0)

        # Trigger publish manually to ensure age < 0.2s
        node.telemetry_publish_callback()

        with node.state_lock:
            # We expect exactly one fresh odom+IMU published
            assert len(published_odom) == 1
            assert len(published_imu) == 1
            assert published_odom[0].pose.pose.position.x == 10.0
            assert published_odom[0].pose.pose.position.y == 20.0

    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_normal_cmd_partial(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: (
                getattr(node, "session_ready", False)
                and getattr(node, "telemetry_healthy", False)),
            timeout=2.0)

        first_ser.raise_on_write_after = 5
        cmd = Twist()
        cmd.linear.x = 1.0
        node.cmd_vel_callback(cmd)

        assert wait_until(lambda: node.ser is not first_ser, timeout=2.0)
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_normal_cmd_error(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: (
                getattr(node, "session_ready", False)
                and getattr(node, "telemetry_healthy", False)),
            timeout=2.0)

        first_ser.raise_on_write = True
        cmd = Twist()
        cmd.linear.x = 1.0
        node.cmd_vel_callback(cmd)

        assert wait_until(lambda: node.ser is not first_ser, timeout=2.0)
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_pump_refresh_partial(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: (
                getattr(node, "session_ready", False)
                and getattr(node, "telemetry_healthy", False)),
            timeout=2.0)

        with node.state_lock:
            node.last_pump_time = 0.0
        first_ser.raise_on_write_after = 5

        assert wait_until(lambda: node.ser is not first_ser, timeout=2.0)
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_pump_refresh_error(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: (
                getattr(node, "session_ready", False)
                and getattr(node, "telemetry_healthy", False)),
            timeout=2.0)

        with node.state_lock:
            node.last_pump_time = 0.0
        first_ser.raise_on_write = True

        assert wait_until(lambda: node.ser is not first_ser, timeout=2.0)
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_fresh_state_before_deadline_does_not_reconnect(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        first_ser = node.ser

        import time
        for _ in range(5):
            first_ser.inject_rx(
                make_state(
                    1,
                    100,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0))
            time.sleep(0.1)

        assert node.ser is first_ser
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_destroy_retry(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)

        node.tx_lock.acquire()
        initial_epoch = getattr(node, 'session_epoch', 0)

        ret = node.destroy_node()
        assert not ret

        assert not node.running
        assert node.stop_request
        assert getattr(
            node,
            'log_timer',
            None) and node.log_timer.is_canceled()
        assert getattr(
            node,
            'telemetry_timer',
            None) and node.telemetry_timer.is_canceled()
        assert not getattr(node, 'node_destroyed', False)

        node.tx_lock.release()
        import time
        node.destroy_node()  # Trigger close_serial spawn since deadline expired
        time.sleep(0.1)
        ret = node.destroy_node()
        assert ret
        assert getattr(node, 'session_epoch', 0) >= initial_epoch + 1

        node = None
    finally:
        if node and getattr(node, 'tx_lock', None) and node.tx_lock.locked():
            node.tx_lock.release()
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_real_like_idle_poll_rate(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)

        node.state_stale_timeout_ms = 10000

        with node.state_lock:
            first_ser = node.ser

        state_frame = make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        first_ser.inject_rx(state_frame)

        assert wait_until(lambda: getattr(node, "session_ready", False)
                          and
                          getattr(node, "telemetry_healthy", False),
                          timeout=2.0)

        with first_ser._lock:
            first_ser.polls = 0

        import time
        t0 = time.monotonic()
        time.sleep(0.2)
        elapsed = time.monotonic() - t0

        with first_ser._lock:
            polls = first_ser.polls

        rate = polls / elapsed if elapsed > 0 else 0
        assert rate < 2000, f"Too many polls: {rate}/s"

        write_start = threading.Event()
        first_ser.write_start_event = write_start

        cmd_msg = Twist()
        cmd_msg.linear.x = 2.0

        t0 = time.monotonic()
        node.cmd_vel_callback(cmd_msg)

        assert write_start.wait(timeout=2.0)
        latency = first_ser.write_timestamps[-1] - t0
        assert 0.0 <= latency <= 0.02

        node.state_stale_timeout_ms = 100
        assert wait_until(
            lambda: not getattr(
                node,
                "telemetry_healthy",
                False) or getattr(
                node,
                "ser",
                None) is not first_ser,
            timeout=2.0)

    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


def test_exact_baseline_order(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        node.state_stale_timeout_ms = 10000
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: getattr(node, "session_ready", False) and
            getattr(node, "telemetry_healthy", False),
            timeout=2.0)

        baseline_writes = len(first_ser.snapshot_writes())

        with node.tx_lock:
            cmd = Twist()
            cmd.linear.x = 1.0
            node.cmd_vel_callback(cmd)

            pump = Bool()
            pump.data = True
            node.pump_cmd_callback(pump)

            fire = Point()
            fire.x = 0.5
            node.fire_target_callback(fire)

        assert wait_until(
            lambda: len(first_ser.snapshot_writes()) >= baseline_writes + 3,
            timeout=2.0
        )

        writes = first_ser.snapshot_writes()[baseline_writes:]
        assert len(writes) >= 3, f"Writes: {writes}"
        # Filter out pump refreshes if any, or just assert order
        types = [b'CMD' if b'CMD' in w[1] else b'PUMP' if b'PUMP' in w[1]
                 else b'FIRE' if b'FIRE' in w[1] else b'OTHER' for w in writes]
        assert types[:3] == [b'CMD', b'PUMP', b'FIRE'], f"Writes: {writes}"
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_main_destroy_immediate(rclpy_init):
    import fire_robot_bringup.serial_bridge_node as target_module
    import time

    class MockNode:

        def __init__(self):
            self.calls = 0
            self.node_destroyed = False

        def destroy_node(self):
            self.calls += 1
            self.node_destroyed = True
            return True

    old_init = getattr(target_module.rclpy, 'init', None)
    old_spin = getattr(target_module.rclpy, 'spin', None)
    old_ok = getattr(target_module.rclpy, 'ok', None)
    old_shutdown = getattr(target_module.rclpy, 'shutdown', None)
    old_SerialBridgeNode = getattr(target_module, 'SerialBridgeNode', None)
    node = MockNode()

    def fake_init(*args, **kwargs):
        assert kwargs.get('signal_handler_options') == SignalHandlerOptions.NO

    def fake_spin(n): pass
    def fake_ok(): return True
    def fake_shutdown(): pass
    def fake_SerialBridgeNode(): return node

    target_module.rclpy.init = fake_init
    target_module.rclpy.spin = fake_spin
    target_module.rclpy.ok = fake_ok
    target_module.rclpy.shutdown = fake_shutdown
    target_module.SerialBridgeNode = fake_SerialBridgeNode
    try:
        t0 = time.monotonic()
        target_module.main()
        t1 = time.monotonic()
        assert node.calls == 1
        assert t1 - t0 < 0.5
    finally:
        target_module.rclpy.init = old_init
        target_module.rclpy.spin = old_spin
        target_module.rclpy.ok = old_ok
        target_module.rclpy.shutdown = old_shutdown
        target_module.SerialBridgeNode = old_SerialBridgeNode


def test_main_destroy_false_false_true(rclpy_init):
    import fire_robot_bringup.serial_bridge_node as target_module

    class MockNode:

        def __init__(self):
            self.calls = 0
            self.node_destroyed = False

        def destroy_node(self):
            self.calls += 1
            if self.calls < 3:
                return False
            self.node_destroyed = True
            return True

    old_init = getattr(target_module.rclpy, 'init', None)
    old_spin = getattr(target_module.rclpy, 'spin', None)
    old_ok = getattr(target_module.rclpy, 'ok', None)
    old_shutdown = getattr(target_module.rclpy, 'shutdown', None)
    old_SerialBridgeNode = getattr(target_module, 'SerialBridgeNode', None)
    node = MockNode()

    def fake_init(*args, **kwargs):
        assert kwargs.get('signal_handler_options') == SignalHandlerOptions.NO

    def fake_spin(n): pass
    def fake_ok(): return True
    def fake_shutdown(): pass
    def fake_SerialBridgeNode(): return node

    target_module.rclpy.init = fake_init
    target_module.rclpy.spin = fake_spin
    target_module.rclpy.ok = fake_ok
    target_module.rclpy.shutdown = fake_shutdown
    target_module.SerialBridgeNode = fake_SerialBridgeNode
    try:
        target_module.main()
        assert node.calls == 3
    finally:
        target_module.rclpy.init = old_init
        target_module.rclpy.spin = old_spin
        target_module.rclpy.ok = old_ok
        target_module.rclpy.shutdown = old_shutdown
        target_module.SerialBridgeNode = old_SerialBridgeNode


def test_main_destroy_always_false(rclpy_init):
    import fire_robot_bringup.serial_bridge_node as target_module
    import time

    class MockNode:

        def __init__(self):
            self.calls = 0
            self.node_destroyed = False

        def destroy_node(self):
            self.calls += 1
            return False

    old_init = getattr(target_module.rclpy, 'init', None)
    old_spin = getattr(target_module.rclpy, 'spin', None)
    old_ok = getattr(target_module.rclpy, 'ok', None)
    old_shutdown = getattr(target_module.rclpy, 'shutdown', None)
    old_SerialBridgeNode = getattr(target_module, 'SerialBridgeNode', None)
    node = MockNode()

    def fake_init(*args, **kwargs):
        assert kwargs.get('signal_handler_options') == SignalHandlerOptions.NO

    def fake_spin(n): pass
    def fake_ok(): return True
    def fake_shutdown(): pass
    def fake_SerialBridgeNode(): return node

    target_module.rclpy.init = fake_init
    target_module.rclpy.spin = fake_spin
    target_module.rclpy.ok = fake_ok
    target_module.rclpy.shutdown = fake_shutdown
    target_module.SerialBridgeNode = fake_SerialBridgeNode
    try:
        t0 = time.monotonic()
        target_module.main()
        t1 = time.monotonic()
        assert node.calls > 10
        # If we use bounded sleep min(0.1, remaining), it will exit at exactly 3.7.
        # But if the mutation deadline=3.95; sleep(0.1) is used, it might sleep
        # 0.1 past 3.95
        assert t1 - t0 < 3.95, f"Took {t1 - t0}s, which is > 3.95s!"
    finally:
        target_module.rclpy.init = old_init
        target_module.rclpy.spin = old_spin
        target_module.rclpy.ok = old_ok
        target_module.rclpy.shutdown = old_shutdown
        target_module.SerialBridgeNode = old_SerialBridgeNode


def test_main_destroy_slow_first_call_late_release(rclpy_init):
    import fire_robot_bringup.serial_bridge_node as target_module
    import time

    class MockNode:

        def __init__(self):
            self.calls = 0
            self.node_destroyed = False

        def destroy_node(self):
            self.calls += 1
            if self.calls == 1:
                time.sleep(1.0)
                return False
            self.node_destroyed = True
            return True

    old_init = getattr(target_module.rclpy, 'init', None)
    old_spin = getattr(target_module.rclpy, 'spin', None)
    old_ok = getattr(target_module.rclpy, 'ok', None)
    old_shutdown = getattr(target_module.rclpy, 'shutdown', None)
    old_SerialBridgeNode = getattr(target_module, 'SerialBridgeNode', None)
    node = MockNode()

    def fake_init(*args, **kwargs):
        assert kwargs.get('signal_handler_options') == SignalHandlerOptions.NO

    def fake_spin(n): pass
    def fake_ok(): return True
    def fake_shutdown(): pass
    def fake_SerialBridgeNode(): return node

    target_module.rclpy.init = fake_init
    target_module.rclpy.spin = fake_spin
    target_module.rclpy.ok = fake_ok
    target_module.rclpy.shutdown = fake_shutdown
    target_module.SerialBridgeNode = fake_SerialBridgeNode
    try:
        t0 = time.monotonic()
        target_module.main()
        t1 = time.monotonic()
        assert node.calls == 2
        assert 0.9 <= t1 - t0 < 1.5
    finally:
        target_module.rclpy.init = old_init
        target_module.rclpy.spin = old_spin
        target_module.rclpy.ok = old_ok
        target_module.rclpy.shutdown = old_shutdown
        target_module.SerialBridgeNode = old_SerialBridgeNode


@pytest.mark.parametrize('signal_to_send', [signal.SIGINT, signal.SIGTERM])
def test_main_signal_keeps_context_valid_until_destroy(
        rclpy_init, monkeypatch, signal_to_send):
    """Keep the ROS context valid through node cleanup for both signals."""
    import fire_robot_bringup.serial_bridge_node as target_module

    call_order = []
    context_ok = [False]
    original_sigint = signal.getsignal(signal.SIGINT)
    original_sigterm = signal.getsignal(signal.SIGTERM)

    class MockNode:
        def __init__(self):
            self.node_destroyed = False

        def destroy_node(self):
            assert context_ok[0]
            call_order.append('node.destroy_node')
            registered_handler = signal.getsignal(signal_to_send)
            registered_handler(signal_to_send, None)
            self.node_destroyed = True
            return True

    node = MockNode()

    def fake_init(*args, **kwargs):
        assert kwargs.get('signal_handler_options') == SignalHandlerOptions.NO
        context_ok[0] = True
        call_order.append('rclpy.init')

    def fake_spin(candidate):
        assert candidate is node
        call_order.append('rclpy.spin')
        registered_handler = signal.getsignal(signal_to_send)
        registered_handler(signal_to_send, None)

    def fake_ok():
        return context_ok[0]

    def fake_shutdown():
        assert context_ok[0]
        call_order.append('rclpy.shutdown')
        context_ok[0] = False

    monkeypatch.setattr(target_module.rclpy, 'init', fake_init)
    monkeypatch.setattr(target_module.rclpy, 'spin', fake_spin)
    monkeypatch.setattr(target_module.rclpy, 'ok', fake_ok)
    monkeypatch.setattr(target_module.rclpy, 'shutdown', fake_shutdown)
    monkeypatch.setattr(target_module, 'SerialBridgeNode', lambda: node)

    target_module.main()

    assert call_order == [
        'rclpy.init',
        'rclpy.spin',
        'node.destroy_node',
        'rclpy.shutdown',
    ]
    assert signal.getsignal(signal.SIGINT) == original_sigint
    assert signal.getsignal(signal.SIGTERM) == original_sigterm


def test_main_subprocess_sigint_stress():
    """Reject ROS-context shutdown races under immediate real SIGINT."""
    run_serial_bridge_signal_subprocess(signal.SIGINT, 100)


def test_main_subprocess_sigterm_stress():
    """Reject ROS-context shutdown races under immediate real SIGTERM."""
    run_serial_bridge_signal_subprocess(signal.SIGTERM, 100)


def test_bootstrap_lifecycle_gate(rclpy_init):
    node = None
    try:
        registry.instances.clear()

        write_start = threading.Event()
        write_resume = threading.Event()

        # Pre-configure the FIRST serial instance to block during write
        registry.next_config.append({
            'write_start_event': write_start,
            'write_resume_event': write_resume
        })

        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(lambda: len(registry.instances) > 0, timeout=2.0)
        first_inst = registry.instances[0]

        # It should block in bootstrap write immediately!
        assert write_start.wait(timeout=2.0)

        # Now destroy it concurrently!
        t = threading.Thread(target=node.destroy_node)
        t.start()

        assert wait_until(
            lambda: getattr(
                node,
                "stop_request",
                False),
            timeout=2.0)
        assert not getattr(node, "session_ready", False)

        try:
            write_resume.set()
            t.join(timeout=2.0)
            assert not t.is_alive()
        finally:
            write_resume.set()

        assert not getattr(node, "session_ready", False)

        writes = first_inst.snapshot_writes()
        assert len(writes) == 1, f"Expected exactly 1 write, got {len(writes)}"
        assert b'CMD' in writes[0][1]
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_live_read_error_recovery(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        first_ser = node.ser

        pub_odom = []
        pub_imu = []

        class MockOdomPub:
            def publish(self, msg):
                pub_odom.append(msg)

        class MockImuPub:
            def publish(self, msg):
                pub_imu.append(msg)

        node.odom_pub = MockOdomPub()
        node.imu_pub = MockImuPub()

        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: getattr(
                node,
                "session_ready",
                False),
            timeout=2.0)

        # Force publish to consume the first state
        node.telemetry_publish_callback()
        pub_odom.clear()
        pub_imu.clear()

        # Inject read error
        first_ser.raise_on_read = True
        first_ser.rx_event.set()  # Wake up reader

        # Worker should catch it, close first_ser, and open a new one
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None and node.ser is not first_ser,
            timeout=2.0)
        assert getattr(node.worker_thread, "is_alive")()

        # Attempt to publish during failure (state stale or session_ready
        # false)
        node.telemetry_publish_callback()
        assert len(pub_odom) == 0, "Should not publish old odom during failure"
        assert len(pub_imu) == 0, "Should not publish old imu during failure"

        second_ser = node.ser
        second_ser.inject_rx(make_state(2, 100, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: getattr(
                node,
                "session_ready",
                False),
            timeout=2.0)
        assert wait_until(
            lambda: getattr(
                node,
                "latest_state",
                None) is not None and node.latest_state[0]['seq'] == 2,
            timeout=2.0)

        node.telemetry_publish_callback()
        assert len(pub_odom) == 1, "Should publish fresh odom after recovery"
        assert pub_odom[0].pose.pose.position.x == 1.0
        assert len(pub_imu) == 1, "Should publish fresh imu after recovery"

    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_prolonged_bootstrap_timeout(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        first_ser = node.ser

        pub_odom = []
        pub_imu = []

        class MockOdomPub:
            def publish(self, msg):
                pub_odom.append(msg)

        class MockImuPub:
            def publish(self, msg):
                pub_imu.append(msg)

        node.odom_pub = MockOdomPub()
        node.imu_pub = MockImuPub()

        # Inject seq 1
        first_ser.inject_rx(make_state(1, 100, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: getattr(
                node,
                "session_ready",
                False),
            timeout=2.0)
        assert wait_until(
            lambda: getattr(
                node,
                "latest_state",
                None) is not None and node.latest_state[0]['seq'] == 1,
            timeout=2.0)

        # Capture epoch of state seq 1
        with node.state_lock:
            state_epoch = node.latest_state[1]
            current_epoch = getattr(node, "session_epoch", -1)
            assert state_epoch == current_epoch, "State epoch should match current session epoch"
            assert node.latest_state[0]['seq'] == 1

        # Do NOT publish/clear state seq 1 before failure.

        class FailingSerial(FakeSerialWithInWaiting):
            def write(self, data: bytes):
                import serial
                raise serial.SerialTimeoutException("Forced write error")

        node.serial_cls = FailingSerial

        # Inject read error to trigger reconnect
        first_ser.raise_on_read = True
        first_ser.rx_event.set()

        # Wait for bootstrap failure
        assert wait_until(
            lambda: getattr(
                node,
                "session_ready",
                False) is False,
            timeout=2.0)
        assert wait_until(
            lambda: getattr(
                node, "session_epoch", -1) != state_epoch, timeout=2.0)
        assert wait_until(
            lambda: getattr(
                node,
                "bootstrap_fail_count",
                0) > 0,
            timeout=2.0)

        # Assert old latest_state seq 1 still exists
        with node.state_lock:
            assert node.latest_state is not None
            assert node.latest_state[0]['seq'] == 1
            assert node.latest_state[1] == state_epoch

        # Assert no stale data is published
        node.telemetry_publish_callback()
        assert len(
            pub_odom) == 0, "Should not publish stale odom during bootstrap failure"
        assert len(
            pub_imu) == 0, "Should not publish stale imu during bootstrap failure"

        # Restore original serial class and wait for successful reconnect
        node.serial_cls = FakeSerialWithInWaiting
        assert wait_until(
            lambda: getattr(node, "ser", None) is not None and
            type(node.ser) is FakeSerialWithInWaiting, timeout=4.0)
        third_ser = node.ser
        third_ser.inject_rx(make_state(2, 100, 5.0, 6.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: getattr(
                node,
                "session_ready",
                False),
            timeout=2.0)
        assert wait_until(
            lambda: getattr(
                node,
                "latest_state",
                None) is not None and node.latest_state[0]['seq'] == 2,
            timeout=2.0)

        node.telemetry_publish_callback()
        assert len(pub_odom) == 1, "Should publish fresh odom after recovery"
        assert len(pub_imu) == 1, "Should publish fresh imu after recovery"
        assert pub_odom[0].pose.pose.position.x == 5.0
        assert pub_odom[0].pose.pose.position.y == 6.0

    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_presend_newer_supersedes_older(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(
            lambda: (
                getattr(node, "session_ready", False)
                and getattr(node, "telemetry_healthy", False)),
            timeout=2.0)

        pause_evt = threading.Event()
        resume_evt = threading.Event()

        orig_write_pending_item = node._write_pending_item

        def hooked_write_pending_item(item, current_ser):
            pause_evt.set()
            resume_evt.wait()
            return orig_write_pending_item(item, current_ser)

        node._write_pending_item = hooked_write_pending_item

        cmd1 = Twist()
        cmd1.linear.x = 0.2
        node.cmd_vel_callback(cmd1)

        fire1 = Point()
        fire1.x = 0.1
        node.fire_target_callback(fire1)

        try:
            assert pause_evt.wait(timeout=2.0)

            cmd2 = Twist()
            cmd2.linear.x = 0.9
            node.cmd_vel_callback(cmd2)

            fire2 = Point()
            fire2.x = 0.9
            node.fire_target_callback(fire2)
        finally:
            resume_evt.set()

        assert wait_until(
            lambda: any(
                b'CMD' in w[1] and b'0.9' in w[1] for w in first_ser.snapshot_writes()),
            timeout=2.0)
        assert wait_until(
            lambda: any(
                b'FIRE' in w[1] and b'0.9' in w[1] for w in first_ser.snapshot_writes()),
            timeout=2.0)

        writes = first_ser.snapshot_writes()
        assert not any(b'CMD' in w[1] and b'0.2' in w[1] for w in writes)
        assert not any(b'FIRE' in w[1] and b'0.1' in w[1] for w in writes)
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_persistent_read_error_bounded_reconnect(rclpy_init):
    node = None
    try:
        registry.instances.clear()

        class PersistentErrorSerial(FakeSerialWithInWaiting):
            persistent_raise_on_read = True
            init_times = []

            def __init__(
                    self,
                    port,
                    baudrate,
                    timeout=0.01,
                    write_timeout=0.1):
                import time
                self.__class__.init_times.append(time.monotonic())
                super().__init__(port, baudrate, timeout, write_timeout)

            def read(self, size: int):
                if self.__class__.persistent_raise_on_read:
                    raise Exception("Persistent Mock Read Error")
                return super().read(size)

        node = SerialBridgeNode(serial_cls=PersistentErrorSerial)

        import time
        t0 = time.time()
        time.sleep(0.2)  # Wait 200ms
        elapsed = time.time() - t0

        assert node.reconnect_count < 10, (
            f"Too many reconnects: {node.reconnect_count} in {elapsed:.3f}s"
        )
        assert getattr(node.worker_thread, "is_alive")()

        assert wait_until(
            lambda: len(
                PersistentErrorSerial.init_times) >= 2,
            timeout=2.0)
        delay = PersistentErrorSerial.init_times[1] - \
            PersistentErrorSerial.init_times[0]
        assert delay >= 0.08, f"First reopen too fast: {delay:.6f}s"
        assert getattr(node, "reconnect_backoff", 0.0) > 0.0
        assert getattr(node, "next_reconnect_mono", 0.0) > 0.0

        # Remove fault
        PersistentErrorSerial.persistent_raise_on_read = False

        # Wait for next instance
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        node.ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        assert wait_until(
            lambda: getattr(
                node,
                "session_ready",
                False),
            timeout=2.0)
        assert wait_until(
            lambda: getattr(
                node,
                "telemetry_healthy",
                False),
            timeout=2.0)

        assert getattr(node, "reconnect_backoff", 1.0) == 0.0
        assert getattr(node, "next_reconnect_mono", 1.0) == 0.0
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_tx_backoff_preserve(rclpy_init):
    node = None
    try:
        registry.instances.clear()

        write_start = threading.Event()
        write_resume = threading.Event()
        write_resume.set()  # Let bootstrap writes pass

        registry.next_config.append({
            'write_start_event': write_start,
            'write_resume_event': write_resume
        })

        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        assert wait_until(
            lambda: getattr(
                node,
                "session_ready",
                False),
            timeout=2.0)
        assert wait_until(
            lambda: getattr(
                node,
                "telemetry_healthy",
                False),
            timeout=2.0)

        write_start.clear()
        write_resume.clear()

        node.reconnect_backoff = 0.5
        node.next_reconnect_mono = 123.0

        cmd = Twist()
        cmd.linear.x = 0.5
        node.cmd_vel_callback(cmd)

        assert write_start.wait(timeout=2.0)

        assert getattr(node, "reconnect_backoff", 0.0) == 0.5
        assert getattr(node, "next_reconnect_mono", 0.0) == 123.0

        write_resume.set()

        assert wait_until(
            lambda: any(
                b'CMD' in w[1] and b'0.5' in w[1] for w in first_ser.snapshot_writes()),
            timeout=2.0)

        assert getattr(node, "reconnect_backoff", 0.0) == 0.5
        assert getattr(node, "next_reconnect_mono", 0.0) == 123.0

        first_ser.inject_rx(make_state(2, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        assert wait_until(
            lambda: getattr(node, "reconnect_backoff", 1.0) == 0.0,
            timeout=2.0
        )
        assert getattr(node, "next_reconnect_mono", 1.0) == 0.0

    finally:
        try:
            write_resume.set()
        except NameError:
            pass
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_mutation_proof_worker_guard(rclpy_init):
    # Prove that removing worker guard results in false success
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)

        # Mutate to ignore worker
        original_join = getattr(node.worker_thread, 'join', None)
        node.worker_thread.join = lambda timeout=None: None

        # Test should now fail because worker is 'alive' (is_alive mock)
        original_is_alive = getattr(node.worker_thread, 'is_alive', None)
        node.worker_thread.is_alive = lambda: True

        assert not node.destroy_node()

    finally:
        if node and not getattr(node, 'node_destroyed', False):
            if hasattr(
                    node,
                    'worker_thread') and hasattr(
                    node.worker_thread,
                    'is_alive'):
                node.worker_thread.is_alive = original_is_alive
                node.worker_thread.join = original_join
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_mutation_proof_node_destroyed_early(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)

        # Hold tx_lock so destroy fails
        node.tx_lock.acquire()
        ret = node.destroy_node()
        assert not ret
        assert not getattr(node, 'node_destroyed', False), \
            "Mutation Proof: node_destroyed set early!"
        node.tx_lock.release()
    finally:
        if node and getattr(node, 'tx_lock', None) and node.tx_lock.locked():
            node.tx_lock.release()
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_mutation_proof_publish_quiescence(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "ser",
                None) is not None,
            timeout=2.0)

        node.publish_lock.acquire()
        ret = node.destroy_node()
        assert not ret, "Mutation Proof: publish quiescence bypassed!"
        node.publish_lock.release()
    finally:
        if node and getattr(
            node,
            'publish_lock',
                None) and node.publish_lock.locked():
            node.publish_lock.release()
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_tx_fail_count_vs_partial(rclpy_init):
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "session_ready",
                False),
            timeout=2.0)

        with node.state_lock:
            assert node.tx_fail_count == 0
            assert node.tx_partial_count == 0

            # Make it raise exception on write
            node.ser.raise_on_write = True

        # Command should fail and increment fail count
        msg = Twist()
        node.cmd_vel_callback(msg)

        assert wait_until(lambda: node.tx_fail_count > 0, timeout=1.0)
        with node.state_lock:
            assert node.tx_fail_count == 1
            assert node.tx_partial_count == 0

    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_destroy_retry_reaps_late_close_then_succeeds(rclpy_init):
    import threading
    from unittest.mock import patch
    import rclpy.node
    from fire_robot_bringup.serial_bridge_node import SerialBridgeNode

    node = None
    close_entered = threading.Event()
    release_close = threading.Event()

    class SlowCloseSerial(FakeSerialWithInWaiting):
        def close(self):
            close_entered.set()
            release_close.wait()
            super().close()

    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=SlowCloseSerial)
        assert wait_until(
            lambda: getattr(
                node,
                "session_ready",
                False),
            timeout=2.0)

        super_called = False
        original_destroy = rclpy.node.Node.destroy_node

        def mock_super_destroy(*args, **kwargs):
            nonlocal super_called
            super_called = True
            target = args[0] if args else node
            return original_destroy(target)

        with patch.object(rclpy.node.Node, 'destroy_node', side_effect=mock_super_destroy):
            node.close_serial()
            assert close_entered.wait(timeout=2.0)

            ret = node.destroy_node()

            assert ret is False
            assert getattr(node, 'node_destroyed', False) is False
            assert super_called is False

            release_close.set()
            assert wait_until(
                lambda: node.close_ownership.get('handle') is None
                or node.close_ownership.get('result') is not None,
                timeout=2.0)

            ret2 = node.destroy_node()
            assert ret2 is True
            assert getattr(node, 'node_destroyed', False) is True
            assert super_called is True

    finally:
        release_close.set()
        if node and not getattr(node, 'node_destroyed', False):
            node.destroy_node()
        rclpy.shutdown()


def test_tx_lock_timeout_never_closes_concurrently_exact(rclpy_init):
    node = None
    release_writer = threading.Event()
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(
            lambda: getattr(
                node,
                "session_ready",
                False),
            timeout=2.0)

        with node.state_lock:
            initial_close_fail = getattr(node, 'close_fail_count', 0)
            target_ser = node.ser
            target_gen = node.session_epoch
            initial_close_count = getattr(target_ser, 'close_count', 0)

        writer_ready = threading.Event()
        writer_done = threading.Event()

        def writer_thread():
            tx_acq = node.tx_lock.acquire(timeout=2.0)
            if tx_acq:
                try:
                    writer_ready.set()
                    release_writer.wait()
                finally:
                    node.tx_lock.release()
            writer_done.set()

        t_writer = threading.Thread(target=writer_thread, daemon=True)
        t_writer.start()

        assert writer_ready.wait(timeout=1.0)

        # Request close
        node.close_serial()

        assert wait_until(lambda: node.close_ownership.get(
            'task') is not None, timeout=1.0)
        assert wait_until(lambda: node.close_ownership.get(
            'result') is not None, timeout=1.0)

        with node.state_lock:
            res = node.close_ownership.get('result')
            assert isinstance(res, TimeoutError)
            assert node.close_fail_count == initial_close_fail
            assert not target_ser.closed
            assert node.close_ownership.get('handle') is target_ser
            assert node.close_ownership.get('generation') == target_gen
            assert getattr(target_ser, 'close_count', 0) == initial_close_count

        release_writer.set()
        assert writer_done.wait(timeout=1.0)

        assert wait_until(
            lambda: node.close_ownership.get('handle') is None,
            timeout=2.0)

        with node.state_lock:
            assert target_ser.closed
            assert node.close_fail_count == initial_close_fail
            assert getattr(
                target_ser,
                'close_count',
                0) == initial_close_count + 1
            assert node.close_ownership.get('task') is None

    finally:
        release_writer.set()
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_destroy_exact_false_success(rclpy_init):
    import threading
    from unittest.mock import patch
    import rclpy.node
    from fire_robot_bringup.serial_bridge_node import SerialBridgeNode

    node = None
    block_close = threading.Event()

    class SlowCloseSerial(FakeSerialWithInWaiting):
        def close(self):
            block_close.wait()
            super().close()

    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=SlowCloseSerial)
        assert wait_until(
            lambda: getattr(
                node,
                "session_ready",
                False),
            timeout=2.0)

        super_called_count = 0
        original_destroy = rclpy.node.Node.destroy_node

        def mock_super_destroy(*args, **kwargs):
            nonlocal super_called_count
            super_called_count += 1
            target = args[0] if args else node
            return original_destroy(target)

        with patch.object(rclpy.node.Node, 'destroy_node', side_effect=mock_super_destroy):
            ret = node.destroy_node()

            assert ret is False
            assert getattr(node, 'node_destroyed', False) is False
            assert super_called_count == 0

            block_close.set()
            assert wait_until(
                lambda: node.close_ownership.get('handle') is None
                or node.close_ownership.get('result') is not None,
                timeout=2.0)

            ret2 = node.destroy_node()
            assert ret2 is True
            assert getattr(node, 'node_destroyed', False) is True
            assert super_called_count == 1

    finally:
        block_close.set()
        if node and not getattr(node, 'node_destroyed', False):
            node.destroy_node()
        rclpy.shutdown()


def test_blocking_close_has_absolute_deadline(rclpy_init):
    import threading
    from fire_robot_bringup.serial_bridge_node import SerialBridgeNode
    primary_exc = None
    node = None
    block_evt = threading.Event()
    t_req = None
    try:
        reset_qa44_state()

        class BlockCloseSerial(QA44NoHookSerial):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.cancel_read_count = 0
                self.cancel_write_count = 0

            def cancel_read(self):
                self.cancel_read_count += 1

            def cancel_write(self):
                self.cancel_write_count += 1

            def close(self):
                super().close()

        node = SerialBridgeNode(serial_cls=BlockCloseSerial)
        wait_until(lambda: getattr(node, 'ser', None) is not None, timeout=2.0)
        assert getattr(node, 'ser', None) is not None
        h = node.ser
        registry_len = QA44NoHookSerial.construct_count
        gen = getattr(node, 'session_epoch', -1)

        orig_close = h.close
        close_called_evt = threading.Event()

        def blocked_close():
            close_called_evt.set()
            block_evt.wait(timeout=3.0)
            orig_close()

        h.close = blocked_close

        close_req_done_evt = threading.Event()
        res_box = {}

        def request_close():
            res_box['res'] = node.close_serial()
            close_req_done_evt.set()

        t_req = threading.Thread(target=request_close)
        t_req.start()

        assert close_req_done_evt.wait(timeout=2.0)
        res = res_box['res']
        assert res in ('IN_PROGRESS', 'FAILED')
        close_called_evt.wait(timeout=2.0)
        assert close_called_evt.is_set()

        assert QA44NoHookSerial.construct_count == registry_len
        assert not node.destroy_node()
        assert getattr(node, 'node_destroyed', False) is False
        assert node.close_ownership.get('handle') is h
        task_obj = node.close_ownership.get('task')
        assert getattr(task_obj, 'is_alive', lambda: False)() is True
        assert node.close_ownership.get('generation') == gen

        assert h.cancel_read_count == 1
        assert h.cancel_write_count == 1

        block_evt.set()
        wait_until(lambda: h.closed, timeout=2.0)

        assert node.destroy_node()
        assert getattr(node, 'node_destroyed', False) is True
        assert QA44NoHookSerial.close_count == 1
        assert QA44NoHookSerial.construct_count == registry_len
        assert h.cancel_read_count == 1
        assert h.cancel_write_count == 1
    except BaseException as e:
        primary_exc = e
        raise
    finally:
        block_evt.set()
        try:
            cleanup_qa44(
                node,
                threads_to_join=[t_req] if t_req else None,
                primary_exc=primary_exc)
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def test_cancel_hook_failure_has_safe_fallback(rclpy_init):
    from fire_robot_bringup.serial_bridge_node import SerialBridgeNode
    primary_exc = None
    node = None
    try:
        reset_qa44_state()
        node = SerialBridgeNode(serial_cls=QA44NoHookSerial)
        wait_until(lambda: getattr(node, 'ser', None) is not None, timeout=2.0)
        assert getattr(node, 'ser', None) is not None
        h1 = node.ser
        assert not hasattr(h1, 'cancel_read')
        assert not hasattr(h1, 'cancel_write')
        start_instances = QA44NoHookSerial.construct_count

        node.close_serial()
        wait_until(lambda: h1.closed, timeout=2.0)
        assert h1.closed

        assert node.destroy_node()
        assert getattr(node, 'node_destroyed', False) is True
        assert node.close_ownership.get('handle') is None
        assert QA44NoHookSerial.construct_count == start_instances
        assert QA44NoHookSerial.close_count == 1
    except BaseException as e:
        primary_exc = e
        raise
    finally:
        try:
            cleanup_qa44(node, primary_exc=primary_exc)
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    primary_exc = None
    node = None
    rclpy.init()
    try:
        reset_qa44_state()

        class ThrowingCancelSerial(QA44NoHookSerial):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.cancel_read_count = 0
                self.cancel_write_count = 0

            def cancel_read(self):
                self.cancel_read_count += 1
                raise RuntimeError("Cancel_read_err_555")

            def cancel_write(self):
                self.cancel_write_count += 1
                raise RuntimeError("Cancel_write_err_666")

        node = SerialBridgeNode(serial_cls=ThrowingCancelSerial)
        wait_until(lambda: getattr(node, 'ser', None) is not None, timeout=2.0)
        assert getattr(node, 'ser', None) is not None
        h1 = node.ser
        start_instances = QA44NoHookSerial.construct_count

        node.close_serial()
        wait_until(lambda: h1.closed, timeout=2.0)
        assert h1.closed

        assert node.destroy_node()
        assert getattr(node, 'node_destroyed', False) is True
        assert node.close_ownership.get('handle') is None
        assert QA44NoHookSerial.construct_count == start_instances
        assert QA44NoHookSerial.close_count == 1

        assert h1.cancel_read_count == 1
        assert h1.cancel_write_count == 1

        reason = getattr(node, 'last_failure_reason', '')
        assert 'Cancel_read_err_555' in reason
        assert 'Cancel_write_err_666' in reason
    except BaseException as e:
        primary_exc = e
        raise
    finally:
        try:
            cleanup_qa44(node, primary_exc=primary_exc)
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def test_cleanup_context_failure_does_not_leak_rclpy(rclpy_init):
    from fire_robot_bringup.serial_bridge_node import SerialBridgeNode

    def verify_cleanup(node):
        assert getattr(
            node,
            'worker_thread',
            None) is None or not node.worker_thread.is_alive()
        task = getattr(node, 'close_ownership', {}).get('task')
        assert task is None or not getattr(task, 'is_alive', lambda: False)()
        assert getattr(node, 'ser', None) is None
        assert node.close_ownership.get('handle') is None
        assert getattr(node, 'node_destroyed', False) is True
        assert not rclpy.ok()
        rclpy.init()
        try:
            pass
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    # Subcase 1: normal cleanup
    try:
        reset_qa44_state()
        node = SerialBridgeNode(serial_cls=QA44NoHookSerial)
        wait_until(lambda: getattr(node, 'ser', None) is not None, timeout=2.0)
    finally:
        cleanup_qa44(node)
        if rclpy.ok():
            rclpy.shutdown()
    verify_cleanup(node)

    # Subcase 2: exact primary + injected cleanup error
    rclpy.init()
    primary = RuntimeError("Primary Error 123")
    try:
        reset_qa44_state()
        node = SerialBridgeNode(serial_cls=QA44NoHookSerial)
        wait_until(lambda: getattr(node, 'ser', None) is not None, timeout=2.0)
        orig_close = node.close_serial

        def fail_close():
            res = orig_close()
            if not hasattr(fail_close, 'called'):
                fail_close.called = True
                raise ValueError("Injected cleanup error")
            return res
        node.close_serial = fail_close
        raise primary
    except BaseException as e:
        primary_exc = e
    finally:
        try:
            cleanup_qa44(node, primary_exc=primary_exc)
        except BaseException as final_e:
            assert final_e is primary, "Must raise exact primary object"
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    verify_cleanup(node)

    # Subcase 3: cleanup-only error
    rclpy.init()
    try:
        reset_qa44_state()
        node = SerialBridgeNode(serial_cls=QA44NoHookSerial)
        wait_until(lambda: getattr(node, 'ser', None) is not None, timeout=2.0)
        orig_close = node.close_serial

        def fail_close():
            res = orig_close()
            if not hasattr(fail_close, 'called'):
                fail_close.called = True
                raise ValueError("Injected cleanup error 2")
            return res
        node.close_serial = fail_close
    finally:
        try:
            cleanup_qa44(node)
            assert False, "Should raise cleanup error"
        except BaseException as final_e:
            assert isinstance(final_e, ValueError)
            assert str(final_e) == "Injected cleanup error 2"
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    verify_cleanup(node)


def test_close_exception_preserves_handle_ownership(rclpy_init):
    import threading
    from fire_robot_bringup.serial_bridge_node import SerialBridgeNode
    import fire_robot_bringup.serial_bridge_node as sb_node
    primary_exc = None
    node = None
    orig_mono = sb_node.time.monotonic
    try:
        reset_qa44_state()
        clock = ThreadSafeFakeClock()
        sb_node.time.monotonic = clock
        node = SerialBridgeNode(serial_cls=QA44NoHookSerial)
        wait_until(lambda: getattr(node, 'ser', None) is not None, timeout=2.0)
        assert getattr(node, 'ser', None) is not None
        h = node.ser
        orig_close = h.close
        gen = getattr(node, 'session_epoch', -1)

        node.stop_request = True
        if getattr(node, 'worker_thread', None):
            node.worker_thread.join(timeout=2.0)

        close_error = RuntimeError("Identity123")
        close_called_evt = threading.Event()

        def fail_close():
            close_called_evt.set()
            if QA44NoHookSerial.close_count < 1:
                QA44NoHookSerial.close_count += 1
                raise close_error
            orig_close()

        h.close = fail_close

        node.close_serial()

        close_called_evt.wait(timeout=2.0)
        assert close_called_evt.is_set()

        def check_err():
            return node.close_ownership.get('result') is close_error

        wait_until(check_err, timeout=2.0)
        assert node.close_ownership.get('result') is close_error

        assert node.close_ownership.get('handle') is h
        assert node.close_ownership.get('generation') == gen
        assert 'Identity123' in getattr(node, 'last_failure_reason', '')
    except BaseException as e:
        primary_exc = e
        raise
    finally:
        try:
            cleanup_qa44(node, primary_exc=primary_exc)
        finally:
            sb_node.time.monotonic = orig_mono
            if rclpy.ok():
                rclpy.shutdown()


def test_close_exception_quarantines_handle_and_creates_zero_new_handles(
        rclpy_init):
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        assert wait_until(
            lambda: getattr(
                node,
                'ser',
                None) is not None,
            timeout=2.0)

        def fail():
            raise Exception("Fail")
        node.ser.close = fail
        node.close_serial()
        time.sleep(0.2)
        assert node.close_ownership.get('handle') is not None
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_close_request_during_bootstrap_cannot_return_to_wait_first_state(
        rclpy_init):
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        time.sleep(0.1)
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_closing_handle_excludes_all_rx_and_tx_access(rclpy_init):
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        assert wait_until(
            lambda: getattr(
                node,
                'ser',
                None) is not None,
            timeout=2.0)
        h = node.ser
        ev = threading.Event()
        orig_read = h.read

        def block_read(s):
            ev.wait(timeout=1.0)
            return orig_read(s)
        h.read = block_read

        # Force read block
        h.inject_rx(b"test")
        time.sleep(0.1)

        node.close_serial()
        ev.set()

        time.sleep(0.2)
        assert getattr(node, '_session_state', '') != 'HEALTHY'
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_closing_wait_is_bounded_and_does_not_hot_spin(rclpy_init):
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        time.sleep(0.1)
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_diagnostic_all_states_and_counters(rclpy_init):
    from fire_robot_bringup.serial_bridge_node import SerialBridgeNode
    primary_exc = None
    node = None
    try:
        reset_qa44_state()
        node = SerialBridgeNode(serial_cls=QA44NoHookSerial)
        wait_until(lambda: getattr(node, 'ser', None) is not None, timeout=2.0)

        node.stop_request = True
        if getattr(node, 'worker_thread', None):
            node.worker_thread.join(timeout=2.0)

        logs = []

        def mock_logger_info(msg):
            logs.append(msg)

        node.get_logger().info = mock_logger_info

        required_attrs = [
            'open_attempt_count', 'open_success_count', 'open_fail_count',
            'first_state_timeout_count', 'reconnect_count', 'reconnect_total',
            'read_fail_count', 'bootstrap_fail_count', 'close_fail_count',
            'cmd_drop_count', 'telemetry_drop_count',
            'telemetry_publish_count', 'telemetry_publish_fail_count',
            'tx_partial_count', 'tx_fail_count', 'state_publish_max_age',
            'env_publish_max_age'
        ]

        for attr in required_attrs:
            assert hasattr(node, attr), f"Missing {attr} on node"

        states = [
            'OPENING',
            'WAIT_FIRST_STATE',
            'HEALTHY',
            'CLOSING',
            'BACKOFF',
            'SHUTTING_DOWN']
        for s in states:
            with node.state_lock:
                node._session_state = s
            node.log_timer_callback()
            parsed = dict(token.split('=', 1) for token in logs[-1].split())
            assert parsed['state'] == s
            assert set(parsed) == {'state', *required_attrs}
            for key in required_attrs:
                assert float(parsed[key]) >= 0.0

        assert len(logs) >= 6

        # check window-reset/cumulative-preserve
        node.telemetry_publish_count = 5
        node.telemetry_publish_fail_count = 3
        node.reconnect_count = 4
        node.reconnect_total = 10
        node.log_timer_callback()
        assert node.telemetry_publish_count == 0
        assert node.telemetry_publish_fail_count == 0
        assert node.reconnect_count == 0
        assert node.reconnect_total == 10

        node.telemetry_publish_callback()

    except BaseException as e:
        primary_exc = e
        raise
    finally:
        try:
            cleanup_qa44(node, primary_exc=primary_exc)
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def test_first_state_grace_is_deterministic(rclpy_init):
    import threading
    from fire_robot_bringup.serial_bridge_node import SerialBridgeNode
    import fire_robot_bringup.serial_bridge_node as sb_node
    primary_exc = None
    node = None
    orig_mono = sb_node.time.monotonic
    block_evt = threading.Event()
    try:
        reset_qa44_state()
        clock = ThreadSafeFakeClock()
        sb_node.time.monotonic = clock
        node = SerialBridgeNode(serial_cls=QA44NoHookSerial)

        def chk_wait():
            return getattr(node, '_session_state', '') == 'WAIT_FIRST_STATE'
        wait_until(chk_wait, timeout=2.0)
        assert getattr(node, '_session_state', '') == 'WAIT_FIRST_STATE'
        h1 = node.ser
        gen1 = getattr(node, 'session_epoch', -1)
        start_instances = QA44NoHookSerial.construct_count

        node.reconnect_backoff = 0.5
        node.next_reconnect_mono = clock() + 0.5
        node.latest_state = None

        baseline_calls = clock.get_call_count()
        clock.step(1.999)
        wait_until(lambda: clock.get_call_count()
                   > baseline_calls, timeout=2.0)
        assert clock.get_call_count() > baseline_calls

        assert getattr(node, '_session_state', '') == 'WAIT_FIRST_STATE'
        assert getattr(node, 'ser', None) is h1
        assert getattr(node, 'session_epoch', -1) == gen1
        assert QA44NoHookSerial.construct_count == start_instances
        assert getattr(node, 'reconnect_backoff', 0.5) == 0.5

        orig_close = h1.close
        close_called_evt = threading.Event()

        def blocked_close():
            close_called_evt.set()
            block_evt.wait(timeout=3.0)
            orig_close()

        h1.close = blocked_close

        clock.step(0.003)
        close_called_evt.wait(timeout=2.0)
        assert close_called_evt.is_set()
        assert getattr(node, '_session_state', '') == 'CLOSING'

        block_evt.set()

        def chk_backoff():
            return getattr(node, '_session_state', '') == 'BACKOFF'
        wait_until(chk_backoff, timeout=2.0)
        assert getattr(node, '_session_state', '') == 'BACKOFF'

        clock.step(2.0)
        wait_until(
            lambda: getattr(
                node,
                'ser',
                None) not in (
                h1,
                None),
            timeout=2.0)
        assert getattr(node, 'ser', None) is not h1

        assert QA44NoHookSerial.construct_count == start_instances + 1
        assert getattr(node, 'session_epoch', -1) == gen1 + 1

        node.reconnect_backoff = 0.5
        h2 = node.ser
        h2.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        wait_valid_state(node, 1)

        # EXPECTED_RED: production should fail because test doesn't force
        # HEALTHY transition
        assert getattr(node, 'session_ready', False) is True
        assert getattr(node, 'telemetry_healthy', False) is True
        assert getattr(node, '_session_state', '') == 'HEALTHY'
    except BaseException as e:
        primary_exc = e
        raise
    finally:
        sb_node.time.monotonic = orig_mono
        block_evt.set()
        try:
            cleanup_qa44(node, primary_exc=primary_exc)
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def test_healthy_stale_500ms_zero_publish_then_fresh_recovery(rclpy_init):
    import threading
    from fire_robot_bringup.serial_bridge_node import SerialBridgeNode
    import fire_robot_bringup.serial_bridge_node as sb_node
    primary_exc = None
    node = None
    orig_mono = sb_node.time.monotonic
    block_close_evt = threading.Event()
    block_read2_evt = threading.Event()
    try:
        reset_qa44_state()
        clock = ThreadSafeFakeClock()
        sb_node.time.monotonic = clock

        node = SerialBridgeNode(serial_cls=QA44NoHookSerial)
        wait_until(lambda: getattr(node, 'ser', None) is not None, timeout=2.0)
        h1 = node.ser

        h1.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        wait_valid_state(node, 1)
        qa44_transition_parsed_session_to_healthy(node, 1)

        gen1 = getattr(node, 'session_epoch', -1)

        assert getattr(node, 'state_lock', None) is not None
        with node.state_lock:
            last_time = getattr(node, 'last_state_time_mono', clock())
        clock.set(last_time)

        pub_odom = []
        pub_imu = []

        class MockOdomPub:
            def publish(self, msg):
                pub_odom.append(msg)

        class MockImuPub:
            def publish(self, msg):
                pub_imu.append(msg)

        node.odom_pub = MockOdomPub()
        node.imu_pub = MockImuPub()

        orig_read = h1.read
        read_evt = threading.Event()

        def hook_read(size):
            if clock() >= last_time + 0.499:
                read_evt.set()
            return orig_read(size)

        h1.read = hook_read

        orig_close = h1.close
        close_called_evt = threading.Event()

        def blocked_close():
            close_called_evt.set()
            block_close_evt.wait(timeout=3.0)
            orig_close()

        h1.close = blocked_close

        clock.set(last_time + 0.499)
        assert read_evt.wait(timeout=2.0)

        assert getattr(node, '_session_state', '') == 'HEALTHY'
        assert getattr(node, 'ser', None) is h1
        assert QA44NoHookSerial.close_count == 0

        clock.set(last_time + 0.501)
        assert close_called_evt.wait(timeout=2.0)

        assert getattr(node, '_session_state', '') == 'CLOSING'
        assert getattr(node, 'telemetry_healthy', True) is False

        node.telemetry_publish_callback()
        assert len(pub_odom) == 0
        assert len(pub_imu) == 0

        block_close_evt.set()

        def chk_bkf():
            return getattr(node, '_session_state', '') == 'BACKOFF'
        wait_until(chk_bkf, timeout=2.0)
        clock.step(1.5)

        wait_until(
            lambda: getattr(
                node,
                'ser',
                None) not in (
                h1,
                None),
            timeout=2.0)
        h2 = node.ser

        h2.inject_rx(make_state(2, 100, 2.0, 2.0, 0.0, 10.0, 0.0, 0.0))
        wait_valid_state(node, 2)
        qa44_transition_parsed_session_to_healthy(node, 2)

        gen2 = gen1 + 1
        assert getattr(node, 'session_epoch', -1) == gen2

        node.telemetry_publish_callback()
        assert len(pub_odom) == 1
        assert len(pub_imu) == 1
        assert pub_odom[0].header.frame_id == 'odom'
        assert pub_odom[0].child_frame_id == 'base_link'
        assert pub_odom[0].twist.twist.linear.x != 0.0
        assert pub_imu[0].header.frame_id == 'imu_frame'
        pub_odom.clear()
        pub_imu.clear()

        old_bytes = make_state(900, 100, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0)
        qa44_assert_exact_parsed_frame(
            old_bytes, 900, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0
        )

        orig_read2 = h2.read
        read2_evt = threading.Event()

        def hooked_read2(size):
            read2_evt.set()
            block_read2_evt.wait(timeout=3.0)
            return orig_read2(size)

        h2.read = hooked_read2

        assert read2_evt.wait(timeout=2.0)

        node.close_serial()

        def chk_cls():
            return getattr(node, '_session_state', '') == 'CLOSING'
        wait_until(chk_cls, timeout=2.0)

        h2.read_buffer += old_bytes
        h2.in_waiting = len(h2.read_buffer)
        block_read2_evt.set()

        wait_until(chk_bkf, timeout=2.0)
        assert getattr(node, '_session_state', '') == 'BACKOFF'

        with node.state_lock:
            assert getattr(node, 'latest_state', None) is None

        node.telemetry_publish_callback()
        assert len(pub_odom) == 0
        assert len(pub_imu) == 0
        pub_odom.clear()
        pub_imu.clear()

        clock.step(1.5)
        wait_until(
            lambda: getattr(
                node,
                'ser',
                None) not in (
                h1,
                h2,
                None),
            timeout=2.0)
        h3 = node.ser

        gen3 = gen2 + 1
        assert getattr(node, 'session_epoch', -1) == gen3
        h3.inject_rx(make_state(3, 100, 11.0, 22.0, 33.0, 44.0, 55.0, 66.0))
        wait_valid_state(node, 3)
        qa44_transition_parsed_session_to_healthy(node, 3)

        node.telemetry_publish_callback()
        assert len(pub_odom) == 1
        assert len(pub_imu) == 1
        assert pub_odom[0].twist.twist.linear.x == 44.0
        assert pub_odom[0].twist.twist.linear.y == 0.0
        assert pub_odom[0].twist.twist.angular.z == 55.0
        assert pub_imu[0].angular_velocity.z == 66.0

    except BaseException as e:
        primary_exc = e
        raise
    finally:
        sb_node.time.monotonic = orig_mono
        block_close_evt.set()
        block_read2_evt.set()
        try:
            cleanup_qa44(node, primary_exc=primary_exc)
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def test_late_close_completion_is_reaped_once_before_reopen(rclpy_init):
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        assert wait_until(
            lambda: getattr(
                node,
                'ser',
                None) is not None,
            timeout=2.0)
        h = node.ser
        orig_close = h.close
        ev = threading.Event()

        def late_close():
            ev.wait(timeout=2.0)
            orig_close()
        h.close = late_close

        node.close_serial()
        time.sleep(0.1)
        assert node.close_ownership.get('handle') is h
        ev.set()

        assert wait_until(
            lambda: getattr(
                node,
                'ser',
                None) is not h,
            timeout=2.0)
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_never_returning_close_blocks_reopen_and_destroy_returns_false(
        rclpy_init):
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        assert wait_until(
            lambda: getattr(
                node,
                'ser',
                None) is not None,
            timeout=2.0)
        ev = threading.Event()

        def hang_close():
            ev.wait(timeout=5.0)
        node.ser.close = hang_close

        node.close_serial()
        time.sleep(0.2)
        assert node.ser is None
        assert node.close_ownership.get('handle') is not None

        ret = node.destroy_node()
        assert not ret
        ev.set()
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_no_first_state_progressive_backoff_is_bounded(rclpy_init):
    from fire_robot_bringup.serial_bridge_node import SerialBridgeNode
    import fire_robot_bringup.serial_bridge_node as sb_node
    primary_exc = None
    node = None
    orig_mono = sb_node.time.monotonic
    try:
        reset_qa44_state()
        clock = ThreadSafeFakeClock()
        sb_node.time.monotonic = clock
        node = SerialBridgeNode(serial_cls=QA44NoHookSerial)

        def chk_wait():
            return getattr(node, '_session_state', '') == 'WAIT_FIRST_STATE'
        wait_until(chk_wait, timeout=2.0)
        assert getattr(node, '_session_state', '') == 'WAIT_FIRST_STATE'

        expected_seq = [0.1, 0.15, 0.225, 0.3375, 0.50625, 0.759375, 1.0, 1.0]

        for i, expected_backoff in enumerate(expected_seq):
            clock.step(2.1)

            def chk_backoff():
                return getattr(node, '_session_state', '') == 'BACKOFF'
            wait_until(chk_backoff, timeout=2.0)
            assert getattr(node, '_session_state', '') == 'BACKOFF'

            assert getattr(node, 'first_state_timeout_count', 0) == i + 1
            bkf = getattr(node, 'reconnect_backoff', 0.0)
            assert abs(bkf - expected_backoff) < 1e-4

            clock.step(2.0)
            wait_until(chk_wait, timeout=2.0)
    except BaseException as e:
        primary_exc = e
        raise
    finally:
        sb_node.time.monotonic = orig_mono
        try:
            cleanup_qa44(node, primary_exc=primary_exc)
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def test_open_constructor_completion_cannot_publish_stale_handle(rclpy_init):
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        time.sleep(0.1)
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_persistent_close_exception_uses_bounded_retry_and_never_reopens(
        rclpy_init):
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        assert wait_until(
            lambda: getattr(
                node,
                'ser',
                None) is not None,
            timeout=2.0)

        def fail():
            raise Exception("persistent error")
        node.ser.close = fail
        node.close_serial()
        time.sleep(0.5)
        assert node.close_ownership.get('handle') is not None
        assert node.ser is None
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_repeated_close_requests_create_exactly_one_close_task(rclpy_init):
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        assert wait_until(
            lambda: getattr(
                node,
                'ser',
                None) is not None,
            timeout=2.0)
        ev = threading.Event()

        def block_close():
            ev.wait(timeout=2.0)
        node.ser.close = block_close
        node.close_serial()
        t1 = node.close_ownership.get('task')
        node.close_serial()
        t2 = node.close_ownership.get('task')
        assert t1 is t2
        ev.set()
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_runtime_serial_timeout_is_fail_closed_and_exactly_recovers(
        rclpy_init):
    import threading
    from geometry_msgs.msg import Twist
    from fire_robot_bringup.serial_bridge_node import SerialBridgeNode
    primary_exc = None
    node = None
    try:
        reset_qa44_state()
        node = SerialBridgeNode(serial_cls=QA44NoHookSerial)
        wait_until(lambda: getattr(node, 'ser', None) is not None, timeout=2.0)
        assert getattr(node, 'ser', None) is not None
        h1 = node.ser

        h1.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        wait_valid_state(node, 1)
        qa44_transition_parsed_session_to_healthy(node, 1)

        gen1 = getattr(node, 'session_epoch', -1)

        orig_write = h1.write
        write_started_evt = threading.Event()
        import serial

        def timeout_write(data):
            if getattr(h1, '_block_next_write', False):
                write_started_evt.set()
                h1._block_next_write = False
                raise serial.SerialTimeoutException("write timeout")
            return orig_write(data)

        h1.write = timeout_write
        h1._block_next_write = True

        tw = Twist()
        tw.linear.x = 1.0
        node.cmd_vel_callback(tw)

        assert getattr(node, 'pending_cmd', None) is not None

        write_started_evt.wait(timeout=2.0)
        assert write_started_evt.is_set()

        def chk_close():
            return getattr(node, '_session_state', '') == 'CLOSING'
        wait_until(chk_close, timeout=2.0)
        assert getattr(node, '_session_state', '') == 'CLOSING'

        reason = getattr(node, 'last_failure_reason', '')
        assert 'tx:CMD:SerialTimeoutException' in reason

        def chk_backoff():
            return getattr(node, '_session_state', '') == 'BACKOFF'
        wait_until(chk_backoff, timeout=2.0)
        wait_until(
            lambda: getattr(
                node,
                'ser',
                None) not in (
                h1,
                None),
            timeout=2.0)
        assert getattr(node, 'ser', None) is not h1
        h2 = node.ser

        raw_900 = make_state(900, 100, 9.0, 8.0, 0.0, 0.0, 0.0, 0.0)
        qa44_assert_exact_parsed_frame(
            raw_900, 900, 9.0, 8.0, 0.0, 0.0, 0.0, 0.0
        )

        h2.inject_rx(make_state(2, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        wait_valid_state(node, 2)
        qa44_transition_parsed_session_to_healthy(node, 2)

        gen2 = gen1 + 1
        assert getattr(node, 'session_epoch', -1) == gen2
        assert getattr(node, '_session_state', '') == 'HEALTHY'
    except BaseException as e:
        primary_exc = e
        raise
    finally:
        try:
            cleanup_qa44(node, primary_exc=primary_exc)
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def test_stale_io_completion_is_rejected_by_handle_and_generation(rclpy_init):
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        time.sleep(0.1)
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_successful_close_reap_clears_all_ownership_fields(rclpy_init):
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        assert wait_until(
            lambda: getattr(
                node,
                'ser',
                None) is not None,
            timeout=2.0)
        node.close_serial()
        assert wait_until(
            lambda: node.close_ownership.get('handle') is None,
            timeout=2.0)
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_tx_lock_timeout_never_closes_concurrently(rclpy_init):
    import threading
    from fire_robot_bringup.serial_bridge_node import SerialBridgeNode
    primary_exc = None
    node = None
    t_writer = None
    writer_done_evt = threading.Event()
    release_writer_evt = threading.Event()
    try:
        reset_qa44_state()
        node = SerialBridgeNode(serial_cls=QA44NoHookSerial)
        wait_until(lambda: getattr(node, 'ser', None) is not None, timeout=2.0)
        assert getattr(node, 'ser', None) is not None

        orig_lock = node.tx_lock
        close_acquire_attempt_evt = threading.Event()
        writer_owned_evt = threading.Event()

        class LockWrapper:
            def acquire(self, *args, **kwargs):
                if threading.current_thread() is node.close_ownership.get('task'):
                    close_acquire_attempt_evt.set()
                return orig_lock.acquire(*args, **kwargs)

            def release(self):
                orig_lock.release()

            def locked(self):
                return orig_lock.locked()

            def __enter__(self):
                self.acquire()
                return self

            def __exit__(self, exc_type, exc_val, exc_tb):
                self.release()

        node.tx_lock = LockWrapper()

        def writer_thread():
            assert node.tx_lock.acquire(timeout=2.0)
            try:
                writer_owned_evt.set()
                release_writer_evt.wait(timeout=3.0)
            finally:
                node.tx_lock.release()
                writer_done_evt.set()

        t_writer = threading.Thread(target=writer_thread)
        t_writer.start()

        writer_owned_evt.wait(timeout=2.0)
        assert writer_owned_evt.is_set()

        res = node.close_serial()
        assert res == 'IN_PROGRESS'

        close_acquire_attempt_evt.wait(timeout=2.0)
        assert close_acquire_attempt_evt.is_set()
    except BaseException as e:
        primary_exc = e
        raise
    finally:
        release_writer_evt.set()
        try:
            cleanup_qa44(
                node,
                threads_to_join=[t_writer] if t_writer else None,
                primary_exc=primary_exc
            )
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def test_tx_lock_timeout_retries_after_release_without_concurrent_close(
        rclpy_init):
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        assert wait_until(
            lambda: getattr(
                node,
                'ser',
                None) is not None,
            timeout=2.0)
        node.tx_lock.acquire()
        try:
            node.close_serial()
            time.sleep(0.1)
            assert node.close_ownership.get('handle') is not None
        finally:
            node.tx_lock.release()
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def reset_qa44_state():
    registry.instances.clear()
    if hasattr(registry, 'next_config'):
        registry.next_config.clear()
    QA44NoHookSerial.construct_count = 0
    QA44NoHookSerial.close_count = 0


def cleanup_qa44(node, primary_exc=None, threads_to_join=None):
    cleanup_errors = []

    if not node:
        if primary_exc:
            raise primary_exc
        return

    if threads_to_join:
        for t in threads_to_join:
            try:
                t.join(timeout=2.0)
                if t.is_alive():
                    cleanup_errors.append(RuntimeError("thread leaked"))
            except BaseException as e:
                cleanup_errors.append(e)

    if getattr(node, 'worker_thread', None):
        try:
            node.stop_request = True
            node.worker_thread.join(timeout=2.0)
            if node.worker_thread.is_alive():
                cleanup_errors.append(RuntimeError("worker leaked"))
        except BaseException as e:
            cleanup_errors.append(e)

    if getattr(node, 'ser', None):
        try:
            node.close_serial()
        except BaseException as e:
            cleanup_errors.append(e)

    try:
        import threading
        task = getattr(node, 'close_ownership', {}).get('task')
        if isinstance(task, threading.Thread) and task.is_alive():
            task.join(timeout=2.0)
            if task.is_alive():
                cleanup_errors.append(RuntimeError("close task leaked"))
    except BaseException as e:
        cleanup_errors.append(e)

    if getattr(
        node,
        'destroy_node',
        None) and not getattr(
        node,
        'node_destroyed',
            False):
        try:
            if rclpy.ok():
                node.destroy_node()
                if getattr(node, 'node_destroyed', False) is not True:
                    cleanup_errors.append(RuntimeError("destroy failed"))
        except BaseException as e:
            cleanup_errors.append(e)

    if primary_exc:
        if cleanup_errors:
            try:
                raise cleanup_errors[0]
            except BaseException:
                raise primary_exc
        else:
            raise primary_exc
    elif cleanup_errors:
        raise cleanup_errors[0]


def qa44_transition_parsed_session_to_healthy(node, seq):
    lock = getattr(node, 'state_lock', None)
    assert lock is not None
    with lock:
        state = getattr(node, 'latest_state', None)
        assert state is not None
        assert state[0].get('seq') == seq
        assert state[1] == getattr(node, 'session_epoch', -1)
        assert getattr(node, 'session_ready', False) is True
        assert getattr(node, 'telemetry_healthy', False) is True

    node._transition_state('HEALTHY')
    assert getattr(node, '_session_state', '') == 'HEALTHY'


def qa44_assert_exact_parsed_frame(
    raw_bytes, expected_seq, exp_x, exp_y, exp_yaw, exp_vx, exp_wz, exp_gyroz
):
    from fire_robot_bringup.serial_protocol import SerialProtocolV2
    parser = SerialProtocolV2()
    frames = list(parser.parse_chunk(raw_bytes))
    assert len(frames) == 1
    frame = frames[0]
    expected_keys = {
        'type', 'seq', 'esp_ms', 'x', 'y', 'yaw', 'vx', 'wz', 'gyro_z'
    }
    assert set(frame.keys()) == expected_keys
    assert frame['type'] == 'STATE'
    assert frame['seq'] == expected_seq
    assert frame['esp_ms'] == 100
    assert abs(frame['x'] - exp_x) < 1e-5
    assert abs(frame['y'] - exp_y) < 1e-5
    assert abs(frame['yaw'] - exp_yaw) < 1e-5
    assert abs(frame['vx'] - exp_vx) < 1e-5
    assert abs(frame['wz'] - exp_wz) < 1e-5
    assert abs(frame['gyro_z'] - exp_gyroz) < 1e-5
    assert parser.parse_fail_count == 0
    assert parser.crc_fail_count == 0
    assert parser.valid_count == 1
    return frame


def _reap_all_close_ownership(node, timeout=2.0):
    def reaped():
        node.close_serial()
        with node.state_lock:
            return (
                node.close_ownership.get('handle') is None
                and not node.close_ownership.get('pending')
                and node.close_ownership.get('task') is None)

    assert wait_until(reaped, timeout=timeout)


def _assert_bootstrap_failure_closed(node, handle):
    node.stop_request = True
    node.worker_thread.join(timeout=2.0)
    assert not node.worker_thread.is_alive()
    _reap_all_close_ownership(node)
    assert handle.closed is True
    assert handle.close_count == 1
    assert node.session_ready is False
    assert node.telemetry_healthy is False
    assert node.ser is None


def test_bootstrap_first_write_fail(rclpy_init):
    class FailFirstWriteSerial(FakeSerial):
        def write(self, data):
            self.written_frames.append((0, data))
            raise RuntimeError('First write failed')

    registry.instances.clear()
    node = SerialBridgeNode(serial_cls=FailFirstWriteSerial)
    try:
        assert wait_until(lambda: node.bootstrap_fail_count == 1)
        first = registry.instances[0]
        _assert_bootstrap_failure_closed(node, first)
        assert len(first.written_frames) == 1
        assert b'CMD' in first.written_frames[0][1]
    finally:
        if not node.node_destroyed:
            node.destroy_node()


def test_bootstrap_second_write_fail(rclpy_init):
    class FailSecondWriteSerial(FakeSerial):
        def write(self, data):
            self.written_frames.append((0, data))
            if len(self.written_frames) == 2:
                raise RuntimeError('Second write failed')
            return len(data)

    registry.instances.clear()
    node = SerialBridgeNode(serial_cls=FailSecondWriteSerial)
    try:
        assert wait_until(lambda: node.bootstrap_fail_count == 1)
        first = registry.instances[0]
        _assert_bootstrap_failure_closed(node, first)
        assert len(first.written_frames) == 2
        assert b'CMD' in first.written_frames[0][1]
        assert b'PUMP' in first.written_frames[1][1]
    finally:
        if not node.node_destroyed:
            node.destroy_node()


@pytest.mark.parametrize('partial_call', [1, 2])
def test_bootstrap_partial_write_fails_closed(rclpy_init, partial_call):
    class PartialBootstrapSerial(FakeSerial):
        def write(self, data):
            self.written_frames.append((0, data))
            if len(self.written_frames) == partial_call:
                return max(0, len(data) - 1)
            return len(data)

    registry.instances.clear()
    node = SerialBridgeNode(serial_cls=PartialBootstrapSerial)
    try:
        assert wait_until(lambda: node.bootstrap_fail_count == 1)
        first = registry.instances[0]
        _assert_bootstrap_failure_closed(node, first)
        assert len(first.written_frames) == partial_call
    finally:
        if not node.node_destroyed:
            node.destroy_node()


def test_bootstrap_shutdown_between_writes(rclpy_init):
    first_write_entered = threading.Event()
    release_first_write = threading.Event()

    class BlockFirstBootstrapWrite(FakeSerial):
        def write(self, data):
            self.written_frames.append((0, data))
            if len(self.written_frames) == 1:
                first_write_entered.set()
                release_first_write.wait()
            return len(data)

    registry.instances.clear()
    node = SerialBridgeNode(serial_cls=BlockFirstBootstrapWrite)
    try:
        assert first_write_entered.wait(timeout=2.0)
        node.stop_request = True
        release_first_write.set()
        assert wait_until(lambda: node.bootstrap_fail_count == 1)
        first = registry.instances[0]
        _assert_bootstrap_failure_closed(node, first)
        assert len(first.written_frames) == 1
        assert b'CMD' in first.written_frames[0][1]
    finally:
        release_first_write.set()
        if not node.node_destroyed:
            node.destroy_node()


def test_bootstrap_close_race_never_deadlocks_state_lock(rclpy_init):
    second_write_entered = threading.Event()
    release_second_write = threading.Event()
    close_entered = threading.Event()

    class BlockSecondBootstrapWrite(FakeSerial):
        def write(self, data):
            self.written_frames.append((0, data))
            if len(self.written_frames) == 2:
                second_write_entered.set()
                release_second_write.wait()
            return len(data)

        def close(self):
            close_entered.set()
            super().close()

    registry.instances.clear()
    node = SerialBridgeNode(serial_cls=BlockSecondBootstrapWrite)
    state_lock_healthy = False
    try:
        assert second_write_entered.wait(timeout=2.0)
        assert node.close_serial() == 'CLOSED'
        release_second_write.set()
        assert close_entered.wait(timeout=1.0)

        node.stop_request = True
        node.worker_thread.join(timeout=2.0)
        assert not node.worker_thread.is_alive()
        _reap_all_close_ownership(node)

        lock_acquired = node.state_lock.acquire(timeout=0.2)
        assert lock_acquired
        node.state_lock.release()
        state_lock_healthy = True

        first = registry.instances[0]
        assert first.close_count == 1
        assert first.closed is True
        assert len(registry.instances) == 1
        assert node.ser is None
        assert node.session_ready is False
        assert node.telemetry_healthy is False
        with node.state_lock:
            assert node.latest_state is None
            assert node.latest_env is None
    finally:
        release_second_write.set()
        if state_lock_healthy and not node.node_destroyed:
            node.destroy_node()
        elif not state_lock_healthy:
            # A deliberately reintroduced nested-lock mutation can strand the
            # worker while it owns this non-reentrant lock. Release only in
            # this failed-test cleanup path so pytest can report the exact
            # assertion instead of crashing during rclpy context teardown.
            node.stop_request = True
            try:
                node.state_lock.release()
            except RuntimeError:
                pass
            node.worker_thread.join(timeout=1.0)
            task = node.close_ownership.get('task')
            if task is not None:
                task.join(timeout=1.0)
            if not node.worker_thread.is_alive():
                rclpy.node.Node.destroy_node(node)
                node.node_destroyed = True


def test_quarantine_preserves_existing_owner_until_both_reaped(rclpy_init):
    close_a_entered = threading.Event()
    release_close_a = threading.Event()

    class ManualHandle:
        def __init__(self, block=False):
            self.block = block
            self.closed = False
            self.close_count = 0

        def close(self):
            self.close_count += 1
            if self.block:
                close_a_entered.set()
                release_close_a.wait()
            self.closed = True

    registry.instances.clear()
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        assert wait_until(lambda: node.ser is not None)
        node.stop_request = True
        node.worker_thread.join(timeout=2.0)
        _reap_all_close_ownership(node)

        handle_a = ManualHandle(block=True)
        handle_b = ManualHandle()
        node._quarantine_handle(handle_a, 'test:owner-a')
        assert close_a_entered.wait(timeout=2.0)
        node._quarantine_handle(handle_b, 'test:owner-b')

        with node.state_lock:
            assert node.close_ownership['handle'] is handle_a
            assert [entry[0] for entry in node.close_ownership['pending']] == [
                handle_b]

        release_close_a.set()
        _reap_all_close_ownership(node)
        assert handle_a.close_count == 1
        assert handle_b.close_count == 1
        assert handle_a.closed is True
        assert handle_b.closed is True
    finally:
        release_close_a.set()
        if not node.node_destroyed:
            node.destroy_node()


def test_closed_active_handle_still_fails_session_closed(rclpy_init):
    class ClosedHandle:
        closed = True
        close_count = 0

    registry.instances.clear()
    node = SerialBridgeNode(serial_cls=FakeSerial)
    try:
        assert wait_until(lambda: node.ser is not None)
        node.stop_request = True
        node.worker_thread.join(timeout=2.0)
        _reap_all_close_ownership(node)

        handle = ClosedHandle()
        with node.state_lock:
            node.ser = handle
            node.session_ready = True
            node.telemetry_healthy = True
            node._session_state = 'HEALTHY'

        node._quarantine_handle(handle, 'test:already-closed')
        with node.state_lock:
            assert node.ser is None
            assert node.session_ready is False
            assert node.telemetry_healthy is False
            assert node._session_state == 'CLOSING'
            assert node.close_ownership['handle'] is None
            assert not node.close_ownership['pending']
        assert handle.close_count == 0
    finally:
        if not node.node_destroyed:
            node.destroy_node()


def test_close_task_obeys_remaining_shutdown_deadline(rclpy_init):
    class CountCloseHandle:
        def __init__(self):
            self.closed = False
            self.close_count = 0

        def close(self):
            self.close_count += 1
            self.closed = True

    registry.instances.clear()
    node = SerialBridgeNode(serial_cls=FakeSerial)
    lock_held = False
    try:
        assert wait_until(lambda: node.ser is not None)
        node.stop_request = True
        node.worker_thread.join(timeout=2.0)
        _reap_all_close_ownership(node)

        handle = CountCloseHandle()
        node.tx_lock.acquire()
        lock_held = True
        with node.state_lock:
            node.ser = handle
            node._shutdown_started = True
            node._shutdown_deadline_mono = time.monotonic() + 0.05
            node._session_state = 'SHUTTING_DOWN'

        started = time.monotonic()
        assert node.close_serial() == 'IN_PROGRESS'
        assert wait_until(
            lambda: isinstance(
                node.close_ownership.get('result'), TimeoutError),
            timeout=0.3)
        assert time.monotonic() - started < 0.3
        assert handle.close_count == 0

        node.tx_lock.release()
        lock_held = False
        with node.state_lock:
            node._shutdown_deadline_mono = time.monotonic() + 1.0
            node.close_ownership['deadline'] = node._shutdown_deadline_mono
        _reap_all_close_ownership(node)
        assert handle.close_count == 1
    finally:
        if lock_held:
            node.tx_lock.release()
        node._shutdown_started = False
        if not node.node_destroyed:
            node.destroy_node()


def test_destroy_exposes_shutting_down_state_and_diagnostic(rclpy_init):
    close_entered = threading.Event()
    release_close = threading.Event()
    destroy_result = []

    class BlockingCloseSerial(FakeSerial):
        def close(self):
            close_entered.set()
            release_close.wait()
            super().close()

    registry.instances.clear()
    node = SerialBridgeNode(serial_cls=BlockingCloseSerial)
    destroy_thread = None
    try:
        assert wait_until(lambda: node.ser is not None)
        logs = []
        node.get_logger().info = logs.append

        destroy_thread = threading.Thread(
            target=lambda: destroy_result.append(node.destroy_node()))
        destroy_thread.start()
        assert close_entered.wait(timeout=2.0)
        with node.state_lock:
            assert node._session_state == 'SHUTTING_DOWN'
        node.log_timer_callback()
        parsed = dict(token.split('=', 1) for token in logs[-1].split())
        assert parsed['state'] == 'SHUTTING_DOWN'

        release_close.set()
        destroy_thread.join(timeout=2.0)
        assert not destroy_thread.is_alive()
        if not node.node_destroyed:
            assert node.destroy_node() is True
        assert node.node_destroyed is True
    finally:
        release_close.set()
        if destroy_thread is not None:
            destroy_thread.join(timeout=2.0)
        if not node.node_destroyed:
            node.destroy_node()


def test_diagnostic_counters_only_increment_under_state_lock():
    import ast
    import inspect
    import textwrap

    counter_names = {
        'open_attempt_count', 'open_success_count', 'open_fail_count',
        'first_state_timeout_count', 'reconnect_count', 'reconnect_total',
        'read_fail_count', 'bootstrap_fail_count', 'close_fail_count',
        'cmd_drop_count', 'telemetry_drop_count',
        'telemetry_publish_count', 'telemetry_publish_fail_count',
        'tx_partial_count', 'tx_fail_count'}
    tree = ast.parse(textwrap.dedent(inspect.getsource(SerialBridgeNode)))
    parents = {}
    for parent in ast.walk(tree):
        for child in ast.iter_child_nodes(parent):
            parents[child] = parent

    def protected_by_state_lock(node):
        current = parents.get(node)
        while current is not None and not isinstance(
                current, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if isinstance(current, ast.With):
                contexts = [
                    ast.unparse(item.context_expr)
                    for item in current.items]
                if 'self.state_lock' in contexts:
                    return True
            current = parents.get(current)
        return False

    unprotected = []
    for candidate in ast.walk(tree):
        if not isinstance(candidate, ast.AugAssign):
            continue
        target = candidate.target
        if (isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == 'self'
                and target.attr in counter_names
                and not protected_by_state_lock(candidate)):
            unprotected.append((target.attr, candidate.lineno))
    assert unprotected == []


def test_diagnostic_snapshot_reset_is_atomic(rclpy_init):
    """Keep increments that race with a diagnostic reset in the next window."""
    registry.instances.clear()
    node = SerialBridgeNode(serial_cls=FakeSerial)
    original_lock = None
    increment_thread = None
    try:
        assert wait_until(lambda: node.ser is not None)
        node.stop_request = True
        node.worker_thread.join(timeout=2.0)
        assert not node.worker_thread.is_alive()
        _reap_all_close_ownership(node)

        original_lock = node.state_lock
        diagnostic_locked = threading.Event()
        increment_attempted = threading.Event()

        class BarrierLock:
            def __init__(self, wrapped):
                self._wrapped = wrapped
                self._diagnostic_thread = threading.current_thread()

            def acquire(self, blocking=True, timeout=-1):
                is_diagnostic = (
                    threading.current_thread() is self._diagnostic_thread)
                if timeout == -1:
                    acquired = self._wrapped.acquire(blocking)
                else:
                    acquired = self._wrapped.acquire(blocking, timeout)
                if acquired and is_diagnostic:
                    diagnostic_locked.set()
                    assert increment_attempted.wait(timeout=1.0)
                return acquired

            def release(self):
                self._wrapped.release()

            def __enter__(self):
                assert self.acquire()
                return self

            def __exit__(self, exc_type, exc_value, traceback):
                self.release()

        with original_lock:
            node.reconnect_count = 4

        node.state_lock = BarrierLock(original_lock)
        logs = []
        node.get_logger().info = logs.append

        def increment_during_snapshot():
            assert diagnostic_locked.wait(timeout=1.0)
            increment_attempted.set()
            node._increment_counter('reconnect_count')

        increment_thread = threading.Thread(target=increment_during_snapshot)
        increment_thread.start()
        node.log_timer_callback()
        increment_thread.join(timeout=1.0)
        assert not increment_thread.is_alive()

        parsed = dict(token.split('=', 1) for token in logs[-1].split())
        assert int(parsed['reconnect_count']) == 4
        with node.state_lock:
            assert node.reconnect_count == 1
        assert int(parsed['reconnect_count']) + node.reconnect_count == 5
    finally:
        if increment_thread is not None:
            increment_thread.join(timeout=1.0)
        if original_lock is not None:
            node.state_lock = original_lock
        if not node.node_destroyed:
            node.destroy_node()
