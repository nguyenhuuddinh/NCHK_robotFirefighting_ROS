"""Tests for node logic with mock serial."""
import json
import rclpy
import threading
import time

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from fire_robot_bringup.serial_bridge_node import SerialBridgeNode
from fire_robot_bringup.serial_protocol import crc16_ccitt_false


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
        super().__init__(port, baudrate, timeout, write_timeout)
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
                raise Exception("Mock write error")
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


def test_env_json_schema():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        inst = registry.instances[0]

        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: node.session_ready)

        published = []

        class MockPub:
            def publish(self, msg):
                published.append(msg)
        node.env_pub = MockPub()

        inst.inject_rx(make_env(2, 100, 1, 120.5, 32.1, 11.8, 1))
        assert wait_until(lambda: node.latest_env is not None)

        node.telemetry_publish_callback()
        assert len(published) == 1
        msg = published[0].data
        d = json.loads(msg)
        assert "fire" in d
        assert "gas" in d
        assert "temp" in d
        assert "batt" in d
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_odom_imu_quaternion():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
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


def test_exact_bootstrap_and_offline_drop():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
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


def test_no_local_nonzero_after_shutdown():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
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


def test_cancel_close_no_unblock_retry_shutdown():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()

        class StubbornSerial(FakeSerial):
            def cancel_read(self):
                pass

            def close(self):
                pass

        node = SerialBridgeNode(serial_cls=StubbornSerial)
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
        assert t1 - t0 < 2.5
        assert not node.node_destroyed

        read_evt.set()
        time.sleep(0.1)

        success2 = node.destroy_node()
        assert success2
        assert node.node_destroyed
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_cancel_yields_zero_and_closes():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
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


def test_typed_mapping_cmd_fire_pump():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
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

        assert wait_until(lambda: any(b'1.0' in w[1] for w in inst.snapshot_writes()))
        assert wait_until(lambda: any(b'FIRE' in w[1] for w in inst.snapshot_writes()))
        assert wait_until(lambda: any(b'PUMP,2' in w[1] and b',1*' in w[1]
                                      for w in inst.snapshot_writes()))
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_latest_wins_cmd_pump_refresh():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
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


def test_stale_state_fails_closed_reconnect_fresh():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)

        registry.instances[0].inject_rx(make_state(200, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: node.session_ready)

        # State stale -> reconnect
        time.sleep(0.6)
        assert len(registry.instances) > 1

        msg = Twist()
        msg.linear.x = 1.0
        node.cmd_vel_callback(msg)
        assert node.cmd_drop_count >= 1
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_full_duplex_rate():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)

        def run_full_duplex():
            for _ in range(10):
                if not getattr(node, 'running', False):
                    break
                if getattr(registry, 'instances', []):
                    registry.instances[-1].inject_rx(
                        make_state(_ * 2 + 1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
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


def test_rx_first_drain_before_tx():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
        inst = node.ser
        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: (getattr(node, "session_ready", False) and getattr(node, "telemetry_healthy", False)),  # noqa: E501
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

            assert wait_until(lambda: getattr(node, "latest_state", None)
                              is not None and node.latest_state[0]['seq'] == 2, timeout=2.0)
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


def test_blocked_publisher_lifecycle():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
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


def test_reconnect_between_odom_imu_env():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
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


def test_pause_before_tx_gate_shutdown_wins():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
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


def test_block_pump_refresh_write_callback_unblocked():
    rclpy.init()
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


def test_presend_cmd_zero_and_pump_off_supersede_unsafe_items():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: (getattr(node, "session_ready", False) and getattr(node, "telemetry_healthy", False)),  # noqa: E501
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
            lambda: any(b'CMD' in w[1] and b'0.0' in w[1] for w in first_ser.snapshot_writes()),
            timeout=2.0
        )
        assert wait_until(
            lambda: any(b'PUMP' in w[1] and b'0' in w[1] for w in first_ser.snapshot_writes()),
            timeout=2.0
        )

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


def test_bootstrap_partial():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        registry.next_config.append({
            'raise_on_write_after': 5
        })

        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        # The first instance will partial write during bootstrap!
        assert wait_until(lambda: len(registry.instances) > 1, timeout=2.0)
        assert getattr(node, 'reconnect_count', 0) > 1
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_bootstrap_error():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        registry.next_config.append({
            'raise_on_write': True
        })

        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        # The first instance will raise on write during bootstrap!
        assert wait_until(lambda: len(registry.instances) > 1, timeout=2.0)
        assert getattr(node, 'reconnect_count', 0) > 1
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_normal_cmd_partial():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: (getattr(node, "session_ready", False) and getattr(node, "telemetry_healthy", False)),  # noqa: E501
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


def test_normal_cmd_error():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: (getattr(node, "session_ready", False) and getattr(node, "telemetry_healthy", False)),  # noqa: E501
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


def test_pump_refresh_partial():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: (getattr(node, "session_ready", False) and getattr(node, "telemetry_healthy", False)),  # noqa: E501
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


def test_pump_refresh_error():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: (getattr(node, "session_ready", False) and getattr(node, "telemetry_healthy", False)),  # noqa: E501
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


def test_fresh_state_before_deadline_does_not_reconnect():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
        first_ser = node.ser

        import time
        for _ in range(5):
            first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            time.sleep(0.1)

        assert node.ser is first_ser
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_destroy_retry():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)

        node.tx_lock.acquire()

        ret = node.destroy_node()
        assert not ret

        assert node.running
        assert not node.stop_request
        assert not node.node_destroyed

        node.tx_lock.release()

        ret = node.destroy_node()
        assert ret

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


def test_real_like_idle_poll_rate():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)

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
        assert wait_until(lambda: not getattr(node, "telemetry_healthy", False)
                          or getattr(node, "ser", None) is not first_ser, timeout=2.0)

    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


def test_exact_baseline_order():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        node.state_stale_timeout_ms = 10000
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
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


def test_main_destroy_retry():
    import fire_robot_bringup.serial_bridge_node as target_module

    class MockNode:
        def __init__(self):
            self.calls = 0

        def destroy_node(self):
            self.calls += 1
            if self.calls < 3:
                return False
            return True

    old_init = getattr(target_module.rclpy, 'init', None)
    old_spin = getattr(target_module.rclpy, 'spin', None)
    old_ok = getattr(target_module.rclpy, 'ok', None)
    old_shutdown = getattr(target_module.rclpy, 'shutdown', None)
    old_SerialBridgeNode = getattr(target_module, 'SerialBridgeNode', None)

    node = MockNode()

    def fake_init(args=None): pass
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


def test_bootstrap_lifecycle_gate():
    rclpy.init()
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

        assert wait_until(lambda: getattr(node, "stop_request", False), timeout=2.0)
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


def test_live_read_error_recovery():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
        first_ser = node.ser

        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: getattr(node, "session_ready", False), timeout=2.0)

        # Inject read error
        first_ser.raise_on_read = True
        first_ser.rx_event.set()  # Wake up reader

        # Worker should catch it, close first_ser, and open a new one
        assert wait_until(lambda: getattr(node, "ser", None)
                          is not None and node.ser is not first_ser, timeout=2.0)
        assert getattr(node.worker_thread, "is_alive")()

        second_ser = node.ser
        second_ser.inject_rx(make_state(2, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: getattr(node, "session_ready", False), timeout=2.0)
        assert wait_until(lambda: getattr(node, "latest_state", None)
                          is not None and node.latest_state[0]['seq'] == 2, timeout=2.0)

    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_presend_newer_supersedes_older():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerialWithInWaiting)
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: (getattr(node, "session_ready", False) and getattr(node, "telemetry_healthy", False)),  # noqa: E501
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
            lambda: any(b'CMD' in w[1] and b'0.9' in w[1] for w in first_ser.snapshot_writes()),
            timeout=2.0
        )
        assert wait_until(
            lambda: any(b'FIRE' in w[1] and b'0.9' in w[1] for w in first_ser.snapshot_writes()),
            timeout=2.0
        )

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


def test_persistent_read_error_bounded_reconnect():
    rclpy.init()
    node = None
    try:
        registry.instances.clear()

        class PersistentErrorSerial(FakeSerialWithInWaiting):
            persistent_raise_on_read = True
            init_times = []

            def __init__(self, port, baudrate, timeout=0.01, write_timeout=0.1):
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

        assert wait_until(lambda: len(PersistentErrorSerial.init_times) >= 2, timeout=2.0)
        delay = PersistentErrorSerial.init_times[1] - PersistentErrorSerial.init_times[0]
        assert delay >= 0.08, f"First reopen too fast: {delay:.6f}s"
        assert getattr(node, "reconnect_backoff", 0.0) > 0.0
        assert getattr(node, "next_reconnect_mono", 0.0) > 0.0

        # Remove fault
        PersistentErrorSerial.persistent_raise_on_read = False

        # Wait for next instance
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
        node.ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        assert wait_until(lambda: getattr(node, "session_ready", False), timeout=2.0)
        assert wait_until(lambda: getattr(node, "telemetry_healthy", False), timeout=2.0)

        assert getattr(node, "reconnect_backoff", 1.0) == 0.0
        assert getattr(node, "next_reconnect_mono", 1.0) == 0.0
    finally:
        if node and not getattr(node, 'node_destroyed', False):
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


def test_tx_backoff_preserve():
    rclpy.init()
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
        assert wait_until(lambda: getattr(node, "ser", None) is not None, timeout=2.0)
        first_ser = node.ser
        first_ser.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        assert wait_until(lambda: getattr(node, "session_ready", False), timeout=2.0)
        assert wait_until(lambda: getattr(node, "telemetry_healthy", False), timeout=2.0)

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
            lambda: any(b'CMD' in w[1] and b'0.5' in w[1] for w in first_ser.snapshot_writes()),
            timeout=2.0
        )

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
