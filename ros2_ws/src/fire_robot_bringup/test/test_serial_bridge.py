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


def keep_alive_thread(node):
    def _run():
        seq = 100
        while getattr(node, 'running', True) and not getattr(node, 'node_destroyed', False):
            if getattr(registry, 'instances', []):
                registry.instances[-1].inject_rx(
                    make_state(seq, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                seq = (seq + 1) % 256
            time.sleep(0.1)
    t = threading.Thread(target=_run, daemon=True)
    t.start()
    return t


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
        keep_alive_thread(node)
        assert wait_until(lambda: node.session_ready)
        inst = registry.instances[0]

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
        keep_alive_thread(node)
        assert wait_until(lambda: node.session_ready)
        inst1 = registry.instances[-1]

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
        keep_alive_thread(node)
        assert wait_until(lambda: node.session_ready)
        inst1 = registry.instances[-1]

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
        keep_alive_thread(node)
        assert wait_until(lambda: node.session_ready)
        inst = registry.instances[-1]

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
        keep_alive_thread(node)
        assert wait_until(lambda: node.session_ready)
        inst = registry.instances[-1]

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
        node = SerialBridgeNode(serial_cls=FakeSerial)
        inst = registry.instances[-1]

        inst.inject_rx(make_state(1, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        assert wait_until(lambda: node.session_ready)

        write_evt = threading.Event()
        inst.block_write_event = write_evt

        msg = Twist()
        msg.linear.x = 1.0
        node.cmd_vel_callback(msg)

        inst.inject_rx(make_state(2, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        write_evt.set()
        assert wait_until(
            lambda: node.latest_state is not None and node.latest_state[0]['seq'] == 2)
    finally:
        if node:
            node.destroy_node()
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

        class BlockingPub:
            def publish(self, msg):
                with pub_lock:
                    pass
        node.odom_pub = BlockingPub()

        pub_lock.acquire()

        inst.inject_rx(make_state(2, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        t = threading.Thread(target=node.telemetry_publish_callback)
        t.start()
        time.sleep(0.1)

        success = node.destroy_node()
        assert not success
        assert not getattr(node, 'node_destroyed', False)

        pub_lock.release()
        t.join()

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
