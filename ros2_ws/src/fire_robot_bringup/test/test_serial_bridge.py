"""Tests for node logic with mock serial."""
import json
import rclpy
import threading
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from fire_robot_bringup.serial_bridge_node import SerialBridgeNode


class MockRegistry:
    """Registry to keep track of fake serial instances."""

    def __init__(self):
        """Initialize."""
        self.instances = []
        self.next_config = []
        self.lock = threading.Lock()

    def get_config(self):
        """Get next config."""
        with self.lock:
            if self.next_config:
                return self.next_config.pop(0)
            return {}


registry = MockRegistry()


class FakeSerial:
    """Mock serial interface."""

    def __init__(self, port, baudrate, timeout=0.01, write_timeout=0.1):
        """Initialize FakeSerial."""
        self.port = port
        self.baudrate = baudrate
        self.is_open = True
        self.closed = False
        self.written_frames = []
        self.read_buffer = b""

        cfg = registry.get_config()
        self.raise_on_write = cfg.get('raise_on_write', False)
        self.raise_on_read = cfg.get('raise_on_read', False)
        self.partial_write = cfg.get('partial_write', False)
        self.block_write_event = cfg.get('block_write_event', None)
        self.block_read_event = cfg.get('block_read_event', None)

        with registry.lock:
            registry.instances.append(self)

    def write(self, data: bytes):
        """Mock write."""
        if self.block_write_event:
            self.block_write_event.wait()
        if self.raise_on_write:
            raise Exception("Mock write error")
        if self.partial_write:
            half = max(1, len(data) // 2)
            self.written_frames.append(data[:half])
            return half
        self.written_frames.append(data)
        return len(data)

    def read(self, size: int):
        """Mock read."""
        if self.block_read_event:
            self.block_read_event.wait()
        if self.raise_on_read:
            raise Exception("Mock read error")
        ret = self.read_buffer[:size]
        self.read_buffer = self.read_buffer[size:]
        if not ret:
            time.sleep(0.01)
        return ret

    def close(self):
        """Mock close."""
        self.is_open = False
        self.closed = True


def wait_until(condition, timeout=2.0):
    """Wait until condition is true or timeout."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if condition():
            return True
        time.sleep(0.01)
    return False


def test_env_json_schema():
    """Test ENV JSON schema is correct."""
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        registry.next_config.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)

        published_msgs = []

        class MockPub:
            def publish(self, msg):
                published_msgs.append(msg)

        node.env_pub = MockPub()
        node.handle_frame({
            'type': 'ENV',
            'fire_flags': 1,
            'gas_ppm': 120.5,
            'temp_c': 32.1,
            'batt_v': 11.8,
            'valid': 1
        })

        msg = published_msgs[0].data
        data = json.loads(msg)

        assert "fire" in data
        assert data["fire"] == "001"
        assert data["gas"] == 120.5
        assert data["temp"] == 32.1
        assert data["batt"] == 11.8

        node.handle_frame({
            'type': 'ENV',
            'valid': 0
        })

        msg2 = published_msgs[1].data
        data2 = json.loads(msg2)
        assert "status" in data2
        assert data2["status"] == "WROOM_OFFLINE"
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_odom_imu_quaternion():
    """Test ODOM and IMU sharing timestamp and quaternion."""
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        registry.next_config.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)

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

        node.handle_frame({
            'type': 'STATE',
            'seq': 1,
            'esp_ms': 1000,
            'x': 0.0,
            'y': 0.0,
            'yaw': 1.57,
            'vx': 0.0,
            'wz': 0.0,
            'gyro_z': 0.0
        })

        odom = pub_odom[0]
        imu = pub_imu[0]

        assert odom.header.stamp.sec == imu.header.stamp.sec
        assert odom.header.stamp.nanosec == imu.header.stamp.nanosec

        assert odom.header.frame_id == 'odom'
        assert odom.child_frame_id == 'base_link'
        assert imu.header.frame_id == 'imu_frame'

        assert odom.pose.pose.orientation.x == imu.orientation.x
        assert odom.pose.pose.orientation.y == imu.orientation.y
        assert odom.pose.pose.orientation.z == imu.orientation.z
        assert odom.pose.pose.orientation.w == imu.orientation.w

        norm = (odom.pose.pose.orientation.x**2 +
                odom.pose.pose.orientation.y**2 +
                odom.pose.pose.orientation.z**2 +
                odom.pose.pose.orientation.w**2)**0.5
        assert abs(norm - 1.0) < 1e-6
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_exact_bootstrap_and_offline_drop():
    """Test exact bootstrap order and offline drops."""
    rclpy.init()
    node = None
    try:
        registry.instances.clear()

        evt = threading.Event()
        registry.next_config = [{'block_write_event': evt}]

        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: len(registry.instances) >= 1)
        inst1 = registry.instances[0]

        assert not node.session_ready

        pump_msg = Bool()
        pump_msg.data = True
        node.pump_cmd_callback(pump_msg)
        assert node.cmd_drop_count == 1

        evt.set()
        assert wait_until(lambda: node.session_ready)

        assert len(inst1.written_frames) == 2
        assert b"CMD,2," in inst1.written_frames[0]
        assert b",0.000,0.000" in inst1.written_frames[0]
        assert b"PUMP,2," in inst1.written_frames[1]
        assert node.pump_state == 0
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_stale_epoch_no_replay():
    """Test race where callback observes ready but reconnect happens before enqueue."""
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        registry.next_config = [{}, {}]
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: node.session_ready)

        pause_evt = threading.Event()
        resume_evt = threading.Event()

        def mock_callback(msg):
            with node.state_lock:
                ready = node.session_ready
                epoch = node.session_epoch
            if not ready:
                return
            pause_evt.set()
            resume_evt.wait()
            with node.state_lock:
                node.pending_cmd = ((msg.linear.x, msg.angular.z), epoch)

        msg = Twist()
        msg.linear.x = 2.0
        t = threading.Thread(target=mock_callback, args=(msg,))
        t.start()

        assert wait_until(lambda: pause_evt.is_set())

        # force read error next loop to trigger reconnect
        registry.instances[0].raise_on_read = True

        assert wait_until(lambda: len(registry.instances) >= 2)
        inst2 = registry.instances[1]
        assert wait_until(lambda: node.session_ready and node.ser == inst2)

        resume_evt.set()
        t.join()

        assert wait_until(lambda: node.pending_cmd is None)

        cmd_frames = [f for f in inst2.written_frames if b"CMD" in f]
        for f in cmd_frames:
            assert b"2.0" not in f
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_blocked_cmd_latest_wins():
    """Test CMD latest wins drops backlog and sends zero CMD."""
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: node.session_ready)
        inst1 = registry.instances[0]

        evt = threading.Event()
        original_write = inst1.write

        def blocking_write(data):
            evt.wait()
            return original_write(data)

        inst1.write = blocking_write

        msg = Twist()
        msg.linear.x = 2.0
        for _ in range(10):
            node.cmd_vel_callback(msg)

        msg.linear.x = 0.0
        node.cmd_vel_callback(msg)

        evt.set()
        inst1.write = original_write

        assert wait_until(lambda: not node.pending_cmd)
        time.sleep(0.1)

        cmd_frames = [f for f in inst1.written_frames if b"CMD" in f]
        # Ignore bootstrap frame, check subsequent frames
        subsequent = cmd_frames[1:]
        for f in subsequent:
            assert b"2.0" not in f
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_pump_off_replaces_on():
    """Test PUMP OFF overrides blocked ON and cache reflects OFF."""
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: node.session_ready)
        inst1 = registry.instances[0]

        evt = threading.Event()
        original_write = inst1.write

        def blocking_write(data):
            evt.wait()
            return original_write(data)

        inst1.write = blocking_write

        pump_msg = Bool()
        pump_msg.data = True
        node.pump_cmd_callback(pump_msg)

        msg = Twist()
        msg.linear.x = 2.0
        for _ in range(10):
            node.cmd_vel_callback(msg)

        pump_msg.data = False
        node.pump_cmd_callback(pump_msg)

        evt.set()
        inst1.write = original_write

        assert wait_until(lambda: node.pump_state == 0 and not node.pending_pump)
        time.sleep(0.1)

        pump_frames = [f for f in inst1.written_frames if b"PUMP" in f]
        for f in pump_frames:
            assert b",0" in f
            assert b",1" not in f
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_partial_write_not_ready():
    """Test partial write during bootstrap keeps session_ready=False."""
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        registry.next_config = [
            {'partial_write': True},
            {}
        ]
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: len(registry.instances) >= 2)
        inst1 = registry.instances[0]
        assert inst1.closed
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_callback_not_blocking_when_io_blocked():
    """Test callbacks return immediately even if IO worker is stuck in open/write."""
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        evt = threading.Event()
        registry.next_config = [{'block_write_event': evt}]
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: len(registry.instances) >= 1)

        t0 = time.time()
        node.cmd_vel_callback(Twist())
        t1 = time.time()
        assert t1 - t0 < 0.1

        evt.set()
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_refresh_worker_alive():
    """Test that pump refresh works and doesn't kill worker."""
    rclpy.init()
    node = None
    try:
        registry.instances.clear()
        node = SerialBridgeNode(serial_cls=FakeSerial)
        assert wait_until(lambda: node.session_ready)
        inst1 = registry.instances[0]

        time.sleep(0.6)

        pump_frames = [f for f in inst1.written_frames if b"PUMP" in f]
        assert len(pump_frames) >= 2
        assert node.running
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
