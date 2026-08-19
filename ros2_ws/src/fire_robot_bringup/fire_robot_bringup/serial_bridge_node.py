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
        self.session_ready = False
        self.session_epoch = 0

        self.pending_cmd = None
        self.pending_pump = None
        self.pending_fire = None

        self.pump_state = 0
        self.last_state_time_mono = 0.0

        self.tx_fail_count = 0
        self.tx_partial_count = 0
        self.reconnect_count = 0
        self.cmd_drop_count = 0

        self.create_timer(10.0, self.log_timer_callback)

        self.running = True
        self.worker_thread = threading.Thread(target=self.serial_worker, daemon=True)
        self.worker_thread.start()

    def cmd_vel_callback(self, msg):
        """Handle /cmd_vel."""
        with self.state_lock:
            if not self.session_ready:
                self.cmd_drop_count += 1
                return
            self.pending_cmd = ((msg.linear.x, msg.angular.z), self.session_epoch)

    def fire_target_callback(self, msg):
        """Handle /fire_target."""
        with self.state_lock:
            if not self.session_ready:
                self.cmd_drop_count += 1
                return
            self.pending_fire = ((msg.x, msg.y), self.session_epoch)

    def pump_cmd_callback(self, msg):
        """Handle /pump_cmd."""
        pump_req = 1 if msg.data else 0
        with self.state_lock:
            if not self.session_ready:
                self.cmd_drop_count += 1
                return
            self.pending_pump = (pump_req, self.session_epoch)

    def close_serial(self):
        """Close serial port and advance epoch."""
        with self.state_lock:
            self.session_ready = False
            self.session_epoch += 1
            self.pending_cmd = None
            self.pending_pump = None
            self.pending_fire = None

        if self.ser:
            try:
                self.ser.close()
            except Exception as e:
                self.get_logger().debug(f"Serial close error: {e}")
            finally:
                self.ser = None

    def _write_frame(self, frame: bytes) -> bool:
        """Write frame directly, handle errors. Return True if success."""
        if not frame or not self.ser:
            return False
        try:
            written = self.ser.write(frame)
            if written < len(frame):
                self.tx_partial_count += 1
                raise Exception("Partial write")
            return True
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")
            self.tx_fail_count += 1
            self.close_serial()
            return False

    def serial_worker(self):
        """Single I/O worker for reading and writing."""
        last_pump_time = time.monotonic()
        try:
            while self.running:
                with self.state_lock:
                    is_closed = (self.ser is None or getattr(self.ser, 'closed', True))
                if is_closed:
                    self.close_serial()
                    try:
                        self.ser = self.serial_cls(
                            self.port, self.baudrate, timeout=0.01, write_timeout=0.1)
                        self.get_logger().info(f"Opened serial port {self.port}")
                        self.reconnect_count += 1
                        self.protocol.reset_parser()

                        cmd = self.protocol.generate_cmd(0.0, 0.0)
                        pump = self.protocol.generate_pump(0)

                        if cmd is None or pump is None:
                            raise Exception("Failed to generate bootstrap commands")

                        w1 = self.ser.write(cmd)
                        w2 = self.ser.write(pump)
                        if w1 == len(cmd) and w2 == len(pump):
                            self.pump_state = 0
                            with self.state_lock:
                                self.session_ready = True
                            last_pump_time = time.monotonic()
                        else:
                            self.tx_partial_count += 1
                            raise Exception("Partial write during bootstrap")
                    except Exception as e:
                        self.get_logger().debug(f"Failed to bootstrap port {self.port}: {e}")
                        self.close_serial()
                        time.sleep(1.0)
                        continue

                # Process pending commands
                to_write = []
                with self.state_lock:
                    ready = self.session_ready
                    current_epoch = self.session_epoch

                    if self.pending_cmd:
                        args, item_epoch = self.pending_cmd
                        self.pending_cmd = None
                        if ready and item_epoch == current_epoch:
                            to_write.append(('cmd', args))
                        else:
                            self.cmd_drop_count += 1

                    if self.pending_pump:
                        pump_req, item_epoch = self.pending_pump
                        self.pending_pump = None
                        if ready and item_epoch == current_epoch:
                            to_write.append(('pump', pump_req))
                        else:
                            self.cmd_drop_count += 1

                    if self.pending_fire:
                        args, item_epoch = self.pending_fire
                        self.pending_fire = None
                        if ready and item_epoch == current_epoch:
                            to_write.append(('fire', args))
                        else:
                            self.cmd_drop_count += 1

                for item_type, data in to_write:
                    frame = None
                    if item_type == 'cmd':
                        frame = self.protocol.generate_cmd(data[0], data[1])
                    elif item_type == 'pump':
                        self.pump_state = data
                        frame = self.protocol.generate_pump(data)
                    elif item_type == 'fire':
                        frame = self.protocol.generate_fire(data[0], data[1])

                    if frame:
                        if not self._write_frame(frame):
                            break

                # Process pump refresh (2Hz)
                with self.state_lock:
                    ready = self.session_ready
                if ready:
                    now_mono = time.monotonic()
                    if now_mono - last_pump_time >= 0.5:
                        last_pump_time = now_mono
                        frame = self.protocol.generate_pump(self.pump_state)
                        if frame:
                            self._write_frame(frame)

                # Process RX
                try:
                    chunk = self.ser.read(1024)
                    if chunk:
                        for frame in self.protocol.parse_chunk(chunk):
                            self.handle_frame(frame)
                except Exception as e:
                    self.get_logger().warn(f"Serial read error: {e}")
                    self.close_serial()
        finally:
            with self.state_lock:
                self.session_ready = False
            self.close_serial()

    def handle_frame(self, frame):
        """Handle parsed telemetry frame."""
        if frame['type'] == 'STATE':
            self.last_state_time_mono = time.monotonic()
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

            self.odom_pub.publish(odom)

            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'imu_frame'
            imu.orientation.x = 0.0
            imu.orientation.y = 0.0
            imu.orientation.z = sy
            imu.orientation.w = cy
            imu.angular_velocity.z = frame['gyro_z']
            self.imu_pub.publish(imu)

        elif frame['type'] == 'ENV':
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
            self.env_pub.publish(msg)

    def log_timer_callback(self):
        """Log diagnostic statistics."""
        age_ms = -1
        if self.last_state_time_mono > 0:
            age_ms = int((time.monotonic() - self.last_state_time_mono) * 1000)

        self.get_logger().info(
            f"RX valid={self.protocol.valid_count} crc_fail={self.protocol.crc_fail_count} "
            f"parse_fail={self.protocol.parse_fail_count} overflow={self.protocol.overflow_count} "
            f"gap={self.protocol.gap_count} dup={self.protocol.dup_count} "
            f"reconnect={self.reconnect_count} "
            f"tx_fail={self.tx_fail_count} tx_partial={self.tx_partial_count} "
            f"cmd_drop={self.cmd_drop_count} state_age_ms={age_ms}"
        )

    def destroy_node(self):
        """Run main entry point."""
        self.running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=2.0)
        self.close_serial()
        super().destroy_node()


def main(args=None):
    """Run main entry point."""
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
