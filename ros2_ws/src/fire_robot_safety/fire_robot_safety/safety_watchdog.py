"""
safety_watchdog.py — Node giám sát heartbeat /cmd_vel trên Pi.

Chạy trên: 🟢 PI (Tầng 1 Fail-Safe)
Chức năng:
    - Subscribe /cmd_vel → ghi nhận timestamp lần nhận cuối.
    - Timer 100ms: nếu (now - last_cmd) > heartbeat_timeout → publish Twist(0,0).
    - Log warning khi kích hoạt watchdog, log info khi Laptop kết nối lại.

Chuỗi an toàn:
    Laptop mất WiFi
        → (500ms) Pi watchdog gửi /cmd_vel = 0 → Xe dừng an toàn
        → (1000ms) Nếu Pi cũng mất → ESP32-S3 firmware EMERGENCY

QoS: /cmd_vel là topic điều khiển → Reliable (theo Tai_Lieu_So_4).
Params: heartbeat_timeout_ms, check_period_ms (từ yaml / launch argument, KHÔNG hardcode).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist


class SafetyWatchdog(Node):
    """Giám sát heartbeat /cmd_vel. Gửi STOP nếu Laptop mất kết nối."""

    def __init__(self):
        super().__init__('safety_watchdog')

        # ── Parameters (KHÔNG hardcode) ──
        self.declare_parameter('heartbeat_timeout_ms', 500)
        self.declare_parameter('check_period_ms', 100)

        self._timeout_ms = (
            self.get_parameter('heartbeat_timeout_ms').get_parameter_value().integer_value
        )
        self._check_period_ms = (
            self.get_parameter('check_period_ms').get_parameter_value().integer_value
        )

        # ── QoS: /cmd_vel là control topic → Reliable ──
        cmd_vel_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── Subscriber: lắng nghe /cmd_vel từ Laptop ──
        self._sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            cmd_vel_qos,
        )

        # ── Publisher: gửi Twist(0,0) khi mất heartbeat ──
        self._pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            cmd_vel_qos,
        )

        # ── Trạng thái ──
        self._last_cmd_time = self.get_clock().now()
        self._watchdog_active = False  # Đang ở trạng thái cảnh báo?
        self._ever_received = False    # Đã từng nhận /cmd_vel chưa?

        # ── Timer kiểm tra heartbeat ──
        timer_period_sec = self._check_period_ms / 1000.0
        self._timer = self.create_timer(timer_period_sec, self._check_heartbeat)

        self.get_logger().info(
            f'[SafetyWatchdog] Khởi động — '
            f'timeout={self._timeout_ms}ms, check_period={self._check_period_ms}ms'
        )

    def _cmd_vel_callback(self, msg: Twist):
        """Ghi nhận timestamp mỗi khi nhận /cmd_vel từ Laptop."""
        self._last_cmd_time = self.get_clock().now()

        if not self._ever_received:
            self._ever_received = True
            self.get_logger().info('[SafetyWatchdog] Nhận /cmd_vel đầu tiên — Laptop đã kết nối.')

        if self._watchdog_active:
            self._watchdog_active = False
            self.get_logger().info('[SafetyWatchdog] Laptop kết nối lại — watchdog tắt cảnh báo.')

    def _check_heartbeat(self):
        """Timer callback: kiểm tra xem /cmd_vel có bị mất quá lâu không."""
        # Chưa từng nhận cmd_vel → không cần watchdog (xe chưa bắt đầu chạy)
        if not self._ever_received:
            return

        elapsed_ms = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e6

        if elapsed_ms > self._timeout_ms:
            if not self._watchdog_active:
                self._watchdog_active = True
                self.get_logger().warn(
                    f'[SafetyWatchdog] ⚠️ Heartbeat lost! '
                    f'Không nhận /cmd_vel trong {elapsed_ms:.0f}ms (>{self._timeout_ms}ms). '
                    f'Gửi STOP.'
                )

            # Gửi Twist(0,0) — dừng xe
            stop_msg = Twist()  # Mặc định linear=0, angular=0
            self._pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdog()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[SafetyWatchdog] Tắt node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
