"""
odom_to_tf_broadcaster.py — Chuyển đổi /odom thành /tf (odom → base_link).

Chạy trên: 🟢 PI
Chức năng:
    - Subscribe /odom (BEST_EFFORT, từ ESP32 qua micro-ROS Agent)
    - Broadcast transform odom → base_link lên /tf (RELIABLE, trên DDS nội bộ Pi)

Thiết kế Timer-based + Stale Guard (Bug 13 fix):
    Timer 20Hz phát /tf đều đặn bằng clock Pi, giảm jitter cho SLAM.
    Callback /odom chỉ cache pose mới nhất + ghi timestamp nhận.
    Nếu /odom quá hạn (> odom_stale_timeout_ms), dừng phát TF và log warn.
    Khi /odom trở lại, tự động phát TF tiếp.

Timestamp dùng đồng hồ Pi (self.get_clock().now()):
    /scan cũng dùng đồng hồ Pi (từ camsense_x1_node trên Pi).
    Cùng nguồn clock → SLAM luôn khớp được timestamp → không bị drop message.

Parameters:
    tf_publish_rate_hz    : Tần số phát TF (default: 20.0 Hz)
    odom_stale_timeout_ms : Thời gian tối đa /odom được coi là tươi (default: 300 ms)
    odom_qos_depth        : Depth của QoS subscriber /odom (default: 10)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTfBroadcaster(Node):
    """Chuyển đổi /odom → /tf (odom → base_link) — Timer 20Hz + stale guard."""

    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')

        # --- Parameters (configurable via launch args hoặc yaml) ---
        self.declare_parameter('tf_publish_rate_hz', 20.0)
        self.declare_parameter('odom_stale_timeout_ms', 300)
        self.declare_parameter('odom_qos_depth', 10)

        tf_rate = self.get_parameter('tf_publish_rate_hz').value
        self._stale_timeout_ns = int(
            self.get_parameter('odom_stale_timeout_ms').value * 1e6
        )
        odom_depth = self.get_parameter('odom_qos_depth').value

        # --- Cached odom data (được cập nhật bởi callback) ---
        self._cached_position = None      # geometry_msgs/Point
        self._cached_orientation = None   # geometry_msgs/Quaternion
        self._last_odom_time_ns = 0       # Thời điểm nhận /odom (clock Pi, ns)
        self._odom_received = False       # Đã nhận ít nhất 1 /odom chưa

        # --- Stale warning throttle ---
        self._stale_warned = False

        # QoS BEST_EFFORT để nhận /odom từ ESP32 (cũng publish BEST_EFFORT)
        odom_qos = QoSProfile(
            depth=odom_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # Subscriber /odom — chỉ cache, không publish TF trực tiếp
        self._sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            odom_qos,
        )

        # TF Broadcaster (tự động publish RELIABLE lên /tf)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Timer phát TF đều đặn
        timer_period_s = 1.0 / tf_rate
        self._timer = self.create_timer(timer_period_s, self._timer_callback)

        # Đếm số TF đã phát (để log)
        self._tf_count = 0
        self._odom_count = 0

        self.get_logger().info(
            f'[OdomToTF] Timer {tf_rate:.0f}Hz + stale guard '
            f'{self._stale_timeout_ns / 1e6:.0f}ms — dùng clock Pi'
        )

    def _odom_callback(self, msg: Odometry):
        """Nhận /odom → cache pose + ghi timestamp nhận."""
        self._cached_position = msg.pose.pose.position
        self._cached_orientation = msg.pose.pose.orientation
        self._last_odom_time_ns = self.get_clock().now().nanoseconds
        self._odom_received = True

        # Reset stale warning khi nhận /odom mới
        if self._stale_warned:
            self.get_logger().info('[OdomToTF] /odom trở lại — tiếp tục phát /tf')
            self._stale_warned = False

        # Log mỗi 100 lần (~10 giây ở 10Hz) để biết node còn sống
        self._odom_count += 1
        if self._odom_count % 100 == 0:
            self.get_logger().info(
                f'[OdomToTF] Đã nhận {self._odom_count} /odom messages'
            )

    def _timer_callback(self):
        """Timer 20Hz — phát TF nếu /odom còn tươi."""
        # Chưa nhận /odom lần nào → im lặng
        if not self._odom_received:
            return

        # Kiểm tra stale: /odom quá hạn → dừng phát TF
        now_ns = self.get_clock().now().nanoseconds
        age_ns = now_ns - self._last_odom_time_ns

        if age_ns > self._stale_timeout_ns:
            # Chỉ warn 1 lần để không spam log
            if not self._stale_warned:
                age_ms = age_ns / 1e6
                self.get_logger().warn(
                    f'[OdomToTF] /odom stale ({age_ms:.0f}ms > '
                    f'{self._stale_timeout_ns / 1e6:.0f}ms) — dừng phát /tf'
                )
                self._stale_warned = True
            return

        # /odom còn tươi → broadcast TF
        t = TransformStamped()

        # Dùng đồng hồ Pi — cùng nguồn clock với /scan
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Copy cached position
        t.transform.translation.x = self._cached_position.x
        t.transform.translation.y = self._cached_position.y
        t.transform.translation.z = self._cached_position.z

        # Copy cached orientation (quaternion)
        t.transform.rotation = self._cached_orientation

        # Broadcast lên /tf (RELIABLE, trên DDS nội bộ)
        self._tf_broadcaster.sendTransform(t)

        # Log mỗi 200 lần (~10 giây ở 20Hz)
        self._tf_count += 1
        if self._tf_count % 200 == 0:
            self.get_logger().info(
                f'[OdomToTF] Đã phát {self._tf_count} /tf messages'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[OdomToTF] Tắt node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
