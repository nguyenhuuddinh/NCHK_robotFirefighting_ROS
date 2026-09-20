"""Đưa JPEG từ usb_cam lên WiFi với QoS và ngân sách băng thông ảnh."""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


def camera_qos_profiles():
    """Giữ JPEG Reliable trong Pi, Best Effort depth 1 ra WiFi."""
    input_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    output_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    return input_qos, output_qos


class ByteRateLimiter:
    """Token bucket cho payload JPEG; frame thiếu ngân sách bị bỏ ngay."""

    def __init__(self, bytes_per_second):
        if bytes_per_second <= 0:
            raise ValueError('max_bytes_per_sec must be positive')
        self.rate = bytes_per_second
        self.tokens = float(bytes_per_second)
        self.last_time = None

    def allow(self, size_bytes, now):
        """Chỉ nhận frame khi còn đủ token, không tạo backlog."""
        if self.last_time is not None:
            elapsed = max(0.0, now - self.last_time)
            self.tokens = min(self.rate, self.tokens + elapsed * self.rate)
        self.last_time = now
        if size_bytes > self.tokens:
            return False
        self.tokens -= size_bytes
        return True


class CameraQosRelay(Node):
    """Relay ảnh JPEG nội bộ Pi sang Best Effort, depth 1."""

    def __init__(self):
        super().__init__('camera_qos_relay')
        self.declare_parameter('max_bytes_per_sec', 750000)
        self._budget = ByteRateLimiter(
            self.get_parameter('max_bytes_per_sec').value)
        self._dropped_frames = 0
        self._published_frames = 0

        input_qos, output_qos = camera_qos_profiles()

        self._pub = self.create_publisher(
            CompressedImage, '/image_raw/compressed', output_qos)
        self._sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed_local',
            self._relay, input_qos)
        self._stats_timer = self.create_timer(10.0, self._log_stats)

    def _relay(self, msg):
        """Giữ nguyên header/stamp và JPEG gốc; không lưu hàng đợi riêng."""
        if not self._budget.allow(len(msg.data), time.monotonic()):
            self._dropped_frames += 1
            return
        self._pub.publish(msg)
        self._published_frames += 1

    def _log_stats(self):
        """Thông báo frame bị bỏ để operator biết khi cần giảm FPS/độ phân giải."""
        if self._dropped_frames:
            self.get_logger().warn(
                f'[CameraQosRelay] 10s: sent={self._published_frames}, '
                f'dropped_by_budget={self._dropped_frames}'
            )
        self._dropped_frames = 0
        self._published_frames = 0


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraQosRelay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
