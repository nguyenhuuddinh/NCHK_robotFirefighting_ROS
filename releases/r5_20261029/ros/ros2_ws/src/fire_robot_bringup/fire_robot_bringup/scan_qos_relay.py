"""
scan_qos_relay.py — Relay /scan_raw (Reliable) → /scan (Best Effort).

Chạy trên: 🟢 PI
Chức năng:
    Driver Camsense X1 publish /scan_raw với QoS mặc định (Reliable).
    Node này subscribe /scan_raw (Reliable) và republish nguyên message
    lên /scan (Best Effort) để laptop nhận qua WiFi không bị tích lũy
    retransmission delay.

    Không sửa timestamp, frame_id, ranges hoặc bất kỳ field nào.
    Không thêm queue tích lũy riêng — dùng KEEP_LAST nhỏ.

ROS 2 Parameters:
    input_topic  : Topic đầu vào (default: /scan_raw)
    output_topic : Topic đầu ra (default: /scan)
    input_depth  : QoS depth đầu vào (default: 5)
    output_depth : QoS depth đầu ra (default: 5)
"""

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanQosRelay(Node):
    """Relay LaserScan từ Reliable sang Best Effort."""

    def __init__(self):
        super().__init__('scan_qos_relay')

        # --- Parameters ---
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('input_depth', 5)
        self.declare_parameter('output_depth', 5)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        input_depth = self.get_parameter('input_depth').value
        output_depth = self.get_parameter('output_depth').value

        # QoS: subscribe Reliable (khớp driver Camsense)
        sub_qos = QoSProfile(
            depth=input_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # QoS: publish Best Effort (cho laptop qua WiFi)
        pub_qos = QoSProfile(
            depth=output_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._sub = self.create_subscription(
            LaserScan,
            input_topic,
            self._relay_callback,
            sub_qos,
        )

        self._pub = self.create_publisher(
            LaserScan,
            output_topic,
            pub_qos,
        )

        # Đếm thưa để log
        self._count = 0

        self.get_logger().info(
            f'[ScanQosRelay] {input_topic} (Reliable) → '
            f'{output_topic} (Best Effort)'
        )

    def _relay_callback(self, msg: LaserScan):
        """Forward nguyên message, không sửa bất kỳ field nào."""
        self._pub.publish(msg)

        self._count += 1
        if self._count % 100 == 0:
            self.get_logger().info(
                f'[ScanQosRelay] Đã relay {self._count} scans'
            )


def main(args=None):
    node = None
    executor = None
    try:
        rclpy.init(args=args)
        executor = SingleThreadedExecutor()
        node = ScanQosRelay()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.uninstall_signal_handlers()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
