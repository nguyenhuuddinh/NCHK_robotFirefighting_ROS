"""
safety_watchdog.py — Safety Command Gate trên Pi.

Chạy trên: 🟢 PI (Tầng 1 Fail-Safe)
Chức năng:
    - Subscribe /cmd_vel_raw (input từ teleop/Nav2/dashboard)
    - Publish  /cmd_vel     (output tới micro-ROS Agent → ESP32)
    - Khi input tươi: republish command cuối ở cmd_vel_output_rate_hz (10Hz)
    - Khi input quá hạn hoặc chưa có: publish Twist(0,0) STOP

[QA5 FIX] Khắc phục self-feedback:
    Phiên bản cũ subscribe VÀ publish cùng /cmd_vel → nhận lại STOP của chính mình
    → reset last_cmd_time → log "Laptop kết nối lại" giả mỗi ~1 giây.
    Fix: Input và output là hai topic khác nhau. Fail-fast nếu trùng.

Chuỗi an toàn:
    Nguồn lệnh mất kết nối
        → (1000ms) Pi gate gửi /cmd_vel = 0 → Xe dừng an toàn
        → (1000ms) Nếu Pi cũng mất → ESP32-S3 firmware EMERGENCY

QoS output: Best Effort để khớp ESP32 micro-ROS subscriber.
Timeout: Dùng monotonic/steady time (time.monotonic()), không bị ảnh hưởng NTP.

ROS 2 Parameters:
    cmd_vel_input_topic     : Topic đầu vào (default: /cmd_vel_raw)
    cmd_vel_output_topic    : Topic đầu ra (default: /cmd_vel)
    heartbeat_timeout_ms    : Timeout ms (default: 1000)
    cmd_vel_output_rate_hz  : Tần suất publish output (default: 10.0)
"""

import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


class SafetyWatchdog(Node):
    """Safety Command Gate: /cmd_vel_raw → /cmd_vel với timeout STOP."""

    def __init__(self):
        super().__init__('safety_watchdog')

        # --- Parameters ---
        self.declare_parameter('cmd_vel_input_topic', '/cmd_vel_raw')
        self.declare_parameter('cmd_vel_output_topic', '/cmd_vel')
        self.declare_parameter('heartbeat_timeout_ms', 1000)
        self.declare_parameter('cmd_vel_output_rate_hz', 10.0)

        input_topic = self.get_parameter('cmd_vel_input_topic').value
        output_topic = self.get_parameter('cmd_vel_output_topic').value
        self._timeout_s = (
            self.get_parameter('heartbeat_timeout_ms').value / 1000.0
        )
        output_rate = self.get_parameter('cmd_vel_output_rate_hz').value

        # Fail-fast: input và output KHÔNG được trùng nhau
        if input_topic == output_topic:
            self.get_logger().fatal(
                f'[SafetyGate] FATAL: input_topic == output_topic == "{input_topic}". '
                f'Đây gây self-feedback! Sửa launch arguments.'
            )
            raise SystemExit(1)

        # QoS input: Best Effort để nhận từ teleop/Nav2 qua WiFi
        input_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # QoS output: Best Effort để khớp ESP32 micro-ROS subscriber
        output_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # --- Subscriber: input command ---
        self._sub = self.create_subscription(
            Twist,
            input_topic,
            self._input_callback,
            input_qos,
        )

        # --- Publisher: output command ---
        self._pub = self.create_publisher(
            Twist,
            output_topic,
            output_qos,
        )

        # --- State ---
        self._last_input_time = 0.0  # monotonic seconds, 0 = chưa nhận
        self._last_cmd = Twist()     # command cuối cùng nhận được
        self._ever_received = False
        self._timeout_warned = False  # chỉ log warning 1 lần

        # --- Timer: publish output ở tần suất đều ---
        timer_period_s = 1.0 / output_rate
        self._timer = self.create_timer(timer_period_s, self._output_timer)

        self.get_logger().info(
            f'[SafetyGate] {input_topic} → {output_topic} | '
            f'timeout={self._timeout_s * 1000:.0f}ms, '
            f'rate={output_rate:.0f}Hz'
        )

    def _input_callback(self, msg: Twist):
        """Nhận command mới từ input topic."""
        self._last_input_time = time.monotonic()
        self._last_cmd = msg

        if not self._ever_received:
            self._ever_received = True
            self.get_logger().info(
                '[SafetyGate] Nhận command đầu tiên — nguồn lệnh đã kết nối.'
            )

        if self._timeout_warned:
            self._timeout_warned = False
            self.get_logger().info(
                '[SafetyGate] Nguồn lệnh trở lại — tiếp tục forward command.'
            )

    def _output_timer(self):
        """Timer callback: publish command hoặc STOP."""
        now = time.monotonic()

        if not self._ever_received:
            # Chưa nhận command nào → publish STOP im lặng
            self._pub.publish(Twist())
            return

        elapsed = now - self._last_input_time

        if elapsed <= self._timeout_s:
            # Input còn tươi → forward command cuối
            self._pub.publish(self._last_cmd)
        else:
            # Input quá hạn → STOP
            self._pub.publish(Twist())

            if not self._timeout_warned:
                self._timeout_warned = True
                self.get_logger().warn(
                    f'[SafetyGate] Heartbeat lost! '
                    f'Không nhận command trong {elapsed * 1000:.0f}ms '
                    f'(>{self._timeout_s * 1000:.0f}ms). Đang gửi STOP.'
                )


def main(args=None):
    node = None
    executor = None

    shutdown_requested = [False]

    def graceful_signal_handler(signum, frame):
        if not shutdown_requested[0]:
            shutdown_requested[0] = True
            raise KeyboardInterrupt()

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    signal.signal(signal.SIGTERM, graceful_signal_handler)
    signal.signal(signal.SIGINT, graceful_signal_handler)

    primary_exc = None
    cleanup_errors = []

    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        executor = SingleThreadedExecutor()
        node = SafetyWatchdog()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        shutdown_requested[0] = True
    except BaseException as e:
        primary_exc = e
    finally:
        shutdown_requested[0] = True

        if node is not None:
            try:
                if hasattr(node, '_timer') and node._timer is not None:
                    if not node._timer.is_canceled():
                        node._timer.cancel()
            except BaseException as e:
                cleanup_errors.append(e)

        if executor is not None:
            try:
                executor.shutdown()
            except BaseException as e:
                cleanup_errors.append(e)

        if node is not None:
            try:
                node.destroy_node()
            except BaseException as e:
                cleanup_errors.append(e)

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException as e:
            cleanup_errors.append(e)

        signal.signal(signal.SIGTERM, original_sigterm)
        signal.signal(signal.SIGINT, original_sigint)

        if primary_exc is not None:
            raise primary_exc
        if cleanup_errors:
            raise cleanup_errors[0]


if __name__ == '__main__':
    main()
