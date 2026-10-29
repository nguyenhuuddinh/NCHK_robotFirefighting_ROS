"""
Khởi chạy micro-ROS Agent trên Raspberry Pi.

[LEGACY/MANUAL] — File này hiện là legacy và chỉ chạy manual khi cần test.
Runtime chính đã chuyển sang serial_bridge_node (Raw Serial V2).

Chạy trên: 🟢 PI
Chức năng: Cầu nối USB Serial (ESP32-S3) ↔ ROS 2 DDS

    ESP32-S3 publish qua micro-ROS:
        - /odom       (nav_msgs/Odometry)      — 10 Hz, Best Effort
        - /imu/data   (sensor_msgs/Imu)        — 10 Hz, Best Effort
        - /env_status (std_msgs/String, JSON)   — 2 Hz, Best Effort
    ESP32-S3 subscribe qua micro-ROS:
        - /cmd_vel    (geometry_msgs/Twist)     — Best Effort QoS
        - /fire_target(geometry_msgs/Point)     — Reliable QoS
        - /pump_cmd   (std_msgs/Bool)           — Reliable QoS

Tham số: Truyền qua Launch Arguments (không hardcode).
Cài đặt: sudo apt install ros-humble-micro-ros-agent
         hoặc build từ source (https://github.com/micro-ROS/micro-ROS-Agent)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ── Launch Arguments (không hardcode port/baudrate) ──
    micro_ros_port_arg = DeclareLaunchArgument(
        'micro_ros_port',
        default_value='/dev/ttyACM0',
        description='Serial port của ESP32-S3 (USB-OTG → Pi)'
    )

    micro_ros_baudrate_arg = DeclareLaunchArgument(
        'micro_ros_baudrate',
        default_value='115200',
        description='Baudrate giao tiếp với ESP32-S3'
    )

    # ── micro-ROS Agent ──
    # Agent nhận tham số qua CLI, không qua yaml.
    # [BUG FIX] Dùng Node với respawn=True thay vì ExecuteProcess.
    # Lý do: Nếu cáp USB bị nhiễu (EMI từ motor) hoặc sụt áp nhẹ làm ngắt kết nối tạm thời,
    # micro_ros_agent sẽ crash. Với respawn=True, nó sẽ tự động khởi động lại sau 2 giây
    # giúp hệ thống tự phục hồi mà không cần bạn phải chạy lại lệnh launch bằng tay!
    from launch_ros.actions import Node as LaunchNode
    micro_ros_agent = LaunchNode(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=[
            'serial',
            '--dev', LaunchConfiguration('micro_ros_port'),
            '-b', LaunchConfiguration('micro_ros_baudrate'),
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        micro_ros_port_arg,
        micro_ros_baudrate_arg,
        micro_ros_agent,
    ])
