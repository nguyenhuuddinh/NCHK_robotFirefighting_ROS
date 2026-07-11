"""
micro_ros.launch.py — Khởi chạy micro-ROS Agent trên Raspberry Pi.

Chạy trên: 🟢 PI
Chức năng: Cầu nối USB Serial (ESP32-S3) ↔ ROS 2 DDS
    ESP32-S3 publish qua micro-ROS:
        - /odom       (nav_msgs/Odometry)      — 50 Hz
        - /imu/data   (sensor_msgs/Imu)        — 50 Hz
        - /env_status (std_msgs/String, JSON)   — 1 Hz
    ESP32-S3 subscribe qua micro-ROS:
        - /cmd_vel    (geometry_msgs/Twist)     — Reliable QoS
        - /fire_target(geometry_msgs/Point)     — Reliable QoS
        - /pump_cmd   (std_msgs/Bool)           — Reliable QoS

Tham số: Truyền qua Launch Arguments (không hardcode).
Cài đặt: sudo apt install ros-humble-micro-ros-agent
         hoặc build từ source (https://github.com/micro-ROS/micro-ROS-Agent)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    # Dùng ExecuteProcess vì micro_ros_agent có giao diện CLI riêng:
    #   ros2 run micro_ros_agent micro_ros_agent serial --dev <port> -b <baud>
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial',
            '--dev', LaunchConfiguration('micro_ros_port'),
            '-b', LaunchConfiguration('micro_ros_baudrate'),
        ],
        output='screen',
    )

    return LaunchDescription([
        micro_ros_port_arg,
        micro_ros_baudrate_arg,
        micro_ros_agent,
    ])
