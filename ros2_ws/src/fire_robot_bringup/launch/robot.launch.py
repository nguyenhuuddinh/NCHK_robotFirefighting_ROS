"""
robot.launch.py — File launch TỔNG HỢP chạy trên Raspberry Pi.

Chạy trên: 🟢 PI
Lệnh:     ros2 launch fire_robot_bringup robot.launch.py

Khởi chạy toàn bộ stack trên Pi bằng 1 lệnh duy nhất:
    1. fire_robot_description  → robot_state_publisher (TF tree: base_link → sensor frames)
    2. sensors.launch.py       → Camsense X1 Lidar (/scan) + USB Camera (/image_raw)
    3. micro_ros.launch.py     → micro-ROS Agent (ESP32-S3 ↔ ROS 2)
    4. safety.launch.py        → Safety Watchdog (heartbeat /cmd_vel)
    5. dashboard.launch.py     → Rosbridge WebSocket (Web Dashboard)

Arguments (theo ros2_engineer.md Section 7.1):
    - use_sim_time     : false (mặc định, Pi chạy phần cứng thực)
    - micro_ros_port   : /dev/ttyACM0
    - micro_ros_baudrate: 115200

🧪 Debug:
    ros2 node list → camsense_x1_node, usb_cam, micro_ros_agent,
                     safety_watchdog, rosbridge_websocket, robot_state_publisher
    CPU Pi < 70% (htop)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Đường dẫn đến các package ──
    bringup_dir = get_package_share_directory('fire_robot_bringup')
    description_dir = get_package_share_directory('fire_robot_description')
    safety_dir = get_package_share_directory('fire_robot_safety')

    # ── Launch Arguments (không hardcode, theo ros2_engineer.md 5.3) ──
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Dùng sim time (false cho phần cứng thực)'
    )

    micro_ros_port_arg = DeclareLaunchArgument(
        'micro_ros_port',
        default_value='/dev/ttyACM0',
        description='Serial port của ESP32-S3'
    )

    micro_ros_baudrate_arg = DeclareLaunchArgument(
        'micro_ros_baudrate',
        default_value='115200',
        description='Baudrate giao tiếp với ESP32-S3'
    )

    # ── 1. Robot Description (TF tree) ──
    # Pi cần TF tree để micro-ROS Agent có thể publish odom → base_link,
    # và static transforms base_link → laser_frame/camera_frame/imu_frame
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'description.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',  # Pi không có màn hình, không cần RViz
        }.items(),
    )

    # ── 2. Sensors (Lidar + Camera) ──
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'sensors.launch.py')
        ),
    )

    # ── 3. micro-ROS Agent (ESP32-S3 ↔ ROS 2) ──
    micro_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'micro_ros.launch.py')
        ),
        launch_arguments={
            'micro_ros_port': LaunchConfiguration('micro_ros_port'),
            'micro_ros_baudrate': LaunchConfiguration('micro_ros_baudrate'),
        }.items(),
    )

    # ── 4. Safety Watchdog (heartbeat /cmd_vel) ──
    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(safety_dir, 'launch', 'safety.launch.py')
        ),
    )

    # ── 5. Rosbridge WebSocket (Web Dashboard) ──
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'dashboard.launch.py')
        ),
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        micro_ros_port_arg,
        micro_ros_baudrate_arg,
        # Sub-launches
        description_launch,
        sensors_launch,
        micro_ros_launch,
        safety_launch,
        dashboard_launch,
    ])
