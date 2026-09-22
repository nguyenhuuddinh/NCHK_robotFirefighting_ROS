"""
robot.launch.py — File launch TỔNG HỢP chạy trên Raspberry Pi.

Chạy trên: 🟢 PI
Lệnh:     ros2 launch fire_robot_bringup robot.launch.py

Khởi chạy toàn bộ stack trên Pi bằng 1 lệnh duy nhất:
    1. fire_robot_description  → robot_state_publisher (TF tree: base_link → sensor frames)
    2. sensors.launch.py       → Camsense X1 Lidar + USB camera MJPEG
    3. serial_bridge_node      → Cầu nối Raw Serial V2 (ESP32-S3 ↔ ROS 2)
    4. safety.launch.py        → Safety Command Gate (/cmd_vel_raw → /cmd_vel)
    5. dashboard.launch.py     → Rosbridge WebSocket (Web Dashboard)
    6. odom_to_tf_broadcaster  → /odom → /tf (odom → base_link)

Arguments (theo ros2_engineer.md Section 7.1):
    - use_sim_time           : false (mặc định, Pi chạy phần cứng thực)
    - use_camera             : true (có thể tắt khi WiFi nghẽn)
    - camera_backend         : native_mjpeg (usb_cam là fallback)
    - camera_fps             : 15 (5/10/15/20/25/30)
    - camera_max_bytes_per_sec: 750000 (drop frame thay vì backlog)
    - serial_port            : /dev/serial/by-id/usb-Espressif_...-if00
    - serial_baudrate        : 115200
    - cmd_vel_input_topic    : /cmd_vel_raw
    - cmd_vel_output_topic   : /cmd_vel
    - heartbeat_timeout_ms   : 1000
    - cmd_vel_output_rate_hz : 10.0

🧪 Debug:
    ros2 node list → camsense_x1_node, scan_qos_relay, serial_bridge_node,
                     safety_watchdog, rosbridge_websocket, robot_state_publisher,
                     odom_to_tf_broadcaster
    CPU Pi < 70% (htop)

🎮 Teleop (phải remap sang /cmd_vel_raw):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_raw
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as LaunchNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Đường dẫn đến các package ──
    bringup_dir = get_package_share_directory('fire_robot_bringup')
    description_dir = get_package_share_directory('fire_robot_description')
    safety_dir = get_package_share_directory('fire_robot_safety')
    pi_params_file = os.path.join(bringup_dir, 'config', 'pi_params.yaml')

    # ── Launch Arguments (không hardcode, theo ros2_engineer.md 5.3) ──
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Dùng sim time (false cho phần cứng thực)'
    )

    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Bật USB camera; use_camera:=false nếu ảnh gây nghẽn WiFi'
    )

    camera_backend_arg = DeclareLaunchArgument(
        'camera_backend',
        default_value='native_mjpeg',
        choices=['native_mjpeg', 'usb_cam'],
        description='Backend camera: native_mjpeg hoặc usb_cam fallback'
    )

    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='15',
        choices=['5', '10', '15', '20', '25', '30'],
        description='FPS camera; 15 đã đạt gate LAN và WiFi AP'
    )

    camera_budget_arg = DeclareLaunchArgument(
        'camera_max_bytes_per_sec',
        default_value='750000',
        description='Giới hạn byte/s cho JPEG qua mạng'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=(
            '/dev/serial/by-id/'
            'usb-Espressif_Systems_Freenove_ESP32-S3_WROOM_N16R8_'
            '_16MB_Flash___8MB_PSRAM__9C139EAAE110-if00'
        ),
        description='Stable USB serial path của ESP32-S3'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Baudrate giao tiếp với ESP32-S3'
    )

    cmd_vel_input_arg = DeclareLaunchArgument(
        'cmd_vel_input_topic',
        default_value='/cmd_vel_raw',
        description='Topic đầu vào command velocity'
    )

    cmd_vel_output_arg = DeclareLaunchArgument(
        'cmd_vel_output_topic',
        default_value='/cmd_vel',
        description='Topic đầu ra command velocity (tới ESP32)'
    )

    heartbeat_timeout_arg = DeclareLaunchArgument(
        'heartbeat_timeout_ms',
        default_value='1000',
        description='Timeout ms cho safety command gate'
    )

    cmd_vel_rate_arg = DeclareLaunchArgument(
        'cmd_vel_output_rate_hz',
        default_value='10.0',
        description='Tần suất publish /cmd_vel (Hz)'
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
        launch_arguments={
            'use_camera': LaunchConfiguration('use_camera'),
            'camera_backend': LaunchConfiguration('camera_backend'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            'camera_max_bytes_per_sec': LaunchConfiguration(
                'camera_max_bytes_per_sec'),
        }.items(),
    )

    # ── 3. Serial Bridge Node (ESP32-S3 ↔ ROS 2 via Raw Serial V2) ──
    serial_bridge = LaunchNode(
        package='fire_robot_bringup',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    # ── 4. Safety Command Gate (/cmd_vel_raw → /cmd_vel) ──
    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(safety_dir, 'launch', 'safety.launch.py')
        ),
        launch_arguments={
            'cmd_vel_input_topic': LaunchConfiguration('cmd_vel_input_topic'),
            'cmd_vel_output_topic': LaunchConfiguration('cmd_vel_output_topic'),
            'heartbeat_timeout_ms': LaunchConfiguration('heartbeat_timeout_ms'),
            'cmd_vel_output_rate_hz': LaunchConfiguration('cmd_vel_output_rate_hz'),
        }.items(),
    )

    # ── 5. Rosbridge WebSocket (Web Dashboard) ──
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'dashboard.launch.py')
        ),
    )

    # ── 6. Odom-to-TF Broadcaster ──
    # Chuyển /odom (BEST_EFFORT từ ESP32) → /tf (RELIABLE trên DDS nội bộ)
    odom_to_tf_node = LaunchNode(
        package='fire_robot_bringup',
        executable='odom_to_tf_broadcaster',
        name='odom_to_tf_broadcaster',
        output='screen',
        parameters=[
            pi_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        use_camera_arg,
        camera_backend_arg,
        camera_fps_arg,
        camera_budget_arg,
        serial_port_arg,
        serial_baudrate_arg,
        cmd_vel_input_arg,
        cmd_vel_output_arg,
        heartbeat_timeout_arg,
        cmd_vel_rate_arg,
        # Sub-launches & Nodes
        description_launch,
        sensors_launch,
        serial_bridge,
        safety_launch,
        dashboard_launch,
        # Nodes
        odom_to_tf_node,
    ])
