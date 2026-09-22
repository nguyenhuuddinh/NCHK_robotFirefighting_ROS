"""
sensors.launch.py — Khởi chạy Lidar + Camera trên Raspberry Pi.

Chạy trên: 🟢 PI
Topic output:
  - /scan                   (sensor_msgs/LaserScan)        — Best Effort qua scan_qos_relay
  - /scan_raw               (sensor_msgs/LaserScan)        — Reliable nội bộ Pi từ Camsense
  - /image_raw/compressed       (sensor_msgs/CompressedImage) — MJPEG Best Effort qua WiFi

[QA5 FIX] Driver Camsense publish /scan với QoS mặc định (Reliable).
    Reliable qua WiFi gây tích lũy retransmission delay → SLAM drop scan.
    Fix: Remap Camsense output → /scan_raw, relay qua scan_qos_relay → /scan (Best Effort).
    Không sửa submodule Camsense.

Tham số: Đọc từ config/pi_params.yaml, KHÔNG hardcode.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Đường dẫn đến file tham số
    bringup_dir = get_package_share_directory('fire_robot_bringup')
    pi_params_file = os.path.join(bringup_dir, 'config', 'pi_params.yaml')

    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Bật USB camera; use_camera:=false để ngắt stream nếu WiFi nghẽn',
    )

    camera_backend_arg = DeclareLaunchArgument(
        'camera_backend',
        default_value='native_mjpeg',
        choices=['native_mjpeg', 'usb_cam'],
        description=(
            'native_mjpeg: giữ JPEG từ webcam; usb_cam: fallback YUYV rồi nén'
        ),
    )

    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='15',
        choices=['5', '10', '15', '20', '25', '30'],
        description='FPS camera; 15 đã đạt gate LAN và WiFi AP',
    )

    camera_budget_arg = DeclareLaunchArgument(
        'camera_max_bytes_per_sec',
        default_value='750000',
        description='Giới hạn payload JPEG mỗi giây; frame dư bị bỏ ngay',
    )

    native_camera_enabled = IfCondition(PythonExpression([
        "'", LaunchConfiguration('use_camera'),
        "'.lower() in ('true', '1', 'yes', 'on') and '",
        LaunchConfiguration('camera_backend'), "' == 'native_mjpeg'",
    ]))
    usb_camera_enabled = IfCondition(PythonExpression([
        "'", LaunchConfiguration('use_camera'),
        "'.lower() in ('true', '1', 'yes', 'on') and '",
        LaunchConfiguration('camera_backend'), "' == 'usb_cam'",
    ]))

    # ── Camsense X1 Lidar Node ──
    # Package: camsense_x1 (clone từ GitHub)
    # Output remap: /scan → /scan_raw (Reliable nội bộ Pi)
    camsense_x1_node = Node(
        package='camsense_x1',
        executable='camsense_x1_node',
        name='camsense_x1_node',
        parameters=[pi_params_file],
        remappings=[('/scan', '/scan_raw')],
        output='screen',
    )

    # ── Scan QoS Relay ──
    # Subscribe /scan_raw (Reliable) → Publish /scan (Best Effort)
    # Reliable chỉ tồn tại nội bộ Pi, không truyền sensor backlog qua WiFi.
    scan_qos_relay_node = Node(
        package='fire_robot_bringup',
        executable='scan_qos_relay',
        name='scan_qos_relay',
        output='screen',
    )

    # ── Native MJPEG Camera ──
    # Logitech C210 xuất MJPEG trực tiếp. Node chỉ tách từng JPEG và publish;
    # không decode RGB rồi encode JPEG lại trên Pi.
    native_mjpeg_camera_node = Node(
        package='fire_robot_bringup',
        executable='native_mjpeg_camera',
        name='native_mjpeg_camera',
        parameters=[
            pi_params_file,
            {
                'framerate': ParameterValue(
                    LaunchConfiguration('camera_fps'), value_type=int),
                'max_bytes_per_sec': ParameterValue(
                    LaunchConfiguration('camera_max_bytes_per_sec'),
                    value_type=int,
                ),
            },
        ],
        output='screen',
        condition=native_camera_enabled,
    )

    # ── usb_cam fallback ──
    # Package: usb_cam (cài bằng: sudo apt install ros-humble-usb-cam)
    # usb_cam 0.8.1 không hỗ trợ pixel_format=mjpeg; yuyv2rgb đã thử trên Pi.
    # Raw chỉ dùng nội bộ; image_transport nén JPEG trước khi relay qua WiFi.
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[
            pi_params_file,
            {
                'framerate': ParameterValue(
                    LaunchConfiguration('camera_fps'), value_type=float),
            },
        ],
        remappings=[
            ('image_raw', '/camera_raw_local'),
            ('image_raw/compressed', '/image_raw/compressed_local'),
        ],
        output='screen',
        condition=usb_camera_enabled,
    )

    camera_qos_relay_node = Node(
        package='fire_robot_bringup',
        executable='camera_qos_relay',
        name='camera_qos_relay',
        parameters=[
            pi_params_file,
            {
                'max_bytes_per_sec': ParameterValue(
                    LaunchConfiguration('camera_max_bytes_per_sec'),
                    value_type=int,
                ),
            },
        ],
        output='screen',
        condition=usb_camera_enabled,
    )

    return LaunchDescription([
        use_camera_arg,
        camera_backend_arg,
        camera_fps_arg,
        camera_budget_arg,
        camsense_x1_node,
        scan_qos_relay_node,
        native_mjpeg_camera_node,
        usb_cam_node,
        camera_qos_relay_node,
    ])
