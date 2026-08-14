"""
sensors.launch.py — Khởi chạy Lidar + Camera trên Raspberry Pi.

Chạy trên: 🟢 PI
Topic output:
  - /scan                   (sensor_msgs/LaserScan)        — Best Effort qua scan_qos_relay
  - /scan_raw               (sensor_msgs/LaserScan)        — Reliable nội bộ Pi từ Camsense
  - /image_raw              (sensor_msgs/Image)            — từ USB Camera
  - /image_raw/compressed   (sensor_msgs/CompressedImage)  — tự động bởi image_transport

[QA5 FIX] Driver Camsense publish /scan với QoS mặc định (Reliable).
    Reliable qua WiFi gây tích lũy retransmission delay → SLAM drop scan.
    Fix: Remap Camsense output → /scan_raw, relay qua scan_qos_relay → /scan (Best Effort).
    Không sửa submodule Camsense.

Tham số: Đọc từ config/pi_params.yaml, KHÔNG hardcode.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Đường dẫn đến file tham số
    bringup_dir = get_package_share_directory('fire_robot_bringup')
    pi_params_file = os.path.join(bringup_dir, 'config', 'pi_params.yaml')

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

    # ── USB Camera Node ──
    # Package: usb_cam (cài bằng: sudo apt install ros-humble-usb-cam)
    # Publish: /image_raw (Image) + /image_raw/compressed (CompressedImage)
    # image_transport tự động tạo topic /compressed khi usb_cam chạy
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[pi_params_file],
        output='screen',
    )

    return LaunchDescription([
        camsense_x1_node,
        scan_qos_relay_node,
        # Tạm thời TẮT camera khi chạy SLAM để tránh nghẽn băng thông WiFi
        # usb_cam_node,
    ])
