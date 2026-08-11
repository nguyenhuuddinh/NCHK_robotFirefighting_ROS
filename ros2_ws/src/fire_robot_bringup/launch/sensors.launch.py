"""
sensors.launch.py — Khởi chạy Lidar + Camera trên Raspberry Pi.

Chạy trên: 🟢 PI
Topic output:
  - /scan                   (sensor_msgs/LaserScan)        — từ Camsense X1
  - /image_raw              (sensor_msgs/Image)            — từ USB Camera
  - /image_raw/compressed   (sensor_msgs/CompressedImage)  — tự động bởi image_transport
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
    # Publish: /scan (sensor_msgs/LaserScan)
    camsense_x1_node = Node(
        package='camsense_x1',
        executable='camsense_x1_node',
        name='camsense_x1_node',
        parameters=[pi_params_file],
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
        # Tạm thời TẮT camera khi chạy SLAM để tránh nghẽn băng thông WiFi
        # usb_cam_node,
    ])
