"""
slam.launch.py — Khởi chạy SLAM Toolbox trên Laptop.

Chạy trên: 💻 LAPTOP
Lệnh:
    ros2 launch fire_robot_navigation slam.launch.py
    ros2 launch fire_robot_navigation slam.launch.py rviz:=true

Arguments:
    use_sim_time    : false (mặc định, chạy phần cứng thực)
    slam_params_file: đường dẫn config SLAM (mặc định mapper_params_online_async.yaml)
    rviz            : false (mặc định). Đặt true để mở RViz với config SLAM tích hợp.

[QA6 FIX] Thêm launch argument 'rviz' (mặc định false) để mở RViz với config
    slam.rviz đã đặt LaserScan Best Effort. Không thay đổi quy trình khởi động
    hiện tại khi không truyền argument.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Thư mục gốc của package
    navigation_dir = get_package_share_directory('fire_robot_navigation')

    # Khai báo argument cho file config của SLAM
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Sử dụng thời gian mô phỏng nếu chạy Gazebo, bằng false nếu chạy thực tế'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(navigation_dir, 'config', 'mapper_params_online_async.yaml'),
        description='Đường dẫn tới file cấu hình của slam_toolbox'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Mở RViz với config SLAM (LaserScan Best Effort, Fixed Frame map)'
    )

    # Định nghĩa Node chạy SLAM Toolbox (chế độ async cho xe chạy thực)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # RViz2 với config SLAM (chỉ chạy khi rviz:=true)
    rviz_config = os.path.join(navigation_dir, 'config', 'slam.rviz')
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Đưa tất cả vào LaunchDescription
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(rviz_arg)
    ld.add_action(slam_node)
    ld.add_action(rviz_node)

    return ld
