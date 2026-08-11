import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    # Đưa tất cả vào LaunchDescription
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(slam_node)
    
    return ld
