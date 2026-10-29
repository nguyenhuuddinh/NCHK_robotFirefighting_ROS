import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('fire_robot_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'fire_robot.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'config', 'fire_robot.rviz')

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Robot description (xacro → URDF string)
    # ParameterValue(value_type=str) để ROS 2 không nhầm XML thành YAML
    robot_description = ParameterValue(
        Command(['xacro "' + urdf_file + '"']),
        value_type=str
    )

    # robot_state_publisher: publish TF from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen',
    )

    # RViz2 (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher,
        rviz_node,
    ])
