import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def preflight_checks(context, *args, **kwargs):
    map_yaml_file = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    if not map_yaml_file:
        raise RuntimeError("The 'map' argument is empty but required.")

    if not os.path.isabs(map_yaml_file):
        raise RuntimeError(f"The 'map' argument must be an absolute path: {map_yaml_file}")

    if not os.path.isfile(map_yaml_file):
        raise RuntimeError(f"The map YAML file does not exist or is not a file: {map_yaml_file}")

    try:
        with open(map_yaml_file, 'r') as f:
            map_data = yaml.safe_load(f)
            if not isinstance(map_data, dict):
                raise RuntimeError(
                    f"The map YAML does not contain a valid mapping: {map_yaml_file}")
            if 'image' not in map_data:
                raise RuntimeError(f"The map YAML file is missing 'image' key: {map_yaml_file}")

            image_path = map_data['image']
            if not os.path.isabs(image_path):
                # resolve relative to yaml
                image_path = os.path.join(os.path.dirname(map_yaml_file), image_path)

            if not os.path.isfile(image_path):
                raise RuntimeError(
                    f"The map image file does not exist or is not a file: {image_path}")
    except Exception as e:
        raise RuntimeError(f"Failed to parse or validate map YAML: {e}")

    if not params_file:
        raise RuntimeError("The 'params_file' argument is empty but required.")

    if not os.path.isabs(params_file):
        raise RuntimeError(f"The 'params_file' argument must be an absolute path: {params_file}")

    if not os.path.isfile(params_file):
        raise RuntimeError(f"The params_file does not exist or is not a file: {params_file}")

    try:
        with open(params_file, 'r') as f:
            params_data = yaml.safe_load(f)
            if params_data is None:
                raise RuntimeError(f"The params_file is empty: {params_file}")
            if not isinstance(params_data, dict):
                raise RuntimeError(
                    f"The params_file does not contain a valid mapping: {params_file}")
    except yaml.YAMLError as e:
        raise RuntimeError(f"The params_file is malformed YAML: {e}")


def generate_launch_description():
    pkg_share = get_package_share_directory('fire_robot_navigation')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time')
    declare_map = DeclareLaunchArgument('map', description='Full path to map yaml file')
    declare_params_file = DeclareLaunchArgument('params_file', default_value=os.path.join(
        pkg_share, 'config', 'nav2_params.yaml'), description='Full path to param file')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the nav2 stack')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    bt_navigator_params = {
        'default_nav_to_pose_bt_xml': os.path.join(
            pkg_share, 'behavior_trees', 'navigate_to_pose_no_reverse.xml'),
        'default_nav_through_poses_bt_xml': os.path.join(
            pkg_share, 'behavior_trees', 'navigate_through_poses_no_reverse.xml')
    }

    lifecycle_nodes_localization = ['map_server', 'amcl']
    lifecycle_nodes_navigation = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother'
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    rviz_config_file = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')

    start_rviz_cmd = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('rviz')),
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        declare_autostart,
        declare_rviz,
        OpaqueFunction(function=preflight_checks),
        start_rviz_cmd,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', '/cmd_vel_raw')]),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params, bt_navigator_params],
            remappings=remappings),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav'),
                                     ('cmd_vel_smoothed', '/cmd_vel_raw')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes_localization}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes_navigation}]),
    ])
