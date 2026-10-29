"""Opt-in laptop Nav2 composition, with the baseline mission's isolated output."""

import importlib.util
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from ament_index_python.resources import get_resource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


SHADOW_CMD_VEL_TOPIC = '/fire_mission/nav2_cmd_vel_shadow'
COMPONENTS = (
    ('map_server', 'nav2_map_server', 'nav2_map_server::MapServer'),
    ('amcl', 'nav2_amcl', 'nav2_amcl::AmclNode'),
    ('controller_server', 'nav2_controller', 'nav2_controller::ControllerServer'),
    ('smoother_server', 'nav2_smoother', 'nav2_smoother::SmootherServer'),
    ('planner_server', 'nav2_planner', 'nav2_planner::PlannerServer'),
    ('behavior_server', 'nav2_behaviors', 'behavior_server::BehaviorServer'),
    ('bt_navigator', 'nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator'),
    ('velocity_smoother', 'nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother'),
    ('lifecycle_manager_localization', 'nav2_lifecycle_manager', 'nav2_lifecycle_manager::LifecycleManager'),
    ('lifecycle_manager_navigation', 'nav2_lifecycle_manager', 'nav2_lifecycle_manager::LifecycleManager'),
)


def verify_runtime_components():
    """Fail before spawning when the installed Humble component set is incomplete."""
    executable = os.path.join(get_package_prefix('fire_robot_navigation'), 'lib',
                              'fire_robot_navigation', 'fire_robot_component_container_isolated')
    if not os.path.isfile(executable) or not os.access(executable, os.X_OK):
        raise RuntimeError('Missing executable: ' + executable)
    for _, package, plugin in COMPONENTS:
        content, prefix = get_resource('rclcpp_components', package)
        registered = dict(line.split(';', 1) for line in content.splitlines() if ';' in line)
        if plugin not in registered:
            raise RuntimeError(f'Component not registered: {package}/{plugin}')
        library = os.path.join(prefix, registered[plugin])
        if not os.path.isfile(library):
            raise RuntimeError('Missing component library: ' + library)


def preflight_checks(context, *args, **kwargs):
    requested = context.launch_configurations.get('cmd_vel_output_topic')
    if requested not in (None, SHADOW_CMD_VEL_TOPIC):
        raise RuntimeError('Composed shadow profile rejects cmd_vel_output_topic: ' + str(requested))
    # Reuse the exact baseline map/YAML/command validation; do not fork its rules.
    path = os.path.join(get_package_share_directory('fire_robot_navigation'),
                        'launch', 'nav2.launch.py')
    spec = importlib.util.spec_from_file_location('baseline_nav2_preflight', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.preflight_checks(context)
    verify_runtime_components()


def configured_parameters():
    """Pass the same rewritten file to the process and its direct components.

    Humble ControllerServer/PlannerServer construct child costmaps with global
    NodeOptions. LoadNode parameters alone do not reach those child nodes.
    """
    return RewrittenYaml(
        source_file=LaunchConfiguration('params_file'), root_key='',
        param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'yaml_filename': LaunchConfiguration('map')},
        convert_types=True)


def build_components(package_dir):
    """Keep baseline node names, parameter order, lifecycle groups and routes."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    configured_params = configured_parameters()
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    result = []
    for name, package, plugin in COMPONENTS:
        parameters = [configured_params]
        remappings = list(tf_remaps)
        if name == 'controller_server':
            remappings.append(('cmd_vel', 'cmd_vel_nav'))
        elif name == 'behavior_server':
            remappings.append(('cmd_vel', SHADOW_CMD_VEL_TOPIC))
        elif name == 'velocity_smoother':
            remappings.extend([('cmd_vel', 'cmd_vel_nav'),
                               ('cmd_vel_smoothed', SHADOW_CMD_VEL_TOPIC)])
        elif name == 'bt_navigator':
            parameters.append({
                'default_nav_to_pose_bt_xml': os.path.join(package_dir, 'behavior_trees',
                                                          'navigate_to_pose_no_reverse.xml'),
                'default_nav_through_poses_bt_xml': os.path.join(package_dir, 'behavior_trees',
                                                                'navigate_through_poses_no_reverse.xml'),
            })
        elif name.startswith('lifecycle_manager_'):
            nodes = (['map_server', 'amcl'] if name.endswith('_localization') else
                     ['controller_server', 'smoother_server', 'planner_server',
                      'behavior_server', 'bt_navigator', 'velocity_smoother'])
            parameters = [{'use_sim_time': use_sim_time}, {'autostart': autostart},
                          {'node_names': nodes}]
            remappings = []
        result.append(ComposableNode(package=package, plugin=plugin, name=name,
                                     parameters=parameters, remappings=remappings))
    return result


def on_container_exit(event, context):
    # Humble launch_ros registers ROSAdapter.shutdown for each Shutdown event.
    # SIGINT already dispatched one; emitting another on the expected child exit
    # attempts to stop the adapter twice. A crash while running must still end
    # the launch (including its optional RViz).
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(reason='Nav2 composed container exited'))]


def generate_launch_description():
    package_dir = get_package_share_directory('fire_robot_navigation')
    container = ComposableNodeContainer(
        name='nav2_container', namespace='', package='fire_robot_navigation',
        executable='fire_robot_component_container_isolated', output='screen',
        parameters=[configured_parameters(), {'refresh_bt_action_status': ParameterValue(
            LaunchConfiguration('refresh_bt_action_status', default='false'), value_type=bool)}],
        composable_node_descriptions=build_components(package_dir))
    # A dead container must end the launch even when its optional RViz is still up.
    container_exit = RegisterEventHandler(OnProcessExit(
        target_action=container,
        on_exit=on_container_exit))
    rviz_config = os.path.join(get_package_share_directory('nav2_bringup'),
                               'rviz', 'nav2_default_view.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', description='Absolute path to the map YAML'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            package_dir, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('refresh_bt_action_status', default_value='false'),
        OpaqueFunction(function=preflight_checks),
        container_exit,
        container,
        ExecuteProcess(condition=IfCondition(LaunchConfiguration('rviz')),
                       cmd=['rviz2', '-d', rviz_config], output='screen'),
    ])
