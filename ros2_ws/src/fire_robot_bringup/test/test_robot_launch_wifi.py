"""Offline launch-contract regression for the scoped Fast DDS C2 patch."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def actions():
    spec = importlib.util.spec_from_file_location(
        'robot_launch_wifi_c2', PACKAGE_DIR / 'launch' / 'robot.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _: str(PACKAGE_DIR)
    return module.generate_launch_description().entities


@pytest.mark.parametrize('executable', ['serial_bridge_node', 'odom_to_tf_broadcaster'])
def test_telemetry_process_receives_async_mode(executable):
    node = next(a for a in actions() if isinstance(a, Node)
                and a.node_executable == executable)
    context = LaunchContext()
    env = {perform_substitutions(context, key): perform_substitutions(context, value)
           for key, value in node.additional_env}
    assert env == {'RMW_FASTRTPS_PUBLICATION_MODE': 'ASYNCHRONOUS'}


def test_async_is_not_exported_to_safety_camera_or_global_launch():
    entities = actions()
    assert not any(isinstance(a, SetEnvironmentVariable) for a in entities)
    for node in (a for a in entities if isinstance(a, Node)):
        if node.node_executable not in ('serial_bridge_node', 'odom_to_tf_broadcaster'):
            assert not node.additional_env
    # The safety include remains parameterized by the normal gate arguments.
    context = LaunchContext()
    context.launch_configurations.update(cmd_vel_input_topic='/fire_robot_static/cmd_vel_raw',
        cmd_vel_output_topic='/cmd_vel', heartbeat_timeout_ms='1000',
        cmd_vel_output_rate_hz='10.0')
    include = next(a for a in entities if isinstance(a, IncludeLaunchDescription)
                   and 'heartbeat_timeout_ms' in dict(a.launch_arguments))
    values = {name: perform_substitutions(context, value if isinstance(value, (list, tuple)) else [value])
              for name, value in include.launch_arguments}
    assert values == {'cmd_vel_input_topic': '/fire_robot_static/cmd_vel_raw',
                      'cmd_vel_output_topic': '/cmd_vel',
                      'heartbeat_timeout_ms': '1000', 'cmd_vel_output_rate_hz': '10.0'}
