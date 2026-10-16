"""Check the Pi launch keeps DDS publication off odometry callbacks."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def test_serial_and_tf_use_async_fastrtps_publication():
    launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'robot.launch.py'
    spec = importlib.util.spec_from_file_location('robot_launch_wifi', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actions = module.generate_launch_description().entities
    context = LaunchContext()

    for executable in ('serial_bridge_node', 'odom_to_tf_broadcaster'):
        node = next(action for action in actions
                    if isinstance(action, Node)
                    and action.node_executable == executable)
        env = {
            perform_substitutions(context, key):
            perform_substitutions(context, value)
            for key, value in node.additional_env
        }
        assert env == {'RMW_FASTRTPS_PUBLICATION_MODE': 'ASYNCHRONOUS'}
