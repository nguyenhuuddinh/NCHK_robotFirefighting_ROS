"""Offline launch checks for the laptop-only perception node."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_launch_is_standalone_and_observe_only(monkeypatch, tmp_path):
    """Launch starts only observe-only perception and preview nodes."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    launch_path = PACKAGE_DIR / 'launch' / 'perception.launch.py'
    spec = importlib.util.spec_from_file_location(
        'perception_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda _: str(PACKAGE_DIR)

    actions = module.generate_launch_description().entities
    arguments = {
        action.name for action in actions
        if isinstance(action, DeclareLaunchArgument)
    }
    nodes = [action for action in actions if isinstance(action, Node)]

    assert {'model_path', 'params_file', 'use_sim_time'} <= arguments
    assert any(isinstance(action, OpaqueFunction) for action in actions)
    assert len(nodes) == 2
    assert all(node.node_package == 'fire_robot_perception' for node in nodes)
    assert {node.node_executable for node in nodes} == {
        'yolo_observer',
        'fire_alignment_observer',
    }
