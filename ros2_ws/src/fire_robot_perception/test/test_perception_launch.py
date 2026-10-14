"""Offline launch checks for the laptop-only perception node."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.utilities import perform_substitutions
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

    assert {
        'model_path',
        'python_executable',
        'params_file',
        'use_sim_time',
    } <= arguments
    assert any(isinstance(action, OpaqueFunction) for action in actions)
    assert len(nodes) == 1
    assert nodes[0].node_package == 'fire_robot_perception'
    assert nodes[0].node_executable == 'fire_alignment_observer'

    processes = [
        action for action in actions
        if isinstance(action, ExecuteProcess) and not isinstance(action, Node)
    ]
    assert len(processes) == 1
    context = LaunchContext()
    context.launch_configurations.update({
        'python_executable': '/tmp/yolo-python',
        'params_file': '/tmp/yolo-params.yaml',
        'model_path': '/tmp/yolo-model.pt',
        'use_sim_time': 'false',
    })
    command = [
        perform_substitutions(context, part) for part in processes[0].cmd
    ]
    assert command == [
        '/tmp/yolo-python',
        '-m',
        'fire_robot_perception.yolo_observer',
        '--ros-args',
        '-r',
        '__node:=yolo_observer',
        '--params-file',
        '/tmp/yolo-params.yaml',
        '-p',
        'model_path:=/tmp/yolo-model.pt',
        '-p',
        'use_sim_time:=false',
    ]


def test_dependency_import_has_bounded_startup_timeout(monkeypatch, tmp_path):
    """Allow slow startup imports without changing runtime sensor deadlines."""
    launch_path = PACKAGE_DIR / 'launch' / 'perception.launch.py'
    spec = importlib.util.spec_from_file_location(
        'perception_launch_timeout', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    model = tmp_path / 'model.pt'
    params = tmp_path / 'params.yaml'
    python = tmp_path / 'python'
    for path in (model, params, python):
        path.touch()
    python.chmod(0o700)

    context = LaunchContext()
    context.launch_configurations.update({
        'model_path': str(model),
        'params_file': str(params),
        'python_executable': str(python),
    })
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(module.subprocess, 'run', fake_run)
    assert module._validate_paths(context) == []
    assert calls[0][0] == [str(python), '-c',
                           'import torch; import ultralytics']
    assert calls[0][1]['timeout'] == 60.0
