"""Static safety checks for the observe-only mission package."""

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR = PACKAGE_ROOT / 'fire_robot_mission' / 'mission_supervisor.py'
CONFIG = PACKAGE_ROOT / 'config' / 'mission_params.yaml'


def test_supervisor_has_only_one_string_publisher():
    tree = ast.parse(SUPERVISOR.read_text())
    publisher_calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_publisher'
    ]
    assert len(publisher_calls) == 1
    message_type = publisher_calls[0].args[0]
    assert isinstance(message_type, ast.Name)
    assert message_type.id == 'String'


def test_supervisor_has_no_action_client_or_command_message_import():
    tree = ast.parse(SUPERVISOR.read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            imported.update(item.name for item in node.names)
    assert 'ActionClient' not in imported
    assert 'Bool' not in imported
    assert 'Point' not in imported


def test_config_output_is_not_a_command_topic():
    params = yaml.safe_load(CONFIG.read_text())[
        'fire_mission_supervisor']['ros__parameters']
    assert params['state_topic'] == '/fire_mission/state'
    assert params['state_topic'] not in {
        '/cmd_vel',
        '/cmd_vel_raw',
        '/cmd_vel_nav',
        '/pump_cmd',
        '/fire_target',
    }
    assert params['fire_acquire_hold_s'] > 0.0
    assert params['max_future_skew_ms'] > 0.0
