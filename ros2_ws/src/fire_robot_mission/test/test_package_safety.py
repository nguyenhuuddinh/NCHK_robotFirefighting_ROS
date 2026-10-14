"""Static safety checks for the observe-only mission package."""

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR = PACKAGE_ROOT / 'fire_robot_mission' / 'mission_supervisor.py'
CONFIG = PACKAGE_ROOT / 'config' / 'mission_params.yaml'
HANDOFF = (
    PACKAGE_ROOT / 'fire_robot_mission' / 'alignment_handoff_shadow.py')
ARBITER = (
    PACKAGE_ROOT / 'fire_robot_mission' / 'motion_arbiter_shadow.py')
CANCEL_INTENT = (
    PACKAGE_ROOT / 'fire_robot_mission' / 'nav2_cancel_intent_shadow.py')
CANCEL_GUARD = (
    PACKAGE_ROOT / 'fire_robot_mission' / 'nav2_cancel_guard.py')
LIVE_GATEWAY = (
    PACKAGE_ROOT / 'fire_robot_mission' / 'live_nav_stop_gateway.py')
LIVE_DIAGNOSTICS = (
    PACKAGE_ROOT / 'fire_robot_mission' / 'live_nav_stop_diagnostics.py')
LIVE_ALIGNMENT = (
    PACKAGE_ROOT / 'fire_robot_mission' / 'live_alignment_takeover.py')
ALIGNMENT_DIAGNOSTICS = (
    PACKAGE_ROOT / 'fire_robot_mission' / 'live_alignment_diagnostics.py')
PUMP_DEMO = (
    PACKAGE_ROOT / 'fire_robot_mission' / 'pump_demo_controller.py')
POST_SPRAY_REVIEW = (
    PACKAGE_ROOT / 'fire_robot_mission' / 'post_spray_review.py')


def test_post_spray_review_has_only_one_string_publisher():
    tree = ast.parse(POST_SPRAY_REVIEW.read_text())
    publishers = [
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call) and
        isinstance(call.func, ast.Attribute) and
        call.func.attr == 'create_publisher'
    ]
    assert len(publishers) == 1
    assert isinstance(publishers[0].args[0], ast.Name)
    assert publishers[0].args[0].id == 'String'
    assert "'/fire_mission/post_spray_review'" in \
        POST_SPRAY_REVIEW.read_text()
    assert 'Bool' not in POST_SPRAY_REVIEW.read_text()
    assert 'Twist' not in POST_SPRAY_REVIEW.read_text()
    assert 'ActionClient' not in POST_SPRAY_REVIEW.read_text()


def test_pump_demo_has_no_motion_or_servo_publisher():
    tree = ast.parse(PUMP_DEMO.read_text())
    publishers = [
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call) and
        isinstance(call.func, ast.Attribute) and
        call.func.attr == 'create_publisher'
    ]
    assert len(publishers) == 2
    assert {call.args[0].id for call in publishers} == {'Bool', 'String'}
    assert "'/pump_cmd'" in PUMP_DEMO.read_text()
    assert "'/cmd_vel_raw'" not in [
        call.args[1].value for call in publishers
        if isinstance(call.args[1], ast.Constant)]


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


def test_handoff_shadow_twist_output_is_not_a_robot_command_topic():
    params = yaml.safe_load(CONFIG.read_text())[
        'alignment_handoff_shadow']['ros__parameters']
    output = params['shadow_command_topic']
    assert output.startswith('/fire_mission/')
    assert output.endswith('_shadow')
    assert output not in {
        '/cmd_vel',
        '/cmd_vel_raw',
        '/cmd_vel_nav',
        '/pump_cmd',
        '/fire_target',
    }


def test_handoff_source_has_no_action_or_actuator_command_type():
    tree = ast.parse(HANDOFF.read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            imported.update(item.name for item in node.names)
    assert 'ActionClient' not in imported
    assert 'Point' not in imported


def test_arbiter_shadow_output_is_not_a_robot_command_topic():
    params = yaml.safe_load(CONFIG.read_text())[
        'motion_arbiter_shadow']['ros__parameters']
    nav_input = params['nav_command_topic']
    alignment_input = params['alignment_command_topic']
    output = params['shadow_command_topic']
    assert nav_input == '/fire_mission/nav2_cmd_vel_shadow'
    assert nav_input.endswith('_shadow')
    assert nav_input not in {
        '/cmd_vel',
        '/cmd_vel_raw',
        '/cmd_vel_nav',
        '/pump_cmd',
        '/fire_target',
    }
    assert alignment_input == '/fire_mission/alignment_cmd_vel_shadow'
    assert alignment_input.endswith('_shadow')
    assert alignment_input not in {
        '/cmd_vel',
        '/cmd_vel_raw',
        '/cmd_vel_nav',
        '/pump_cmd',
        '/fire_target',
    }
    assert output.startswith('/fire_mission/')
    assert output.endswith('_shadow')
    assert output not in {
        '/cmd_vel',
        '/cmd_vel_raw',
        '/cmd_vel_nav',
        '/pump_cmd',
        '/fire_target',
    }


def test_arbiter_source_has_no_action_or_actuator_command_type():
    tree = ast.parse(ARBITER.read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            imported.update(item.name for item in node.names)
    assert 'ActionClient' not in imported
    assert 'Bool' not in imported
    assert 'Point' not in imported


def test_arbiter_has_one_isolated_twist_publisher():
    tree = ast.parse(ARBITER.read_text())
    publisher_calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_publisher'
    ]
    twist_publishers = [
        call for call in publisher_calls
        if isinstance(call.args[0], ast.Name)
        and call.args[0].id == 'Twist'
    ]
    assert len(twist_publishers) == 1
    assert 'FORBIDDEN_COMMAND_TOPICS' in ARBITER.read_text()


def test_cancel_intent_has_only_one_string_publisher_and_no_action_client():
    tree = ast.parse(CANCEL_INTENT.read_text())
    publishers = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_publisher'
    ]
    assert len(publishers) == 1
    assert isinstance(publishers[0].args[0], ast.Name)
    assert publishers[0].args[0].id == 'String'
    imported = {
        item.name for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) for item in node.names
    }
    assert 'ActionClient' not in imported
    assert 'Twist' not in imported
    assert 'Bool' not in imported


def test_real_cancel_guard_only_publishes_telemetry_not_motion():
    tree = ast.parse(CANCEL_GUARD.read_text())
    publishers = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_publisher'
    ]
    assert len(publishers) == 1
    assert isinstance(publishers[0].args[0], ast.Name)
    assert publishers[0].args[0].id == 'String'
    clients = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_client'
    ]
    assert len(clients) == 1
    assert isinstance(clients[0].args[0], ast.Name)
    assert clients[0].args[0].id == 'CancelGoal'
    assert 'CANONICAL_ENDPOINTS' in CANCEL_GUARD.read_text()


def test_live_gateway_has_one_raw_twist_publisher_and_no_actuator_client():
    tree = ast.parse(LIVE_GATEWAY.read_text())
    publishers = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_publisher'
    ]
    twist_publishers = [
        call for call in publishers
        if isinstance(call.args[0], ast.Name)
        and call.args[0].id == 'Twist'
    ]
    assert len(twist_publishers) == 1
    assert isinstance(twist_publishers[0].args[1], ast.Constant)
    assert twist_publishers[0].args[1].value == '/cmd_vel_raw'
    imported = {
        item.name for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) for item in node.names
    }
    assert 'ActionClient' not in imported
    assert 'CancelGoal' not in imported
    assert 'Point' not in imported
    assert 'CANONICAL_TOPICS' in LIVE_GATEWAY.read_text()
    assert 'resolve_topic_name' in LIVE_GATEWAY.read_text()


def test_live_gateway_diagnostics_has_no_publisher_or_action_client():
    tree = ast.parse(LIVE_DIAGNOSTICS.read_text())
    calls = [
        node.func.attr for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    ]
    assert 'create_subscription' in calls
    assert 'create_publisher' not in calls
    assert 'create_client' not in calls


def test_live_alignment_has_one_raw_twist_publisher_and_no_actuator():
    source = LIVE_ALIGNMENT.read_text()
    tree = ast.parse(source)
    publishers = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_publisher'
    ]
    twists = [call for call in publishers
              if isinstance(call.args[0], ast.Name)
              and call.args[0].id == 'Twist']
    assert len(twists) == 1
    assert isinstance(twists[0].args[1], ast.Constant)
    assert twists[0].args[1].value == '/cmd_vel_raw'
    assert 'CANONICAL_TOPICS' in source
    assert 'resolve_topic_name' in source
    assert 'create_client' not in source
    assert not any(
        len(call.args) > 1 and isinstance(call.args[1], ast.Constant) and
        call.args[1].value == '/pump_cmd' for call in publishers)


def test_live_alignment_diagnostics_has_no_publisher_or_client():
    tree = ast.parse(ALIGNMENT_DIAGNOSTICS.read_text())
    calls = [
        node.func.attr for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
    ]
    assert calls.count('create_subscription') == 2
    assert 'create_publisher' not in calls
    assert 'create_client' not in calls
