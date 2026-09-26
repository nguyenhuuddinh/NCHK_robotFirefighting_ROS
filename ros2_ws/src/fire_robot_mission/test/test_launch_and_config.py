"""Package wiring tests for mission preview launch and parameters."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_config_declares_expected_inputs_and_safety_limits():
    params = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'mission_params.yaml').read_text()
    )['fire_mission_supervisor']['ros__parameters']
    assert params['detections_topic'] == '/yolo/detections'
    assert params['alignment_topic'] == '/yolo/alignment_preview'
    assert params['nav_command_topic'] == '/cmd_vel_nav'
    assert params['raw_command_topic'] == '/cmd_vel_raw'
    assert params['nav_status_topic'] == (
        '/navigate_to_pose/_action/status')
    assert params['detection_timeout_s'] == 0.5
    assert params['max_source_age_ms'] == 300.0
    assert params['max_future_skew_ms'] == 20.0


def test_launch_exposes_params_and_sim_time_without_other_nodes():
    source = (
        PACKAGE_ROOT / 'launch' / 'mission_preview.launch.py').read_text()
    assert "'params_file'" in source
    assert "'use_sim_time'" in source
    assert "package='fire_robot_mission'" in source
    assert "executable='fire_mission_supervisor'" in source
    assert "package='fire_robot_navigation'" not in source
    assert "package='fire_robot_bringup'" not in source
