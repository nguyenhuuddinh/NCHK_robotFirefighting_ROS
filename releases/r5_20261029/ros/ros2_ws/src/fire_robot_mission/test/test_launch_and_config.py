"""Package wiring tests for mission preview launch and parameters."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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
    assert params['post_spray_review_topic'] == (
        '/fire_mission/post_spray_review')
    assert params['actuator_state_topic'] == (
        '/fire_sensors/actuator_state')
    assert params['review_timeout_s'] <= 0.6
    assert params['actuator_timeout_s'] <= 0.35
    assert params['detection_timeout_s'] == 0.5
    assert 0 < params['fire_reacquire_hold_s'] < \
        params['fire_acquire_hold_s']
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


def test_shadow_config_is_isolated_and_fail_closed():
    params = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'mission_params.yaml').read_text()
    )['alignment_handoff_shadow']['ros__parameters']
    assert params['shadow_command_topic'] == (
        '/fire_mission/alignment_cmd_vel_shadow')
    assert params['shadow_command_topic'].endswith('_shadow')
    assert params['shadow_command_topic'] not in {
        '/cmd_vel', '/cmd_vel_raw', '/cmd_vel_nav'
    }
    assert params['arm_timeout_s'] < 0.5
    assert params['max_angular_speed_rad_s'] <= 0.35
    assert params['require_live_cancel_proof'] is False
    assert params['gateway_state_topic'] == (
        '/fire_mission/live_nav_stop_state')
    assert params['cancel_guard_state_topic'] == (
        '/fire_mission/nav2_cancel_guard_state')
    assert params['gateway_timeout_s'] < 0.5
    assert params['cancel_guard_timeout_s'] < 0.5


def test_shadow_launch_contains_only_supervisor_and_shadow_gate():
    source = (
        PACKAGE_ROOT / 'launch' /
        'mission_handoff_shadow.launch.py').read_text()
    assert "executable='fire_mission_supervisor'" in source
    assert "executable='alignment_handoff_shadow'" in source
    assert "package='fire_robot_navigation'" not in source
    assert "package='fire_robot_bringup'" not in source


def test_motion_arbiter_shadow_config_is_isolated_and_bounded():
    params = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'mission_params.yaml').read_text()
    )['motion_arbiter_shadow']['ros__parameters']
    assert params['nav_command_topic'] == (
        '/fire_mission/nav2_cmd_vel_shadow')
    assert params['nav_command_topic'].endswith('_shadow')
    assert params['alignment_command_topic'] == (
        '/fire_mission/alignment_cmd_vel_shadow')
    assert params['alignment_command_topic'].endswith('_shadow')
    assert params['shadow_command_topic'] == (
        '/fire_mission/cmd_vel_arbiter_shadow')
    assert params['shadow_command_topic'].endswith('_shadow')
    assert params['shadow_command_topic'] not in {
        '/cmd_vel', '/cmd_vel_raw', '/cmd_vel_nav'
    }
    assert params['max_nav_linear_speed_m_s'] <= 0.10
    assert params['max_nav_angular_speed_rad_s'] == 0.8
    assert params['max_alignment_angular_speed_rad_s'] <= 0.35
    assert params['mission_timeout_s'] < 0.5


def test_motion_arbiter_launch_has_no_robot_or_nav2_nodes():
    source = (
        PACKAGE_ROOT / 'launch' /
        'mission_motion_arbiter_shadow.launch.py').read_text()
    assert "executable='fire_mission_supervisor'" in source
    assert "executable='alignment_handoff_shadow'" in source
    assert "executable='motion_arbiter_shadow'" in source
    assert "executable='nav2_cancel_intent_shadow'" in source
    assert "'require_live_cancel_proof'" in source
    assert "default_value='false'" in source
    assert "value_type=bool" in source
    assert "package='fire_robot_navigation'" not in source
    assert "package='fire_robot_bringup'" not in source
    assert "package='fire_robot_safety'" not in source


def test_cancel_intent_config_is_read_only_and_isolated():
    params = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'mission_params.yaml').read_text()
    )['nav2_cancel_intent_shadow']['ros__parameters']
    assert params['mission_state_topic'] == '/fire_mission/state'
    assert params['nav_status_topic'] == (
        '/navigate_to_pose/_action/status')
    assert params['intent_topic'] == (
        '/fire_mission/nav2_cancel_intent_shadow')
    assert params['intent_topic'].endswith('_shadow')
    assert params['mission_timeout_s'] < 0.5
    assert 'status_timeout_s' not in params


def test_nav_cancel_guard_is_default_off_and_shadow_only():
    params = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'mission_params.yaml').read_text()
    )['nav2_cancel_guard']['ros__parameters']
    assert params['enable_real_cancel'] is False
    assert params['cancel_service'] == (
        '/navigate_to_pose/_action/cancel_goal')
    assert params['raw_cmd_topic'] == '/cmd_vel_raw'
    assert params['shadow_nav_cmd_topic'] == (
        '/fire_mission/nav2_cmd_vel_shadow')
    assert params['arm_topic'] == '/fire_mission/nav_cancel_arm'
    assert params['state_topic'] == (
        '/fire_mission/nav2_cancel_guard_state')
    assert params['zero_hold_s'] >= 0.5
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'nav2_cancel_guard.launch.py').read_text()
    assert "default_value='false'" in launch_source
    assert "executable='nav2_cancel_guard'" in launch_source
    assert "package='fire_robot_navigation'" not in launch_source


def test_live_nav_gateway_requires_explicit_opt_in():
    params = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'mission_params.yaml').read_text()
    )['live_nav_stop_gateway']['ros__parameters']
    assert params['enable_live_output'] is False
    assert params['arm_timeout_s'] <= 0.40
    assert params['stationary_hold_s'] >= 0.50
    assert params['max_nav_linear_m_s'] <= 0.10
    assert params['max_nav_angular_rad_s'] == 0.8
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'mission_live_nav_stop.launch.py'
    ).read_text()
    assert "default_value='false'" in launch_source
    assert "executable='live_nav_stop_gateway'" in launch_source
    assert "package='fire_robot_navigation'" not in launch_source


def test_nav2_gateway_and_shadow_share_the_same_turn_ceiling():
    mission = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'mission_params.yaml').read_text())
    nav2_share = Path(get_package_share_directory('fire_robot_navigation'))
    nav2 = yaml.safe_load(
        (nav2_share / 'config' / 'nav2_params.yaml').read_text())
    ceiling = mission['live_nav_stop_gateway']['ros__parameters'][
        'max_nav_angular_rad_s']
    shadow_ceiling = mission['motion_arbiter_shadow']['ros__parameters'][
        'max_nav_angular_speed_rad_s']
    smoother = nav2['velocity_smoother']['ros__parameters']
    recovery = nav2['behavior_server']['ros__parameters']
    controller = nav2['controller_server']['ros__parameters'][
        'FollowPath']
    assert ceiling == shadow_ceiling == 0.8
    assert smoother['max_velocity'][2] == ceiling
    assert smoother['min_velocity'][2] == -ceiling
    assert recovery['max_rotational_vel'] == ceiling
    assert controller['rotate_to_heading_angular_vel'] == ceiling


def test_live_alignment_is_separate_default_off_and_speed_limited():
    params = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'mission_params.yaml').read_text()
    )['live_alignment_takeover']['ros__parameters']
    assert params['enable_live_alignment'] is False
    assert params['max_angular_speed_rad_s'] == 0.60
    assert params['max_observed_angular_rad_s'] == 0.80
    assert 'max_turn_duration_s' not in params
    assert 'max_total_turn_duration_s' not in params
    assert 0 < params['max_total_rotation_rad'] <= 6.283185307179586
    assert params['min_rotation_clearance_m'] >= 0.30
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'mission_live_alignment.launch.py'
    ).read_text()
    assert "default_value='false'" in launch_source
    assert "executable='live_alignment_takeover'" in launch_source
    assert "package='fire_robot_navigation'" not in launch_source


def test_pump_demo_is_separate_and_requires_two_opt_ins():
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'pump_demo.launch.py').read_text()
    assert "executable='pump_demo_controller'" in launch_source
    assert "DeclareLaunchArgument('enable_pump_demo', default_value='false')" \
        in launch_source
    assert "DeclareLaunchArgument('allow_provisional_ir', " \
        "default_value='false')" in launch_source
    assert "DeclareLaunchArgument('pulse_s', default_value='5.0')" \
        in launch_source
    assert "'pulse_s': ParameterValue(" in launch_source
    assert "package='fire_robot_navigation'" not in launch_source
