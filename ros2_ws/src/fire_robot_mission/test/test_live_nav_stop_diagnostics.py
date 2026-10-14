"""Read-only gateway report invariants without any ROS hardware graph."""

from fire_robot_mission.live_nav_stop_diagnostics import GatewayAccumulator


def frame(state, linear=0.0, angular=0.0, **overrides):
    """Construct a minimal valid gateway telemetry frame."""
    payload = {
        'schema_version': 1, 'one_shot': True, 'state': state,
        'enabled': True, 'fire_latched': False, 'arm_fresh': True,
        'released': state in {'RELEASED_STOPPED', 'HANDOFF_READY'},
        'handoff_ready': state == 'HANDOFF_READY',
        'nav_shadow_fingerprint_ok': True,
        'raw_owner_fingerprint_ok': state == 'NAVIGATION',
        'safety_gate_fingerprint_ok': True,
        'raw_publishers': 1 if state == 'NAVIGATION' else 0,
        'mission_publishers': 1, 'odom_publishers': 1,
        'live_arm_publishers': 1, 'nav_status_publishers': 1,
        'cancel_guard_publishers': 1,
        'odom_stationary': state == 'HANDOFF_READY',
        'cmd_vel_fresh_zero': state == 'HANDOFF_READY',
        'blockers': [],
        'command': {'linear_x_m_s': linear, 'angular_z_rad_s': angular},
        'safety': {
            'actuator_command_publisher': False,
            'alignment_command_publisher': False,
        },
    }
    payload.update(overrides)
    return payload


def test_navigation_stop_handoff_sequence_is_valid():
    report = GatewayAccumulator()
    report.add(frame('NAVIGATION', 0.05, 0.2), 1.0)
    report.add(frame('STOPPING'), 1.1)
    report.add(frame('RELEASING'), 1.6)
    report.add(frame('HANDOFF_READY'), 1.7)
    summary = report.summary()
    assert summary['contract_violations'] == 0
    assert summary['max_abs_linear_x_m_s'] == 0.05
    assert summary['transitions']['RELEASING->HANDOFF_READY'] == 1


def test_nonzero_after_stop_or_resume_after_release_is_violation():
    report = GatewayAccumulator()
    report.add(frame('RELEASED_STOPPED'), 1.0)
    report.add(frame('NAVIGATION', 0.05), 1.1)
    report.add(frame('STOPPING', 0.05), 1.2)
    assert report.summary()['contract_violations'] == 2


def test_handoff_requires_zero_stopped_and_no_raw_owner():
    report = GatewayAccumulator()
    report.add(frame('HANDOFF_READY', raw_publishers=1), 1.0)
    report.add(frame('HANDOFF_READY', cmd_vel_fresh_zero=False), 1.1)
    report.add(frame('HANDOFF_READY', odom_stationary=False), 1.2)
    assert report.summary()['contract_violations'] == 3


def test_navigation_turn_limit_is_eight_tenths_radian_per_second():
    report = GatewayAccumulator()
    report.add(frame('NAVIGATION', angular=0.8), 1.0)
    report.add(frame('NAVIGATION', angular=0.81), 1.1)
    assert report.summary()['contract_violations'] == 1
