"""Only the historical ready_core fixture; no old test is collected or run."""
from fire_robot_mission.pump_demo_core import PumpDemoCore
ZERO=(0.,)*6

def ready_core(pulse_s=5.0):
    core = PumpDemoCore(
        enabled=True, allow_provisional_ir=True, pulse_s=pulse_s)
    core.update('ir', {
        'schema_version': 1, 'mode': 'OBSERVE_ONLY', 'observe_only': True,
        'zone': 'SPRAY_RANGE', 'blockers': [],
        'calibration': 'PROVISIONAL_SINGLE_FLAME',
        'validated_for_actuation': False,
        'safety': {'actuator_command_publisher': False,
                   'motion_command_publisher': False}}, 1.0)
    core.update('mission', {
        'schema_version': 1, 'observe_only': True,
        'state': 'ALIGNMENT_READY',
        'perception': {'detected_now': True, 'confirmed': True,
                       'alignment_state': 'CENTERED'},
        'navigation': {'goal_active': False, 'goal_state': 'CANCELED'},
        'robot': {'odom_fresh': True, 'stationary': True},
        'health': {'status': 'OK', 'blockers': []},
        'decision': {'eligible_for_future_alignment_handoff': True,
                     'readiness_blockers': []},
        'safety': {'motion_command_publisher': False,
                   'actuator_command_publisher': False,
                   'collision_check_bypassed': False}}, 1.0)
    core.update('alignment', {
        'schema_version': 1, 'state': 'COMPLETE',
        'enabled': True, 'one_shot': True,
        'stop_reason': 'TARGET_CENTERED', 'publisher_owned': False,
        'goal_uuid': 'a' * 32,
        'command': {'angular_z_rad_s': 0.0},
        'safety': {'pump_command_publisher': False,
                   'linear_motion': False}}, 1.0)
    core.update('gateway', {
        'schema_version': 1, 'state': 'HANDOFF_READY', 'enabled': True,
        'fire_latched': True, 'ever_forwarded': True, 'released': True,
        'stop_reason': 'FIRE_ACQUIRED', 'handoff_ready': True,
        'raw_publishers': 0, 'nav_shadow_fingerprint_ok': True,
        'safety_gate_fingerprint_ok': True,
        'cmd_vel_fresh_zero': True, 'odom_stationary': True,
        'goal_active': False,
        'safety': {'raw_command_publisher': False}}, 1.0)
    core.update('guard', {
        'schema_version': 1, 'state': 'CANCEL_CONFIRMED_STOPPED',
        'enabled': True, 'goal_uuid': 'a' * 32, 'send_count': 1,
        'cancel_service_accepted': True, 'cancel_status_seen': True,
        'goal_status': 5, 'cmd_vel_fresh_zero': True,
        'odom_fresh_stationary': True, 'arm_fresh': True,
        'nav_shadow_fingerprint_ok': True, 'raw_cmd_publishers': 0,
        'blockers': [],
        'safety': {'motion_command_publisher': False,
                   'actuator_command_publisher': False}}, 1.0)
    core.update_statuses([('a' * 32, 5, 100, 0)], 1.0)
    core.update_arm(False, 0.9)
    core.update_arm(True, 1.0)
    core.update_odom(ZERO, 1.0)
    core.update_cmd(ZERO, 1.0)
    return core
