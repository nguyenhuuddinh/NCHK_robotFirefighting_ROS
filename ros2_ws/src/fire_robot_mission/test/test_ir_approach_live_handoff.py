"""Check the live IR node's exact post-alignment admission contract."""

from copy import deepcopy
from types import SimpleNamespace

from fire_robot_mission.ir_approach_live import IrApproachLiveNode


UUID = 'ab' * 16


def ready():
    node = object.__new__(IrApproachLiveNode)
    ir = {'zone': 'APPROACH', 'ir_band': 'APPROACH', 'blockers': []}
    node.core = SimpleNamespace(
        input_timeout=0.35, continuous_approach=False,
        _input=lambda name, now: ir if name == 'ir' else None)
    payloads = {
        'mission': {
            'schema_version': 1, 'observe_only': True,
            'state': 'ALIGNMENT_READY',
            'perception': {'confirmed': True,
                           'alignment_state': 'CENTERED'},
            'navigation': {'goal_active': False,
                           'goal_state': 'CANCELED'},
            'robot': {'odom_fresh': True},
            'health': {'status': 'OK'},
            'safety': {'motion_command_publisher': False,
                       'actuator_command_publisher': False},
        },
        'alignment': {
            'schema_version': 1, 'state': 'COMPLETE',
            'goal_uuid': UUID, 'stop_reason': 'TARGET_CENTERED',
            'publisher_owned': False,
            'command': {'angular_z_rad_s': 0.0},
        },
        'gateway': {
            'schema_version': 1, 'state': 'HANDOFF_READY',
            'cycle_goal_uuid': UUID, 'released': True,
            'fire_latched': True, 'ever_forwarded': True,
            'stop_reason': 'FIRE_ACQUIRED', 'handoff_ready': True,
            'raw_publishers': 0, 'cmd_vel_fresh_zero': True,
            'odom_stationary': True,
        },
        'guard': {
            'schema_version': 1, 'state': 'CANCEL_CONFIRMED_STOPPED',
            'goal_uuid': UUID, 'enabled': True, 'send_count': 1,
            'cancel_service_accepted': True,
            'cancel_status_seen': True, 'goal_status': 5,
            'arm_fresh': True, 'nav_shadow_fingerprint_ok': True,
            'raw_cmd_publishers': 0, 'blockers': [],
        },
    }
    node._proofs = {key: (payload, 1.0)
                    for key, payload in payloads.items()}
    node._statuses = ((UUID, 5),)
    counts = {key + '_source_ok': True for key in (
        'mission', 'alignment', 'gateway', 'guard', 'status')}
    counts['nav_shadow_ok'] = True
    return node, counts, ir


def test_exact_uuid_admits_ir_after_centered_cancel():
    node, counts, _ = ready()
    assert node._handoff_evidence(1.1, counts) == (UUID, True, True, False)


def test_latched_exact_cancel_admits_after_status_array_prunes_old_goal():
    node, counts, _ = ready()
    node._statuses = ()
    assert node._handoff_evidence(1.1, counts) == (UUID, True, True, False)
    node._statuses = ((UUID, 6),)
    assert node._handoff_evidence(1.1, counts) == (
        None, False, False, False)


def test_spray_range_after_recenter_has_stopped_proof_without_motion_admission():
    node, counts, ir = ready()
    ir['zone'] = ir['ir_band'] = 'SPRAY_RANGE'
    assert node._handoff_evidence(1.1, counts) == (UUID, False, True, True)


def test_runtime_accepts_only_same_cancel_proof_while_ir_owns_raw():
    node, counts, _ = ready()
    node._proofs['mission'][0]['state'] = 'FIRE_TRACKING'
    node._proofs['gateway'][0].update(
        state='RELEASED_STOPPED', handoff_ready=False)
    node._proofs['guard'][0].update(
        state='IR_APPROACH_IN_PROGRESS', raw_cmd_publishers=1)
    assert node._handoff_evidence(1.1, counts) == (UUID, False, True, False)


def test_wrong_uuid_new_goal_or_stale_proof_blocks():
    for change in ('wrong_uuid', 'new_goal', 'stale'):
        node, counts, ir = ready()
        if change == 'wrong_uuid':
            node._proofs['alignment'][0]['goal_uuid'] = 'cd' * 16
        elif change == 'new_goal':
            node._statuses = ((UUID, 5), ('cd' * 16, 2))
        elif change == 'stale':
            payload = deepcopy(node._proofs['guard'][0])
            node._proofs['guard'] = (payload, 0.0)
        assert node._handoff_evidence(1.1, counts) == (
            None, False, False, False)


def test_ir_stale_or_off_center_retains_uuid_but_never_motion_proof():
    for blockers in (['IR_TIMEOUT'], ['FIRE_NOT_CENTERED']):
        node, counts, ir = ready()
        ir['zone'] = 'UNKNOWN'
        ir['blockers'] = blockers
        assert node._handoff_evidence(1.1, counts) == (
            UUID, False, False, False)


def test_camera_candidate_during_ir_gap_retains_only_cancel_identity():
    node, counts, ir = ready()
    ir['zone'] = 'UNKNOWN'
    ir['blockers'] = ['IR_TIMEOUT']
    mission = node._proofs['mission'][0]
    mission['state'] = 'FIRE_CANDIDATE'
    mission['perception']['confirmed'] = False
    mission['perception']['alignment_state'] = 'NO_TARGET'
    assert node._handoff_evidence(1.1, counts) == (
        UUID, False, False, False)
    # An exact UUID is not permission to move, even if IR recovers first.
    ir['zone'] = ir['ir_band'] = 'APPROACH'
    ir['blockers'] = []
    assert node._handoff_evidence(1.1, counts) == (
        UUID, False, False, False)
