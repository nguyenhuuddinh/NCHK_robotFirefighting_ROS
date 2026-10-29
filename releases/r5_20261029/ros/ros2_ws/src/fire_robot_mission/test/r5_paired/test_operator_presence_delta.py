from fire_robot_mission.startup_health import HealthCore
from operational_entry import StartupSequence


def operator(reason='WAIT_STABLE_STANDBY', **changes):
    value = dict(schema_version=1, scope='R5_SESSION_INTENT_ONLY', phase='WAIT_ARM', reason=reason,
                 fault=None, closed=False, armed=False, physical_commands_published=False,
                 live_intents_published=False, motion_authorized=False,
                 startup_readiness=dict(checks={'ir_fresh': False}, blockers=['ir_fresh'], stable_s=0.))
    value.update(changes)
    return value


def health(value, *, source=True, now=10.):
    core = HealthCore('session', 'laptop')
    core.update('operator', value, now)
    report = core.report(now, sources=dict(operator=source), io_detail={})
    return report


def test_safe_waiting_operator_is_present_but_not_arm_ready():
    report = health(operator())
    assert report['checks']['operator_present'] is True
    assert report['checks']['operator_ready'] is False
    assert report['checks']['operator'] is False
    assert report['operator_diagnostics']['startup_readiness']['blockers'] == ['ir_fresh']


def test_ready_operator_preserves_strict_arm_readiness_check():
    report = health(operator('READY_PRESS_A'))
    assert report['checks']['operator_present'] is True
    assert report['checks']['operator_ready'] is True
    assert report['checks']['operator'] is True


def test_gate2_false_heartbeat_is_presence_not_active_live_intent():
    report = health(operator(live_intents_published=True, live_intents_active=False))
    assert report['checks']['operator_present'] is True
    assert report['checks']['operator_ready'] is False
    assert report['operator_diagnostics']['checks']['no_live_active'] is True


def test_fault_closed_armed_or_command_capability_never_counts_as_present():
    bad = (dict(fault='BAD_GRAPH'), dict(closed=True), dict(armed=True),
           dict(physical_commands_published=True),
           dict(live_intents_published=True, live_intents_active=True),
           dict(live_intents_published=True, live_intents_active=None),
           dict(motion_authorized=True), dict(phase='NAVIGATION'))
    for change in bad:
        assert health(operator(**change))['checks']['operator_present'] is False
    assert health(operator(), source=False)['checks']['operator_present'] is False
    assert health(operator('UNKNOWN_WAIT'))['checks']['operator_present'] is False


def test_consumer_startup_accepts_safe_presence_without_weakening_arm_gate():
    state = StartupSequence(0.)
    state.stage = 'CONSUMERS'; state.since = 0.; state.last = 0.
    observation = dict(io=True, camera=True, pose=True, yolo=True, operator_present=True)
    assert state.step(.6, observation) == []
    assert state.report() == dict(stage='READY_OBSERVE', done=True, error=None, retries=0, live=False)


def test_absent_operator_still_times_out_with_original_budget():
    state = StartupSequence(0.)
    state.stage = 'CONSUMERS'; state.since = 0.; state.last = 0.
    observation = dict(io=True, camera=True, pose=True, yolo=True, operator_present=False)
    assert state.step(StartupSequence.LIMITS['CONSUMERS'], observation) == ['STOP']
    assert state.error == 'STARTUP_TIMEOUT:CONSUMERS'
