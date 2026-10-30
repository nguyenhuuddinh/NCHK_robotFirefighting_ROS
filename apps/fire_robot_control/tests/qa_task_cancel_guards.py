"""Negative admissions for the app's new retirement-only cancel path."""
import copy

import pytest

from qa_app_operator_timing import app_class
from qa_full_operator import simulated_camera_and_no_ros_context
from qa_x_cycle_audit import at_stage, wait


def waiting(monkeypatch, tmp_path):
    h = at_stage(monkeypatch, tmp_path, 'nav', True)
    guard = h.by_name['nav2_cancel_guard']
    monkeypatch.setattr(guard.cancel_client, 'service_is_ready', lambda: False)
    h.fire = True
    wait(h, lambda r: r['cycle'] is not None)
    h.fire = False
    h.step('X')
    wait(h, lambda r: not any(p.topic in ('/cmd_vel_raw', '/pump_cmd') for p in h.bus.pubs))
    for _ in range(15): h.step()
    ep = guard._cycle
    guard.core.service_ready = True
    assert ep._retirement_goal(h.bus.now) == 'a'*32
    assert guard.core._preflight(h.bus.now) == ([], 'a'*32)
    return h, ep


BAD = ['missing_tag', 'schema_bool', 'request_id', 'goal', 'cycle', 'after_pulse',
       'closed', 'unarmed', 'session_fault', 'session_stale', 'session_future',
       'input_hold', 'graph_hold', 'identity_hold', 'operator_source', 'status_source',
       'identity_epoch', 'identity_stale', 'graph', 'raw_owner', 'pump_owner',
       'cmd_nonzero', 'odom_nonzero', 'cmd_stale', 'odom_stale',
       'actuator_stale', 'pump_on', 'servo_on', 'actuator_locked',
       'duplicate_uuid', 'wrong_active', 'status_future', 'control_stale']


@pytest.mark.parametrize('bad', BAD)
def test_retirement_cancel_never_bypasses_independent_proofs(monkeypatch, tmp_path, app_class, bad):
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from harness import BE, REL
    h, ep = waiting(monkeypatch, tmp_path)
    p = ep.participant
    now = h.bus.now
    session, received = p.inputs['operator_session']
    session = copy.deepcopy(session)
    p.inputs['operator_session'] = (session, received)
    tag = session['app_retirement_cancel']
    if bad == 'missing_tag': session['app_retirement_cancel'] = None
    elif bad == 'schema_bool': tag['schema_version'] = True
    elif bad == 'request_id': tag['request_id'] = 'invalid'
    elif bad == 'goal': tag['goal_uuid'] = '56'*16
    elif bad == 'cycle': tag['cycle_id'] = '56'*16
    elif bad == 'after_pulse': tag['before_pulse'] = False
    elif bad == 'closed': session['closed'] = True
    elif bad == 'unarmed': session['armed'] = False
    elif bad == 'session_fault': session['fault'] = 'HARD_FAULT'
    elif bad == 'session_stale': p.inputs['operator_session'] = (session, now-1.)
    elif bad == 'session_future': p.inputs['operator_session'] = (session, now+1.)
    elif bad in ('input_hold', 'graph_hold', 'identity_hold'): session[bad] = True
    elif bad in ('operator_source', 'status_source'):
        topic = '/fire_mission/operator_state' if bad == 'operator_source' else '/navigate_to_pose/_action/status'
        pub = next(x for x in h.bus.pubs if x.topic == topic)
        pub.info.endpoint_gid = list((999).to_bytes(24, 'big'))
    elif bad == 'identity_epoch': p.inputs['identity'][0]['connection_epoch'] += 1
    elif bad == 'identity_stale': p.inputs['identity'] = (p.inputs['identity'][0], now-1.)
    elif bad == 'graph': ep.graph_ok = False
    elif bad == 'raw_owner': h.bus.publisher('foreign_driver', Twist, '/cmd_vel_raw', BE)
    elif bad == 'pump_owner': h.bus.publisher('foreign_pump', Bool, '/pump_cmd', REL)
    elif bad in ('cmd_nonzero', 'odom_nonzero'):
        key = bad.split('_')[0]; p.inputs[key] = ((.2, 0., 0., 0., 0., 0.), now)
    elif bad in ('cmd_stale', 'odom_stale'):
        key = bad.split('_')[0]; p.inputs[key] = (p.inputs[key][0], now-1.)
    elif bad == 'actuator_stale': p.inputs['actuator'] = (p.inputs['actuator'][0], now-1.)
    elif bad == 'pump_on': p.inputs['actuator'][0]['pump_pwm_commanded_on'] = True
    elif bad == 'servo_on': p.inputs['actuator'][0]['servo_pwm_commanded_on'] = True
    elif bad == 'actuator_locked': p.inputs['actuator'][0]['servo_phase'] = 'LOCKED'
    elif bad == 'duplicate_uuid': p.inputs['status'] = ([('a'*32, 2), ('a'*32, 2)], now)
    elif bad == 'wrong_active': p.inputs['status'] = ([('56'*16, 2)], now)
    elif bad == 'status_future': p.inputs['status'] = (p.inputs['status'][0], now+1.)
    elif bad == 'control_stale': p.request_at = now-1.
    assert ep._retirement_goal(now) is None, bad
    assert ep.node.core._preflight(now)[0], bad
    assert not h.bus.cancelled and not h.pulses


@pytest.mark.parametrize('bad', ['valid', 'after_pulse', 'missing_policy', 'old_active',
                               'old_missing', 'duplicate_old', 'control_stale', 'epoch',
                               'operator_source', 'raw_owner', 'pump_owner', 'actuator_stale'])
def test_foreign_cancel_requires_typed_pre_pulse_terminal_and_STOP(monkeypatch, tmp_path, app_class, bad):
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from harness import BE, REL
    h, ep = waiting(monkeypatch, tmp_path)
    p = ep.participant
    now = h.bus.now
    p.inputs['status'] = ([('a'*32, 4), ('34'*16, 2)], now)
    p.request = dict(p.request, phase='PREPARE', after_pulse=False, terminal_goal_release=True)
    p.request_at = now
    monkeypatch.setattr(ep.node.cancel_client, 'service_is_ready', lambda: True)
    if bad == 'after_pulse': p.request['after_pulse'] = True
    elif bad == 'missing_policy': p.request['terminal_goal_release'] = False
    elif bad == 'old_active': p.inputs['status'] = ([('a'*32, 2), ('34'*16, 2)], now)
    elif bad == 'old_missing': p.inputs['status'] = ([('34'*16, 2)], now)
    elif bad == 'duplicate_old': p.inputs['status'] = ([('a'*32, 4), ('a'*32, 4), ('34'*16, 2)], now)
    elif bad == 'control_stale': p.request_at = now-1.
    elif bad == 'epoch': p.inputs['identity'][0]['connection_epoch'] += 1
    elif bad == 'operator_source':
        pub = next(x for x in h.bus.pubs if x.topic == '/fire_mission/operator_state')
        pub.info.endpoint_gid = list((999).to_bytes(24, 'big'))
    elif bad == 'raw_owner': h.bus.publisher('foreign_driver', Twist, '/cmd_vel_raw', BE)
    elif bad == 'pump_owner': h.bus.publisher('foreign_pump', Bool, '/pump_cmd', REL)
    elif bad == 'actuator_stale': p.inputs['actuator'] = (p.inputs['actuator'][0], now-1.)
    ep.blocked_goals.step(ep, now)
    assert [uid for at, uid in h.bus.cancelled] == (['34'*16] if bad == 'valid' else [])
    assert not h.pulses and not any(pub.topic == '/cmd_vel_raw' and pub.info.node_name != 'foreign_driver' for pub in h.bus.pubs)


def test_gateway_recovery_requires_continuous_STOP_samples(monkeypatch, app_class):
    import test_live_nav_stop_gateway as fixture
    from test_normal_goal_rearm import finish_goal, feed, OLD, NEW
    from services.app_task_recovery import AppNavCore
    monkeypatch.setattr(fixture, 'LiveNavStopCore', AppNavCore)
    core = finish_goal(4)
    core.app_session_current = True
    core.app_operator_session = dict(cycle=None, navigation_cancel_uuid=None)
    def fresh(at):
        feed(core, at, active=True)
        core.update_statuses([(OLD, 4), (NEW, 2)], at)
        return fixture.step(core, at+.001)
    for i in range(9):
        assert fresh(10.9+i*.1)[2]['cycle_goal_uuid'] == OLD
    # Fresh inputs after a 2-second scheduling gap cannot reuse old idle proof.
    assert fresh(13.9)[2]['cycle_goal_uuid'] == OLD
    for i in range(1, 10):
        assert fresh(13.9+i*.1)[2]['cycle_goal_uuid'] == OLD
    assert fresh(15.01)[2]['cycle_goal_uuid'] == NEW


@pytest.mark.parametrize('bad', ['no_session', 'cycle_active', 'cancel_active', 'status_future',
                               'contradict_terminal', 'moving', 'pump_on', 'arm_lost'])
def test_gateway_new_goal_cannot_use_ready_proof_after_contradiction(monkeypatch, app_class, bad):
    import test_live_nav_stop_gateway as fixture
    from test_normal_goal_rearm import finish_goal, idle_hold, propose, OLD, NEW
    from services.app_task_recovery import AppNavCore
    monkeypatch.setattr(fixture, 'LiveNavStopCore', AppNavCore)
    core = finish_goal(4)
    core.app_session_current = True
    core.app_operator_session = dict(cycle=None, navigation_cancel_uuid=None)
    idle_hold(core)
    assert core.normal_ready_at is not None
    propose(core)
    if bad == 'no_session': core.app_session_current = False
    elif bad == 'cycle_active': core.app_operator_session['cycle'] = dict(cycle_id='56'*16)
    elif bad == 'cancel_active': core.app_operator_session['navigation_cancel_uuid'] = NEW
    elif bad == 'status_future': core.status_at = 13.
    elif bad == 'contradict_terminal': core.update_statuses([(OLD, 2), (NEW, 2)], 12.)
    elif bad == 'moving': core.update_odom(fixture.MOVING, 12.)
    elif bad == 'pump_on': core.actuator['pump_pwm_commanded_on'] = True
    elif bad == 'arm_lost': core.update_arm(False, 12.)
    result = fixture.step(core, 12.01)
    assert result[:2] == (0., 0.) and result[3] != 'CREATE'
    assert core.goal_uuid == OLD
