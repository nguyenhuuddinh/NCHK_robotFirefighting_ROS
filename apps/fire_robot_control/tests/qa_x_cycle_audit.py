"""Actual app operator, real mission nodes, in-memory transport only.

Audits completion of X through STOP/OFF/HOME, retirement and next Goal
forwarding. No ROS init, Pi, DDS or physical motor/pump.
"""
import copy
import json
import os
from pathlib import Path

import pytest

from qa_goal_retry_audit import PKG, ui_goal_permission
from qa_app_operator_timing import app_class
from qa_full_operator import simulated_camera_and_no_ros_context
from test_ir_freshness_hold_115816 import robot as automatic_robot


def wait(h, predicate, limit=400):
    for _ in range(limit):
        report = h.step()
        if predicate(report):
            return report
        assert report['fault'] is None, compact(h, report)
        assert not report['closed'], compact(h, report)
    raise AssertionError(compact(h, report))


def compact(h, report):
    gateway = h.by_name['live_nav_stop_gateway'].core
    guard = h.bus.report('/fire_mission/nav2_cancel_guard_state')
    return dict(phase=report.get('phase'), reason=report.get('reason'), fault=report.get('fault'),
        cycle=report.get('cycle'), input_hold=report.get('input_hold'), identity_hold=report.get('identity_hold'),
        graph_hold=report.get('graph_hold'), feedback=report.get('operator_feedback'),
        cancel_guard={key: guard.get(key) for key in ('state', 'goal_uuid', 'blockers',
            'send_count', 'cancel_service_accepted', 'cancel_status_seen')},
        handback=report.get('handback'), gateway=dict(state=h.bus.report('/fire_mission/live_nav_stop_state').get('state'),
            goal=gateway.goal_uuid, reason=gateway.stop_reason, normal_terminal=gateway.normal_terminal),
        participants={name:dict(phase=node._cycle.participant.phase, error=node._cycle.participant.error)
            for name, node in h.by_name.items() if hasattr(node, '_cycle')},
        raw_owners=[p.info.node_name for p in h.bus.pubs if p.topic == '/cmd_vel_raw'],
        pump_owners=[p.info.node_name for p in h.bus.pubs if p.topic == '/pump_cmd'],
        cancel_uuids=[uid for at, uid in h.bus.cancelled], x=h.x, pulses=len(h.pulses), home=h.home)


def evidence(name, row):
    out = os.environ.get('FIRE_ROBOT_QA_X_AUDIT')
    if out:
        (Path(out)/f'{name}.json').write_text(json.dumps(row, indent=2)+'\n')


def verify_Q(h):
    h.step('Q')
    for _ in range(120):
        report = h.step()
        if report.get('shutdown', {}).get('done'):
            return dict(done=True, confirmed=report['shutdown']['confirmed'],
                        reason=report['shutdown']['reason'])
    return dict(done=False, confirmed=False)


def at_stage(monkeypatch, tmp_path, stage, goal):
    h = automatic_robot(monkeypatch, tmp_path, goal=goal)
    for pub in h.nav:
        original = pub.publish
        def publish(message, send=original):
            message = copy.deepcopy(message)
            if any(status in (1, 2, 3) for uid, status in h.status):
                message.linear.x = .06
            send(message)
        monkeypatch.setattr(pub, 'publish', publish)
    if stage == 'idle':
        return h
    if stage == 'nav':
        wait(h, lambda r: h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
        return h
    h.fire = True
    wait(h, lambda r: r['cycle'] is not None)
    if stage == 'detected':
        return h
    wait(h, lambda r: abs(h.yaw) > .025)
    if stage == 'turn':
        return h
    h.centered = True
    x = h.x
    wait(h, lambda r: h.x > x + .025)
    if stage == 'ir':
        return h
    h.right = 950
    if stage == 'confirm':
        wait(h, lambda r: len(h.op.core.confirm_samples) > 0 and not h.op.core.p_used)
        return h
    wait(h, lambda r: h.by_name['fire_pulse_controller'].pub is not None)
    if stage == 'pump_registered':
        return h
    wait(h, lambda r: h.on_at is not None)
    if stage == 'pump':
        return h
    wait(h, lambda r: h.off_at is not None)
    if stage == 'home':
        return h
    wait(h, lambda r: h.home >= 1)
    return h


STAGES = ('idle', 'nav', 'detected', 'turn', 'ir', 'confirm',
          'pump_registered', 'pump', 'home', 'post_pulse')


@pytest.mark.parametrize('stage', STAGES)
@pytest.mark.parametrize('goal', [False, True])
@pytest.mark.parametrize('fire_after_x', [False, True])
def test_X_finishes_cycle_and_actual_next_goal_forwards(monkeypatch, tmp_path, app_class,
                                                       stage, goal, fire_after_x):
    if stage == 'nav' and not goal:
        pytest.skip('No active navigation when fixture starts without Goal')
    h = at_stage(monkeypatch, tmp_path, stage, goal)
    h.fire = fire_after_x
    h.ir_flame = fire_after_x
    at = h.bus.now
    pulses_before = len(h.pulses)
    first = h.step('X')
    assert first['operator_feedback']['accepted'], compact(h, first)
    # A double-click must not start a new cycle, duplicate an ON, extend a
    # budget, or close the session while the previous X drains.
    second = h.step('X')
    recovered = False
    for _ in range(500):
        report = h.step()
        if (report['cycle'] is None and report['phase'] == 'ARMED_IDLE'
                and h.op.core._rest(h.bus.now)):
            recovered = True
            break
        if report['fault'] or report['closed']:
            break
    if not recovered:
        failure = dict(stage=stage, original_goal=goal, fire_after_x=fire_after_x,
                       at_x=compact(h, first), last=compact(h, report), recovered=False)
        failure['Q_after_wait'] = verify_Q(h)
        evidence(f'{stage}_{goal}_{fire_after_x}', failure)
        pytest.fail(str(failure))
    # Wait for explicit post-stop admission rather than merely ARMED_IDLE.
    for _ in range(30):
        report = h.step()
    before_goal = compact(h, report)
    assert ui_goal_permission(report), before_goal
    assert len(h.pulses) == pulses_before, before_goal
    assert not h.pump_requested, before_goal
    old_terminal = [(uid, status) for uid, status in h.status if status in (4, 5, 6)]
    h.status = old_terminal + [('56' * 16, 2)]
    forwarded = False
    for _ in range(100):
        report = h.step()
        raw = h.bus.last.get('/cmd_vel_raw')
        if (h.by_name['live_nav_stop_gateway'].core.goal_uuid == '56' * 16
                and any(p.topic == '/cmd_vel_raw' and p.info.node_name == 'live_nav_stop_gateway' for p in h.bus.pubs)
                and raw is not None and raw.linear.x > 0):
            forwarded = True
            break
    record = dict(stage=stage, original_goal=goal, fire_after_x=fire_after_x,
        wait_s=round(h.bus.now-at, 3), pulses_before=pulses_before, before_goal=before_goal,
        after_goal=compact(h, report), next_goal_forwarded=forwarded,
        scope='REAL_NODES_IN_MEMORY_NO_PHYSICAL_ACCEPTANCE')
    evidence(f'{stage}_{goal}_{fire_after_x}', record)
    assert forwarded, record
    assert not report['closed'] and report['fault'] is None
    assert len(h.pulses) == pulses_before and h.bus.max_raw <= 1


@pytest.mark.parametrize('warm_navigation', [False, True])
@pytest.mark.parametrize('clear_delay', [-1, 0, 1, 3])
@pytest.mark.parametrize('action', ['X', 'target_lost'])
def test_early_fire_cancel_boundary_audit(monkeypatch, tmp_path, app_class,
                                        warm_navigation, clear_delay, action):
    stage = 'nav' if warm_navigation else 'idle'
    h = at_stage(monkeypatch, tmp_path, stage, True)
    h.fire = True
    wait(h, lambda r: r['cycle'] is not None)
    before = compact(h, h.reports[-1])
    for _ in range(max(clear_delay, 0)):
        h.step()
    if clear_delay == -1:
        h.fire = False
        h.step()
    h.fire = False
    h.ir_flame = False
    report = h.step('X' if action == 'X' else None)
    for _ in range(320):
        report = h.step()
        if report['cycle'] is None or report['fault'] or report['closed']:
            break
    row = dict(warm_navigation=warm_navigation, clear_delay=clear_delay, action=action,
               before=before, after=compact(h, report), recovered=report['cycle'] is None)
    if not row['recovered']:
        row['Q_after_wait'] = verify_Q(h)
    evidence(f'early_{warm_navigation}_{clear_delay}_{action}', row)
    assert row['recovered'], row
    assert not report['closed'] and report['fault'] is None
    for _ in range(30): h.step()
    h.status = [(uid, status) for uid, status in h.status if status in (4, 5, 6)] + [('56'*16, 2)]
    wait(h, lambda r: h.by_name['live_nav_stop_gateway'].core.goal_uuid == '56'*16
         and any(p.topic == '/cmd_vel_raw' for p in h.bus.pubs))


@pytest.mark.parametrize('goal', [False, True])
@pytest.mark.parametrize('presses', [1, 2, 6])
def test_X_at_navigation_boundary_does_not_start_unrequested_fire_cycle(monkeypatch, tmp_path,
                                                                     app_class, goal, presses):
    h = at_stage(monkeypatch, tmp_path, 'nav' if goal else 'idle', goal)
    h.fire = True
    at = h.bus.now
    for _ in range(presses): h.step('X')
    for _ in range(130): report = h.step()
    row = dict(original_goal=goal, presses=presses, after=compact(h, report),
               accepted_motion_after_x=any(t > at+.8 and topic == '/cmd_vel_raw' and abs(msg.angular.z) > 0
                   for t, owner, topic, msg in h.bus.sent),
               Q_after_wait=verify_Q(h))
    evidence(f'fire_boundary_{goal}_{presses}', row)
    assert row['after']['cycle'] is None, row
    assert not row['accepted_motion_after_x'], row
