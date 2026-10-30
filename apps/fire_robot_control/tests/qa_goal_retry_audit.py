"""Read-only runtime audit: reproduce goal retry gaps on the shipped core.

No ROS init, DDS, Pi or actuators. XFAIL cases describe desired recovery,
not a fix or a claim that existing fault stops should be bypassed.
"""
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / 'releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission'
for folder in (PKG / 'test', PKG / 'test/r5_lifecycle', PKG / 'test/r5_delta'):
    sys.path.insert(0, str(folder))

from qa_app_operator_timing import app_class
from qa_full_operator import simulated_camera_and_no_ros_context
from test_navigation_no_fire_125512 import driving
from test_live_nav_stop_gateway import RAW_SOURCE, navigate, ready_core, step
from test_normal_goal_rearm import OLD, NEW, feed, finish_goal, idle_hold, propose


def test_operator_under_audit_matches_generated_package(tmp_path):
    from services.app_operator_source import patch_operator
    from services.candidate_build import build
    candidate = tmp_path/'candidate'
    build(ROOT/'releases/r5_20261029/ros', ROOT/'apps/fire_robot_control', candidate)
    generated = candidate/'desktop/generated/app_operational_session.py'
    source = PKG / 'fire_robot_mission/operational_session.py'
    assert generated.read_text() == patch_operator(source.read_text())


@pytest.mark.parametrize('status', [4, 5, 6])
def test_clean_terminal_can_run_next_goal(status):
    core = idle_hold(finish_goal(status))
    assert core.normal_terminal == status
    propose(core)
    core.update_statuses([(OLD, status), (NEW, 2)], 12.)
    assert step(core, 12.01)[2]['cycle_goal_uuid'] == NEW
    assert step(core, 12.02)[3] == 'CREATE'
    core.publisher_created(12.02)
    assert step(core, 12.03, owned=True, raw=RAW_SOURCE)[2]['state'] == 'NAVIGATION'


FAULTS = ('nav_gap', 'odom_gap', 'arm_gap', 'arm_false', 'mission_gap',
          'mission_health', 'nav_topology', 'safety_topology', 'status_source',
          'uuid_preempt', 'raw_conflict')


def fault_then_terminal(fault):
    core = ready_core()
    navigate(core)
    now = 10.20
    feed(core, now, active=True)
    kw = dict(owned=True, raw=RAW_SOURCE)
    if fault == 'nav_gap': core.nav_at = 9.0
    elif fault == 'odom_gap': core.odom_at = 9.0
    elif fault == 'arm_gap': core.arm_at = 9.0
    elif fault == 'arm_false': core.update_arm(False, now)
    elif fault == 'mission_gap': core.mission_at = 9.0
    elif fault == 'mission_health': core.mission['health']['status'] = 'BLOCKED'
    elif fault == 'nav_topology': kw['nav'] = []
    elif fault == 'safety_topology': kw['safety'] = []
    elif fault == 'status_source': kw['status'] = 0
    elif fault == 'uuid_preempt': core.update_statuses([(OLD, 5), (NEW, 2)], now)
    elif fault == 'raw_conflict': kw['raw'] = RAW_SOURCE + RAW_SOURCE
    linear, angular, report, action = step(core, now + .01, **kw)
    assert (linear, angular) == (0., 0.)
    assert core.stop_latched
    assert core.normal_terminal is None
    assert report['state'] == 'STOPPING'
    # Restore inputs, then receive a late exact cancel result. This must not
    # retrospectively convert a fault STOP into a clean terminal transition.
    feed(core, 10.30)
    core.update_statuses([(OLD, 5)], 10.30)
    step(core, 10.31, owned=True, raw=RAW_SOURCE)
    feed(core, 10.95)
    assert step(core, 10.95, owned=True, raw=RAW_SOURCE)[3] == 'RELEASE'
    core.publisher_released()
    for index in range(13):
        now = 11. + index * .1
        feed(core, now)
        step(core, now)
    return core


@pytest.mark.parametrize('fault', FAULTS)
def test_fault_stops_safely_but_late_cancel_does_not_rearm(fault):
    core = fault_then_terminal(fault)
    assert core.normal_terminal is None and core.normal_ready_at is None
    propose(core, 12.3)
    linear, angular, report, action = step(core, 12.31)
    assert (linear, angular, action) == (0., 0., None)
    assert report['state'] == 'RELEASED_STOPPED'
    assert report['cycle_goal_uuid'] == OLD


@pytest.mark.parametrize('terminal', [4, 5, 6])
def test_bare_core_waits_when_new_goal_arrives_before_idle_hold(terminal):
    core = finish_goal(terminal)
    for i in range(31):
        now = 10.9 + i * .1
        feed(core, now, active=True)
        core.update_statuses([(OLD, terminal), (NEW, 2)], now)
        step(core, now)
    assert core.goal_uuid == OLD and core.released


def ui_goal_permission(report):
    """The current native permission predicate, excluding wall-time freshness."""
    return (report.get('schema_version') == 1
            and report.get('scope') == 'R5_SESSION_INTENT_ONLY'
            and report.get('armed') is True and report.get('closed') is False
            and report.get('automatic', {}).get('enabled') is True
            and report.get('app_goal_admission', {}).get('allowed') is True
            and report.get('fault') is None
            and not any(report.get(k) for k in ('input_hold', 'identity_hold', 'graph_hold'))
            and not report.get('runtime_rebind', {}).get('held')
            and report.get('cycle') is None
            and report.get('phase') in ('ARMED_IDLE', 'NAVIGATION'))


def gap_robot(monkeypatch, tmp_path):
    h = driving(monkeypatch, tmp_path)
    held = [True]
    h.bus.drop = lambda p, s, m: (held[0] and p.topic == '/fire_mission/nav2_cmd_vel_shadow'
                                and s.info.node_name == 'live_nav_stop_gateway')
    h.until(lambda r: h.by_name['live_nav_stop_gateway'].core.released)
    assert h.by_name['live_nav_stop_gateway'].core.stop_reason == 'NAV_INPUT_OR_TOPOLOGY_LOST'
    held[0] = False
    for _ in range(10): h.step()
    return h


def node_gap_then_x(monkeypatch, tmp_path, *, preempt=False, gap=True,
                    terminal=6, evict=False):
    h = gap_robot(monkeypatch, tmp_path) if gap else driving(monkeypatch, tmp_path)
    if preempt:
        h.status = [('a' * 32, terminal), ('34' * 16, 2)]
        h.until(lambda r: h.by_name['live_nav_stop_gateway'].core.released)
        if evict:
            h.status = [('34' * 16, 2)]
            for _ in range(5): h.step()
    result = h.step('X')
    assert result['operator_feedback']['accepted']
    h.until(lambda r: r['phase'] == 'ARMED_IDLE' and not r['navigation_cancel_uuid'])
    for _ in range(35): result = h.step()
    return h, result


@pytest.mark.parametrize('gap', [False, True])
@pytest.mark.parametrize('terminal', [4, 5, 6])
@pytest.mark.parametrize('evict', [False, True])
def test_real_app_X_after_preemption_releases_old_fault_and_accepts_next_goal(monkeypatch, tmp_path, app_class,
                                                                 gap, terminal, evict):
    h, result = node_gap_then_x(monkeypatch, tmp_path, preempt=True, gap=gap,
                               terminal=terminal, evict=evict)
    core = h.by_name['live_nav_stop_gateway'].core
    assert h.bus.cancelled and h.status == ([] if evict else [('a' * 32, terminal)]) + [('34' * 16, 5)]
    assert {uid for _, uid in h.bus.cancelled} == {'34' * 16}
    assert core.normal_terminal == terminal
    assert not result['closed'] and result['fault'] is None
    assert ui_goal_permission(result)
    x = h.x
    h.status.append(('56' * 16, 2))
    for _ in range(50): result = h.step()
    assert result['phase'] == 'NAVIGATION' and ui_goal_permission(result)
    assert not core.released and core.goal_uuid == '56' * 16
    assert h.x > x and not h.pulses
    assert any(p.topic == '/cmd_vel_raw' for p in h.bus.pubs)
    # Retry failure does not imply Q/cleanup is broken.
    h.step('Q')
    result = h.until(lambda r: r.get('shutdown', {}).get('done'))
    assert result['shutdown']['confirmed']


def test_desired_real_app_X_after_preemption_then_next_goal_moves(monkeypatch, tmp_path, app_class):
    h, result = node_gap_then_x(monkeypatch, tmp_path, preempt=True)
    x = h.x
    h.status.append(('56' * 16, 2))
    for _ in range(50): h.step()
    assert h.x > x + .01


def test_real_app_nav_gap_X_same_goal_then_next_goal_runs(monkeypatch, tmp_path, app_class):
    h, result = node_gap_then_x(monkeypatch, tmp_path)
    assert h.by_name['live_nav_stop_gateway'].core.stop_reason == 'NAV_GOAL_CANCELED'
    x = h.x
    h.status.append(('56' * 16, 2))
    h.until(lambda r: h.x > x + .01)
    assert not h.pulses and not h.op.core.closed


@pytest.mark.parametrize('terminal', [4, 5, 6])
def test_real_app_fault_goal_ends_before_X_recovers(monkeypatch, tmp_path, app_class, terminal):
    h = gap_robot(monkeypatch, tmp_path)
    h.status = [('a' * 32, terminal)]
    for _ in range(15): h.step()
    result = h.step('X')
    assert result['operator_feedback']['accepted']
    assert result['operator_feedback']['reason'] == 'NO_ACTIVE_NAV_GOAL'
    for _ in range(35): result = h.step()
    assert not h.bus.cancelled
    assert h.by_name['live_nav_stop_gateway'].core.normal_terminal == terminal
    x = h.x
    h.status.append(('56' * 16, 2))
    for _ in range(50): result = h.step()
    assert h.x > x and result['phase'] == 'NAVIGATION' and ui_goal_permission(result)
    assert not h.pulses


@pytest.mark.parametrize('terminal', [4, 5, 6])
def test_real_app_early_new_goal_waits_for_STOP_hold_then_runs(monkeypatch, tmp_path, app_class, terminal):
    h = driving(monkeypatch, tmp_path)
    h.status = [('a' * 32, terminal)]
    h.until(lambda r: h.by_name['live_nav_stop_gateway'].core.released)
    core = h.by_name['live_nav_stop_gateway'].core
    assert core.normal_terminal == terminal and core.normal_ready_at is None
    h.status.append(('34' * 16, 2))
    x = h.x
    for _ in range(50): result = h.step()
    assert h.x > x and result['phase'] == 'NAVIGATION' and ui_goal_permission(result)
    assert h.step('X')['operator_feedback']['accepted']
    h.until(lambda r: r['reason'] == 'NAV2_READY_NEW_GOAL')
    h.status.append(('56' * 16, 2))
    h.until(lambda r: h.x > x + .01)
    assert not h.pulses


def test_real_app_healthy_X_then_next_goal_runs(monkeypatch, tmp_path, app_class):
    h = driving(monkeypatch, tmp_path)
    old = h.status[-1][0]
    assert h.step('X')['operator_feedback']['accepted']
    h.until(lambda r: r['reason'] == 'NAV2_READY_NEW_GOAL')
    assert sum(uid == old for at, uid in h.bus.cancelled) == 1
    x = h.x
    h.status.append(('56' * 16, 2))
    h.until(lambda r: h.x > x + .01)
    assert not h.pulses and not h.op.core.closed
