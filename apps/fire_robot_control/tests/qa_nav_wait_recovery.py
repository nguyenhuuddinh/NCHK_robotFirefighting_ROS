"""Real app nodes on an in-memory bus: silent Nav2 Wait and safe same-Goal resume.

No ROS init, DDS, Pi, motor or pump. Action status remains event driven.
"""
import copy
import json
import os
from pathlib import Path

import pytest

from qa_goal_retry_audit import app_class, simulated_camera_and_no_ros_context
from test_ir_freshness_hold_115816 import robot
from test_live_nav_stop_gateway import ZERO, MOVING, RAW_SOURCE, navigate, step
from test_normal_goal_rearm import feed
from services.app_task_recovery import AppNavCore


def moving_robot(monkeypatch, tmp_path):
    h = robot(monkeypatch, tmp_path, goal=True)
    mode = ['move']
    for publisher in h.nav:
        send = publisher.publish
        def publish(message, original=send):
            if mode[0] == 'silence':
                return
            value = copy.deepcopy(message)
            value.linear.x = .06 if mode[0] == 'move' else 0.
            original(value)
        monkeypatch.setattr(publisher, 'publish', publish)
    h.until(lambda r: h.x > .03)
    return h, mode


def wait_silent(h, mode, ticks=40):
    mode[0] = 'zero'
    for _ in range(5): h.step()
    mode[0] = 'silence'
    for _ in range(ticks): report = h.step()
    gateway = h.by_name['live_nav_stop_gateway'].core
    assert gateway.released and gateway.stop_latched
    assert gateway.stop_reason == 'NAV_INPUT_OR_TOPOLOGY_LOST'
    assert not any(p.topic == '/cmd_vel_raw' for p in h.bus.pubs)
    assert report['app_navigation_recovery']['state'] == 'WAIT_COMMAND'
    assert not report['app_goal_admission']['allowed']
    assert not h.pulses
    return gateway


@pytest.mark.parametrize('ticks', [25, 100, 600])
def test_silent_wait_then_new_commands_resume_same_goal_without_restart(monkeypatch, tmp_path, app_class, ticks):
    h, mode = moving_robot(monkeypatch, tmp_path)
    gateway = wait_silent(h, mode, ticks)
    stopped_x = h.x
    nodes = tuple(h.nodes)
    mode[0] = 'move'
    for _ in range(12):
        report = h.step()
        assert h.x == stopped_x and not h.pulses
    assert report['app_navigation_recovery']['state'] == 'VERIFYING'
    h.until(lambda r: h.x > stopped_x + .01)
    assert gateway.goal_uuid == 'a'*32 and h.status == [('a'*32, 2)]
    assert gateway.app_nav_recovery['state'] == 'RESUMED'
    assert not gateway.used_goal_uuids and not h.bus.cancelled
    assert tuple(h.nodes) == nodes and h.bus.max_raw == 1
    assert not h.pulses and h.reports[-1]['fault'] is None
    assert not h.reports[-1].get('app_navigation_recovery')
    events = [dict(at=at, node=node, message=args[0]) for at, node, args in h.bus.logs
              if args and str(args[0]).startswith(('APP_NAV_STOP ', 'APP_NAV_RECOVERY '))]
    assert any('APP_NAV_STOP ' in event['message'] and 'NAV_COMMAND_MISSING_OR_INVALID' in event['message']
               for event in events)
    assert any('"state": "RESUMED"' in event['message'] for event in events)
    if os.environ.get('FIRE_ROBOT_QA_NAV_DIR'):
        (Path(os.environ['FIRE_ROBOT_QA_NAV_DIR'])/f'wait_{ticks}.json').write_text(json.dumps(dict(
            scope='IN_MEMORY_NOT_HARDWARE', silent_ticks=ticks, goal_uuid=gateway.goal_uuid,
            resumed_x=h.x, stopped_x=stopped_x, max_raw=h.bus.max_raw, pulses=len(h.pulses),
            action_cancel_count=len(h.bus.cancelled), events=events), indent=2)+'\n')


def test_repeated_waits_and_zero_keepalive_preserve_same_goal(monkeypatch, tmp_path, app_class):
    h, mode = moving_robot(monkeypatch, tmp_path)
    for _ in range(3):
        gateway = wait_silent(h, mode)
        stopped_x = h.x
        mode[0] = 'move'
        h.until(lambda r: h.x > stopped_x + .01)
        assert gateway.app_nav_recovery['state'] == 'RESUMED'
    mode[0] = 'zero'
    for _ in range(100): h.step()
    assert not gateway.stop_latched
    stopped_x = h.x
    mode[0] = 'move'
    h.until(lambda r: h.x > stopped_x + .01)
    assert not h.pulses and h.bus.max_raw == 1


@pytest.mark.parametrize('when', ['waiting', 'verifying', 'reacquiring', 'resumed'])
@pytest.mark.parametrize('key', ['X', 'Q'])
def test_X_Q_win_over_same_goal_recovery(monkeypatch, tmp_path, app_class, when, key):
    h, mode = moving_robot(monkeypatch, tmp_path)
    gateway = wait_silent(h, mode)
    if when != 'waiting':
        mode[0] = 'move'
        desired = {'verifying': 'VERIFYING', 'reacquiring': 'REACQUIRING', 'resumed': 'RESUMED'}[when]
        h.until(lambda r: gateway.app_nav_recovery['state'] == desired)
    h.step(key)
    mode[0] = 'move'
    if key == 'Q':
        report = h.until(lambda r: r.get('shutdown', {}).get('done'), limit=180)
        assert report['shutdown']['confirmed']
    else:
        h.until(lambda r: r['reason'] == 'NAV2_READY_NEW_GOAL', limit=180)
        stopped_x = h.x
        for _ in range(50): h.step()
        assert h.x == stopped_x
        h.status.append(('56'*16, 2))
        h.until(lambda r: h.x > stopped_x + .01)
        assert gateway.goal_uuid == '56'*16
    assert not h.pulses and h.bus.max_raw <= 1


def held_core():
    c = AppNavCore(enabled=True)
    c.app_recovery_binding = ('session', 0, 'identity', 0, 'pins')
    c.app_session_current = True
    c.app_recovery_off = True
    c.app_operator_session = dict(cycle=None, navigation_cancel_uuid=None)
    feed(c, 10., active=True)
    c.update_statuses([('ab'*16, 2)], 10.)
    navigate(c)
    # A valid velocity ages out while all other inputs remain current.
    for i in range(1, 31):
        now = 10.10+i*.05
        nav, nav_at = c.nav, c.nav_at
        feed(c, now, active=True)
        c.nav, c.nav_at = nav, nav_at
        result = step(c, now, owned=not c.released, raw=RAW_SOURCE if not c.released else [])
        if result[3] == 'RELEASE': c.publisher_released()
    assert c.app_nav_ticket and c.released
    return c, now


NEGATIVES = ('binding', 'cancel_epoch', 'goal_changed', 'duplicate_goal', 'terminal4', 'terminal5', 'terminal6',
    'disabled', 'fire', 'mission_gap', 'mission_health', 'mission_contract', 'odom_gap', 'odom_moving',
    'cmd_gap', 'cmd_nonzero', 'arm_false', 'arm_gap', 'actuator_gap', 'actuator_contract', 'pump_on', 'servo_on',
    'nav_nan', 'nav_out_of_bounds', 'raw_conflict', 'nav_topology', 'safety_topology',
    'status_source', 'mission_source', 'odom_source', 'arm_source', 'guard_source')


def inject(c, now, kind):
    kw = {}
    if kind == 'binding': c.app_recovery_binding = None
    elif kind == 'cancel_epoch': c.app_recovery_binding = ('session', 1, 'identity', 0, 'pins')
    elif kind == 'goal_changed': c.update_statuses([('ab'*16, 5), ('cd'*16, 2)], now)
    elif kind == 'duplicate_goal': c.update_statuses([('ab'*16, 2), ('ab'*16, 2)], now)
    elif kind.startswith('terminal'): c.update_statuses([('ab'*16, int(kind[-1]))], now)
    elif kind == 'disabled': c.enabled = False
    elif kind == 'fire': c.fire_latched = True
    elif kind == 'mission_gap': c.mission_at = now-1
    elif kind == 'mission_health': c.mission['health']['status'] = 'BLOCKED'
    elif kind == 'mission_contract': c.mission['safety']['collision_check_bypassed'] = True
    elif kind == 'odom_gap': c.odom_at = now-1
    elif kind == 'odom_moving': c.update_odom(MOVING, now)
    elif kind == 'cmd_gap': c.cmd_at = now-1
    elif kind == 'cmd_nonzero': c.update_cmd(MOVING, now)
    elif kind == 'arm_false': c.update_arm(False, now)
    elif kind == 'arm_gap': c.arm_at = now-1
    elif kind == 'actuator_gap': c.actuator_at = now-1
    elif kind == 'actuator_contract': c.app_recovery_off = False
    elif kind == 'pump_on': c.actuator['pump_pwm_commanded_on'] = True
    elif kind == 'servo_on': c.actuator['servo_pwm_commanded_on'] = True
    elif kind == 'nav_nan': c.update_nav((float('nan'), 0., 0., 0., 0., 0.), now)
    elif kind == 'nav_out_of_bounds': c.update_nav((.11, 0., 0., 0., 0., 0.), now)
    elif kind == 'raw_conflict': kw['raw'] = RAW_SOURCE
    elif kind == 'nav_topology': kw['nav'] = []
    elif kind == 'safety_topology': kw['safety'] = []
    else: kw[{'status_source':'status', 'mission_source':'mission_count', 'odom_source':'odom_count',
              'arm_source':'arm_count', 'guard_source':'guard_count'}[kind]] = 0
    return kw


@pytest.mark.parametrize('kind', NEGATIVES)
def test_missing_or_unsafe_proof_never_resumes(kind):
    c, now = held_core()
    for _ in range(40):
        now += .05
        feed(c, now, active=True)
        args = inject(c, now, kind)
        linear, angular, payload, action = step(c, now, **args)
        assert (linear, angular, action) == (0., 0., None), (kind, payload)
    assert c.stop_latched and c.released


def test_single_fresh_command_and_cached_motion_never_build_resume_proof():
    c, now = held_core()
    last = now
    for _ in range(100):
        now += .05
        feed(c, now, active=True)
        c.nav_at = last
        result = step(c, now)
        assert result[3] is None and result[:2] == (0., 0.)
    assert c.stop_latched and c.released


def test_stopped_proof_must_be_continuous_and_is_invalid_after_clock_gap():
    c, now = held_core()
    for _ in range(15):
        now += .05
        feed(c, now, active=True)
        assert step(c, now)[3] is None and c.released
    now += 1.1  # A blocked event loop cannot count as a stable proof interval.
    for _ in range(15):
        now += .05
        feed(c, now, active=True)
        assert step(c, now)[3] is None and c.released
    for _ in range(10):
        now += .05
        feed(c, now, active=True)
        result = step(c, now)
        if not c.released: break
    assert not c.released and result[:2] == (0., 0.) and result[3] is None
    assert step(c, now+.01)[3] == 'CREATE'


@pytest.mark.parametrize('kind', ['binding', 'cancel_epoch', 'goal_changed', 'duplicate_goal', 'terminal4',
    'terminal5', 'terminal6', 'disabled', 'fire', 'mission_health', 'mission_contract',
    'nav_nan', 'nav_out_of_bounds', 'raw_conflict', 'nav_topology',
    'safety_topology', 'status_source', 'mission_source', 'odom_source', 'arm_source', 'guard_source'])
def test_revoked_ticket_cannot_be_reused_when_fault_or_old_uuid_returns(kind):
    c, now = held_core()
    feed(c, now+.05, active=True)
    result = step(c, now+.05, **inject(c, now+.05, kind))
    assert result[:2] == (0., 0.) and c.app_nav_ticket is None
    c.app_recovery_binding = ('session', 0, 'identity', 0, 'pins')
    c.enabled = True
    c.fire_latched = False
    c.update_statuses([('ab'*16, 2)], now+.10)
    for _ in range(50):
        now += .05
        feed(c, now, active=True)
        result = step(c, now)
        assert (result[0], result[1], result[3]) == (0., 0., None)
    assert c.released and c.stop_latched


@pytest.mark.parametrize('kind', ['cmd_gap', 'cmd_nonzero', 'odom_moving', 'actuator_gap',
                                 'actuator_contract', 'pump_on', 'servo_on'])
def test_incomplete_stop_proof_restarts_continuous_verification(kind):
    c, now = held_core()
    for _ in range(15):
        now += .05; feed(c, now, active=True); step(c, now)
    now += .05; feed(c, now, active=True)
    step(c, now, **inject(c, now, kind))
    c.app_recovery_off = True
    for _ in range(15):
        now += .05; feed(c, now, active=True)
        result = step(c, now)
        assert result[3] is None and c.released
    for _ in range(10):
        now += .05; feed(c, now, active=True); step(c, now)
        if not c.released: break
    assert not c.released and c.goal_uuid == 'ab'*16


@pytest.mark.parametrize('field', ['lease_missing', 'operator_instance', 'cancel_epoch', 'runtime_generation',
                                 'runtime_held', 'session_closed', 'session_fault', 'identity_changed', 'pump_owner'])
def test_endpoint_requires_current_lease_and_rejects_changed_session(monkeypatch, tmp_path, app_class, field):
    h, mode = moving_robot(monkeypatch, tmp_path)
    gateway = wait_silent(h, mode)
    mode[0] = 'move'
    publisher = next(p for p in h.bus.pubs if p.topic == '/fire_mission/operator_state')
    send = publisher.publish
    bad = [True]
    def publish(message):
        value = copy.deepcopy(message)
        report = json.loads(value.data)
        if bad[0]:
            if field == 'lease_missing': report.pop('app_navigation_lease', None)
            elif field == 'operator_instance': report['app_navigation_lease']['session'] = 'cd'*16
            elif field == 'cancel_epoch': report['app_navigation_lease']['cancel_epoch'] += 1
            elif field == 'runtime_generation':
                report['runtime_rebind']['generation'] += 1
                report['runtime_rebind']['applied_generation'] += 1
            elif field == 'runtime_held': report['runtime_rebind']['held'] = True
            elif field == 'session_closed': report['closed'] = True
            elif field == 'session_fault': report['fault'] = 'STOP_REQUESTED'
            elif field == 'identity_changed': report['armed_identity'][1] += 1
        value.data = json.dumps(report)
        send(value)
    monkeypatch.setattr(publisher, 'publish', publish)
    pump = None
    if field == 'pump_owner':
        from std_msgs.msg import Bool
        from harness import BE
        pump = h.bus.publisher('fire_pulse_controller', Bool, '/pump_cmd', BE)
    stopped_x = h.x
    for _ in range(12): h.step()
    assert gateway.app_nav_ticket is None and h.x == stopped_x
    bad[0] = False
    if pump: h.bus.pubs.remove(pump)
    for _ in range(50): h.step()
    assert h.x == stopped_x and not h.pulses
    assert h.bus.max_raw <= 1


def test_odom_gap_stops_then_requires_new_proof_before_same_goal_resume(monkeypatch, tmp_path, app_class):
    h, mode = moving_robot(monkeypatch, tmp_path)
    gateway = h.by_name['live_nav_stop_gateway'].core
    h.bus.drop = lambda p, s, m: p.topic == '/odom' and s.info.node_name == 'live_nav_stop_gateway'
    for _ in range(25): h.step()
    assert gateway.stop_latched and gateway.app_nav_ticket is not None
    stopped_x = h.x
    h.bus.drop = lambda *args: False
    for _ in range(12):
        h.step(); assert h.x == stopped_x
    h.until(lambda r: h.x > stopped_x+.01)
    assert gateway.app_nav_recovery['state'] == 'RESUMED' and not h.pulses


@pytest.mark.parametrize('after_create', [False, True])
@pytest.mark.parametrize('kind', ['binding', 'cancel_epoch', 'goal_changed', 'fire', 'raw_conflict',
                                 'nav_topology', 'status_source', 'cmd_nonzero', 'actuator_gap', 'pump_on'])
def test_reacquisition_checks_withdrawal_before_first_motion(kind, after_create):
    c, now = held_core()
    for _ in range(30):
        now += .05; feed(c, now, active=True); step(c, now)
        if not c.released: break
    assert not c.released and c.app_nav_recovery['state'] == 'REACQUIRING'
    if after_create:
        now += .05; feed(c, now, active=True)
        assert step(c, now)[3] == 'CREATE'
        c.publisher_created(now)
    now += .05; feed(c, now, active=True)
    args = inject(c, now, kind)
    if after_create:
        args.update(owned=True, raw=RAW_SOURCE if kind != 'raw_conflict' else RAW_SOURCE+RAW_SOURCE)
    result = step(c, now, **args)
    assert result[:2] == (0., 0.) and result[3] != 'CREATE'
    assert c.stop_latched
    if kind not in ('cmd_nonzero', 'actuator_gap', 'pump_on'):
        assert c.app_nav_ticket is None


def test_query_delay_cannot_spend_an_expired_recovery_lease():
    c, now = held_core()
    for _ in range(30):
        now += .05; feed(c, now, active=True); step(c, now)
        if not c.released: break
    assert not c.released
    feed(c, now+.01, active=True)
    assert step(c, now+.01)[3] == 'CREATE'
    c.publisher_created(now+.01)
    feed(c, now+.02, active=True)
    c.app_recovery_checked_at = now+.02
    c.app_recovery_clock = lambda: now+.6  # A graph query stalled after sampling now.
    result = step(c, now+.02, owned=True, raw=RAW_SOURCE)
    assert result[:2] == (0., 0.) and c.stop_latched and c.app_nav_ticket is not None


@pytest.mark.parametrize('hold_ticks', [20, 60])
def test_operator_identity_pause_cannot_skip_nav_recovery_verification(monkeypatch, tmp_path, app_class, hold_ticks):
    h, mode = moving_robot(monkeypatch, tmp_path)
    gateway = wait_silent(h, mode)
    h.bus.drop = lambda p, s, m: (p.topic == '/fire_sensors/serial_session_identity'
                                and s.info.node_name == 'fire_robot_operator')
    for _ in range(hold_ticks): report = h.step()
    assert report['identity_hold'] and h.x > 0
    stopped_x = h.x
    h.bus.drop = lambda *args: False
    mode[0] = 'move'
    h.until(lambda r: not r['identity_hold'], limit=150)
    for _ in range(12):
        report = h.step()
        assert h.x == stopped_x
    h.until(lambda r: h.x > stopped_x+.01)
    assert gateway.app_nav_recovery['state'] == 'RESUMED'
    assert not h.pulses and h.bus.max_raw == 1


def test_event_driven_unchanged_status_need_not_be_republished_during_wait(monkeypatch, tmp_path, app_class):
    h, mode = moving_robot(monkeypatch, tmp_path)
    gateway = h.by_name['live_nav_stop_gateway'].core
    received = gateway.status_at
    h.bus.drop = lambda p, s, m: p.topic == '/navigate_to_pose/_action/status'
    wait_silent(h, mode, ticks=100)
    stopped_x = h.x
    mode[0] = 'move'
    h.until(lambda r: h.x > stopped_x+.01)
    assert gateway.status_at == received
    assert gateway.app_nav_recovery['state'] == 'RESUMED' and not h.bus.cancelled


@pytest.mark.parametrize('kind', ['odom_gap', 'mission_gap', 'arm_gap', 'arm_false', 'operator_pause'])
@pytest.mark.parametrize('after_create', [False, True])
def test_returning_input_during_reacquire_requires_full_proof_again(kind, after_create):
    c, now = held_core()
    for _ in range(30):
        now += .05; feed(c, now, active=True); step(c, now)
        if not c.released: break
    assert c.app_nav_recovery['state'] == 'REACQUIRING'
    if after_create:
        now += .05; feed(c, now, active=True)
        assert step(c, now)[3] == 'CREATE'
        c.publisher_created(now)
    now += .05; feed(c, now, active=True)
    if kind == 'operator_pause':
        c.app_session_current = False; args = {}
    else:
        args = inject(c, now, kind)
    if after_create: args.update(owned=True, raw=RAW_SOURCE)
    result = step(c, now, **args)
    assert result[:2] == (0., 0.) and result[3] != 'CREATE'
    assert c.app_nav_ticket is not None and c.stop_latched
    # Simulate the actual zero/release drain after CREATE, then return inputs.
    for _ in range(20):
        now += .05; feed(c, now, active=True)
        if kind == 'operator_pause': c.app_session_current = False; args = {}
        else: args = inject(c, now, kind)
        result = step(c, now, owned=after_create and not c.released,
                      raw=RAW_SOURCE if after_create and not c.released else [], **args)
        if result[3] == 'RELEASE': c.publisher_released()
        assert result[:2] == (0., 0.)
    c.app_session_current = True
    for _ in range(15):
        now += .05; feed(c, now, active=True)
        assert step(c, now)[3] is None and c.released
    for _ in range(10):
        now += .05; feed(c, now, active=True); step(c, now)
        if not c.released: break
    assert not c.released and c.goal_uuid == 'ab'*16



def test_graph_query_delay_returns_without_permanent_goal_lock():
    c, now=held_core()
    for _ in range(30):
        now+=.05; feed(c,now,active=True); step(c,now)
        if not c.released:break
    assert c.app_nav_recovery['state']=='REACQUIRING'
    checked=now+.01
    feed(c,checked,active=True);c.app_recovery_checked_at=checked
    c.app_recovery_clock=lambda:checked+.6
    result=step(c,checked)
    assert result[:2]==(0.,0.) and result[3]!='CREATE'
    assert c.app_nav_ticket is not None, 'A delayed query must pause proof, not erase the SAME valid intent'
    now=checked+.6;c.app_recovery_clock=None
    for _ in range(15):
        now+=.05; feed(c,now,active=True);c.app_recovery_checked_at=now
        assert step(c,now)[3] is None and c.released
    for _ in range(10):
        now+=.05;feed(c,now,active=True);c.app_recovery_checked_at=now;step(c,now)
        if not c.released:break
    assert not c.released and c.goal_uuid=='ab'*16
