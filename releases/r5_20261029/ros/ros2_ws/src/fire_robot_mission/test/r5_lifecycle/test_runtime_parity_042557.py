"""Delta audit: real six adapters with independent sensor/report clocks.

No ROS context/devices. Unlike the old all-at-once fixtures, ADC/ACT/identity
arrive at 5 Hz, vision at 10 Hz and Nav2 status only on an actual transition.
"""
import copy
import json
from pathlib import Path
import pytest
import yaml
from harness import Robot, Bus
from fire_robot_mission.ir_range_preview import IrRangePreviewNode


def scheduled_robot(monkeypatch, *, goal=False, phase=1, operator_parameters=None):
    # Preserve real create_timer periods instead of invoking every callback at
    # the 20 Hz operator clock. One tick is 50 ms; round up slower periods.
    from rclpy.node import Node
    init = Bus.__init__
    def timed_bus(bus, patch, overrides=None):
        init(bus, patch, overrides)
        config = Path(__file__).parents[2] / 'config'
        # Load exactly the stack's parameter layers, including the new IR
        # band policy. Constructor defaults alone use the legacy IR policy.
        for name in ('mission_params.yaml','operational_params.yaml',
                     'operational_gate2.yaml','ir_range_preview.yaml'):
            for node, values in yaml.safe_load((config/name).read_text()).items():
                bus.overrides.setdefault(node, {}).update(values.get('ros__parameters', {}))
        bus.overrides['fire_robot_operator']['enable_live_intents'] = True
        bus.overrides['fire_robot_operator']['allow_close_stationary'] = True
        bus.overrides['fire_pulse_controller']['allow_close_stationary'] = True
        bus.overrides['fire_robot_operator'].update(operator_parameters or {})
        def timer(node, period, callback):
            every = max(1, round(period / .05))
            offset = sum(map(ord, node.test_name)) % every
            node.test_timers.append(lambda: callback() if round((bus.now-1.)/.05) % every == offset else None)
        patch.setattr(Node, 'create_timer', timer)
    monkeypatch.setattr(Bus, '__init__', timed_bus)
    h = Robot(monkeypatch, goal=goal)
    b = h.bus
    b.pubs.remove(h.external['/fire_mission/ir_range_preview'])
    preview = IrRangePreviewNode()
    h.nodes.append(preview)
    emit = h.emit
    previous_status = [None]
    h.silence_serial = False
    serial = {'/odom', '/fire_sensors/ir_raw', '/fire_sensors/actuator_state',
              '/fire_sensors/serial_session_identity'}
    def cadence(topic, value):
        tick = round((b.now-1.)/.05)
        if topic == '/fire_mission/ir_range_preview': return
        if topic in serial and h.silence_serial: return
        offset = {'/fire_sensors/ir_raw': phase,
                  '/fire_sensors/actuator_state': (phase+1) % 4,
                  '/fire_sensors/serial_session_identity': (phase+2) % 4}.get(topic)
        if offset is not None and tick % 4 != offset: return
        if topic.startswith('/yolo/') and tick % 2: return
        if topic == '/navigate_to_pose/_action/status':
            status = [(bytes(x.goal_info.goal_id.uuid).hex(), x.status) for x in value.status_list]
            if status == previous_status[0]: return
            previous_status[0] = copy.deepcopy(status)
        emit(topic, value)
    h.emit = cadence
    sample = h.op.core._range_samples
    h.range_trace = []
    def traced(now, live):
        core = h.op.core
        ir = core.get('ir', now)
        result = sample(now, live)
        h.range_trace.append(dict(at=now, result=result, count=len(core.confirm_samples),
            aligned=core._aligned(now), ir=copy.deepcopy(ir)))
        return result
    h.op.core._range_samples = traced
    return h


def wait(h, predicate, limit=400):
    try: return h.until(predicate, limit=limit)
    except AssertionError:
        evidence=Path(__file__).parents[6]/'evidence_r5_5_full_parity_042557'
        (evidence/f'range_trace_{h.op.core.cycles.current.nav_goal_uuid if h.op.core.cycles.current else "none"}.json').write_text(json.dumps(h.range_trace[-100:],indent=2))
        raise


def turn(h):
    h.arm()
    # Age the retained Nav2 event before entering fire/cancel.
    for _ in range(20): h.step()
    h.fire = True
    h.step('F')
    wait(h, lambda r:r['phase'] == 'WAIT_T')
    r = h.step('T')
    assert r['operator_feedback']['accepted'], h.diagnostic(r)
    wait(h, lambda r:abs(h.yaw) > .025)
    h.centered = True
    wait(h, lambda r:r['phase'] == 'WAIT_IR_I')


@pytest.mark.parametrize('goal,phase', [(False,0),(True,1),(False,2),(True,3)])
def test_async_T_I_P_clear_and_new_goal(monkeypatch, goal, phase):
    h = scheduled_robot(monkeypatch, goal=goal, phase=phase)
    turn(h)
    # I is deliberately pressed only on a valid fresh-band report.
    wait(h, lambda r:r['ir_band'] == 'APPROACH' and r['phase'] == 'WAIT_IR_I')
    r = h.step('I')
    assert r['operator_feedback']['accepted'], h.diagnostic(r)
    start = h.op.core.cycles.budget.requested_at
    wait(h, lambda r:h.x > .03)
    h.right = 950
    wait(h, lambda r:r['phase'] == 'WAIT_P')
    assert h.step('P')['operator_feedback']['accepted']
    wait(h, lambda r:h.home == 1)
    h.fire = False
    r = wait(h, lambda r:r['handback']['consumed'])
    assert r['reason'] == 'NAV2_READY_NEW_GOAL'
    assert h.on_at is not None and h.off_at-h.on_at <= 5.1
    assert h.bus.max_raw == 1 and .03 < h.x <= .30
    assert {q['budget']['requested_at'] for q in h.reports if q['budget'] and q['budget']['requested_at']} == {start}
    h.status = [('56'*16, 2)]
    wait(h, lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    assert not r['closed']


@pytest.mark.parametrize('stage', ['WAIT_T','ALIGNING','IR_RUNNING'])
def test_serial_loss_then_epoch_change_blocks_T_and_all_live_intents(monkeypatch, stage):
    h = scheduled_robot(monkeypatch, goal=True)
    h.arm(); h.fire = True; h.step('F')
    wait(h, lambda r:r['phase'] == 'WAIT_T')
    if stage != 'WAIT_T':
        h.step('T'); wait(h, lambda r:abs(h.yaw) > .025)
    if stage == 'IR_RUNNING':
        h.centered = True; wait(h, lambda r:r['phase'] == 'WAIT_IR_I')
        h.step('I'); wait(h, lambda r:h.x > .02)
    h.silence_serial = True
    for _ in range(12): r = h.step()
    assert r['identity_hold'] and not r['closed']
    identity = copy.deepcopy(h.op.core.inputs['identity'][0])
    identity.update(connection_epoch=identity['connection_epoch']+1,valid=False,
                    session_ready=False,telemetry_healthy=False,elf_sha256=None)
    # Publish a real bridge-style invalid new epoch, while other serial data
    # remain missing. The USB event itself is not simulated as healthy data.
    h.external['/fire_sensors/serial_session_identity'].publish(
        __import__('std_msgs.msg',fromlist=['String']).String(data=json.dumps(identity)))
    h.bus.flush()
    r = h.step('T')
    assert r['fault'] == 'BOOT_OR_SERIAL_EPOCH_CHANGED'
    assert not r['armed'] and not r['closed'] and not any(r['intents'].values())
    assert r['operator_feedback']['key'] == 'T'
    assert not r['operator_feedback']['accepted']
    assert 'BOOT_OR_SERIAL_EPOCH_CHANGED' in r['operator_feedback']['reason']


@pytest.mark.parametrize('terminal', [4,5,6])
def test_event_driven_terminal_allows_new_goal_without_full_restart(monkeypatch, terminal):
    h = scheduled_robot(monkeypatch, goal=True)
    h.arm(); wait(h, lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    for _ in range(30): h.step()
    h.status = [('a'*32, terminal)]
    wait(h, lambda r:r['reason'] == 'NAV2_READY_NEW_GOAL')
    h.status.append(('56'*16,2))
    wait(h, lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    assert h.op.core.armed and not h.op.core.closed and not h.bus.cancelled


@pytest.mark.parametrize('goal,band', [(False,750),(False,950),(True,750),(True,950)])
def test_async_stationary_I_P_no_forward_motion(monkeypatch, goal, band):
    h = scheduled_robot(monkeypatch, goal=goal, phase=1)
    h.right = band
    turn(h)
    wait(h, lambda r:r['ir_band'] in ('SPRAY_RANGE','TOO_CLOSE'))
    assert h.step('I')['operator_feedback']['accepted']
    wait(h, lambda r:r['phase']=='WAIT_P' and r['p_admission_blocker'] is None)
    assert h.step('P')['operator_feedback']['accepted']
    wait(h, lambda r:h.home==1)
    assert h.x == 0 and h.bus.max_raw <= 1
    h.fire=False
    wait(h, lambda r:r['handback']['consumed'])


@pytest.mark.parametrize('goal',[False,True])
def test_async_target_loss_during_I_X_clear_new_goal(monkeypatch, goal):
    h=scheduled_robot(monkeypatch,goal=goal,phase=1)
    turn(h);wait(h,lambda r:r['ir_band']=='APPROACH')
    assert h.step('I')['operator_feedback']['accepted']
    wait(h,lambda r:h.x>.03)
    budget_at=h.op.core.cycles.budget.requested_at
    h.fire=False
    for _ in range(20):r=h.step()
    assert not r['closed'] and h.on_at is None
    assert h.op.core.cycles.budget.requested_at==budget_at
    assert h.step('X')['operator_feedback']['accepted']
    wait(h,lambda r:r['handback']['consumed'])
    h.status=[('56'*16,2)]
    wait(h,lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)


@pytest.mark.parametrize('age,valid',[(.15,True),(.150001,False),(.20,False)])
def test_ir_boundary_roundoff_does_not_extend_freshness(age,valid):
    from fire_robot_mission.operational_session_core import OperationalSessionCore
    c=OperationalSessionCore()
    c.update('ir',{'ir_receive_age_ms':150.},13.)
    assert (c.get('ir',13.+age) is not None)==valid


@pytest.mark.parametrize('case',['negative','missing','wrong_cycle','old_index','used','stale'])
def test_P_negative_receipt_only_releases_proven_unused_current_request(case):
    from fire_robot_mission.operational_session_core import OperationalSessionCore
    from types import SimpleNamespace as N
    c=OperationalSessionCore()
    c.cycles.current=N(cycle_id='12'*16,nav_goal_uuid='34'*16)
    c.identity_lease.bound=('ab'*16,1,'cd'*32)
    c.p_used=True;c.p_request_baseline=3
    c.p_request_feedback=dict(index=9,key='P',accepted=True,reason='P_ACCEPTED_ONE_PULSE',at=1.)
    actor=dict(r5_cycle_context=dict(cycle_id='12'*16,nav_goal_uuid='34'*16,
        bridge_instance_id='ab'*16,connection_epoch=1,elf_sha256='cd'*32),
        state='WAIT_P',request_used=False,pulse_request_feedback=dict(index=4,accepted=False,reason='PROOF_LOST'))
    if case=='missing':actor.pop('pulse_request_feedback')
    if case=='wrong_cycle':actor['r5_cycle_context']['cycle_id']='ef'*16
    if case=='old_index':actor['pulse_request_feedback']['index']=3
    if case=='used':actor['request_used']=True
    c.update('actor',actor,0. if case=='stale' else 2.)
    c._rest=lambda now:True
    c._pulse_receipt(2.)
    assert c.p_used == (case!='negative')
    if case=='negative':
        assert not c.feedback['accepted'] and c.feedback['key']=='P'
        assert c.p_request_baseline is None


@pytest.mark.parametrize('age,admit,accepted',[(.099,True,True),(.1,True,True),(.100001,True,False),(.1,False,False)])
def test_P_edge_roundoff_and_negative_actor_receipt(age,admit,accepted):
    from fire_robot_mission.pulse_transport import Transport
    class Session:
        used=False
        def update(self,*args):pass
        def get(self,*args):return None
        def step(self,now,counts,request=False,stop=False):
            self.used=bool(request and admit)
            return dict(state='FIRING' if self.used else 'WAIT_P',blockers=[] if admit else ['IR_STALE'])
    t=Transport(Session());t.arm(False,10.);t.arm(True,10.05)
    r=t.step(10.05+age,{})
    assert r['pulse_request_feedback']['accepted']==accepted
    assert r['pulse_request_feedback']['index']==1
    t.arm(True,10.05+age+.05)
    r2=t.step(10.05+age+.05,{})
    assert r2['pulse_request_feedback']==r['pulse_request_feedback']


@pytest.mark.parametrize('fault',[None,'no_receipt','wrong_cycle','epoch','graph','raw','moving','guard_other','active_goal'])
def test_retire_after_IR_stop_loss_requires_cancel_history_and_current_STOP(fault):
    from test_release_core import Chain
    from fire_robot_mission.cycle_release import ReleaseInputs,ReleaseContext
    c=Chain();c.tick()
    p=ReleaseInputs();p.inputs=copy.deepcopy(c.master.inputs)
    ctx=ReleaseContext.parse(c.context)
    p.stop_errors(ctx,c.now,True,0,0)
    assert (ctx.cycle_id,ctx.nav_goal_uuid,ctx.identity) in p.cancel_seen
    g,at=p.inputs['guard'];g.update(state='FAILED',blockers=['IR_APPROACH_STOP_PROOF_INVALID'])
    if fault=='no_receipt':p.cancel_seen.clear()
    if fault=='wrong_cycle':ctx=ReleaseContext.parse(dict(c.context,cycle_id='98'*16))
    if fault=='epoch':p.inputs['identity'][0]['connection_epoch']=2
    if fault=='moving':p.inputs['odom']=([.03,0,0,0,0,0],c.now)
    if fault=='guard_other':g['blockers']=['CANCEL_SERVICE_FAILED']
    if fault=='active_goal':p.inputs['status']=([('98'*16,2)],c.now)
    errors=p.stop_errors(ctx,c.now,fault!='graph',1 if fault=='raw' else 0,0)
    assert bool(errors)==(fault is not None),errors
