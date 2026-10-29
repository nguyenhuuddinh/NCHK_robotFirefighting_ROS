"""115816: real adapter clocks, fresh sample continuity and asymmetric holds."""
import copy
import json
from pathlib import Path
import sys
import pytest
from test_runtime_parity_042557 import scheduled_robot
from test_automatic_065731 import marker,SESSION,PACKET

sys.path.insert(0,str(Path(__file__).parents[1]/'r5_delta'))


def robot(monkeypatch,tmp_path,*,goal=True,phase=1):
    path=tmp_path/'automatic_start.json'
    h=scheduled_robot(monkeypatch,goal=goal,phase=phase,operator_parameters=dict(
        automatic_mode=True,automatic_start_file=str(path),automatic_runner_session=SESSION,automatic_packet=PACKET))
    for _ in range(90):h.step()
    path.write_text(json.dumps(marker()));path.chmod(0o600)
    h.until(lambda r:r['armed']);h.until(lambda r:h.op.core.fire_permission)
    return h


def approach(h):
    h.fire=True;h.ir_flame=True
    h.until(lambda r:abs(h.yaw)>.025);h.centered=True
    h.until(lambda r:h.x>.025)


@pytest.mark.parametrize('phase',[0,1,2,3])
def test_actual_IR_source_age_and_clock_phase_reach_one_automatic_pulse(monkeypatch,tmp_path,phase):
    h=robot(monkeypatch,tmp_path,phase=phase)
    emit=h.emit
    def aged(topic,value):
        if topic=='/fire_sensors/ir_raw':value=dict(value,receive_age_ms=90.)
        emit(topic,value)
    h.emit=aged
    approach(h);h.right=950
    h.until(lambda r:h.home==1,limit=350)
    assert len(h.pulses)==1 and h.off_at-h.on_at<=5.1+1e-6 and h.bus.max_raw==1
    h.fire=False;h.ir_flame=False
    h.until(lambda r:r['handback']['consumed'],limit=250)
    h.status=[('56'*16,2)]
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)


@pytest.mark.parametrize('clear',[False,True])
@pytest.mark.parametrize('delay',[0,3])
def test_operator_only_identity_gap_after_pump_registration_cannot_latch_guard(monkeypatch,tmp_path,clear,delay):
    h=robot(monkeypatch,tmp_path);approach(h);h.right=950
    actor=h.by_name['fire_pulse_controller']
    h.until(lambda r:actor.pub is not None)
    assert not h.pulses
    budget=h.op.core.cycles.budget.requested_at
    start=h.bus.now;active=[True]
    def gap(p,s,msg):
        if not active[0]:return False
        if p.topic=='/fire_sensors/serial_session_identity' and s.info.node_name=='fire_robot_operator':return True
        if (p.topic=='/fire_mission/cycle_control' and s.info.node_name=='nav2_cancel_guard'
                and 'session_hold' in json.loads(msg.data) and h.bus.now<start+.4+delay*.05):return True
        return False
    h.bus.drop=gap
    h.until(lambda r:r['identity_hold'],limit=20)
    for _ in range(10):h.step()
    assert not h.pulses
    if clear:h.fire=False;h.ir_flame=False
    active[0]=False
    if clear:
        h.until(lambda r:r['handback']['consumed'],limit=250)
        assert actor.pub is None and not h.pulses
    else:
        h.until(lambda r:h.home==1,limit=350)
        assert len(h.pulses)==1 and h.op.core.cycles.budget.requested_at==budget
        h.fire=False;h.ir_flame=False;h.until(lambda r:r['handback']['consumed'])
    h.status=[('56'*16,2)]
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)


def test_range_confirmation_counts_five_fresh_sequences_across_short_expiry_only():
    from fire_robot_mission.operational_session_core import OperationalSessionCore
    from test_installed_pulse import ready
    c=OperationalSessionCore();c._aligned=lambda now:True
    ir=ready()['ir'];ir.update(ir_receive_age_ms=170.,zone='SPRAY_RANGE',blockers=[])
    good=[]
    for n in range(6):
        at=10.+n*.2;c.update('ir',dict(ir,ir_sample_seq=100+n),at)
        good.append(c._range_samples(at,{}))
        # A later 20 Hz poll has exceeded the 300ms ON grant; it must not
        # authorize P nor erase distinct already validated ADC observations.
        assert not c._range_samples(at+.15,{})
    assert good[-1] and len(c.confirm_samples)==5


@pytest.mark.parametrize('fault',['stale_ACT','ON','moving','bad_identity'])
def test_actor_OFF_endpoint_release_never_claims_completion_without_STOP(monkeypatch,tmp_path,fault):
    h=robot(monkeypatch,tmp_path);approach(h);h.right=950
    node=h.by_name['fire_pulse_controller'];h.until(lambda r:node.pub is not None)
    ep=node._cycle;p=ep.participant;now=h.bus.now
    # Current guard failure must not be the reason to keep a safe OFF endpoint.
    p.inputs['guard'][0].update(state='FAILED',blockers=['OPERATOR_ARM_LOST'])
    if fault=='stale_ACT':p.inputs['actuator']=(p.inputs['actuator'][0],now-1.)
    if fault=='ON':p.inputs['actuator'][0]['pump_pwm_commanded_on']=True
    if fault=='moving':p.inputs['odom']=((.03,0,0,0,0,0),now)
    if fault=='bad_identity':p.inputs['identity'][0]['connection_epoch']+=1
    ep._withdraw(now,force=True)
    assert node.pub is not None


@pytest.mark.parametrize('goal',[False,True])
def test_new_goal_during_clear_retirement_is_canceled_without_deadlock(monkeypatch,tmp_path,goal):
    h=robot(monkeypatch,tmp_path,goal=goal);approach(h)
    h.fire=False;h.ir_flame=False
    h.until(lambda r:h.op.core.cancel_requested)
    blocked='78'*16;h.status.append((blocked,2))
    h.until(lambda r:r['handback']['consumed'],limit=300)
    assert (any(u==blocked for at,u in h.bus.cancelled)) and not h.pulses
    assert not any(s in (1,2,3) for u,s in h.status)
    h.status.append(('56'*16,2))
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)


@pytest.mark.parametrize('fault',['graph','epoch','ACT_ON','stale_ACT','cmd','raw','pump','no_history','original_changed','multiple_active','service_down','send_error'])
def test_retirement_foreign_cancel_never_bypasses_STOP_or_original_cancel(fault):
    from types import SimpleNamespace as N
    from test_release_core import Chain
    from fire_robot_mission.cycle_blocked_goal import BlockedGoalCancel
    c=Chain();c.tick();p=c.peers['guard'];now=c.now
    p.update('status',[(c.context['nav_goal_uuid'],5),('78'*16,2)],now)
    sent=[]
    def send(request):
        sent.append(bytes(request.goal_info.goal_id.uuid).hex())
        if fault=='send_error':raise RuntimeError('send failed')
        return N(done=lambda:False)
    client=N(service_is_ready=lambda:fault!='service_down',call_async=send)
    node=N(cancel_client=client,core=N(failure=None),get_publishers_info_by_topic=lambda t:
        [object()] if fault=='raw' and t=='/cmd_vel_raw' or fault=='pump' and t=='/pump_cmd' else [])
    ep=N(participant=p,node=node,graph_ok=fault!='graph')
    if fault=='epoch':p.inputs['identity'][0]['connection_epoch']+=1
    if fault=='ACT_ON':p.inputs['actuator'][0]['pump_pwm_commanded_on']=True
    if fault=='stale_ACT':p.inputs['actuator']=(p.inputs['actuator'][0],now-1.)
    if fault=='cmd':p.inputs['cmd']=((.01,0,0,0,0,0),now)
    if fault=='no_history':
        p.cancel_seen.clear()
        p.inputs['guard'][0].update(state='FAILED',blockers=['OPERATOR_ARM_LOST'])
    if fault=='original_changed':p.inputs['status'][0][0]=(c.context['nav_goal_uuid'],6)
    if fault=='multiple_active':p.inputs['status'][0].append(('56'*16,2))
    helper=BlockedGoalCancel();result=helper.step(ep,now);helper.step(ep,now+.01)
    assert result is not None
    assert sent==(['78'*16] if fault=='send_error' else [])


@pytest.mark.parametrize('fault',['band','center','invalid','long_gap','regression','repeated'])
def test_IR_confirmation_cannot_survive_contradiction_or_count_old_samples(fault):
    from fire_robot_mission.operational_session_core import OperationalSessionCore
    from test_installed_pulse import ready
    c=OperationalSessionCore();c._aligned=lambda now:True
    ir=ready()['ir'];ir.update(zone='SPRAY_RANGE',blockers=[])
    for n in range(4):
        c.update('ir',dict(ir,ir_sample_seq=100 if fault=='repeated' else 100+n),10.+n*.2)
        assert not c._range_samples(10.+n*.2,{})
    if fault=='repeated':assert not c.confirm_samples;return
    at=10.8;data=dict(ir,ir_sample_seq=104)
    if fault=='band':data.update(ir_band='APPROACH',zone='APPROACH')
    if fault=='center':c._aligned=lambda now:False
    if fault=='invalid':data['schema_version']=99
    if fault=='regression':data['ir_sample_seq']=1
    if fault=='long_gap':at=11.4
    else:c.update('ir',data,at)
    assert not c._range_samples(at,{})
    assert not c.confirm_samples or c.fault=='IR_SEQUENCE_REGRESSION'


@pytest.mark.parametrize('fault',['bad_tag','stale_control','old_hold','no_history','epoch','other_failure','ACT_ON','active_goal'])
def test_controlled_hold_cannot_repair_unrelated_cancel_fault(monkeypatch,tmp_path,fault):
    h=robot(monkeypatch,tmp_path);approach(h);h.right=950
    actor=h.by_name['fire_pulse_controller'];h.until(lambda r:actor.pub is not None)
    actor.destroy_publisher(actor.pub);actor.pub=None
    node=h.by_name['nav2_cancel_guard'];ep=node._cycle;p=ep.participant;now=h.bus.now
    node.core.failure='OPERATOR_ARM_LOST'
    p.request=copy.deepcopy(p.request);p.request['session_hold']=dict(schema_version=1,cause='SAME_BOOT_IDENTITY_RECOVERY')
    ep.controlled_hold_seen=False
    if fault=='bad_tag':p.request['session_hold']['schema_version']=True
    if fault=='stale_control':p.request_at=now-.4
    if fault=='old_hold':
        p.request.pop('session_hold');ep.controlled_hold_seen=True;ep.controlled_hold_at=now-2.
    if fault=='no_history':p.cancel_seen.clear()
    if fault=='epoch':p.inputs['identity'][0]['connection_epoch']+=1
    if fault=='other_failure':node.core.failure='CANCEL_SERVICE_FAILED'
    if fault=='ACT_ON':p.inputs['actuator'][0]['pump_pwm_commanded_on']=True
    if fault=='active_goal':p.inputs['status']=([('78'*16,2)],now)
    ep.before_step(now)
    assert node.core.failure is not None


def test_OFF_release_is_separate_from_cycle_ACK_and_needs_new_ACT(monkeypatch,tmp_path):
    h=robot(monkeypatch,tmp_path);approach(h);h.right=950
    node=h.by_name['fire_pulse_controller'];h.until(lambda r:node.pub is not None)
    ep=node._cycle;p=ep.participant;now=h.bus.now
    p.inputs['guard'][0].update(state='FAILED',blockers=['CANCEL_SERVICE_FAILED'])
    ep._withdraw(now,force=True);assert node.pub is not None
    a=copy.deepcopy(p.inputs['actuator'][0]);a['esp_ms']+=1
    p.update('actuator',a,now+.05)
    ep._withdraw(now+.05,force=True);assert node.pub is None
    assert 'WAIT_REAL_GOAL_CANCELED' in p.stop_errors(p.context,now+.05,True,0,0)


def test_UI_explains_IR_age_confirmation_and_guard_without_granting_permissions():
    from fire_robot_mission.operational_view import status_lines
    r=dict(armed=True,phase='IR_CONFIRM',reason='WAIT_FIVE_NEW_IR_SAMPLES',
        ir_source_age_ms=320.,range_confirm_samples=3,aligned_stop_proof=False,
        cancel_guard_state='FAILED',cancel_guard_blockers=['OPERATOR_ARM_LOST'])
    before=copy.deepcopy(r);lines='\n'.join(status_lines(r))
    assert '320.0ms' in lines and '3/5' in lines and 'OPERATOR_ARM_LOST' in lines
    assert r==before


@pytest.mark.parametrize('during_pulse',[False,True])
def test_Q_drains_actual_OFF_endpoint_and_waits_home_if_pulse_started(monkeypatch,tmp_path,during_pulse):
    h=robot(monkeypatch,tmp_path);approach(h);h.right=950
    actor=h.by_name['fire_pulse_controller']
    if during_pulse:h.until(lambda r:h.device_on)
    else:h.until(lambda r:actor.pub is not None)
    h.step('Q')
    r=h.until(lambda r:r.get('shutdown',{}).get('done'),limit=90)
    assert r['shutdown']['confirmed'] and actor.pub is None and not h.device_on
    if during_pulse:assert h.home==1 and len(h.pulses)==1
    else:assert not h.pulses
