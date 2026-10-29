"""153207: bounded stage outcomes on real adapters, no ROS/device runtime."""
import pytest
from test_automatic_065731 import robot


def stationary(h):
    h.right=950;h.fire=True;h.ir_flame=True
    h.until(lambda r:abs(h.yaw)>.025)
    h.centered=True
    h.until(lambda r:h.op.core.pending_stationary)


def test_stationary_confirm_lost_center_realigns_then_sprays(monkeypatch,tmp_path):
    h,_=robot(monkeypatch,tmp_path,goal=True)
    stationary(h)
    yaw=h.yaw;h.centered=False
    h.until(lambda r:h.yaw>yaw+.025,limit=100)
    h.centered=True
    h.until(lambda r:h.home==1,limit=300)
    assert h.x==0 and len(h.pulses)==1 and h.bus.max_raw==1


def test_stationary_confirm_timeout_releases_with_fire_still_present(monkeypatch,tmp_path):
    h,_=robot(monkeypatch,tmp_path,goal=True)
    stationary(h)
    # Keep fresh IR in APPROACH: it cannot prove the stationary spray range.
    h.right=5000
    h.until(lambda r:r['handback']['consumed'],limit=550)
    assert h.on_at is None and not h.op.core.closed and h.op.core.cycles.current is None
    h.status=[('56'*16,2)]
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)


def stage(h, name):
    if name=='STATIONARY':stationary(h);return
    h.fire=True;h.ir_flame=True;h.right=5000
    h.until(lambda r:abs(h.yaw)>.025)
    if name=='IR':
        h.centered=True
        h.until(lambda r:h.x>.025)


@pytest.mark.parametrize('goal,name',[(False,'ALIGN'),(True,'ALIGN'),(False,'IR'),(True,'IR'),(False,'STATIONARY'),(True,'STATIONARY')])
def test_X_interrupt_returns_nav_even_with_IR_and_YOLO_fire(monkeypatch,tmp_path,goal,name):
    h,_=robot(monkeypatch,tmp_path,goal=goal);stage(h,name)
    r=h.step('X');assert r['operator_feedback']['accepted']
    h.until(lambda r:r['handback']['consumed'],limit=180)
    assert h.op.core.retry_fire_clear_required and h.op.core.cycles.current is None
    assert h.on_at is None
    for _ in range(12):h.step()
    assert h.op.core.cycles.current is None
    h.status=[('56'*16,2)]
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    h.step('X')
    h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL')
    assert h.op.core.cycles.current is None and h.on_at is None
    # New fire edge is eligible without a new Nav2 goal.
    h.fire=False
    for _ in range(20):h.step()
    assert not h.op.core.retry_fire_clear_required
    h.fire=True;h.centered=False
    h.until(lambda r:h.op.core.cycles.current is not None)
    assert h.bus.max_raw==1 and not h.op.core.closed


@pytest.mark.parametrize('goal,name',[(False,'ALIGN'),(True,'IR'),(True,'STATIONARY')])
def test_target_not_recovered_interrupts_then_new_goal(monkeypatch,tmp_path,goal,name):
    h,_=robot(monkeypatch,tmp_path,goal=goal);stage(h,name)
    old=h.op.core.cycles.current
    h.fire=False # IR still sees flame, now explicitly allowed to end unused attempt.
    h.until(lambda r:r['handback']['consumed'],limit=230)
    assert 'TARGET_NOT_RECOVERED' in h.op.core.interruption_reason
    assert h.on_at is None and h.op.core.cycles.current is None
    h.status=[('56'*16,2)]
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    assert old.cycle_id in h.op.core.cycles.retired


def test_explicit_alignment_timeout_then_nav(monkeypatch,tmp_path):
    h,_=robot(monkeypatch,tmp_path,goal=True)
    h.op.automatic.stage_timeout_s=30.  # optional bounded profile; automatic default is now 0/unlimited
    stage(h,'ALIGN')
    h.until(lambda r:r['handback']['consumed'],limit=750)
    assert h.op.core.interruption_reason=='ALIGN_TASK_TIMEOUT'
    assert h.on_at is None and h.bus.max_raw==1
    h.status=[('56'*16,2)]
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)


def test_stationary_completed_proof_revoked_for_recenter(monkeypatch,tmp_path):
    h,_=robot(monkeypatch,tmp_path,goal=True)
    old=h.op.automatic.next_intent;hold=[True]
    def delay(core,now,**kw):
        intent=old(core,now,**kw)
        return None if intent=='PULSE' and hold[0] else intent
    monkeypatch.setattr(h.op.automatic,'next_intent',delay)
    stationary(h)
    h.until(lambda r:r['phase']=='WAIT_P' and r['p_admission_blocker'] is None)
    assert h.by_name['fire_pulse_controller'].pub is not None
    yaw=h.yaw;h.centered=False
    h.until(lambda r:h.yaw>yaw+.025,limit=120)
    assert h.on_at is None
    h.centered=True;hold[0]=False
    h.until(lambda r:h.home==1,limit=300)
    assert len(h.pulses)==1 and h.x==0


def test_stationary_actor_temporary_missing_preview_restarts_confirmation(monkeypatch,tmp_path):
    h,_=robot(monkeypatch,tmp_path,goal=True)
    h.right=950;h.fire=True;h.ir_flame=True
    h.until(lambda r:abs(h.yaw)>.025);h.centered=True
    dropping=[True]
    h.bus.drop=lambda p,s,m:dropping[0] and s.info.node_name=='fire_pulse_controller' and p.topic=='/yolo/alignment_preview'
    h.until(lambda r:h.op.core.pending_stationary)
    for _ in range(12):h.step()
    assert h.on_at is None
    dropping[0]=False
    h.until(lambda r:h.home==1,limit=300)
    assert len(h.pulses)==1


@pytest.mark.parametrize('failure',['serial_missing','epoch_change'])
def test_interrupt_never_releases_without_current_device_STOP(monkeypatch,tmp_path,failure):
    import copy,json
    from std_msgs.msg import String
    h,_=robot(monkeypatch,tmp_path,goal=True);stage(h,'IR')
    h.step('X');h.silence_serial=True
    if failure=='epoch_change':
        identity=copy.deepcopy(h.op.core.inputs['identity'][0]);identity['connection_epoch']+=1
        h.external['/fire_sensors/serial_session_identity'].publish(String(data=json.dumps(identity)));h.bus.flush()
    for _ in range(90):r=h.step()
    assert not r['handback']['consumed'] and h.on_at is None
    assert not any(r['intents'].values())
    assert not r['closed']


def test_new_goal_during_IR_interrupts_old_attempt_without_pump(monkeypatch,tmp_path):
    h,_=robot(monkeypatch,tmp_path,goal=True);stage(h,'IR')
    h.status.append(('56'*16,2))
    h.until(lambda r:r['handback']['consumed'],limit=200)
    assert h.op.core.interruption_reason=='NEW_GOAL_REQUESTED' and h.on_at is None
    assert any(uid=='56'*16 for at,uid in h.bus.cancelled)
    h.status=[('78'*16,2)]
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)


def test_automatic_view_has_one_task_outcome():
    from fire_robot_mission.operational_view import status_lines
    r=dict(armed=True,automatic={'enabled':True},task=dict(stage='RETURN_NAV2',outcome='INTERRUPTED',reason='IR_TASK_TIMEOUT'),
        phase='CANCEL_WAIT_CLEAR',reason='legacy_detail',interruption_reason='IR_TASK_TIMEOUT',retry_fire_clear_required=True)
    lines=status_lines(r)
    assert lines[1]=='TRA NAV2 | BI NGAT'
    assert not any(line.startswith(('GUARD ','KEY ')) for line in lines)
    assert any('IR_TASK_TIMEOUT' in line for line in lines)


@pytest.mark.parametrize('bad',[-1,float('nan'),True])
def test_task_timeout_parameters_reject_invalid(bad):
    from fire_robot_mission.automatic_policy import AutomaticPolicy
    with pytest.raises(ValueError):AutomaticPolicy(True,alignment_timeout_s=bad)


def test_stationary_count_and_reason_show_actor_confirmation(monkeypatch,tmp_path):
    h,_=robot(monkeypatch,tmp_path,goal=True)
    old=h.op.automatic.next_intent
    monkeypatch.setattr(h.op.automatic,'next_intent',lambda core,now,**kw:
        None if (intent:=old(core,now,**kw))=='PULSE' else intent)
    stationary(h)
    r=h.until(lambda r:r['phase']=='WAIT_P' and r['p_admission_blocker'] is None)
    assert r['range_confirm_samples']==5
    assert r['stationary_confirmation_reason']=='FIVE_FRESH_IR_SAMPLES_CENTERED_STOP'
    assert h.on_at is None


def test_interruption_view_never_promises_new_goal_before_release():
    from fire_robot_mission.operational_view import status_lines
    r=dict(armed=True,automatic={'enabled':True},task={'stage':'RETURN_NAV2','outcome':'INTERRUPTED'},
        phase='CANCEL_WAIT_CLEAR',cycle={'cycle_id':'a'*32},retry_fire_clear_required=True)
    assert not any('NHAN GOAL MOI' in x for x in status_lines(r))
    r.update(cycle=None,phase='ARMED_IDLE')
    assert any('NHAN GOAL MOI' in x for x in status_lines(r))
