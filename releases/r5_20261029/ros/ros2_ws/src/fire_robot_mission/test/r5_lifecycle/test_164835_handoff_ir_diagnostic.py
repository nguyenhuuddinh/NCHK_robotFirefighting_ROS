"""164835 delta: slow cancel must not consume target recovery; IR proof is observable."""
from types import SimpleNamespace

from fire_robot_mission.automatic_policy import AutomaticPolicy
from fire_robot_mission.operational_session import stationary_proof_snapshot
from fire_robot_mission.operational_view import status_lines


class HandoffCore:
    def __init__(self):
        self.cycles=SimpleNamespace(current=SimpleNamespace(nav_goal_uuid='a'*32),
                                    budget=SimpleNamespace(requested_at=None,TIME_S=15.))
        self.cancel_requested=False;self.p_used=False;self.handoff=False;self.fire=False
        self.interrupted=[]

    def _handoff(self, now):return self.handoff
    def _fire(self, now):return self.fire
    def get(self, name, now):return {}
    def _status(self, now):return None
    def interrupt_cycle(self, reason):self.interrupted.append(reason)


def test_target_clock_starts_only_after_slow_cancel_handoff():
    core=HandoffCore();policy=AutomaticPolicy(True)
    # More than the old 3 s target timeout and even the 30 s alignment task
    # may pass while the exact goal cancel/STOP handoff is still pending.
    for now in (1.,2.,4.1,31.5):policy.finish_incomplete_stage(core,now)
    assert not core.interrupted and policy.target_lost_since is None
    core.handoff=True
    policy.finish_incomplete_stage(core,32.)
    assert policy.handoff_at==policy.cycle_started==policy.target_lost_since==32.
    policy.finish_incomplete_stage(core,34.9)
    assert not core.interrupted
    core.fire=True;policy.finish_incomplete_stage(core,35.)
    assert policy.target_lost_since is None and not core.interrupted
    core.fire=False;policy.finish_incomplete_stage(core,36.)
    policy.finish_incomplete_stage(core,39.1)
    assert core.interrupted==['ALIGN_TARGET_NOT_RECOVERED']


def test_after_handoff_timer_keeps_running_when_raw_alignment_takes_owner():
    core=HandoffCore();policy=AutomaticPolicy(True,target_recovery_s=3.)
    core.handoff=True;policy.finish_incomplete_stage(core,1.)
    core.handoff=False
    policy.finish_incomplete_stage(core,4.1)
    assert core.interrupted==['ALIGN_TARGET_NOT_RECOVERED']


def test_stationary_proof_snapshot_and_camera_reason_are_diagnostic_only():
    core=SimpleNamespace(inputs={'actor':({'state':'WAIT_I'},10.)},
        get=lambda name, now: {'actor':dict(state='WAIT_I',blockers=['IR_STALE'],
            stationary_confirmation=dict(state='CONFIRMING',reason='WAIT_NEW_WINDOW:IR_STALE',sample_count=0)),
            'ir':dict(ir_band='SPRAY_RANGE',ir_sample_seq=42,blockers=['IR_STALE'])}.get(name))
    report=dict(phase='IR_CONFIRM',ir_source_age_ms=327.7,
        intents=dict(stationary_confirmation=True),automatic=dict(enabled=True),
        task=dict(stage='IR',outcome='RUNNING',reason='WAIT_FIVE_NEW_IR_SAMPLES'))
    diagnostic=stationary_proof_snapshot(core,report,10.2)
    assert diagnostic['request_active'] is True
    assert diagnostic['ir_sample_seq']==42 and diagnostic['ir_source_age_ms']==327.7
    assert diagnostic['sample_count']==0 and diagnostic['first_actor_blocker']=='IR_STALE'
    assert diagnostic['actor_receive_age_ms']==200.
    report['stationary_proof']=diagnostic
    assert any('WAIT_NEW_WINDOW:IR_STALE' in row for row in status_lines(report))
    assert stationary_proof_snapshot(core,dict(phase='NAVIGATION'),10.2) is None


def test_live_adapter_stationary_log_is_rate_limited_without_pump(monkeypatch,tmp_path):
    from test_automatic_065731 import robot
    h,_=robot(monkeypatch,tmp_path,goal=True)
    h.right=950;h.fire=True;h.ir_flame=True
    h.until(lambda r:abs(h.yaw)>.025)
    h.centered=True
    h.bus.drop=lambda p,s,m:(p.topic=='/yolo/alignment_preview'
                             and s.info.node_name=='fire_pulse_controller')
    h.until(lambda r:h.op.core.pending_stationary)
    for _ in range(27):h.step()
    logs=[(at,str(args)) for at,who,args in h.bus.logs
          if who=='fire_robot_operator' and 'OPERATOR_STATIONARY_PROOF ' in str(args)]
    assert 1<=len(logs)<=3,logs
    assert all(b[0]-a[0]>=.95 for a,b in zip(logs,logs[1:])),logs
    assert h.on_at is None and not h.op.core.p_used
