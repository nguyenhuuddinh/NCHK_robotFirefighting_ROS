import json
import pytest
from harness import Robot
from test_full_chain import returned,old_cores,align

CONTROL='/fire_mission/cycle_control'
ROLES=['fire_mission_supervisor','live_nav_stop_gateway','nav2_cancel_guard',
       'live_alignment_takeover','ir_approach_live','fire_pulse_controller']


@pytest.mark.parametrize('late',ROLES)
def test_actual_participant_late_COMMIT_never_reports_ready_from_five_ACKs(monkeypatch,late):
    h=Robot(monkeypatch,goal=late in ('nav2_cancel_guard','fire_pulse_controller'))
    h.arm();cores=old_cores(h);h.fire=True;h.step('F');h.until(lambda r:r['phase']=='WAIT_T')
    blocked=[True]
    def drop(p,s,msg):
        return (blocked[0] and p.topic==CONTROL and s.info.node_name==late
                and json.loads(msg.data).get('phase')=='COMMIT')
    h.bus.drop=drop
    h.step('X');h.fire=False
    h.until(lambda r:r['handback'].get('release',{}).get('phase')=='COMMIT')
    for _ in range(20):
        r=h.step()
        assert not r['handback']['consumed'] and not r['intents']['navigation'] and r['fault'] is None
    participant=h.by_name[late]._cycle.participant
    assert participant.phase!='COMMITTED' and getattr(h.by_name[late],h.by_name[late]._cycle.core_attr) is cores[late]
    blocked[0]=False
    r=h.until(lambda r:r['handback']['consumed'])
    returned(h,r,cores)


def test_canceled_goal_status_pruned_after_real_proof_still_allows_T_I_and_P(monkeypatch):
    h=Robot(monkeypatch,goal=True);h.arm();cores=old_cores(h);h.fire=True;h.step('F')
    h.until(lambda r:r['phase']=='WAIT_T')
    assert h.bus.cancelled and h.status==[('a'*32,5)]
    h.status=[]
    # Nav2 status arrays can prune old terminal records. Fresh actual guard
    # still carries its exact response and observed CANCELED result.
    h.until(lambda r:r['phase']=='WAIT_T',limit=10)
    assert h.step('T')['operator_feedback']['accepted']
    h.until(lambda r:abs(h.yaw)>.025);h.centered=True
    h.until(lambda r:r['phase']=='WAIT_IR_I');h.right=950
    h.until(lambda r:r['ir_band']=='SPRAY_RANGE');h.step('I')
    h.until(lambda r:r['phase']=='WAIT_P');h.step('P');h.until(lambda r:h.home==1)
    h.fire=False;r=h.until(lambda r:r['handback']['consumed']);returned(h,r,cores)


@pytest.mark.parametrize('goal',[False,True])
def test_target_loss_during_T_holds_zero_and_reacquires_same_cycle_without_new_T(monkeypatch,goal):
    h=Robot(monkeypatch,goal=goal);h.arm();h.fire=True;h.ir_flame=True
    h.step('F');h.until(lambda r:r['phase']=='WAIT_T');h.step('T')
    h.until(lambda r:abs(h.yaw)>.025)
    cycle=h.op.core.cycles.current
    h.fire=False
    for _ in range(16):
        r=h.step();assert not r['closed'] and r['fault'] is None
    yaw=h.yaw
    for _ in range(5):h.step()
    assert h.yaw==yaw and h.op.core.cycles.current==cycle and r['phase']=='WAIT_TARGET'
    h.fire=True
    h.until(lambda r:h.yaw>yaw+.025)
    assert h.op.core.cycles.current==cycle and h.op.core.turn_permission
    h.centered=True;h.until(lambda r:r['phase']=='WAIT_IR_I')
    h.step('X');h.fire=False;h.ir_flame=False
    h.until(lambda r:r['handback']['consumed'])


@pytest.mark.parametrize('goal',[False,True])
def test_same_epoch_identity_gap_during_IR_holds_zero_then_resumes_original_I(monkeypatch,goal):
    h=Robot(monkeypatch,goal=goal);h.arm();align(h);h.step('I')
    h.until(lambda r:h.x>.025)
    cycle=h.op.core.cycles.current;start=h.op.core.cycles.budget.requested_at
    h.identity_gap=True
    h.until(lambda r:r['identity_hold'],limit=16)
    for _ in range(12):
        r=h.step();assert r['identity_hold'] and r['fault'] is None and r['armed'] and not r['closed']
    x=h.x
    for _ in range(4):h.step()
    assert h.x==x
    h.identity_gap=False
    h.until(lambda r:not r['identity_hold'])
    h.until(lambda r:h.x>x+.01)
    assert h.op.core.cycles.current==cycle and h.op.core.cycles.budget.requested_at==start
    h.right=950;h.until(lambda r:r['phase']=='WAIT_P')
    h.step('X');h.fire=False;h.until(lambda r:r['handback']['consumed'])


@pytest.mark.parametrize('goal',[False,True])
def test_visual_loss_during_IR_releases_realigns_then_resumes_same_I(monkeypatch,goal):
    h=Robot(monkeypatch,goal=goal);h.arm();align(h);h.step('I');h.until(lambda r:h.x>.025)
    cycle=h.op.core.cycles.current;start=h.op.core.cycles.budget.requested_at
    h.ir_flame=True;h.fire=False
    h.until(lambda r:r['phase']=='WAIT_TARGET')
    x=h.x;yaw=h.yaw
    assert not any(p.topic=='/cmd_vel_raw' for p in h.bus.pubs)
    h.fire=True;h.centered=False
    h.until(lambda r:h.yaw>yaw+.02)
    h.centered=True
    h.until(lambda r:h.x>x+.01)
    assert h.op.core.cycles.current==cycle and h.op.core.cycles.budget.requested_at==start
    assert h.by_name['ir_approach_live'].core.step_count>=1
    h.right=950;h.until(lambda r:r['phase']=='WAIT_P')
    h.step('X');h.fire=False;h.ir_flame=False;h.until(lambda r:r['handback']['consumed'])


@pytest.mark.parametrize('goal',[False,True])
def test_true_clear_during_IR_returns_Nav2_without_requiring_X_or_any_P(monkeypatch,goal):
    h=Robot(monkeypatch,goal=goal);h.arm();cores=old_cores(h);align(h);h.step('I')
    h.until(lambda r:h.x>.025);h.fire=False
    r=h.until(lambda r:r['handback']['consumed'])
    assert h.on_at is None
    returned(h,r,cores)


@pytest.mark.parametrize('bad',['active_other','contradictory_terminal','missing_real_cancel'])
def test_pruned_status_policy_never_substitutes_for_real_cancel_proof(monkeypatch,bad):
    h=Robot(monkeypatch,goal=True);h.arm();h.fire=True;h.step('F')
    if bad=='missing_real_cancel':
        # Real Nav2 has not acknowledged/finished the cancel. A pruned array
        # cannot turn an unobserved result into CANCELED.
        h.status=[]
    else:
        h.until(lambda r:r['phase']=='WAIT_T')
        h.status=[('56'*16,2)] if bad=='active_other' else [('a'*32,4)]
    for _ in range(18):
        r=h.step('T');assert not r['intents']['alignment'] and not r['intents']['pulse'] and h.on_at is None
    assert not any(p.info.node_name=='live_alignment_takeover' and p.topic=='/cmd_vel_raw' for p in h.bus.pubs)


@pytest.mark.parametrize('terminal',[4,5,6])
def test_normal_navigation_terminal_releases_and_accepts_next_goal_in_same_session(monkeypatch,terminal):
    h=Robot(monkeypatch,goal=True);h.arm()
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    h.status=[('a'*32,terminal)]
    h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL')
    h.status.append(('56'*16,2))
    r=h.step();assert r['phase']=='NAVIGATION' and r['fault'] is None and not r['closed']
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    assert not h.bus.cancelled and h.on_at is None


def test_after_actual_six_resets_a_second_fire_cycle_can_T_I_P_and_retire(monkeypatch):
    from test_full_chain import cancel_cycle
    h=Robot(monkeypatch,goal=True);cancel_cycle(h)
    previous=h.op.handback.active.copy();cores=old_cores(h)
    h.fire=True;h.step('F');h.until(lambda r:r['phase']=='WAIT_T')
    assert h.op.core.cycles.current.cycle_id!=previous['cycle_id']
    assert h.op.core.cycles.current.nav_goal_uuid=='56'*16
    h.step('T');h.until(lambda r:abs(h.yaw)>.025);h.centered=True
    h.until(lambda r:r['phase']=='WAIT_IR_I');h.right=950
    h.until(lambda r:r['ir_band']=='SPRAY_RANGE');h.step('I');h.until(lambda r:r['phase']=='WAIT_P')
    h.step('P');h.until(lambda r:h.home==1);h.fire=False
    r=h.until(lambda r:r['handback']['consumed'])
    # Verify the next UUID rather than reusing a retired goal.
    for n in h.nodes:
        if hasattr(n,'_cycle'):
            assert n._cycle.participant.phase=='COMMITTED'
            assert getattr(n,n._cycle.core_attr) is not cores[n.test_name]
    h.status=[('78'*16,2)];assert h.step()['phase']=='NAVIGATION'
    assert [uid for _,uid in h.bus.cancelled]==['a'*32,'56'*16]


@pytest.mark.parametrize('goal',[False,True])
def test_I_expires_while_identity_held_releases_without_new_forward_budget(monkeypatch,goal):
    h=Robot(monkeypatch,goal=goal);h.arm();align(h);h.step('I');h.until(lambda r:h.x>.025)
    start=h.op.core.cycles.budget.requested_at;cycle=h.op.core.cycles.current
    h.identity_gap=True;h.until(lambda r:r['identity_hold'])
    h.until(lambda r:r['budget']['exhausted'],limit=340)
    x=h.x;h.identity_gap=False
    h.until(lambda r:not r['identity_hold'])
    h.until(lambda r:not any(p.topic=='/cmd_vel_raw' for p in h.bus.pubs))
    for _ in range(12):
        r=h.step();assert r['budget']['exhausted'] and not r['intents']['ir'] and not r['closed'] and r['fault'] is None
    assert h.x==x and h.op.core.cycles.current==cycle and h.op.core.cycles.budget.requested_at==start
    h.step('X');h.fire=False;h.until(lambda r:r['handback']['consumed'])


def test_used_P_resets_only_after_six_ACKs_then_a_second_explicit_I_P_cycle_works(monkeypatch):
    from test_full_chain import pulse_cycle
    h=Robot(monkeypatch);pulse_cycle(h,stationary=True)
    assert len(h.pulses)==1 and not h.by_name['fire_pulse_controller'].session.used
    # First handback already accepted a new real goal. New F/T/I/P must be
    # explicit; past P authority and I request are never inherited.
    h.fire=True;h.centered=False;h.right=5000;h.step('F');h.until(lambda r:r['phase']=='WAIT_T')
    r=h.step();assert not r['intents']['ir'] and not r['intents']['pulse'] and r['budget']['requested_at'] is None
    yaw=h.yaw;h.step('T');h.until(lambda r:h.yaw>yaw+.02);h.centered=True
    h.until(lambda r:r['phase']=='WAIT_IR_I');h.right=950;h.until(lambda r:r['ir_band']=='SPRAY_RANGE')
    h.step('I');h.until(lambda r:r['phase']=='WAIT_P');h.step('P')
    h.until(lambda r:h.home==2);h.fire=False;r=h.until(lambda r:r['handback']['consumed'])
    assert len(h.pulses)==2 and not r['closed'] and r['reason']=='NAV2_READY_NEW_GOAL'
    assert [p['home_after'] for p in h.pulses]==[1,2]
    assert all(p['off_at']-p['on_at']<=5.1 for p in h.pulses)
