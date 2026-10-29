import pytest
from harness import Robot


def returned(h,r,old_cores):
    assert r['reason']=='NAV2_READY_NEW_GOAL' and r['cycle'] is None and not r['closed']
    for n in h.nodes:
        if not hasattr(n,'_cycle'):continue
        assert n._cycle.participant.phase=='COMMITTED'
        assert getattr(n,n._cycle.core_attr) is not old_cores[n.test_name]
        assert n._cycle.participant.report([])['reset_completed']
    assert not any(p.topic in ('/cmd_vel_raw','/pump_cmd') for p in h.bus.pubs)
    assert h.bus.max_raw<=1
    h.status=[('56'*16,2)]
    assert h.step()['phase']=='NAVIGATION'
    h.until(lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)


def old_cores(h):return {n.test_name:getattr(n,n._cycle.core_attr) for n in h.nodes if hasattr(n,'_cycle')}


def cancel_cycle(h):
    h.arm();cores=old_cores(h);h.fire=True
    h.step('F');h.until(lambda r:r['phase']=='WAIT_T')
    assert h.step('X')['operator_feedback']['accepted']
    h.fire=False
    r=h.until(lambda r:r['handback']['consumed'])
    returned(h,r,cores)
    return r


def align(h):
    h.fire=True;h.step('F');h.until(lambda r:r['phase']=='WAIT_T')
    assert h.step('T')['operator_feedback']['accepted']
    h.until(lambda r:abs(h.yaw)>.025)
    h.centered=True
    return h.until(lambda r:r['phase']=='WAIT_IR_I')


def pulse_cycle(h,*,stationary=False):
    h.arm();cores=old_cores(h);align(h)
    if stationary:
        h.right=950
        h.until(lambda r:r['ir_band']=='SPRAY_RANGE')
    assert h.step('I')['operator_feedback']['accepted']
    start=h.op.core.cycles.budget.requested_at
    if not stationary:
        h.until(lambda r:h.x>.04);h.right=950
    h.until(lambda r:r['phase']=='WAIT_P')
    assert h.step('P')['operator_feedback']['accepted']
    h.until(lambda r:h.home==1)
    h.fire=False
    r=h.until(lambda r:r['handback']['consumed'])
    assert h.on_at is not None and h.off_at-h.on_at<=5.1
    assert (h.x==0. if stationary else .04<h.x<.30)
    assert all(q['budget']['requested_at'] in (None,start) for q in h.reports if q['budget'] is not None)
    returned(h,r,cores)
    return r


def test_no_goal_X_clear_resets_six_real_nodes_then_accepts_new_goal(monkeypatch):
    h=Robot(monkeypatch);cancel_cycle(h)
    assert not h.bus.cancelled


@pytest.mark.parametrize('goal,reverse',[(True,False),(False,True),(True,True)])
def test_X_cancel_other_orders_and_real_active_UUID(monkeypatch,goal,reverse):
    h=Robot(monkeypatch,goal=goal,reverse=reverse);cancel_cycle(h)
    assert [uid for _,uid in h.bus.cancelled]==(['a'*32] if goal else [])


def test_no_goal_T_real_IR_approach_P_review_six_resets_and_new_goal(monkeypatch):
    h=Robot(monkeypatch);pulse_cycle(h)
    assert not h.bus.cancelled and h.bus.max_raw==1


@pytest.mark.parametrize('goal,reverse,stationary',[
    (True,False,False),(False,True,False),(True,True,False),
    (False,False,True),(True,False,True),(False,True,True),(True,True,True)])
def test_P_actual_six_adapters_approach_or_stationary_other_orders(monkeypatch,goal,reverse,stationary):
    h=Robot(monkeypatch,goal=goal,reverse=reverse);pulse_cycle(h,stationary=stationary)
    assert [uid for _,uid in h.bus.cancelled]==(['a'*32] if goal else [])
