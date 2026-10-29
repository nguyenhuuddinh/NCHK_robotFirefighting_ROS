"""201558 delta: target loss/reacquisition and bounded no-goal handoff.

The six-adapter robot is synthetic: no ROS init, devices, motor or pump.
"""
import pytest

from test_automatic_065731 import robot


def until(h, predicate, limit=350):
    try:return h.until(predicate, limit=limit)
    except AssertionError:
        r=h.reports[-1]
        raise AssertionError(dict(phase=r['phase'],reason=r['reason'],status=h.status,
            cycle=r['cycle'],target=r['fire_confirmed'],
            alignment=h.by_name['live_alignment_takeover'].core.phase,
            aligned_report=(h.op.core.get('alignment',h.bus.now) or {}).get('blockers'),
            handoff=h.op.core._handoff(h.bus.now),
            gateway=(h.op.core.get('gateway',h.bus.now) or {}).get('no_goal_proof'),
            guard=(h.op.core.get('guard',h.bus.now) or {}).get('no_goal_proof'))) from None


@pytest.mark.parametrize('goal',[False, True])
def test_short_fire_loss_during_turn_resumes_same_cycle(monkeypatch,tmp_path,goal):
    h,_=robot(monkeypatch,tmp_path)
    if goal:
        h.status=[('a'*32,2)]
        until(h,lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    h.fire=True
    until(h,lambda r:r['phase']=='ALIGNING')
    first=h.op.core.cycles.current.cycle_id
    h.fire=False
    for _ in range(12):
        r=h.step()
        assert not h.op.core.cancel_requested and h.on_at is None
    h.fire=True
    until(h,lambda r:abs(h.yaw)>.025)
    assert h.op.core.cycles.current.cycle_id==first
    h.centered=True
    until(h,lambda r:r['phase']=='WAIT_IR_I' or h.op.core.cycles.budget.requested_at is not None)
    assert not r['closed'] and h.on_at is None


@pytest.mark.parametrize('goal',[False, True])
def test_long_fire_loss_releases_then_fresh_fire_can_realign(monkeypatch,tmp_path,goal):
    h,_=robot(monkeypatch,tmp_path)
    if goal:
        h.status=[('a'*32,2)]
        until(h,lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    h.fire=True
    until(h,lambda r:abs(h.yaw)>.025)
    first=h.op.core.cycles.current.cycle_id
    h.fire=False
    until(h,lambda r:h.op.core.cancel_requested,limit=130)
    r=until(h,lambda r:r['handback']['consumed'],limit=180)
    assert r['reason']=='NAV2_READY_NEW_GOAL' and h.on_at is None
    if not goal:
        # ROS action status is an event stream: no callback is guaranteed
        # between a release and the next fire with no active goal.
        old_emit=h.emit
        h.emit=lambda topic,value:(None if topic=='/navigate_to_pose/_action/status'
            else old_emit(topic,value))
    # Let the fresh-clear hold finish before offering another target.
    for _ in range(15):h.step()
    assert not h.op.core.retry_fire_clear_required
    h.fire=True
    until(h,lambda r:h.op.core.cycles.current is not None)
    assert h.op.core.cycles.current.cycle_id!=first
    until(h,lambda r:abs(h.yaw)>.05,limit=180)
    assert not r['closed'] and h.on_at is None
    h.centered=True
    h.right=950
    h.ir_flame=True
    until(h,lambda r:h.home==1,limit=300)
    assert len(h.pulses)==1 and h.off_at-h.on_at<=5.1+1e-6


@pytest.mark.parametrize('lost',[False,True])
def test_no_goal_missing_handoff_has_bounded_exit_and_new_goal(monkeypatch,tmp_path,lost):
    h,_=robot(monkeypatch,tmp_path)
    original=h.op.core._handoff
    h.op.core._handoff=lambda now,**kw:False
    if lost:
        h.ir_flame=True
        h.right=950
    h.fire=True
    until(h,lambda r:r['phase']=='WAIT_NO_GOAL_HANDSHAKE')
    if lost:
        until(h,lambda r:(h.op.core.get('ir',h.bus.now) or {}).get('ir_band')=='SPRAY_RANGE')
        h.fire=False
    until(h,lambda r:h.op.core.cancel_requested,limit=230)
    assert h.op.core.interruption_reason==(
        'ALIGN_TARGET_NOT_RECOVERED_BEFORE_HANDOFF' if lost else 'NO_GOAL_HANDOFF_TIMEOUT'), [
            (r['phase'],r['reason'],r['interruption_reason']) for r in h.reports[-6:]]
    h.op.core._handoff=original
    r=until(h,lambda r:r['handback']['consumed'],limit=180)
    assert r['reason']=='NAV2_READY_NEW_GOAL' and h.on_at is None
    h.status=[('56'*16,2)]
    until(h,lambda r:h.by_name['live_nav_stop_gateway'].core.ever_forwarded)
    assert not r['closed']
