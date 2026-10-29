"""New regression: target shifts after IR stops IN spray range, before P."""
import pytest
from test_runtime_parity_042557 import scheduled_robot,turn


@pytest.mark.parametrize('goal,expire',[(False,False),(True,False),(False,True),(True,True)])
def test_spray_stop_target_shift_recenter_same_I_no_extra_translation(monkeypatch,goal,expire):
    h=scheduled_robot(monkeypatch,goal=goal,phase=1)
    turn(h)
    assert h.step('I')['operator_feedback']['accepted']
    h.until(lambda r:h.x>.03)
    h.right=950
    h.until(lambda r:h.by_name['ir_approach_live'].core.phase=='COMPLETE')
    ir=h.by_name['ir_approach_live'].core
    assert ir.stop_reason=='SPRAY_RANGE_REACHED'
    cycle=h.op.core.cycles.current;requested=h.op.core.cycles.budget.requested_at
    x=h.x; yaw=h.yaw
    h.centered=False
    if expire:
        # The maximum I budget is consumed by waiting for the target; no
        # translation can be renewed by recentering or repeated I keys.
        h.ir_flame=True;h.fire=False
        h.until(lambda r:r['budget']['exhausted'],limit=340)
        h.fire=True
    h.until(lambda r:h.yaw>yaw+.025,limit=150)
    assert h.x==x and h.op.core.cycles.current==cycle
    h.centered=True
    h.until(lambda r:r['phase']=='WAIT_P' and r['p_admission_blocker'] is None,limit=150)
    assert h.x==x and h.op.core.cycles.budget.requested_at==requested
    assert h.bus.max_raw==1 and not h.op.core.closed
    assert h.step('P')['operator_feedback']['accepted']
    h.until(lambda r:h.home==1)


@pytest.mark.parametrize('bad',['actuator_on','actuator_stale','ir_foreign','ir_source','pump_owner'])
def test_spray_recenter_keeps_stop_when_new_rearm_proof_invalid(monkeypatch,bad):
    import copy
    h=scheduled_robot(monkeypatch,phase=1)
    turn(h);h.step('I');h.until(lambda r:h.x>.03);h.right=950
    h.until(lambda r:h.by_name['ir_approach_live'].core.phase=='COMPLETE')
    turn_core=h.by_name['live_alignment_takeover'].core
    original=turn_core.step
    def invalid(now,publisher_owned=False,raw_sources=(),nav_sources=(),safety_sources=(),counts=None):
        counts=dict(counts)
        if bad=='ir_source':counts['ir_source_ok']=False
        if bad=='pump_owner':counts['pump']=1
        if bad in ('actuator_on','actuator_stale'):
            value,at=turn_core.inputs['actuator'];value=copy.deepcopy(value)
            value['pump_pwm_commanded_on']=bad=='actuator_on'
            if bad=='actuator_stale':value['receive_age_ms']=351.
            turn_core.inputs['actuator']=(value,at)
        if bad=='ir_foreign':
            value,at=turn_core.inputs['ir'];value=copy.deepcopy(value)
            value['goal_uuid']='98'*16
            turn_core.inputs['ir']=(value,at)
        return original(now,publisher_owned,raw_sources,nav_sources,safety_sources,counts)
    monkeypatch.setattr(turn_core,'step',invalid)
    h.centered=False;x=h.x;yaw=h.yaw
    for _ in range(24):h.step()
    assert h.x==x and h.yaw==yaw and h.on_at is None
    assert turn_core.phase=='COMPLETE'


def test_automatic_UI_does_not_ask_for_stage_keys():
    from fire_robot_mission.operational_view import status_lines
    rows=status_lines(dict(phase='WAIT_P',reason='FRESH_IR_STOP_CONFIRMED_PRESS_P',armed=True,
        automatic=dict(enabled=True,start_ready=True,last_intent='APPROACH',blocker=None),
        execution=dict(enabled=True)))
    text='\n'.join(rows)
    assert 'NHAN P' not in text and 'PRESS_P' not in text and 'A arm' not in text
    assert 'AUTO: SAN SANG MOT LUOT PHUN' in text and 'Q / ESC STOP' in text
