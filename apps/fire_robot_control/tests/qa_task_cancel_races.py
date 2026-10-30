"""Goals arriving during fire cancellation; real app nodes, no hardware."""
import pytest
from qa_app_operator_timing import app_class
from qa_full_operator import simulated_camera_and_no_ros_context
from qa_x_cycle_audit import at_stage, wait, compact


@pytest.mark.parametrize('stage', ['detected', 'turn', 'ir', 'pump', 'home'])
@pytest.mark.parametrize('terminal', [4, 5, 6])
def test_foreign_goal_during_X_is_drained_before_next_goal(monkeypatch, tmp_path, app_class, stage, terminal):
    h = at_stage(monkeypatch, tmp_path, stage, True)
    h.fire = h.ir_flame = False
    # Before fire-cancel completes, Nav2 may preempt/finish the original Goal.
    # Once the guard has really canceled, keep its actual CANCELED result.
    old = next(status for uid, status in h.status if uid == 'a'*32)
    h.status = [('a'*32, terminal if old in (1, 2, 3) else old), ('34'*16, 2)]
    assert h.step('X')['operator_feedback']['accepted']
    wait(h, lambda r: r['cycle'] is None and h.op.core._rest(h.bus.now), limit=420)
    assert all(status in (4, 5, 6) for uid, status in h.status), compact(h, h.reports[-1])
    for _ in range(30): h.step()
    h.status.append(('56'*16, 2))
    x = h.x
    wait(h, lambda r: h.x > x+.01)
    assert h.by_name['live_nav_stop_gateway'].core.goal_uuid == '56'*16


def test_X_during_missing_gateway_report_remembers_recovery_request(monkeypatch, tmp_path, app_class):
    from qa_goal_retry_audit import gap_robot
    h = gap_robot(monkeypatch, tmp_path)
    h.status = [('a'*32, 6), ('34'*16, 2)]
    for _ in range(5): h.step()
    held = [True]
    h.bus.drop = lambda p, s, m: (held[0] and p.topic == '/fire_mission/live_nav_stop_state'
                                and s.info.node_name == 'fire_robot_operator')
    for _ in range(20): h.step()
    assert h.step('X')['operator_feedback']['accepted']
    for _ in range(15): h.step()
    held[0] = False
    wait(h, lambda r: r['reason'] == 'NAV2_READY_NEW_GOAL', limit=220)
    h.status.append(('56'*16, 2))
    x = h.x
    wait(h, lambda r: h.x > x+.01)
