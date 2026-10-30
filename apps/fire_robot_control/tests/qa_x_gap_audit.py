"""X across recoverable telemetry gaps; real nodes, no ROS context or robot."""
import pytest

from qa_app_operator_timing import app_class
from qa_full_operator import simulated_camera_and_no_ros_context
from qa_x_cycle_audit import at_stage, compact, evidence


TOPICS = {
    'vision': {'/yolo/detections', '/yolo/alignment_preview', '/yolo/image_annotated/compressed'},
    'identity': {'/fire_sensors/serial_session_identity'},
    'odom': {'/odom'},
    'actuator': {'/fire_sensors/actuator_state'},
    'ir': {'/fire_sensors/ir_raw', '/fire_mission/ir_range_preview'},
}


@pytest.mark.parametrize('stage', ['nav', 'turn', 'ir', 'pump'])
@pytest.mark.parametrize('gap', list(TOPICS))
def test_X_during_gap_then_fresh_data_allows_new_goal(monkeypatch, tmp_path, app_class, stage, gap):
    h = at_stage(monkeypatch, tmp_path, stage, True)
    held = [True]
    h.bus.drop = lambda p, s, m: held[0] and p.topic in TOPICS[gap]
    for _ in range(16): report = h.step()
    before = compact(h, report)
    pulse_count = len(h.pulses)
    h.fire = h.ir_flame = False
    report = h.step('X')
    feedback = report['operator_feedback']
    for _ in range(8): h.step()
    held[0] = False
    recovered = False
    for _ in range(400):
        report = h.step()
        if (report['cycle'] is None and report['phase'] == 'ARMED_IDLE'
                and not any(report.get(k) for k in ('identity_hold', 'input_hold', 'graph_hold'))
                and h.op.core._rest(h.bus.now)):
            recovered = True
            break
        if report['closed'] or report['fault']:
            break
    row = dict(stage=stage, gap=gap, held_s=1.2, before_x=before, feedback=feedback,
               after=compact(h, report), retired=recovered, next_goal_forwarded=False)
    if recovered:
        for _ in range(30): h.step()
        h.status = [(uid, status) for uid, status in h.status if status in (4, 5, 6)] + [('56'*16, 2)]
        for _ in range(100):
            report = h.step()
            core = h.by_name['live_nav_stop_gateway'].core
            command = h.bus.last.get('/cmd_vel_raw')
            if (core.goal_uuid == '56'*16 and command is not None and command.linear.x > 0
                    and any(p.topic == '/cmd_vel_raw' and p.info.node_name == 'live_nav_stop_gateway' for p in h.bus.pubs)):
                row['next_goal_forwarded'] = True
                break
        row['after_goal'] = compact(h, report)
    row['new_pulses_after_x'] = len(h.pulses) - pulse_count
    evidence(f'gap_{stage}_{gap}', row)
    assert row['new_pulses_after_x'] == 0 and h.bus.max_raw <= 1, row
    assert row['next_goal_forwarded'], row
