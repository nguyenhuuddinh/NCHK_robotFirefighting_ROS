"""Cancel transport recovery audit on app nodes; no ROS init or real actions."""
from types import SimpleNamespace as N

import pytest

from qa_app_operator_timing import app_class
from qa_full_operator import simulated_camera_and_no_ros_context
from qa_goal_retry_audit import ui_goal_permission
from qa_x_cycle_audit import at_stage, wait, compact, evidence, verify_Q


@pytest.mark.parametrize('failure', ['service_down', 'send_failed', 'rejected', 'response_lost'])
def test_X_cancel_transport_recovery_and_Q_escape(monkeypatch, tmp_path, app_class, failure):
    h = at_stage(monkeypatch, tmp_path, 'nav', True)
    client = h.by_name['nav2_cancel_guard'].cancel_client
    original_call = client.call_async
    original_ready = client.service_is_ready
    ready = [False]
    monkeypatch.setattr(client, 'service_is_ready', lambda: ready[0])
    calls = []
    def call(request):
        calls.append(bytes(request.goal_info.goal_id.uuid).hex())
        if failure == 'send_failed':
            raise RuntimeError('SYNTHETIC_CANCEL_TRANSPORT')
        if failure == 'rejected':
            return N(done=lambda: True, result=lambda: N(return_code=1, goals_canceling=[]))
        if failure == 'response_lost':
            return N(done=lambda: False)
        return original_call(request)
    monkeypatch.setattr(client, 'call_async', call)
    assert h.step('X')['operator_feedback']['accepted']
    for _ in range(25): h.step()
    assert not calls
    ready[0] = True
    for _ in range(85): report = h.step()
    if failure == 'service_down':
        wait(h, lambda r: r['reason'] == 'NAV2_READY_NEW_GOAL')
        h.status.append(('56'*16, 2))
        wait(h, lambda r: h.by_name['live_nav_stop_gateway'].core.goal_uuid == '56'*16
             and any(p.topic == '/cmd_vel_raw' for p in h.bus.pubs))
        assert calls == ['a'*32]
    else:
        monkeypatch.setattr(client, 'call_async', original_call)
        for _ in range(10): h.step('X')
        for _ in range(60): report = h.step()
        row = dict(failure=failure, calls=calls, after=compact(h, report),
                   ui_goal_permission=ui_goal_permission(report))
        assert calls == ['a'*32], row
        assert report['phase'] == 'NAV_CANCEL_WAIT' and not row['ui_goal_permission'], row
        assert not report['closed'] and report['fault'] is None, row
        row['Q_after_wait'] = verify_Q(h)
        evidence(f'cancel_transport_{failure}', row)
        assert row['Q_after_wait']['confirmed'], row
