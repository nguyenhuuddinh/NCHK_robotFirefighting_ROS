"""Actual operator main/finally; fake context, signal delivery and device I/O."""
import signal
import pytest
from types import SimpleNamespace as N
from harness import Robot
from fire_robot_mission import operational_session as ros


@pytest.mark.parametrize('cause',['Q','SIGINT','exception','stale_ACT','context_lost','view_close_exception'])
def test_main_drains_before_destroy_and_context_shutdown_reports_unconfirmed_honestly(monkeypatch,cause):
    h=Robot(monkeypatch);h.arm();events=[];handlers={};context=[False];calls=[0];view_key=[True]
    monkeypatch.setattr(ros,'OperationalSessionNode',lambda:h.op)
    monkeypatch.setattr(ros.signal,'getsignal',lambda s:'saved')
    monkeypatch.setattr(ros.signal,'signal',lambda s,fn:handlers.update({s:fn}))
    monkeypatch.setattr(ros.rclpy,'init',lambda **kw:context.__setitem__(0,True))
    monkeypatch.setattr(ros.rclpy,'ok',lambda:context[0])
    def shutdown():events.append(('context_shutdown',h.bus.now));context[0]=False
    monkeypatch.setattr(ros.rclpy,'shutdown',shutdown)
    def key(report):
        result='Q' if view_key[0] else None;view_key[0]=False;return result
    def close_view():
        events.append(('view_close',h.bus.now))
        if cause=='view_close_exception':raise RuntimeError('fake GUI close failure')
    h.op.view=N(tick=key,close=close_view)
    def spin(node,timeout_sec):
        calls[0]+=1
        if cause=='exception' and calls[0]==1:raise RuntimeError('fake child/runtime exception')
        h.bus.advance();h.inputs();h.bus.flush()
        for n in h.nodes:
            for callback in n.test_timers:callback()
            h.bus.flush()
        if cause=='SIGINT' and calls[0] in (1,3):handlers[signal.SIGINT](None,None)
        if cause=='context_lost' and calls[0]==1:context[0]=False
        if cause=='stale_ACT':
            h.bus.drop=lambda p,s,m:p.topic=='/fire_sensors/actuator_state'
    monkeypatch.setattr(ros.rclpy,'spin_once',spin)
    def destroy():
        if cause in ('Q','SIGINT','exception','view_close_exception'):assert h.op.stop_verification.done and h.op.stop_verification.confirmed
        else:assert not h.op.stop_verification.confirmed
        events.append(('node_destroy',h.bus.now))
    monkeypatch.setattr(h.op,'destroy_node',destroy)
    rc=ros.main()
    assert rc==(0 if cause in ('Q','SIGINT') else 2)
    assert any(e[0]=='node_destroy' for e in events)
    if cause!='context_lost':assert [e[0] for e in events][-2:]==['node_destroy','context_shutdown']
    if cause=='stale_ACT':
        assert h.op.stop_verification.done and h.op.stop_verification.reason=='STOP_UNCONFIRMED'
        assert h.op.stop_verification.last_at-h.op.stop_verification.start>=4.
    assert not h.op.core.armed and h.op.intent_executor.closed
    assert not context[0]

    if cause=='context_lost':
        assert h.op.stop_verification.done and h.op.stop_verification.reason=='STOP_UNCONFIRMED'
        assert 'ROS_CONTEXT_LOST' in h.op.stop_verification.blockers
