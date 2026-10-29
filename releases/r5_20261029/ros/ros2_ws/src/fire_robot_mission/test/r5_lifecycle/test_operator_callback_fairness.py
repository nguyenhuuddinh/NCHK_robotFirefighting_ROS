"""Single-callback dispatch versus expensive GUI/graph work, without ROS I/O."""
from types import SimpleNamespace as N

from fire_robot_mission import operational_session as ros
from fire_robot_mission.operational_session_core import OperationalSessionCore


def run_busy_operator(monkeypatch, *, missing=None, signal_at=None):
    clock = N(now=1.)
    context = N(ok=False)
    handlers = {}
    core = OperationalSessionCore()
    values = {
        'cmd': [0.] * 6, 'odom': [0.] * 6,
        'actuator': dict(schema_version=1, mode='COMMAND_TELEMETRY_ONLY', esp_ms=1000,
                         receive_age_ms=20., pump_pwm_ready=True,
                         pump_pwm_commanded_on=False, servo_pwm_commanded_on=False,
                         servo_phase='IDLE', home_command_completed_count=0),
        'vision': dict(source={'age_ms': 20.}),
        'ir': dict(ir_receive_age_ms=20.),
        'ir_raw': dict(schema_version=1,valid=True,left=26000,center=26000,right=26000,
                       sample_age_ms=20.,receive_age_ms=20.),
    }
    # Independent latest-only subscription slots: no FIFO sensor replay is
    # needed to reproduce starvation. Their callbacks compete with 13 other
    # subscribed telemetry/status/parameter events.
    slots = tuple(values) + tuple('other_' + str(i) for i in range(13))
    dispatch = N(index=0, count=0)
    result = N(armed=False, blocked=False, ticks=0, spins_at_tick=[], reports=[], signal_stop=False)
    stop = N(done=False, confirmed=False)

    def request_shutdown(reason):
        result.signal_stop |= reason == 'SIGNAL'
        core.armed = False
        core.closed = True
        stop.done = stop.confirmed = True

    def spin(node, timeout_sec):
        clock.now += .0005
        name = slots[dispatch.index % len(slots)]
        dispatch.index += 1
        dispatch.count += 1
        if name in values and name != missing:
            core.update(name, values[name], clock.now)
        if signal_at is not None and clock.now >= signal_at and not result.signal_stop:
            handlers[ros.signal.SIGINT](None, None)

    def view_tick(report):
        clock.now += .015  # GUI cost; never opens an actual window.
        if report and report.get('reason') == 'READY_PRESS_A':
            return 'A'
        if result.armed or clock.now > 8.:
            return 'Q'
        return None

    def tick(key):
        clock.now += .015  # Exact graph checks and report serialization cost.
        core.update('graph', dict(valid=True, raw_count=0, raw_owner=None), clock.now)
        report = core.step(clock.now, [key] if key else ())
        result.armed |= report['armed']
        result.blocked |= bool(report['startup_readiness']['blockers'])
        result.ticks += 1
        result.spins_at_tick.append(dispatch.count)
        result.reports.append(report)
        node.last_report = report
        if report['closed']:
            request_shutdown(key or 'Q')
        if stop.done:
            report['shutdown'] = dict(done=True)
        return report

    node = N(core=core, stop_verification=stop, last_report=None,
             view=N(tick=view_tick, close=lambda: None), tick=tick,
             request_shutdown=request_shutdown, destroy_node=lambda: None,
             get_logger=lambda: N(error=lambda *a: None, info=lambda *a: None))
    stop.report = lambda now: dict(done=stop.done, confirmed=stop.confirmed)
    monkeypatch.setattr(ros, 'OperationalSessionNode', lambda: node)
    monkeypatch.setattr(ros.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(ros.signal, 'getsignal', lambda s: None)
    monkeypatch.setattr(ros.signal, 'signal', lambda s, fn: handlers.update({s: fn}))
    monkeypatch.setattr(ros.rclpy, 'init', lambda **kw: setattr(context, 'ok', True))
    monkeypatch.setattr(ros.rclpy, 'ok', lambda: context.ok)
    monkeypatch.setattr(ros.rclpy, 'spin_once', spin)
    monkeypatch.setattr(ros.rclpy, 'shutdown', lambda: setattr(context, 'ok', False))
    result.rc = ros.main()
    result.elapsed = clock.now - 1.
    return result


def test_actual_main_services_all_latest_topics_before_GUI_graph_repeats(monkeypatch):
    result = run_busy_operator(monkeypatch)
    assert result.rc == 0
    assert result.armed, 'Single-callback/UI/graph loop starves fresh standby indefinitely'
    assert result.elapsed < 5.
    assert any(b-a > 1 for a,b in zip(result.spins_at_tick, result.spins_at_tick[1:]))


def test_callback_batching_does_not_bypass_a_missing_odom_proof(monkeypatch):
    result = run_busy_operator(monkeypatch, missing='odom')
    assert result.rc == 0 and not result.armed and result.blocked


def test_signal_interrupts_callback_batch_without_waiting_for_GUI_deadline(monkeypatch):
    result = run_busy_operator(monkeypatch, signal_at=1.12)
    assert result.rc == 0 and result.signal_stop and not result.armed
    assert result.elapsed < .2
