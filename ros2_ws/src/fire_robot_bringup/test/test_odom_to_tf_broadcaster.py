import os
import select
import signal
import subprocess
import sys
import time
from rclpy.signals import SignalHandlerOptions

import pytest


def _run_subprocess_test(signal_to_send, iterations, multiple_signals=False):
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    env['ROS_LOG_DIR'] = f'/tmp/ros2_test_odom_to_tf_broadcaster_{signal_to_send}'
    env['ROS_DOMAIN_ID'] = '98'
    target_marker = (
        '[OdomToTF] Timer 20Hz + stale guard 300ms — '
        'stamp_source="pi_receive_time", offset=0ms'
    )

    for iteration in range(iterations):
        proc = subprocess.Popen(
            [sys.executable, '-m', 'fire_robot_bringup.odom_to_tf_broadcaster'],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )

        try:
            ready = False
            start_time = time.monotonic()
            output_buffer = ""

            while time.monotonic() - start_time < 5.0:
                if proc.poll() is not None:
                    stdout, _ = proc.communicate()
                    output_buffer += stdout or ""
                    pytest.fail(
                        f"Iter {iteration} Child exited early ({proc.returncode}).\n"
                        f"OUTPUT:\n{output_buffer}")

                rlist, _, _ = select.select([proc.stdout], [], [], 0.1)
                if rlist:
                    line = proc.stdout.readline()
                    if not line:
                        break
                    output_buffer += line
                    if target_marker in line:
                        ready = True
                        proc.send_signal(signal_to_send)
                        if multiple_signals:
                            proc.send_signal(signal_to_send)
                            proc.send_signal(signal_to_send)
                        break

            if not ready:
                proc.terminate()
                stdout, _ = proc.communicate(timeout=2.0)
                output_buffer += stdout or ""
                pytest.fail(
                    f"Iter {iteration} Timeout waiting for readiness.\n"
                    f"OUTPUT:\n{output_buffer}"
                )

            try:
                stdout, _ = proc.communicate(timeout=5.0)
                output_buffer += stdout or ""
            except subprocess.TimeoutExpired:
                pass

        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    stdout, _ = proc.communicate(timeout=2.0)
                    output_buffer += stdout or ""
                except subprocess.TimeoutExpired:
                    proc.kill()
                    stdout, _ = proc.communicate()
                    output_buffer += stdout or ""

        assert proc.returncode == 0, (
            f"Iter {iteration} Process exited with {proc.returncode}\n"
            f"OUTPUT:\n{output_buffer}"
        )
        assert "Traceback" not in output_buffer, (
            f"Iter {iteration} Traceback found in output:\n{output_buffer}"
        )
        assert "rcl_shutdown already called" not in output_buffer, (
            f"Iter {iteration} Invalid shutdown found in output:\n{output_buffer}"
        )
        assert "KeyboardInterrupt" not in output_buffer, (
            f"Iter {iteration} KeyboardInterrupt leaked in output:\n{output_buffer}"
        )
        assert "ExternalShutdownException" not in output_buffer, (
            f"Iter {iteration} ExternalShutdownException found in output:\n{output_buffer}"
        )
        assert "RCLError" not in output_buffer, (
            f"Iter {iteration} RCLError found in output:\n{output_buffer}"
        )
        assert "publisher's context is invalid" not in output_buffer, (
            f"Iter {iteration} context is invalid found in output:\n{output_buffer}"
        )
        assert "Executor.__del__" not in output_buffer, (
            f"Iter {iteration} Executor.__del__ found in output:\n{output_buffer}"
        )


def test_odom_to_tf_broadcaster_subprocess_sigint_stress():
    """Stress test real SIGINT on odom_to_tf_broadcaster process."""
    _run_subprocess_test(signal.SIGINT, 100)


def test_odom_to_tf_broadcaster_subprocess_sigterm():
    """Test SIGTERM handling."""
    _run_subprocess_test(signal.SIGTERM, 100)


def test_odom_to_tf_broadcaster_main_behavioral_order(monkeypatch):
    """Test behavioral ordering/idempotence of timer -> executor -> node -> context."""
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    call_order = []

    class MockTimer:
        def __init__(self):
            self.canceled = False

        def is_canceled(self):
            return self.canceled

        def cancel(self):
            self.canceled = True
            call_order.append('timer.cancel')

    class MockNode:
        def __init__(self):
            self._timer = MockTimer()

        def destroy_node(self):
            call_order.append('node.destroy_node')

    class MockExecutor:
        def shutdown(self):
            call_order.append('executor.shutdown')

    def mock_init(*args, **kwargs):
        assert kwargs.get('signal_handler_options') == SignalHandlerOptions.NO
        call_order.append('rclpy.init')
        otf.rclpy.is_ok = True

    def mock_spin(node, executor=None):
        raise KeyboardInterrupt()

    def mock_shutdown():
        if not getattr(otf.rclpy, 'is_ok', False):
            raise RuntimeError("Double shutdown or early shutdown detected!")
        call_order.append('rclpy.shutdown')
        otf.rclpy.is_ok = False

    def mock_ok():
        return getattr(otf.rclpy, 'is_ok', False)

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)
    monkeypatch.setattr(otf.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(otf.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(otf.rclpy, 'ok', mock_ok)
    monkeypatch.setattr(otf, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(otf, 'OdomToTfBroadcaster', MockNode)

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    otf.main()

    # Verify order
    assert call_order == [
        'rclpy.init',
        'timer.cancel',
        'executor.shutdown',
        'node.destroy_node',
        'rclpy.shutdown'
    ], f"Invalid cleanup order: {call_order}"

    assert signal.getsignal(
        signal.SIGTERM) == original_sigterm, "SIGTERM handler not restored"
    assert signal.getsignal(
        signal.SIGINT) == original_sigint, "SIGINT handler not restored"


@pytest.mark.parametrize('failure_point', [
    'timer.cancel',
    'executor.shutdown',
    'node.destroy_node',
    'rclpy.shutdown'
])
def test_odom_to_tf_broadcaster_main_cleanup_failure_matrix(
        monkeypatch, failure_point):
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    call_order = []

    class MockTimer:
        def __init__(self):
            self.canceled = False

        def is_canceled(self):
            return self.canceled

        def cancel(self):
            call_order.append('timer.cancel')
            assert getattr(otf.rclpy, 'is_ok', False), "Context must be valid"
            if failure_point == 'timer.cancel':
                raise RuntimeError("timer cancel failed")
            self.canceled = True

    class MockNode:
        def __init__(self):
            self._timer = MockTimer()

        def destroy_node(self):
            call_order.append('node.destroy_node')
            assert getattr(otf.rclpy, 'is_ok', False), "Context must be valid"
            if failure_point == 'node.destroy_node':
                raise RuntimeError("node destroy failed")

    class MockExecutor:
        def shutdown(self):
            call_order.append('executor.shutdown')
            assert getattr(otf.rclpy, 'is_ok', False), "Context must be valid"
            if failure_point == 'executor.shutdown':
                raise RuntimeError("executor shutdown failed")

    def mock_init(*args, **kwargs):
        assert kwargs.get('signal_handler_options') == SignalHandlerOptions.NO
        call_order.append('rclpy.init')
        otf.rclpy.is_ok = True

    def mock_spin(node, executor=None):
        raise KeyboardInterrupt()

    shutdown_calls = [0]

    def mock_shutdown():
        shutdown_calls[0] += 1
        if not getattr(otf.rclpy, 'is_ok', False):
            raise RuntimeError("Double shutdown or early shutdown detected!")
        call_order.append('rclpy.shutdown')
        otf.rclpy.is_ok = False
        if failure_point == 'rclpy.shutdown':
            raise RuntimeError("rclpy shutdown failed")

    def mock_ok():
        return getattr(otf.rclpy, 'is_ok', False)

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)
    monkeypatch.setattr(otf.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(otf.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(otf.rclpy, 'ok', mock_ok)
    monkeypatch.setattr(otf, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(otf, 'OdomToTfBroadcaster', MockNode)

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    with pytest.raises(RuntimeError):
        otf.main()

    assert shutdown_calls[0] <= 1, f"Context shutdown called {shutdown_calls[0]} times"

    expected_calls = [
        'rclpy.init',
        'timer.cancel',
        'executor.shutdown',
        'node.destroy_node',
        'rclpy.shutdown']
    assert call_order == expected_calls, (
        f"Later cleanup should be attempted. Expected {expected_calls}, got {call_order}"
    )
    assert signal.getsignal(signal.SIGTERM) == original_sigterm, (
        f"SIGTERM handler not restored on failure at {failure_point}"
    )
    assert signal.getsignal(signal.SIGINT) == original_sigint, (
        f"SIGINT handler not restored on failure at {failure_point}"
    )


def test_odom_to_tf_broadcaster_main_exception_handler_restore(monkeypatch):
    """Ensure SIGTERM handler is restored even if an exception occurs during init."""
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    def mock_init(*args, **kwargs):
        raise RuntimeError("Fake init failure")

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    with pytest.raises(RuntimeError):
        otf.main()

    assert signal.getsignal(signal.SIGTERM) == original_sigterm, (
        "SIGTERM handler not restored on exception"
    )
    assert signal.getsignal(signal.SIGINT) == original_sigint, (
        "SIGINT handler not restored on exception"
    )


@pytest.mark.parametrize('failure_point', [
    'timer.cancel',
    'executor.shutdown',
    'node.destroy_node',
    'rclpy.shutdown'
])
def test_odom_to_tf_broadcaster_main_keyboard_interrupt_during_cleanup(
        monkeypatch, failure_point):
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    call_order = []

    class MockTimer:
        def __init__(self):
            self.canceled = False

        def is_canceled(self):
            return self.canceled

        def cancel(self):
            call_order.append('timer.cancel')
            if failure_point == 'timer.cancel':
                os.kill(os.getpid(), signal.SIGINT)
            self.canceled = True

    class MockNode:
        def __init__(self):
            self._timer = MockTimer()

        def destroy_node(self):
            call_order.append('node.destroy_node')
            if failure_point == 'node.destroy_node':
                os.kill(os.getpid(), signal.SIGINT)

    class MockExecutor:
        def shutdown(self):
            call_order.append('executor.shutdown')
            if failure_point == 'executor.shutdown':
                os.kill(os.getpid(), signal.SIGINT)

    def mock_init(*args, **kwargs):
        assert kwargs.get('signal_handler_options') == SignalHandlerOptions.NO
        call_order.append('rclpy.init')
        otf.rclpy.is_ok = True

    def mock_spin(node, executor=None):
        os.kill(os.getpid(), signal.SIGINT)

    def mock_shutdown():
        call_order.append('rclpy.shutdown')
        if failure_point == 'rclpy.shutdown':
            os.kill(os.getpid(), signal.SIGINT)
        otf.rclpy.is_ok = False

    def mock_ok():
        return getattr(otf.rclpy, 'is_ok', False)

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)
    monkeypatch.setattr(otf.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(otf.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(otf.rclpy, 'ok', mock_ok)
    monkeypatch.setattr(otf, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(otf, 'OdomToTfBroadcaster', MockNode)

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    otf.main()

    expected_calls = [
        'rclpy.init',
        'timer.cancel',
        'executor.shutdown',
        'node.destroy_node',
        'rclpy.shutdown']
    assert call_order == expected_calls, (
        f"Repeated KeyboardInterrupt should not skip later cleanup. "
        f"Expected {expected_calls}, got {call_order}"
    )
    assert signal.getsignal(signal.SIGTERM) == original_sigterm, (
        "SIGTERM handler not restored after repeated KeyboardInterrupt"
    )
    assert signal.getsignal(signal.SIGINT) == original_sigint, (
        "SIGINT handler not restored after repeated KeyboardInterrupt"
    )


def test_odom_to_tf_broadcaster_main_exception_preservation(monkeypatch):
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    call_order = []
    primary_error = ValueError("primary spin failure")
    cleanup_error1 = RuntimeError("first cleanup failure")
    cleanup_error2 = RuntimeError("second cleanup failure")

    class MockTimer:
        def __init__(self):
            self.canceled = False

        def is_canceled(self):
            return self.canceled

        def cancel(self):
            call_order.append('timer.cancel')
            raise cleanup_error1

    class MockNode:
        def __init__(self):
            self._timer = MockTimer()

        def destroy_node(self):
            call_order.append('node.destroy_node')

    class MockExecutor:
        def shutdown(self):
            call_order.append('executor.shutdown')
            raise cleanup_error2

    def mock_init(*args, **kwargs):
        assert kwargs.get('signal_handler_options') == SignalHandlerOptions.NO
        call_order.append('rclpy.init')
        otf.rclpy.is_ok = True

    def mock_spin(node, executor=None):
        raise primary_error

    def mock_shutdown():
        call_order.append('rclpy.shutdown')
        otf.rclpy.is_ok = False

    def mock_ok():
        return getattr(otf.rclpy, 'is_ok', False)

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)
    monkeypatch.setattr(otf.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(otf.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(otf.rclpy, 'ok', mock_ok)
    monkeypatch.setattr(otf, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(otf, 'OdomToTfBroadcaster', MockNode)

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    with pytest.raises(ValueError) as excinfo:
        otf.main()

    assert excinfo.value is primary_error, "Exact primary exception must be preserved"

    expected_calls = [
        'rclpy.init',
        'timer.cancel',
        'executor.shutdown',
        'node.destroy_node',
        'rclpy.shutdown']
    assert call_order == expected_calls, "Multiple cleanup failures should still attempt all"
    assert signal.getsignal(signal.SIGTERM) == original_sigterm
    assert signal.getsignal(signal.SIGINT) == original_sigint


def test_odom_to_tf_broadcaster_main_cleanup_only_exception_identity(
        monkeypatch):
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    cleanup_error = TypeError("first cleanup failure only")

    class MockTimer:
        def __init__(self):
            self.canceled = False

        def is_canceled(self):
            return self.canceled

        def cancel(self):
            self.canceled = True

    class MockNode:
        def __init__(self):
            self._timer = MockTimer()

        def destroy_node(self):
            pass

    class MockExecutor:
        def shutdown(self):
            raise cleanup_error

    def mock_init(*args, **kwargs):
        otf.rclpy.is_ok = True

    def mock_spin(node, executor=None):
        raise KeyboardInterrupt()

    def mock_shutdown():
        otf.rclpy.is_ok = False
        raise RuntimeError("second cleanup failure")

    def mock_ok():
        return getattr(otf.rclpy, 'is_ok', False)

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)
    monkeypatch.setattr(otf.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(otf.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(otf.rclpy, 'ok', mock_ok)
    monkeypatch.setattr(otf, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(otf, 'OdomToTfBroadcaster', MockNode)

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    with pytest.raises(TypeError) as excinfo:
        otf.main()

    assert excinfo.value is cleanup_error, (
        "Exact first cleanup exception must be preserved when no primary exception"
    )
    assert signal.getsignal(signal.SIGTERM) == original_sigterm
    assert signal.getsignal(signal.SIGINT) == original_sigint


def test_odom_to_tf_broadcaster_subprocess_repeated_sigint_stress():
    """Test multiple SIGINTs during cleanup."""
    _run_subprocess_test(signal.SIGINT, 50, multiple_signals=True)


def test_odom_to_tf_broadcaster_subprocess_repeated_sigterm_stress():
    """Test multiple SIGTERMs during cleanup."""
    _run_subprocess_test(signal.SIGTERM, 50, multiple_signals=True)


def test_odom_to_tf_broadcaster_main_deterministic_handler_behavior(
        monkeypatch):
    """Test deterministic first/second handler invocation without subprocess races."""
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    registered_handlers = {}
    restored_handlers = {}

    orig_signal = signal.signal

    def mock_signal(sig, handler):
        if handler in (original_sigterm, original_sigint):
            call_order.append(f'signal.restore.{sig.name}')
            restored_handlers[sig] = handler
        else:
            registered_handlers[sig] = handler
        return orig_signal(sig, handler)

    monkeypatch.setattr(signal, 'signal', mock_signal)

    call_order = []

    class MockTimer:
        def is_canceled(self):
            return False

        def cancel(self):
            call_order.append('timer.cancel')

    class MockNode:
        def __init__(self):
            self._timer = MockTimer()

        def destroy_node(self):
            call_order.append('node.destroy_node')

    class MockExecutor:
        def shutdown(self):
            call_order.append('executor.shutdown')

    def mock_init(*args, **kwargs):
        assert kwargs.get('signal_handler_options') == SignalHandlerOptions.NO
        call_order.append('rclpy.init')
        otf.rclpy.is_ok = True

    def mock_spin(node, executor=None):
        assert signal.SIGINT in registered_handlers, (
            "SIGINT handler not registered"
        )
        assert signal.SIGTERM in registered_handlers, (
            "SIGTERM handler not registered"
        )

        handler_int = registered_handlers[signal.SIGINT]
        handler_term = registered_handlers[signal.SIGTERM]
        assert handler_int is handler_term, (
            "SIGINT and SIGTERM should use the same shared handler"
        )

        # First invocation should raise KeyboardInterrupt
        with pytest.raises(KeyboardInterrupt):
            handler_int(signal.SIGINT, None)

        raise KeyboardInterrupt()

    def mock_shutdown():
        call_order.append('rclpy.shutdown')

        # Second invocation should return, not raise
        handler_int = registered_handlers[signal.SIGINT]
        handler_int(signal.SIGINT, None)

        otf.rclpy.is_ok = False

    def mock_ok():
        return getattr(otf.rclpy, 'is_ok', False)

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)
    monkeypatch.setattr(otf.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(otf.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(otf.rclpy, 'ok', mock_ok)
    monkeypatch.setattr(otf, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(otf, 'OdomToTfBroadcaster', MockNode)

    otf.main()

    assert call_order == [
        'rclpy.init',
        'timer.cancel',
        'executor.shutdown',
        'node.destroy_node',
        'rclpy.shutdown',
        'signal.restore.SIGTERM',
        'signal.restore.SIGINT'
    ]

    assert restored_handlers.get(signal.SIGINT) == original_sigint, (
        "Exact original SIGINT not restored"
    )
    assert restored_handlers.get(signal.SIGTERM) == original_sigterm, (
        "Exact original SIGTERM not restored"
    )


def test_odom_to_tf_broadcaster_main_executor_init_failure(monkeypatch):
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    call_order = []
    restored_handlers = {}
    registered_handlers = {}

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)
    orig_signal = signal.signal

    def mock_signal(sig, handler):
        if handler in (original_sigterm, original_sigint):
            call_order.append(f'signal.restore.{sig.name}')
            restored_handlers[sig] = handler
        else:
            registered_handlers[sig] = handler
        return orig_signal(sig, handler)

    monkeypatch.setattr(signal, 'signal', mock_signal)

    def mock_init(*args, **kwargs):
        assert kwargs.get('signal_handler_options') == SignalHandlerOptions.NO
        call_order.append('rclpy.init')
        otf.rclpy.is_ok = True

    def mock_shutdown():
        call_order.append('rclpy.shutdown')
        otf.rclpy.is_ok = False

    def mock_ok():
        return getattr(otf.rclpy, 'is_ok', False)

    monkeypatch.setattr(otf.rclpy, 'ok', mock_ok)

    class FailingExecutor:
        def __init__(self):
            raise RuntimeError("Executor failed")

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)
    monkeypatch.setattr(otf.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(otf, 'SingleThreadedExecutor', FailingExecutor)

    with pytest.raises(RuntimeError, match="Executor failed"):
        otf.main()

    assert call_order == [
        'rclpy.init',
        'rclpy.shutdown',
        'signal.restore.SIGTERM',
        'signal.restore.SIGINT'
    ]


def test_odom_to_tf_broadcaster_main_node_init_failure(monkeypatch):
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    call_order = []
    restored_handlers = {}
    registered_handlers = {}

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)
    orig_signal = signal.signal

    def mock_signal(sig, handler):
        if handler in (original_sigterm, original_sigint):
            call_order.append(f'signal.restore.{sig.name}')
            restored_handlers[sig] = handler
        else:
            registered_handlers[sig] = handler
        return orig_signal(sig, handler)

    monkeypatch.setattr(signal, 'signal', mock_signal)

    def mock_init(*args, **kwargs):
        assert kwargs.get('signal_handler_options') == SignalHandlerOptions.NO
        call_order.append('rclpy.init')
        otf.rclpy.is_ok = True

    def mock_shutdown():
        call_order.append('rclpy.shutdown')
        otf.rclpy.is_ok = False

    def mock_ok():
        return getattr(otf.rclpy, 'is_ok', False)

    monkeypatch.setattr(otf.rclpy, 'ok', mock_ok)

    class MockExecutor:
        def shutdown(self):
            call_order.append('executor.shutdown')

    class FailingNode:
        def __init__(self):
            raise RuntimeError("Node failed")

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)
    monkeypatch.setattr(otf.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(otf, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(otf, 'OdomToTfBroadcaster', FailingNode)

    with pytest.raises(RuntimeError, match="Node failed"):
        otf.main()

    assert call_order == [
        'rclpy.init',
        'executor.shutdown',
        'rclpy.shutdown',
        'signal.restore.SIGTERM',
        'signal.restore.SIGINT'
    ]


def test_odom_to_tf_broadcaster_main_registration_failure(monkeypatch):
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    orig_signal = signal.signal
    call_order = []

    def mock_signal(sig, handler):
        if handler in (original_sigterm, original_sigint):
            call_order.append(f'signal.restore.{sig.name}')
        elif sig == signal.SIGINT:
            raise RuntimeError("SIGINT registration failed")
        else:
            call_order.append(f'signal.register.{sig.name}')
        return orig_signal(sig, handler)

    monkeypatch.setattr(signal, 'signal', mock_signal)

    with pytest.raises(RuntimeError, match="SIGINT registration failed"):
        otf.main()

    assert call_order == [
        'signal.register.SIGTERM',
        'signal.restore.SIGTERM',
        'signal.restore.SIGINT'
    ]


def test_odom_to_tf_broadcaster_main_restore_failure(monkeypatch):
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    orig_signal = signal.signal
    call_order = []

    def mock_signal(sig, handler):
        if handler in (original_sigterm, original_sigint):
            call_order.append(f'signal.restore.{sig.name}')
            if sig == signal.SIGTERM:
                raise RuntimeError("SIGTERM restore failed")
        return orig_signal(sig, handler)

    monkeypatch.setattr(signal, 'signal', mock_signal)

    def mock_init(*args, **kwargs):
        pass

    def mock_spin(node, executor=None):
        raise KeyboardInterrupt()

    def mock_ok():
        return False

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)
    monkeypatch.setattr(otf.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(otf.rclpy, 'ok', mock_ok)

    class MockExecutor:
        def shutdown(self):
            pass
    monkeypatch.setattr(otf, 'SingleThreadedExecutor', MockExecutor)

    class MockTimer:
        def is_canceled(self):
            return False

        def cancel(self):
            pass

    class MockNode:
        def __init__(self):
            self._timer = MockTimer()

        def destroy_node(self):
            pass
    monkeypatch.setattr(otf, 'OdomToTfBroadcaster', MockNode)

    with pytest.raises(RuntimeError, match="SIGTERM restore failed"):
        otf.main()

    assert call_order == [
        'signal.restore.SIGTERM',
        'signal.restore.SIGINT'
    ]


def test_odom_to_tf_broadcaster_main_primary_identity_not_masked(monkeypatch):
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    orig_signal = signal.signal

    def mock_signal(sig, handler):
        if handler in (original_sigterm, original_sigint):
            if sig == signal.SIGTERM:
                raise RuntimeError("SIGTERM restore failed")
        return orig_signal(sig, handler)

    monkeypatch.setattr(signal, 'signal', mock_signal)

    primary_error = ValueError("primary exception")

    def mock_init(*args, **kwargs):
        pass

    def mock_spin(node, executor=None):
        raise primary_error

    def mock_ok():
        return False

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)
    monkeypatch.setattr(otf.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(otf.rclpy, 'ok', mock_ok)

    class MockExecutor:
        def shutdown(self):
            pass
    monkeypatch.setattr(otf, 'SingleThreadedExecutor', MockExecutor)

    class MockTimer:
        def is_canceled(self):
            return False

        def cancel(self):
            pass

    class MockNode:
        def __init__(self):
            self._timer = MockTimer()

        def destroy_node(self):
            pass
    monkeypatch.setattr(otf, 'OdomToTfBroadcaster', MockNode)

    with pytest.raises(ValueError) as excinfo:
        otf.main()

    assert excinfo.value is primary_error


def test_odom_to_tf_broadcaster_main_cleanup_only_restore_error_identity(
        monkeypatch):
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    orig_signal = signal.signal

    first_error = RuntimeError("First restore error")

    def mock_signal(sig, handler):
        if handler in (original_sigterm, original_sigint):
            if sig == signal.SIGTERM:
                raise first_error
            if sig == signal.SIGINT:
                raise RuntimeError("Second restore error")
        return orig_signal(sig, handler)

    monkeypatch.setattr(signal, 'signal', mock_signal)

    def mock_init(*args, **kwargs):
        pass

    def mock_spin(node, executor=None):
        raise KeyboardInterrupt()

    def mock_ok():
        return False

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)
    monkeypatch.setattr(otf.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(otf.rclpy, 'ok', mock_ok)

    class MockExecutor:
        def shutdown(self):
            pass
    monkeypatch.setattr(otf, 'SingleThreadedExecutor', MockExecutor)

    class MockTimer:
        def is_canceled(self):
            return False

        def cancel(self):
            pass

    class MockNode:
        def __init__(self):
            self._timer = MockTimer()

        def destroy_node(self):
            pass
    monkeypatch.setattr(otf, 'OdomToTfBroadcaster', MockNode)

    with pytest.raises(RuntimeError) as excinfo:
        otf.main()

    assert excinfo.value is first_error


def test_odom_to_tf_broadcaster_main_signal_during_registration(monkeypatch):
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    orig_signal = signal.signal
    call_order = []

    init_called = [False]

    def mock_signal(sig, handler):
        if handler in (original_sigterm, original_sigint):
            call_order.append(f'signal.restore.{sig.name}')
            return orig_signal(sig, handler)
        else:
            call_order.append(f'signal.register.{sig.name}')
            # Inject KeyboardInterrupt right during registration window
            if sig == signal.SIGTERM:
                # Capture the handler to invoke it!
                handler(sig.value, None)
            return orig_signal(sig, handler)

    monkeypatch.setattr(signal, 'signal', mock_signal)

    def mock_init(*args, **kwargs):
        init_called[0] = True

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)

    # Clean return, no KeyboardInterrupt leaked
    otf.main()

    assert not init_called[0]
    assert call_order == [
        'signal.register.SIGTERM',
        'signal.restore.SIGTERM',
        'signal.restore.SIGINT'
    ]


def test_odom_to_tf_broadcaster_main_signal_during_registration_with_restore_error(monkeypatch):
    import fire_robot_bringup.odom_to_tf_broadcaster as otf
    import pytest

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    orig_signal = signal.signal
    call_order = []

    init_called = [False]
    first_error = RuntimeError("Restore failed during outer KeyboardInterrupt handling")

    def mock_signal(sig, handler):
        if handler in (original_sigterm, original_sigint):
            call_order.append(f'signal.restore.{sig.name}')
            if sig == signal.SIGTERM:
                raise first_error
            return orig_signal(sig, handler)
        else:
            call_order.append(f'signal.register.{sig.name}')
            if sig == signal.SIGTERM:
                handler(sig.value, None)
            return orig_signal(sig, handler)

    monkeypatch.setattr(signal, 'signal', mock_signal)

    def mock_init(*args, **kwargs):
        init_called[0] = True

    monkeypatch.setattr(otf.rclpy, 'init', mock_init)

    with pytest.raises(RuntimeError) as excinfo:
        otf.main()

    assert excinfo.value is first_error
    assert not init_called[0]
    assert call_order == [
        'signal.register.SIGTERM',
        'signal.restore.SIGTERM',
        'signal.restore.SIGINT'
    ]
