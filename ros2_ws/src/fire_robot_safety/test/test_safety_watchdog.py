import os
import select
import signal
import subprocess
import sys
import time

import pytest


def test_safety_watchdog_subprocess_sigint_stress():
    """
    Stress test real SIGINT on safety_watchdog process.

    Runs >= 100 iterations with 0 delay after readiness to catch races.
    """
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    env['ROS_LOG_DIR'] = '/tmp/ros2_test_safety_watchdog'
    env['ROS_DOMAIN_ID'] = '99'
    target_marker = "[SafetyGate] /cmd_vel_raw → /cmd_vel | timeout=1000ms, rate=10Hz"

    for iteration in range(100):
        proc = subprocess.Popen(
            [sys.executable, '-m', 'fire_robot_safety.safety_watchdog'],
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
                        f"OUTPUT:\n{output_buffer}"
                    )

                rlist, _, _ = select.select([proc.stdout], [], [], 0.1)
                if rlist:
                    line = proc.stdout.readline()
                    if not line:
                        break
                    output_buffer += line
                    if target_marker in line:
                        ready = True
                        proc.send_signal(signal.SIGINT)
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


def test_safety_watchdog_subprocess_sigterm():
    """Test SIGTERM handling."""
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    env['ROS_LOG_DIR'] = '/tmp/ros2_test_safety_watchdog_sigterm'
    env['ROS_DOMAIN_ID'] = '99'
    target_marker = "[SafetyGate] /cmd_vel_raw → /cmd_vel | timeout=1000ms, rate=10Hz"

    for iteration in range(100):
        proc = subprocess.Popen(
            [sys.executable, '-m', 'fire_robot_safety.safety_watchdog'],
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
                        f"OUTPUT:\n{output_buffer}"
                    )

                rlist, _, _ = select.select([proc.stdout], [], [], 0.1)
                if rlist:
                    line = proc.stdout.readline()
                    if not line:
                        break
                    output_buffer += line
                    if target_marker in line:
                        ready = True
                        proc.send_signal(signal.SIGTERM)
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


def test_safety_watchdog_main_behavioral_order(monkeypatch):
    """Test behavioral ordering/idempotence of timer -> executor -> node -> context."""
    import fire_robot_safety.safety_watchdog as sw

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
        call_order.append('rclpy.init')
        sw.rclpy.is_ok = True

    def mock_spin(node, executor=None):
        raise KeyboardInterrupt()

    def mock_shutdown():
        if not getattr(sw.rclpy, 'is_ok', False):
            raise RuntimeError("Double shutdown or early shutdown detected!")
        call_order.append('rclpy.shutdown')
        sw.rclpy.is_ok = False

    def mock_ok():
        return getattr(sw.rclpy, 'is_ok', False)

    monkeypatch.setattr(sw.rclpy, 'init', mock_init)
    monkeypatch.setattr(sw.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(sw.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(sw.rclpy, 'ok', mock_ok)
    monkeypatch.setattr(sw, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(sw, 'SafetyWatchdog', MockNode)

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    sw.main()

    # Verify order
    assert call_order == [
        'rclpy.init',
        'timer.cancel',
        'executor.shutdown',
        'node.destroy_node',
        'rclpy.shutdown'
    ], f"Invalid cleanup order: {call_order}"

    assert signal.getsignal(signal.SIGTERM) == original_sigterm, "SIGTERM handler not restored"
    assert signal.getsignal(signal.SIGINT) == original_sigint, "SIGINT handler not restored"


@pytest.mark.parametrize('failure_point', [
    'timer.cancel',
    'executor.shutdown',
    'node.destroy_node',
    'rclpy.shutdown'
])
def test_safety_watchdog_main_cleanup_failure_matrix(monkeypatch, failure_point):
    import fire_robot_safety.safety_watchdog as sw

    call_order = []

    class MockTimer:
        def __init__(self):
            self.canceled = False

        def is_canceled(self):
            return self.canceled

        def cancel(self):
            call_order.append('timer.cancel')
            assert getattr(sw.rclpy, 'is_ok', False), "Context must be valid"
            if failure_point == 'timer.cancel':
                raise RuntimeError("timer cancel failed")
            self.canceled = True

    class MockNode:
        def __init__(self):
            self._timer = MockTimer()

        def destroy_node(self):
            call_order.append('node.destroy_node')
            assert getattr(sw.rclpy, 'is_ok', False), "Context must be valid"
            if failure_point == 'node.destroy_node':
                raise RuntimeError("node destroy failed")

    class MockExecutor:
        def shutdown(self):
            call_order.append('executor.shutdown')
            assert getattr(sw.rclpy, 'is_ok', False), "Context must be valid"
            if failure_point == 'executor.shutdown':
                raise RuntimeError("executor shutdown failed")

    def mock_init(*args, **kwargs):
        call_order.append('rclpy.init')
        sw.rclpy.is_ok = True

    def mock_spin(node, executor=None):
        raise KeyboardInterrupt()

    shutdown_calls = [0]

    def mock_shutdown():
        shutdown_calls[0] += 1
        if not getattr(sw.rclpy, 'is_ok', False):
            raise RuntimeError("Double shutdown or early shutdown detected!")
        call_order.append('rclpy.shutdown')
        sw.rclpy.is_ok = False
        if failure_point == 'rclpy.shutdown':
            raise RuntimeError("rclpy shutdown failed")

    def mock_ok():
        return getattr(sw.rclpy, 'is_ok', False)

    monkeypatch.setattr(sw.rclpy, 'init', mock_init)
    monkeypatch.setattr(sw.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(sw.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(sw.rclpy, 'ok', mock_ok)
    monkeypatch.setattr(sw, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(sw, 'SafetyWatchdog', MockNode)

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    with pytest.raises(RuntimeError):
        sw.main()

    assert shutdown_calls[0] <= 1, f"Context shutdown called {shutdown_calls[0]} times"

    expected_calls = [
        'rclpy.init', 'timer.cancel', 'executor.shutdown', 'node.destroy_node', 'rclpy.shutdown'
    ]
    assert call_order == expected_calls, (
        f"Later cleanup should be attempted. Expected {expected_calls}, got {call_order}"
    )
    assert signal.getsignal(signal.SIGTERM) == original_sigterm, (
        f"SIGTERM handler not restored on failure at {failure_point}"
    )
    assert signal.getsignal(signal.SIGINT) == original_sigint, (
        f"SIGINT handler not restored on failure at {failure_point}"
    )


def test_safety_watchdog_main_exception_handler_restore(monkeypatch):
    """Ensure SIGTERM handler is restored even if an exception occurs during init."""
    import fire_robot_safety.safety_watchdog as sw

    def mock_init(*args, **kwargs):
        raise RuntimeError("Fake init failure")

    monkeypatch.setattr(sw.rclpy, 'init', mock_init)

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    with pytest.raises(RuntimeError):
        sw.main()

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
def test_safety_watchdog_main_keyboard_interrupt_during_cleanup(monkeypatch, failure_point):
    import fire_robot_safety.safety_watchdog as sw

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
        call_order.append('rclpy.init')
        sw.rclpy.is_ok = True

    def mock_spin(node, executor=None):
        os.kill(os.getpid(), signal.SIGINT)

    def mock_shutdown():
        call_order.append('rclpy.shutdown')
        if failure_point == 'rclpy.shutdown':
            os.kill(os.getpid(), signal.SIGINT)
        sw.rclpy.is_ok = False

    def mock_ok():
        return getattr(sw.rclpy, 'is_ok', False)

    monkeypatch.setattr(sw.rclpy, 'init', mock_init)
    monkeypatch.setattr(sw.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(sw.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(sw.rclpy, 'ok', mock_ok)
    monkeypatch.setattr(sw, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(sw, 'SafetyWatchdog', MockNode)

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    sw.main()

    expected_calls = [
        'rclpy.init', 'timer.cancel', 'executor.shutdown', 'node.destroy_node', 'rclpy.shutdown'
    ]
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


def test_safety_watchdog_main_exception_preservation(monkeypatch):
    import fire_robot_safety.safety_watchdog as sw

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
        call_order.append('rclpy.init')
        sw.rclpy.is_ok = True

    def mock_spin(node, executor=None):
        raise primary_error

    def mock_shutdown():
        call_order.append('rclpy.shutdown')
        sw.rclpy.is_ok = False

    def mock_ok():
        return getattr(sw.rclpy, 'is_ok', False)

    monkeypatch.setattr(sw.rclpy, 'init', mock_init)
    monkeypatch.setattr(sw.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(sw.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(sw.rclpy, 'ok', mock_ok)
    monkeypatch.setattr(sw, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(sw, 'SafetyWatchdog', MockNode)

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    with pytest.raises(ValueError) as excinfo:
        sw.main()

    assert excinfo.value is primary_error, "Exact primary exception must be preserved"

    expected_calls = [
        'rclpy.init', 'timer.cancel', 'executor.shutdown', 'node.destroy_node', 'rclpy.shutdown'
    ]
    assert call_order == expected_calls, "Multiple cleanup failures should still attempt all"
    assert signal.getsignal(signal.SIGTERM) == original_sigterm
    assert signal.getsignal(signal.SIGINT) == original_sigint


def test_safety_watchdog_main_cleanup_only_exception_identity(monkeypatch):
    import fire_robot_safety.safety_watchdog as sw

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
        sw.rclpy.is_ok = True

    def mock_spin(node, executor=None):
        raise KeyboardInterrupt()

    def mock_shutdown():
        sw.rclpy.is_ok = False
        raise RuntimeError("second cleanup failure")

    def mock_ok():
        return getattr(sw.rclpy, 'is_ok', False)

    monkeypatch.setattr(sw.rclpy, 'init', mock_init)
    monkeypatch.setattr(sw.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(sw.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(sw.rclpy, 'ok', mock_ok)
    monkeypatch.setattr(sw, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(sw, 'SafetyWatchdog', MockNode)

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    with pytest.raises(TypeError) as excinfo:
        sw.main()

    assert excinfo.value is cleanup_error, (
        "Exact first cleanup exception must be preserved when no primary exception"
    )
    assert signal.getsignal(signal.SIGTERM) == original_sigterm
    assert signal.getsignal(signal.SIGINT) == original_sigint


def test_safety_watchdog_subprocess_repeated_sigint_stress():
    """Stress test real repeated SIGINT on safety_watchdog process."""
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    env['ROS_LOG_DIR'] = '/tmp/ros2_test_safety_watchdog_repeated_sigint'
    env['ROS_DOMAIN_ID'] = '99'
    target_marker = "[SafetyGate] /cmd_vel_raw → /cmd_vel | timeout=1000ms, rate=10Hz"

    for iteration in range(50):
        proc = subprocess.Popen(
            [sys.executable, '-m', 'fire_robot_safety.safety_watchdog'],
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
                        f"OUTPUT:\n{output_buffer}"
                    )

                rlist, _, _ = select.select([proc.stdout], [], [], 0.1)
                if rlist:
                    line = proc.stdout.readline()
                    if not line:
                        break
                    output_buffer += line
                    if target_marker in line:
                        ready = True
                        proc.send_signal(signal.SIGINT)
                        proc.send_signal(signal.SIGINT)
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
        assert "Traceback" not in output_buffer
        assert "rcl_shutdown already called" not in output_buffer
        assert "KeyboardInterrupt" not in output_buffer
        assert "ExternalShutdownException" not in output_buffer
        assert "RCLError" not in output_buffer
        assert "publisher's context is invalid" not in output_buffer
        assert "Executor.__del__" not in output_buffer


def test_safety_watchdog_subprocess_repeated_sigterm_stress():
    """Test repeated SIGTERM handling."""
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    env['ROS_LOG_DIR'] = '/tmp/ros2_test_safety_watchdog_repeated_sigterm'
    env['ROS_DOMAIN_ID'] = '99'
    target_marker = "[SafetyGate] /cmd_vel_raw → /cmd_vel | timeout=1000ms, rate=10Hz"

    for iteration in range(50):
        proc = subprocess.Popen(
            [sys.executable, '-m', 'fire_robot_safety.safety_watchdog'],
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
                        f"OUTPUT:\n{output_buffer}"
                    )

                rlist, _, _ = select.select([proc.stdout], [], [], 0.1)
                if rlist:
                    line = proc.stdout.readline()
                    if not line:
                        break
                    output_buffer += line
                    if target_marker in line:
                        ready = True
                        proc.send_signal(signal.SIGTERM)
                        proc.send_signal(signal.SIGTERM)
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
        assert "Traceback" not in output_buffer
        assert "rcl_shutdown already called" not in output_buffer
        assert "KeyboardInterrupt" not in output_buffer
        assert "ExternalShutdownException" not in output_buffer
        assert "RCLError" not in output_buffer
        assert "publisher's context is invalid" not in output_buffer
        assert "Executor.__del__" not in output_buffer


def test_safety_watchdog_main_deterministic_handler_behavior(monkeypatch):
    """Test deterministic first/second handler invocation without subprocess races."""
    import fire_robot_safety.safety_watchdog as sw

    original_sigterm = signal.getsignal(signal.SIGTERM)
    original_sigint = signal.getsignal(signal.SIGINT)

    registered_handlers = {}
    restored_handlers = {}

    orig_signal = signal.signal

    def mock_signal(sig, handler):
        if handler in (original_sigterm, original_sigint):
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
        call_order.append('rclpy.init')
        sw.rclpy.is_ok = True

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

        sw.rclpy.is_ok = False

    def mock_ok():
        return getattr(sw.rclpy, 'is_ok', False)

    monkeypatch.setattr(sw.rclpy, 'init', mock_init)
    monkeypatch.setattr(sw.rclpy, 'spin', mock_spin)
    monkeypatch.setattr(sw.rclpy, 'shutdown', mock_shutdown)
    monkeypatch.setattr(sw.rclpy, 'ok', mock_ok)
    monkeypatch.setattr(sw, 'SingleThreadedExecutor', MockExecutor)
    monkeypatch.setattr(sw, 'SafetyWatchdog', MockNode)

    sw.main()

    assert call_order == [
        'rclpy.init',
        'timer.cancel',
        'executor.shutdown',
        'node.destroy_node',
        'rclpy.shutdown'
    ]

    assert restored_handlers.get(signal.SIGINT) == original_sigint, (
        "Exact original SIGINT not restored"
    )
    assert restored_handlers.get(signal.SIGTERM) == original_sigterm, (
        "Exact original SIGTERM not restored"
    )
