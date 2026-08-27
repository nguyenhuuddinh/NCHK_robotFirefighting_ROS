import os
import signal
import subprocess
import sys
import time

from fire_robot_bringup.scan_qos_relay import main

import pytest
import rclpy


def test_scan_qos_relay_subprocess_sigint():
    """
    Test real SIGINT on scan_qos_relay process.

    It must exit 0 and have no traceback.
    This simulates ROS 2 signal handler shutting down the context.
    """
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'

    proc = subprocess.Popen(
        [sys.executable, '-m', 'fire_robot_bringup.scan_qos_relay'],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    try:
        # Bounded readiness poll
        ready = False
        start_time = time.time()
        output_buffer = ""

        import fcntl
        flags = fcntl.fcntl(proc.stdout.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(proc.stdout.fileno(), fcntl.F_SETFL, flags | os.O_NONBLOCK)

        while time.time() - start_time < 5.0:
            if proc.poll() is not None:
                stdout, _ = proc.communicate()
                output_buffer += stdout or ""
                pytest.fail(
                    f"Child exited early with {proc.returncode}.\n"
                    f"OUTPUT:\n{output_buffer}"
                )

            try:
                chunk = proc.stdout.read()
                if chunk:
                    output_buffer += chunk
                    if "Reliable" in output_buffer or "Best Effort" in output_buffer:
                        ready = True
                        break
            except (IOError, TypeError):
                pass

            time.sleep(0.1)

        if not ready:
            proc.terminate()
            stdout, _ = proc.communicate(timeout=2.0)
            output_buffer += stdout or ""
            pytest.fail(f"Timeout waiting for readiness.\nOUTPUT:\n{output_buffer}")

        # Send SIGINT (same as Ctrl+C)
        proc.send_signal(signal.SIGINT)

        # Wait for it to exit
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
        f"Process exited with {proc.returncode}\n"
        f"OUTPUT:\n{output_buffer}"
    )
    assert "Traceback" not in output_buffer, (
        f"Traceback found in output:\n{output_buffer}"
    )
    assert "rcl_shutdown already called" not in output_buffer, (
        f"Invalid shutdown found in output:\n{output_buffer}"
    )


def test_scan_qos_relay_main_keyboard_interrupt(monkeypatch):
    """Test main() directly with KeyboardInterrupt when context is STILL valid."""
    import fire_robot_bringup.scan_qos_relay as sqr

    def mock_spin(node):
        raise KeyboardInterrupt()

    monkeypatch.setattr(sqr.rclpy, 'spin', mock_spin)

    if rclpy.ok():
        rclpy.shutdown()

    try:
        main()
    except Exception as e:
        pytest.fail(f"main() raised an exception: {e}")

    assert not rclpy.ok(), "Context should be shutdown"
