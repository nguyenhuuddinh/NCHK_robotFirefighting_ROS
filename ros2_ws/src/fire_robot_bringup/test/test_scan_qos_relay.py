import os
import select
import signal
import subprocess
import sys
import time

from fire_robot_bringup.scan_qos_relay import main

import pytest
import rclpy


def test_scan_qos_relay_subprocess_sigint_stress():
    """
    Stress test real SIGINT on scan_qos_relay process.

    Runs >= 20 iterations with 0 delay after readiness to catch races.
    Simulates ROS 2 signal handler shutting down the context.
    """
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    target_marker = "[ScanQosRelay] /scan_raw (Reliable) → /scan (Best Effort)"

    for iteration in range(100):
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
                        # Send SIGINT (same as Ctrl+C) immediately in same branch
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
        assert "Executor.__del__" not in output_buffer, (
            f"Iter {iteration} Executor.__del__ found in output:\n{output_buffer}"
        )


def test_scan_qos_relay_main_keyboard_interrupt(monkeypatch):
    """Test main() directly with KeyboardInterrupt when context is STILL valid."""
    import fire_robot_bringup.scan_qos_relay as sqr

    def mock_spin(node, executor=None):
        raise KeyboardInterrupt()

    monkeypatch.setattr(sqr.rclpy, 'spin', mock_spin)

    if rclpy.ok():
        rclpy.shutdown()

    try:
        main()
    except Exception as e:
        pytest.fail(f"main() raised an exception: {e}")

    assert not rclpy.ok(), "Context should be shutdown"
