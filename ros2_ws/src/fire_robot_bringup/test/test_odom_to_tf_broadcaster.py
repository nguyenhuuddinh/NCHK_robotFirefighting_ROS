import os
import select
import signal
import subprocess
import sys
import time

import pytest
import rclpy


def test_odom_to_tf_broadcaster_subprocess_sigint_stress():
    """
    Stress test real SIGINT on odom_to_tf_broadcaster process.

    Runs >= 100 iterations with 0 delay after readiness to catch races.
    """
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    env['ROS_LOG_DIR'] = '/tmp/ros2_test_odom_to_tf'
    env['ROS_DOMAIN_ID'] = '98'
    target_marker = (
        '[OdomToTF] Timer 20Hz + stale guard 300ms — '
        'stamp_source="pi_receive_time", offset=0ms'
    )

    for iteration in range(100):
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


def test_odom_to_tf_broadcaster_main_keyboard_interrupt(monkeypatch):
    """Test main() directly with KeyboardInterrupt when context is STILL valid."""
    import fire_robot_bringup.odom_to_tf_broadcaster as otf

    def mock_spin(node, executor=None):
        raise KeyboardInterrupt()

    monkeypatch.setattr(otf.rclpy, 'spin', mock_spin)

    if rclpy.ok():
        rclpy.shutdown()

    try:
        otf.main()
    except Exception as e:
        pytest.fail(f"main() raised an exception: {e}")

    assert not rclpy.ok(), "Context should be shutdown"
