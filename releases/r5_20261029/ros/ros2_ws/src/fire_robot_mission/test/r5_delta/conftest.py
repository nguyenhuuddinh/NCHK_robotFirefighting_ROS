"""All ROS/device execution is forbidden in these integration tests."""
import pytest
import rclpy


@pytest.fixture(autouse=True)
def no_ros_context(monkeypatch):
    assert not rclpy.ok()
    monkeypatch.setattr(rclpy, 'init', lambda *a, **k: pytest.fail('real ROS init forbidden'))
    yield
    assert not rclpy.ok()
