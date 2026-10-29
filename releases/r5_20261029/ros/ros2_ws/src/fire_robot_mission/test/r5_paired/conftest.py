import sys
from pathlib import Path
import pytest
import rclpy
ROOT=Path(__file__).parents[5]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/fire_robot_mission'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/fire_robot_bringup'))
@pytest.fixture(autouse=True)
def no_runtime(monkeypatch):
    assert not rclpy.ok()
    monkeypatch.setattr(rclpy,'init',lambda *a,**k:pytest.fail('No real ROS init permitted'))
    yield
    assert not rclpy.ok()
