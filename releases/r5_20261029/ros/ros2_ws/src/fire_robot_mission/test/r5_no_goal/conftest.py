import sys
from pathlib import Path
import pytest
import rclpy
sys.path.insert(0,str(Path(__file__).parents[1]/'r5_delta'))

@pytest.fixture(autouse=True)
def no_devices_or_DDS(monkeypatch):
    assert not rclpy.ok()
    monkeypatch.setattr(rclpy,'init',lambda *a,**k:pytest.fail('ROS runtime forbidden'))
    yield
    assert not rclpy.ok()
