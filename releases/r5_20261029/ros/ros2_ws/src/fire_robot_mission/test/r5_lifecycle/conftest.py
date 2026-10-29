import sys
from pathlib import Path
import pytest
import rclpy
sys.path.insert(0,str(Path(__file__).parents[1]/'r5_full_adapter'))

@pytest.fixture(autouse=True)
def forbid_ROS_devices(monkeypatch):
    assert not rclpy.ok()


@pytest.fixture(autouse=True)
def synthetic_decodable_camera_for_new_lifecycle_chains(monkeypatch):
    from harness import Robot,BE
    from sensor_msgs.msg import CompressedImage
    import cv2
    import numpy as np
    original_init=Robot.__init__;original_inputs=Robot.inputs
    good,data=cv2.imencode('.jpg',np.zeros((4,4,3),dtype=np.uint8));assert good
    def init(self,*args,**kwargs):
        original_init(self,*args,**kwargs)
        self.camera_enabled=True;self.camera_bytes=data.tobytes()
        self.camera_pub=self.bus.publisher('yolo_observer',CompressedImage,'/yolo/image_annotated/compressed',BE)
    def inputs(self):
        original_inputs(self)
        if self.camera_enabled:
            msg=CompressedImage(format='jpeg',data=self.camera_bytes)
            stamp=round(self.bus.now*1e9);msg.header.stamp.sec=stamp//10**9;msg.header.stamp.nanosec=stamp%10**9
            msg.header.frame_id='camera_frame';self.camera_pub.publish(msg)
    monkeypatch.setattr(Robot,'__init__',init);monkeypatch.setattr(Robot,'inputs',inputs)
    monkeypatch.setattr(rclpy,'init',lambda *a,**k:pytest.fail('real ROS/device runtime forbidden'))
    yield
    assert not rclpy.ok()
