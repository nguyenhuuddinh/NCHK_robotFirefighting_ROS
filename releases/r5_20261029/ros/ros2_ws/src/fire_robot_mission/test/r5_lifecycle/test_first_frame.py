import pytest
from sensor_msgs.msg import CompressedImage
from fire_robot_mission.operational_frame import FrameProof
from harness import Robot


def message(data,stamp=1_000_000_000):
    m=CompressedImage(format='jpeg',data=data);m.header.frame_id='camera_frame'
    m.header.stamp.sec=stamp//10**9;m.header.stamp.nanosec=stamp%10**9;return m


@pytest.mark.parametrize('kind',['no_frame','empty','bad_JPEG','stale','future','wrong_frame','duplicate'])
def test_first_frame_requires_actual_decode_fresh_stamp_and_new_sequence(monkeypatch,kind):
    h=Robot(monkeypatch);c=FrameProof();m=message(h.camera_bytes)
    if kind=='no_frame':assert not c.fresh(1.);return
    if kind=='empty':m.data=b''
    if kind=='bad_JPEG':m.data=b'not a jpeg'
    if kind=='stale':m.header.stamp.sec=0;m.header.stamp.nanosec=600_000_000
    if kind=='future':m.header.stamp.sec=2
    if kind=='wrong_frame':m.header.frame_id='odom'
    if kind=='duplicate':assert c.observe(m,1.,1_000_000_000) is not None
    assert c.observe(m,1.,1_000_000_000) is None and not c.fresh(1.)


def test_valid_frame_total_source_and_receive_age_are_combined(monkeypatch):
    h=Robot(monkeypatch);c=FrameProof();m=message(h.camera_bytes)
    assert c.observe(m,1.,1_200_000_000) is not None
    assert c.fresh(1.09) and not c.fresh(1.11)


@pytest.mark.parametrize('kind',['silent','empty','bad_JPEG'])
def test_camera_node_present_but_no_decodable_frame_never_arms_then_recovers(monkeypatch,kind):
    h=Robot(monkeypatch)
    if kind=='silent':h.camera_enabled=False
    else:h.camera_bytes=b'' if kind=='empty' else b'bad jpeg'
    for _ in range(90):r=h.step('A')
    assert not r['armed'] and not any(r['intents'].values()) and r['camera_frame']['valid_frames']==0
    assert r['operator_feedback']['reason']=='WAIT_FIRST_OR_FRESH_DECODED_CAMERA_FRAME'
    # Restore a real encoded frame without restarting any ROS/device context.
    import cv2
    import numpy as np
    ok,data=cv2.imencode('.jpg',np.zeros((4,4,3),dtype=np.uint8));assert ok
    h.camera_enabled=True;h.camera_bytes=data.tobytes();h.arm()
    assert h.op.frame_proof.valid_frames>0 and h.op.core.armed
