"""Render the app's real RViz RobotModel/Path using localhost-only fixtures."""
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest

APP=Path(__file__).resolve().parents[1];ROOT=APP.parents[1]

@pytest.fixture(scope='module')
def visual_result(tmp_path_factory):
    assert os.environ.get('ROS_DOMAIN_ID')=='211' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    import xacro
    base=tmp_path_factory.mktemp('rviz-display-qa')
    xml=xacro.process_file(str(ROOT/'releases/r5_20261029/ros/ros2_ws/src/fire_robot_description/urdf/fire_robot.urdf.xacro')).toxml()
    joints=[]
    for joint in ET.fromstring(xml).findall('joint'):
        assert joint.attrib['type']=='fixed','Fixture must not synthesize any real moving joint state'
        origin=joint.find('origin')
        xyz=[float(x) for x in (origin.get('xyz','0 0 0') if origin is not None else '0 0 0').split()]
        roll,pitch,yaw=[float(x) for x in (origin.get('rpy','0 0 0') if origin is not None else '0 0 0').split()]
        cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2);cy,sy=math.cos(yaw/2),math.sin(yaw/2)
        joints.append(dict(parent=joint.find('parent').get('link'),child=joint.find('child').get('link'),xyz=xyz,
            quaternion=[sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]))
    fixture=base/'fixture.json';fixture.write_text(json.dumps(dict(urdf=xml,joints=joints)))
    evidence=Path(os.environ.get('FIRE_ROBOT_QA_EVIDENCE','/tmp/fire_app_fix_visual_20261030'))
    result=subprocess.run(['/tmp/fire_robot_pre_sync_ui/rviz_display_probe',str(APP/'native_rviz/operational.rviz'),
        str(fixture),str(evidence/'robot_path_native.png')],capture_output=True,text=True,timeout=25)
    (evidence/'robot_path_native.log').write_text(result.stdout+result.stderr)
    assert result.returncode==0,result.stdout+result.stderr
    row=json.loads(next(x.removeprefix('QA_VISUAL ') for x in result.stdout.splitlines() if x.startswith('QA_VISUAL ')))
    (evidence/'robot_path_native.json').write_text(json.dumps(row,indent=2)+'\n')
    return row

@pytest.mark.parametrize('key',['idle_overlays_disabled','idle_overlay_readers_zero','mapping_robot_on_path_off',
    'fire_overlays_enabled','robot_urdf_and_tf_ok','robot_tracks_tf','path_messages_ok','stop_disables_and_clears_overlays',
    'stop_overlay_readers_zero','no_drive_or_pump_publishers'])
def test_real_robot_and_path_display_lifecycle(visual_result,key):
    assert visual_result[key] is True,(key,visual_result)
