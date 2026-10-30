"""Additional Qt probes and isolated idle launcher. No ROS publishers or Pi."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import pytest

APP=Path(__file__).resolve().parents[1]
EVIDENCE=Path(os.environ.get('FIRE_ROBOT_QA_EVIDENCE','/tmp/fire_app_qa2_20261030'))


@pytest.fixture(scope='module')
def ui_result():
    env=dict(os.environ,ROS_DOMAIN_ID='211',ROS_LOCALHOST_ONLY='1',QT_QPA_PLATFORM='offscreen')
    row=subprocess.run(['/tmp/fire_robot_pre_sync_ui/offline_supplement_probe'],env=env,capture_output=True,text=True,timeout=8)
    (EVIDENCE/'ui_supplement.log').write_text(row.stdout+row.stderr)
    assert row.returncode==0,row.stdout+row.stderr
    value=next(x.removeprefix('QA_SUPPLEMENT ') for x in row.stdout.splitlines() if x.startswith('QA_SUPPLEMENT '))
    return json.loads(value)


@pytest.mark.parametrize('key',['save_dialog_exists','typing_xq_does_not_stop','save_name_xq_sent_once',
    'keyboard_exists','keyboard_focus_out_sends_zero_key','cancel_auth_requests_stop'])
def test_native_dialog_and_focus_behaviour(ui_result,key):
    assert ui_result[key] is True


@pytest.mark.parametrize('size',[(1366,768),(1920,1080)])
@pytest.mark.parametrize('scale',[1.,1.25,1.5])
@pytest.mark.parametrize('error',['normal','long'])
def test_native_layout_fits_screen_and_keeps_stop_visible(size,scale,error):
    name=f'layout_{size[0]}_{size[1]}_{scale}_{error}'
    env=dict(os.environ,ROS_DOMAIN_ID='211',ROS_LOCALHOST_ONLY='1',QT_QPA_PLATFORM='offscreen',QT_SCALE_FACTOR=str(scale))
    row=subprocess.run(['/tmp/fire_robot_pre_sync_ui/layout_supplement_probe',str(size[0]),str(size[1]),error,
         str(EVIDENCE/(name+'.png'))],env=env,capture_output=True,text=True,timeout=8)
    (EVIDENCE/(name+'.log')).write_text(row.stdout+row.stderr)
    assert row.returncode==0,row.stdout+row.stderr
    value=json.loads(next(x.removeprefix('QA_LAYOUT ') for x in row.stdout.splitlines() if x.startswith('QA_LAYOUT ')))
    (EVIDENCE/(name+'.json')).write_text(json.dumps(value,indent=2)+'\n')
    assert value['fits_requested_screen'],value
    assert value['stop_visible_enabled'] and value['progress_visible'],value
    assert value['camera_height']>=180 and value['camera_width']>=280,value


def test_packaged_launcher_opens_idle_from_different_cwd(tmp_path):
    artifact=Path(os.environ.get('FIRE_ROBOT_QA_ARTIFACT',str(APP/'artifacts/b3b81081d95e7c1a0ec974a29f61654dc517263594132977df1426069573c828')))
    cwd=tmp_path/'cwd with spaces';cwd.mkdir()
    env=dict(os.environ,XDG_STATE_HOME=str(tmp_path/'private-state'),ROS_DOMAIN_ID='211',ROS_LOCALHOST_ONLY='1',
             QT_QPA_PLATFORM='offscreen',PYTHONDONTWRITEBYTECODE='1')
    child=subprocess.Popen(['bash',str(APP/'launchers/run_app.sh'),str(artifact)],cwd=cwd,env=env,
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        time.sleep(1.2)
        assert child.poll() is None,'Launcher or UI exited before idle test'
        child.send_signal(signal.SIGTERM)
        stdout,stderr=child.communicate(timeout=8)
        (EVIDENCE/'launcher_other_cwd.log').write_text(stdout+stderr)
        logs=[line.removeprefix('UI_LOG=') for line in stdout.splitlines() if line.startswith('UI_LOG=')]
        if logs:shutil.copy2(logs[0],EVIDENCE/'launcher_other_cwd_gui.log')
        assert child.returncode==0,(stdout,stderr)
        assert not (tmp_path/'private-state/fire_robot_control/pending.json').exists()
    finally:
        if child.poll() is None:
            child.terminate()
            try:child.wait(timeout=8)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
