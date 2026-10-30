"""Worker → supervisor → actual Qt password modal → IPC → worker; no SSH."""
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import pytest
from services.candidate_build import build
APP=Path(__file__).resolve().parents[1];ROOT=APP.parents[1]
@pytest.mark.parametrize('mode',['accept','cancel','timeout'])
def test_actual_auth_dialog_round_trip_without_ssh_or_secret_logs(mode):
    assert os.environ.get('ROS_DOMAIN_ID')=='211' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    binary=Path(os.environ.get('FIRE_AUTH_QA_BINARY','/tmp/fire_robot_pre_sync_ui/auth_relay_probe'))
    with tempfile.TemporaryDirectory(prefix='auth-relay-qa-') as tmp:
        p=Path(tmp);candidate=p/'candidate';maps=p/'maps';maps.mkdir()
        build(ROOT/'releases/r5_20261029/ros',APP,candidate)
        env=dict(os.environ,QT_QPA_PLATFORM='offscreen',FIRE_AUTH_QA_MODE=mode)
        env['PYTHONPATH']=os.pathsep.join((str(candidate/'desktop'),str(candidate/'scripts'),env.get('PYTHONPATH','')))
        result=subprocess.run([sys.executable,'-B',str(APP/'tests/auth_fake_supervisor.py'),
            '--candidate',str(candidate),'--binary',str(binary),'--binary-sha256',hashlib.sha256(binary.read_bytes()).hexdigest(),
            '--state-dir',str(p/'state'),'--map-root',str(maps),'--maps',str(maps)],
            env=env,capture_output=True,text=True,timeout=15)
        log=Path(next(x.removeprefix('UI_LOG=') for x in result.stdout.splitlines() if x.startswith('UI_LOG='))).read_text()
        evidence=Path(os.environ.get('FIRE_ROBOT_QA_EVIDENCE','/tmp/fire_app_auth_fix_20261030'))
        (evidence/('auth_'+mode+'.log')).write_text(result.stdout+result.stderr+log)
        assert result.returncode==0,(result.stdout,result.stderr,log)
        assert 'AUTH_QA dialog=1 masked=1 done='+('1' if mode=='accept' else '0')+' finished=1' in log
        assert 'fixture-only' not in log+result.stdout+result.stderr
        assert not (p/'state/pending.json').exists()
