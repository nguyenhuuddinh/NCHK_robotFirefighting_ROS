"""Reproduce the old native GIL failure and exercise the process replacement."""
import json
import os
from pathlib import Path
import subprocess
import sys


def test_real_native_graph_delay_before_and_after(tmp_path):
    app=Path(__file__).resolve().parents[1]
    source=app/'docs/qa_evidence/heartbeat_461ce77_20261030/offline_probe/graph_delay.c'
    library=tmp_path/'native_delay.so'
    subprocess.run(['gcc','-shared','-fPIC',str(source),'-ldl','-o',str(library)],check=True)
    fixture=Path(f'/tmp/fire_robot_control_preview_{os.getuid()}/fire_robot_frontend_fixture')
    assert fixture.is_file(), 'Build the current Qt fixture before this QA'
    env=dict(os.environ,ROS_DOMAIN_ID='211',ROS_LOCALHOST_ONLY='1',LD_PRELOAD=str(library))
    result=subprocess.run([sys.executable,str(app/'tests/native_liveness_probe.py'),str(fixture)],
        env=env,capture_output=True,text=True,timeout=12.)
    assert result.returncode==0,result.stdout+'\n'+result.stderr
    rows=json.loads(next(line.split('=',1)[1] for line in result.stdout.splitlines() if line.startswith('NATIVE_LIVENESS_RESULT=')))
    assert rows[0]['key']=='Q' and rows[1]['key'] is None
