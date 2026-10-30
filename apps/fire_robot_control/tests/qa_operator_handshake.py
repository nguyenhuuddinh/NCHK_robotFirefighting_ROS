"""Real supervisor to Qt AppClient to private operator channel; no hardware."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from services.candidate_build import build

APP = Path(__file__).resolve().parents[1]
ROOT = APP.parents[1]
FIXTURE = Path(f'/tmp/fire_robot_control_preview_{os.getuid()}/fire_robot_app_fixture')

def test_supervisor_forwards_operator_connects_qt_and_stop_reaches_channel(tmp_path):
    candidate = tmp_path/'candidate'; state = tmp_path/'state'; maps = tmp_path/'maps'
    maps.mkdir(mode=0o700)
    (maps/'test.pgm').write_bytes(b'P5\n2 2\n255\n\x00\x00\xff\xff')
    (maps/'test.yaml').write_text('image: test.pgm\nresolution: 0.05\norigin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    build(ROOT/'releases/r5_20261029/ros',APP,candidate)
    env = dict(os.environ, ROS_DOMAIN_ID='211', ROS_LOCALHOST_ONLY='1', FIRE_ROBOT_QA_FIXTURE='handshake')
    env['PYTHONPATH'] = os.pathsep.join((str(candidate/'desktop'),str(candidate/'scripts'),env.get('PYTHONPATH','')))
    cmd=[sys.executable,str(APP/'tests/qa_handshake_supervisor.py'),'--candidate',str(candidate),
        '--binary',str(FIXTURE),'--binary-sha256',hashlib.sha256(FIXTURE.read_bytes()).hexdigest(),
        '--state-dir',str(state),'--map-root',str(maps),'--maps',str(maps)]
    process=subprocess.run(cmd,env=env,cwd=candidate/'desktop',capture_output=True,text=True,timeout=18)
    paths=[x.removeprefix('UI_LOG=') for x in process.stdout.splitlines() if x.startswith('UI_LOG=')]
    assert len(paths)==1,(process.stdout,process.stderr)
    log=Path(paths[0]).read_text()
    assert process.returncode==0,(process.stdout,process.stderr,log)
    assert 'FIXTURE_OPERATOR_CONNECTED' in log and 'FIXTURE_CLOSE_READY' in log,log
    results=list(state.glob('session_*/handshake.json'))
    assert len(results)==1
    assert json.loads(results[0].read_text())=={'active':True,'reason':'OPERATOR_Q'}
    assert not (state/'pending.json').exists()
