"""Real Qt/RViz view lifecycle, fake sessions, DDS localhost211 only."""
import hashlib
import json
import os
import re
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import pytest
from services.candidate_build import build
APP=Path(__file__).resolve().parents[1]
ROOT=APP.parents[1]
BINARY=Path('/tmp/fire_robot_control_preview_1000/fire_robot_gui_fixture')

@pytest.mark.parametrize('mode',['idle','cycles'])
def test_idle_no_ros_and_repeated_real_view_attach_detach(mode):
    import rclpy
    from rclpy.context import Context
    assert os.environ.get('ROS_DOMAIN_ID')=='211' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    context=Context();rclpy.init(args=['app_view_graph_qa'],context=context)
    probe=rclpy.create_node('app_view_graph_qa',context=context)
    executor=rclpy.executors.SingleThreadedExecutor(context=context);executor.add_node(probe)
    evidence=Path(os.environ.get('FIRE_ROBOT_QA_EVIDENCE','/tmp/fire_app_session1_20261030'));evidence.mkdir(exist_ok=True)
    process=None
    try:
        with tempfile.TemporaryDirectory(prefix='app-view-qa-') as tmp:
            base=Path(tmp);candidate=base/'candidate';maps=base/'maps';maps.mkdir()
            (maps/'old.pgm').write_bytes(b'P5\n2 2\n255\n\x00\x00\xff\xff')
            (maps/'old.yaml').write_text('image: old.pgm\nresolution: 0.05\norigin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
            build(ROOT/'releases/r5_20261029/ros',APP,candidate)
            env=dict(os.environ,FIRE_ROBOT_GUI_QA=mode,FIRE_ROBOT_GUI_QA_CAPTURE=str(evidence/(mode+'.png')))
            env['PYTHONPATH']=os.pathsep.join((str(candidate/'desktop'),str(candidate/'scripts'),env.get('PYTHONPATH','')))
            cmd=[sys.executable,'-B',str(APP/'tests/qa_fake_supervisor.py'),'--candidate',str(candidate),
                 '--binary',str(BINARY),'--binary-sha256',hashlib.sha256(BINARY.read_bytes()).hexdigest(),
                 '--state-dir',str(base/'state'),'--map-root',str(maps),'--maps',str(maps)]
            with (base/'stdout').open('w') as stdout,(base/'stderr').open('w') as stderr:
                process=subprocess.Popen(cmd,env=env,stdout=stdout,stderr=stderr)
                deadline=time.monotonic()+55;gui_log=None;balanced_at=None;graph_samples=[]
                while process.poll() is None and time.monotonic()<deadline:
                    text=(base/'stdout').read_text()
                    logs=[x.removeprefix('UI_LOG=') for x in text.splitlines() if x.startswith('UI_LOG=')]
                    if logs:gui_log=Path(logs[0])
                    log=gui_log.read_text(errors='replace') if gui_log and gui_log.exists() else ''
                    idle=(mode=='idle' and 'GUI_IDLE_ROS_CONTEXT=0' in log)
                    phases=re.findall(r'UI_SESSION_STATUS phase=([A-Z_]+)',log)
                    balanced=(phases and phases[-1]=='IDLE' and log.count('APP_VIEW_ATTACHED')>0
                              and log.count('APP_VIEW_ATTACHED')==log.count('APP_VIEW_DETACHED'))
                    if idle or balanced:
                        if balanced_at is None:balanced_at=time.monotonic()
                        if time.monotonic()-balanced_at>.45:
                            readers={}
                            for topic in ('/tf','/tf_static','/map','/scan','/yolo/image_annotated/compressed','/robot_description','/plan'):
                                names=[x.node_name for x in probe.get_subscriptions_info_by_topic(topic)]
                                readers[topic]=[x for x in names if x.startswith(('fire_robot_control_preview','fire_robot_ui_observer','transform_listener_impl_'))]
                            graph_samples.append(readers)
                    else:balanced_at=None
                    executor.spin_once(timeout_sec=.025)
                assert process.poll() is not None,'GUI timeout'
                assert process.returncode==0,(base/'stderr').read_text()
            assert gui_log is not None
            log=gui_log.read_text(errors='replace');(evidence/(mode+'.gui.log')).write_text(log)
            (evidence/(mode+'.graph.json')).write_text(json.dumps(graph_samples,indent=2)+'\n')
            assert graph_samples,'No graph samples while idle/detached'
            assert all(not names for sample in graph_samples for names in sample.values()),graph_samples
            assert 'APP_VIEW_ATTACH_FAILED' not in log and 'ItemIdentityException' not in log
            if mode=='idle':
                assert 'GUI_IDLE_ROS_CONTEXT=0' in log and 'APP_VIEW_ATTACHED' not in log
            else:
                assert log.count('APP_VIEW_ATTACHED')==6,log[-2500:]
                assert log.count('APP_VIEW_DETACHED')==6,log[-2500:]
                assert log.count('GUI_CYCLE_IDLE_ROS_CONTEXT=0')==3,log[-2500:]
                assert 'GUI_FIXTURE_EXIT_STEP=15' in log
            assert not (base/'state/pending.json').exists()
    finally:
        if process and process.poll() is None:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        executor.shutdown();probe.destroy_node();rclpy.shutdown(context=context)
