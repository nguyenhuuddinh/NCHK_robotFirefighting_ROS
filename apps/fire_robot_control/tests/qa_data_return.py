"""Regression acceptance for the app data-return and mission-rebuild fixes.

Uses actual app nodes on a memory bus. No ROS init, SSH, hardware or changes
to production. Diagnostic tests describe why a hold exists, not motion PASS.
"""
import ast
import copy
import json
import sys
from pathlib import Path
from types import SimpleNamespace as N

import pytest

from qa_goal_retry_audit import app_class, simulated_camera_and_no_ros_context
from qa_nav_wait_recovery import moving_robot

ROOT = Path.cwd()
APP = ROOT/'apps/fire_robot_control'
sys.path.insert(0,str(ROOT/'releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission/test/r5_paired'))
PACKET = '62c78ddf92eacb5d800f0b8137eeb1223302ad6e2ee9f557bddf0558e5ad0e4b'
GAPS = {
    'nav_gateway': ('/fire_mission/nav2_cmd_vel_shadow', 'live_nav_stop_gateway'),
    'odom_gateway': ('/odom', 'live_nav_stop_gateway'),
    'mission_gateway': ('/fire_mission/state', 'live_nav_stop_gateway'),
    'arm_gateway': ('/fire_mission/live_nav_arm', 'live_nav_stop_gateway'),
    'odom_all': ('/odom', None),
    'actuator_all': ('/fire_sensors/actuator_state', None),
    'identity_operator': ('/fire_sensors/serial_session_identity', 'fire_robot_operator'),
    'identity_all': ('/fire_sensors/serial_session_identity', None),
}


def snapshot(h, report):
    c = h.by_name['live_nav_stop_gateway'].core
    return dict(now=h.bus.now, x=h.x, yaw=h.yaw, phase=report.get('phase'), reason=report.get('reason'),
        fault=report.get('fault'), goal=c.goal_uuid, released=c.released, stop_latched=c.stop_latched,
        stop_reason=c.stop_reason, recovery=c.app_nav_recovery, nav_age=h.bus.now-c.nav_at,
        odom_age=h.bus.now-c.odom_at, mission_age=h.bus.now-c.mission_at, arm_age=h.bus.now-c.arm_at,
        raw_count=sum(p.topic=='/cmd_vel_raw' for p in h.bus.pubs), max_raw=h.bus.max_raw,
        pulses=len(h.pulses), statuses=list(h.status))


@pytest.mark.parametrize('gap', GAPS)
@pytest.mark.parametrize('ticks', [4, 16, 40])
def test_same_identity_data_returns_should_resume_same_goal(monkeypatch,tmp_path,app_class,gap,ticks):
    h, mode = moving_robot(monkeypatch,tmp_path)
    topic, node = GAPS[gap]
    h.bus.drop = lambda p,s,m: p.topic==topic and (node is None or s.info.node_name==node)
    for _ in range(ticks): report=h.step()
    held=snapshot(h,report)
    h.bus.drop=lambda *args:False
    for _ in range(200): report=h.step()
    restored=snapshot(h,report)
    moved=h.x>held['x']+.015
    h.step('Q')
    stopped=h.until(lambda r:r.get('shutdown',{}).get('done'),limit=180)
    row=dict(gap=gap,duration_s=ticks*.05,held=held,restored=restored,resumed_same_goal=moved,
        q=stopped['shutdown'],scope='MEMORY_BUS_NOT_HARDWARE')
    (tmp_path/f'return_{gap}_{ticks}.json').write_text(json.dumps(row,indent=2)+'\n')
    assert not h.pulses and h.bus.max_raw<=1
    assert stopped['shutdown']['confirmed'],row
    assert moved,row


def test_recovery_factory_should_keep_the_app_mission_launch(monkeypatch,tmp_path):
    from operational_entry import ros, launch
    from services.desktop_hooks import DesktopHooks
    import services.app_model
    monkeypatch.setattr(services.app_model,'validate_model',lambda:tmp_path/'model-fixture.pt')
    candidate=APP/'artifacts'/PACKET/'ros'
    from services.candidate_build import patch_runner
    tree=ast.parse(patch_runner((ROOT/'releases/r5_20261029/ros/scripts/r5_laptop_session.py').read_text()))
    command_def=next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=='commands')
    env=dict(ROOT=candidate,ros=ros,launch=launch)
    exec(compile(ast.Module(body=[command_def],type_ignores=[]),'<shipped commands>','exec'),env)
    command=env['commands']
    hooks=object.__new__(DesktopHooks)
    hooks.app=candidate/'desktop';hooks.out=tmp_path;hooks.runtime=tmp_path
    hooks.binding=N(session='session');hooks.operator=N(command=['fake-operator'])
    a=N(map=tmp_path/'map.yaml',rviz=tmp_path/'rviz.rviz')
    initial=hooks.commands(command(a.map,a.rviz,gate2=True,automatic=True,
        auto_start=tmp_path/'automatic_start.json',auto_session='session',auto_packet=PACKET))
    call=next(n for n in ast.walk(tree) if isinstance(n,ast.Call)
              and isinstance(n.func,ast.Name) and n.func.id=='RuntimeRecoveryRunner')
    factory=eval(compile(ast.Expression(call.args[4]),'<shipped recovery lambda>','eval'),
                 dict(commands=command,a=a,packet=PACKET,desktop=hooks))
    resumed=factory(tmp_path/'automatic_start.json','session',False)
    row=dict(initial_mission=initial['mission'],recovery_mission=resumed['mission'],
             app_wrapper_preserved=initial['mission']==resumed['mission'],scope='AST_FACTORY_NO_PROCESSES')
    (tmp_path/'recovery_factory.json').write_text(json.dumps(row,indent=2)+'\n')
    assert row['app_wrapper_preserved'],row


def test_diagnostic_invalid_pose_latches_until_actual_new_pose(monkeypatch,tmp_path):
    from fire_robot_mission.startup_health import HealthCore,StartupHealthNode
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from test_health import ready,report
    core=HealthCore('session','laptop');core.initial_pose_at=1.
    age=[.1]
    host=N(core=core,age=lambda message:age[0])
    now=[2.]
    import time
    monkeypatch.setattr(time,'monotonic',lambda:now[0])
    msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.pose.pose.orientation.w=1.
    StartupHealthNode.pose(host,msg)
    ready(core,2.);assert report(core,2.)['checks']['localization']
    age[0]=.7;now[0]=3.;StartupHealthNode.pose(host,msg)
    for n in range(100):
        now[0]=4.+n*.1;ready(core,now[0]);r=report(core,now[0])
        assert r['checks']['io'] and not r['checks']['localization']
    age[0]=.1;now[0]+=.1;StartupHealthNode.pose(host,msg);ready(core,now[0])
    assert report(core,now[0])['checks']['localization']
    (tmp_path/'pose_latch_diagnostic.json').write_text(json.dumps(dict(
        diagnostic='LATE_INVALID_AMCL_POSE_NEEDS_NEW_VALID_POSE_EVENT',
        io_returns_alone_not_sufficient=True,exact_runtime_invalid_field='NOT_RECORDED_IN_RAW'),indent=2)+'\n')


def test_diagnostic_Q_during_epoch_rebind_needs_current_epoch_stop_proof(monkeypatch,tmp_path,app_class):
    h,mode=moving_robot(monkeypatch,tmp_path)
    emit=h.emit
    def new_epoch(topic,value):
        if topic=='/fire_sensors/serial_session_identity':
            value=copy.deepcopy(value);value['connection_epoch']=2
        emit(topic,value)
    h.emit=new_epoch
    h.until(lambda r:r['fault']=='BOOT_OR_SERIAL_EPOCH_CHANGED')
    h.step('Q')
    for _ in range(180):
        r=h.step()
        if r.get('shutdown',{}).get('done'):break
    assert r.get('shutdown',{}).get('done')
    assert not r['shutdown']['confirmed']
    assert 'STOP_BOOT_EPOCH_CHANGED' in r['shutdown']['blockers']
    assert not h.pulses and h.bus.max_raw<=1
    (tmp_path/'q_epoch_diagnostic.json').write_text(json.dumps(r['shutdown'],indent=2)+'\n')


@pytest.mark.parametrize('order', ['nav_then_odom','odom_then_nav','both'])
def test_overlapping_gaps_should_resume_when_all_current_inputs_return(monkeypatch,tmp_path,app_class,order):
    h,mode=moving_robot(monkeypatch,tmp_path)
    wanted=set()
    h.bus.drop=lambda p,s,m:s.info.node_name=='live_nav_stop_gateway' and p.topic in wanted
    if order=='nav_then_odom':wanted.add('/fire_mission/nav2_cmd_vel_shadow')
    elif order=='odom_then_nav':wanted.add('/odom')
    else:wanted.update(('/odom','/fire_mission/nav2_cmd_vel_shadow'))
    for _ in range(25):h.step()
    wanted.update(('/odom','/fire_mission/nav2_cmd_vel_shadow'))
    for _ in range(25):report=h.step()
    held=snapshot(h,report);wanted.clear()
    for _ in range(200):report=h.step()
    row=dict(order=order,held=held,restored=snapshot(h,report),resumed=h.x>held['x']+.015)
    h.step('Q');r=h.until(lambda r:r.get('shutdown',{}).get('done'),limit=180)
    row['q_confirmed']=r['shutdown']['confirmed']
    (tmp_path/f'overlap_{order}.json').write_text(json.dumps(row,indent=2)+'\n')
    assert row['q_confirmed'] and h.bus.max_raw<=1 and not h.pulses
    assert row['resumed'],row


@pytest.mark.parametrize('gap',['odom_gateway','mission_gateway'])
def test_X_still_cancels_held_goal_and_new_goal_moves(monkeypatch,tmp_path,app_class,gap):
    h,mode=moving_robot(monkeypatch,tmp_path)
    topic,node=GAPS[gap]
    h.bus.drop=lambda p,s,m:p.topic==topic and s.info.node_name==node
    for _ in range(25):h.step()
    h.bus.drop=lambda *args:False
    h.step('X');h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL',limit=200)
    before=h.x
    for _ in range(20):h.step()
    assert h.x==before
    h.status.append(('56'*16,2));h.until(lambda r:h.x>before+.01)
    assert h.bus.max_raw<=1 and not h.pulses
    (tmp_path/f'x_escape_{gap}.json').write_text(json.dumps(dict(canceled=h.bus.cancelled,
        next_goal_moved=True,no_old_goal_replay=True,scope='MEMORY_BUS_NOT_HARDWARE'),indent=2)+'\n')


@pytest.mark.parametrize('goal', [False, True])
def test_app_preserves_R5_three_six_second_sprays_and_off_home(monkeypatch,tmp_path,app_class,goal):
    from test_ir_freshness_hold_115816 import robot
    h=robot(monkeypatch,tmp_path,goal=goal)
    actor=h.by_name['fire_pulse_controller']
    assert actor.session.max_pulses == 3
    assert actor.session.pump.pulse_s == 6. and actor.session.pump.start_on_ack
    h.fire=True; h.ir_flame=True
    h.until(lambda r:abs(h.yaw)>.025); h.centered=True
    h.until(lambda r:h.x>.025); h.right=950
    h.until(lambda r:h.home==3,limit=800)
    assert len(h.pulses)==3
    for pulse in h.pulses:
        assert 6. <= pulse['off_at']-pulse['on_at'] <= 6.21
        assert pulse['home_after'] > pulse['home_before']
    # Quota retirement prevents an automatic fourth consecutive ON.
    for _ in range(50): h.step()
    assert len(h.pulses)==3 and not h.device_on and not h.pump_requested
    h.step('Q'); report=h.until(lambda r:r.get('shutdown',{}).get('done'),limit=180)
    assert report['shutdown']['confirmed'] and h.bus.max_raw<=1


def test_upgrading_packet_keeps_old_pending_and_only_allows_fresh_stop(tmp_path):
    from services.supervisor import SessionController
    import uuid
    state=tmp_path/'state'; state.mkdir()
    runtime=tmp_path/'runtime'; runtime.mkdir()
    maps=tmp_path/'maps'; maps.mkdir()
    marker=state/'pending.json'
    old=dict(session=str(uuid.uuid4()),packet='a'*64,out=str(state/'session_old'))
    marker.write_text(json.dumps(old)); before=marker.read_bytes()
    calls=[]
    def factory(**fields):
        calls.append(fields)
        return N(session=str(uuid.uuid4()),start=lambda:None)
    c=SessionController(candidate=tmp_path/'new_candidate',packet='b'*64,
        state_dir=state,runtime=runtime,map_roots=[maps],map_root=maps,session_factory=factory)
    assert c.phase=='BLOCKED'
    c.action('SCAN'); c.action('FIRE')
    assert not calls and marker.read_bytes()==before
    c.action('VERIFY_STOP')
    assert len(calls)==1 and calls[0]['kind']=='verify_stop' and calls[0]['packet']=='b'*64
    assert marker.read_bytes()==before and c.phase=='STARTING'
