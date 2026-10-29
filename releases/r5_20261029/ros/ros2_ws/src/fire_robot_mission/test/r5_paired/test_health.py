import copy
import pytest
from fire_robot_mission.startup_health import HealthCore
from fire_robot_mission.operational_contract import FIRMWARE_ELF_SHA256


def ready(h,now):
    h.update('identity',dict(schema_version=1,bridge_instance_id='ab'*16,connection_epoch=1,
        elf_sha256=FIRMWARE_ELF_SHA256,frame_protocol=2,source='SERIAL_V2_BOOT_WITH_LIVE_STATE',valid=True,session_ready=True,
        telemetry_healthy=True,state_age_ms=10,state_esp_ms=100),now)
    h.update('actuator',dict(schema_version=1,mode='COMMAND_TELEMETRY_ONLY',pump_pwm_ready=True,home_command_completed_count=0,receive_age_ms=10,pump_pwm_commanded_on=False,servo_pwm_commanded_on=False,servo_phase='IDLE'),now)
    h.update('odom',[0.]*6,now);h.update('cmd',[0.]*6,now);h.update('scan',True,now)


def report(h,now,**changes):
    args=dict(sources=dict(io=True,camera=True,pose=True,annotated=True),tf_ok=True,odom_tf_ok=True,action_ready=True,safety_ok=True,lifecycle_ok=True)
    args.update(changes);return h.report(now,**args)


def test_pose_after_initial_can_be_retained_while_static_amcl_silent_with_current_tf_and_services():
    h=HealthCore('session','laptop');h.initial_pose_at=1.;h.update('pose',True,2.)
    ready(h,10.);r=report(h,10.);assert r['checks']['io'] and r['checks']['pose']
    assert not r['checks']['camera'] and not r['commands_published']


@pytest.mark.parametrize('bad',['safety','lifecycle','action','tf','odom_tf','old_pose','no_initial','moving','stale','epoch','source'])
def test_unproven_inputs_never_pose_ready(bad):
    h=HealthCore('session','laptop');h.initial_pose_at=1.;h.update('pose',True,2.);ready(h,10.);args={}
    if bad in ('safety','lifecycle','action','tf','odom_tf'):args[dict(safety='safety_ok',lifecycle='lifecycle_ok',action='action_ready',tf='tf_ok',odom_tf='odom_tf_ok')[bad]]=False
    if bad=='old_pose':h.update('pose',True,.5)
    if bad=='no_initial':h.initial_pose_at=None
    if bad=='moving':h.update('odom',[.1,0,0,0,0,0],10.)
    if bad=='stale':ready(h,8.)
    if bad=='epoch':h.inputs['identity'][0]['valid']=False
    if bad=='source':args['sources']=dict(io=False,pose=True,camera=True,annotated=True)
    assert not report(h,10.,**args)['checks']['pose']


def test_bad_camera_and_stale_source_never_count_as_readiness():
    h=HealthCore('session','laptop');ready(h,10.)
    assert report(h,10.)['checks']['camera'] is False
    assert not report(h,11.)['checks']['io']


@pytest.mark.parametrize('bad',[None,'fault','closed','armed','physical','live','reason'])
def test_consumers_ready_needs_actual_fresh_observer_report_not_process_presence(bad):
    h=HealthCore('session','laptop')
    op=dict(schema_version=1,scope='R5_SESSION_INTENT_ONLY',phase='WAIT_ARM',reason='READY_PRESS_A',fault=None,
        closed=False,armed=False,physical_commands_published=False,live_intents_published=False,motion_authorized=False)
    if bad=='fault':op['fault']='GRAPH_SOURCE_CHANGED'
    if bad in ('closed','armed'):op[bad]=True
    if bad=='physical':op['physical_commands_published']=True
    if bad=='live':op['live_intents_published']=True
    if bad=='reason':op['reason']='WAIT_STABLE_STANDBY'
    h.update('operator',op,10.)
    kwargs=dict(sources=dict(operator=True))
    assert report(h,10.,**kwargs)['checks']['operator'] is (bad is None)
    assert not report(h,11.,**kwargs)['checks']['operator']
