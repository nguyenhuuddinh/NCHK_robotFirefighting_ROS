"""New regression cases only: pinned TF offset + startup diagnostic/record delta.
No ROS context, DDS, SSH, timers, NTP or device access.
"""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as N
import sys
import pytest
from fire_robot_mission import startup_health as health
from test_health import ready,report
from test_async_health import Future,Client
sys.path.insert(0,str(Path(__file__).parents[1]/'r5_full_adapter'))
from harness import Bus
from fire_robot_bringup.odom_to_tf_broadcaster import OdomToTfBroadcaster


def fake_node(monkeypatch,module=health):
    bus=Bus(monkeypatch);bus.now=10.
    broadcaster=object.__new__(OdomToTfBroadcaster)
    broadcaster._stamp_source='pi_receive_time';broadcaster._stamp_offset_ns=-100_000_000
    broadcaster._last_odom_time_ns=10_000_000_000
    def lookup(parent,child,at):
        if parent=='map':raise module.TransformException('no initial pose')
        return N(header=N(stamp=broadcaster._get_tf_stamp()))
    monkeypatch.setattr(module,'Buffer',lambda:N(lookup_transform=lookup))
    monkeypatch.setattr(module,'TransformListener',lambda *a:None)
    monkeypatch.setattr(module,'ActionClient',lambda *a:N(server_is_ready=lambda:False))
    node=module.StartupHealthNode('delta','laptop')
    monkeypatch.setattr(node,'source',lambda k:True)
    monkeypatch.setattr(node,'poll_services',lambda:None)
    node.control={'safety':(True,bus.now),'odom_tf_profile':(True,bus.now)}
    return node,bus,broadcaster


def test_actual_baseline_broadcaster_10hz_future_offset_old_gate_reproduces_new_gate_continuous(monkeypatch):
    # Execute the unchanged baseline stamp method, not a duplicate formula.
    before=Path(__file__).parents[6]/'evidence_r5_5_startup_235627/before/ros/ros2_ws/src/fire_robot_mission/fire_robot_mission/startup_health.py'
    spec=importlib.util.spec_from_file_location('fire_robot_mission.startup_health_before_235627',before)
    old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    results={}
    for name,module in [('before',old),('after',health)]:
        node,bus,broadcaster=fake_node(monkeypatch,module);values=[];stable=[]
        for tick in range(251):
            bus.now=10.+tick*.01
            if tick%10==0:
                broadcaster._last_odom_time_ns=round(bus.now*1e9)
                ready(node.core,bus.now)
            r=node.sample();values.append(r['checks']['io']);stable.append(r['stable_s'].get('io',0))
        results[name]={'true':sum(values),'false':values.count(False),'max_stable':max(stable)}
    assert results['before']['false']>150 and results['before']['max_stable']<.1
    assert results['after']['false']==0 and results['after']['max_stable']>=2.5


@pytest.mark.parametrize('source_age,expected',[(0.,True),(.1,True),(.299,True),(.301,False),(-.021,False),(-.019,True)])
def test_normalization_keeps_source_age_bound_not_arbitrary_future_tolerance(monkeypatch,source_age,expected):
    node,bus,broadcaster=fake_node(monkeypatch)
    broadcaster._last_odom_time_ns=round((bus.now-source_age)*1e9)
    ready(node.core,bus.now);r=node.sample()
    assert r['checks']['io'] is expected
    assert r['io_diagnostics']['detail']['odom_tf']['source_age_ms']==pytest.approx(source_age*1000)


@pytest.mark.parametrize('profile',[(False,10.),(True,-6.),(True,11.),None])
def test_unknown_stale_future_or_rejected_profile_never_admits_tf(monkeypatch,profile):
    node,bus,broadcaster=fake_node(monkeypatch)
    if profile is None:node.control.pop('odom_tf_profile')
    else:node.control['odom_tf_profile']=profile
    ready(node.core,bus.now);r=node.sample()
    assert not r['checks']['io'] and 'odom_tf' in r['io_diagnostics']['blockers']


@pytest.mark.parametrize('change',['none','offset','mode','stale','type','length','exception'])
def test_parameter_response_exact_profile_and_request_fields(monkeypatch,change):
    vals=[N(type=4,string_value='pi_receive_time'),N(type=2,integer_value=-100),N(type=2,integer_value=300)]
    if change=='offset':vals[1].integer_value=-200
    if change=='mode':vals[0].string_value='pi_now'
    if change=='stale':vals[2].integer_value=1000
    if change=='type':vals[1].type=3
    if change=='length':vals.pop()
    f=Future(ValueError('fake') if change=='exception' else N(values=vals),True)
    node=object.__new__(health.StartupHealthNode);node.control={};node.pending={};node.next_query={}
    node.control_clients={'odom_tf_profile':Client(f)}
    monkeypatch.setattr(node,'get_logger',lambda:N(warning=lambda *a:None))
    node.poll_services();node.poll_services()
    assert node.control['odom_tf_profile'][0] is (change=='none')
    assert node.control_clients['odom_tf_profile'].sent[0].names==['tf_stamp_source','tf_stamp_offset_ms','odom_stale_timeout_ms']


def test_cached_tf_rejected_even_with_new_fresh_odom_payload(monkeypatch):
    node,bus,broadcaster=fake_node(monkeypatch);ready(node.core,bus.now);assert node.sample()['checks']['io']
    bus.now+=.31;ready(node.core,bus.now);r=node.sample()
    assert not r['checks']['io'] and r['io_diagnostics']['checks']['odom_stationary']


def test_actual_profile_matches_pinned_yaml():
    import yaml
    p=Path(__file__).parents[3]/'fire_robot_bringup/config/pi_params.yaml'
    cfg=yaml.safe_load(p.read_text())['odom_to_tf_broadcaster']['ros__parameters']
    assert tuple(cfg[k] for k in ('tf_stamp_source','tf_stamp_offset_ms','odom_stale_timeout_ms'))==health.ODOM_TF_PROFILE


@pytest.mark.parametrize('bad',['identity','actuator','odom','cmd','scan','tf','graph','safety'])
def test_diagnostics_identify_each_blocker_and_do_not_relax_io(monkeypatch,bad):
    h=health.HealthCore('delta','pi');ready(h,10.);kw={}
    if bad in ('identity','actuator','odom','cmd','scan'):h.inputs.pop(bad)
    if bad=='tf':kw['odom_tf_ok']=False
    if bad=='graph':kw['sources']={'io':False}
    if bad=='safety':kw['safety_ok']=False
    r=report(h,10.,**kw);expected={'identity':'identity','actuator':'actuator_off','odom':'odom_stationary','cmd':'cmd_zero','scan':'scan','tf':'odom_tf','graph':'graph','safety':'safety_route'}[bad]
    assert not r['checks']['io'] and expected in r['io_diagnostics']['blockers']
    json.dumps(r,allow_nan=False)


def test_bounded_transition_history_retains_failure_counts_and_first_failure():
    h=health.HealthCore('delta','pi')
    for tick in range(100):
        t=10+tick*.01;ready(h,t);r=report(h,t,odom_tf_ok=bool(tick%2))
    d=r['io_diagnostics'];assert d['transition_count']==100 and len(d['recent_transitions'])==16
    assert d['false_sample_counts']['odom_tf']==50 and d['first_failure']['sample_mono']==10
    assert r['stable_s']['io']==0
    ready(h,11.);report(h,11.);ready(h,11.2);r=report(h,11.2)
    assert r['io_diagnostics']['max_stable_s']>=.2


def test_startup_timeout_final_record_matches_failure_and_preserves_predrain_health(monkeypatch,tmp_path):
    import r5_laptop_session as lap
    now=[10.];events=[];out=tmp_path/'run';pair=tmp_path/'pair';pair.write_text('{"packet":"test"}')
    m=tmp_path/'map';m.touch();v=tmp_path/'view';v.touch()
    monkeypatch.setattr(lap,'preflight',lambda *a:None);monkeypatch.setattr(lap,'packet_digest',lambda:'test')
    monkeypatch.setattr(lap.Path,'home',lambda:tmp_path)
    monkeypatch.setattr(lap.time,'monotonic',lambda:now[0]);monkeypatch.setattr(lap.time,'sleep',lambda dt:now.__setitem__(0,now[0]+dt))
    monkeypatch.setattr(lap,'run_clock_gate',lambda *a:None)
    class Peer:
        def __init__(self,*a):pass
        def startup(self,cmd):return {'phase':'WAIT_LAPTOP'}
        def stop(self):events.append('pi_stop');return {'confirmed':True}
    class Local:
        def __init__(self,*a):self.processes={}
        def start(self,role,*a,**kw):assert role=='startup_health';self.processes[role]=object()
        def check(self,*a):pass
        def drain(self,**kw):events.append('local_drain');return {'clean':False}
        def close_logs(self):pass
    monkeypatch.setattr(lap,'SshPi',Peer);monkeypatch.setattr(lap,'LocalProcesses',Local)
    sample={'checks':{'io':False,'camera':True},'stable_s':{'camera':10.},'io_diagnostics':{'blockers':['odom_tf']}}
    monkeypatch.setattr(lap,'read_health',lambda *a:sample)
    assert lap.main(['--run-observe','--pairing',str(pair),'--remote-script','/peer','--out',str(out),'--map',str(m),'--rviz',str(v)])==2
    failure=json.loads((out/'failure.json').read_text());startup=json.loads((out/'startup.json').read_text())
    assert startup==failure['startup'] and startup['done'] and startup['error']=='STARTUP_TIMEOUT:PI_IO'
    assert json.loads((out/'health_at_failure.json').read_text())==sample
    assert events==['local_drain','pi_stop']


@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-float('inf')])
def test_bad_numeric_diagnostics_stay_serializable_and_blocked(bad):
    h=health.HealthCore('delta','pi');ready(h,10.)
    h.update('odom',[bad,0,0,0,0,0],10.)
    h.inputs['identity'][0]['state_age_ms']=bad
    h.inputs['actuator'][0]['receive_age_ms']=bad
    r=report(h,10.)
    assert not r['checks']['io']
    assert {'odom_stationary','identity','actuator_off'}<=set(r['io_diagnostics']['blockers'])
    json.dumps(r,allow_nan=False)


def test_tf_profile_timeout_revokes_proof_and_fresh_retry_recovers(monkeypatch):
    now=[10.];monkeypatch.setattr(health.time,'monotonic',lambda:now[0])
    future=Future();client=Client(future)
    node=object.__new__(health.StartupHealthNode);node.control={'odom_tf_profile':(True,9.)}
    node.pending={};node.next_query={};node.control_clients={'odom_tf_profile':client}
    node.poll_services();now[0]=13.;node.poll_services()
    assert future.cancelled and node.control['odom_tf_profile']==(False,13.)
    client.future=Future(N(values=[N(type=4,string_value='pi_receive_time'),N(type=2,integer_value=-100),N(type=2,integer_value=300)]),True)
    node.poll_services();node.poll_services()
    assert node.control['odom_tf_profile']==(True,13.)


def test_out_of_order_tf_not_admitted_by_offset_correction(monkeypatch):
    node,bus,broadcaster=fake_node(monkeypatch);ready(node.core,bus.now)
    assert node.sample()['checks']['io']
    bus.now+=.01;broadcaster._last_odom_time_ns-=1_000_000
    ready(node.core,bus.now);assert not node.sample()['checks']['io']
