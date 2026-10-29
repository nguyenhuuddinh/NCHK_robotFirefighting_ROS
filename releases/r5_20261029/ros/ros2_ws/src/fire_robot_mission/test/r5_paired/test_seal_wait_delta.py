"""Paired startup seal regressions with fake transport/processes; never ROS/device I/O."""
import importlib.util
import json
from pathlib import Path
import shlex
from types import SimpleNamespace as N
import pytest
import operational_entry as entry
import operational_peer as peer
import r5_laptop_session as laptop
from r5_pi_session import PiController
from test_pi_protocol import B,controller


def old_module(file):
    path=Path(__file__).parents[6]/'evidence_r5_5_startup_003210/before/ros/scripts'/file
    spec=importlib.util.spec_from_file_location('old_'+path.stem,path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod


def test_old_unready_pi_seal_reproduces_terminal_exception_new_wait_recovers():
    old=old_module('r5_pi_session.py');h={'io':False};events=[]
    p=old.PiController(B,lambda:events.append('start'),lambda:None,lambda:None,lambda:h,0)
    p.command('START',1)
    with pytest.raises(ValueError,match='PI_NOT_IO_READY'):p.command('SEAL',2)
    p=PiController(B,lambda:None,lambda:None,lambda:None,lambda:h,0)
    p.command('START',1);r=p.command('SEAL',2)
    assert r['seal_requested'] and not r['sealed'] and r['phase']=='IO_STARTING'
    h['io']=True;r=p.tick(3)
    assert r['sealed'] and r['phase']=='RUNNING_OBSERVE' and r['seal_confirmed_at']==3
    assert r['seal_health_at_request']['io'] is False


def test_file_replaced_between_pre_read_clock_and_read_reproduces_old_false_negative(tmp_path):
    p=tmp_path/'health';p.write_text(json.dumps(dict(schema_version=1,session='x',commands_published=False,sample_mono=10.01)))
    with pytest.raises(ValueError,match='STALE_OR_FOREIGN'):
        old_module('operational_entry.py').read_health(p,'x',10.)
    calls=[]
    def clock():calls.append(p.read_text());return 10.02
    assert entry.read_health(p,'x',clock)['sample_mono']==10.01 and len(calls)==1


@pytest.mark.parametrize('bad',['stale','future','session','command','schema','nan'])
def test_post_read_clock_does_not_relax_freshness_or_identity(tmp_path,bad):
    row=dict(schema_version=1,session='x',commands_published=False,sample_mono=10.)
    if bad=='stale':row['sample_mono']=9.249
    if bad=='future':row['sample_mono']=10.001
    if bad=='session':row['session']='other'
    if bad=='command':row['commands_published']=True
    if bad=='schema':row['schema_version']=2
    if bad=='nan':row['sample_mono']=float('nan')
    p=tmp_path/'health';p.write_text(json.dumps(row))
    with pytest.raises(ValueError):entry.read_health(p,'x',lambda:10.)


def to_seal():
    s=entry.StartupSequence(0)
    assert s.step(1,{'clock_ok':True})==['START_PI','START_HEALTH']
    assert s.step(2,{'io':True})==[]
    assert s.step(3,{'io':True,'camera':True})==['SEAL_PI']
    assert s.stage=='SEAL_PI';return s


@pytest.mark.parametrize('missing',['pi_sealed','io','camera'])
def test_seal_wait_needs_both_hosts_no_nav_or_retry_before_proof(missing):
    s=to_seal();obs=dict(pi_sealed=True,io=True,camera=True);obs[missing]=False
    for t in [3.1,5.,10.,22.9]:assert s.step(t,obs)==[] and s.stage=='SEAL_PI'
    obs[missing]=True
    assert s.step(22.99,obs)==['START_NAV','START_RVIZ'] and s.stage=='POSE'
    assert s.retry==0


def test_expired_seal_stage_rejects_even_ready_response_and_does_not_camera_retry():
    s=to_seal();assert s.step(23.,dict(pi_sealed=True,io=True,camera=True))==['STOP']
    assert s.error=='STARTUP_TIMEOUT:SEAL_PI' and s.done and s.retry==0


def test_repeated_pi_seal_does_not_extend_budget_and_stop_cancels_pending():
    p,e,s,h=controller();p.command('START',1);h['io']=False
    p.command('SEAL',2);p.command('SEAL',10);assert p.seal_requested_at==2
    p.command('STOP',11);h['io']=True;p.tick(12)
    assert not p.sealed and p.phase=='STOPPING' and e.count('BEGIN_STOP')==1
    with pytest.raises(ValueError,match='PI_SEAL_WRONG_PHASE'):p.command('SEAL',13)


def test_pi_seal_pending_disallows_restart_and_times_out_before_late_health():
    p,e,s,h=controller();p.command('START',1);h['io']=False;p.command('SEAL',2)
    with pytest.raises(ValueError):p.command('RETRY_PREARM',3)
    h['io']=True;p.tick(22.)
    assert p.error=='PI_SEAL_HEALTH_TIMEOUT' and p.phase=='STOPPING' and not p.sealed
    assert e==['START_IO','BEGIN_STOP']


@pytest.mark.parametrize('state,accepted',[
    ({'phase':'IO_STARTING','seal_requested':True,'sealed':False},True),
    ({'phase':'RUNNING_OBSERVE','sealed':True},True),
    ({'phase':'IO_STARTING','sealed':False},False),
    ({'phase':'IO_STARTING','seal_requested':False,'sealed':False},False),
    ({'phase':'STOPPING','seal_requested':True,'sealed':False},False),
    ({'phase':'DONE','seal_requested':True,'sealed':True},False),
])
def test_lost_seal_ack_only_confirmed_intent_or_sealed_state_no_replay(state,accepted):
    p=peer.SshPi(B,'/peer');calls=[]
    def request(cmd):
        calls.append(cmd)
        if cmd=='SEAL':raise peer.PeerTransportError('lost ack')
        return state
    p.request=request
    if accepted:assert p.startup('SEAL',sleep=lambda *_:None)==state
    else:
        with pytest.raises(peer.PeerTransportError):p.startup('SEAL',sleep=lambda *_:None)
    assert calls==['SEAL','STATE']


@pytest.mark.parametrize('scenario',['recover','lost_ack','never_ready','local_dip','remote_dip','wrong_binding','nav_spawn_error'])
def test_laptop_entry_real_peer_and_pi_controller_wait_then_nav_or_bounded_stop(monkeypatch,tmp_path,scenario):
    now=[1.];events=[];requests=[];out=tmp_path/'run';pair=tmp_path/'pair';pair.write_text(json.dumps(B))
    mapfile=tmp_path/'map';mapfile.touch();rviz=tmp_path/'view';rviz.touch()
    monkeypatch.setattr(laptop.Path,'home',lambda:tmp_path)
    monkeypatch.setattr(laptop,'preflight',lambda *a:None);monkeypatch.setattr(laptop,'packet_digest',lambda:B['packet'])
    monkeypatch.setattr(laptop.time,'monotonic',lambda:now[0]);monkeypatch.setattr(laptop.time,'sleep',lambda dt:now.__setitem__(0,now[0]+dt))
    monkeypatch.setattr(laptop,'run_clock_gate',lambda *a:None)
    pi_health={'io':False};p=PiController(B,lambda:events.append('PI_START'),lambda:events.append('PI_STOP'),
        lambda:dict(remote_groups_exited=True,children_clean=True,forced=False),lambda:dict(pi_health),now[0])
    def run(argv,**kw):
        cmd=shlex.split(argv[-1])[-1];requests.append(cmd)
        pi_health['io']=(now[0]>=5 and scenario!='never_ready')
        if scenario=='remote_dip' and 5<=now[0]<8:pi_health['io']=False
        p.tick(now[0]);state=p.command(cmd,now[0]);p.tick(now[0]);state=p.report()
        if cmd=='SEAL' and scenario=='lost_ack':return N(returncode=255,stdout='',stderr='lost ack')
        binding=dict(B)
        if scenario=='wrong_binding' and cmd=='SEAL':binding['boot_id']='different'
        return N(returncode=0,stdout=json.dumps(dict(binding=binding,state=state)))
    monkeypatch.setattr(laptop,'SshPi',lambda b,s:peer.SshPi(b,s,run=run))
    class Local:
        def __init__(self,*a):self.processes={}
        def start(self,role,*a,**kw):
            if role=='nav':
                assert p.sealed and pi_health['io'];assert now[0]>=5
                if scenario=='local_dip':assert now[0]>=8
                if scenario=='nav_spawn_error':raise RuntimeError('fake_nav_spawn_error')
            events.append(role);self.processes[role]=object()
        def check(self,*a,**kw):return kw.get('allow_operator_exit',False)
        def drain(self,**kw):events.append('LOCAL_DRAIN');return {'clean':True}
        def close_logs(self):events.append('CLOSE')
    monkeypatch.setattr(laptop,'LocalProcesses',Local)
    def health(*a):
        assert callable(a[-1]),'Production freshness clock must be read after file'
        checks=dict(io=True,camera=True,pose=True,yolo=True,operator=True)
        if scenario=='local_dip' and 2<=now[0]<8:checks['io']=False
        return dict(checks=checks,stable_s={k:5 for k in checks})
    monkeypatch.setattr(laptop,'read_health',health)
    rc=laptop.main(['--run-observe','--pairing',str(pair),'--remote-script','/peer','--out',str(out),'--map',str(mapfile),'--rviz',str(rviz)])
    success=scenario in ('recover','lost_ack','local_dip','remote_dip')
    assert rc==(0 if success else 2)
    assert requests.count('SEAL')==1 and 'RETRY_PREARM' not in requests
    assert events.index('LOCAL_DRAIN')<events.index('PI_STOP')
    if success:
        assert events.index('nav')<events.index('rviz')<events.index('yolo')<events.index('mission')<events.index('operator')
        assert json.loads((out/'startup.json').read_text())['stage']=='READY_OBSERVE'
    else:
        assert 'nav' not in events and 'operator' not in events
        failure=json.loads((out/'failure.json').read_text());startup=json.loads((out/'startup.json').read_text())
        assert failure['startup']==startup and startup['done'] and startup['error']
    assert now[0]<30,'Wait budget cannot restart with fresh retry/ack'
