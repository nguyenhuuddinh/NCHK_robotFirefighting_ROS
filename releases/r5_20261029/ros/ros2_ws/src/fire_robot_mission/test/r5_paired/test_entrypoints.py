import json
from pathlib import Path
from types import SimpleNamespace as N
import pytest
import r5_laptop_session as laptop
import r5_pi_session as pi


def test_default_entrypoints_only_plan_never_spawn(monkeypatch,capsys):
    monkeypatch.setattr(laptop.subprocess,'Popen',lambda *a,**k:pytest.fail('default must not spawn'))
    monkeypatch.setattr(pi.subprocess,'run',lambda *a,**k:pytest.fail('default must not call clock/SSH'))
    assert pi.main([])==0;assert json.loads(capsys.readouterr().out)['mode']=='PLAN_ONLY_LIVE_FALSE'
    assert laptop.main([])==0;data=json.loads(capsys.readouterr().out)
    assert data['mode']=='PLAN_ONLY_LIVE_FALSE'
    assert 'enable_live_intents:=false' in data['commands']['operator']
    assert data['gate2_token_required']=='R5_GATE2_SUPERVISED_LIVE'


@pytest.mark.parametrize('args',[['--run-gate2'],['--run-gate2','--gate2-token','wrong'],['--run-observe','--gate2-token','R5_GATE2_SUPERVISED_LIVE']])
def test_gate2_requires_exact_explicit_token_before_any_io(monkeypatch,args):
    monkeypatch.setattr(laptop.subprocess,'Popen',lambda *a,**k:pytest.fail('invalid mode/token must not spawn'))
    with pytest.raises(SystemExit):laptop.main(args)


@pytest.mark.parametrize('failure',[None,'clock','pose','local_drain'])
def test_laptop_main_actual_order_fake_IO_remote_stop_always_after_operator_drain(monkeypatch,tmp_path,failure):
    events=[];now=[1.];root=tmp_path/'root';root.mkdir();(root/'clock').mkdir()
    (root/'clock/settle.json').write_text(json.dumps(dict(status='CLOCK_SETTLED')))
    pair=tmp_path/'pair.json';pair.write_text(json.dumps(dict(packet='expected',session='pi-session')))
    mapfile=tmp_path/'map.yaml';mapfile.touch();rviz=tmp_path/'nav.rviz';rviz.touch()
    monkeypatch.setattr(laptop,'packet_digest',lambda:'expected');monkeypatch.setattr(laptop,'preflight',lambda *a:None);monkeypatch.setattr(laptop.Path,'home',lambda:tmp_path)
    monkeypatch.setattr(laptop.time,'monotonic',lambda:now[0]);monkeypatch.setattr(laptop.time,'sleep',lambda s:now.__setitem__(0,now[0]+s))
    monkeypatch.setattr(laptop.Path,'mkdir',lambda self,*a,**k:None if self==root else original_mkdir(self,*a,**k))
    class Peer:
        def __init__(self,*a):self.sealed=False
        def request(self,key):
            events.append('PEER_'+key)
            if key=='SEAL':self.sealed=True
            return dict(phase='RUNNING_OBSERVE' if self.sealed else 'WAIT_LAPTOP' if key=='STATE' else 'IO_STARTING',sealed=self.sealed,health=dict(io=True))
        def startup(self,key):return self.request(key)
        def stop(self):events.append('REMOTE_STOP');return dict(confirmed=True)
    monkeypatch.setattr(laptop,'SshPi',Peer)
    def clock_gate(*a,**kw):
        events.append('CLOCK')
        if failure=='clock':raise RuntimeError('fake clock failure')
    monkeypatch.setattr(laptop,'run_clock_gate',clock_gate)
    class Local:
        def __init__(self,*a):self.processes={};self.registry=N(logs={})
        def start(self,role,args,**kw):events.append('START_'+role);self.processes[role]=object()
        def check(self,now,**kw):return kw.get('allow_operator_exit',False)
        def drain(self,**kw):
            events.append('OPERATOR_DRAIN_LOCAL_OWNERS_NAV')
            if failure=='local_drain':raise RuntimeError('fake drain exception')
            return dict(clean=True)
        def close_logs(self):events.append('CLOSE_LOGS')
    monkeypatch.setattr(laptop,'LocalProcesses',Local)
    def health(*a):
        checks=dict(io=True,camera=True,pose=failure!='pose',yolo=True,
                    operator=True,operator_present=True,operator_ready=False)
        return dict(checks=checks,stable_s={k:10 for k in checks})
    monkeypatch.setattr(laptop,'read_health',health)
    rc=laptop.main(['--run-observe','--pairing',str(pair),'--remote-script','/pi/peer.py','--out',str(root),'--map',str(mapfile),'--rviz',str(rviz)])
    assert rc==(0 if failure is None else 2)
    assert events[-2:]==['REMOTE_STOP','CLOSE_LOGS']
    if failure=='clock':assert not any(e.startswith('START_') for e in events)
    else:assert events.index('OPERATOR_DRAIN_LOCAL_OWNERS_NAV')<events.index('REMOTE_STOP')
    if failure is None:
        assert events.index('CLOCK')<events.index('PEER_START')<events.index('START_nav')<events.index('START_yolo')<events.index('START_mission')<events.index('START_operator')
    if failure=='pose':assert 'START_yolo' not in events and 'START_operator' not in events

original_mkdir=Path.mkdir


def test_gate2_main_uses_explicit_profile_and_truthful_live_receipt(monkeypatch,tmp_path,capsys):
    events=[];commands={};now=[1.];out=tmp_path/'out';out.mkdir();pair=tmp_path/'pair.json'
    pair.write_text(json.dumps(dict(packet='expected',session='pi-session')))
    mapfile=tmp_path/'map.yaml';mapfile.touch();rviz=tmp_path/'nav.rviz';rviz.touch()
    monkeypatch.setattr(laptop,'packet_digest',lambda:'expected')
    monkeypatch.setattr(laptop,'preflight',lambda *a:events.append('PREFLIGHT'))
    monkeypatch.setattr(laptop,'validate_gate2_profile',lambda *a:events.append('GATE2_PROFILE'))
    monkeypatch.setattr(laptop.Path,'home',lambda:tmp_path)
    monkeypatch.setattr(laptop.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(laptop.time,'sleep',lambda s:now.__setitem__(0,now[0]+s))
    monkeypatch.setattr(laptop.Path,'mkdir',lambda self,*a,**k:None if self==out else original_mkdir(self,*a,**k))
    class Peer:
        def __init__(self,*a):self.sealed=False
        def startup(self,key):
            if key=='SEAL':self.sealed=True
            return dict(phase='RUNNING_OBSERVE' if self.sealed else 'WAIT_LAPTOP' if key=='STATE' else 'IO_STARTING',
                        sealed=self.sealed,health=dict(io=True))
        def stop(self):return dict(confirmed=True)
    monkeypatch.setattr(laptop,'SshPi',Peer);monkeypatch.setattr(laptop,'run_clock_gate',lambda *a,**k:None)
    class Local:
        def __init__(self,*a):self.processes={};self.registry=N(logs={})
        def start(self,role,args,**kw):self.processes[role]=object();commands[role]=args
        def check(self,now,**kw):return kw.get('allow_operator_exit',False)
        def drain(self,**kw):return dict(clean=True)
        def close_logs(self):pass
    monkeypatch.setattr(laptop,'LocalProcesses',Local)
    def health(*a):
        checks=dict(io=True,camera=True,pose=True,yolo=True,operator=True,
                    operator_present=True,operator_ready=False)
        return dict(checks=checks,stable_s={k:10 for k in checks})
    monkeypatch.setattr(laptop,'read_health',health)
    rc=laptop.main(['--run-gate2','--gate2-token','R5_GATE2_SUPERVISED_LIVE',
        '--pairing',str(pair),'--remote-script','/pi/peer.py','--out',str(out),
        '--map',str(mapfile),'--rviz',str(rviz)])
    assert rc==0 and events==['PREFLIGHT','GATE2_PROFILE']
    assert 'operational_stack_gate2.launch.py' in commands['mission']
    assert 'enable_live_intents:=true' in commands['operator']
    startup=json.loads((out/'startup.json').read_text());receipt=json.loads((out/'receipt.json').read_text())
    assert startup['stage']=='READY_GATE2' and startup['live'] is True
    assert receipt['profile']=='R5_GATE2_SUPERVISED' and receipt['live'] is True
    assert 'R5_GATE2_READY' in capsys.readouterr().out


@pytest.mark.parametrize('role',['pi','laptop'])
def test_entrypoint_preflight_failure_has_saved_raw_reason_no_NTP_SSH_ROS(monkeypatch,tmp_path,role):
    module=pi if role=='pi' else laptop;out=tmp_path/'out'
    monkeypatch.setattr(module,'packet_digest',lambda:(_ for _ in ()).throw(ValueError('pinned source mismatch')))
    monkeypatch.setattr(module.subprocess,'Popen',lambda *a,**k:pytest.fail('must not spawn'))
    monkeypatch.setattr(module.subprocess,'run',lambda *a,**k:pytest.fail('must not SSH/NTP'))
    args=['--run-observe','--out',str(out)]
    if role=='laptop':
        m=tmp_path/'map';m.touch();v=tmp_path/'view';v.touch()
        args+=['--pairing',str(tmp_path/'pair'),'--remote-script','/pi/helper','--map',str(m),'--rviz',str(v)]
    assert module.main(args)==2
    failure=json.loads((out/'failure.json').read_text());assert failure['hardware_started'] is False
    assert 'pinned source mismatch' in failure['error']
