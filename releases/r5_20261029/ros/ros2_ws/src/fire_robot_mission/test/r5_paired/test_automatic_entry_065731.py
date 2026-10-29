"""Automatic CLI is opt-in, bound to startup; never starts a subprocess here."""
import json
from pathlib import Path
from types import SimpleNamespace as N
import pytest
import r5_laptop_session as laptop
from fire_robot_mission.automatic_policy import AutomaticStartGate


@pytest.mark.parametrize('args',[
    ['--run-automatic'],['--run-automatic','--automatic-token','wrong'],
    ['--run-observe','--automatic-token',laptop.AUTOMATIC_TOKEN],
    ['--run-gate2','--automatic-token',laptop.AUTOMATIC_TOKEN],
    ['--run-automatic','--automatic-token',laptop.AUTOMATIC_TOKEN,'--gate2-token',laptop.GATE2_TOKEN]])
def test_automatic_token_rejected_before_any_IO(monkeypatch,args):
    monkeypatch.setattr(laptop.subprocess,'Popen',lambda *a,**k:pytest.fail('no process before valid profile'))
    with pytest.raises(SystemExit):laptop.main(args)


@pytest.mark.parametrize('failure',[None,'clock','pose'])
def test_automatic_entry_receipt_written_only_after_ready_and_matches_operator_binding(monkeypatch,tmp_path,failure):
    now=[1.];events=[];out=tmp_path/'out';pair=tmp_path/'pair.json';commands={}
    packet='ab'*32;pair.write_text(json.dumps(dict(packet=packet,session='pi-session')))
    mapfile=tmp_path/'map.yaml';mapfile.touch();rviz=tmp_path/'nav.rviz';rviz.touch()
    monkeypatch.setattr(laptop,'packet_digest',lambda:packet)
    monkeypatch.setattr(laptop,'preflight',lambda *a:None)
    monkeypatch.setattr(laptop,'validate_gate2_profile',lambda *a:None)
    monkeypatch.setattr(laptop.Path,'home',lambda:tmp_path)
    monkeypatch.setattr(laptop.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(laptop.time,'sleep',lambda s:now.__setitem__(0,now[0]+s))
    class Peer:
        def __init__(self,*a):self.sealed=False
        def startup(self,key):
            if key=='SEAL':self.sealed=True
            return dict(phase='RUNNING_OBSERVE' if self.sealed else 'WAIT_LAPTOP' if key=='STATE' else 'IO_STARTING',sealed=self.sealed,health=dict(io=True))
        def stop(self):events.append('REMOTE_STOP');return dict(confirmed=True)
    monkeypatch.setattr(laptop,'SshPi',Peer)
    def clock(*a):
        if failure=='clock':raise RuntimeError('fake clock')
    monkeypatch.setattr(laptop,'run_clock_gate',clock)
    class Local:
        def __init__(self,*a):self.processes={};self.registry=N(logs={})
        def start(self,role,args,**kw):
            assert not (out/'automatic_start.json').exists()
            commands[role]=args;self.processes[role]=object()
        def check(self,*a,**kw):return kw.get('allow_operator_exit',False)
        def drain(self,**kw):events.append('LOCAL_STOP');return dict(clean=True)
        def close_logs(self):pass
    monkeypatch.setattr(laptop,'LocalProcesses',Local)
    def health(*a):
        d=dict(io=True,camera=True,pose=failure!='pose',yolo=True,operator_present=True)
        return dict(checks=d,stable_s={k:10 for k in d})
    monkeypatch.setattr(laptop,'read_health',health)
    rc=laptop.main(['--run-automatic','--automatic-token',laptop.AUTOMATIC_TOKEN,
        '--pairing',str(pair),'--remote-script','/pi/peer.py','--out',str(out),
        '--map',str(mapfile),'--rviz',str(rviz)])
    assert rc==(0 if failure is None else 2)
    assert events==(['REMOTE_STOP'] if failure=='clock' else ['LOCAL_STOP','REMOTE_STOP'])
    start=out/'automatic_start.json'
    if failure:assert not start.exists()
    else:
        d=json.loads(start.read_text());args=commands['operator']
        assert 'automatic_mode:=true' in args
        assert 'automatic_runner_session:='+d['session'] in args
        assert 'automatic_packet:='+packet in args
        assert AutomaticStartGate(start,d['session'],packet).ready()
        assert json.loads((out/'receipt.json').read_text())['profile']=='R5_AUTOMATIC'


def test_automatic_is_off_in_default_observe_and_supervised_commands():
    for gate2 in (False,True):
        d=laptop.commands(Path('/map'),Path('/rviz'),gate2=gate2)
        assert 'automatic_mode:=true' not in d['operator']
