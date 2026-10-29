import json
from types import SimpleNamespace as N
from pathlib import Path
import pytest
import r5_pi_session as pi


def test_actual_pi_main_fake_io_keeps_NTP_before_ROS_and_records_remote_group_receipt(monkeypatch,tmp_path):
    events=[];now=[1.];out=tmp_path/'pi_run';gen=[0];bound={}
    monkeypatch.setattr(pi,'packet_digest',lambda:'manifest');monkeypatch.setattr(pi,'preflight',lambda *a:None)
    monkeypatch.setattr(pi.Path,'home',lambda:tmp_path)
    monkeypatch.setattr(pi,'binding',lambda root,packet:dict(session='session',packet=packet,root=str(root)))
    monkeypatch.setattr(pi.time,'monotonic',lambda:now[0]);monkeypatch.setattr(pi.time,'sleep',lambda s:now.__setitem__(0,now[0]+s))
    monkeypatch.setattr(pi.subprocess,'run',lambda *a,**kw:events.append('NTP_REFRESH') or N(returncode=0))
    class Mail:
        def __init__(self,*a):self.i=0
        def requests(self):
            self.i+=1
            if self.i==1:yield Path('fake_request'),dict(command='START')
            if self.i==2:yield Path('fake_request2'),dict(command='STOP')
        def reply(self,*a):pass
    monkeypatch.setattr(pi,'Mailbox',Mail)
    class Registry:
        def audit(self,*a):return dict(clean=True)
    class Proc:
        def report(self):return dict(group_exit_confirmed=True,forced=False,leader_rc=0,error=None)
    class Local:
        def __init__(self,folder,session):self.out=folder;self.processes={};self.registry=Registry();self.stack=None
        def start(self,role,argv,**kw):events.append('START_'+role);self.processes[role]=Proc()
        def check(self,at):assert at>=now[0]
        def begin_stop(self,*a):
            events.append('BEGIN_STOP');self.stack=N(done=False,tick=lambda n:setattr(self.stack,'done',True))
        def close_logs(self):events.append('CLOSE_LOGS')
    monkeypatch.setattr(pi,'LocalProcesses',Local)
    monkeypatch.setattr(pi,'read_health',lambda *a:dict(checks=dict(io=True),sample_mono=now[0]))
    assert pi.main(['--run-observe','--out',str(out)])==0
    assert events[:3]==['NTP_REFRESH','START_pi','START_startup_health']
    done=json.loads((out/'DONE.json').read_text());assert done['cleanup']['remote_groups_exited'] and done['cleanup']['children_clean']
    assert events[-2:]==['BEGIN_STOP','CLOSE_LOGS']
