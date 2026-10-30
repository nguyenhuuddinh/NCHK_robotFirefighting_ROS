"""Execute the generated Pi main with fake processes/mailbox/clock only."""
import importlib.util
from pathlib import Path
import sys
import signal
import json
import pytest
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
from services.candidate_build import build
from proof_fixture import group,children
APP=Path(__file__).resolve().parents[1];ROOT=APP.parents[1]

@pytest.mark.parametrize('stop_mode',['mailbox','SIGHUP'])
def test_generated_pi_prearm_drain_uses_fake_io_until_laptop_stop(stop_mode):
    with tempfile.TemporaryDirectory() as tmp:
        base=Path(tmp);candidate=base/'candidate';build(ROOT/'releases/r5_20261029/ros',APP,candidate)
        sys.path.insert(0,str(candidate/'scripts'))
        try:
            spec=importlib.util.spec_from_file_location('qa_app_pi_entry',candidate/'desktop/generated/app_pi_entry.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            clock=[0.];calls=[];states=[]
            class Local:
                def __init__(self,out,session):
                    self.out=out;self.processes={};self.stack=SimpleNamespace(done=True,tick=lambda _:None)
                    self.registry=SimpleNamespace(audit=lambda _:children())
                def start(self,role,argv,**kwargs):
                    calls.append(('start',role,clock[0]));self.processes[role]=SimpleNamespace(report=lambda:group(90002 if role=='pi' else 90003))
                def begin_stop(self,*_):calls.append(('stop',clock[0]))
                def close_logs(self):pass
                def check(self,_):pass
            class Mail:
                def __init__(self,*_):self.started=self.sealed=self.stopped=False
                def requests(self):
                    cmd='STATE'
                    if not self.started:self.started=True;cmd='START'
                    elif not self.sealed:self.sealed=True;cmd='SEAL'
                    elif clock[0]>=26 and not self.stopped:
                        self.stopped=True
                        if stop_mode=='SIGHUP':signal.getsignal(signal.SIGHUP)(signal.SIGHUP,None)
                        else:cmd='STOP'
                    return [(base/'request',{'command':cmd})]
                def reply(self,path,request,state):states.append((clock[0],state['phase']))
            with patch.object(Path,'home',return_value=base),patch.object(module,'preflight'), \
                 patch.object(module,'packet_digest',return_value='a'*64), \
                 patch.object(module,'binding',return_value={'session':'fixture','packet':'a'*64}), \
                 patch.object(module,'Mailbox',Mail),patch.object(module,'LocalProcesses',Local), \
                 patch.object(module,'read_health',return_value={'checks':{'io':False},'sample_mono':0.}), \
                 patch.object(module.subprocess,'run',return_value=SimpleNamespace(returncode=0)), \
                 patch.object(module.time,'monotonic',side_effect=lambda:clock[0]), \
                 patch.object(module.time,'sleep',side_effect=lambda _:clock.__setitem__(0,clock[0]+1.)):
                rc=module.main(['--run-observe','--out',str(base/'session')])
            assert rc==0
            final=json.loads((base/'session/DONE.json').read_text())
            assert final['app_stop_signal']==('SIGHUP' if stop_mode=='SIGHUP' else None)
            assert any(phase=='WAIT_LAPTOP_DRAIN' for _,phase in states)
            stops=[x[1] for x in calls if x[0]=='stop']
            assert stops and stops[0]>=26,stops
            assert len([x for x in calls if x[:2]==('start','pi')])==1
        finally:sys.path.remove(str(candidate/'scripts'))
