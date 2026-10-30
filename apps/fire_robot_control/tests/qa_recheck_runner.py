"""Execute generated fire entry with actual sequence, fake children, no ROS/SSH."""
import importlib.util
import json
from pathlib import Path
import signal
import sys
from types import SimpleNamespace as Obj
from unittest.mock import patch
import uuid

import pytest
from services.candidate_build import build
from services import desktop_hooks
from proof_fixture import clean_local, clean_remote

APP=Path(__file__).resolve().parents[1];ROOT=APP.parents[1]


@pytest.mark.parametrize('role',['nav','yolo','mission','operator'])
@pytest.mark.parametrize('fault',['spawn','exit'])
def test_child_failure_keeps_primary_error_and_drains_before_pi(tmp_path,role,fault):
    candidate=tmp_path/'candidate';build(ROOT/'releases/r5_20261029/ros',APP,candidate)
    spec=importlib.util.spec_from_file_location('qa_recheck_generated_fire',candidate/'desktop/generated/app_fire_entry.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    packet='a'*64;sid=str(uuid.uuid4());records=[];now=[1.]
    pair=tmp_path/'pairing.json';pair.write_text(json.dumps(dict(packet=packet)))
    mapfile=tmp_path/'map with spaces.yaml';mapfile.touch()
    view=tmp_path/'view.rviz';view.touch()
    class Local:
        def __init__(self,*args,**kwargs):self.processes={};self.history=[];self.real_roles=set()
        def start(self,name,argv,**kwargs):
            records.append(('spawn',name))
            if name==role and not str(argv[0]).startswith('monitor') and fault=='spawn':raise RuntimeError('QA_CHILD_SPAWN:'+role)
            self.processes[name]=Obj(report=lambda:dict(group_exit_confirmed=True))
            if not str(argv[0]).startswith('monitor'):self.real_roles.add(name)
        def check(self,*args,**kwargs):
            if role in self.real_roles and fault=='exit':raise RuntimeError('QA_CHILD_EXIT:'+role)
            return kwargs.get('allow_operator_exit',False)
        def drain(self,**kwargs):records.append(('drain',));return clean_local()
        def close_logs(self):pass
    class Peer:
        def __init__(self,*args):self.bind={}
        def startup(self,command):
            records.append(('peer',command))
            if command=='STATE':return dict(phase='WAIT_LAPTOP')
            return dict(phase='RUNNING_OBSERVE',sealed=True,health=dict(io=True))
        def stop(self):records.append(('pi_stop',));return clean_remote({})
    class Desktop:
        def __init__(self,*args):self.binding=Obj(packet=packet,session=sid)
        def backend(self,*_):return None
        def commands(self,original):return {**original,'rviz':['inert-view'],'operator':['inert-operator']}
        def prepare_start_monitor(self,local,stop):local.start('operator',['monitor-only'])
        def handoff_monitor(self,local):local.processes.pop('operator');records.append(('monitor_reaped',))
        def wait_view(self,*_):records.append(('view_ready',))
        def prepare_stop_monitor(self,*args,**kwargs):pass
        def mark_stopping(self):pass
    class Recovery:
        excluded=()
        def __init__(self,*_):self.core=Obj(report=lambda:{})
        def step(self,*args,**kwargs):pass
    identity=['fixture',1,'firmware']
    health=dict(checks={key:True for key in ('io','camera','pose','yolo','operator_present','localization')},
        stable_s=dict(io=5.,camera=5.,yolo=5.,operator_present=5.,localization=5.),identity_key=identity,localization_identity_key=identity)
    with patch.object(desktop_hooks,'DesktopHooks',Desktop),patch.object(Path,'home',return_value=tmp_path), \
         patch.object(m,'packet_digest',return_value=packet),patch.object(m,'preflight'),patch.object(m,'validate_gate2_profile'), \
         patch.object(m,'SshPi',Peer),patch.object(m,'LocalProcesses',Local),patch.object(m,'run_clock_gate'), \
         patch.object(m,'read_health',return_value=health),patch.object(m.time,'monotonic',side_effect=lambda:now[0]), \
         patch.object(m.time,'sleep',side_effect=lambda dt:now.__setitem__(0,now[0]+max(.01,dt))), \
         patch.dict(sys.modules,{'runtime_recovery_runner':Obj(RuntimeRecoveryRunner=Recovery)}):
        rc=m.main(['--desktop-fd','9','--run-automatic','--automatic-token',m.AUTOMATIC_TOKEN,
            '--pairing',str(pair),'--remote-script','/fake','--out',str(tmp_path/'out'),'--map',str(mapfile),'--rviz',str(view)])
    failure=json.loads((tmp_path/'out/failure.json').read_text())
    assert rc==2
    assert ('QA_CHILD_SPAWN:' if fault=='spawn' else 'QA_CHILD_EXIT:')+role in failure['error'],(failure,records)
    assert records.index(('drain',))<records.index(('pi_stop',))
    receipt=json.loads((tmp_path/'out/receipt.json').read_text())
    assert receipt['session']==sid and receipt['local']['stop_confirmed'] is True
