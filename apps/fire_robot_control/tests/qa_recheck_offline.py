"""Supplemental app QA: local files, fake transport/processes, no Pi or devices.

Assertions express the required behaviour. Failures are retained as findings;
this QA suite must not rewrite or patch production sources to make them pass.
"""
import errno
import json
from pathlib import Path
from types import SimpleNamespace as Obj
from unittest.mock import patch
import fcntl
import os
import signal
import subprocess
import threading
import uuid

import pytest

from services.app_peer import AppPeer
from services.app_pi_lifecycle import AppPiController
from services.live_session import LiveSession
from services import map_save
from services.map_catalog import catalog
from services.supervisor import SessionController
from proof_fixture import clean_remote


def controller_peer():
    now = [0.]
    commands = []
    bind = dict(session=str(uuid.uuid4()), secret='1'*32, packet='a'*64,
                boot_id=str(uuid.uuid4()), runner_pid=123, runner_start_ticks=123, root='/fake/owned')
    cleanup = clean_remote({})['receipt']
    controller = AppPiController(bind, lambda: None, lambda: commands.append('IO_STOP'),
                                 lambda: cleanup, lambda: dict(io=False), 0.)
    def run(argv, **kwargs):
        # AppPeer builds an SSH argv; the injectable transport executes nothing.
        import shlex
        command = shlex.split(argv[-1])[-1]
        commands.append(command)
        controller.tick(now[0])
        state = controller.command(command, now[0])
        controller.tick(now[0])
        return Obj(returncode=0, stdout=json.dumps(dict(binding=bind, state=controller.report())), stderr='')
    peer = AppPeer(bind, '/unused', run=run)
    return peer, controller, now, commands


@pytest.mark.parametrize('seal_timeout', [False, True])
def test_peer_sends_stop_while_app_pi_is_waiting_for_drain(seal_timeout):
    peer, controller, now, commands = controller_peer()
    peer.startup('START')
    if seal_timeout:
        now[0] = 1.
        peer.startup('SEAL')
        now[0] = 21.
        controller.tick(now[0])
        assert controller.phase == 'WAIT_LAPTOP_DRAIN'
    result = peer.stop(clock=lambda: now[0], sleep=lambda dt: now.__setitem__(0, now[0]+dt))
    assert 'STOP' in commands, {'commands': commands, 'phase': controller.phase, 'result': result}
    assert controller.phase == 'DONE'
    assert result['cleanup_confirmed'] is True


def generate_map(prefix):
    prefix.with_suffix('.pgm').write_bytes(b'P5\n2 2\n255\n\x00\xff\xff\x00')
    prefix.with_suffix('.yaml').write_text(f'image: {prefix.name}.pgm\nresolution: 0.05\norigin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


@pytest.mark.parametrize('fail_on', ['generate', 'link_image', 'link_yaml', 'final_proof'])
def test_map_enospc_and_stop_roll_back_only_owned_pair(tmp_path, monkeypatch, fail_on):
    sentinel = tmp_path/'old-map.txt'; sentinel.write_text('keep')
    checks = []
    real_link = os.link
    def owned():
        checks.append(True)
        if fail_on == 'final_proof' and len(checks) == 5:
            raise map_save.MapSaveError('STOP before commit result')
    def generate(prefix):
        if fail_on == 'generate': raise OSError(errno.ENOSPC, 'fake disk full')
        generate_map(prefix)
    def link(source, target, **kwargs):
        if (fail_on=='link_image' and str(target).endswith('.pgm')) or (fail_on=='link_yaml' and str(target).endswith('.yaml')):
            raise OSError(errno.ENOSPC, 'fake disk full')
        return real_link(source, target, **kwargs)
    monkeypatch.setattr(map_save.os, 'link', link)
    with pytest.raises((map_save.MapSaveError, OSError)):
        map_save.save_map_pair(tmp_path, 'new', assert_owned_mapping=owned, generate=generate)
    assert not (tmp_path/'new.yaml').exists() and not (tmp_path/'new.pgm').exists()
    assert not list(tmp_path.glob('.fire_robot_save_*'))
    assert sentinel.read_text() == 'keep'


@pytest.mark.parametrize('failure', ['timeout','disk_full','nonzero'])
def test_map_saver_error_is_bounded_and_does_not_create_map(tmp_path, monkeypatch, failure):
    calls=[]
    def run(argv, **kwargs):
        calls.append((argv,kwargs))
        if failure=='timeout':raise subprocess.TimeoutExpired(argv,25)
        if failure=='disk_full':raise OSError(errno.ENOSPC,'fake disk full')
        return Obj(returncode=1,stderr='map publisher missing')
    monkeypatch.setattr(map_save.subprocess,'run',run)
    with pytest.raises(map_save.MapSaveError):
        map_save.save_map_pair(tmp_path,'new',assert_owned_mapping=lambda:None,generate=map_save.ros_map_saver)
    assert calls[0][1]['timeout']==25
    assert not (tmp_path/'new.yaml').exists() and not (tmp_path/'new.pgm').exists()


@pytest.mark.parametrize('stage', ['PREFLIGHT','STARTING_PI','READING_PAIRING','STARTING_FIRE','PI_IO','CAMERA','SEAL_PI','POSE','YOLO','CONSUMERS','READY_GATE2','STOPPING'])
def test_controller_stop_and_late_status_never_reopen_session(tmp_path,stage):
    for name in ('runtime','maps','state'):(tmp_path/name).mkdir()
    workers=[]
    class Fake:
        def __init__(self,**kw):
            self.session=str(uuid.uuid4());self.kind=kw['kind'];self.out=kw['out'];self.commands=[];self.emit=kw['emit']
        def start(self):workers.append(self)
        def send(self,event,*args):self.commands.append(event)
        def heartbeat(self,_):pass
    c=SessionController(candidate=tmp_path,packet='a'*64,state_dir=tmp_path/'state',runtime=tmp_path/'runtime',
                        map_root=tmp_path/'maps',map_roots=[tmp_path/'maps'],session_factory=Fake)
    c.action('SCAN');worker=c.session
    c.inbox.put(dict(event='SESSION',session=worker.session,stage=stage));c.tick()
    c.action('STOP')
    for action in ('SCAN','FIRE','SAVE','VERIFY_STOP'):c.action(action,'map')
    c.inbox.put(dict(event='SESSION',session=worker.session,stage='READY_GATE2'));c.tick()
    assert c.phase=='STOPPING' and c.session is worker and len(workers)==1
    assert worker.commands==['STOP']
    assert (tmp_path/'state/pending.json').exists()


@pytest.mark.parametrize('error_point', ['pending_write','pending_unlink'])
def test_enospc_does_not_leave_a_dead_controller_worker(tmp_path,monkeypatch,error_point):
    from services import supervisor
    for name in ('runtime','maps','state'):(tmp_path/name).mkdir()
    class Fake:
        def __init__(self,**kw):self.session=str(uuid.uuid4());self.kind=kw['kind'];self.out=kw['out']
        def start(self):pass
        def send(self,*_):pass
    c=SessionController(candidate=tmp_path,packet='a'*64,state_dir=tmp_path/'state',runtime=tmp_path/'runtime',
                        map_root=tmp_path/'maps',map_roots=[tmp_path/'maps'],session_factory=Fake)
    def full(*args,**kwargs):raise OSError(errno.ENOSPC,'fake disk full')
    if error_point=='pending_write':
        monkeypatch.setattr(supervisor,'atomic',full);c.action('SCAN')
    else:
        c.action('SCAN');sid=c.session.session
        real=Path.unlink
        def unlink(p,*args,**kwargs):
            if p.name=='pending.json':return full()
            return real(p,*args,**kwargs)
        monkeypatch.setattr(Path,'unlink',unlink)
        c.inbox.put(dict(event='FINISHED',session=sid,robot_stop_confirmed=True,cleanup_clean=True,session_was_started=True));c.tick()
    assert c.session is None and c.phase=='BLOCKED'
    c.action('CLOSE');assert c.status()['close_ready']


@pytest.mark.parametrize('kind',['fire','mapping'])
def test_worker_stop_dominates_a_full_action_queue(tmp_path,kind):
    for name in ('runtime','out'):(tmp_path/name).mkdir()
    worker=LiveSession(candidate=tmp_path,packet='a'*64,runtime=tmp_path/'runtime',out=tmp_path/'out',
                       map_root=tmp_path,map_roots=[tmp_path],kind=kind,selected_map=None,emit=lambda _:None)
    writes=[];worker.auth_child=Obj(write=lambda b:writes.append(b))
    for _ in range(64):worker.send('AUTH','private-test-value')
    worker.send('STOP');worker._pump()
    assert worker.stop_event.is_set() and not writes and worker.commands.empty()


@pytest.mark.parametrize('moved',['tf_flap','camera_flap','seal_missing'])
def test_startup_does_not_admit_on_late_health_recovery(moved):
    from operational_entry import StartupSequence
    s=StartupSequence(0.,live=True)
    s.step(0.,dict(clock_ok=True));s.step(1.,dict(io=True));s.step(2.,dict(io=True,camera=True))
    for at in (3.,10.,21.):
        effects=s.step(at,dict(io=moved!='tf_flap',camera=moved!='camera_flap',pi_sealed=False))
        assert not any(e in effects for e in ('START_NAV','START_RVIZ','START_YOLO','START_CONSUMERS'))
    effects=s.step(22.,dict(io=True,camera=True,pi_sealed=True))
    assert effects==['STOP'] and s.error=='STARTUP_TIMEOUT:SEAL_PI'


def make_fake_worker(base, kind='fire'):
    from test_live_session_fake import FakeLiveSession
    for name in ('candidate','runtime','out','maps'):(base/name).mkdir(mode=0o700,parents=True)
    (base/'candidate/desktop').mkdir()
    events=[]
    worker=FakeLiveSession(candidate=base/'candidate',packet='a'*64,runtime=base/'runtime',out=base/'out',
             map_root=base/'maps',map_roots=[base/'maps'],kind=kind,selected_map=None,emit=events.append,
             pi_receipt=base/'fake-receipt')
    if kind=='mapping':
        original_spawn=worker._spawn_laptop
        def spawn_mapping(remote,pairing):
            original_spawn(remote,pairing)
            from services.live_session import atomic
            atomic(worker.out/'laptop/mapping.json',dict(session=worker.session,stage='MAPPING'))
            original_stop=worker.local.stop
            def stop_mapping(now):
                was_done=worker.local.done
                original_stop(now)
                if not was_done:
                    receipt_path=worker.out/'laptop/receipt.json'
                    receipt=json.loads(receipt_path.read_text())
                    receipt['profile']='DESKTOP_MAPPING'
                    atomic(receipt_path,receipt)
            worker.local.stop=stop_mapping
        worker._spawn_laptop=spawn_mapping
    return worker,events


@pytest.mark.parametrize('point',['before_pi','auth_wait','pairing_read','running'])
def test_transport_loss_does_not_restart_or_claim_stop_without_proof(tmp_path,point):
    import time
    from test_live_session_fake import FakeTransfer, FakeChild, FakeOwnedRunner
    worker,events=make_fake_worker(tmp_path)
    result=Obj(returncode=1 if point=='before_pi' else 0,stderr='fake network unavailable')
    class Transfer(FakeTransfer):
        def finish(self):
            if point=='pairing_read':raise ConnectionError('fake pairing transport lost')
            return super().finish()
    class Runner(FakeOwnedRunner):
        def __init__(self,owner):
            super().__init__(owner)
            if point=='running':owner.pi.code=255
        def stop(self,now):
            if self.done:return
            # Remote I/O proof was lost along with transport. Never synthesize it.
            self.owner.order.append('laptop-drain')
            from services.live_session import atomic
            from proof_fixture import clean_local
            atomic(self.owner.out/'laptop/receipt.json',dict(session=self.owner.session,packet=self.owner.packet,
                rc=2,profile='R5_AUTOMATIC',local={**clean_local(),'stop_confirmed':False,'clean':False},remote=dict(confirmed=False,
                binding={k:v for k,v in self.owner.remote_binding.items() if k!='secret'},receipt=None)))
            self.done,self.rc=True,2
    original_launch=worker._launch_pi
    def launch():
        path=original_launch()
        if point=='auth_wait':worker.pi.code=255
        return path
    worker._launch_pi=launch
    def spawn(remote,pairing):
        worker.runner_log=open(os.devnull,'wb');worker.local=Runner(worker)
    worker._spawn_laptop=spawn
    with patch('services.live_session.PairingTransfer',Transfer),patch('services.live_session.subprocess.run',return_value=result), \
         patch('services.live_session.validate_pi_receipt',return_value={}):
        worker.start();until=time.monotonic()+3
        while worker.thread.is_alive() and time.monotonic()<until:
            worker.heartbeat(time.monotonic());worker.thread.join(.01)
        assert not worker.thread.is_alive(),'fake transport cleanup did not finish'
    final=[e for e in events if e.get('event')=='FINISHED'][-1]
    assert final['robot_stop_confirmed'] is not True
    if point=='before_pi':
        assert worker.pi is None and not final['session_was_started']
    else:assert final['cleanup_clean'] is False
    assert worker.local is None or worker.local.done
    assert sum(e.get('stage')=='STARTING_PI' for e in events)<=1


@pytest.mark.parametrize('kind',['fire','mapping'])
def test_ten_start_stop_cycles_leave_no_fd_or_worker_growth(tmp_path,kind):
    import time
    from test_live_session_fake import FakeTransfer
    def cycle(index):
        base=tmp_path/str(index);base.mkdir()
        worker,events=make_fake_worker(base,kind)
        result=Obj(returncode=0,stderr='')
        with patch('services.live_session.PairingTransfer',FakeTransfer),patch('services.live_session.subprocess.run',return_value=result), \
             patch('services.live_session.validate_pi_receipt',return_value={}):
            worker.start();until=time.monotonic()+3
            while worker.stage!=('MAPPING' if kind=='mapping' else 'READY_GATE2') and worker.thread.is_alive() and time.monotonic()<until:
                worker.heartbeat(time.monotonic());worker.thread.join(.005)
            assert worker.stage==('MAPPING' if kind=='mapping' else 'READY_GATE2')
            worker.send('STOP');worker.send('STOP')
            while worker.thread.is_alive() and time.monotonic()<until:
                worker.heartbeat(time.monotonic());worker.thread.join(.005)
            assert not worker.thread.is_alive()
        final=[e for e in events if e.get('event')=='FINISHED'][-1]
        assert final['robot_stop_confirmed'] is True and final['cleanup_clean'] is True
        assert worker.order==['laptop-drain','pi-stop']
    cycle('warmup')
    before_fd=len(list(Path('/proc/self/fd').iterdir()))
    before_threads={t.ident for t in threading.enumerate()}
    for i in range(10):cycle(i)
    assert len(list(Path('/proc/self/fd').iterdir()))==before_fd
    assert {t.ident for t in threading.enumerate()}==before_threads


@pytest.mark.parametrize('kind',['fire','mapping'])
def test_desktop_receipt_disk_full_finishes_and_keeps_cleanup_unconfirmed(tmp_path,kind):
    import time
    from test_live_session_fake import FakeTransfer
    from services.live_session import atomic as real_atomic
    worker,events=make_fake_worker(tmp_path,kind)
    def full(path,row):
        if path.name=='desktop_receipt.json':raise OSError(errno.ENOSPC,'fake full disk')
        return real_atomic(path,row)
    with patch('services.live_session.PairingTransfer',FakeTransfer),patch('services.live_session.subprocess.run',return_value=Obj(returncode=0,stderr='')), \
         patch('services.live_session.validate_pi_receipt',return_value={}),patch('services.live_session.atomic',full):
        worker.start();until=time.monotonic()+3
        while worker.stage!=('MAPPING' if kind=='mapping' else 'READY_GATE2') and worker.thread.is_alive() and time.monotonic()<until:
            worker.heartbeat(time.monotonic());worker.thread.join(.005)
        worker.send('STOP')
        while worker.thread.is_alive() and time.monotonic()<until:
            worker.heartbeat(time.monotonic());worker.thread.join(.005)
        assert not worker.thread.is_alive()
    final=[e for e in events if e.get('event')=='FINISHED'][-1]
    assert final['cleanup_clean'] is False
    assert worker.local.done and worker.pi.poll()==0
    assert not (worker.out/'desktop_receipt.json').exists()


@pytest.mark.parametrize('sample,now', [(9.2,10.),(10.1,10.),(10.,9.),(float('nan'),10.),(float('inf'),10.),(10.,float('nan'))])
def test_health_snapshot_stale_future_nonfinite_or_clock_reversal_never_admits(tmp_path,sample,now):
    from operational_entry import read_health
    row=dict(schema_version=1,session='current',commands_published=False,sample_mono=sample,checks=dict(io=True))
    p=tmp_path/'health.json';p.write_text(json.dumps(row))
    with pytest.raises(ValueError):read_health(p,'current',lambda:now)


def test_health_file_replaced_after_earlier_clock_uses_post_read_clock(tmp_path):
    from operational_entry import read_health
    p=tmp_path/'health.json';p.write_text(json.dumps(dict(schema_version=1,session='current',commands_published=False,sample_mono=10.01)))
    assert read_health(p,'current',lambda:10.02)['sample_mono']==10.01


@pytest.mark.parametrize('mode',['write_full','stop_during_sample'])
def test_sampled_health_write_failure_or_stop_restores_signals_and_never_creates_ready(tmp_path,monkeypatch,mode):
    import sys
    import types
    from services import app_health_entry
    events=[]
    class Node:
        def __init__(self,*_):self.cb=None
        def create_timer(self,seconds,cb):self.cb=cb
        def sample(self):
            if mode=='stop_during_sample':signal.getsignal(signal.SIGTERM)(None,None)
            return dict(checks={'io':False},commands_published=False)
        def destroy_node(self):events.append('destroy')
    package=types.ModuleType('fire_robot_mission');package.startup_health=Obj(StartupHealthNode=Node)
    fake=types.ModuleType('rclpy');fake.init=lambda **_:None;fake.ok=lambda:True
    fake.spin_once=lambda node,**_:node.cb();fake.shutdown=lambda:events.append('shutdown')
    sigmodule=types.ModuleType('rclpy.signals');sigmodule.SignalHandlerOptions=Obj(NO=0)
    monkeypatch.setitem(sys.modules,'fire_robot_mission',package)
    monkeypatch.setitem(sys.modules,'rclpy',fake);monkeypatch.setitem(sys.modules,'rclpy.signals',sigmodule)
    handlers={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    out=tmp_path/'health.json'
    real_write=Path.write_text
    def write(p,*args,**kwargs):
        if mode=='write_full' and p.name=='health.tmp':raise OSError(errno.ENOSPC,'fake disk full')
        return real_write(p,*args,**kwargs)
    monkeypatch.setattr(Path,'write_text',write)
    args=['--out',str(out),'--session','s','--role','pi','--mode','sampled']
    if mode=='write_full':
        with pytest.raises(OSError):app_health_entry.main(args)
        assert not out.exists()
    else:
        app_health_entry.main(args)
        assert json.loads(out.read_text())['checks']['io'] is False
    assert events==['destroy','shutdown']
    assert all(signal.getsignal(s) is h for s,h in handlers.items())
