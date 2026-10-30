"""Race/idempotency probes with synthetic workers and deterministic seeds."""
import json
import random
from pathlib import Path
import tempfile
import unittest
import uuid
from services.supervisor import SessionController
from services.display_guard import wait_view
from services.desktop_hooks import wait_monitor_ready

class ActionsTests(unittest.TestCase):
    def test_repeated_random_actions_never_overlap_or_reopen_stopped_worker(self):
        for seed in (7,19,29):
            with self.subTest(seed=seed),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp)
                for name in ('maps','runtime','state'):(root/name).mkdir()
                live=[]
                class Fake:
                    def __init__(obj,**kw):
                        obj.session=str(uuid.uuid4());obj.emit=kw['emit'];obj.kind=kw['kind'];obj.out=kw['out'];obj.stop=False
                    def start(obj):live.append(obj);self.assertEqual(len(live),1)
                    def send(obj,event,*_):
                        if event=='STOP':obj.stop=True
                    def heartbeat(obj,_):pass
                c=SessionController(candidate=root,packet='a'*64,state_dir=root/'state',runtime=root/'runtime',
                    map_root=root/'maps',map_roots=[root/'maps'],session_factory=Fake)
                rng=random.Random(seed)
                for _ in range(200):
                    action=rng.choice(('SCAN','FIRE','STOP','SAVE','SELECT','LATE','FINISH'))
                    before=c.session
                    if action=='LATE':
                        c.inbox.put(dict(event='SESSION',session=str(uuid.uuid4()),stage='READY_GATE2'));c.tick()
                        self.assertIs(c.session,before)
                    elif action=='FINISH' and before:
                        live.remove(before)
                        before.emit(dict(event='FINISHED',session=before.session,robot_stop_confirmed=True,
                            cleanup_clean=True,session_was_started=True))
                        c.tick()
                    else:c.action(action,'invalid')
                    self.assertLessEqual(len(live),1)
                    if c.session and c.session.stop:
                        self.assertEqual(c.phase,'STOPPING')
                c.action('STOP')
    def test_view_requires_current_heartbeat_and_ack_with_bounded_wait(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'display.json';clock=[10.]
            def save(ready=False,session='s',at=10.):path.write_text(json.dumps(dict(session=session,sample_mono=at,view_ready=ready)))
            save(True);wait_view(path,'s',lambda:False,clock=lambda:10.)
            save(True,'foreign')
            with self.assertRaises(ValueError):wait_view(path,'s',lambda:False,clock=lambda:10.)
            save(False)
            with self.assertRaises(InterruptedError):wait_view(path,'s',lambda:True,clock=lambda:10.)
            def tick(_):clock[0]+=1;save(False,at=clock[0])
            with self.assertRaises(TimeoutError):wait_view(path,'s',lambda:False,clock=lambda:clock[0],sleep=tick)
            self.assertLessEqual(clock[0],20.)
    def test_stop_before_monitor_ready_never_starts_pi(self):
        # No file read/process step is permitted when STOP is already pending.
        class Never:
            def tick(self,_):raise AssertionError('must not tick')
        local=type('Local',(),{'processes':{'operator':Never()},'registry':type('Registry',(),{'logs':{'operator':type('Log',(),{'file':None})()}})()})()
        with self.assertRaises(InterruptedError):wait_monitor_ready(local,lambda:True)

    def test_busy_r5_common_lock_refused_without_signaling_or_modifying_owner(self):
        import fcntl
        from services.live_preflight import check_existing_runner_lock
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'common.lock';p.write_bytes(b'owned-marker')
            with p.open('r+') as owner:
                fcntl.flock(owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
                with self.assertRaisesRegex(ValueError,'EXISTING_LAPTOP_SESSION_LOCKED'):
                    check_existing_runner_lock(p)
                self.assertEqual(p.read_bytes(),b'owned-marker')
            check_existing_runner_lock(p)
            self.assertEqual(p.read_bytes(),b'owned-marker')
    def test_missing_common_lock_is_not_created_during_preflight(self):
        from services.live_preflight import check_existing_runner_lock
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'absent.lock';check_existing_runner_lock(p)
            self.assertFalse(p.exists())
