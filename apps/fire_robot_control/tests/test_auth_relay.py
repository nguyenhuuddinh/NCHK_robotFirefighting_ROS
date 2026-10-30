"""Authentication relay must reach Qt; injected endpoints, no credentials/SSH."""
import json
from pathlib import Path
import tempfile
import unittest
import uuid

from services.supervisor import SessionController
from services.live_session import LiveSession


class AuthRelayTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        for name in ('maps','runtime','state'):(self.root/name).mkdir()
        class Fake:
            def __init__(obj,**kw):
                obj.session=str(uuid.uuid4());obj.kind=kw['kind'];obj.out=kw['out'];obj.sent=[];obj.emit=kw['emit']
            def start(obj):pass
            def send(obj,*args):obj.sent.append(args)
            def heartbeat(obj,*args):pass
        self.c=SessionController(candidate=self.root,packet='a'*64,state_dir=self.root/'state',runtime=self.root/'runtime',
            map_root=self.root/'maps',map_roots=[self.root/'maps'],session_factory=Fake)
        self.c.action('SCAN');self.c.tick();self.worker=self.c.session

    def test_password_request_from_worker_reaches_ui_and_answer_returns_to_worker(self):
        row=dict(event='AUTH_REQUIRED',session=self.worker.session,label='Nhập mật khẩu SSH/sudo của Pi')
        self.worker.emit(row)
        self.assertIn(row,self.c.tick())
        self.c.action('AUTH','fixture-only')
        self.assertEqual(self.worker.sent,[('AUTH','fixture-only')])

    def test_stale_missing_or_finished_session_cannot_prompt(self):
        for sid in ('foreign',None):
            row=dict(event='AUTH_REQUIRED',session=sid,label='unsafe')
            self.c.inbox.put(row);self.assertFalse(self.c.tick())
        self.c.action('STOP')
        self.c.inbox.put(dict(event='AUTH_REQUIRED',session=self.worker.session))
        self.assertFalse(self.c.tick())
        self.c.inbox.put(dict(event='FINISHED',session=self.worker.session,robot_stop_confirmed=True,
            cleanup_clean=True,session_was_started=True))
        self.c.tick();self.c.inbox.put(dict(event='AUTH_REQUIRED',session=self.worker.session))
        self.assertFalse(self.c.tick())

    def test_done_is_relayed_and_remote_label_contents_are_not_forwarded(self):
        self.c.inbox.put(dict(event='AUTH_DONE',session=self.worker.session,secret='private-not-log'))
        self.assertEqual(self.c.tick(),[dict(event='AUTH_DONE',session=self.worker.session)])
        self.c.inbox.put(dict(event='AUTH_REQUIRED',session=self.worker.session,label='private-raw-ssh',secret='private'))
        text=json.dumps(self.c.tick());self.assertNotIn('private',text);self.assertIn('AUTH_REQUIRED',text)

    def test_fragmented_sudo_prompt_crosses_real_worker_controller_chain(self):
        events=[]
        runner=LiveSession(candidate=self.root,packet='a'*64,runtime=self.root/'runtime',out=self.root/'state',
            map_root=self.root/'maps',map_roots=[self.root/'maps'],kind='verify_stop',selected_map=None,emit=self.c.inbox.put)
        self.c.session=runner
        class Child:
            pieces=[b'helper diagnostic\r\n[sudo] pass',b'word for pi: ']
            def read(obj):return obj.pieces.pop(0)
            def write(obj,data):events.append(data);return len(data)
        child=Child();runner._read_pty(child);self.assertFalse(self.c.tick())
        runner._read_pty(child);rows=self.c.tick();self.assertEqual(rows[0]['event'],'AUTH_REQUIRED')
        self.assertEqual(rows[0]['session'],runner.session)
        # Only an inert child receives this fixture, never SSH or sudo.
        runner.send('AUTH','fixture-only');runner._pump()
        self.assertEqual(events,[b'fixture-only\n'])
        self.assertEqual(self.c.tick(),[dict(event='AUTH_DONE',session=runner.session)])

    def test_close_dominates_pending_prompt_and_answer(self):
        self.c.action('CLOSE')
        self.c.inbox.put(dict(event='AUTH_REQUIRED',session=self.worker.session))
        self.assertFalse(self.c.tick())

    def test_exit_in_same_batch_retracts_unsent_password_modal(self):
        self.c.inbox.put(dict(event='AUTH_REQUIRED',session=self.worker.session))
        self.c.inbox.put(dict(event='FINISHED',session=self.worker.session,robot_stop_confirmed=None,
            cleanup_clean=True,session_was_started=False))
        rows=self.c.tick()
        self.assertFalse(any(r['event']=='AUTH_REQUIRED' for r in rows))
        self.assertTrue(any(r['event']=='FINISHED' for r in rows))
