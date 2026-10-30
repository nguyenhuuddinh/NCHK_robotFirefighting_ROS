"""Bounded preadmission handshake: no live motion lease is extended."""
import json
import os
from pathlib import Path
import tempfile
import unittest
import uuid
from services.operator_runner import OperatorBootstrap
from services.supervisor import ControlProtocol,SessionController
from services.live_session import LiveSession

class ViewBootstrapTests(unittest.TestCase):
    def setUp(self):
        self.binding=OperatorBootstrap(str(uuid.uuid4()),'b'*32,'c'*32,'a'*64,Path('/tmp/test.sock'))
        self.protocol=ControlProtocol(self.binding,10.)
    def send(self,event,seq,when,argument=None,nonce=None):
        b=self.binding
        row=dict(version=1,session=b.session,generation=b.generation,nonce=nonce or b.nonce,
                 seq=seq,sent_mono=when,event=event,argument=argument)
        return self.protocol.accept(json.dumps(row).encode(),when,os.getuid())
    def test_normal_350ms_lease_is_unchanged(self):
        self.assertIsNotNone(self.send('HEARTBEAT',1,10.))
        self.assertTrue(self.protocol.expired(10.351))
    def test_scoped_wait_authenticates_but_cannot_start_and_never_extends_deadline(self):
        self.send('HEARTBEAT',1,10.);self.protocol.begin_view_wait(13.,10.)
        self.assertFalse(self.protocol.expired(12.9))
        self.assertIsNotNone(self.send('HEARTBEAT',2,12.9))
        self.assertIsNone(self.send('SCAN',3,12.91))
        self.assertIsNone(self.send('HEARTBEAT',3,12.92,nonce='d'*32))
        self.assertTrue(self.protocol.expired(13.))
    def test_ready_restores_strict_lease_stop_cancels_bootstrap(self):
        for event in ('VIEW_READY','STOP','CLOSE'):
            self.setUp();self.send('HEARTBEAT',1,10.);self.protocol.begin_view_wait(13.,10.)
            self.assertIsNotNone(self.send(event,2,12.))
            self.assertIsNone(self.protocol.view_wait_until)
            self.assertTrue(self.protocol.expired(12.351))
    def test_worker_only_allows_one_view_init_before_admission(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            worker=LiveSession(candidate=root,packet='a'*64,runtime=root,out=root,map_root=root,
                map_roots=[root],kind='fire',selected_map=None,emit=lambda _:None,clock=lambda:10.)
            for stage in ('PREFLIGHT','SEAL_PI','READY_GATE2','MAPPING'):
                worker.stage=stage
                with self.assertRaises(ValueError):worker.begin_view_init()
            worker.stage='POSE';self.assertEqual(worker.begin_view_init(),13.)
            with self.assertRaises(ValueError):worker.begin_view_init()
    def test_async_mapping_does_not_grant_heartbeat_grace(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            worker=LiveSession(candidate=root,packet='a'*64,runtime=root,out=root,map_root=root,
                map_roots=[root],kind='mapping',selected_map=None,emit=lambda _:None,clock=lambda:10.)
            with self.assertRaises(ValueError): worker.begin_async_mapping_view()
            worker.stage='WAIT_SLAM'
            worker.begin_async_mapping_view()
            self.assertEqual(worker.view_init_until, 0.)
            with self.assertRaises(ValueError): worker.begin_async_mapping_view()
            with self.assertRaises(ValueError): worker.begin_view_init()
            self.assertIsNotNone(self.send('HEARTBEAT',1,10.))
            self.assertIsNotNone(self.send('VIEW_ASYNC_INIT',2,10.01,worker.session))
            self.assertTrue(self.protocol.expired(10.361))
    def test_async_view_rejected_for_fire(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            worker=LiveSession(candidate=root,packet='a'*64,runtime=root,out=root,map_root=root,
                map_roots=[root],kind='fire',selected_map=None,emit=lambda _:None,clock=lambda:10.)
            worker.stage='POSE'
            with self.assertRaises(ValueError): worker.begin_async_mapping_view()
    def test_controller_rejects_runtime_or_foreign_view_init(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for name in ('maps','runtime','state'):(root/name).mkdir()
            c=SessionController(candidate=root,packet='a'*64,state_dir=root/'state',runtime=root/'runtime',
                 map_root=root/'maps',map_roots=[root/'maps'])
            for event in ('VIEW_INIT','VIEW_READY'):
                c.action(event,'foreign')
                self.assertIsNone(c.session)
            self.assertFalse(any(r.get('event')=='VIEW_INIT_GRANTED' for r in c.tick()))
