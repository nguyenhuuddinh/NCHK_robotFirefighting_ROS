"""Owned monitor STOP parsing and Pi-exit races; never SSH or ROS commands."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

from operational_entry import LocalProcesses
from operational_stream_log import Audit
from services.monitor_log import MonitorAudit, start_monitor
from services.live_session import LiveSession
from services.session_outcome import outcome
from proof_fixture import clean_local, clean_remote


PROOF = dict(scope='R5_STOP_TELEMETRY_VERIFICATION', requested=True, done=True,
             confirmed=True, reason='FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED',
             blockers=[], fault=None, motion_authorized=False, pulse_replayed=False)


def line(proof=PROOF, name='fire_robot_app_stop_monitor'):
    return ('[INFO] [1793311656.519762994] ['+name+']: OPERATOR_STOP '+json.dumps(proof)+'\n').encode()


class MonitorReceiptTests(unittest.TestCase):
    def test_real_saved_log_old_parser_misses_monitor_app_parser_recognizes(self):
        raw=b'MAPPING_MONITOR_READY\n'+line()
        old=Audit();old.feed(raw)
        self.assertEqual(old.receipts,0)
        new=MonitorAudit()
        for b in raw:new.feed(bytes([b]))
        self.assertEqual(new.receipts,1);self.assertEqual(new.proof,PROOF)
        self.assertEqual(new.errors,[])

    def test_real_owned_process_stack_requires_single_complete_valid_stop(self):
        bad=copy.deepcopy(PROOF);bad['confirmed']=False
        for raw,code,expected in [(line(),0,True),(b'',0,False),(line(bad),0,False),
                                  (line()+line(),0,False),(line(name='fire_robot_operator'),0,False),
                                  (line()[:-1],0,False),(line(),2,False)]:
            with self.subTest(raw=raw,code=code),tempfile.TemporaryDirectory() as tmp:
                local=LocalProcesses(Path(tmp),'fixture')
                script=('import signal,sys\n'
                        'def stop(*_):\n'
                        ' sys.stdout.buffer.write('+repr(raw)+');sys.stdout.flush();sys.exit('+str(code)+')\n'
                        'signal.signal(signal.SIGINT,stop)\n'
                        'print("MAPPING_MONITOR_READY",flush=True)\n'
                        'signal.pause()\n')
                try:
                    start_monitor(local,[sys.executable,'-u','-c',script])
                    deadline=time.monotonic()+3
                    while b'MAPPING_MONITOR_READY' not in local.registry.logs['operator'].path.read_bytes():
                        self.assertLess(time.monotonic(),deadline);time.sleep(.01)
                    result=local.drain(cause='SYNTHETIC_MONITOR_QA')
                    self.assertEqual(result['stop_confirmed'],expected,result)
                    self.assertEqual(result['clean'],expected,result)
                    self.assertTrue(result['processes']['operator']['group_exit_confirmed'])
                finally:
                    if local.processes:local.drain()
                    local.close_logs()

    def test_monitor_parser_rejects_invalid_json_and_does_not_change_parent_parser(self):
        a=MonitorAudit();a.feed(b'[INFO] [1.0] [fire_robot_app_stop_monitor]: OPERATOR_STOP {bad}\n')
        self.assertIn('INVALID_STOP_JSON',a.errors)
        parent=Audit();parent.feed(line(name='fire_robot_operator'))
        self.assertEqual(parent.proof,PROOF)

    def test_only_bound_stopping_status_and_successful_ssh_can_defer_exit_error(self):
        worker=object.__new__(LiveSession);worker.session='owned'
        for code,stage,sid,expected in [(0,'STOPPING','owned',True),(0,'WAIT_PI_IO','owned',False),
                (0,'STOPPING','foreign',False),(2,'STOPPING','owned',False),
                (None,'STOPPING','owned',False)]:
            worker.pi=SimpleNamespace(poll=lambda:code)
            self.assertEqual(worker._pi_exit_is_drain(dict(stage=stage,session=sid)),expected)

    def test_drain_status_cannot_turn_missing_or_failed_receipt_into_stop_proof(self):
        for local in ({},dict(stop_confirmed=False),dict(clean=True)):
            self.assertEqual(outcome(dict(session='s',packet='p',rc=0,local=local,
                remote=clean_remote({})),'s','p'),(False,False))
        good=dict(session='s',packet='p',rc=0,local=clean_local(),remote=clean_remote({}))
        self.assertEqual(outcome(good,'s','p'),(True,True))

    def test_status_reader_rejects_foreign_and_broken_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            worker=object.__new__(LiveSession);worker.out=Path(tmp);worker.kind='mapping';worker.session='s'
            (worker.out/'laptop').mkdir();path=worker.out/'laptop/mapping.json'
            for content in ['{bad','[]',json.dumps(dict(session='other',stage='STOPPING'))]:
                path.write_text(content);self.assertIsNone(worker._runner_status())
            path.write_text(json.dumps(dict(session='s',stage='STOPPING')))
            self.assertEqual(worker._runner_status()['stage'],'STOPPING')
