"""A runtime fault stays a failed session after a verified, clean STOP."""
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import os
import subprocess
import sys

from services.operator_runner import OperatorBootstrap, run_integrated
from test_operator_runner import FakeRos, FakeNode, FakeChannel, Clock, SESSION, PACKET


class TerminalResultTests(unittest.TestCase):
    def test_runtime_fault_is_persisted_but_does_not_invalidate_stop_process_exit(self):
        clock = Clock()
        node = FakeNode()
        node.core.fault = 'RUNTIME_RECOVERY_HOLD:NAV2_BACKEND_LOST'
        boot = OperatorBootstrap(SESSION, 'b'*32, 'c'*32, PACKET, Path('/tmp/private/control.sock'))
        with tempfile.TemporaryDirectory() as tmp:
            result = Path(tmp) / 'operator_result.json'
            rc = run_integrated(boot, ros=FakeRos(clock), node_factory=lambda: node,
                channel_factory=FakeChannel, clock=clock, result_path=result)
            self.assertEqual(rc, 0)
            self.assertTrue(node.stop_verification.confirmed)
            row = json.loads(result.read_text())
            self.assertEqual(row['runtime_fault'], node.core.fault)
            self.assertEqual(row['runtime_status'], 'FAILED')
            self.assertEqual(row['session'], SESSION)
            self.assertEqual(row['packet'], PACKET)
            self.assertTrue(row['stop_confirmed'] and row['process_cleanup_ok'])
            from services.operator_result import read_result
            proc = SimpleNamespace(done=True, rc=0, forced=False, error=None,
                pid=row['pid'], leader_birth=row['process_start_ticks'],
                report=lambda: {'group_exit_confirmed': True})
            bound, _ = read_result(result, proc, boot)
            self.assertEqual(bound, row)
            for key, value in [('session', 'old'), ('packet', 'old'), ('generation', 'old'),
                               ('pid', row['pid']+1), ('process_start_ticks', row['process_start_ticks']+1),
                               ('process_cleanup_ok', False), ('stop_confirmed', False),
                               ('runtime_status', 'SUCCEEDED')]:
                with self.subTest(key=key):
                    result.write_text(json.dumps(dict(row, **{key: value})))
                    with self.assertRaises(ValueError): read_result(result, proc, boot)

    def test_closed_log_proof_remains_required_even_with_a_valid_runtime_result(self):
        from services.app_local_processes import AppLocalProcesses
        from services.operator_owned_backend import OperatorOwnedBackend
        result = {'stop_confirmed': False, 'clean': False, 'group_cleanup_clean': False, 'errors': []}
        backend = OperatorOwnedBackend.__new__(OperatorOwnedBackend)
        backend.bootstrap = object()
        with tempfile.TemporaryDirectory() as tmp:
            local = AppLocalProcesses(Path(tmp), SESSION)
            local.processes['operator'] = SimpleNamespace(io=backend)
            local.stack = SimpleNamespace(done=True, report=lambda: dict(result, errors=[]))
            with patch.object(local, 'begin_stop'), patch('services.operator_result.read_result',
                    return_value=({'runtime_status': 'FAILED'}, 'a'*64)):
                row = local.drain()
            self.assertFalse(row['stop_confirmed'] or row['clean'])
            self.assertIn('APP_OPERATOR_RESULT_INVALID:ValueError', row['errors'])

    def test_runtime_fault_without_a_record_cannot_be_silently_successful(self):
        clock = Clock()
        node = FakeNode()
        node.core.fault = 'RUNTIME_RECOVERY_HOLD:NAV2_BACKEND_LOST'
        boot = OperatorBootstrap(SESSION, 'b'*32, 'c'*32, PACKET, Path('/tmp/private/control.sock'))
        self.assertEqual(run_integrated(boot, ros=FakeRos(clock), node_factory=lambda: node,
            channel_factory=FakeChannel, clock=clock), 2)

    def test_teardown_error_still_fails_despite_stop_confirmation(self):
        clock = Clock()
        node = FakeNode()
        node.destroy_node = lambda: (_ for _ in ()).throw(RuntimeError('teardown'))
        boot = OperatorBootstrap(SESSION, 'b'*32, 'c'*32, PACKET, Path('/tmp/private/control.sock'))
        with tempfile.TemporaryDirectory() as tmp:
            result = Path(tmp) / 'operator_result.json'
            self.assertEqual(run_integrated(boot, ros=FakeRos(clock), node_factory=lambda: node,
                channel_factory=FakeChannel, clock=clock, result_path=result), 2)
            self.assertFalse(json.loads(result.read_text())['process_cleanup_ok'])

    def test_result_write_failure_does_not_allow_a_clean_exit(self):
        clock = Clock()
        node = FakeNode()
        boot = OperatorBootstrap(SESSION, 'b'*32, 'c'*32, PACKET, Path('/tmp/private/control.sock'))
        with tempfile.TemporaryDirectory() as tmp:
            result = Path(tmp) / 'operator_result.json'
            result.write_text('sentinel')
            self.assertEqual(run_integrated(boot, ros=FakeRos(clock), node_factory=lambda: node,
                channel_factory=FakeChannel, clock=clock, result_path=result), 2)
            self.assertEqual(result.read_text(), 'sentinel')

    def test_session_error_retains_runtime_fault_with_clean_stop_and_groups(self):
        from services.operator_result import SCOPE
        from services.session_outcome import outcome, primary_error
        from proof_fixture import clean_local, clean_remote
        local = clean_local()
        local.update(runtime_failure=True, runtime_result=dict(scope=SCOPE, session=SESSION,
            packet=PACKET, runtime_fault='RUNTIME_RECOVERY_HOLD:NAV2_BACKEND_LOST'))
        row = dict(session=SESSION, packet=PACKET, profile='R5_AUTOMATIC', rc=2,
                   local=local, remote=clean_remote({'packet': PACKET}))
        self.assertEqual(outcome(row, SESSION, PACKET), (True, True))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path/'receipt.json').write_text(json.dumps(row))
            self.assertEqual(primary_error(path, 'RUNNER_EXIT_2'),
                'OPERATOR_RUNTIME_FAULT:RUNTIME_RECOVERY_HOLD:NAV2_BACKEND_LOST')

    def test_real_owned_child_log_pid_and_result_round_trip_with_fake_ros(self):
        from services.app_local_processes import AppLocalProcesses
        from services.operator_owned_backend import OperatorOwnedBackend
        from services.operator_bootstrap import create_bootstrap_file
        from operational_owned_process import Backend
        app = Path(__file__).resolve().parents[1]
        code = '''
import sys
from pathlib import Path
from services.operator_bootstrap import read_bootstrap_fd
from services.operator_runner import run_integrated
from test_operator_runner import Clock, FakeRos, FakeNode, FakeChannel
b = read_bootstrap_fd(int(sys.argv[1])); clock = Clock(); node = FakeNode()
node.core.fault = sys.argv[3] if sys.argv[3] != 'NONE' else None
node.logger.info = lambda message: print('[INFO] [1.0] [fire_robot_operator]: '+message, flush=True)
node.stop_verification.report = lambda now: dict(scope='R5_STOP_TELEMETRY_VERIFICATION',
    requested=True, done=node.stop_verification.done, confirmed=node.stop_verification.confirmed,
    reason='FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED', blockers=[], fault=None,
    motion_authorized=False, pulse_replayed=False)
raise SystemExit(run_integrated(b, ros=FakeRos(clock), node_factory=lambda: node,
    channel_factory=FakeChannel, clock=clock, result_path=Path(sys.argv[2])))
'''
        for fault in (None, 'RUNTIME_RECOVERY_HOLD:NAV2_BACKEND_LOST'):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as tmp:
                out = Path(tmp)
                boot = OperatorBootstrap(SESSION, 'b'*32, 'c'*32, PACKET, out/'control.sock')
                class TestBackend(OperatorOwnedBackend):
                    def start(self, argv, output):
                        with create_bootstrap_file(boot, out) as private:
                            return subprocess.Popen([sys.executable, '-B', '-c', code, str(private.fileno()),
                                str(out/'operator_result.json'), fault or 'NONE'], pass_fds=(private.fileno(),),
                                start_new_session=True, stdout=output, stderr=subprocess.STDOUT,
                                env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1',
                                    PYTHONPATH=str(app)+os.pathsep+str(app/'tests')))
                backend = TestBackend(boot, delegate=Backend(), app_dir=app, candidate_root=app,
                    session_dir=out, runtime_dir=out, env=dict(os.environ))
                local = AppLocalProcesses(out, SESSION, backend_factory=lambda _: backend)
                process = local.start('operator', backend.command)
                process.process.wait(timeout=4.)
                try:
                    result = local.drain()
                finally:
                    local.close_logs()
                self.assertTrue(result['stop_confirmed'] and result['clean'], result)
                self.assertEqual(result['runtime_failure'], fault is not None)
                self.assertEqual(result['runtime_result']['runtime_fault'], fault)
                self.assertEqual(process.rc, 0)
