"""Fresh STOP recovery with synthetic Pi state and temporary ownership files."""
import copy
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from services import recovery, recovery_probe as probe, recovery_ownership as ownership
from services import stop_recovery_runner as stop
import test_reboot_recovery as remote_fixture
import test_stop_recovery as local_fixture


class PiQuiescentTests(unittest.TestCase):
    def setUp(self):
        self.fixture = remote_fixture.IdlePiProbeTests()
        self.fixture.setUp()
        self.fixture.current = self.fixture.binding['boot_id']
        self.addCleanup(self.fixture.doCleanups)
        self.pairing, self.done, self.lock, self.absent, self.quiet = self.fixture.fixture([self.fixture.current]*30)
        self.done['cleanup']['children_clean'] = False
        self.done['cleanup']['child_audit']['clean'] = False

    def check(self):
        return probe.probe_pi_idle(self.fixture.binding)

    def test_failed_child_gets_current_absence_only_not_old_stop_success(self):
        raw = copy.deepcopy(self.done)
        row = self.check()
        self.assertEqual(row['mode'], 'SAME_BOOT_QUIESCENT_ONLY')
        self.assertEqual(row['scope'], 'QUIESCENT_ONLY_REQUIRES_NEW_STOP')
        self.assertIn(self.fixture.binding['runner_pid'], row['pids_checked'])
        self.assertNotIn('stop_confirmed', row)
        self.assertEqual(self.done, raw)
        self.absent.assert_called_once()
        self.assertIn('[o]dom_to_tf_broadcaster', self.quiet.call_args.kwargs['pattern'])
        with self.assertRaisesRegex(ValueError, 'CLEANUP_MISSING'):
            probe.probe_pi(self.fixture.binding)

    def test_missing_done_can_only_authorize_fresh_verification(self):
        def read(path):
            if path.name == 'pairing.json': return self.pairing, 'c'*64
            raise FileNotFoundError(2, 'synthetic missing DONE', str(path))
        probe.read_object.side_effect = read
        row = self.check()
        self.assertEqual(row['mode'], 'SAME_BOOT_QUIESCENT_ONLY')
        self.assertIsNone(row['historical_done_sha256'])

    def test_live_group_and_unknown_process_stay_blocked(self):
        for target, code in ((self.absent, 'RECOVERY_PROCESS_STILL_LIVE'),
                             (self.quiet, 'RECOVERY_ROBOT_PROCESS_OR_SNAPSHOT_UNKNOWN')):
            target.side_effect = ValueError(code)
            with self.assertRaisesRegex(ValueError, code): self.check()
            target.side_effect = None

    def test_bad_done_identity_and_malformed_pid_stay_blocked(self):
        self.done['session'] = 'foreign'
        with self.assertRaisesRegex(ValueError, 'PI_NOT_DONE'): self.check()
        self.done['session'] = self.fixture.binding['session']
        self.done['cleanup']['groups']['pi']['pid'] = True
        with self.assertRaisesRegex(ValueError, 'PID_INVALID'): self.check()

    def test_busy_lock_and_changed_boot_stay_blocked(self):
        self.lock.side_effect = BlockingIOError('synthetic busy owner')
        with self.assertRaises(BlockingIOError): self.check()
        self.lock.side_effect = None
        Path.read_text.side_effect = [self.fixture.current, 'foreign-boot']
        with self.assertRaisesRegex(ValueError, 'BOOT_CHANGED'): self.check()


class QuiescentTransportTests(unittest.TestCase):
    def setUp(self):
        self.fixture = remote_fixture.IdleTransportTests()
        self.fixture.setUp()
        self.fixture.boot = self.fixture.binding['boot_id']
        self.good = dict(mode='SAME_BOOT_QUIESCENT_ONLY', pids_checked=[self.fixture.binding['runner_pid']],
                         historical_done_sha256=None, scope='QUIESCENT_ONLY_REQUIRES_NEW_STOP')

    def test_transport_accepts_only_challenged_current_absence_scope(self):
        result = self.fixture.request(self.good)
        self.assertEqual(result['mode'], 'SAME_BOOT_QUIESCENT_ONLY')
        for key, value in [('pids_checked', []), ('pids_checked', [True]),
                           ('scope', 'STOP_CONFIRMED'), ('stop_confirmed', True),
                           ('historical_done_sha256', 'invalid'), ('request_nonce', 'replay')]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.fixture.request(dict(self.good, **{key: value}))


class OwnershipTests(unittest.TestCase):
    def setUp(self):
        self.fixture = local_fixture.StopRecoveryTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.tearDown)
        f = self.fixture
        self.journal = dict(schema_version=1, session=f.marker['session'], packet=f.marker['packet'],
            kind='mapping', stage='LAPTOP_STARTED', laptop_runner=dict(pid=90005, start_ticks=123),
            pi=copy.deepcopy(f.row['remote']['binding']))
        self.path = f.old/'recovery_ownership.json'
        self.path.write_text(json.dumps(self.journal))
        (f.old/'laptop/receipt.json').unlink()

    def read(self):
        return ownership.journal_groups(self.fixture.marker, self.fixture.old)

    def test_missing_final_receipt_uses_owned_identity_but_not_read_only_stop(self):
        with patch.object(probe, 'absent') as absent:
            binding, pids = self.read()
            self.assertEqual(binding, self.journal['pi'])
            self.assertEqual(pids, [90005])
            absent.assert_called_once_with([90005])
        with self.assertRaises(FileNotFoundError):
            recovery.local_proof(self.fixture.marker, self.fixture.old)

    def test_missing_both_receipts_can_prepare_when_owned_groups_absent(self):
        f = self.fixture
        (f.old/'desktop_receipt.json').unlink()
        with patch.object(probe, 'absent'), patch.object(probe, 'no_robot_processes'):
            row = stop.prepare(f.root, remote=lambda _: dict(current_boot_id=self.journal['pi']['boot_id']), lock_path=f.lock)
        self.assertEqual(row['marker_sha256'], f.digest)
        self.assertEqual((f.root/'pending.json').read_bytes(), f.raw)

    def test_incomplete_spawn_wrong_binding_and_live_pid_are_not_accepted(self):
        for key, value in [('stage', 'LAPTOP_SPAWNING'), ('session', 'foreign'), ('laptop_runner', None)]:
            self.path.write_text(json.dumps(dict(self.journal, **{key: value})))
            with self.subTest(key=key), self.assertRaises(ValueError): self.read()
        self.path.write_text(json.dumps(self.journal))
        with patch.object(probe, 'absent', side_effect=ValueError('RECOVERY_PROCESS_STILL_LIVE')):
            with self.assertRaisesRegex(ValueError, 'STILL_LIVE'): self.read()

    def test_contradictory_receipt_and_symlink_are_rejected(self):
        f = self.fixture
        row = copy.deepcopy(f.row); row['session'] = 'foreign'
        (f.old/'laptop/receipt.json').write_text(json.dumps(row))
        with self.assertRaisesRegex(ValueError, 'BINDING_MISMATCH'): self.read()
        self.path.unlink(); self.path.symlink_to(f.old/'desktop_receipt.json')
        with self.assertRaises(OSError): self.read()

    def test_launch_child_identity_is_checked_even_without_group_receipt(self):
        (self.fixture.old/'laptop/nav.log').write_text('[INFO] [container-1]: process started with pid [90009]\n')
        with patch.object(probe, 'absent') as absent:
            _, pids = self.read()
            self.assertEqual(pids, [90005, 90009])
            absent.assert_called_once_with(pids)

    def test_pairing_only_is_valid_but_never_contains_secret(self):
        self.journal.update(stage='PAIRED', laptop_runner=None)
        self.path.write_text(json.dumps(self.journal))
        with patch.object(probe, 'absent'):
            _, pids = self.read()
        self.assertEqual(pids, [])
        private = dict(self.journal['pi'], secret='private-test-value')
        self.assertNotIn('secret', ownership.public_binding(private, self.journal['packet']))

    def test_real_self_process_identity_is_pinned_without_signals(self):
        row = ownership.process_identity(os.getpid())
        self.assertEqual(row['pid'], os.getpid())
        self.assertGreater(row['start_ticks'], 0)

    def test_missing_receipt_can_archive_only_after_fresh_stop_and_cleanup(self):
        f = self.fixture
        original = self.path.read_bytes()
        new = f.new_proof(False)
        with patch.object(probe, 'absent'), patch.object(probe, 'no_robot_processes'):
            with self.assertRaisesRegex(ValueError, 'NEW_STOP_NOT_CONFIRMED'):
                stop.audit_new_stop(f.root, new, lock_path=f.lock)
            self.assertEqual((f.root/'pending.json').read_bytes(), f.raw)
            receipt_path = new/'laptop/receipt.json'
            receipt = json.loads(receipt_path.read_text()); receipt['local']['stop_confirmed'] = True
            receipt_path.write_text(json.dumps(receipt))
            desktop_path = new/'desktop_receipt.json'
            desktop = json.loads(desktop_path.read_text()); desktop['robot_stop_confirmed'] = True
            desktop_path.write_text(json.dumps(desktop))
            result = stop.audit_new_stop(f.root, new, remote=lambda _: dict(fixture=True),
                idle_remote=lambda *_, **__: dict(fixture=True), lock_path=f.lock)
            recovery.archive_verified(f.root, result, '8'*32, lock_path=f.lock)
        self.assertFalse((f.root/'pending.json').exists())
        self.assertEqual(self.path.read_bytes(), original)

    def test_worker_records_private_pairing_as_public_ownership(self):
        from services.live_session import LiveSession
        f = self.fixture
        worker = LiveSession(candidate=f.root, packet=f.marker['packet'], runtime=f.root,
            out=f.old, map_root=f.root, map_roots=[f.root], kind='mapping', selected_map=None, emit=lambda _: None)
        worker.remote_binding = dict(self.journal['pi'], secret='private-test-value')
        worker._record_ownership('PAIRED')
        row = json.loads(self.path.read_text())
        self.assertEqual(row['stage'], 'PAIRED')
        self.assertNotIn('private-test-value', self.path.read_text())
        worker._record_ownership('LAPTOP_STARTED', os.getpid())
        row = json.loads(self.path.read_text())
        self.assertEqual(row['laptop_runner']['pid'], os.getpid())
        self.assertGreater(row['laptop_runner']['start_ticks'], 0)
