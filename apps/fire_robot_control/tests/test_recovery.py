"""Retry proof/transaction QA. Fixtures only; never runs ROS or device I/O."""
import copy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import uuid
from unittest.mock import patch

from services import recovery, recovery_probe as probe
from services.supervisor import SessionController

APP = Path(__file__).resolve().parents[1]
EVIDENCE = APP / 'docs/qa_evidence/mapping_failure_195012_20261029'


class Fixture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name)
        self.out = self.base / ('session_' + '1' * 32)
        (self.out / 'laptop').mkdir(parents=True)
        self.desktop = json.loads((EVIDENCE / 'laptop_session/desktop_receipt.json').read_text())
        self.receipt = json.loads((EVIDENCE / 'laptop_session/laptop/receipt.json').read_text())
        self.done = json.loads((EVIDENCE / 'pi/DONE.json').read_text())
        self.marker = dict(session=self.desktop['session'], packet=self.desktop['packet'], out=str(self.out))
        self.marker_raw = json.dumps(self.marker).encode()
        (self.base / 'pending.json').write_bytes(self.marker_raw)
        self.save_receipts()
        self.lock = self.base / 'runner.lock'
        self.lock.touch()
        self.result = dict(marker_sha256=hashlib.sha256(self.marker_raw).hexdigest(),
                           session=self.marker['session'], packet=self.marker['packet'], checked_mono=100.)

    def save_receipts(self):
        (self.out / 'desktop_receipt.json').write_text(json.dumps(self.desktop))
        (self.out / 'laptop/receipt.json').write_text(json.dumps(self.receipt))

    def tearDown(self):
        self.temp.cleanup()

    def local(self):
        return recovery.local_proof(self.marker, self.out)


class ProofTests(Fixture):
    def test_actual_failed_mapping_proof_has_stop_and_clean_groups_despite_runner_error(self):
        pids, binding, hashes = self.local()
        self.assertEqual(len(pids), 2)
        self.assertEqual(binding['packet'], self.marker['packet'])
        self.assertEqual(self.done['error'], 'PI_SEAL_HEALTH_TIMEOUT')
        self.assertEqual(len(probe.remote_cleanup(self.done['cleanup'])), 2)
        self.assertEqual(len(hashes['laptop_receipt_sha256']), 64)

    def test_missing_stop_binding_or_unknown_profile_rejects(self):
        variants = [
            ('desktop', 'robot_stop_confirmed', False),
            ('desktop', 'session_was_started', False),
            ('desktop', 'session', str(uuid.uuid4())),
            ('receipt', 'packet', 'c' * 64),
            ('receipt', 'profile', 'R5_GATE2'),
        ]
        for target, key, value in variants:
            with self.subTest(target=target, key=key):
                row = getattr(self, target)
                old = row[key]
                row[key] = value
                self.save_receipts()
                with self.assertRaises(ValueError): self.local()
                row[key] = old
        self.save_receipts()

    def test_fire_profile_uses_the_same_stop_and_cleanup_proofs(self):
        self.receipt['profile'] = 'R5_AUTOMATIC'
        self.save_receipts()
        self.local()
        for target, key, bad in [('desktop', 'robot_stop_confirmed', False),
                                 ('local', 'clean', False), ('local', 'stop_confirmed', False),
                                 ('local', 'group_cleanup_clean', False), ('local', 'errors', ['ERROR'])]:
            row = self.desktop if target == 'desktop' else self.receipt['local']
            old = row[key]; row[key] = bad; self.save_receipts()
            with self.subTest(key=key):
                with self.assertRaises(ValueError): self.local()
            row[key] = old
        self.save_receipts()

    def test_fire_crashed_nav_child_or_forced_group_cannot_recover(self):
        self.receipt['profile'] = 'R5_AUTOMATIC'
        good = copy.deepcopy(self.receipt['local'])
        for bad in ('crash', 'missing_child', 'forced_group'):
            local = self.receipt['local'] = copy.deepcopy(good)
            if bad == 'forced_group':
                next(iter(local['processes'].values()))['forced'] = True
            else:
                local['child_log_audit']['roles']['nav'] = dict(clean=False, started=4,
                    terminal=3 if bad == 'missing_child' else 4, missing=[10] if bad == 'missing_child' else [],
                    errors=['exit -6'] if bad == 'crash' else [], internal_signal_fallback=False)
            self.save_receipts()
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError): self.local()

    def test_actual_failed_fire_receipt_passes_local_proof(self):
        evidence = APP / 'docs/qa_evidence/fire_retry_20261029/failed_fire'
        self.desktop = json.loads((evidence / 'desktop_receipt.json').read_text())
        self.receipt = json.loads((evidence / 'receipt.json').read_text())
        self.marker.update(session=self.desktop['session'], packet=self.desktop['packet'])
        self.save_receipts()
        pids, binding, _ = self.local()
        self.assertEqual(self.receipt['profile'], 'R5_AUTOMATIC')
        self.assertEqual(len(pids), 2)
        self.assertEqual(binding['packet'], self.marker['packet'])

    def test_failure_messages_explain_network_profile_and_do_not_echo_contents(self):
        for exc, expected in [(ValueError('RECOVERY_CONNECT_PI_WIFI_OR_LAN'), 'kết nối'),
                              (ValueError('RECOVERY_PROFILE_UNSUPPORTED'), 'chưa được hỗ trợ'),
                              (PermissionError('private secret contents'), 'quyền truy cập'),
                              (RuntimeError('private secret contents'), 'Chưa đủ bằng chứng')]:
            result = recovery.failure_message(exc)
            self.assertIn(expected, result)
            self.assertNotIn('private secret contents', result)

    def test_partial_forced_unclean_or_incomplete_children_reject(self):
        variants = [({},), (dict(self.done['cleanup'], forced=True),),
                    (dict(self.done['cleanup'], remote_groups_exited=False),)]
        for args in variants:
            with self.subTest(row=args):
                with self.assertRaises(ValueError): probe.remote_cleanup(*args)
        for key, value in [('forced', True), ('leader_rc', -15), ('error', 'FAILED'),
                           ('done', False), ('live_pids', [7113]), ('group_exit_confirmed', False)]:
            row = copy.deepcopy(self.done['cleanup'])
            row['groups']['pi'][key] = value
            with self.subTest(key=key):
                with self.assertRaises(ValueError): probe.remote_cleanup(row)
        row = copy.deepcopy(self.done['cleanup'])
        row['child_audit']['roles']['pi']['terminal'] = 7
        with self.assertRaises(ValueError): probe.remote_cleanup(row)

    def test_json_duplicate_nonfinite_truncated_oversize_and_symlink_reject(self):
        for raw in (b'{"a":1,"a":2}', b'{"a":NaN}', b'{', b'[]', b'x' * (1024 * 1024 + 1)):
            with self.subTest(raw=raw[:30]):
                with self.assertRaises(ValueError): probe.decode(raw)
        link = self.base / 'link'
        link.symlink_to(self.base / 'pending.json')
        with self.assertRaises(OSError): probe.read_object(link)

    def test_process_group_live_descendant_pid_reuse_and_zombie(self):
        proc = self.base / 'proc'
        (proc / '501').mkdir(parents=True)
        def stat(state, pgid=500, sid=500):
            # PID (comm) state ppid pgrp session ... starttime
            (proc / '501/stat').write_text('501 (test (name)) ' + ' '.join(
                [state, '1', str(pgid), str(sid)] + ['0'] * 15 + ['123']))
        stat('S')
        with self.assertRaisesRegex(ValueError, 'STILL_LIVE'): probe.absent([500], proc)
        stat('Z')
        probe.absent([500], proc)
        stat('S', 999, 999)
        with self.assertRaisesRegex(ValueError, 'STILL_LIVE'): probe.absent([501], proc)
        probe.absent([500], proc)

    def test_route_requires_direct_reviewed_device_and_source(self):
        def run_for(row):
            return lambda *a, **kw: SimpleNamespace(returncode=0, stdout=json.dumps([row]))
        for row in [dict(dev='wlp8s0', prefsrc='10.0.0.253', gateway='10.0.0.1'),
                    dict(dev='enp7s0', prefsrc='10.251.11.86'), dict(dev='other', prefsrc='10.0.0.253')]:
            with self.assertRaisesRegex(ValueError, 'CONNECT_PI'): recovery.pi_host(run_for(row))
        self.assertEqual(recovery.pi_host(run_for(dict(dev='wlp8s0', prefsrc='10.0.0.253'))), '10.0.0.1')
        self.assertEqual(recovery.pi_host(run_for(dict(dev='enp7s0', prefsrc='10.42.0.1'))), '10.42.0.185')

    def test_remote_transport_and_mismatched_proof_reject(self):
        binding = self.receipt['remote']['binding']
        with patch.object(recovery, 'pi_host', return_value='10.42.0.185'):
            for result in [SimpleNamespace(returncode=255, stdout=''),
                           SimpleNamespace(returncode=0, stdout='{"ok":false,"error":"BAD"}'),
                           SimpleNamespace(returncode=0, stdout='{"ok":true,"proof":{}}')]:
                with self.assertRaises(ValueError): recovery.remote_proof(binding, run=lambda *a, **kw: result)

    def test_audit_holds_laptop_lock_and_does_not_start_or_signal(self):
        with patch.object(probe, 'absent') as absent, patch.object(probe, 'no_robot_processes') as names:
            def remote(binding):
                fd = os.open(self.lock, os.O_RDWR)
                try:
                    with self.assertRaises(BlockingIOError): fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                finally: os.close(fd)
                return {'binding': binding}
            result = recovery.audit(self.base, remote=remote, lock_path=self.lock)
            self.assertEqual(result['session'], self.marker['session'])
            self.assertEqual(absent.call_count, 2)
            self.assertEqual(names.call_count, 2)
        self.assertEqual((self.base / 'pending.json').read_bytes(), self.marker_raw)

    def test_busy_runner_lock_blocks_before_ssh(self):
        with self.lock.open('r+') as owner:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with patch.object(recovery, 'remote_proof') as remote:
                with self.assertRaises(BlockingIOError): recovery.audit(self.base, remote=remote, lock_path=self.lock)
                remote.assert_not_called()

    def test_missing_receipt_or_changed_marker_during_network_audit_blocks(self):
        path = self.out / 'desktop_receipt.json'
        path.unlink()
        with self.assertRaises(FileNotFoundError): self.local()
        self.save_receipts()
        with patch.object(probe, 'absent'), patch.object(probe, 'no_robot_processes'):
            def remote(_):
                (self.base / 'pending.json').write_text(json.dumps(dict(self.marker, supervisor_pid=999)))
                return {}
            with self.assertRaisesRegex(ValueError, 'MARKER_CHANGED'):
                recovery.audit(self.base, remote=remote, lock_path=self.lock)
        self.assertTrue((self.base / 'pending.json').exists())

    def test_pi_boot_binding_live_runner_and_clean_error_paths(self):
        binding = self.receipt['remote']['binding']
        pairing = dict(binding, secret='private-never-return')
        def read(path):
            return (pairing if path.name == 'pairing.json' else self.done), 'a' * 64
        with patch.object(Path, 'is_symlink', return_value=False), patch.object(Path, 'resolve', lambda p: p), \
             patch.object(Path, 'read_text', return_value=binding['boot_id']), \
             patch.object(probe, 'read_object', side_effect=read), \
             patch.object(probe, 'lock_existing', return_value=123), patch.object(probe.os, 'close'), \
             patch.object(probe, 'absent') as absent, patch.object(probe, 'no_robot_processes'):
            result = probe.probe_pi(binding)
            self.assertNotIn('private-never-return', json.dumps(result))
            self.assertEqual(result['original_runner_error'], 'PI_SEAL_HEALTH_TIMEOUT')
            with patch.object(Path, 'read_text', return_value='different-boot'):
                with self.assertRaisesRegex(ValueError, 'BOOT_CHANGED'): probe.probe_pi(binding)
            pairing['session'] = str(uuid.uuid4())
            with self.assertRaisesRegex(ValueError, 'BINDING_MISMATCH'): probe.probe_pi(binding)
            pairing['session'] = binding['session']
            absent.side_effect = ValueError('RECOVERY_PROCESS_STILL_LIVE')
            with self.assertRaisesRegex(ValueError, 'STILL_LIVE'): probe.probe_pi(binding)


class ArchiveTests(Fixture):
    def test_archive_preserves_old_marker_receipts_and_failure(self):
        before = (self.out / 'laptop/receipt.json').read_bytes()
        rid = uuid.uuid4().hex
        recovery.archive_verified(self.base, self.result, rid, now=lambda: 101.)
        self.assertFalse((self.base / 'pending.json').exists())
        self.assertEqual((self.base / 'recoveries' / rid / 'pending.json').read_bytes(), self.marker_raw)
        self.assertEqual((self.out / 'laptop/receipt.json').read_bytes(), before)

    def test_stale_changed_or_unwritable_marker_never_clears(self):
        for result in [dict(self.result, checked_mono=90.), dict(self.result, marker_sha256='0' * 64)]:
            with self.assertRaises(ValueError): recovery.archive_verified(self.base, result, uuid.uuid4().hex, now=lambda: 101.)
            self.assertTrue((self.base / 'pending.json').exists())
        for target in ('atomic', 'os.replace'):
            with patch('services.recovery.' + target, side_effect=PermissionError('disk')):
                with self.assertRaises(OSError): recovery.archive_verified(self.base, self.result, uuid.uuid4().hex, now=lambda: 101.)
            self.assertEqual((self.base / 'pending.json').read_bytes(), self.marker_raw)


class FakeAudit:
    def __init__(self, **kw):
        self.id, self.emit, self.started = uuid.uuid4().hex, kw['emit'], False
    def start(self): self.started = True
    def finish(self, **kw): self.emit(dict(event='RECOVERY_FINISHED', recovery_id=self.id, **kw))


class ControllerTests(Fixture):
    def controller(self, factory=FakeAudit):
        maps, runtime = self.base / 'maps', self.base / 'runtime'
        maps.mkdir(exist_ok=True); runtime.mkdir(exist_ok=True)
        # A new app packet must be allowed to audit the original failed packet.
        return SessionController(candidate=APP, packet='b' * 64, state_dir=self.base,
            runtime=runtime, map_roots=[maps], map_root=maps, recovery_factory=factory, clock=lambda: 101.)

    def test_duplicate_check_and_scan_during_check_cannot_start_session(self):
        c = self.controller()
        c.action('RECHECK')
        worker = c.recovery
        self.assertEqual(c.phase, 'CHECKING')
        self.assertEqual(c.status()['recovery_attempt'], 1)
        self.assertEqual(c.status()['recovery_result'], 'CHECKING')
        for event in ('RECHECK', 'SCAN', 'FIRE', 'SELECT', 'SAVE'):
            c.action(event, 'bad')
            self.assertIs(c.recovery, worker)
            self.assertIsNone(c.session)
            self.assertEqual(c.status()['recovery_attempt'], 1)
        worker.finish(result=self.result)
        c.tick()
        self.assertEqual(c.phase, 'IDLE')
        self.assertIsNone(c.session)  # A second explicit Quét map click is required.
        self.assertIsNone(c.last_stop)
        self.assertEqual(c.status()['recovery_result'], 'SUCCEEDED')
        self.assertEqual(c.tick(), [])

    def test_close_or_stop_during_audit_keeps_marker_and_waits(self):
        for event in ('CLOSE', 'STOP'):
            c = self.controller()
            c.action('RECHECK'); worker = c.recovery
            c.action(event)
            self.assertFalse(c.status()['close_ready'])
            worker.finish(result=self.result); c.tick()
            self.assertEqual(c.phase, 'BLOCKED')
            self.assertEqual(c.status()['recovery_result'], 'CANCELLED')
            self.assertIsNone(c.recovery)
            self.assertTrue((self.base / 'pending.json').exists())
            self.assertEqual(c.status()['close_ready'], event == 'CLOSE')

    def test_failure_unknown_old_callback_and_disk_error_keep_blocked(self):
        c = self.controller()
        c.action('RECHECK'); worker = c.recovery
        c.inbox.put(dict(event='RECOVERY_FINISHED', recovery_id='old', result=self.result)); c.tick()
        self.assertIs(c.recovery, worker)
        worker.finish(error='Pi disconnected'); c.tick()
        self.assertEqual(c.phase, 'BLOCKED')
        self.assertEqual(c.status()['recovery_result'], 'FAILED')
        self.assertIn('Đã kiểm tra', c.status()['recovery_message'])
        c.action('RECHECK'); c.recovery.finish(result=self.result)
        with patch('services.supervisor.archive_verified', side_effect=PermissionError('disk')): c.tick()
        self.assertEqual(c.phase, 'BLOCKED')
        self.assertTrue((self.base / 'pending.json').exists())

    def test_repeated_fast_failure_keeps_a_distinct_persistent_result(self):
        c = self.controller()
        for attempt in (1, 2, 3):
            c.action('RECHECK')
            # Worker finishes before the next 100 ms UI STATUS: the result
            # still proves this specific click was handled.
            c.recovery.finish(error='RECOVERY_PI_SSH_FAILED'); c.tick()
            row = c.status()
            self.assertEqual(row['recovery_attempt'], attempt)
            self.assertEqual(row['recovery_result'], 'FAILED')
            self.assertEqual(row['error'], 'RECOVERY_PI_SSH_FAILED')
            self.assertIsNone(c.session)
            self.assertTrue((self.base / 'pending.json').exists())
        restarted = self.controller()
        self.assertEqual(restarted.phase, 'BLOCKED')
        self.assertEqual(restarted.status()['recovery_attempt'], 0)

    def test_worker_start_failure_does_not_trap_close(self):
        class Broken(FakeAudit):
            def start(self): raise RuntimeError('thread failed')
        c = self.controller(Broken); c.action('RECHECK'); c.action('CLOSE')
        self.assertEqual(c.phase, 'BLOCKED')
        self.assertEqual(c.status()['recovery_result'], 'FAILED')
        self.assertTrue(c.status()['close_ready'])
