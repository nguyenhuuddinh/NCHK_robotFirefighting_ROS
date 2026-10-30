"""Regressions from the real 7ed5b7dc Q/cleanup receipt; no robot I/O."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from services import recovery, stop_recovery_runner as stop
from services import session_outcome

APP = Path(__file__).resolve().parents[1]
RAW = APP / 'docs/qa_evidence/q_cleanup_7ed5b7dc_20261030/raw_laptop'


class QCleanupRegression(unittest.TestCase):
    def setUp(self):
        self.row = json.loads((RAW / 'laptop/receipt.json').read_text())
        self.desktop = json.loads((RAW / 'desktop_receipt.json').read_text())
        self.marker = {k: self.row[k] for k in ('session', 'packet')}

    def fixture(self, folder, row):
        (folder / 'laptop').mkdir()
        (folder / 'laptop/receipt.json').write_text(json.dumps(row))
        (folder / 'desktop_receipt.json').write_text(json.dumps(self.desktop))

    def test_real_confirmed_stop_can_prepare_new_verification_after_absence(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            self.fixture(out, self.row)
            with patch.object(stop.probe, 'absent') as absent:
                binding, pids = stop.old_groups(self.marker, out, return_pids=True)
            self.assertEqual(binding, self.row['remote']['binding'])
            self.assertIn(359312, pids)
            absent.assert_called_once_with(pids)
            self.assertEqual(session_outcome.outcome(self.row, **self.marker), (True, False))
            with self.assertRaisesRegex(ValueError, 'LOCAL_CLEANUP_MISSING'):
                recovery.local_proof(self.marker, out)

    def test_confirmed_case_never_waives_unknown_failures_or_live_pids(self):
        for mutation in ('extra_error', 'operator_rc2', 'other_forced', 'live_group',
                         'wrong_signal', 'wrong_packet', 'missing_child', 'mission_failed', 'mapping'):
            row = copy.deepcopy(self.row)
            local = row['local']
            if mutation == 'extra_error': local['errors'].append('OTHER')
            elif mutation == 'operator_rc2': local['processes']['operator']['leader_rc'] = 2
            elif mutation == 'other_forced': local['processes']['mission']['forced'] = True
            elif mutation == 'live_group': local['processes']['nav']['live_pids'] = [359312]
            elif mutation == 'wrong_signal': local['processes']['nav']['signals'][1]['signal'] = 'SIGKILL'
            elif mutation == 'wrong_packet': row['packet'] = 'b' * 64
            elif mutation == 'missing_child': local['child_log_audit']['roles']['nav']['missing'] = []
            elif mutation == 'mission_failed': local['child_log_audit']['roles']['mission']['clean'] = False
            elif mutation == 'mapping': row['profile'] = 'DESKTOP_MAPPING'
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as tmp:
                out = Path(tmp)
                self.fixture(out, row)
                with patch.object(stop.probe, 'absent'), self.assertRaises(ValueError):
                    stop.old_groups(self.marker, out)

    def test_current_live_child_still_refuses_confirmed_stop_recovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            self.fixture(out, self.row)
            with patch.object(stop.probe, 'absent', side_effect=ValueError('RECOVERY_PROCESS_STILL_LIVE')):
                with self.assertRaisesRegex(ValueError, 'PROCESS_STILL_LIVE'):
                    stop.old_groups(self.marker, out)

    def test_real_cleanup_error_is_not_a_generic_runner_exit(self):
        self.assertEqual(session_outcome.primary_error(RAW / 'laptop', 'RUNNER_EXIT_2'),
                         'NAV2_SHUTDOWN_TIMEOUT')

    def test_mapping_cleanup_is_never_relabelled_as_nav2(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            row = copy.deepcopy(self.row)
            row['profile'] = 'DESKTOP_MAPPING'
            (folder / 'receipt.json').write_text(json.dumps(row))
            self.assertEqual(session_outcome.primary_error(folder, 'RUNNER_EXIT_2'), 'RUNNER_EXIT_2')

    def test_ready_startup_stays_ready_after_teardown_failure(self):
        self.assertEqual(session_outcome.startup_result('READY_GATE2', 'NAV2_SHUTDOWN_TIMEOUT'),
                         dict(stage='READY_GATE2', status='READY'))
        self.assertEqual(session_outcome.startup_result('MAPPING', 'OTHER'),
                         dict(stage='MAPPING', status='READY'))
        self.assertEqual(session_outcome.startup_result('POSE', 'STARTUP_TIMEOUT:POSE')['status'], 'FAILED')
        self.assertEqual(session_outcome.startup_result('POSE', 'USER_OR_FRONTEND_STOP')['status'], 'CANCELLED')
