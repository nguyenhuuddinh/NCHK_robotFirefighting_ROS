import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from services.diagnostic_stream import DiagnosticStream
from services.fire_health_entry import instrument
from services.nav_performance import process_stat


class DiagnosticTests(unittest.TestCase):
    def test_stream_is_rate_limited_rotated_and_private(self):
        now = [0.]
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / 'performance.jsonl'
            stream = DiagnosticStream(p, clock=lambda: now[0], limit=30)
            stream.emit({'x': 'one'})
            stream.emit({'x': 'suppressed'})
            self.assertEqual(len(p.read_text().splitlines()), 1)
            now[0] = 1.
            stream.emit({'x': 'two'})
            now[0] = 2.
            stream.emit({'x': 'three'})
            stream.close()
            self.assertTrue(p.with_suffix('.jsonl.1').exists())
            self.assertEqual(p.stat().st_mode & 0o777, 0o600)

    def test_disk_error_disables_diagnostics_without_changing_policy(self):
        with tempfile.TemporaryDirectory() as tmp:
            stream = DiagnosticStream(Path(tmp)/'missing'/'out')
            stream.emit({'sample': True})
            self.assertTrue(stream.disabled)
            stream.emit({'sample': False})

    def test_existing_file_or_symlink_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/'data'
            p.write_text('sentinel')
            link = Path(tmp)/'link'
            link.symlink_to(p)
            for path in (p, link):
                stream = DiagnosticStream(path)
                stream.emit({'x': 1})
                self.assertTrue(stream.disabled)
            self.assertEqual(p.read_text(), 'sentinel')

    def test_health_wrapper_records_same_decision_inputs_without_extra_queries(self):
        rows, calls = [], []
        class Node:
            def __init__(self):
                def report(now, **fields):
                    calls.append(fields)
                    return dict(sample_mono=now, identity_key=['b',1,'e'], checks={'pose': False}, stable_s={})
                self.core = SimpleNamespace(report=report, inputs={'pose': (True, 4.)}, initial_pose_at=3.)
            def sample(self):
                return self.core.report(5., sources={'pose': True}, tf_ok=False, action_ready=True, lifecycle_ok=True)
        stream = SimpleNamespace(due=lambda: True, emit=rows.append)
        cls = instrument(Node, stream, 'session', 'packet')
        with patch('services.fire_health_entry.time.monotonic', side_effect=[5., 5.4]):
            result = cls().sample()
        self.assertEqual(len(calls), 1)
        self.assertEqual(result['checks'], {'pose': False})
        self.assertEqual(rows[0]['localization_blockers'], ['map_tf'])
        self.assertAlmostEqual(rows[0]['policy_sample_age_ms'], 400.)
        self.assertAlmostEqual(rows[0]['sample_ms'], 400.)

    def test_proc_stat_handles_spaces_and_parentheses(self):
        fields = ['S','1','50','50','0','0','0','0','0','0','0','120','30','0','0','0','0','7','0','987','0','512']
        row = process_stat('123 (node (with spaces)) ' + ' '.join(fields))
        self.assertEqual((row['pid'],row['pgid'],row['sid'],row['ticks'],row['threads'],row['birth'],row['rss_pages']),
                         (123,50,50,150,7,987,512))
