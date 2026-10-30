"""Pure helper/guard tests; integration runs are separate and isolated."""
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import common
import run_probe


class DiagnosticTests(unittest.TestCase):
    def test_active_robot_and_unknown_snapshot_prevent_diagnostics(self):
        for rc in (0, 2, 255):
            with self.subTest(rc=rc), self.assertRaises(RuntimeError):
                run_probe.require_no_robot(run=lambda *a, **kw: SimpleNamespace(returncode=rc))
        run_probe.require_no_robot(run=lambda *a, **kw: SimpleNamespace(returncode=1))

    def test_failed_worker_start_is_cleaned_up_without_global_signals(self):
        class Child:
            returncode = 2
            def poll(self): return self.returncode
            def wait(self, **kw): return self.returncode
            def send_signal(self, sig): raise AssertionError('Already exited worker must not be signalled')
        with tempfile.TemporaryDirectory() as tmp, patch.object(run_probe, 'require_no_robot'), \
             patch.object(run_probe.subprocess, 'Popen', return_value=Child()) as popen:
            with self.assertRaisesRegex(RuntimeError, 'NOT_READY'):
                run_probe.experiment(Path(tmp), Path(tmp) / 'case', 5, 'callback', 'none', 0)
            self.assertEqual(popen.call_count, 3)
            for call in popen.call_args_list:
                env = call.kwargs['env']
                self.assertEqual(env['ROS_DOMAIN_ID'], '213')
                self.assertEqual(env['ROS_LOCALHOST_ONLY'], '1')
                self.assertEqual(env['RMW_FASTRTPS_USE_QOS_FROM_XML'], '0')

    def test_robot_domain_and_nonlocal_environment_are_refused(self):
        for env in ({}, {'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1'},
                    {'ROS_DOMAIN_ID': '213', 'ROS_LOCALHOST_ONLY': '0'}):
            with self.subTest(env=env), patch.dict(os.environ, env, clear=True):
                with self.assertRaises(ValueError): common.require_isolation()
        with patch.dict(os.environ, ROS_DOMAIN_ID='213', ROS_LOCALHOST_ONLY='1'):
            common.require_isolation()

    def test_packet_and_source_tampering_block_before_ros(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); (root / 'one.py').write_text('original')
            manifest = hashlib.sha256(b'original').hexdigest() + '  one.py\n'
            (root / 'operational_packet.sha256').write_text(manifest)
            with self.assertRaises(ValueError): common.verify_source(root)
            with patch.object(common, 'R5_PACKET', hashlib.sha256(manifest.encode()).hexdigest()):
                self.assertEqual(common.verify_source(root)['pins'], 1)
                (root / 'one.py').write_text('changed')
                with self.assertRaisesRegex(ValueError, 'SOURCE_CHANGED'): common.verify_source(root)

    def test_manifest_cannot_escape_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            parent = Path(tmp); root = parent / 'source'; root.mkdir()
            (parent / 'outside').write_bytes(b'a')
            raw = (hashlib.sha256(b'a').hexdigest() + '  ../outside\n').encode()
            (root / 'operational_packet.sha256').write_bytes(raw)
            with patch.object(common, 'R5_PACKET', hashlib.sha256(raw).hexdigest()):
                with self.assertRaises(ValueError): common.verify_source(root)

    def test_duplicate_manifest_entry_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); (root / 'one').write_bytes(b'a')
            raw = ((hashlib.sha256(b'a').hexdigest() + '  one\n') * 2).encode()
            (root / 'operational_packet.sha256').write_bytes(raw)
            with patch.object(common, 'R5_PACKET', hashlib.sha256(raw).hexdigest()):
                with self.assertRaises(ValueError): common.verify_source(root)

    def test_leading_tf_stamp_is_undone_without_extending_freshness(self):
        self.assertAlmostEqual(common.source_age_ms(1_100_000_000, 1_000_000_000), 0.)
        self.assertAlmostEqual(common.source_age_ms(1_100_000_000, 5_700_000_000), 4700.)

    def test_percentiles_and_nonfinite_values(self):
        self.assertEqual(common.stats([])['count'], 0)
        row = common.stats([4., 1., 3., 2., float('nan'), float('inf')])
        self.assertEqual(row, dict(count=4, p50=2., p95=4., p99=4., maximum=4.))

    def test_trace_capacity_and_no_overwrite(self):
        trace = common.Trace(limit=2)
        for _ in range(3): trace.add('sample')
        self.assertEqual(len(trace.events), 2); self.assertEqual(trace.dropped, 1)
        with tempfile.TemporaryDirectory() as tmp:
            trace.save(tmp, 'test')
            with self.assertRaises(FileExistsError): trace.save(tmp, 'test')

    def test_summary_excludes_discovery_and_does_not_invent_transit_latency(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for role in ('fixture', 'health', 'fast'):
                rows = [dict(event='start', mono_ns=0),
                        dict(event='tf_callback', mono_ns=1_000_000_000, source_age_ms=999.),
                        dict(event='tf_callback', mono_ns=3_000_000_000, source_age_ms=12.)]
                (root / (role + '.json')).write_text(json.dumps(dict(trace_dropped=0, events=rows)))
            row = common.summarize(root)
            self.assertEqual(row['metrics']['fast_callback_source_age_ms']['maximum'], 12.)
            self.assertFalse(row['unique_tf_stamp_assumption'])

    def test_output_inside_r5_or_existing_folder_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for target in (root, root / 'new_inside'):
                args = ['probe', '--source', str(root), '--output', str(target)]
                with patch('sys.argv', args), self.assertRaises(SystemExit): run_probe.main()
            other = root / 'already'; other.mkdir()
            source = root / 'source'; source.mkdir()
            with patch('sys.argv', ['probe', '--source', str(source), '--output', str(other)]), self.assertRaises(SystemExit):
                run_probe.main()

    def test_duration_and_injected_delay_are_bounded(self):
        for extra in (['--duration', '100'], ['--duration', 'nan'], ['--delay-ms', '-1'], ['--delay-ms', '31']):
            with patch('sys.argv', ['probe', '--source', '.', '--output', '/tmp/unused', *extra]), self.assertRaises(SystemExit):
                run_probe.main()


if __name__ == '__main__':
    unittest.main()
