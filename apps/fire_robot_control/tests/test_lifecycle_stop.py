"""STOP causes and terminal hangup recovery; no robot or credentials."""
import json
from pathlib import Path
import signal
import tempfile
import unittest
from unittest.mock import patch

from services.app_pi_lifecycle import AppStopSignal
from services.live_session import LiveSession


class LifecycleStopTests(unittest.TestCase):
    def worker(self, folder, now):
        return LiveSession(candidate=folder,packet='a'*64,runtime=folder,out=folder,
            map_root=folder,map_roots=[folder],kind='verify_stop',selected_map=None,
            emit=lambda _:None,clock=lambda:now[0])

    def test_first_stop_cause_survives_later_stop_and_queue_arguments_not_logged(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);now=[10.];w=self.worker(p,now)
            w.send('STOP','untrusted-input-not-for-log')
            now[0]=11.;w._request_stop('PI_RUNNER_EXITED_DURING_SESSION')
            row=json.loads((p/'stop_request.json').read_text())
            self.assertTrue(w.stop_event.is_set())
            self.assertEqual(row['cause'],'FRONTEND_STOP_REQUEST')
            self.assertNotIn('untrusted',json.dumps(row))
            self.assertEqual(row['sample_mono'],10.)

    def test_heartbeat_timeout_keeps_exact_threshold_and_reports_age(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);now=[10.];w=self.worker(p,now)
            now[0]=10.75;w._pump();self.assertFalse(w.stop_event.is_set())
            now[0]=10.751;w._pump()
            self.assertTrue(w.stop_event.is_set())
            row=json.loads((p/'stop_request.json').read_text())
            self.assertEqual(row['cause'],'FRONTEND_HEARTBEAT_TIMEOUT')
            self.assertAlmostEqual(row['heartbeat_age_s'],.751)

    def test_diagnostic_write_failure_cannot_skip_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);w=self.worker(p,[10.])
            with patch('services.live_session.atomic',side_effect=OSError('fixture full disk')):
                w.send('STOP')
            self.assertTrue(w.stop_event.is_set())
            self.assertTrue(w.stop_diagnostic_error)

    def test_signal_handler_only_latches_first_reason_and_requests_existing_stop_path(self):
        stop=[False];handler=AppStopSignal(stop)
        handler(signal.SIGHUP,None);handler(signal.SIGINT,None)
        self.assertTrue(stop[0]);self.assertEqual(handler.reason,'SIGHUP')
