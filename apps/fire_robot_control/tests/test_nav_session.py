"""Real process/signals for the app Nav2 owner; no ROS or network."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from services import nav_session

APP = Path(__file__).resolve().parents[1]
CHILD = """
import os,signal,time
signal.signal(signal.SIGINT, lambda *_: exit(0))
print('CHILD_READY pgid='+str(os.getpgrp()), flush=True)
while True: time.sleep(.01)
"""


class NavSessionTests(unittest.TestCase):
    def run_case(self, *, helper='raise SystemExit(0)', child=CHILD, repeat=False, terminate=False):
        with tempfile.TemporaryDirectory() as tmp:
            logpath = Path(tmp) / 'output'
            driver = ('from services import nav_session as n\n'
                      'n.RPC_BUDGET=.15; n.LAUNCH_INT_BUDGET=.15; n.LAUNCH_TERM_BUDGET=.15\n'
                      'raise SystemExit(n.supervise(' + repr([sys.executable, '-c', child]) +
                      ', rpc_command=' + repr([sys.executable, '-c', helper]) + '))\n')
            with logpath.open('wb') as log:
                proc = subprocess.Popen([sys.executable, '-B', '-c', driver], stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True,
                    env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONPATH=str(APP)))
                try:
                    until = time.monotonic() + 3.
                    while b'CHILD_READY' not in logpath.read_bytes() and time.monotonic() < until:
                        self.assertIsNone(proc.poll(), logpath.read_text())
                        time.sleep(.01)
                    text = logpath.read_text()
                    self.assertIn('CHILD_READY pgid=' + str(proc.pid), text)
                    self.assertNotIn('LIFECYCLE_BEGIN', text)
                    proc.send_signal(signal.SIGINT)
                    if repeat:
                        time.sleep(.05)
                        proc.send_signal(signal.SIGINT)
                    if terminate:
                        time.sleep(.05)
                        os.killpg(proc.pid, signal.SIGTERM)
                    rc = proc.wait(timeout=4.)
                    text = logpath.read_text()
                    return rc, text
                finally:
                    # Only the group created by this test; never name matching.
                    try: os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError: pass
                    proc.wait(timeout=2.)

    def test_shutdown_helper_starts_only_after_q_and_repeated_q_is_latched(self):
        rc, text = self.run_case(helper='import time;time.sleep(.09)', repeat=True)
        self.assertEqual(rc, 0, text)
        self.assertEqual(text.count('LIFECYCLE_BEGIN'), 1)
        self.assertIn('LAUNCH_SIGINT', text)

    def test_unavailable_service_falls_back_to_launch_and_still_requires_its_exit(self):
        rc, text = self.run_case(helper='raise SystemExit(2)')
        self.assertEqual(rc, 0, text)
        self.assertIn('"rc": 2', text)

    def test_stuck_native_rpc_is_bounded_and_does_not_block_q_forever(self):
        rc, text = self.run_case(helper='import time;time.sleep(60)')
        self.assertEqual(rc, 0, text)
        self.assertIn('LIFECYCLE_HELPER_TIMEOUT', text)

    def test_launch_ignoring_int_and_term_cannot_return_clean(self):
        child = CHILD.replace('lambda *_: exit(0)', 'signal.SIG_IGN').replace(
            "print('CHILD_READY", "signal.signal(signal.SIGTERM, signal.SIG_IGN)\nprint('CHILD_READY")
        rc, text = self.run_case(child=child)
        self.assertEqual(rc, 2, text)
        self.assertIn('LAUNCH_STILL_LIVE', text)

    def test_outer_terminate_does_not_become_a_successful_cleanup(self):
        rc, text = self.run_case(helper='import time;time.sleep(60)', terminate=True)
        self.assertNotEqual(rc, 0, text)
        self.assertIn('OUTER_TERMINATE', text)

    def test_cli_cannot_wrap_arbitrary_commands(self):
        with self.assertRaisesRegex(ValueError, 'COMMAND_INVALID'):
            nav_session.supervise(['/bin/true'])
