"""Real helper process and private IPC; no ROS or robot connection."""
import ctypes
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from services.operator_bootstrap import create_bootstrap_file
from services.operator_process_channel import OperatorProcessChannel
from services.operator_runner import OperatorBootstrap

FIXTURE = Path(f'/tmp/fire_robot_control_preview_{os.getuid()}/fire_robot_frontend_fixture')


class ProcessChannelTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name)
        self.boot = OperatorBootstrap('8518a120-a012-4db3-bbf3-f73f67aed61e', 'b'*32, 'c'*32, 'a'*64, self.path/'ui.sock')
        self.diagnostic = self.path/'operator_ui.json'
        self.channel = OperatorProcessChannel(self.boot, diagnostic_path=self.diagnostic)
        self.client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.client.connect(str(self.boot.socket_path))
        self.seq = 0

    def tearDown(self):
        self.client.close()
        try:
            self.channel.close()
        finally:
            self.tmp.cleanup()

    def send(self, event, **overrides):
        self.seq += 1
        row = dict(version=1, session=self.boot.session, generation=self.boot.generation, nonce=self.boot.nonce,
                   seq=self.seq, sent_mono=time.monotonic(), event=event)
        row.update(overrides)
        self.client.sendall(json.dumps(row).encode()+b'\n')

    def wait_active(self):
        deadline = time.monotonic()+1.
        while time.monotonic()<deadline:
            self.assertIsNone(self.channel.poll(time.monotonic()))
            if self.channel.lease.active:
                return
            time.sleep(.01)
        self.fail('No authenticated frontend heartbeat')

    def test_q_dominates_x_and_cannot_be_revived(self):
        self.send('HEARTBEAT')
        self.wait_active()
        self.send('X')
        self.send('Q')
        time.sleep(.03)
        self.assertEqual(self.channel.poll(time.monotonic()), 'Q')
        self.send('HEARTBEAT')
        self.send('X')
        self.assertIsNone(self.channel.poll(time.monotonic()))

    def test_x_delivered_once_and_q_has_priority_later(self):
        self.send('HEARTBEAT')
        self.wait_active()
        self.send('X')
        time.sleep(.02)
        self.assertEqual(self.channel.poll(time.monotonic()), 'X')
        self.assertIsNone(self.channel.poll(time.monotonic()))
        self.send('Q')
        time.sleep(.02)
        self.assertEqual(self.channel.poll(time.monotonic()), 'Q')

    def test_frozen_ui_still_expires_at_original_deadline(self):
        self.send('HEARTBEAT')
        self.wait_active()
        time.sleep(.42)
        self.assertEqual(self.channel.poll(time.monotonic()), 'Q')
        self.assertEqual(self.channel.lease.reason, 'FRONTEND_LEASE_EXPIRED')

    def test_disconnect_still_requests_stop(self):
        self.send('HEARTBEAT')
        self.wait_active()
        self.client.close()
        time.sleep(.02)
        self.assertEqual(self.channel.poll(time.monotonic()), 'Q')
        self.assertEqual(self.channel.lease.reason, 'FRONTEND_DISCONNECTED')

    def test_wrong_identity_and_replayed_sequence_cannot_keep_lease_alive(self):
        self.send('HEARTBEAT')
        self.wait_active()
        for _ in range(6):
            self.send('HEARTBEAT', generation='d'*32)
            self.send('HEARTBEAT', seq=1)
            time.sleep(.08)
        self.assertEqual(self.channel.poll(time.monotonic()), 'Q')

    def test_helper_does_not_escape_owned_process_group_or_expose_nonce(self):
        self.assertEqual(os.getpgid(self.channel.process.pid), os.getpgrp())
        command = Path(f'/proc/{self.channel.process.pid}/cmdline').read_bytes()
        self.assertNotIn(self.boot.nonce.encode(), command)
        self.assertNotIn(self.boot.nonce, self.diagnostic.read_text())

    def test_helper_death_is_a_latched_failure(self):
        self.channel.process.terminate()
        self.channel.process.wait(timeout=1.)
        with self.assertRaisesRegex(RuntimeError, 'HELPER_EXITED'):
            self.channel.poll(time.monotonic())

    def test_socket_message_overflow_requests_stop(self):
        self.send('HEARTBEAT')
        self.wait_active()
        self.client.sendall(b'x'*3000)
        time.sleep(.02)
        self.assertEqual(self.channel.poll(time.monotonic()), 'Q')
        self.assertEqual(self.channel.lease.reason, 'FRONTEND_MESSAGE_OVERFLOW')

    def test_stale_or_wrong_generation_ipc_state_fails_closed(self):
        import services.operator_process_channel as transport
        row = dict(event='STATE', id=1, session=self.boot.session, generation=self.boot.generation,
                   sample_mono=time.monotonic()-1., active=True, q=False, x=0, reason='FRONTEND_ACTIVE')
        with patch.object(transport, 'receive', return_value=row), patch.object(transport, 'send'):
            with self.assertRaisesRegex(ValueError, 'STATE_INVALID'):
                self.channel.poll(time.monotonic())
        with self.assertRaises(RuntimeError):
            self.channel.poll(time.monotonic())

    def test_helper_retires_when_its_parent_dies(self):
        directory=self.path/'parent-death'
        directory.mkdir(mode=0o700)
        program='''from pathlib import Path
import sys
from services.operator_runner import OperatorBootstrap
from services.operator_process_channel import OperatorProcessChannel
boot=OperatorBootstrap('8518a120-a012-4db3-bbf3-f73f67aed61e','b'*32,'c'*32,'a'*64,Path(sys.argv[1])/'parent.sock')
channel=OperatorProcessChannel(boot)
print(channel.process.pid,flush=True)
sys.stdin.readline()
channel.close()
'''
        parent=subprocess.Popen([sys.executable,'-c',program,str(directory)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            import select
            self.assertTrue(select.select([parent.stdout],[],[],4.)[0])
            pid=int(parent.stdout.readline())
            parent.kill()
            parent.wait(timeout=1.)
            deadline=time.monotonic()+1.5
            state=None
            while time.monotonic()<deadline:
                try:
                    raw=Path(f'/proc/{pid}/stat').read_text()
                except FileNotFoundError:
                    break
                state=raw[raw.rfind(')')+2:].split()[0]
                if state in ('Z','X'):
                    break
                time.sleep(.01)
            else:
                self.fail('Helper remained live after parent death')
            self.assertFalse((directory/'parent.sock').exists())
        finally:
            if parent.poll() is None:
                parent.terminate()
            parent.communicate(timeout=2.)

    @unittest.skipUnless(FIXTURE.exists(), 'Build the Qt fixture')
    def test_real_qt_heartbeat_survives_native_call_holding_parent_gil(self):
        # Replace the unused test socket with a real Qt sender in another OS
        # process. Native PyDLL sleep holds the parent's GIL for >350ms.
        self.client.close()
        with create_bootstrap_file(self.boot, self.path) as bootstrap:
            child = subprocess.Popen([str(FIXTURE), str(self.boot.socket_path), 'alive', str(bootstrap.fileno())],
                pass_fds=(bootstrap.fileno(),), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            self.wait_active()
            ctypes.PyDLL(None).usleep(550000)
            self.assertIsNone(self.channel.poll(time.monotonic()))
            self.assertEqual(self.channel.lease.reason, 'FRONTEND_ACTIVE')
        finally:
            child.terminate()
            child.communicate(timeout=2.)
