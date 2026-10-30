"""UI lease reader in a separate process; never imports or publishes ROS.

The helper receives the existing private bootstrap FD. Its private socketpair
answers bounded requests with current state; no heartbeat is synthesized and
no queued state can extend an expired frontend lease.
"""
import argparse
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import time
from types import SimpleNamespace

from .frontend_lease import FrontendLease
from .operator_channel import OperatorChannel
from .recovery_probe import decode

MAX_STATE_BYTES = 2048
IPC_TIMEOUT_S = .35
REASONS = {'WAIT_FRONTEND', 'FRONTEND_ACTIVE', 'FRONTEND_LEASE_EXPIRED',
           'FRONTEND_DISCONNECTED', 'FRONTEND_MESSAGE_OVERFLOW', 'FRONTEND_CLOCK_INVALID', 'OPERATOR_Q'}


class ObservedLease(FrontendLease):
    """Record accepted sender times without logging the authenticated payload."""
    last_sent_mono = None
    max_delivery_delay_s = 0.
    max_poll_gap_s = 0.

    def ingest(self, raw, now, peer_uid):
        accepted = super().ingest(raw, now, peer_uid)
        if accepted:
            message = json.loads(raw)
            if message['event'] == 'HEARTBEAT':
                self.last_sent_mono = message['sent_mono']
                self.max_delivery_delay_s = max(self.max_delivery_delay_s, now-self.last_sent_mono)
        return accepted


def send(sock, value):
    raw = json.dumps(value, allow_nan=False, separators=(',', ':')).encode()
    if len(raw) > MAX_STATE_BYTES or sock.send(raw) != len(raw):
        raise ValueError('OPERATOR_UI_IPC_SEND_FAILED')


def receive(sock, timeout):
    if not select.select([sock], [], [], timeout)[0]:
        raise TimeoutError('OPERATOR_UI_IPC_TIMEOUT')
    raw, _, flags, _ = sock.recvmsg(MAX_STATE_BYTES)
    if not raw or flags & socket.MSG_TRUNC:
        raise ValueError('OPERATOR_UI_IPC_CLOSED_OR_OVERSIZED')
    return decode(raw)


class OperatorProcessChannel:
    def __init__(self, bootstrap, *, diagnostic_path=None, clock=time.monotonic):
        from .operator_bootstrap import create_bootstrap_file
        bootstrap.validate()
        self.bootstrap, self.clock = bootstrap, clock
        self.lease = SimpleNamespace(active=False, lease_s=.35, reason='WAIT_FRONTEND')
        self.request_id = self.x_seen = 0
        self.q_delivered = self.failed = self.closed = False
        self.process = None
        self.socket, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.socket.setblocking(False)
        try:
            with create_bootstrap_file(bootstrap, bootstrap.socket_path.parent) as private:
                argv = [sys.executable, '-B', '-m', 'services.operator_process_channel',
                        '--bootstrap-fd', str(private.fileno()), '--state-fd', str(child.fileno())]
                if diagnostic_path is not None:
                    argv += ['--diagnostic-path', str(diagnostic_path)]
                # Inherit the owned operator's process group, not a detached
                # session. EOF from the parent also retires this helper.
                self.process = subprocess.Popen(argv, pass_fds=(private.fileno(), child.fileno()), close_fds=True)
            child.close()
            ready = receive(self.socket, 3.)
            if ready != dict(event='READY', session=bootstrap.session, generation=bootstrap.generation):
                raise ValueError('OPERATOR_UI_HELPER_NOT_READY')
        except BaseException:
            child.close()
            self.close()
            raise

    def poll(self, _now):
        if self.failed or self.closed or self.process.poll() is not None:
            raise RuntimeError('OPERATOR_UI_HELPER_EXITED')
        self.request_id += 1
        try:
            send(self.socket, dict(event='POLL', id=self.request_id))
            row = receive(self.socket, IPC_TIMEOUT_S)
            now = self.clock()  # after receiving, never before a blocking query
            if (set(row) != {'event', 'id', 'session', 'generation', 'sample_mono', 'active', 'q', 'x', 'reason'}
                    or row['event'] != 'STATE' or type(row['id']) is not int or row['id'] != self.request_id
                    or row['session'] != self.bootstrap.session or row['generation'] != self.bootstrap.generation
                    or type(row['sample_mono']) not in (int, float) or not math.isfinite(row['sample_mono'])
                    or not 0 <= now-row['sample_mono'] <= self.lease.lease_s
                    or type(row['active']) is not bool or type(row['q']) is not bool
                    or type(row['x']) is not int or not self.x_seen <= row['x'] <= 2**53
                    or not isinstance(row['reason'], str) or row['reason'] not in REASONS
                    or row['q'] != (row['reason'] not in {'WAIT_FRONTEND', 'FRONTEND_ACTIVE'})):
                raise ValueError('OPERATOR_UI_IPC_STATE_INVALID')
            self.lease.active, self.lease.reason = row['active'], row['reason']
            if row['q'] and not self.q_delivered:
                self.q_delivered = True
                print('APP_OPERATOR_UI_STOP reason=' + row['reason'], flush=True)
                return 'Q'
            if not self.q_delivered and row['x'] > self.x_seen:
                self.x_seen = row['x']
                return 'X'
            return None
        except BaseException:
            self.failed = True
            raise

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.socket.close()  # helper sees EOF even if it cannot receive a command
        if self.process is not None:
            try:
                rc = self.process.wait(timeout=1.)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=1.)
                raise RuntimeError('OPERATOR_UI_HELPER_FORCED_EXIT')
            if rc != 0:
                raise RuntimeError('OPERATOR_UI_HELPER_UNCLEAN_EXIT')


def helper(bootstrap, state, diagnostic_path=None):
    stop = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))
    state.setblocking(False)
    lease = ObservedLease(bootstrap.session, bootstrap.generation, bootstrap.nonce, os.getuid(), time.monotonic())
    q, x = False, 0
    last_diagnostic = None
    next_diagnostic = 0.
    last_poll = time.monotonic()
    with OperatorChannel(bootstrap.socket_path, lease, clock=time.monotonic) as channel:
        if diagnostic_path is not None:
            from .liveness_diagnostic import write_diagnostic
            write_diagnostic(diagnostic_path, bootstrap, lease, time.monotonic())
            last_diagnostic, next_diagnostic = lease.reason, time.monotonic()+1.
        send(state, dict(event='READY', session=bootstrap.session, generation=bootstrap.generation))
        while not stop[0]:
            poll_at = time.monotonic()
            lease.max_poll_gap_s = max(lease.max_poll_gap_s, poll_at-last_poll)
            last_poll = poll_at
            key = channel.poll(poll_at)
            q = q or key == 'Q'
            if key == 'X' and not q:
                x += 1
            now = time.monotonic()
            if diagnostic_path is not None and (lease.reason != last_diagnostic or now >= next_diagnostic):
                from .liveness_diagnostic import write_diagnostic
                write_diagnostic(diagnostic_path, bootstrap, lease, now)
                last_diagnostic, next_diagnostic = lease.reason, now+1.
            if select.select([state], [], [], .01)[0]:
                try:
                    request = receive(state, 0.)
                except ValueError as exc:
                    if str(exc) == 'OPERATOR_UI_IPC_CLOSED_OR_OVERSIZED':
                        break
                    raise
                if (set(request) != {'event', 'id'} or request['event'] != 'POLL'
                        or type(request['id']) is not int or request['id'] <= 0):
                    raise ValueError('OPERATOR_UI_IPC_REQUEST_INVALID')
                # Observe liveness again after waiting for the parent's request.
                key = channel.poll(time.monotonic())
                q = q or key == 'Q'
                if key == 'X' and not q:
                    x += 1
                send(state, dict(event='STATE', id=request['id'], session=bootstrap.session,
                    generation=bootstrap.generation, sample_mono=time.monotonic(), active=lease.active,
                    q=q, x=x, reason=lease.reason))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bootstrap-fd', type=int, required=True)
    parser.add_argument('--state-fd', type=int, required=True)
    parser.add_argument('--diagnostic-path', type=Path)
    args = parser.parse_args()
    from .operator_bootstrap import read_bootstrap_fd
    bootstrap = read_bootstrap_fd(args.bootstrap_fd)
    os.close(args.bootstrap_fd)
    with socket.socket(fileno=args.state_fd) as state:
        helper(bootstrap, state, args.diagnostic_path)


if __name__ == '__main__':
    main()
