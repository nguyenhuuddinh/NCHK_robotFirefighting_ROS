"""Private local transport for future frontend X/Q and liveness messages.

Nothing is started on import. This transport has not yet been connected to the
R5 operator and must not be interpreted as an operational emergency stop.
"""
from __future__ import annotations

import os
from pathlib import Path
import socket
import stat
import struct

from .frontend_lease import FrontendLease, MAX_MESSAGE

MAX_BUFFER = 8192
MAX_BATCH_MESSAGES = 128


class OperatorChannel:
    def __init__(self, path, lease, *, clock=None):
        if not isinstance(lease, FrontendLease) or lease.uid != os.getuid():
            raise ValueError("Channel requires a lease owned by this user")
        self.path = Path(path)
        parent = self.path.parent
        info = parent.lstat()
        if (not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid()
                or stat.S_IMODE(info.st_mode) & 0o077):
            raise ValueError("Channel directory must be private and owned")
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.client = None
        self.buffer = b""
        self.identity = None
        self.lease = lease
        self.clock = clock
        try:
            # bind fails if any path already exists; never delete an existing
            # file or assume it belongs to this session.
            self.server.bind(str(self.path))
            self.identity = self.path.lstat()
            os.chmod(self.path, 0o600)
            self.server.listen(1)
            self.server.setblocking(False)
        except BaseException:
            self.server.close()
            if self.identity is not None:
                self._unlink_owned()
            raise

    def _unlink_owned(self):
        try:
            info = self.path.lstat()
            if self.identity is not None and (info.st_dev, info.st_ino) == (
                    self.identity.st_dev, self.identity.st_ino) and stat.S_ISSOCK(info.st_mode):
                self.path.unlink()
        except FileNotFoundError:
            pass

    def _drop_client(self):
        if self.client is not None:
            self.client.close()
            self.client = None
        self.buffer = b""

    def poll(self, now):
        """Nonblocking poll; call only from the operator's own event loop."""
        if self.client is None:
            try:
                peer, _ = self.server.accept()
            except BlockingIOError:
                peer = None
            if peer is not None:
                try:
                    creds = peer.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED,
                                            struct.calcsize("3i"))
                    _pid, uid, _gid = struct.unpack("3i", creds)
                    if uid != self.lease.uid:
                        peer.close()
                    else:
                        self.client = peer
                        self.client.setblocking(False)
                except OSError:
                    peer.close()
        if self.client is not None:
            try:
                chunk = self.client.recv(MAX_BUFFER + 1)
            except BlockingIOError:
                chunk = None
            except OSError:
                chunk = b""
            if chunk == b"":
                self._drop_client()
                if self.lease.active:
                    self.lease.request_stop("FRONTEND_DISCONNECTED")
            elif chunk is not None:
                self.buffer += chunk
                # A peer can send after the caller sampled `now` and before
                # recv completed. Sample after receipt in production so such a
                # heartbeat is not incorrectly treated as coming from the future.
                if self.clock is not None:
                    now = max(now, self.clock())
                if len(self.buffer) > MAX_BUFFER:
                    self.lease.request_stop("FRONTEND_MESSAGE_OVERFLOW")
                    self._drop_client()
                # Limit work per tick. Overflow must request Q instead of
                # delivering an earlier X while Q is hidden in a long batch.
                count = 0
                while self.client is not None:
                    if b"\n" not in self.buffer:
                        break
                    count += 1
                    if count > MAX_BATCH_MESSAGES:
                        self.lease.request_stop("FRONTEND_MESSAGE_OVERFLOW")
                        self._drop_client()
                        break
                    line, self.buffer = self.buffer.split(b"\n", 1)
                    if len(line) > MAX_MESSAGE:
                        self.lease.request_stop("FRONTEND_MESSAGE_OVERFLOW")
                        self._drop_client()
                        break
                    self.lease.ingest(line, now, self.lease.uid)
                if self.client is not None and len(self.buffer) > MAX_MESSAGE:
                    self.lease.request_stop("FRONTEND_MESSAGE_OVERFLOW")
                    self._drop_client()
        return self.lease.next_key(now)

    def close(self):
        self._drop_client()
        self.server.close()
        self._unlink_owned()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
