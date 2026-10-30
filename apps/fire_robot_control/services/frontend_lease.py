"""Pure validation for a future local frontend/operator control channel.

This class performs no I/O and does not connect to the robot. The actual IPC
transport and operator integration must be reviewed and tested before live use.
"""
from __future__ import annotations

import hmac
import json
import math


MAX_MESSAGE = 2048


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate key")
        result[key] = value
    return result


def _nonfinite(_):
    raise ValueError("non-finite number")


def _valid_time(value):
    return type(value) in (int, float) and math.isfinite(value)


class FrontendLease:
    """Allow X/Q from one frontend generation and fail closed on UI loss.

    Transport must independently obtain the OS peer UID; it must not trust a
    UID supplied inside JSON. The frontend must send HEARTBEAT from its actual
    UI event loop rather than a background thread.
    """

    def __init__(self, session, generation, nonce, uid, now,
                 *, connect_grace_s=3.0, lease_s=0.35):
        if (not all(isinstance(x, str) and x for x in (session, generation, nonce))
                or type(uid) is not int or uid < 0 or not _valid_time(now)
                or not _valid_time(connect_grace_s) or not _valid_time(lease_s)
                or not 0 < connect_grace_s <= 3.0 or not 0 < lease_s <= 0.35):
            raise ValueError("invalid frontend lease configuration")
        self.session, self.generation, self.nonce = session, generation, nonce
        self.uid = uid
        self.connect_deadline = now + connect_grace_s
        self.lease_s = lease_s
        self.last_now = now
        self.last_heartbeat = None
        self.sequence = -1
        self.active = False
        self.pending_x = False
        self.pending_q = False
        self.q_delivered = False
        self.reason = "WAIT_FRONTEND"

    def _advance(self, now):
        if not _valid_time(now) or now < self.last_now:
            self.pending_q = True
            self.pending_x = False
            self.reason = "FRONTEND_CLOCK_INVALID"
            return False
        self.last_now = now
        return True

    def request_stop(self, reason):
        """Fail closed on a local transport failure or explicit UI loss."""
        if not self.q_delivered:
            self.pending_q = True
            self.pending_x = False
            self.reason = reason

    def ingest(self, raw, now, peer_uid):
        """Validate one message; return whether it was accepted."""
        if not self._advance(now) or self.pending_q or self.q_delivered:
            return False
        # Lease cannot be revived by a late heartbeat before the next poll.
        if ((self.active and now - self.last_heartbeat > self.lease_s)
                or (not self.active and now > self.connect_deadline)):
            self.pending_q = True
            self.pending_x = False
            self.reason = "FRONTEND_LEASE_EXPIRED"
            return False
        if type(peer_uid) is not int or peer_uid != self.uid or not isinstance(raw, bytes) or len(raw) > MAX_MESSAGE:
            return False
        try:
            msg = json.loads(raw, object_pairs_hook=_object, parse_constant=_nonfinite)
        except (ValueError, UnicodeDecodeError, RecursionError):
            return False
        if not isinstance(msg, dict) or set(msg) != {
                "version", "session", "generation", "nonce", "seq", "sent_mono", "event"}:
            return False
        try:
            nonce_ok = isinstance(msg["nonce"], str) and hmac.compare_digest(
                msg["nonce"].encode("utf-8"), self.nonce.encode("utf-8"))
        except UnicodeEncodeError:
            nonce_ok = False
        if (type(msg["version"]) is not int or msg["version"] != 1
                or msg["session"] != self.session or msg["generation"] != self.generation
                or not nonce_ok
                or type(msg["seq"]) is not int or not self.sequence < msg["seq"] <= 2**53
                or not _valid_time(msg["sent_mono"]) or not 0 <= now - msg["sent_mono"] <= self.lease_s
                or msg["event"] not in ("HEARTBEAT", "X", "Q")):
            return False
        if not self.active and msg["event"] != "HEARTBEAT":
            return False
        self.sequence = msg["seq"]
        if msg["event"] == "HEARTBEAT":
            self.active = True
            self.last_heartbeat = now
            self.reason = "FRONTEND_ACTIVE"
        elif msg["event"] == "Q":
            self.pending_q = True
            self.pending_x = False
            self.reason = "OPERATOR_Q"
        elif msg["event"] == "X":
            self.pending_x = True
        return True

    def next_key(self, now):
        """Return Q before X, at most once, when polled by the operator loop."""
        self._advance(now)
        if not self.q_delivered and not self.pending_q:
            if ((not self.active and now > self.connect_deadline)
                    or (self.active and now - self.last_heartbeat > self.lease_s)):
                self.pending_q = True
                self.pending_x = False
                self.reason = "FRONTEND_LEASE_EXPIRED"
        if self.pending_q and not self.q_delivered:
            self.q_delivered = True
            return "Q"
        if not self.q_delivered and self.pending_x:
            self.pending_x = False
            return "X"
        return None
