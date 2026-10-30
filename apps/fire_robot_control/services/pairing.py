"""Parse the pairing path from one Pi runner's output without connecting to Pi.

The parser associates PI_PAIRING with LOG_PI from the same stream. It is not a
substitute for validating the private pairing JSON, packet or remote runner.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import re
import uuid


_RUN = re.compile(r"/home/pi/qa_logs/r5_automatic_[0-9]{8}_[0-9]{6}\Z")
MAX_LINE = 8192


class PairingError(ValueError):
    pass


@dataclass(frozen=True)
class PairingAnnouncement:
    log_dir: str
    pairing_path: str


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise PairingError("Duplicate pairing key")
        result[key] = value
    return result


def _invalid_constant(_):
    raise PairingError("Invalid pairing value")


def validate_binding(raw, announcement, expected_packet):
    """Validate copied pairing data; no claim about remote runner liveness.

    Return the binding for the existing runner only. Never print or persist the
    returned secret in UI logs. The existing runner must still authenticate its
    session over its own mailbox, and the caller must protect the copied file.
    """
    if (not isinstance(raw, bytes) or len(raw) > MAX_LINE
            or not isinstance(announcement, PairingAnnouncement)
            or not _RUN.fullmatch(announcement.log_dir)
            or announcement.pairing_path != announcement.log_dir + "/pairing.json"
            or not isinstance(expected_packet, str)
            or re.fullmatch(r"[0-9a-f]{64}", expected_packet) is None):
        raise PairingError("Invalid binding input")
    try:
        value = json.loads(raw, object_pairs_hook=_unique_object, parse_constant=_invalid_constant)
        if not isinstance(value, dict) or set(value) != {
                "session", "secret", "packet", "boot_id", "runner_pid", "runner_start_ticks", "root"}:
            raise PairingError("Invalid binding fields")
        for name in ("session", "boot_id"):
            if not isinstance(value[name], str) or str(uuid.UUID(value[name])) != value[name]:
                raise PairingError("Invalid binding identity")
        if (not isinstance(value["secret"], str)
                or re.fullmatch(r"[0-9a-f]{32}", value["secret"]) is None
                or value["packet"] != expected_packet or value["root"] != announcement.log_dir
                or type(value["runner_pid"]) is not int or value["runner_pid"] <= 1
                or type(value["runner_start_ticks"]) is not int or value["runner_start_ticks"] < 0):
            raise PairingError("Pairing differs from the current session")
        return value
    except (ValueError, UnicodeDecodeError, RecursionError, TypeError, AttributeError) as exc:
        if isinstance(exc, PairingError):
            raise
        raise PairingError("Invalid pairing data") from exc


class PairingOutputParser:
    """Accept complete lines for a single, still-running owned Pi process."""

    def __init__(self):
        self.buffer = b""
        self.log_dir = None
        self.pairing = None
        self.closed = False

    def feed(self, chunk):
        if self.closed:
            raise PairingError("Pi runner already exited")
        if not isinstance(chunk, bytes):
            raise TypeError("Pi output must be bytes")
        self.buffer += chunk
        announcements = []
        while b"\n" in self.buffer:
            raw, self.buffer = self.buffer.split(b"\n", 1)
            if len(raw) > MAX_LINE:
                raise PairingError("Pi output line exceeds size limit")
            try:
                line = raw.rstrip(b"\r").decode("utf-8", errors="strict")
            except UnicodeDecodeError as exc:
                raise PairingError("Pi output is not UTF-8") from exc
            if line.startswith("LOG_PI="):
                value = line.removeprefix("LOG_PI=")
                if not _RUN.fullmatch(value) or self.log_dir is not None:
                    raise PairingError("Invalid or duplicate Pi log directory")
                self.log_dir = value
            elif line.startswith("PI_PAIRING="):
                value = line.removeprefix("PI_PAIRING=")
                if self.log_dir is None or value != self.log_dir + "/pairing.json" or self.pairing:
                    raise PairingError("Pairing is foreign, premature or duplicated")
                self.pairing = PairingAnnouncement(self.log_dir, value)
                announcements.append(self.pairing)
        if len(self.buffer) > MAX_LINE:
            raise PairingError("Pi output line exceeds size limit")
        return announcements

    def close(self):
        """Discard partial late output: process exit is not pairing proof."""
        self.closed = True
        self.buffer = b""
