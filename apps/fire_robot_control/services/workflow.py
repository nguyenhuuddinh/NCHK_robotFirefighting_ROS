"""Offline-tested session transition policy; contains no I/O or robot control.

An eventual adapter must serialize callbacks, own processes and verify each
proof before reporting it here. Tokens bind late callbacks to their session.
Returning an effect is a request, never proof of robot or process state.
"""
from dataclasses import dataclass
from enum import Enum, auto
import secrets

from .pairing import PairingError, PairingOutputParser, validate_binding


class Phase(Enum):
    IDLE = auto()
    STARTING_MAP = auto()
    MAPPING = auto()
    SAVING = auto()
    STARTING_PI = auto()
    READING_PAIRING = auto()
    STARTING_FIRE = auto()
    RUNNING_FIRE = auto()
    STOPPING = auto()
    BLOCKED = auto()


@dataclass(frozen=True)
class Effect:
    kind: str
    token: str
    argument: object = None
    operation: str | None = None


class WorkflowError(ValueError):
    pass


class Workflow:
    """Single-owner state machine, not a runner or safety implementation."""

    def __init__(self, packet):
        if not isinstance(packet, str) or len(packet) != 64 or any(c not in "0123456789abcdef" for c in packet):
            raise ValueError("Invalid packet")
        self.packet = packet
        self.phase = Phase.IDLE
        self.token = None
        self.kind = None
        self.selected_map = None
        self.parser = None
        self.announcement = None
        self.cancel_pending = False
        self.save_operation = None
        self.cancel_operation = None
        self.stop_status = "NOT_REQUESTED"
        self.cleanup_status = "NOT_REQUESTED"

    def _start(self, phase, kind, *, ownership_clear):
        if self.phase is not Phase.IDLE or ownership_clear is not True:
            raise WorkflowError("Phiên khác đang chạy hoặc chưa chứng minh đã dừng")
        self.token = secrets.token_hex(16)
        self.kind = kind
        self.phase = phase
        self.stop_status = "NOT_REQUESTED"
        self.cleanup_status = "NOT_REQUESTED"
        self.cancel_pending = False
        self.save_operation = None
        self.cancel_operation = None

    def _current(self, token, *phases):
        # A stale callback must not mutate the current session.
        return token == self.token and self.phase in phases

    def scan(self, *, ownership_clear):
        self._start(Phase.STARTING_MAP, "mapping", ownership_clear=ownership_clear)
        return Effect("START_MAPPING", self.token)

    def mapping_ready(self, token, *, owned_and_ready):
        if not self._current(token, Phase.STARTING_MAP):
            return None
        if owned_and_ready is not True:
            return self._fail()
        self.phase = Phase.MAPPING
        return None

    def save(self, name):
        if self.phase is not Phase.MAPPING:
            raise WorkflowError("Không thể lưu map lúc này")
        # The saver validates the name and locks out session transitions.
        if not isinstance(name, str) or not name:
            raise WorkflowError("Chưa nhập tên map")
        self.save_operation = secrets.token_hex(16)
        self.phase = Phase.SAVING
        return Effect("SAVE_MAP", self.token, name, self.save_operation)

    def save_finished(self, token, *, operation, record=None, error=None):
        if not self._current(token, Phase.SAVING) or operation != self.save_operation:
            return False
        if (record is None) == (error is None):
            raise WorkflowError("Cần một kết quả lưu rõ ràng")
        self.phase = Phase.MAPPING
        self.save_operation = None
        if error is None:
            self.selected_map = record
            return True
        return False

    def select_map(self, record):
        if self.phase not in (Phase.IDLE, Phase.MAPPING):
            raise WorkflowError("Chưa thể đổi map")
        if record is None:
            raise WorkflowError("Chưa chọn map")
        self.selected_map = record

    def start_fire(self, *, ownership_clear, map_matches):
        if self.selected_map is None or map_matches is not True:
            raise WorkflowError("Map chưa chọn hoặc đã thay đổi")
        self._start(Phase.STARTING_PI, "fire", ownership_clear=ownership_clear)
        self.parser = PairingOutputParser()
        self.announcement = None
        return Effect("START_PI", self.token)

    def pi_output(self, token, chunk, *, owned_pi_alive):
        if not self._current(token, Phase.STARTING_PI, Phase.READING_PAIRING):
            return None
        if owned_pi_alive is not True:
            return self._fail()
        try:
            announcements = self.parser.feed(chunk)
        except (PairingError, TypeError):
            return self._fail()
        if announcements:
            if self.phase is not Phase.STARTING_PI:
                return self._fail()
            self.announcement = announcements[0]
            self.phase = Phase.READING_PAIRING
            return Effect("READ_PAIRING", self.token, self.announcement.pairing_path)
        return None

    def source_lost(self, token):
        """A runner/teleop/SLAM loss or startup timeout needs the stop path."""
        if token != self.token:
            return None
        return self.stop()

    def pairing_ready(self, token, raw, *, owned_pi_alive, map_matches):
        if not self._current(token, Phase.READING_PAIRING):
            return None
        if owned_pi_alive is not True or map_matches is not True:
            return self._fail()
        try:
            validate_binding(raw, self.announcement, self.packet)
        except PairingError:
            return self._fail()
        self.phase = Phase.STARTING_FIRE
        # No pairing secret is returned in an effect or log.
        return Effect("START_LAPTOP", self.token, self.selected_map)

    def fire_ready(self, token, *, verified):
        if not self._current(token, Phase.STARTING_FIRE):
            return None
        if verified is not True:
            return self._fail()
        self.phase = Phase.RUNNING_FIRE
        return None

    def cancel(self):
        if self.phase is not Phase.RUNNING_FIRE or self.cancel_pending:
            return None
        self.cancel_pending = True
        self.cancel_operation = secrets.token_hex(16)
        return Effect("CANCEL_CYCLE", self.token, operation=self.cancel_operation)

    def cancel_finished(self, token, *, operation, verified):
        if (self._current(token, Phase.RUNNING_FIRE) and operation == self.cancel_operation
                and verified is True):
            self.cancel_pending = False
            self.cancel_operation = None

    def _fail(self):
        return self.stop()

    def stop(self):
        if self.phase in (Phase.IDLE, Phase.STOPPING, Phase.BLOCKED):
            return None
        self.phase = Phase.STOPPING
        self.cancel_pending = False
        self.save_operation = None
        self.cancel_operation = None
        self.stop_status = "PENDING"
        self.cleanup_status = "PENDING"
        return Effect("STOP_MAPPING" if self.kind == "mapping" else "REQUEST_STOP", self.token)

    def stop_finished(self, token, *, robot_stop_confirmed, cleanup_clean):
        if not self._current(token, Phase.STOPPING):
            return False
        self.stop_status = "CONFIRMED" if robot_stop_confirmed is True else "UNCONFIRMED"
        self.cleanup_status = "CLEAN" if cleanup_clean is True else "DEGRADED"
        # A future explicit recovery workflow must handle degraded cleanup.
        self.phase = Phase.IDLE if robot_stop_confirmed is True and cleanup_clean is True else Phase.BLOCKED
        return True
