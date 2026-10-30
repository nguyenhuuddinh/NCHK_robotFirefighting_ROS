"""Offline model of the Pi-to-laptop startup and STOP state transitions.

This model performs no process, network, ROS, or file operations. A future
runtime adapter must supply ownership/liveness proof and serialize events;
passing these model tests is not permission to start a physical robot.
"""
from .pairing import PairingError, PairingOutputParser, validate_binding


class SessionError(RuntimeError):
    pass


class SessionModel:
    def __init__(self, packet):
        self.packet = packet
        self.state = "IDLE"
        self.parser = None
        self.announcement = None
        self.selected_map = None
        self.stop_status = "NOT_REQUESTED"
        self.cleanup_status = "NOT_REQUESTED"

    def begin(self, selected_map, *, map_matches, no_other_owned_session):
        if self.state != "IDLE" or not no_other_owned_session or not map_matches:
            raise SessionError("Không thể bắt đầu phiên mới")
        self.selected_map = selected_map
        self.parser = PairingOutputParser()
        self.state = "WAITING_PAIRING"
        return "START_PI"

    def pi_output(self, chunk, *, owned_pi_alive):
        if self.state != "WAITING_PAIRING":
            raise SessionError("Pi không còn thuộc phiên hiện tại")
        if not owned_pi_alive:
            self.state = "STOP_REQUIRED"
            raise SessionError("Pi không còn thuộc phiên hiện tại")
        try:
            announcements = self.parser.feed(chunk)
        except PairingError as exc:
            self.state = "STOP_REQUIRED"
            raise SessionError("Dữ liệu pairing không hợp lệ") from exc
        if announcements:
            self.announcement = announcements[0]
            self.state = "CHECKING_PAIRING"
            return "READ_PAIRING"
        return None

    def pairing_copied(self, raw, *, owned_pi_alive, map_matches):
        if self.state != "CHECKING_PAIRING" or not owned_pi_alive or not map_matches:
            self.state = "STOP_REQUIRED"
            raise SessionError("Phiên Pi hoặc map đã thay đổi")
        try:
            validate_binding(raw, self.announcement, self.packet)
        except PairingError as exc:
            self.state = "STOP_REQUIRED"
            raise SessionError("Pairing không thuộc phiên hiện tại") from exc
        self.state = "STARTING_LAPTOP"
        return "START_LAPTOP"

    def backend_ready(self, *, verified):
        if self.state != "STARTING_LAPTOP" or not verified:
            raise SessionError("Chưa có bằng chứng backend sẵn sàng")
        self.state = "RUNNING"

    def request_stop(self):
        if self.state in ("IDLE", "STOPPED"):
            return None
        if self.state != "STOPPING":
            self.state = "STOPPING"
            self.stop_status = "PENDING"
            self.cleanup_status = "PENDING"
            return "REQUEST_STOP"
        return None

    def finish_stop(self, *, robot_stop_confirmed, cleanup_clean):
        if self.state != "STOPPING":
            raise SessionError("Chưa yêu cầu STOP")
        self.stop_status = "CONFIRMED" if robot_stop_confirmed else "UNCONFIRMED"
        self.cleanup_status = "CLEAN" if cleanup_clean else "DEGRADED"
        # An unconfirmed robot stop blocks the next startup even if processes
        # have exited. Clean process exit is never treated as robot proof.
        self.state = "STOPPED" if robot_stop_confirmed else "STOP_UNCONFIRMED"

