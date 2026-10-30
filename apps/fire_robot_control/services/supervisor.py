"""Independent desktop supervisor. Opening the app never starts the robot."""
import argparse
from dataclasses import asdict
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
import secrets
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import uuid

from .frontend_lease import FrontendLease, _object, _nonfinite
from .map_catalog import catalog, still_matches
from .map_save import NAME
from .operator_bootstrap import create_bootstrap_file
from .operator_runner import OperatorBootstrap
from .live_session import LiveSession, atomic
from .recovery import RecoveryAudit, archive_verified, failure_message


class ControlProtocol:
    EVENTS = frozenset(("HEARTBEAT", "SCAN", "RECHECK", "SAVE", "SELECT", "FIRE", "STOP", "CLOSE", "AUTH", "KEY", "VIEW_READY", "VIEW_INIT", "VIEW_ASYNC_INIT", "VERIFY_STOP", "RECYCLE_UI"))

    def __init__(self, binding, now):
        self.binding = binding
        self.view_wait_until = None
        self.lease = FrontendLease(binding.session, binding.generation, binding.nonce, os.getuid(), now,
                                   connect_grace_s=3., lease_s=.35)

    def accept(self, raw, now, uid):
        if len(raw) > 4096:
            return None
        try:
            msg = json.loads(raw, object_pairs_hook=_object, parse_constant=_nonfinite)
            if not isinstance(msg, dict) or set(msg) != {"version", "session", "generation", "nonce", "seq", "sent_mono", "event", "argument"}:
                return None
            event, argument = msg["event"], msg.pop("argument")
            if event not in self.EVENTS:
                return None
            # Authenticate each action with the same bound, sequenced UI-loop
            # lease. Application actions never bypass OS peer credentials.
            msg["event"] = "HEARTBEAT"
            if self.view_wait_until is not None:
                if now >= self.view_wait_until or event not in ("HEARTBEAT", "VIEW_READY", "STOP", "CLOSE"):
                    return None
                # Authentication/freshness are unchanged. This bounded bootstrap
                # has no motion authority; the runner awaits VIEW_READY before
                # teleop or any later fire startup effect can run.
                b = self.binding
                auth = FrontendLease(b.session, b.generation, b.nonce, self.lease.uid, now)
                auth.sequence = self.lease.sequence
                auth.last_now = self.lease.last_now
                if not auth.ingest(json.dumps(msg).encode(), now, uid):
                    return None
                if event in ("VIEW_READY", "STOP", "CLOSE"):
                    self.lease = auth
                    self.view_wait_until = None
                else:
                    self.lease.sequence = auth.sequence
                    self.lease.last_now = auth.last_now
            elif not self.lease.ingest(json.dumps(msg).encode(), now, uid):
                return None
            return event, argument
        except (ValueError, UnicodeDecodeError, TypeError, RecursionError):
            return None

    def begin_view_wait(self, deadline, now):
        if not self.lease.active or not now < deadline <= now + 3.1:
            raise ValueError("INVALID_VIEW_BOOTSTRAP_WINDOW")
        self.view_wait_until = deadline

    def expired(self, now):
        if self.view_wait_until is not None and now < self.view_wait_until:
            return False
        return self.lease.next_key(now) == "Q" or self.lease.q_delivered


class SessionController:
    """Serialized UI actions; worker callbacks enter only through inbox."""
    def __init__(self, *, candidate, packet, state_dir, runtime, map_roots, map_root,
                 session_factory=LiveSession, recovery_factory=RecoveryAudit, clock=time.monotonic):
        self.candidate, self.state_dir, self.runtime = map(Path, (candidate, state_dir, runtime))
        self.packet, self.map_roots, self.map_root = packet, map_roots, Path(map_root)
        self.factory, self.clock = session_factory, clock
        self.inbox = queue.Queue()
        self.session = None
        self.recovery = None
        self.recovery_factory = recovery_factory
        self.recovery_cancelled = False
        self.recovery_message = None
        self.recovery_attempt = 0
        self.recovery_result = None
        self.phase = "BLOCKED" if (self.state_dir / "pending.json").exists() else "IDLE"
        self.error = "Phiên trước cần kiểm tra cleanup. Kết nối Pi rồi bấm Kiểm tra lại." if self.phase == "BLOCKED" else None
        self.close_requested = False
        self.selected = None
        self.save_id = None
        self.last_stop = None
        self.stage = self.phase
        self.progress = None
        self.events = []
        self.refresh_maps()

    def refresh_maps(self):
        self.maps, self.invalid_maps = catalog(self.map_roots)

    def clear_pending_after_cleanup(self):
        """A filesystem error must block reuse, never retain a dead worker."""
        try:
            (self.state_dir / "pending.json").unlink(missing_ok=True)
        except OSError as exc:
            self.phase = self.stage = "BLOCKED"
            detail = "Không xóa được file đánh dấu phiên; kiểm tra quyền/ổ đĩa và receipt: " + str(exc)[:200]
            self.error = (self.error + " · " if self.error else "") + detail

    def status(self):
        return dict(event="STATUS", phase=self.phase, stage=self.stage, error=self.error,
            session=self.session.session if self.session else None,
            selected=self.selected.yaml_path if self.selected else None,
            maps=[asdict(m) for m in self.maps], invalid_maps=self.invalid_maps,
            last_stop=self.last_stop,
            recovery_message=self.recovery_message,
            recovery_attempt=self.recovery_attempt, recovery_result=self.recovery_result,
            save_pending=self.save_id is not None,
            progress=self.progress, fault_kind=(self.last_stop or {}).get("fault_kind"),
            close_ready=self.close_requested and self.session is None and self.recovery is None)

    def action(self, event, argument=None):
        try:
            if event == "HEARTBEAT":
                if self.session:
                    self.session.heartbeat(self.clock())
            elif event in ("STOP", "CLOSE"):
                self.close_requested |= event == "CLOSE"
                if self.recovery:
                    self.recovery_cancelled = True
                if self.session:
                    self.phase = "STOPPING"
                    self.session.send("STOP")
            elif event == "RECHECK":
                if self.phase != "BLOCKED" or self.session is not None or self.recovery is not None or self.close_requested:
                    raise ValueError("Chỉ kiểm tra lại khi phiên cũ đang bị khóa và đã kết thúc")
                self.recovery = self.recovery_factory(state_dir=self.state_dir, emit=self.inbox.put)
                self.recovery_cancelled = False
                self.recovery_attempt += 1
                self.recovery_result = "CHECKING"
                self.phase = self.stage = "CHECKING"
                self.error = None
                self.recovery_message = "Đang kiểm tra receipt và tiến trình laptop/Pi; chưa khởi động robot"
                try:
                    self.recovery.start()
                except (OSError, RuntimeError) as exc:
                    self.recovery = None
                    self.phase = self.stage = "BLOCKED"
                    self.recovery_result = "FAILED"
                    self.error = failure_message(exc)
                    self.recovery_message = "Đã kiểm tra nhưng chưa thể mở phiên mới. Có thể bấm Kiểm tra lại sau khi xử lý lỗi."
            elif event == "SELECT":
                if self.phase not in ("IDLE", "MAPPING") or self.save_id:
                    raise ValueError("Chưa thể đổi map khi đang chuyển phiên/lưu")
                record = next((m for m in self.maps if m.yaml_path == argument), None)
                if record is None or not still_matches(record, self.map_roots):
                    raise ValueError("Map không hợp lệ hoặc đã thay đổi")
                self.selected = record
            elif event in ("SCAN", "FIRE", "VERIFY_STOP"):
                verifying = event == "VERIFY_STOP"
                if self.phase != ("BLOCKED" if verifying else "IDLE") or self.session is not None or self.recovery is not None or self.close_requested:
                    raise ValueError("Cần dừng sạch phiên hiện tại trước khi mở phiên mới")
                if not verifying and (self.state_dir / "pending.json").exists():
                    self.phase = self.stage = "BLOCKED"
                    raise ValueError("Có file phiên chưa được kiểm tra; bấm Kiểm tra lại")
                if event == "FIRE" and (self.selected is None or not still_matches(self.selected, self.map_roots)):
                    raise ValueError("Hãy chọn bản đồ hợp lệ trước")
                kind = "verify_stop" if verifying else "mapping" if event == "SCAN" else "fire"
                folder = self.state_dir / ("session_" + uuid.uuid4().hex)
                folder.mkdir(mode=0o700)
                run = Path(tempfile.mkdtemp(prefix="s-", dir=self.runtime))
                session = self.factory(candidate=self.candidate, packet=self.packet, runtime=run,
                    out=folder, map_root=self.map_root, map_roots=self.map_roots, kind=kind,
                    selected_map=self.selected, emit=self.inbox.put,
                    pi_receipt=self.state_dir / "pi_receipts" / (self.packet + ".json"))
                try:
                    if not verifying:
                        atomic(self.state_dir / "pending.json", dict(session=session.session,
                        packet=self.packet, out=str(folder), supervisor_pid=os.getpid()))
                except OSError:
                    # No worker has started. Preserve any partial marker for
                    # inspection, but don't wait for a FINISHED that cannot arrive.
                    self.phase = self.stage = "BLOCKED"
                    raise
                self.session = session
                self.phase = "STARTING"
                self.stage = "PREFLIGHT"
                self.error = None
                self.last_stop = None
                self.progress = None
                self.recovery_message = None
                self.recovery_result = None
                try:
                    self.session.start()
                except (OSError, RuntimeError) as exc:
                    # LiveSession.start only starts its thread; a failed start
                    # has no worker to own or wait for during CLOSE.
                    self.session = None
                    self.phase = self.stage = "IDLE"
                    self.error = str(exc)[:500]
                    if not verifying:
                        self.clear_pending_after_cleanup()
                    else:
                        self.phase = self.stage = "BLOCKED"
                else:
                    self.events.append(dict(event="NEW_SESSION", kind=kind, session=self.session.session))
            elif event == "SAVE":
                if self.phase != "MAPPING" or self.save_id is not None:
                    raise ValueError("Chỉ lưu được khi đang quét; chờ lần lưu hiện tại xong")
                if not isinstance(argument, str) or NAME.fullmatch(argument) is None:
                    raise ValueError("Tên map chỉ dùng chữ, số, dấu _ và - (tối đa 64 ký tự)")
                if any((self.map_root / (argument + suffix)).exists() or (self.map_root / (argument + suffix)).is_symlink() for suffix in (".yaml", ".pgm")):
                    raise ValueError("Tên map đã tồn tại; hãy dùng tên khác")
                self.save_id = uuid.uuid4().hex
                self.session.send("SAVE", (self.save_id, argument))
            elif event in ("VIEW_INIT", "VIEW_ASYNC_INIT"):
                if (self.session is None or self.phase != "STARTING"
                        or self.stage not in ("POSE", "WAIT_SLAM") or argument != self.session.session):
                    raise ValueError("VIEW_INIT_NOT_IN_PREADMISSION")
                if event == "VIEW_ASYNC_INIT":
                    self.session.begin_async_mapping_view()
                    deadline = None
                else:
                    deadline = self.session.begin_view_init()
                self.events.append(dict(event="VIEW_INIT_GRANTED", session=self.session.session,
                                        deadline_mono=deadline))
            elif event in ("AUTH", "KEY", "VIEW_READY"):
                if self.session is not None and self.phase != "STOPPING":
                    if event == "VIEW_READY" and argument == self.session.session:
                        self.session.heartbeat(self.clock())
                    self.session.send(event, argument)
        except (OSError, ValueError, RuntimeError) as exc:
            self.error = str(exc)[:500]
        return self.status()

    def tick(self):
        while True:
            try:
                row = self.inbox.get_nowait()
            except queue.Empty:
                break
            if not isinstance(row, dict):
                continue
            if row.get("event") == "RECOVERY_FINISHED":
                if self.recovery is None or row.get("recovery_id") != self.recovery.id:
                    continue
                recovery_id = self.recovery.id
                self.recovery = None
                self.phase = self.stage = "BLOCKED"
                self.recovery_result = "FAILED"
                self.recovery_message = "Đã kiểm tra nhưng chưa thể mở phiên mới. Có thể bấm Kiểm tra lại sau khi xử lý lỗi."
                if self.recovery_cancelled or self.close_requested:
                    self.recovery_result = "CANCELLED"
                    self.recovery_message = "Kiểm tra đã bị hủy; phiên cũ vẫn bị khóa."
                    self.error = "Đã hủy kiểm tra lại; giữ nguyên khóa phiên cũ"
                elif row.get("error"):
                    self.error = row["error"]
                else:
                    try:
                        archive_verified(self.state_dir, row["result"], recovery_id, now=self.clock)
                    except (OSError, ValueError, KeyError, TypeError) as exc:
                        self.error = failure_message(exc)
                    else:
                        self.phase = self.stage = "IDLE"
                        self.error = None
                        self.last_stop = None
                        self.recovery_result = "SUCCEEDED"
                        self.recovery_message = "Phiên cũ đã thoát sạch. Chọn Quét map hoặc chọn map rồi Khởi động chữa cháy để tạo phiên mới."
                continue
            if self.session is None:
                continue
            if row.get("session", self.session.session) != self.session.session:
                continue
            event = row.get("event")
            if event in ("AUTH_REQUIRED", "AUTH_DONE"):
                # Transient worker prompts need the same session binding as
                # commands. STATUS alone cannot deliver the modal to Qt.
                if (row.get("session") == self.session.session and self.phase in
                        ("STARTING", "MAPPING", "RUNNING") and not self.close_requested):
                    notice = dict(event=event, session=self.session.session)
                    if event == "AUTH_REQUIRED":
                        notice["label"] = "Nhập mật khẩu SSH/sudo của Pi"
                    self.events.append(notice)
            elif event == "OPERATOR":
                # Forward only the current worker's private IPC binding. A
                # stale or stopping session must never connect a new client.
                binding = getattr(self.session, "binding", None)
                if (binding is not None and getattr(self.session, "kind", None) == "fire"
                        and self.phase in ("STARTING", "RUNNING") and not self.close_requested
                        and row.get("session") == self.session.session == binding.session
                        and row.get("generation") == binding.generation
                        and row.get("nonce") == binding.nonce
                        and row.get("socket_path") == str(binding.socket_path)):
                    self.events.append(dict(event="OPERATOR", session=binding.session,
                        generation=binding.generation, nonce=binding.nonce,
                        socket_path=str(binding.socket_path)))
            elif event == "SESSION":
                self.stage = row["stage"]
                self.progress = row.get("startup")
                if self.phase != "STOPPING":
                    self.phase = "MAPPING" if self.stage == "MAPPING" else "RUNNING" if self.stage == "READY_GATE2" else "STARTING"
                if row.get("error"):
                    self.error = row["error"]
            elif event == "SAVED" and row.get("id") == self.save_id:
                self.save_id = None
                self.refresh_maps()
                if row.get("map"):
                    path = row["map"].get("yaml_path")
                    self.selected = next((m for m in self.maps if m.yaml_path == path), self.selected)
                self.error = row.get("error")
            elif event == "FINISHED":
                # The child can exit in the same inbox batch as its prompt.
                # Never open a password modal after its owner has retired.
                self.events = [notice for notice in self.events
                               if notice.get("event") not in ("AUTH_REQUIRED", "OPERATOR")]
                stopped = row.get("robot_stop_confirmed") is True
                clean = row.get("cleanup_clean") is True
                no_session_started = row.get("session_was_started") is False
                self.phase = "IDLE" if clean and (stopped or no_session_started) else "BLOCKED"
                self.stage = self.phase
                self.save_id = None
                self.error = row.get("error")
                self.last_stop = {key: row.get(key) for key in ("robot_stop_confirmed", "cleanup_clean", "session_was_started", "log_dir")}
                from .session_outcome import fault_kind
                self.last_stop["fault_kind"] = fault_kind(stopped, clean, not no_session_started)
                verification = getattr(self.session, "kind", None) == "verify_stop"
                verification_out = self.session.out if verification else None
                # FINISHED is already consumed. Retire the worker even if a
                # marker deletion fails; the marker still blocks the next run.
                self.session = None
                if verification:
                    self.phase = self.stage = "BLOCKED"
                    if clean and stopped and not self.close_requested:
                        self.recovery = self.recovery_factory(state_dir=self.state_dir,
                            emit=self.inbox.put, new_stop_out=verification_out)
                        self.recovery_cancelled = False
                        self.recovery_attempt += 1
                        self.recovery_result = "CHECKING"
                        self.phase = self.stage = "CHECKING"
                        self.recovery_message = "Đã lấy xác nhận dừng mới; đang đối chiếu và lưu bằng chứng"
                        try:
                            self.recovery.start()
                        except (OSError, RuntimeError) as exc:
                            self.recovery = None
                            self.phase = self.stage = "BLOCKED"
                            self.error = failure_message(exc)
                elif self.phase == "IDLE":
                    self.clear_pending_after_cleanup()
                else:
                    kind = self.last_stop["fault_kind"]
                    self.error = (self.error or "") + " · " + str(kind) + ". Log: " + row.get("log_dir", "")
                self.events.append(row)

        events, self.events = self.events, []
        return events


def verify_packet(root):
    from .packet_archive import source_packet
    data, _ = source_packet(root)
    return hashlib.sha256(data).hexdigest()


def main(argv=None, *, session_factory=LiveSession):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--binary-sha256", required=True)
    parser.add_argument("--state-dir", type=Path, required=True)
    parser.add_argument("--map-root", type=Path, required=True)
    parser.add_argument("--maps", nargs="+", type=Path, required=True)
    a = parser.parse_args(argv)
    os.umask(0o077)
    packet = verify_packet(a.candidate)
    if not a.binary.is_file() or hashlib.sha256(a.binary.read_bytes()).hexdigest() != a.binary_sha256:
        raise ValueError("DESKTOP_BINARY_HASH_MISMATCH")
    a.state_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    if a.state_dir.is_symlink() or a.state_dir.stat().st_uid != os.getuid() or a.state_dir.stat().st_mode & 0o077:
        raise ValueError("Private state directory required")
    lock = (a.state_dir / "supervisor.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    a.map_root.mkdir(mode=0o700, parents=True, exist_ok=True)
    # Preserve this directory on abnormal exit for diagnostics; never remove a
    # live socket/PTY just because the frontend window closed.
    runtime = Path(tempfile.mkdtemp(prefix="fire-ui-", dir="/tmp"))
    binding = OperatorBootstrap(str(uuid.uuid4()), secrets.token_hex(16), secrets.token_hex(16), packet, runtime / "control.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(binding.socket_path))
    os.chmod(binding.socket_path, 0o600)
    server.listen(1)
    server.setblocking(False)
    controller = SessionController(candidate=a.candidate, packet=packet, state_dir=a.state_dir,
        runtime=runtime, map_roots=a.maps, map_root=a.map_root, session_factory=session_factory)
    protocol = None
    connect_deadline = time.monotonic() + 20.
    stopping = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.__setitem__(0, True))
    gui_log = (runtime / "gui.log").open("wb")
    print("UI_LOG=" + str(runtime / "gui.log"), flush=True)
    def spawn_gui():
        with create_bootstrap_file(binding, runtime) as boot:
            return subprocess.Popen([str(a.binary), "--control", str(a.candidate / "desktop/native_rviz/operational.rviz"), str(boot.fileno())],
                pass_fds=(boot.fileno(),), close_fds=True, stdin=subprocess.DEVNULL, stdout=gui_log, stderr=subprocess.STDOUT,
                start_new_session=True)
    gui = spawn_gui()
    recycling = False
    recycle_at = None
    peer = None
    buffer = outgoing = b""
    last_sent = 0.
    close_at = None
    uid = None
    try:
        while True:
            now = time.monotonic()
            if peer is None and not controller.close_requested and not recycling:
                try:
                    peer, _ = server.accept()
                    pid, uid, _ = struct.unpack("3i", peer.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
                    if pid != gui.pid or uid != os.getuid():
                        peer.close()
                        peer = None
                    else:
                        peer.setblocking(False)
                        protocol = ControlProtocol(binding, now)
                except BlockingIOError:
                    pass
            if peer is not None:
                try:
                    raw = peer.recv(8192)
                    if not raw:
                        raise ConnectionError("Frontend disconnected")
                    buffer += raw
                    if len(buffer) > 16384:
                        raise ConnectionError("Frontend overflow")
                    count = 0
                    # A batch is bounded. CLOSE/STOP dominate all start/save
                    # actions in the same read, regardless of click order.
                    actions = []
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        action = protocol.accept(line, time.monotonic(), uid)
                        count += 1
                        if action is None or count > 64:
                            raise ConnectionError("Invalid frontend protocol")
                        actions.append(action)
                    stop_actions = [v for v in actions if v[0] in ("STOP", "CLOSE")]
                    for event, argument in stop_actions or actions:
                        if event == "RECYCLE_UI":
                            if (controller.session is None and controller.recovery is None
                                    and controller.phase in ("IDLE", "BLOCKED") and not controller.close_requested):
                                recycling = True
                                recycle_at = time.monotonic()
                                outgoing += (json.dumps(dict(event="RESTART_UI")) + "\n").encode()
                        elif not recycling or event in ("HEARTBEAT", "STOP", "CLOSE"):
                            before_events = len(controller.events)
                            controller.action(event, argument)
                            if (event == "VIEW_INIT" and controller.session is not None
                                    and len(controller.events) > before_events
                                    and controller.events[-1].get("event") == "VIEW_INIT_GRANTED"):
                                deadline = getattr(controller.session, "view_init_until", 0.)
                                if deadline > time.monotonic():
                                    protocol.begin_view_wait(deadline, time.monotonic())
                except BlockingIOError:
                    pass
                except (OSError, ValueError, ConnectionError):
                    peer.close()
                    peer = None
                    if not recycling:
                        controller.action("CLOSE")
            # Messages are validated using a post-recv clock sample. Never
            # immediately poll the lease with the older loop-start sample.
            now = time.monotonic()
            if recycling and recycle_at is not None and now - recycle_at >= 3.:
                controller.error = "FRONTEND_RECYCLE_TIMEOUT"
                controller.action("CLOSE")
                recycling = False
            if recycling and gui.poll() is not None and not controller.close_requested and not stopping[0]:
                code = gui.wait()
                if code == 0:
                    if peer is not None:
                        peer.close()
                    peer = protocol = None
                    buffer = outgoing = b""
                    binding = OperatorBootstrap(str(uuid.uuid4()), secrets.token_hex(16), secrets.token_hex(16),
                        packet, binding.socket_path)
                    gui = spawn_gui()
                    connect_deadline = time.monotonic() + 3.
                    last_sent = 0.
                    recycling = False
                    print("GUI_RESTARTED_AFTER_SESSION", flush=True)
                else:
                    controller.action("CLOSE")
            if stopping[0] or (not recycling and (gui.poll() is not None
                    or (protocol is not None and protocol.expired(now))
                    or (protocol is None and now > connect_deadline))):
                controller.action("CLOSE")
            events = controller.tick()
            if peer is not None and protocol is not None and protocol.lease.active:
                if now - last_sent >= .1 or events:
                    for row in events + [controller.status()]:
                        outgoing += (json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n").encode()
                    last_sent = now
                if len(outgoing) > 1024 * 1024:
                    controller.action("CLOSE")
                    peer.close()
                    peer = None
                elif outgoing:
                    try:
                        sent = peer.send(outgoing)
                        outgoing = outgoing[sent:]
                    except BlockingIOError:
                        pass
                    except OSError:
                        controller.action("CLOSE")
                        peer.close()
                        peer = None
            if controller.close_requested and controller.session is None and controller.recovery is None:
                if gui.poll() is not None:
                    break
                # Let the UI consume the final receipt/status and close on its
                # own; a forced stop remains a bounded last resort.
                if close_at is None:
                    close_at = now
                elif now - close_at >= 2.:
                    gui.send_signal(signal.SIGTERM)
                    break
            time.sleep(.02)
    finally:
        controller.action("CLOSE")
        # Exceptions in IPC cannot skip the worker's local→Pi cleanup.
        while controller.session is not None or controller.recovery is not None:
            controller.tick()
            time.sleep(.05)
        if peer:
            peer.close()
        server.close()
        binding.socket_path.unlink(missing_ok=True)
        if gui.poll() is None:
            gui.terminate()
            try:
                gui.wait(timeout=3)
            except subprocess.TimeoutExpired:
                gui.kill()
                gui.wait(timeout=2)
        gui_log.close()
        lock.close()
    print("GUI_EXIT_CODE=" + str(gui.poll()), flush=True)
    return 0 if controller.phase == "IDLE" and gui.poll() == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
