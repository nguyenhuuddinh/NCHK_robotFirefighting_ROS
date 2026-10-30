"""Real process adapter. Constructed only after an explicit Scan/Start click.

One worker owns Pi PTY, private pairing, laptop runner and receipts. UI loss
requests STOP but never closes Pi's controlling terminal before laptop drain.
"""
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import secrets
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid

from .map_catalog import still_matches
from .map_snapshot import snapshot_map
from .pi_receipt import validate_pi_receipt
from .operator_bootstrap import create_bootstrap_file
from .operator_runner import OperatorBootstrap
from .pairing import PairingOutputParser, validate_binding
from .pairing_transfer import PairingTransfer
from .pty_child import PtyChild


def atomic(path, value):
    temp = path.with_suffix(path.suffix + ".tmp")
    fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "w") as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False)
    os.replace(temp, path)


def preflight_message(detail):
    explanations = {
        "LAN_MUST_BE_DISCONNECTED_FOR_WIFI_PROFILE": "Hãy rút cáp LAN và kết nối WiFi robot trước khi chạy phiên",
        "WIFI_ADDRESS_OR_ROUTE_MISMATCH": "Kết nối WiFi robot chưa đúng địa chỉ hoặc tuyến mạng",
        "WIFI_ROUTE_NOT_AVAILABLE": "Chưa kiểm tra được tuyến mạng WiFi tới Pi",
        "EXISTING_ROBOT_OR_UNKNOWN_PROCESS_SNAPSHOT": "Đang có tiến trình robot khác hoặc không kiểm tra được tiến trình",
        "EXISTING_LAPTOP_SESSION_LOCKED": "Một phiên R5/app khác đang giữ quyền vận hành; hãy kết thúc phiên đó trước",
        "UNREVIEWED_ROS_DOMAIN_OR_LOCALHOST": "Thiết lập ROS domain/localhost không khớp cấu hình vận hành",
        "LAPTOP_STATIC_CHECK_FAILED": "Bản cài laptop chưa khớp phiên bản ứng viên",
        "CANDIDATE_PACKET_CHANGED": "Mã nguồn ứng viên đã thay đổi; cần build lại gói",
    }
    for code, message in explanations.items():
        if code in detail:
            return message + " (" + detail[:220] + ")"
    return detail[:300]


def receipt_result(row, session, packet, *, remote_binding=None):
    from .session_outcome import outcome
    return outcome(row, session, packet, remote_binding=remote_binding)


class LiveSession:
    def __init__(self, *, candidate, packet, runtime, out, map_root, map_roots,
                 kind, selected_map, emit, pi_receipt=None, clock=time.monotonic):
        self.candidate, self.runtime, self.out = map(Path, (candidate, runtime, out))
        self.packet, self.kind, self.selected_map = packet, kind, selected_map
        self.pi_receipt = Path(pi_receipt) if pi_receipt is not None else None
        self.map_root, self.map_roots = Path(map_root), map_roots
        self.emit, self.clock = emit, clock
        self.session = str(uuid.uuid4())
        self.binding = OperatorBootstrap(self.session, secrets.token_hex(16), secrets.token_hex(16),
            packet, self.runtime / ("mapping.sock" if kind == "mapping" else "operator.sock"))
        self.stop_event = threading.Event()
        self.stop_reason = None
        self.stop_diagnostic_error = False
        self.commands = queue.Queue(maxsize=64)
        self.pi = self.transfer = self.local = None
        self.key_socket = None
        self.remote_binding = None
        self.pi_birth = None
        self.auth_child = None
        self.prompt = b""
        self.operator_announced = False
        self.stage = "PREFLIGHT"
        self.last_startup_stage = "PREFLIGHT"
        self.recovery_marker_sha256 = None
        self.recovery_boot_id = None
        self.view_ready = False
        self.view_init_until = 0.
        self.view_init_requested = False
        self.finished = False
        self.status_lock = threading.Lock()
        self.last_ui_heartbeat = self.clock()
        self.env = dict(os.environ)
        self.env["PYTHONDONTWRITEBYTECODE"] = "1"
        self.env["PYTHONPATH"] = os.pathsep.join((str(self.candidate / "desktop"), str(self.candidate / "scripts"), self.env.get("PYTHONPATH", "")))
        self.thread = threading.Thread(target=self.run, name="owned-robot-session", daemon=False)

    def begin_view_init(self):
        with self.status_lock:
            if (self.view_ready or self.view_init_requested or self.kind not in ("fire", "mapping")
                    or self.stage not in ("POSE", "WAIT_SLAM")):
                raise ValueError("VIEW_INIT_NOT_IN_PREADMISSION")
            self.view_init_requested = True
            self.view_init_until = self.clock() + 3.
            return self.view_init_until

    def begin_async_mapping_view(self):
        # The UI event loop remains live while a separate RViz process loads.
        # Keep the ordinary frontend and worker heartbeat deadlines in force.
        with self.status_lock:
            if (self.view_ready or self.view_init_requested or self.kind != "mapping"
                    or self.stage != "WAIT_SLAM"):
                raise ValueError("VIEW_INIT_NOT_IN_PREADMISSION")
            self.view_init_requested = True

    def start(self): self.thread.start()

    def heartbeat(self, at):
        with self.status_lock:
            self.last_ui_heartbeat = at

    def send(self, kind, argument=None):
        if kind == "STOP":
            self._request_stop("FRONTEND_STOP_REQUEST")
            return
        try:
            self.commands.put_nowait((kind, argument))
        except queue.Full:
            self._request_stop("COMMAND_QUEUE_FULL")

    def _request_stop(self, reason):
        self.stop_event.set()
        with self.status_lock:
            if self.stop_reason is not None:
                return
            self.stop_reason = reason
            ui_at = self.last_ui_heartbeat
        now = self.clock()
        try:
            atomic(self.out / "stop_request.json", dict(session=self.session,
                cause=reason, stage=self.stage, sample_mono=now,
                last_ui_heartbeat=ui_at, heartbeat_age_s=now-ui_at))
        except OSError:
            # Diagnostics must never prevent STOP or teardown.
            self.stop_diagnostic_error = True

    def _status(self, stage, **extra):
        self.stage = stage
        if stage != "STOPPING":
            self.last_startup_stage = stage
        self.emit(dict(event="SESSION", session=self.session, stage=stage, **extra))

    def _owned_pi(self):
        if self.pi is None or self.pi.poll() is not None:
            raise RuntimeError("PI_RUNNER_EXITED")
        from operational_owned_process import Backend
        rows = Backend().snapshot(self.pi.process.pid)
        leader = next((r for r in rows if r["pid"] == self.pi.process.pid and r["state"] not in ("Z", "X")), None)
        if leader is None or (self.pi_birth is not None and leader["start_ticks"] != self.pi_birth):
            raise RuntimeError("PI_SSH_OWNERSHIP_LOST")
        self.pi_birth = leader["start_ticks"]

    def _runner_status(self):
        path = self.out / "laptop" / ("startup.json" if self.kind == "fire" else "mapping.json")
        try:
            row = json.loads(path.read_text())
            if isinstance(row, dict) and row.get("session") == self.session:
                return row
        except (OSError, ValueError):
            pass
        return None

    def _pi_exit_is_drain(self, startup):
        # The laptop owns Pi STOP as the final cleanup step. SSH can exit
        # before its DONE response/receipt has reached the laptop. This only
        # avoids an incorrect startup error; final STOP/group/binding proofs
        # and both process exit codes are still mandatory below.
        return (self.pi.poll() == 0 and isinstance(startup, dict)
                and startup.get("session") == self.session
                and startup.get("stage") == "STOPPING")

    def _pump(self):
        with self.status_lock:
            ui_at = self.last_ui_heartbeat
        if self.clock() - ui_at > .75 and self.clock() >= self.view_init_until:
            self._request_stop("FRONTEND_HEARTBEAT_TIMEOUT")
        atomic(self.runtime / "display.json", dict(session=self.session, sample_mono=ui_at, view_ready=self.view_ready, view_initializing_until=self.view_init_until))
        while True:
            try:
                kind, arg = self.commands.get_nowait()
            except queue.Empty:
                break
            if self.stop_event.is_set():
                continue
            if kind == "AUTH" and self.auth_child is not None:
                if not isinstance(arg, str) or len(arg) > 512 or any(c in arg for c in "\n\r\0"):
                    raise ValueError("INVALID_AUTH_INPUT")
                data = arg.encode("utf-8") + b"\n"
                if self.auth_child.write(data) != len(data):
                    raise RuntimeError("AUTH_WRITE_INCOMPLETE")
                self.auth_child = None
                self.emit(dict(event="AUTH_DONE", session=self.session))
            elif kind == "VIEW_READY" and arg == self.session:
                self.view_ready = True
                self.view_init_until = 0.
            elif kind == "KEY" and self.kind == "mapping" and self.key_socket is not None:
                from .mapping_runner import KEYS
                if isinstance(arg, str) and len(arg) == 1 and arg in KEYS:
                    self.key_socket.sendall((json.dumps(dict(session=self.session, key=arg)) + "\n").encode())
            elif kind == "SAVE" and self.kind == "mapping" and self.stage == "MAPPING":
                atomic(self.out / "laptop/map_request.json", dict(session=self.session, id=arg[0], name=arg[1]))

    def _read_pty(self, child):
        raw = child.read()
        if raw:
            # Never write SSH/scp/sudo raw output to disk, UI or exception text.
            # Only the strict pairing parser consumes the Pi marker stream.
            self.prompt = (self.prompt + raw)[-2048:]
            tail = self.prompt.lower().rstrip()
            if (b"password" in tail or b"passphrase" in tail) and tail.endswith(b":"):
                if self.auth_child is not child:
                    self.auth_child = child
                    self.emit(dict(event="AUTH_REQUIRED", session=self.session,
                                   label="Nhập mật khẩu SSH/sudo của Pi"))
                self.prompt = b""
        return raw

    def _wait(self, predicate, seconds):
        deadline = self.clock() + seconds
        while True:
            self._pump()
            if self.stop_event.is_set():
                raise InterruptedError("USER_OR_FRONTEND_STOP")
            self._owned_pi()
            if predicate():
                if self.stop_event.is_set():
                    raise InterruptedError("USER_OR_FRONTEND_STOP")
                return
            if self.clock() >= deadline:
                raise TimeoutError("SESSION_STAGE_TIMEOUT:" + self.stage)
            time.sleep(.03)

    def _launch_pi(self):
        remote = "/home/pi/fire_robot_desktop/" + self.packet[:12] + "/ros"
        script = ("umask 077\nexport PYTHONDONTWRITEBYTECODE=1\nsource /opt/ros/humble/setup.bash &&\n"
            "source /home/pi/NCHK_robotFirefighting_ROS_baseline/ros2_ws/install/setup.bash &&\n"
            "source " + shlex.quote(remote + "/ros2_ws/install/local_setup.bash") + " &&\n"
            "export PYTHONPATH=" + shlex.quote(remote + "/desktop:" + remote + "/scripts") + ":\"${PYTHONPATH:-}\" &&\n" +
            "R5_PI_OUT=\"/home/pi/qa_logs/r5_automatic_$(date +%Y%m%d_%H%M%S)\" &&\n"
            "printf 'LOG_PI=%s\\n' \"$R5_PI_OUT\" &&\npython3 -B " + shlex.quote(remote + "/desktop/generated/app_pi_entry.py") +
            " --run-observe --out \"$R5_PI_OUT\"")
        self.pi = PtyChild(["ssh", "-tt", "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=8",
            "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2", "pi@10.0.0.1",
            "bash -lc " + shlex.quote(script)])
        self._owned_pi()
        return remote

    def _spawn_laptop(self, remote, pairing_path):
        from operational_owned_process import Backend, OwnedProcess
        outer = self
        with create_bootstrap_file(self.binding, self.runtime) as bootstrap:
            fd = bootstrap.fileno()
            class FdBackend(Backend):
                def start(self, argv, output):
                    return subprocess.Popen(argv, cwd=outer.candidate / "desktop", env=outer.env,
                        stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                        start_new_session=True, pass_fds=(fd,), close_fds=True)
            self.local = OwnedProcess("desktop_laptop_runner", backend=FdBackend(), int_grace=120., term_grace=10.)
            argv = [sys.executable, "-B"]
            if self.kind == "fire":
                if not still_matches(self.selected_map, self.map_roots):
                    raise ValueError("SELECTED_MAP_CHANGED")
                selected_path = snapshot_map(self.selected_map, self.map_roots, self.out / "selected_map")
                argv += [str(self.candidate / "desktop/generated/app_fire_entry.py"), "--desktop-fd", str(fd),
                         "--run-automatic", "--automatic-token", "R5_AUTOMATIC_FIRE_LIVE",
                         "--map", str(selected_path)]
            else:
                argv += ["-m", "services.mapping_runner", "--bootstrap-fd", str(fd), "--map-root", str(self.map_root)]
            if self.kind == "verify_stop":
                argv += ["--verify-stop"]
            argv += ["--pairing", str(pairing_path), "--remote-script", remote + "/scripts/operational_peer.py",
                     "--out", str(self.out / "laptop")]
            fd_log = os.open(self.out / "laptop_runner.log", os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            self.runner_log = os.fdopen(fd_log, "wb", buffering=0)
            self._record_ownership('LAPTOP_SPAWNING')
            self.local.start(argv, self.runner_log)
            self._record_ownership('LAPTOP_STARTED', self.local.process.pid)

    def _record_ownership(self, stage, pid=None):
        from .recovery_ownership import public_binding, process_identity
        atomic(self.out / 'recovery_ownership.json', dict(schema_version=1,
            session=self.session, packet=self.packet, kind=self.kind, stage=stage,
            pi=public_binding(self.remote_binding, self.packet),
            laptop_runner=process_identity(pid) if pid is not None else None))

    def run(self):
        stopped = clean = False
        error = None
        transport_ok = True
        preflight_only = True
        try:
            from .app_environment import record
            record(self.out, self.candidate, "laptop", atomic)
            if self.kind == "verify_stop":
                from .stop_recovery_runner import prepare
                self._status("CHECKING_OLD_SESSION")
                prepared = prepare(self.out.parent)
                self.recovery_marker_sha256 = prepared["marker_sha256"]
                self.recovery_boot_id = prepared["pi"]["current_boot_id"]
            result = subprocess.run([sys.executable, "-m", "services.live_preflight", str(self.candidate), self.packet, self.kind],
                env=self.env, cwd=self.candidate / "desktop", text=True, capture_output=True, timeout=15)
            if result.returncode:
                # A local, non-auth diagnostic. Bound it and never include the
                # complete environment or any private bootstrap in the UI.
                detail = result.stderr.strip().splitlines()[-1:] or ["Local preflight failed"]
                raise RuntimeError(preflight_message(detail[0]))
            if self.pi_receipt is None:
                raise ValueError("PI_STATIC_RECEIPT_MISSING")
            validate_pi_receipt(self.pi_receipt, self.packet)
            self._pump()
            if self.stop_event.is_set():
                raise InterruptedError("STOP_BEFORE_PI")
            self._status("STARTING_PI")
            remote = self._launch_pi()
            preflight_only = False
            parser = PairingOutputParser()
            announcement = []
            def pairing_announced():
                announcement.extend(parser.feed(self._read_pty(self.pi)))
                pending = parser.buffer.lower().rstrip()
                if self.auth_child is self.pi and pending.endswith(b":") and (b"password" in pending or b"passphrase" in pending):
                    # Interactive prompts need not end with a newline. Without
                    # removing this partial prompt the next PI_PAIRING marker
                    # could be joined to it and be missed.
                    parser.buffer = b""
                return bool(announcement)
            self._wait(pairing_announced, 125)
            self._status("READING_PAIRING")
            self.transfer = PairingTransfer(announcement[0], self.packet, host="10.0.0.1",
                runtime_dir=self.runtime, assert_owned_pi=self._owned_pi)
            def copied():
                self._read_pty(self.transfer.child)
                if parser.feed(self._read_pty(self.pi)):
                    raise RuntimeError("DUPLICATE_PI_PAIRING")
                return self.transfer.child.poll() is not None
            self._wait(copied, 30)
            pairing_path = self.transfer.finish()
            self.remote_binding = validate_binding(pairing_path.read_bytes(), announcement[0], self.packet)
            self._record_ownership('PAIRED')
            if self.kind == "verify_stop" and self.remote_binding.get("boot_id") != self.recovery_boot_id:
                raise ValueError("RECOVERY_PI_BOOT_CHANGED")
            self._pump()
            if self.stop_event.is_set():
                raise InterruptedError("STOP_BEFORE_LAPTOP")
            self._status("STARTING_MAP" if self.kind == "mapping" else
                         "STARTING_STOP_CHECK" if self.kind == "verify_stop" else "STARTING_FIRE")
            self._spawn_laptop(remote, pairing_path)
            last_stage = last_save = None
            while not self.local.done:
                self._pump()
                if parser.feed(self._read_pty(self.pi)):
                    raise RuntimeError("DUPLICATE_PI_PAIRING")
                startup = self._runner_status()
                if self.pi.poll() is not None and not self._pi_exit_is_drain(startup):
                    self._request_stop("PI_RUNNER_EXITED_DURING_SESSION")
                    from .session_outcome import primary_error
                    error = primary_error(self.out / "laptop", error or "PI_RUNNER_EXITED_DURING_SESSION")
                if self.stop_event.is_set():
                    self._status("STOPPING") if self.stage != "STOPPING" else None
                    self.local.stop(self.clock())
                else:
                    self.local.tick(self.clock())
                    if self.kind == "fire" and self.binding.socket_path.exists() and not self.operator_announced:
                        self.operator_announced = True
                        self.emit(dict(event="OPERATOR", session=self.session, generation=self.binding.generation,
                            nonce=self.binding.nonce, socket_path=str(self.binding.socket_path)))
                    if self.kind == "mapping" and self.binding.socket_path.exists() and self.key_socket is None:
                        self.key_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                        self.key_socket.settimeout(.1)
                        self.key_socket.connect(str(self.binding.socket_path))
                    if startup is not None:
                        row = startup
                        stage = row.get("stage")
                        if row != last_stage:
                            last_stage = row
                            self._status(stage or "STARTING", startup=row)
                    saved = self.out / "laptop/map_result.json"
                    if saved.exists():
                        row = json.loads(saved.read_text())
                        if row.get("session") == self.session and row.get("id") != last_save:
                            last_save = row["id"]
                            self.emit(dict(event="SAVED", **row))
                time.sleep(.03)
            row = json.loads((self.out / "laptop/receipt.json").read_text())
            from .session_outcome import primary_error
            error = primary_error(self.out / "laptop", error or ("RUNNER_EXIT_" + str(row.get("rc")) if row.get("rc") != 0 else None))
            stopped, clean = receipt_result(row, self.session, self.packet, remote_binding=self.remote_binding)
            clean = clean and self.local.report()["group_exit_confirmed"] and not self.local.forced and self.local.rc == row.get("rc") and self.local.rc in (0, 2)
        except BaseException as exc:
            safe_reasons = {
                "PI_STATIC_RECEIPT_MISSING": "Chưa có receipt đồng bộ Pi cho bản app này",
                "PI_STATIC_RECEIPT_MISMATCH": "Receipt Pi không khớp phiên bản app",
                "PI_STATIC_RECEIPT_INVALID": "Receipt Pi không hợp lệ",
                "SELECTED_MAP_CHANGED": "Map được chọn đã thay đổi; hãy chọn lại",
            }
            if isinstance(exc, ValueError) and re.fullmatch(r'RECOVERY_[A-Z0-9_]+(?::RECOVERY_[A-Z0-9_]+)*', str(exc)):
                from .recovery import failure_message
                error = failure_message(exc)
            else:
                error = safe_reasons.get(str(exc), str(exc)[:400] if isinstance(exc, RuntimeError)
                                         else type(exc).__name__ + ":" + self.stage)
            self.stop_event.set()
        finally:
            from .session_outcome import primary_error
            error = primary_error(self.out / "laptop", error)
            self._status("STOPPING", error=error)
            # Keep the Pi PTY open through the complete local STOP/drain, even
            # if the frontend process disappeared or the worker raised.
            if self.local is not None and self.local.process is None:
                # The spawn failed before a child existed; the paired Pi can
                # still be cleanly stopped by its own mailbox below.
                self.local = None
                if getattr(self, "runner_log", None) is not None:
                    self.runner_log.close()
            if self.local is not None:
                while not self.local.done:
                    self.local.stop(self.clock())
                    if self.pi is not None:
                        try:
                            self._read_pty(self.pi)
                        except OSError:
                            transport_ok = False
                    time.sleep(.03)
                row = None
                try:
                    row = json.loads((self.out / "laptop/receipt.json").read_text())
                    stopped, clean = receipt_result(row, self.session, self.packet, remote_binding=self.remote_binding)
                    clean = clean and self.local.report()["group_exit_confirmed"] and not self.local.forced and self.local.rc == row.get("rc") and self.local.rc in (0, 2)
                except (OSError, ValueError):
                    stopped = clean = False
                try:
                    self.runner_log.close()
                except OSError:
                    transport_ok = False
                # A failed R5 cleanup cannot be turned into authority to stop
                # Pi while its registered laptop groups still exist. The R5
                # candidate refuses remote STOP in that case; retain the PTY
                # and observe group disappearance before attempting it here.
                if isinstance(row, dict):
                    local_result = row.get("local", {})
                    reports = local_result.get("processes", {}) if isinstance(local_result, dict) else {}
                    unproven = {name: report for name, report in reports.items()
                                if isinstance(report, dict) and report.get("group_exit_confirmed") is not True}
                    if unproven:
                        self.emit(dict(event="PI_DRAIN_PENDING", error="Đang chờ tiến trình laptop thoát trước khi dừng Pi"))
                        from operational_owned_process import Backend
                        while unproven:
                            for name, report in tuple(unproven.items()):
                                pid = report.get("pid")
                                if type(pid) is int and pid > 1:
                                    try:
                                        alive = [p for p in Backend().snapshot(pid) if p["state"] not in ("Z", "X")]
                                        if not alive:
                                            del unproven[name]
                                    except (OSError, ValueError, KeyError):
                                        pass
                            if unproven:
                                time.sleep(.1)
                        if self.remote_binding is not None:
                            from operational_peer import SshPi
                            try:
                                late_remote = SshPi(self.remote_binding, remote + "/scripts/operational_peer.py").stop()
                                atomic(self.out / "remote_late_cleanup.json", late_remote)
                            except Exception:
                                pass
                        # Even if groups eventually disappear, earlier cleanup
                        # failed and needs explicit inspection before reuse.
                        clean = False
                elif self.pi is not None and self.pi.poll() is None:
                    # With no laptop receipt there is no list of owned child
                    # groups to audit. Keep the Pi PTY until its runner ends;
                    # a top-level process exit alone is insufficient proof.
                    self.emit(dict(event="PI_DRAIN_PENDING", error="Thiếu receipt laptop; giữ Pi để kiểm tra cleanup"))
                    while self.pi.poll() is None:
                        try:
                            self._read_pty(self.pi)
                        except OSError:
                            pass
                        time.sleep(.1)
                    clean = False
            elif self.remote_binding is not None:
                from operational_peer import SshPi
                try:
                    peer = SshPi(self.remote_binding, remote + "/scripts/operational_peer.py")
                    state = peer.startup("STATE")
                    never_started = state.get("phase") == "WAIT_LAPTOP"
                    remote_result = peer.stop()
                    atomic(self.out / "remote_only_cleanup.json", remote_result)
                    stopped = never_started and remote_result.get("confirmed") is True
                    clean = stopped
                except Exception:
                    stopped = clean = False
            elif preflight_only and self.pi is None:
                # No session was started, so there is no robot STOP proof to
                # claim. This is a clean preflight rejection, not a STOP.
                stopped, clean = None, True
            if self.key_socket is not None:
                try:
                    self.key_socket.close()
                except OSError:
                    clean = False
            if self.transfer is not None:
                child = self.transfer.child
                try:
                    if child.poll() is None:
                        # Owned scp only; no robot runtime lives in this group.
                        try:
                            os.killpg(child.process.pid, signal.SIGINT)
                        except ProcessLookupError:
                            pass
                        try:
                            child.process.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            try:
                                os.killpg(child.process.pid, signal.SIGKILL)
                            except ProcessLookupError:
                                pass
                            child.process.wait(timeout=2)
                    self.transfer.close_after_exit()
                except (OSError, subprocess.TimeoutExpired, RuntimeError):
                    clean = False
            if self.pi is not None:
                # After local drain, request the still-owned SSH foreground
                # runner to stop. A local SSH exit alone proves no robot STOP.
                if self.pi.poll() is None:
                    try:
                        self.pi.write(b"\x03")
                    except (OSError, ValueError):
                        transport_ok = False
                    until = self.clock() + 40
                    while self.pi.poll() is None and self.clock() < until:
                        try:
                            self._read_pty(self.pi)
                        except OSError:
                            transport_ok = False
                        time.sleep(.03)
                if self.pi.poll() is None:
                    clean = False
                    self.emit(dict(event="PI_DRAIN_PENDING", error="Pi SSH chưa thoát; supervisor tiếp tục giữ PTY"))
                    # Do not orphan a controlling terminal or kill Pi because
                    # a UI-close timeout expired. Retain ownership until exit.
                    while self.pi.poll() is None:
                        try:
                            self._read_pty(self.pi)
                        except OSError:
                            transport_ok = False
                        time.sleep(.05)
                clean = clean and self.pi.poll() == 0 and transport_ok
                try:
                    self.pi.close_after_exit()
                except (OSError, RuntimeError):
                    clean = False
            if self.stop_reason == "FRONTEND_HEARTBEAT_TIMEOUT":
                error = self.stop_reason + (":" + error if error else "")
            from .session_outcome import startup_result
            startup = startup_result(self.last_startup_stage, error)
            try:
                atomic(self.out / "desktop_receipt.json", dict(session=self.session, packet=self.packet,
                    startup_result=startup, primary_error=error,
                    schema_version=2, recovery_marker_sha256=self.recovery_marker_sha256,
                    recovery_boot_id=self.recovery_boot_id,
                    stop_reason=self.stop_reason, stop_diagnostic_error=self.stop_diagnostic_error,
                    pi_ssh_rc=None if self.pi is None else self.pi.poll(),
                    robot_stop_confirmed=stopped, cleanup_clean=clean,
                    session_was_started=self.pi is not None, error=error))
            except OSError:
                clean = False
                error = "Không ghi được receipt phiên; kiểm tra ổ đĩa và log"
            self.finished = True
            self.emit(dict(event="FINISHED", session=self.session, robot_stop_confirmed=stopped,
                           cleanup_clean=clean, session_was_started=self.pi is not None,
                           error=error, log_dir=str(self.out)))
