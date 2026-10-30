"""Owned mapping session: R5 Pi pairing, SLAM, keyboard PTY, atomic map save."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import struct
import sys
import threading
import time
from dataclasses import asdict

from .operator_bootstrap import read_bootstrap_fd
from .map_save import save_map_pair, MapSaveError
from .pty_child import PtyChild


class StopOnlyComplete(Exception):
    pass


KEYS = frozenset("uiojklm,. ")


class PtyBackend:
    def __init__(self):
        from operational_owned_process import Backend
        self.delegate = Backend()
        self.child = None
        self.output = None

    def start(self, argv, output):
        self.output = output
        self.child = PtyChild(argv)
        return self.child.process

    def snapshot(self, pid): return self.delegate.snapshot(pid)
    def send(self, pid, sig, leader_only): return self.delegate.send(pid, sig, leader_only)
    def reap(self, process): return self.delegate.reap(process)

    def pump(self):
        if self.child is not None:
            raw = self.child.read()
            if raw:
                # Teleop has no auth input or pairing data. Keep complete log.
                self.output.write(raw)


def run(argv=None):
    from operational_entry import ROOT, LocalProcesses, atomic, packet_digest, read_health, ros, launch, run_clock_gate
    from operational_preflight import preflight
    from .app_peer import AppPeer as SshPi
    from operational_logs import strict_json
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bootstrap-fd", type=int, required=True)
    parser.add_argument("--pairing", type=Path, required=True)
    parser.add_argument("--remote-script", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--map-root", type=Path, required=True)
    parser.add_argument("--verify-stop", action="store_true")
    a = parser.parse_args(argv)
    boot = read_bootstrap_fd(a.bootstrap_fd)
    os.close(a.bootstrap_fd)
    a.out.mkdir(mode=0o700)
    stop = threading.Event()
    transition_lock = threading.Lock()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    local = None
    lockfd = None
    peer = None
    probe = None
    listener = connection = None
    backend = PtyBackend()
    saver = ThreadPoolExecutor(max_workers=1)
    future = None
    seen_request = None
    status = "STARTING"
    rc = 2
    map_at = [None]
    map_gid = [None]
    proof = [False, 0.]
    buffer = b""
    deadline = None
    status_clock = [None, time.monotonic()]
    def status_write(**extra):
        if status_clock[0] != status:
            status_clock[:] = [status, time.monotonic()]
        atomic(a.out / "mapping.json", dict(session=boot.session, sample_mono=time.monotonic(),
            stage=status, elapsed_s=max(0.,time.monotonic()-status_clock[1]),
            remaining_s=None if deadline is None else max(0.,deadline-time.monotonic()), **extra))
    def owned_mapping():
        if stop.is_set() or status != "MAPPING" or not proof[0] or time.monotonic() - proof[1] > .5:
            raise MapSaveError("SLAM không còn thuộc phiên đang quét hoặc có Nav2")
    def generate(prefix):
        from operational_owned_process import OwnedProcess
        from operational_logs import OwnedLog
        proc = OwnedProcess("map_saver")
        log = OwnedLog(prefix.parent / "save.log")
        try:
            log.start(proc, ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(prefix),
                "--ros-args", "-p", "save_map_timeout:=10.0", "-p", "map_subscribe_transient_local:=true"])
            deadline = time.monotonic() + 25
            while not proc.done:
                now = time.monotonic()
                if stop.is_set() or now >= deadline:
                    proc.stop(now)
                else:
                    proc.tick(now)
                time.sleep(.03)
            if stop.is_set() or proc.rc != 0 or proc.forced or proc.error:
                raise MapSaveError("Lưu map bị ngắt hoặc không thành công")
        finally:
            if proc.process is not None and not proc.done:
                while not proc.done:
                    proc.stop(time.monotonic())
                    time.sleep(.03)
            log.close()
    def save_owned(name):
        # Session teardown waits for this worker before retiring SLAM/Pi;
        # stop.set() is independent so a running map_saver is interrupted.
        with transition_lock:
            return save_map_pair(a.map_root, name, assert_owned_mapping=owned_mapping, generate=generate)
    try:
        if packet_digest() != boot.packet:
            raise ValueError("MAPPING_PACKET_MISMATCH")
        preflight(ROOT, "laptop")
        lock_path = Path.home() / ".cache/fire_robot_r5_laptop.lock"
        lock_path.parent.mkdir(exist_ok=True)
        lockfd = lock_path.open("a")
        fcntl.flock(lockfd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        bind = strict_json(a.pairing.read_text())
        if bind.get("packet") != boot.packet:
            raise ValueError("MAPPING_PI_PACKET_MISMATCH")
        peer = SshPi(bind, a.remote_script)
        if peer.startup("STATE").get("phase") != "WAIT_LAPTOP":
            raise ValueError("PI_NOT_WAITING_FOR_MAPPING")
        run_clock_gate(a.out, stop.is_set)
        if stop.is_set():
            raise InterruptedError("MAPPING_STOP_DURING_CLOCK")
        local = LocalProcesses(a.out, boot.session, backend_factory=lambda role: backend if role == "rviz" else None)
        from .desktop_hooks import wait_monitor_ready
        from .monitor_log import start_monitor
        start_monitor(local, [sys.executable, "-B", "-m", "services.mapping_monitor"])
        wait_monitor_ready(local, stop.is_set)
        peer.startup("START")
        # Pi already owns its own `r5_startup_health_pi` node. Use the laptop
        # role here to avoid a second node with the same name on the network.
        from .app_health_entry import health_command
        local.start("startup_health", health_command( "--out", a.out / "health.json", "--session", boot.session, "--role", "laptop"))
        deadline = time.monotonic() + 65
        status = "WAIT_PI_IO"
        while not stop.is_set():
            local.check(time.monotonic())
            try:
                h = read_health(a.out / "health.json", boot.session, time.monotonic)
                if h["checks"].get("io") is True and h.get("stable_s", {}).get("io", 0) >= 1:
                    break
            except (OSError, ValueError, KeyError):
                pass
            if time.monotonic() >= deadline:
                raise TimeoutError("MAPPING_PI_IO_TIMEOUT")
            status_write()
            time.sleep(.05)
        if stop.is_set():
            raise InterruptedError("MAPPING_STOP_DURING_IO")
        if a.verify_stop:
            status = "VERIFYING_STOP"
            status_write()
            raise StopOnlyComplete()
        sealed = peer.startup("SEAL")
        deadline = time.monotonic() + 20.
        status = "WAIT_PI_SEAL"
        peer_next = time.monotonic() + 2.
        while not stop.is_set() and not (sealed.get("phase") == "RUNNING_OBSERVE"
                and sealed.get("sealed") is True and sealed.get("health", {}).get("io") is True):
            if time.monotonic() >= deadline:
                raise TimeoutError("MAPPING_PI_SEAL_TIMEOUT")
            local.check(time.monotonic())
            status_write()
            time.sleep(.05)
            if time.monotonic() >= peer_next:
                sealed = peer.startup("STATE")
                peer_next = time.monotonic() + 2.
        if stop.is_set():
            raise InterruptedError("MAPPING_STOP_DURING_SEAL")
        local.start("nav", launch("fire_robot_navigation", "slam.launch.py", "rviz:=false", "use_sim_time:=false"), launch_leader=True)
        status = "WAIT_SLAM"
        status_write()
        rclpy.init(args=["mapping_probe"], signal_handler_options=SignalHandlerOptions.NO)
        probe = rclpy.create_node("fire_robot_mapping_probe")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def on_map(msg):
            # Humble's Python executor invokes subscriptions with the message
            # alone. Pin the sole graph endpoint when a valid map arrives; do
            # not rely on MessageInfo, which this executor does not deliver.
            infos = probe.get_publishers_info_by_topic("/map")
            if (msg.header.frame_id == "map" and msg.info.width > 0 and msg.info.height > 0
                    and len(infos) == 1 and infos[0].node_name == "slam_toolbox"
                    and infos[0].node_namespace == "/"
                    and infos[0].topic_type == "nav_msgs/msg/OccupancyGrid"):
                gid = bytes(infos[0].endpoint_gid)
                if any(gid) and (map_gid[0] is None or map_gid[0] == gid):
                    map_at[0] = time.monotonic()
                    map_gid[0] = gid
        probe.create_subscription(OccupancyGrid, "/map", on_map, qos)
        def graph_proof():
            names = probe.get_node_names_and_namespaces()
            infos = probe.get_publishers_info_by_topic("/map")
            return (names.count(("slam_toolbox", "/")) == 1 and not any(n == "bt_navigator" for n, _ in names)
                and len(infos) == 1 and infos[0].node_name == "slam_toolbox"
                and infos[0].node_namespace == "/" and infos[0].topic_type == "nav_msgs/msg/OccupancyGrid"
                and bytes(infos[0].endpoint_gid) == map_gid[0] and map_at[0] is not None)
        status = "WAIT_SLAM"
        deadline = time.monotonic() + 60
        while not stop.is_set():
            rclpy.spin_once(probe, timeout_sec=.03)
            local.check(time.monotonic())
            if graph_proof():
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("MAPPING_SLAM_TIMEOUT")
            status_write()
        if stop.is_set():
            raise InterruptedError("MAPPING_STOP_DURING_SLAM")
        if probe.get_publishers_info_by_topic("/cmd_vel_raw"):
            raise ValueError("MAPPING_RAW_OWNER_PRESENT")
        from .display_guard import wait_view
        wait_view(boot.socket_path.parent / "display.json", boot.session, stop.is_set)
        local.start("rviz", ["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard", "--ros-args",
            "-r", "cmd_vel:=/cmd_vel_raw", "-p", "speed:=0.1", "-p", "turn:=0.8"])
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        listener.bind(str(boot.socket_path))
        os.chmod(boot.socket_path, 0o600)
        listener.listen(1)
        listener.setblocking(False)
        status = "MAPPING"
        while not stop.is_set():
            rclpy.spin_once(probe, timeout_sec=.01)
            local.check(time.monotonic())
            backend.pump()
            proof[:] = [graph_proof(), time.monotonic()]
            if not proof[0]:
                raise RuntimeError("MAPPING_GRAPH_LOST")
            display = strict_json((boot.socket_path.parent / "display.json").read_text())
            if display.get("session") != boot.session or not 0 <= time.monotonic() - display["sample_mono"] <= .75:
                raise RuntimeError("MAPPING_FRONTEND_LOST")
            if connection is None:
                try:
                    connection, _ = listener.accept()
                    uid = struct.unpack("3i", connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))[1]
                    if uid != os.getuid():
                        raise ValueError("MAPPING_PEER_UID")
                    connection.setblocking(False)
                except BlockingIOError:
                    pass
            if connection is not None:
                try:
                    raw = connection.recv(2048)
                    if not raw:
                        raise RuntimeError("MAPPING_KEYBOARD_DISCONNECTED")
                    buffer += raw
                    if len(buffer) > 4096:
                        raise ValueError("MAPPING_KEYBOARD_FLOOD")
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        row = strict_json(line)
                        if set(row) != {"session", "key"} or row["session"] != boot.session or row["key"] not in KEYS:
                            raise ValueError("MAPPING_KEY_INVALID")
                        backend.child.write(row["key"].encode("ascii"))
                except BlockingIOError:
                    pass
            request_path = a.out / "map_request.json"
            if request_path.exists():
                row = strict_json(request_path.read_text())
                if row.get("session") != boot.session:
                    raise ValueError("FOREIGN_SAVE_REQUEST")
                if row["id"] != seen_request and future is None:
                    seen_request = row["id"]
                    future = saver.submit(save_owned, row["name"])
            if future is not None and future.done():
                try:
                    saved = future.result()
                    atomic(a.out / "map_result.json", dict(session=boot.session, id=seen_request, map=asdict(saved)))
                except Exception as exc:
                    atomic(a.out / "map_result.json", dict(session=boot.session, id=seen_request, error=str(exc)))
                future = None
            status_write()
            time.sleep(.02)
        rc = 0
    except StopOnlyComplete:
        rc = 0
    except BaseException as exc:
        atomic(a.out / "failure.json", dict(error=type(exc).__name__ + ":" + str(exc)))
    finally:
        stop.set()
        status = "STOPPING"
        try:
            status_write()
        except OSError:
            pass  # Never skip process teardown because a status write failed.
        # Withdraw keyboard publisher before asking the existing STOP verifier.
        if local is not None and "rviz" in local.processes:
            p = local.processes["rviz"]
            # Teleop reads Ctrl-C as a byte in raw terminal mode and sends its
            # final zero before shutting ROS down. A SIGINT can shut rclpy down
            # before that publish, so only escalate if this graceful path fails.
            try:
                backend.child.write(b"\x03")
            except (OSError, ValueError):
                pass
            try:
                deadline = time.monotonic() + 3.
                while not p.done and time.monotonic() < deadline:
                    p.tick(time.monotonic())
                    backend.pump()
                    time.sleep(.03)
                while not p.done:
                    p.stop(time.monotonic())
                    backend.pump()
                    time.sleep(.03)
                backend.pump()
                backend.child.close_after_exit()
            except (OSError, RuntimeError, ValueError):
                # local.drain still owns this role and will attempt cleanup;
                # its receipt cannot claim clean if exit isn't proven.
                rc = 2
        saver.shutdown(wait=True)
        result = dict(clean=False, stop_confirmed=False)
        if local is not None:
            try:
                result = local.drain(cause="MAPPING_FINALLY")
            except Exception as exc:
                result = dict(clean=False, stop_confirmed=False, error=type(exc).__name__)
            finally:
                try:
                    local.close_logs()
                except OSError:
                    result = dict(clean=False, stop_confirmed=False, error="LOG_CLOSE_FAILED")
        try:
            from .desktop_hooks import stop_peer_after_local
            remote = (stop_peer_after_local(local, peer) if local is not None else peer.stop()) if peer is not None else dict(confirmed=False)
        except Exception as exc:
            remote = dict(confirmed=False, error=type(exc).__name__)
        atomic(a.out / "remote_cleanup.json", remote)
        if not result.get("clean") or remote.get("confirmed") is not True:
            rc = 2
        atomic(a.out / "receipt.json", dict(rc=rc, session=boot.session, packet=boot.packet, local=result, remote=remote, profile="DESKTOP_STOP_RECOVERY" if a.verify_stop else "DESKTOP_MAPPING"))
        for obj in (connection, listener):
            if obj is not None:
                obj.close()
        if listener is not None:
            boot.socket_path.unlink(missing_ok=True)
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if lockfd is not None:
            fcntl.flock(lockfd, fcntl.LOCK_UN)
            lockfd.close()
    return rc


if __name__ == "__main__":
    raise SystemExit(run())
