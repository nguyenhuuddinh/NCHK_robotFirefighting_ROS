"""Versioned hooks used by the generated desktop R5 candidate only."""
import os
from pathlib import Path
import sys
import time

from .operator_bootstrap import read_bootstrap_fd
from .operator_owned_backend import OperatorOwnedBackend


def stop_peer_after_local(local, peer):
    """Never request Pi STOP while an owned laptop group is unproven."""
    remaining = []
    for name, process in local.processes.items():
        try:
            if process.report().get("group_exit_confirmed") is not True:
                remaining.append(name)
        except Exception:
            remaining.append(name)
    if remaining:
        return dict(confirmed=False, reason="LOCAL_GROUP_EXIT_UNCONFIRMED",
                    remaining=remaining,
                    binding={k: v for k, v in peer.bind.items() if k != "secret"})
    return peer.stop()


class DesktopHooks:
    def __init__(self, fd, root, out):
        self.binding = read_bootstrap_fd(fd)
        os.close(fd)
        self.root, self.out = Path(root), Path(out)
        self.app = self.root / "desktop"
        self.runtime = self.binding.socket_path.parent
        self.fallback = False
        from operational_owned_process import Backend
        self.operator = OperatorOwnedBackend(self.binding, delegate=Backend(),
            app_dir=self.app, candidate_root=self.root, session_dir=self.out,
            runtime_dir=self.runtime, env=dict(os.environ))

    def backend(self, role):
        return self.operator if role == "operator" and not self.fallback else None

    def prepare_start_monitor(self, local, stop):
        if stop():
            raise InterruptedError("STOP_BEFORE_PI_START")
        if not self.prepare_stop_monitor(local, remote_started=True):
            raise RuntimeError("PREARM_STOP_MONITOR_NOT_READY")
        if stop():
            raise InterruptedError("STOP_BEFORE_PI_START")

    def wait_view(self, stop):
        from .display_guard import wait_view
        wait_view(self.runtime / "display.json", self.binding.session, stop)

    def mark_stopping(self):
        from .live_session import atomic
        try:
            atomic(self.out / "startup.json", dict(session=self.binding.session, stage="STOPPING"))
        except OSError:
            print("APP_STOP_STATUS_WRITE_FAILED", flush=True)

    def handoff_monitor(self, local):
        if not self.fallback or "operator" not in local.processes:
            return
        retirement = local.begin_retirement(time.monotonic(), self.out / "prearm_stop", ("operator",))
        deadline = time.monotonic() + 30.
        while not local.tick_retirement(retirement, time.monotonic()):
            if time.monotonic() >= deadline:
                raise TimeoutError("STOP_MONITOR_HANDOFF_TIMEOUT")
            time.sleep(.03)
        from operational_logs import strict_json
        result = strict_json((self.out / "prearm_stop/retirement.json").read_text())
        if result.get("clean") is not True:
            raise RuntimeError("PREARM_STOP_HANDOFF_UNCONFIRMED")
        self.fallback = False

    def prepare_stop_monitor(self, local, *, remote_started, clock=time.monotonic, sleep=time.sleep):
        """For an early startup stop, start the existing STOP-only verifier.

        The normal operator is never replaced or duplicated; a missing ready
        marker remains a failed/unconfirmed STOP, never a synthetic receipt.
        """
        if not remote_started or "operator" in local.processes:
            return False
        self.fallback = True
        from .monitor_log import start_monitor
        process = start_monitor(local, [sys.executable, "-m", "services.mapping_monitor"])
        return wait_monitor_ready(local, lambda: False, clock=clock, sleep=sleep)

    def commands(self, original):
        result = dict(original)
        from .nav_session import NAV_PREFIX
        if original['nav'][:len(NAV_PREFIX)] != NAV_PREFIX:
            raise ValueError('APP_NAV_COMMAND_INVALID')
        result['nav'] = [sys.executable, '-m', 'services.nav_session',
                         '--diagnostics', str(self.out / 'nav_performance.jsonl'), '--',
                         *original['nav'], 'params_file:=' + str(self.app / 'generated/app_nav2_params.yaml'),
                         'sigterm_timeout:=8.0', 'sigkill_timeout:=3.0']
        # Only the generated desktop candidate overrides the R5 launch model.
        from .app_model import validate_model
        result["yolo"] = [*original["yolo"], "model_path:=" + str(validate_model())]
        result["operator"] = self.operator.command
        if original['mission'][:4] != ['ros2', 'launch', '--noninteractive', 'fire_robot_mission']:
            raise ValueError('APP_MISSION_COMMAND_INVALID')
        if original['mission'][4] == 'operational_stack_gate2.launch.py':
            result['mission'] = ['ros2', 'launch', '--noninteractive', str(self.app/'generated/app_mission.launch.py'),
                                 *original['mission'][5:]]
        # The supervisor owns the actual native RViz window. This guard is an
        # owned attachment in the R5 registry, not a second RViz process.
        result["rviz"] = [sys.executable, "-m", "services.display_guard",
                          str(self.runtime / "display.json"), self.binding.session]
        return result


def wait_monitor_ready(local, stop, *, clock=time.monotonic, sleep=time.sleep):
    process = local.processes["operator"]
    log = local.registry.logs["operator"].file
    deadline = clock() + 5.
    while clock() < deadline:
        if stop():
            raise InterruptedError("STOP_BEFORE_MONITOR_READY")
        process.tick(clock())
        if process.done:
            raise RuntimeError("STOP_MONITOR_EXITED_BEFORE_READY")
        data = os.pread(log.fileno(), min(os.fstat(log.fileno()).st_size, 65536), 0)
        if b"MAPPING_MONITOR_READY\n" in data:
            return True
        sleep(.03)
    raise TimeoutError("STOP_MONITOR_READY_TIMEOUT")
