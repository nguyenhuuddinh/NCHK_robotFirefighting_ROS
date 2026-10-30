"""Exercise the generated R5 laptop entrypoint with fully fake I/O."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
import uuid
import pytest

from services.candidate_build import build
import services.desktop_hooks

ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / "apps/fire_robot_control"
PACKET = "a" * 64


@pytest.mark.parametrize('runtime_failure', [False, True])
def test_generated_runner_uses_ui_attachment_and_existing_operator_cleanup_without_real_processes(runtime_failure):
    with tempfile.TemporaryDirectory(prefix="fire-runner-qa-") as temp:
        tmp = Path(temp)
        candidate = tmp / "candidate"
        build(ROOT / "releases/r5_20261029/ros", APP, candidate)
        sys.path.insert(0, str(candidate / "scripts"))
        try:
            spec = importlib.util.spec_from_file_location("qa_generated_r5_runner", candidate / "desktop/generated/app_fire_entry.py")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            map_path, rviz = tmp / "map.yaml", tmp / "rviz.rviz"
            map_path.write_text("fixture")
            rviz.write_text("fixture")
            pairing = tmp / "pairing.json"
            pairing.write_text(json.dumps({"packet": PACKET}))
            session = str(uuid.uuid4())
            records = []

            class FakeOwned:
                def report(self): return {"group_exit_confirmed": True}
            class FakeLocal:
                def __init__(self, out, session_id, backend_factory=None):
                    self.out, self.session, self.backend_factory = out, session_id, backend_factory
                    self.processes, self.history = {}, []
                def start(self, role, argv, **kwargs):
                    records.append((role, list(argv)))
                    self.processes[role] = FakeOwned()
                def check(self, now, allow_operator_exit=False, exclude=()): return allow_operator_exit
                def drain(self, **kwargs):
                    records.append(("drain", []))
                    return {"clean": True, "stop_confirmed": True, "runtime_failure": runtime_failure}
                def close_logs(self): pass
            class FakePeer:
                bind = {"session": "pi", "secret": "fixture"}
                def __init__(self, bind, remote_script): pass
                def startup(self, command):
                    return {"phase": "WAIT_LAPTOP"} if command == "STATE" else {"phase": "RUNNING_OBSERVE", "sealed": True, "health": {"io": True}}
                def stop(self):
                    records.append(("pi_stop", []))
                    return {"confirmed": True}
            class FakeSequence:
                def __init__(self, now, live=False):
                    self.stage, self.error, self.done, self.index = "CLOCK", None, False, 0
                def step(self, now, obs):
                    self.index += 1
                    if self.index == 1: self.stage = "PI_IO"; return ["START_PI", "START_HEALTH"]
                    if self.index == 2: self.stage = "POSE"; return ["START_NAV", "START_RVIZ"]
                    if self.index == 3: self.stage = "CONSUMERS"; return ["START_YOLO", "START_CONSUMERS", "START_OPERATOR"]
                    self.stage, self.done = "READY_GATE2", True
                    return []
                def diagnostics(self): return {"stage": self.stage, "remaining_s": 10}
                def report(self): return {"stage": self.stage}
            class FakeDesktop:
                def __init__(self, fd, root, out): self.binding = SimpleNamespace(packet=PACKET, session=session)
                def backend(self, role): return None
                def commands(self, commands):
                    result = dict(commands)
                    result["rviz"] = ["ui-attachment-fixture"]
                    result["operator"] = ["operator-fixture", session]
                    result['mission'] = ['app-mission-fixture', *commands['mission']]
                    return result
                def prepare_start_monitor(self, local, stop): pass
                def handoff_monitor(self, local): pass
                def mark_stopping(self):pass
                def wait_view(self, stop): records.append(("view_ready", []))
                def prepare_stop_monitor(self, local, **kwargs):
                    records.append(("monitor_fallback", []))
            class FakeRecovery:
                excluded = ()
                def __init__(self, *args):
                    self.core = SimpleNamespace(report=lambda: {})
                    self.factory, self.start = args[4:6]
                def step(self, *args, **kwargs):
                    for after_pulse in (False, True):
                        rebuilt = self.factory(self.start, session, after_pulse)
                        records.append(('recovery_mission_'+str(after_pulse), rebuilt['mission']))
                        assert rebuilt['operator'] == ['operator-fixture', session]
            health = {"checks": {}, "stable_s": {}, "operator_diagnostics": {}, "io_diagnostics": {}}
            with patch.object(services.desktop_hooks, "DesktopHooks", FakeDesktop), \
                 patch.object(Path, "home", return_value=tmp), \
                 patch.object(module, "packet_digest", return_value=PACKET), \
                 patch.object(module, "preflight"), patch.object(module, "validate_gate2_profile"), \
                 patch.object(module, "SshPi", FakePeer), patch.object(module, "LocalProcesses", FakeLocal), \
                 patch.object(module, "StartupSequence", FakeSequence), patch.object(module, "run_clock_gate"), \
                 patch.object(module, "read_health", return_value=health), \
                 patch.object(module, "startup_observations", return_value={"io": True, "camera": True, "pose": True, "yolo": True}), \
                 patch.object(module.time, "sleep"), \
                 patch.dict(sys.modules, {"runtime_recovery_runner": SimpleNamespace(RuntimeRecoveryRunner=FakeRecovery)}):
                rc = module.main(["--desktop-fd", "9", "--run-automatic", "--automatic-token", module.AUTOMATIC_TOKEN,
                    "--pairing", str(pairing), "--remote-script", "/fake/remote", "--out", str(tmp / "session"),
                    "--map", str(map_path), "--rviz", str(rviz)])
            assert rc == (2 if runtime_failure else 0)
            roles = dict(records)
            assert roles["rviz"] == ["ui-attachment-fixture"]
            order = [role for role, _ in records]
            assert order.index("rviz") < order.index("view_ready") < order.index("yolo") < order.index("operator")
            assert roles["operator"] == ["operator-fixture", session]
            assert roles['mission'][0] == 'app-mission-fixture'
            assert roles['recovery_mission_False'] == roles['mission']
            assert roles['recovery_mission_True'] == roles['mission']
            assert all("rviz2" not in argv and "show_gui:=true" not in argv for _, argv in records)
            assert [role for role, _ in records].index("drain") < [role for role, _ in records].index("pi_stop")
            receipt = json.loads((tmp / "session/receipt.json").read_text())
            assert receipt["session"] == session and receipt["packet"] == PACKET and receipt["local"]["stop_confirmed"] is True
        finally:
            sys.path.remove(str(candidate / "scripts"))
