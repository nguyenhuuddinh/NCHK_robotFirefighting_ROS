"""Real native RViz/Qt button smoke test; ROS is isolated before startup."""
import hashlib
import os
import re
from pathlib import Path
import subprocess
import sys
import tempfile
import pytest

from services.candidate_build import build

ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / "apps/fire_robot_control"
FIXTURE = Path(f"/tmp/fire_robot_control_preview_{os.getuid()}/fire_robot_gui_fixture")


@pytest.mark.parametrize("mode", ["scan", "fire", "workflow", "slow_workflow", "slow_stop", "child_crash", "child_key"])
def test_real_rviz_window_buttons_fail_closed_in_localhost_domain(mode):
    assert os.environ.get("DISPLAY"), "A display is needed for Ogre/RViz"
    assert FIXTURE.is_file(), "Build the native GUI fixture first"
    with tempfile.TemporaryDirectory(prefix="fire-rviz-qa-") as tmp:
        base = Path(tmp)
        candidate, state, maps = base / "candidate", base / "state", base / "maps"
        maps.mkdir(mode=0o700)
        (maps / "test.pgm").write_bytes(b"P5\n2 2\n255\n\0\0\xff\xff")
        (maps / "test.yaml").write_text("image: test.pgm\nresolution: 0.05\norigin: [0,0,0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        build(ROOT / "releases/r5_20261029/ros", APP, candidate)
        env = dict(os.environ, ROS_DOMAIN_ID="211", ROS_LOCALHOST_ONLY="1", FIRE_ROBOT_GUI_QA=mode)
        env["PYTHONPATH"] = os.pathsep.join((str(candidate / "desktop"), str(candidate / "scripts"), env.get("PYTHONPATH", "")))
        entry = [str(APP / "tests/qa_fake_supervisor.py")] if mode in ("workflow", "slow_workflow", "slow_stop", "child_crash", "child_key") else ["-m", "services.supervisor"]
        cmd = [sys.executable, *entry, "--candidate", str(candidate), "--binary", str(FIXTURE),
            "--binary-sha256", hashlib.sha256(FIXTURE.read_bytes()).hexdigest(), "--state-dir", str(state),
            "--map-root", str(maps), "--maps", str(maps)]
        process = subprocess.run(cmd, env=env, cwd=candidate / "desktop", text=True,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=25)
        lines = [s.removeprefix("UI_LOG=") for s in process.stdout.splitlines() if s.startswith("UI_LOG=")]
        assert len(lines) == 1, (process.stdout, process.stderr)
        log = Path(lines[0]).read_text()
        assert process.returncode == 0, (process.stdout, process.stderr, log)
        if mode in ("workflow", "slow_workflow"):
            assert "GUI_MAPPING_SPEED_LABEL_OK" in log, log
            match = re.search(r"GUI_MAPPING_VIEW_SIZE=(\d+)x(\d+) CAMERA_SIZE=(\d+)x(\d+)", log)
            assert match and all(int(value) >= 180 for value in match.groups()), log
            for event in ("SCAN", "SAVE", "STOP_MAPPING", "START_FIRE", "INITIAL_POSE_TOOL", "UNARMED_GOAL_LOCKED", "STOP_FIRE"):
                assert "GUI_WORKFLOW_" + event in log, log
            assert "GUI_FIXTURE_EXIT_STEP=15" in log, log
            assert (maps / "qa_new_map.yaml").is_file()
        elif mode == "slow_stop":
            assert "GUI_SLOW_STOP" in log, log
            assert "APP_VIEW_CHILD_STOP_REQUESTED" in log, log
            assert "APP_VIEW_ATTACHED" not in log, log
            assert "GUI_FIXTURE_EXIT_STEP=22" in log, log
        elif mode == "child_crash":
            assert "APP_VIEW_CHILD_EXITED" in log, log
            assert "APP_VIEW_ATTACHED" not in log, log
            assert "GUI_FIXTURE_EXIT_STEP=22" in log, log
        elif mode == "child_key":
            assert "APP_VIEW_ATTACHED" in log and "APP_CHILD_STOP_RECEIVED" in log, log
            assert "GUI_FIXTURE_EXIT_STEP=22" in log, log
        else:
            assert "GUI_FIXTURE_" + mode.upper() in log, log
            assert "GUI_FIXTURE_PREFLIGHT_REJECTED" in log, log
            assert "GUI_FIXTURE_EXIT_STEP=3" in log, log
        assert not (state / "pending.json").exists()
