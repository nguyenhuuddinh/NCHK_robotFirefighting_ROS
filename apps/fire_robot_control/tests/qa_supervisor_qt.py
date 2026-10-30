"""Real Qt client + supervisor integration, confined to localhost ROS domain.

The preflight must reject the isolated ROS domain before any SSH is started.
The fixture has no ROS node and the test starts no robot process.
"""
import os
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import pytest

from services.candidate_build import build

ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / "apps/fire_robot_control"
FIXTURE = Path(f"/tmp/fire_robot_control_preview_{os.getuid()}/fire_robot_app_fixture")


@pytest.mark.parametrize("mode", ["scan", "fire"])
def test_qt_click_runs_supervisor_and_rejects_isolated_domain_without_ssh(mode):
    assert FIXTURE.is_file(), "Build the Qt fixture first"
    with tempfile.TemporaryDirectory(prefix="fire-ui-qa-") as tmp:
        base = Path(tmp)
        candidate, state, maps = base / "candidate", base / "state", base / "maps"
        maps.mkdir(mode=0o700)
        (maps / "test.pgm").write_bytes(b"P5\n2 2\n255\n\0\0\xff\xff")
        (maps / "test.yaml").write_text("image: test.pgm\nresolution: 0.05\norigin: [0,0,0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        build(ROOT / "releases/r5_20261029/ros", APP, candidate)
        env = dict(os.environ, ROS_DOMAIN_ID="211", ROS_LOCALHOST_ONLY="1", FIRE_ROBOT_QA_FIXTURE=mode)
        env["PYTHONPATH"] = os.pathsep.join((str(candidate / "desktop"), str(candidate / "scripts"), env.get("PYTHONPATH", "")))
        cmd = [sys.executable, "-m", "services.supervisor", "--candidate", str(candidate), "--binary", str(FIXTURE),
            "--binary-sha256", hashlib.sha256(FIXTURE.read_bytes()).hexdigest(),
            "--state-dir", str(state), "--map-root", str(maps), "--maps", str(maps)]
        process = subprocess.run(cmd, env=env, cwd=candidate / "desktop", text=True,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
        lines = [line.removeprefix("UI_LOG=") for line in process.stdout.splitlines() if line.startswith("UI_LOG=")]
        assert len(lines) == 1, (process.stdout, process.stderr)
        log = Path(lines[0]).read_text()
        assert process.returncode == 0, (process.stdout, process.stderr, log)
        assert "FIXTURE_FINISHED stopped=-1 clean=1 started=0" in log
        assert "UNREVIEWED_ROS_DOMAIN_OR_LOCALHOST" in log
        assert "FIXTURE_CLOSE_READY" in log
        assert not (state / "pending.json").exists()
