"""Open the packaged native app in an isolated domain, then close it idle."""
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / "apps/fire_robot_control"


def test_packaged_binary_opens_and_supervisor_closes_it_without_a_robot():
    packet = (APP / "artifacts/current.txt").read_text().strip()
    artifact = Path(os.environ.get("FIRE_ROBOT_QA_ARTIFACT", str(APP / "artifacts" / packet)))
    candidate, binary = artifact / "ros", artifact / "bin/fire_robot_control"
    assert os.environ.get("DISPLAY")
    with tempfile.TemporaryDirectory(prefix="fire-packaged-qa-") as tmp:
        base = Path(tmp)
        maps = base / "maps"
        maps.mkdir(mode=0o700)
        env = dict(os.environ, ROS_DOMAIN_ID="211", ROS_LOCALHOST_ONLY="1")
        env.pop("FIRE_ROBOT_GUI_QA", None)
        env["PYTHONPATH"] = os.pathsep.join((str(candidate / "desktop"), str(candidate / "scripts"), env.get("PYTHONPATH", "")))
        argv = [sys.executable, "-m", "services.supervisor", "--candidate", str(candidate), "--binary", str(binary),
            "--binary-sha256", hashlib.sha256(binary.read_bytes()).hexdigest(), "--state-dir", str(base / "state"),
            "--map-root", str(maps), "--maps", str(maps)]
        child = subprocess.Popen(argv, cwd=candidate / "desktop", env=env, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, text=True)
        try:
            time.sleep(2)
            assert child.poll() is None, "Native GUI or supervisor exited during idle startup"
            if (candidate / "desktop/generated/app_fire_entry.py").exists():
                import rclpy
                from rclpy.context import Context
                context = Context()
                rclpy.init(args=["artifact_idle_graph"], context=context)
                probe = rclpy.create_node("artifact_idle_graph", context=context)
                try:
                    time.sleep(.5)
                    for topic in ("/tf", "/tf_static", "/map", "/scan", "/image_raw/compressed", "/robot_description", "/plan"):
                        assert not [x for x in probe.get_subscriptions_info_by_topic(topic)
                            if x.node_name.startswith(("fire_robot_control_preview", "fire_robot_ui_observer", "transform_listener_impl_"))]
                finally:
                    probe.destroy_node()
                    rclpy.shutdown(context=context)
            child.send_signal(signal.SIGTERM)
            out, err = child.communicate(timeout=8)
            assert child.returncode == 0, (out, err)
            assert "UI_LOG=" in out
            assert not (base / "state/pending.json").exists()
        finally:
            if child.poll() is None:
                child.kill()
                child.wait(timeout=2)
