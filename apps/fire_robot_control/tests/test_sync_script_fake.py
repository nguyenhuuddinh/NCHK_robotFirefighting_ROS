"""Exercise sync orchestration with SSH/SCP replaced by inert local fixtures."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from services.candidate_build import build

ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / "apps/fire_robot_control"


class SyncScriptTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="fire-sync-fake-")
        self.base = Path(self.tmp.name)
        self.artifact = self.base / "artifact"
        self.artifact.mkdir()
        self.packet = build(ROOT / "releases/r5_20261029/ros", APP, self.artifact / "ros")
        (self.artifact / "bin").mkdir()
        shutil.copy2("/bin/true", self.artifact / "bin/fire_robot_control")
        from services.packet_archive import export_packet
        export_packet(self.artifact / "ros", self.artifact / "source.tar.gz")
        sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
        (self.artifact / "build_receipt.json").write_text(json.dumps(dict(schema_version=1,
            packet=self.packet, binary_sha256=sha(self.artifact / "bin/fire_robot_control"),
            archive_sha256=sha(self.artifact / "source.tar.gz"), runtime_verified=False)))
        self.fakebin = self.base / "fakebin"
        self.fakebin.mkdir()
        (self.fakebin / "ssh").write_text("#!/bin/bash\ncase \"$*\" in *'bash -s'*) [[ ${FAKE_PI_FAIL:-0} == 0 ]] || exit 19; echo PI_STATIC_HASH_PASS=1;; esac\n")
        (self.fakebin / "scp").write_text("#!/bin/bash\nlast=${@: -1}\nif [[ $last != pi@* ]]; then cp -- \"$FAKE_PI_RECEIPT\" \"$last\"; fi\n")
        for path in self.fakebin.iterdir(): path.chmod(0o700)
        self.receipt = self.base / "fake_pi_receipt.json"
        stage = "/home/pi/fire_robot_desktop/" + self.packet[:12] + "/ros/ros2_ws/install/pkg/"
        row = dict(schema_version=1, role="pi", host_machine="aarch64", live=False,
            runtime_verified=False, status="STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED", errors=[],
            checks=dict(source_packet_sha256=self.packet,
                installed_sources=[dict(path=stage + str(i), sha256="b" * 64) for i in range(103)]))
        self.receipt.write_text(json.dumps(row))
        self.env = dict(os.environ, PATH=str(self.fakebin) + os.pathsep + os.environ["PATH"],
            XDG_STATE_HOME=str(self.base / "state"), FAKE_PI_RECEIPT=str(self.receipt))

    def tearDown(self): self.tmp.cleanup()

    def run_script(self, **extra):
        env = dict(self.env, **extra)
        return subprocess.run(["bash", str(APP / "launchers/sync_pi_candidate.sh"), str(self.artifact)],
            env=env, capture_output=True, text=True, timeout=10)

    def test_matching_fake_receipt_is_stored_under_the_exact_packet(self):
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertIn("PI_STATIC_RECEIPT_VALID=1", result.stdout)
        self.assertEqual((self.base / "state/fire_robot_control/pi_receipts" / (self.packet + ".json")).read_bytes(),
                         self.receipt.read_bytes())

    def test_failed_remote_build_never_installs_receipt(self):
        result = self.run_script(FAKE_PI_FAIL="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.base / "state/fire_robot_control/pi_receipts" / (self.packet + ".json")).exists())

    def test_foreign_receipt_never_installs(self):
        row = json.loads(self.receipt.read_text())
        row["checks"]["source_packet_sha256"] = "f" * 64
        self.receipt.write_text(json.dumps(row))
        result = self.run_script()
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.base / "state/fire_robot_control/pi_receipts" / (self.packet + ".json")).exists())


if __name__ == "__main__": unittest.main()
