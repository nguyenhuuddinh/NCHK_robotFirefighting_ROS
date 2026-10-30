import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from services.operator_bootstrap import read_bootstrap_fd
from services.operator_launch import launch_candidate_operator
from services.operator_runner import OperatorBootstrap


APP = Path(__file__).resolve().parents[1]


class OperatorLaunchTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.candidate = self.root / "candidate"
        self.session = self.root / "session"
        self.runtime = self.root / "runtime"
        self.session.mkdir()
        self.runtime.mkdir(mode=0o700)
        config = self.candidate / "ros2_ws/src/fire_robot_mission/config/operational_gate2.yaml"
        config.parent.mkdir(parents=True)
        config.write_text("operator: {}\n")
        self.config = config
        self.bootstrap = OperatorBootstrap("8518a120-a012-4db3-bbf3-f73f67aed61e",
            "b" * 32, "c" * 32, "a" * 64, self.runtime / "control.sock")
        self.env = dict(os.environ)
    def tearDown(self): self.tmp.cleanup()

    def launch(self, **kwargs):
        args = dict(app_dir=APP, candidate_root=self.candidate, session_dir=self.session,
                    runtime_dir=self.runtime, env=self.env, stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE)
        args.update(kwargs)
        return launch_candidate_operator(self.bootstrap, **args)

    def test_passes_exact_candidate_profile_and_anonymous_fd(self):
        seen = {}
        def fake(argv, **kwargs):
            seen.update(argv=argv, kwargs=kwargs, binding=read_bootstrap_fd(kwargs["pass_fds"][0]))
            return "child"
        self.assertEqual(self.launch(popen=fake), "child")
        self.assertEqual(seen["binding"], self.bootstrap)
        self.assertNotIn(self.bootstrap.nonce, repr(seen["argv"]))
        self.assertEqual(seen["argv"].count("show_gui:=false"), 1)
        self.assertIn("enable_live_intents:=true", seen["argv"])
        self.assertIn("automatic_mode:=true", seen["argv"])
        self.assertIn(str(self.config), seen["argv"])
        self.assertIn("automatic_runner_session:=" + self.bootstrap.session, seen["argv"])
        self.assertTrue(seen["kwargs"]["start_new_session"])
        with self.assertRaises(OSError): os.fstat(seen["kwargs"]["pass_fds"][0])

    def test_real_local_fake_child_receives_inherited_fd_after_parent_closes_it(self):
        def fake(argv, **kwargs):
            fd = kwargs["pass_fds"][0]
            source = ("from services.operator_bootstrap import read_bootstrap_fd; "
                      "import sys; b=read_bootstrap_fd(int(sys.argv[1])); "
                      "print('OK' if b.session==sys.argv[2] else 'BAD')")
            return subprocess.Popen([sys.executable, "-c", source, str(fd), self.bootstrap.session], **kwargs)
        child = self.launch(popen=fake)
        out, err = child.communicate(timeout=3)
        self.assertEqual((child.returncode, out.strip()), (0, b"OK"), err.decode())
        self.assertNotIn(self.bootstrap.nonce.encode(), out + err)

    def test_stale_start_file_foreign_profile_and_nonce_environment_rejected(self):
        (self.session / "automatic_start.json").write_text("stale")
        with self.assertRaises(ValueError): self.launch(popen=lambda *_a, **_k: None)
        (self.session / "automatic_start.json").unlink()
        env = dict(self.env, BAD_SECRET=self.bootstrap.nonce)
        with self.assertRaises(ValueError): self.launch(env=env, popen=lambda *_a, **_k: None)
        self.config.unlink()
        foreign = self.root / "foreign.yaml"
        foreign.write_text("x")
        self.config.symlink_to(foreign)
        with self.assertRaises(ValueError): self.launch(popen=lambda *_a, **_k: None)

    def test_failed_spawn_closes_private_fd(self):
        seen = []
        def fail(argv, **kwargs):
            seen.append(kwargs["pass_fds"][0])
            raise OSError("fixture spawn failed")
        with self.assertRaises(OSError): self.launch(popen=fail)
        with self.assertRaises(OSError): os.fstat(seen[0])


if __name__ == "__main__":
    unittest.main()
