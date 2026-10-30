import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import unittest

from services.operator_bootstrap import create_bootstrap_file, read_bootstrap_fd
from services.operator_entry import main
from services.operator_runner import OperatorBootstrap


class OperatorBootstrapTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.bootstrap = OperatorBootstrap("8518a120-a012-4db3-bbf3-f73f67aed61e",
            "b" * 32, "c" * 32, "a" * 64, self.root / "socket")
    def tearDown(self): self.temp.cleanup()

    def test_anonymous_read_only_fd_survives_offset_and_hides_nonce(self):
        with create_bootstrap_file(self.bootstrap, self.root) as fd:
            info = os.fstat(fd.fileno())
            self.assertTrue(stat.S_ISREG(info.st_mode))
            self.assertEqual(stat.S_IMODE(info.st_mode), 0o600)
            self.assertEqual(info.st_nlink, 0)
            self.assertNotIn("c" * 32, repr(self.bootstrap))
            with self.assertRaises(OSError): os.write(fd.fileno(), b"bad")
            os.lseek(fd.fileno(), 0, os.SEEK_END)
            self.assertEqual(read_bootstrap_fd(fd.fileno()), self.bootstrap)

    def test_descriptor_can_be_inherited_without_secret_in_argv(self):
        with create_bootstrap_file(self.bootstrap, self.root) as file:
            argv = [sys.executable, "-c", "import sys; from services.operator_bootstrap import "
                "read_bootstrap_fd; assert read_bootstrap_fd(int(sys.argv[1])).session == sys.argv[2]",
                str(file.fileno()), self.bootstrap.session]
            self.assertNotIn(self.bootstrap.nonce, repr(argv))
            result = subprocess.run(argv, pass_fds=(file.fileno(),), capture_output=True, timeout=3)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            self.assertNotIn(self.bootstrap.nonce.encode(), result.stdout + result.stderr)

    def test_bad_permissions_writable_fd_and_untrusted_json_are_rejected(self):
        path = self.root / "bootstrap.json"
        value = {"version": 1, "session": self.bootstrap.session, "generation": self.bootstrap.generation,
                 "nonce": self.bootstrap.nonce, "packet": self.bootstrap.packet,
                 "socket_path": str(self.bootstrap.socket_path)}
        path.write_text(json.dumps(value))
        os.chmod(path, 0o644)
        with path.open("rb") as fd, self.assertRaises(ValueError): read_bootstrap_fd(fd.fileno())
        os.chmod(path, 0o600)
        with path.open("r+b") as fd, self.assertRaises(ValueError): read_bootstrap_fd(fd.fileno())
        for content in ("{}", json.dumps(value)[:-1] + ',"version":1}',
                        json.dumps(value).replace('"version": 1', '"version": NaN'), "x" * 4097):
            path.write_text(content)
            with path.open("rb") as fd, self.assertRaises(ValueError): read_bootstrap_fd(fd.fileno())

    def test_public_runtime_dir_and_bad_descriptors_rejected(self):
        os.chmod(self.root, 0o755)
        with self.assertRaises(ValueError): create_bootstrap_file(self.bootstrap, self.root)
        os.chmod(self.root, 0o700)
        for fd in (-1, 0, 1, 2, True):
            with self.subTest(fd=fd), self.assertRaises(ValueError): read_bootstrap_fd(fd)

    def test_invalid_order_extra_fields_and_bad_path_encoding_are_rejected(self):
        with create_bootstrap_file(self.bootstrap, self.root) as file:
            valid = os.pread(file.fileno(), 4097, 0)
        parts = valid.split(b"\n")
        cases = [b"2\n" + valid[2:], valid + b"extra\n", valid.replace(parts[2], b"SHORT"),
                 b"\n".join(parts[:5] + [b"%%%", b""]),
                 b"\n".join(parts[:5] + [b"L3RtcC8AdGVzdA==", b""])]
        path = self.root / "invalid-bootstrap"
        for raw in cases:
            path.write_bytes(raw)
            os.chmod(path, 0o600)
            with self.subTest(raw=raw[:8]), path.open("rb") as file, self.assertRaises(ValueError):
                read_bootstrap_fd(file.fileno())

    def test_nil_session_cannot_be_created(self):
        invalid = OperatorBootstrap("00000000-0000-0000-0000-000000000000", "b" * 32,
                                    "c" * 32, "a" * 64, self.root / "socket")
        with self.assertRaises(ValueError): create_bootstrap_file(invalid, self.root)

    def test_entrypoint_passes_binding_and_ros_arguments_without_leaking(self):
        calls = []
        with create_bootstrap_file(self.bootstrap, self.root) as fd:
            inherited = os.dup(fd.fileno())
            result = main(["--bootstrap-fd", str(inherited), "--enable-integrated-candidate",
                           "--ros-args", "-p", "show_gui:=false"],
                          run=lambda bootstrap, args: calls.append((bootstrap, args)) or 0)
            self.assertEqual(result, 0)
            self.assertEqual(calls, [(self.bootstrap, ["fire_robot_integrated_operator", "--ros-args", "-p", "show_gui:=false"])])
            with self.assertRaises(OSError): os.fstat(inherited)


if __name__ == "__main__":
    unittest.main()
