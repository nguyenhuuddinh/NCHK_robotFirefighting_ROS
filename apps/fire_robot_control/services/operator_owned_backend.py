"""Adapter for R5 OwnedProcess: spawn the integrated operator via a private FD.

The caller must install this backend only for the operator role, preserve the
R5 process registry and STOP proof, and verify the candidate manifest first.
No process is launched until R5 OwnedProcess calls start().
"""
import subprocess

from .operator_launch import launch_candidate_operator
from .operator_runner import OperatorBootstrap


class OperatorOwnedBackend:
    def __init__(self, bootstrap, *, delegate, app_dir, candidate_root, session_dir,
                 runtime_dir, env, popen=subprocess.Popen):
        if not isinstance(bootstrap, OperatorBootstrap):
            raise TypeError("OperatorBootstrap required")
        bootstrap.validate()
        if not all(callable(getattr(delegate, name, None)) for name in ("snapshot", "send", "reap")):
            raise TypeError("R5 process backend required")
        self.bootstrap = bootstrap
        self.delegate = delegate
        self.options = dict(app_dir=app_dir, candidate_root=candidate_root,
            session_dir=session_dir, runtime_dir=runtime_dir, env=env, popen=popen)
        self.started = False

    @property
    def command(self):
        """Non-secret marker used only for the owned operator role."""
        return ["fire_robot_integrated_operator", self.bootstrap.session]

    def start(self, argv, output):
        if self.started or list(argv) != self.command:
            raise ValueError("unexpected operator role or duplicate start")
        self.started = True
        return launch_candidate_operator(self.bootstrap, stdout=output,
            stderr=subprocess.STDOUT, **self.options)

    def snapshot(self, pid):
        return self.delegate.snapshot(pid)

    def send(self, pid, sig, leader_only):
        return self.delegate.send(pid, sig, leader_only)

    def reap(self, process):
        return self.delegate.reap(process)
