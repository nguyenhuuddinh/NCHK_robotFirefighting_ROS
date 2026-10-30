"""Launch the candidate operator with a private inherited bootstrap FD.

This is an integration building block, not the R5 runner. Its caller must run
the existing packet/install preflight, own the child and perform STOP drain.
No process or network access occurs on import.
"""
from pathlib import Path
import subprocess
import sys

from .operator_bootstrap import create_bootstrap_file
from .operator_runner import OperatorBootstrap


def launch_candidate_operator(bootstrap, *, app_dir, candidate_root, session_dir,
                              runtime_dir, env, stdout, stderr, popen=subprocess.Popen):
    if not isinstance(bootstrap, OperatorBootstrap):
        raise TypeError("OperatorBootstrap required")
    bootstrap.validate()
    app = Path(app_dir).resolve(strict=True)
    root = Path(candidate_root).resolve(strict=True)
    session = Path(session_dir).resolve(strict=True)
    runtime = Path(runtime_dir).resolve(strict=True)
    if not all(path.is_dir() for path in (app, root, session, runtime)):
        raise ValueError("invalid operator launch directory")
    if not (app / "services" / "operator_entry.py").is_file():
        raise ValueError("operator entrypoint missing")
    if bootstrap.socket_path.parent.resolve(strict=True) != runtime:
        raise ValueError("operator socket is not in the runtime directory")
    config = root / "ros2_ws/src/fire_robot_mission/config/operational_gate2.yaml"
    resolved_config = config.resolve(strict=True)
    if not resolved_config.is_relative_to(root) or not resolved_config.is_file():
        raise ValueError("operator profile outside candidate")
    start_file = session / "automatic_start.json"
    if start_file.exists() or start_file.is_symlink():
        raise ValueError("automatic start file must be new")
    if not isinstance(env, dict) or not all(isinstance(k, str) and isinstance(v, str) for k, v in env.items()):
        raise ValueError("explicit environment required")
    if any(bootstrap.nonce in value for value in env.values()):
        raise ValueError("frontend nonce cannot appear in environment")
    with create_bootstrap_file(bootstrap, runtime) as bootstrap_file:
        fd = bootstrap_file.fileno()
        argv = [sys.executable, "-m", "services.operator_entry", "--bootstrap-fd", str(fd),
            "--enable-integrated-candidate", "--ui-diagnostic", str(session / 'operator_ui.json'),
            "--ros-args", "-p", "show_gui:=false",
            "-p", "enable_live_intents:=true", "-p", "allow_close_stationary:=true",
            "--params-file", str(resolved_config), "-p", "automatic_mode:=true",
            "-p", "automatic_start_file:=" + str(start_file),
            "-p", "automatic_runner_session:=" + bootstrap.session,
            "-p", "automatic_packet:=" + bootstrap.packet]
        # pass_fds holds the anonymous read-only descriptor until child exec;
        # the nonce is never present in argv or the supplied environment.
        return popen(argv, cwd=str(app), env=env, stdout=stdout, stderr=stderr,
                     pass_fds=(fd,), close_fds=True, start_new_session=True)
