"""Reproducible desktop candidate. Never edits the sealed R5 source."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError("R5 integration anchor changed: " + old[:60])
    return text.replace(old, new, 1)


def patch_runner(text):
    changes = [
        ("p.add_argument('--automatic-token')", "p.add_argument('--desktop-fd',type=int,required=True)\n    p.add_argument('--automatic-token')"),
        ("a.out.mkdir(mode=0o700)", "a.out.mkdir(mode=0o700)\n    from services.desktop_hooks import DesktopHooks\n    desktop=DesktopHooks(a.desktop_fd,ROOT,a.out)"),
        ("packet=packet_digest();preflight(ROOT,'laptop')", "packet=packet_digest();preflight(ROOT,'laptop')\n        if desktop.binding.packet!=packet:raise ValueError('DESKTOP_PACKET_MISMATCH')"),
        ("peer=SshPi(bind,a.remote_script);session=str(uuid.uuid4())", "peer=SshPi(bind,a.remote_script);session=desktop.binding.session"),
        ("local=LocalProcesses(a.out,session);stop=", "argv_by_role=desktop.commands(argv_by_role)\n    local=LocalProcesses(a.out,session,backend_factory=desktop.backend);stop="),
        ("try:local_result=local.drain(cause='LAPTOP_FINALLY') if local.processes else dict(clean=False,reason='NO_LOCAL_PROCESSES')", "try:\n                try:desktop.prepare_stop_monitor(local,remote_started=remote_started)\n                except Exception as monitor_error:print('STOP_MONITOR_START_FAILED:'+type(monitor_error).__name__,flush=True)\n                local_result=local.drain(cause='LAPTOP_FINALLY') if local.processes else dict(clean=False,reason='NO_LOCAL_PROCESSES')"),
        ("remote_result=peer.stop();atomic(a.out/'remote_cleanup.json',remote_result)", "from services.desktop_hooks import stop_peer_after_local\n            remote_result=stop_peer_after_local(local,peer);atomic(a.out/'remote_cleanup.json',remote_result)"),
        ("atomic(a.out/'receipt.json',dict(rc=rc,local=local_result,remote=remote_result,", "atomic(a.out/'receipt.json',dict(rc=rc,session=session,packet=packet,local=local_result,remote=remote_result,"),
    ]
    for old, new in changes:
        text = replace_once(text, old, new)
    # Recovery rebuilds only mission, but must use the SAME app wrappers as
    # initial startup. Calling R5 commands directly silently drops app fixes.
    text = replace_once(text,
        "lambda path,nonce,after_pulse:commands(a.map,a.rviz,gate2=True,automatic=True,\n"
        "                    auto_start=path,auto_session=nonce,auto_packet=packet,recovery=True,recovery_after_pulse=after_pulse),auto_start)",
        "lambda path,nonce,after_pulse:desktop.commands(commands(a.map,a.rviz,gate2=True,automatic=True,\n"
        "                    auto_start=path,auto_session=nonce,auto_packet=packet,recovery=True,recovery_after_pulse=after_pulse)),auto_start)")
    return text


def build(source, app, destination):
    source, app, destination = map(Path, (source, app, destination))
    source, app = source.resolve(strict=True), app.resolve(strict=True)
    if destination.resolve().is_relative_to(source):
        raise ValueError("APP_OUTPUT_INSIDE_IMMUTABLE_R5")
    manifest = source / "operational_packet.sha256"
    parent = hashlib.sha256(manifest.read_bytes()).hexdigest()
    if parent != "0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0":
        raise ValueError("UNREVIEWED_R5_PARENT")
    from .packet_archive import source_packet
    source_packet(source)
    if destination.exists():
        raise ValueError("Candidate destination already exists")
    destination.mkdir(parents=True)
    try:
        for line in manifest.read_text().splitlines():
            sha, name = line.split("  ", 1)
            path = (source / name).resolve(strict=True)
            if not path.is_relative_to(source.resolve()) or hashlib.sha256(path.read_bytes()).hexdigest() != sha:
                raise ValueError("Sealed source mismatch")
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
        desktop = destination / "desktop"
        shutil.copytree(app / "services", desktop / "services", ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        shutil.copytree(app / "config", desktop / "config")
        shutil.copytree(app / "native_rviz", desktop / "native_rviz", ignore=shutil.ignore_patterns("build", "__pycache__"))
        # CMake builds the Qt fixtures as well, so include their exact source
        # in the candidate manifest; no duplicate ROS package is installed.
        (desktop / "tests").mkdir()
        for name in ("qt_frontend_test.cpp", "qt_frontend_fixture.cpp", "qt_app_fixture.cpp"):
            shutil.copy2(app / "tests" / name, desktop / "tests" / name)
        generated = desktop / "generated"
        generated.mkdir()
        from .app_nav_params import write_params
        write_params(source / 'ros2_ws/src/fire_robot_navigation/config/nav2_params.yaml',
                     desktop / 'config/nav2_tuning.yaml', generated / 'app_nav2_params.yaml')
        from .app_operator_source import patch_operator
        operator_source = source / 'ros2_ws/src/fire_robot_mission/fire_robot_mission/operational_session.py'
        (generated / 'app_operational_session.py').write_text(patch_operator(operator_source.read_text()))
        mission_launch = (source / 'ros2_ws/src/fire_robot_mission/launch/operational_stack_gate2.launch.py').read_text()
        mission_launch = replace_once(mission_launch, 'from pathlib import Path',
            'from pathlib import Path\nimport shlex\nimport sys')
        mission_launch = replace_once(mission_launch,
            "executable=name,name=name,output='screen',",
            "executable=name,name=name,exec_name=name,output='screen',\n"
            "                prefix=(shlex.join([sys.executable,'-m','services.app_mission_node','--'])\n"
            "                        if name in ('live_nav_stop_gateway','nav2_cancel_guard') else None),")
        (generated / 'app_mission.launch.py').write_text(mission_launch)
        runner = patch_runner((source / "scripts/r5_laptop_session.py").read_text())
        runner = "from services.app_local_processes import AppLocalProcesses as LocalProcesses\n" + runner
        # Keep the imported alias above authoritative; do not edit the R5 module.
        runner = replace_once(runner, "from operational_entry import ROOT,LocalProcesses,",
                              "from operational_entry import ROOT,")
        runner = replace_once(runner, "from operational_peer import SshPi", "from services.app_peer import AppPeer as SshPi")
        runner = replace_once(runner,
            "if not local_result.get('clean') or not remote_result['confirmed']:rc=2",
            "if not local_result.get('clean') or not remote_result['confirmed'] or local_result.get('runtime_failure') is True:rc=2")
        runner = replace_once(runner, "if effect=='START_PI':peer.startup('START');remote_started=True",
            "if effect=='START_PI':desktop.prepare_start_monitor(local,lambda:stop[0]);peer.startup('START');remote_started=True")
        runner = replace_once(runner, "local.start(role,argv_by_role[role],launch_leader=role in ('nav','yolo','mission'))",
            "if role=='mission':desktop.handoff_monitor(local)\n                    local.start(role,argv_by_role[role],launch_leader=role in ('nav','yolo','mission'))")
        runner = replace_once(runner, "if role=='rviz':print('WAIT_INITIAL_POSE:", "if role=='rviz':desktop.wait_view(lambda:stop[0]);print('WAIT_INITIAL_POSE:")
        runner = replace_once(runner, "ros('r5_startup_health','--out'", "health_command('--packet',packet,'--out'")
        runner = "from services.fire_health_entry import health_command\n" + runner
        anchor = "atomic(a.out/'startup.json',state.diagnostics())"
        if runner.count(anchor) != 2:
            raise ValueError("R5 startup status anchors changed")
        runner = runner.replace(anchor, "atomic(a.out/'startup.json',dict(session=session,**state.diagnostics()))")
        runner = replace_once(runner, "try:desktop.prepare_stop_monitor(local,remote_started=remote_started)",
            "try:\n                    desktop.mark_stopping()\n                    desktop.prepare_stop_monitor(local,remote_started=remote_started)")
        (generated / "app_fire_entry.py").write_text(runner)
        pi = (source / "scripts/r5_pi_session.py").read_text()
        pi = replace_once(pi, "a.out.mkdir(mode=0o700)", "a.out.mkdir(mode=0o700)\n    from services.app_environment import record\n    record(a.out,ROOT,'pi',atomic)")
        pi = replace_once(pi, "controller=PiController(bind,start,begin,poll,health,time.monotonic())",
                         "controller=AppPiController(bind,start,begin,poll,health,time.monotonic());controller.stop_signal=app_stop")
        pi = replace_once(pi, "stop=[False];previous={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}",
                         "stop=[False];app_stop=AppStopSignal(stop);previous={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}")
        pi = replace_once(pi, "for sig in previous:signal.signal(sig,lambda *_:stop.__setitem__(0,True))",
                         "for sig in previous:signal.signal(sig,app_stop)")
        pi = replace_once(pi, "controller.phase in ('IO_STARTING','RUNNING_OBSERVE')",
                         "controller.phase in ('IO_STARTING','RUNNING_OBSERVE','WAIT_LAPTOP_DRAIN')")
        pi = replace_once(pi, "ros('r5_startup_health','--out'", "health_command('--out'")
        pi = "from services.app_pi_lifecycle import AppPiController, AppStopSignal\nfrom services.app_health_entry import health_command\n" + pi
        (generated / "app_pi_entry.py").write_text(pi)
        (destination / "desktop_parent.json").write_text(json.dumps({
            "schema_version": 1, "parent_packet": parent,
            "changes": "App-only generated laptop/Pi entrypoints; all 364 R5 parent source bytes unchanged",
        }, indent=2) + "\n")
        files = sorted(p for p in destination.rglob("*") if p.is_file())
        data = "".join(hashlib.sha256(p.read_bytes()).hexdigest() + "  " +
                       p.relative_to(destination).as_posix() + "\n" for p in files)
        (destination / "operational_packet.sha256").write_text(data)
        return hashlib.sha256(data.encode()).hexdigest()
    except BaseException:
        shutil.rmtree(destination)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("app", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    print(build(args.source, args.app, args.destination))


if __name__ == "__main__":
    main()
