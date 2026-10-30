"""Explicit STOP-only recovery preparation. Never called by read-only RECHECK."""
import hashlib
import os
import re
import stat
import uuid
from pathlib import Path
from . import recovery_probe as probe


def forced_nav_child(local, process, profile, groups, mission_log):
    """Recognize one specific audited Nav2 SIGINT->SIGTERM cleanup.

    This permits only a fresh STOP-only verification, never marks the old
    STOP or old cleanup as successful. Every other forced cleanup stays blocked.
    """
    # Do not generalize this exception to mapping, unknown processes, or other
    # failures: the new STOP-only session must independently prove safety.
    probe.require(profile == "R5_AUTOMATIC" and set(groups) == {
        "startup_health", "nav", "rviz", "yolo", "ir_monitor", "mission", "operator"}
        and local.get("requested") is True and local.get("done") is True
        and type(local.get("stop_confirmed")) is bool and local.get("clean") is False
        and local.get("group_cleanup_clean") is False,
        "RECOVERY_FORCED_NAV_PROOF_INVALID")
    errors = local.get("errors")
    confirmed = local['stop_confirmed']
    allowed = {"PROCESS_UNCLEAN:nav", "CHILD_LOG_AUDIT_FAILED"}
    if not confirmed:
        allowed |= {"OPERATOR_STOP_UNCONFIRMED", "PROCESS_UNCLEAN:operator"}
    probe.require(isinstance(errors, list) and all(isinstance(e, str) for e in errors)
                  and len(errors) == len(set(errors)) and set(errors) == allowed,
                  "RECOVERY_FORCED_NAV_PROOF_INVALID")
    for name, row in groups.items():
        if name != "nav":
            probe.require(isinstance(row, dict) and row.get("error") is None
                          and row.get("leader_rc") == (2 if name == "operator" and not confirmed else 0),
                          "RECOVERY_FORCED_NAV_PROOF_INVALID")
    audit = local.get("child_log_audit")
    probe.require(isinstance(audit, dict) and audit.get("complete") is True
                  and audit.get("clean") is False and isinstance(audit.get("roles"), dict),
                  "RECOVERY_FORCED_NAV_PROOF_INVALID")
    roles = audit["roles"]
    probe.require(set(roles) == {"nav", "yolo", "mission"}, "RECOVERY_FORCED_NAV_PROOF_INVALID")
    nav = roles.get("nav")
    probe.require(isinstance(nav, dict) and nav.get("clean") is False
                  and nav.get("started") == 1 and nav.get("terminal") == 0
                  and nav.get("internal_signal_fallback") is True,
                  "RECOVERY_FORCED_NAV_PROOF_INVALID")
    missing, fallback = nav.get("missing"), nav.get("fallbacks")
    probe.require(isinstance(missing, list) and len(missing) == 1
                  and isinstance(fallback, list) and len(fallback) == 1
                  and isinstance(missing[0], dict) and isinstance(fallback[0], dict),
                  "RECOVERY_FORCED_NAV_PROOF_INVALID")
    child, escalation = missing[0], fallback[0]
    pid, label = child.get("pid"), child.get("label")
    probe.require(type(pid) is int and pid > 1 and isinstance(label, str)
                  and re.fullmatch(r"fire_robot_component_container_isolated-[0-9]+", label)
                  and escalation.get("pid") == pid and escalation.get("label") == label
                  and escalation.get("from_signal") == "SIGINT" and escalation.get("to_signal") == "SIGTERM"
                  and type(escalation.get("timeout_s")) in (int, float) and escalation["timeout_s"] == 5.0
                  and nav.get("errors") == ["CHILD_SIGNAL_ESCALATION:" + label + ":SIGINT_TO_SIGTERM"],
                  "RECOVERY_FORCED_NAV_PROOF_INVALID")
    signals = process.get("signals")
    probe.require(process.get("leader_rc") == -15 and process.get("error") is None
                  and isinstance(signals, list) and len(signals) == 2
                  and all(isinstance(s, dict) for s in signals)
                  and [(s.get("signal"), s.get("scope")) for s in signals]
                      == [("SIGINT", "LEADER"), ("SIGTERM", "GROUP")],
                  "RECOVERY_FORCED_NAV_PROOF_INVALID")
    probe.child_clean(dict(complete=True, clean=True, roles={'yolo': roles['yolo']}))
    mission = roles['mission']
    extra_pids = []
    if mission.get('clean') is True:
        probe.child_clean(dict(complete=True, clean=True, roles={'mission': mission}))
    else:
        probe.require(not confirmed, 'RECOVERY_MISSION_PROOF_INVALID')
        # Accept this known failed shutdown only as evidence that a NEW STOP
        # may be attempted. It is never evidence that the robot stopped.
        probe.require(mission.get('started') == 9 and mission.get('terminal') == 9
                      and mission.get('missing') == [] and mission.get('fallbacks') == []
                      and mission.get('internal_signal_fallback') is False
                      and mission.get('errors') == ['CHILD_DIED:fire_pulse_controller-8:2'],
                      'RECOVERY_MISSION_PROOF_INVALID')
        fd = os.open(mission_log, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, 'rb') as stream:
            info = os.fstat(stream.fileno())
            probe.require(stat.S_ISREG(info.st_mode) and info.st_uid == os.getuid(),
                          'RECOVERY_MISSION_LOG_INVALID')
            raw = stream.read(1024 * 1024 + 1)
        probe.require(len(raw) <= 1024 * 1024, 'RECOVERY_MISSION_LOG_INVALID')
        from operational_logs import audit_launch_children
        text = raw.decode('utf-8')
        probe.require(audit_launch_children(text) == mission, 'RECOVERY_MISSION_LOG_MISMATCH')
        starts = re.findall(r'^\[INFO\] \[fire_pulse_controller-8\]: process started with pid \[(\d+)\]$', text, re.M)
        ends = re.findall(r'^\[ERROR\] \[fire_pulse_controller-8\]: process has died \[pid (\d+), exit code 2, cmd .*\]\.?$', text, re.M)
        signal_at = text.find("[INFO] [fire_pulse_controller-8]: sending signal 'SIGINT'")
        drain_at = text.find('ACTOR_DRAIN ')
        exit_at = text.find('[ERROR] [fire_pulse_controller-8]: process has died')
        probe.require(len(starts) == len(ends) == 1 and starts == ends and int(starts[0]) > 1
                      and 0 <= signal_at < drain_at < exit_at
                      and 'ACTUATOR_OR_MOTION_STOP_UNCONFIRMED' in text[drain_at:exit_at],
                      'RECOVERY_MISSION_LOG_INVALID')
        extra_pids.append(int(starts[0]))
    return [pid, *extra_pids]


def old_groups(marker, out, *, return_pids=False):
    try:
        desktop, _ = probe.read_object(out / "desktop_receipt.json")
        row, _ = probe.read_object(out / "laptop/receipt.json")
    except FileNotFoundError:
        from .recovery_ownership import journal_groups
        binding, pids = journal_groups(marker, out)
        return (binding, pids) if return_pids else binding
    for obj in (desktop, row):
        probe.require(obj.get("session") == marker["session"] and obj.get("packet") == marker["packet"],
                      "RECOVERY_LOCAL_BINDING_MISMATCH")
    probe.require(row.get("profile") in ("DESKTOP_MAPPING", "R5_AUTOMATIC", "DESKTOP_STOP_RECOVERY"),
                  "RECOVERY_PROFILE_UNSUPPORTED")
    local = row.get("local", {})
    groups = local.get("processes")
    probe.require(isinstance(groups, dict) and bool(groups), "RECOVERY_GROUP_PROOF_MISSING")
    pids = []
    for name, process in groups.items():
        # Historical unconfirmed STOP may have returned rc2. It remains FAIL.
        # Current absence, not that return code, permits a new STOP-only check.
        probe.require(isinstance(process, dict) and process.get("group_exit_confirmed") is True and process.get("done") is True
                      and process.get("phase") == "GROUP_EXITED" and process.get("live_pids") == [],
                      "RECOVERY_GROUP_NOT_CLEAN")
        if process.get("forced") is True and name == "nav":
            pids.extend(forced_nav_child(local, process, row.get("profile"), groups, out / 'laptop/mission.log'))
        else:
            probe.require(process.get("forced") is False, "RECOVERY_GROUP_NOT_CLEAN")
        pids.append(process.get("pid"))
    probe.require(all(type(pid) is int and pid > 1 for pid in pids) and len(set(pids)) == len(pids),
                  "RECOVERY_GROUP_PID_INVALID")
    probe.absent(pids)
    binding = row.get("remote", {}).get("binding")
    probe.require(isinstance(binding, dict) and binding.get("packet") == marker["packet"]
                  and "secret" not in binding, "RECOVERY_REMOTE_BINDING_MISSING")
    return (binding, pids) if return_pids else binding


def prepare(state_dir, *, remote=None, lock_path=None):
    from .recovery import pending_record, remote_idle_proof
    marker, digest, out = pending_record(state_dir)
    fd = probe.lock_existing(lock_path or Path.home() / ".cache/fire_robot_r5_laptop.lock")
    try:
        binding = old_groups(marker, out)
        probe.no_robot_processes(pattern=probe.IDLE_PATTERN)
        pi = (remote or remote_idle_proof)(binding)
        probe.require(old_groups(marker, out) == binding, 'RECOVERY_REMOTE_BINDING_MISSING')
        probe.no_robot_processes(pattern=probe.IDLE_PATTERN)
        probe.require(pending_record(state_dir)[1] == digest, "RECOVERY_MARKER_CHANGED")
        return dict(marker_sha256=digest, pi=pi)
    finally:
        import os
        os.close(fd)


def audit_new_stop(state_dir, new_out, *, remote=None, idle_remote=None, lock_path=None):
    from .recovery import pending_record, remote_proof, remote_idle_proof
    from .session_outcome import outcome
    import os
    import time
    marker, digest, old_out = pending_record(state_dir)
    new_out = Path(new_out)
    probe.require(new_out.resolve(strict=True) == new_out and new_out.parent == Path(state_dir).resolve() and new_out != old_out
                  and not Path(new_out).is_symlink(), "RECOVERY_LOG_PATH_INVALID")
    desktop, desktop_sha = probe.read_object(new_out / "desktop_receipt.json")
    receipt, receipt_sha = probe.read_object(new_out / "laptop/receipt.json")
    probe.identity(receipt.get("session"), receipt.get("packet"))
    probe.require(receipt.get("profile") == "DESKTOP_STOP_RECOVERY"
                  and desktop.get("recovery_marker_sha256") == digest
                  and desktop.get("session") == receipt["session"]
                  and desktop.get("packet") == receipt["packet"]
                  and desktop.get("robot_stop_confirmed") is True
                  and desktop.get("cleanup_clean") is True
                  and outcome(receipt, receipt["session"], receipt["packet"]) == (True, True),
                  "RECOVERY_NEW_STOP_NOT_CONFIRMED")
    new_binding = receipt["remote"].get("binding", {})
    boot = desktop.get("recovery_boot_id")
    probe.require(isinstance(boot, str) and re.fullmatch('[0-9a-f-]{36}', boot)
                  and str(uuid.UUID(boot)) == boot and new_binding.get("boot_id") == boot
                  and new_binding.get("packet") == receipt["packet"]
                  and 'secret' not in new_binding, "RECOVERY_PI_BOOT_CHANGED")
    fd = probe.lock_existing(lock_path or Path.home() / ".cache/fire_robot_r5_laptop.lock")
    try:
        binding, old_pids = old_groups(marker, old_out, return_pids=True)
        probe.require(new_binding.get("session") != binding.get("session"), "RECOVERY_NEW_STOP_NOT_CONFIRMED")
        probe.absent(probe.groups_clean(receipt["local"]["processes"]))
        probe.no_robot_processes(pattern=probe.IDLE_PATTERN)
        new_pi = (remote or remote_proof)(new_binding)
        old_idle = (idle_remote or remote_idle_proof)(binding, expected_boot=boot)
        probe.require(old_groups(marker, old_out) == binding, 'RECOVERY_REMOTE_BINDING_MISSING')
        probe.absent(probe.groups_clean(receipt["local"]["processes"]))
        probe.no_robot_processes(pattern=probe.IDLE_PATTERN)
        probe.require(pending_record(state_dir)[1] == digest, "RECOVERY_MARKER_CHANGED")
        return dict(session=marker["session"], packet=marker["packet"], marker_sha256=digest,
                    checked_mono=time.monotonic(), remote=new_pi, old_pi_idle=old_idle,
                    recheck_pids=probe.groups_clean(receipt["local"]["processes"]) + old_pids,
                    new_stop_proof=dict(session=receipt["session"], packet=receipt["packet"],
                        log_dir=str(new_out), desktop_receipt_sha256=desktop_sha,
                        laptop_receipt_sha256=receipt_sha),
                    scope="CURRENT_STOP_VERIFIED_BY_NEW_SESSION_OLD_RESULT_UNCHANGED")
    finally:
        os.close(fd)
