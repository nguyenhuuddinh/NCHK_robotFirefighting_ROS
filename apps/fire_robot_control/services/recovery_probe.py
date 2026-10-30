"""Read-only cleanup inspection shared by laptop and the SSH stdin probe.

No ROS imports, signals, device access, clock changes or robot starts. A previous
runner error remains an error; this probe only establishes that a retry is safe.
"""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import uuid


PATTERN = ('[s]erial_bridge_node|[n]ative_mjpeg_camera|[b]t_navigator|'
           '[f]ire_robot_component_container_isolated|'
           '[f]ire_mission_supervisor|[f]ire_pulse_controller|[y]olo_observer|[c]ontroller_server|'
           '[s]afety_watchdog|[r]5_startup_health|[f]ire_robot_operator|'
           '[r]5_pi_session.py|[a]pp_pi_entry.py|[a]pp_fire_entry.py|[s]ervices.mapping_runner|[o]perational_entry.py|'
           '[s]ervices.operator_entry|[s]ervices.operator_process_channel|'
           '[s]lam_toolbox|[t]eleop_twist_keyboard')

# A fresh STOP session also needs the I/O descendants absent. Historical
# nonzero exits never authorize stopping a process that is still alive.
IDLE_PATTERN = PATTERN + ('|[o]dom_to_tf_broadcaster|[s]can_qos_relay|[c]amsense_x1_node|'
    '[r]obot_state_publisher|[r]osbridge_websocket|[m]icro_ros_agent|[u]sb_cam_node|'
    '[f]ire_robot_bringup|[m]ap_saver_cli|[i]r_approach_monitor|'
    '[s]ervices.mapping_monitor|[s]ervices.app_health_entry|[s]ervices.fire_health_entry|[i]r_range_preview')


def require(ok, code):
    if not ok:
        raise ValueError(code)


def unique(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'RECOVERY_DUPLICATE_JSON_KEY')
        result[key] = value
    return result


def nonfinite(_):
    raise ValueError('RECOVERY_NONFINITE_JSON')


def decode(raw):
    require(len(raw) <= 1024 * 1024, 'RECOVERY_JSON_TOO_LARGE')
    row = json.loads(raw, object_pairs_hook=unique, parse_constant=nonfinite)
    require(isinstance(row, dict), 'RECOVERY_OBJECT_REQUIRED')
    return row


def read_object(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as stream:
        import stat
        info = os.fstat(stream.fileno())
        require(stat.S_ISREG(info.st_mode) and info.st_uid == os.getuid(), 'RECOVERY_FILE_OWNER_OR_TYPE')
        raw = stream.read(1024 * 1024 + 1)
    return decode(raw), hashlib.sha256(raw).hexdigest()


def identity(session, packet):
    require(isinstance(session, str) and str(uuid.UUID(session)) == session, 'RECOVERY_SESSION_INVALID')
    require(isinstance(packet, str) and re.fullmatch('[0-9a-f]{64}', packet), 'RECOVERY_PACKET_INVALID')


def groups_clean(groups):
    require(isinstance(groups, dict) and bool(groups), 'RECOVERY_GROUP_PROOF_MISSING')
    pids = []
    for row in groups.values():
        require(isinstance(row, dict), 'RECOVERY_GROUP_INVALID')
        pid = row.get('pid')
        require(type(pid) is int and pid > 1, 'RECOVERY_GROUP_PID_INVALID')
        require(row.get('done') is True and row.get('phase') == 'GROUP_EXITED'
                and row.get('group_exit_confirmed') is True and row.get('forced') is False
                and type(row.get('leader_rc')) is int and row['leader_rc'] == 0 and row.get('error') is None
                and row.get('live_pids') == [], 'RECOVERY_GROUP_NOT_CLEAN')
        pids.append(pid)
    require(len(set(pids)) == len(pids), 'RECOVERY_GROUP_PID_DUPLICATE')
    return pids


def child_clean(row):
    require(isinstance(row, dict) and row.get('complete') is True and row.get('clean') is True,
            'RECOVERY_CHILD_PROOF_MISSING')
    require(isinstance(row.get('roles'), dict), 'RECOVERY_CHILD_ROLES_MISSING')
    for role in row['roles'].values():
        require(isinstance(role, dict) and role.get('clean') is True
                and type(role.get('started')) is int and role['started'] > 0
                and type(role.get('terminal')) is int and role['terminal'] == role['started'] and role.get('missing') == []
                and role.get('errors') == [] and role.get('internal_signal_fallback') is False,
                'RECOVERY_CHILD_NOT_CLEAN')


def remote_cleanup(row):
    require(isinstance(row, dict) and row.get('remote_groups_exited') is True
            and row.get('children_clean') is True and row.get('forced') is False,
            'RECOVERY_PI_CLEANUP_MISSING')
    pids = groups_clean(row.get('groups'))
    child_clean(row.get('child_audit'))
    require(bool(row['child_audit']['roles']), 'RECOVERY_PI_CHILDREN_MISSING')
    return pids


def absent(pids, proc=Path('/proc')):
    # Refuse any live member of a recorded PGID/SID, including a reused PID.
    require(all(type(pid) is int and pid > 1 and pid != os.getpgrp() for pid in pids),
            'RECOVERY_PID_INVALID')
    for entry in proc.iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            raw = (entry / 'stat').read_text()
        except (FileNotFoundError, ProcessLookupError):
            continue
        fields = raw[raw.rfind(')') + 2:].split()
        require(len(fields) >= 20, 'RECOVERY_PROC_STAT_INVALID')
        if (int(fields[2]) in pids or int(fields[3]) in pids or int(entry.name) in pids):
            require(fields[0] in ('Z', 'X'), 'RECOVERY_PROCESS_STILL_LIVE')


def no_robot_processes(run=subprocess.run, *, pattern=PATTERN):
    result = run(['pgrep', '-af', pattern], capture_output=True, text=True, timeout=3, check=False)
    require(result.returncode == 1, 'RECOVERY_ROBOT_PROCESS_OR_SNAPSHOT_UNKNOWN')


def lock_existing(path):
    fd = os.open(path, os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        import stat
        info = os.fstat(fd)
        require(stat.S_ISREG(info.st_mode) and info.st_uid == os.getuid(), 'RECOVERY_LOCK_INVALID')
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return fd
    except BaseException:
        os.close(fd)
        raise


def probe_pi(binding):
    # All paths are fixed or strictly validated. The pairing secret is read
    # only inside this process and is never returned or included in argv.
    identity(binding.get('session'), binding.get('packet'))
    require(set(binding) == {'session', 'packet', 'boot_id', 'runner_pid', 'runner_start_ticks', 'root'},
            'RECOVERY_BINDING_FIELDS')
    require(re.fullmatch('/home/pi/qa_logs/r5_automatic_[0-9]{8}_[0-9]{6}', binding['root']),
            'RECOVERY_PI_ROOT_INVALID')
    require(type(binding['runner_start_ticks']) is int and binding['runner_start_ticks'] > 0,
            'RECOVERY_RUNNER_IDENTITY_INVALID')
    root = Path(binding['root'])
    require(not root.is_symlink() and root.resolve() == root, 'RECOVERY_PI_ROOT_SYMLINK')
    require(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == binding['boot_id'],
            'RECOVERY_PI_BOOT_CHANGED')
    pairing, _ = read_object(root / 'pairing.json')
    require(all(pairing.get(k) == v for k, v in binding.items()), 'RECOVERY_PI_BINDING_MISMATCH')
    fd = lock_existing(Path.home() / '.cache/fire_robot_r5_pi.lock')
    try:
        done, digest = read_object(root / 'DONE.json')
        require(done.get('session') == binding['session'] and done.get('phase') == 'DONE'
                and done.get('live') is False, 'RECOVERY_PI_NOT_DONE')
        pids = remote_cleanup(done.get('cleanup'))
        absent(pids + [binding['runner_pid']])
        no_robot_processes()
        return dict(binding=binding, done_sha256=digest, cleanup=done['cleanup'],
                    original_runner_error=done.get('error'), processes_absent=True,
                    common_lock_free=True, boot_matches=True)
    finally:
        os.close(fd)


def remote_main(binding):
    try:
        result = probe_pi(binding)
        print(json.dumps(dict(ok=True, proof=result), allow_nan=False))
    except Exception as exc:
        # Do not print arbitrary remote file contents or authentication data.
        code = str(exc) if isinstance(exc, ValueError) and str(exc).startswith('RECOVERY_') else type(exc).__name__
        print(json.dumps(dict(ok=False, error=code)))


def launch_pids(folder):
    """Read owned launch identities, never signal them or accept old STOP."""
    paths = sorted(folder.glob('*.log'))
    require(len(paths) <= 32, 'RECOVERY_LOG_INVENTORY_INVALID')
    pids = set()
    total = 0
    pattern = re.compile(rb'^\[INFO\] \[[A-Za-z0-9_-]+\]: process started with pid \[([0-9]+)\]$', re.M)
    for path in paths:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, 'rb') as stream:
            import stat
            info = os.fstat(stream.fileno())
            require(stat.S_ISREG(info.st_mode) and info.st_uid == os.getuid(), 'RECOVERY_FILE_OWNER_OR_TYPE')
            raw = stream.read(16 * 1024 * 1024 + 1)
        total += len(raw)
        require(len(raw) <= 16 * 1024 * 1024 and total <= 64 * 1024 * 1024, 'RECOVERY_LOG_TOO_LARGE')
        pids.update(int(pid) for pid in pattern.findall(raw))
    require(len(pids) <= 1024 and all(pid > 1 for pid in pids), 'RECOVERY_PID_INVALID')
    return sorted(pids)


def quiescent_pi(binding, current, pairing_sha):
    """Current process absence permits ONLY a new, independently verified STOP.

    Do not edit/reinterpret the failed DONE receipt. The strict read-only
    RECHECK and the fresh STOP's own cleanup still use remote_cleanup().
    """
    root = Path(binding['root'])
    fd = lock_existing(Path.home() / '.cache/fire_robot_r5_pi.lock')
    try:
        pids = {binding['runner_pid']}
        done_sha = None
        try:
            done, done_sha = read_object(root / 'DONE.json')
        except FileNotFoundError:
            done = None
        if done is not None:
            require(done.get('session') == binding['session'] and done.get('phase') == 'DONE'
                    and done.get('live') is False, 'RECOVERY_PI_NOT_DONE')
            cleanup = done.get('cleanup')
            require(isinstance(cleanup, dict) and isinstance(cleanup.get('groups'), dict),
                    'RECOVERY_PI_PROOF_INVALID')
            for group in cleanup['groups'].values():
                require(isinstance(group, dict) and type(group.get('pid')) is int
                        and group['pid'] > 1, 'RECOVERY_GROUP_PID_INVALID')
                pids.add(group['pid'])
        folders = sorted(root.glob('generation_[0-9]*'))
        require(len(folders) <= 4, 'RECOVERY_LOG_INVENTORY_INVALID')
        for folder in folders:
            require(not folder.is_symlink() and folder.resolve() == folder, 'RECOVERY_PI_ROOT_SYMLINK')
            pids.update(launch_pids(folder))
        absent(sorted(pids))
        no_robot_processes(pattern=IDLE_PATTERN)
        require(Path('/proc/sys/kernel/random/boot_id').read_text().strip() == current,
                'RECOVERY_PI_BOOT_CHANGED')
        return dict(binding=binding, current_boot_id=current, mode='SAME_BOOT_QUIESCENT_ONLY',
                    pairing_sha256=pairing_sha, historical_done_sha256=done_sha,
                    pids_checked=sorted(pids), processes_absent=True, common_lock_free=True,
                    boot_stable=True, scope='QUIESCENT_ONLY_REQUIRES_NEW_STOP')
    finally:
        os.close(fd)


def probe_pi_idle(binding, expected_boot=None):
    """Authorize only a new STOP-only attempt, never historical STOP success.

    An authenticated Pi reboot retires its old process identities, but says
    nothing about the actuators. A fresh telemetry STOP is still mandatory.
    """
    identity(binding.get('session'), binding.get('packet'))
    require(set(binding) == {'session', 'packet', 'boot_id', 'runner_pid', 'runner_start_ticks', 'root'},
            'RECOVERY_BINDING_FIELDS')
    require(str(uuid.UUID(binding['boot_id'])) == binding['boot_id'], 'RECOVERY_PI_BOOT_INVALID')
    require(type(binding['runner_pid']) is int and binding['runner_pid'] > 1
            and type(binding['runner_start_ticks']) is int and binding['runner_start_ticks'] > 0,
            'RECOVERY_RUNNER_IDENTITY_INVALID')
    require(re.fullmatch('/home/pi/qa_logs/r5_automatic_[0-9]{8}_[0-9]{6}', binding['root']),
            'RECOVERY_PI_ROOT_INVALID')
    root = Path(binding['root'])
    require(not root.is_symlink() and root.resolve() == root, 'RECOVERY_PI_ROOT_SYMLINK')
    boot = Path('/proc/sys/kernel/random/boot_id')
    current = boot.read_text().strip()
    require(str(uuid.UUID(current)) == current, 'RECOVERY_PI_BOOT_INVALID')
    require(expected_boot is None or current == expected_boot, 'RECOVERY_PI_BOOT_CHANGED')
    pairing, pairing_sha = read_object(root / 'pairing.json')
    require(all(pairing.get(k) == v for k, v in binding.items()), 'RECOVERY_PI_BINDING_MISMATCH')
    if current == binding['boot_id']:
        try:
            proof = probe_pi(binding)
        except ValueError as exc:
            # Only historical cleanup failures may use a current absence
            # proof. Identity, boot, live-process and lock failures never do.
            if str(exc) not in {'RECOVERY_PI_CLEANUP_MISSING', 'RECOVERY_GROUP_PROOF_MISSING',
                    'RECOVERY_GROUP_NOT_CLEAN', 'RECOVERY_CHILD_PROOF_MISSING',
                    'RECOVERY_CHILD_NOT_CLEAN', 'RECOVERY_PI_CHILDREN_MISSING'}:
                raise
            return quiescent_pi(binding, current, pairing_sha)
        except FileNotFoundError as exc:
            if exc.filename not in (None, str(root / 'DONE.json')):
                raise
            return quiescent_pi(binding, current, pairing_sha)
        require(boot.read_text().strip() == current, 'RECOVERY_PI_BOOT_CHANGED')
        return dict(binding=binding, current_boot_id=current, mode='SAME_BOOT_CLEANUP',
                    pairing_sha256=pairing_sha, historical_cleanup=proof,
                    processes_absent=True, common_lock_free=True, boot_stable=True)
    fd = lock_existing(Path.home() / '.cache/fire_robot_r5_pi.lock')
    try:
        # Include I/O descendants which can outlive a manually launched runner.
        no_robot_processes(pattern=IDLE_PATTERN)
        require(boot.read_text().strip() == current, 'RECOVERY_PI_BOOT_CHANGED')
        return dict(binding=binding, current_boot_id=current, mode='NEW_BOOT_QUIESCENT_ONLY',
                    pairing_sha256=pairing_sha, processes_absent=True, common_lock_free=True,
                    boot_stable=True)
    finally:
        os.close(fd)


def remote_idle_main(binding, expected_boot, nonce):
    try:
        proof = probe_pi_idle(binding, expected_boot)
        proof['request_nonce'] = nonce
        print(json.dumps(dict(ok=True, proof=proof), allow_nan=False))
    except Exception as exc:
        code = str(exc) if isinstance(exc, ValueError) and re.fullmatch('RECOVERY_[A-Z0-9_]+', str(exc)) else type(exc).__name__
        print(json.dumps(dict(ok=False, error=code)))
