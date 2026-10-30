"""App-only ownership journal for recovery when a final receipt is missing.

No secret, STOP claim, signals or device operations. A journal can authorize
only a fresh STOP attempt after current process/lock checks, never RECHECK.
"""
from pathlib import Path
import re

from . import recovery_probe as proof


def public_binding(binding, packet):
    fields = {'session', 'packet', 'boot_id', 'runner_pid', 'runner_start_ticks', 'root'}
    proof.require(isinstance(binding, dict), 'RECOVERY_REMOTE_BINDING_MISSING')
    row = {key: value for key, value in binding.items() if key != 'secret'}
    proof.require(set(row) == fields and row.get('packet') == packet, 'RECOVERY_REMOTE_BINDING_MISSING')
    proof.identity(row['session'], row['packet'])
    proof.identity(row['boot_id'], row['packet'])
    proof.require(type(row['runner_pid']) is int and row['runner_pid'] > 1
                  and type(row['runner_start_ticks']) is int and row['runner_start_ticks'] > 0
                  and isinstance(row['root'], str)
                  and re.fullmatch('/home/pi/qa_logs/r5_automatic_[0-9]{8}_[0-9]{6}', row['root']),
                  'RECOVERY_REMOTE_BINDING_MISSING')
    return row


def process_identity(pid):
    proof.require(type(pid) is int and pid > 1, 'RECOVERY_PID_INVALID')
    raw = (Path('/proc') / str(pid) / 'stat').read_text()
    fields = raw[raw.rfind(')') + 2:].split()
    proof.require(len(fields) >= 20 and fields[19].isdecimal(), 'RECOVERY_PROC_STAT_INVALID')
    return dict(pid=pid, start_ticks=int(fields[19]))


def journal_groups(marker, out):
    try:
        row, _ = proof.read_object(out / 'recovery_ownership.json')
    except FileNotFoundError as exc:
        raise ValueError('RECOVERY_OWNERSHIP_MISSING') from exc
    proof.require(row.get('schema_version') == 1 and row.get('session') == marker['session']
                  and row.get('packet') == marker['packet']
                  and row.get('kind') in ('mapping', 'fire', 'verify_stop'),
                  'RECOVERY_LOCAL_BINDING_MISMATCH')
    binding = public_binding(row.get('pi', {}), marker['packet'])
    pids = []
    if row.get('stage') == 'LAPTOP_STARTED':
        process = row.get('laptop_runner')
        proof.require(isinstance(process, dict) and type(process.get('pid')) is int
                      and process['pid'] > 1 and type(process.get('start_ticks')) is int
                      and process['start_ticks'] > 0, 'RECOVERY_OWNERSHIP_INCOMPLETE')
        pids.append(process['pid'])
    else:
        proof.require(row.get('stage') == 'PAIRED' and row.get('laptop_runner') is None,
                      'RECOVERY_OWNERSHIP_INCOMPLETE')
    # A missing file does not make a contradictory surviving receipt valid.
    for path in (out / 'desktop_receipt.json', out / 'laptop/receipt.json'):
        try:
            existing, _ = proof.read_object(path)
        except FileNotFoundError:
            continue
        proof.require(existing.get('session') == marker['session']
                      and existing.get('packet') == marker['packet'], 'RECOVERY_LOCAL_BINDING_MISMATCH')
        if path.parent.name == 'laptop':
            remote = existing.get('remote', {}).get('binding')
            if remote is not None:
                proof.require(public_binding(remote, marker['packet']) == binding,
                              'RECOVERY_REMOTE_BINDING_MISSING')
            for group in existing.get('local', {}).get('processes', {}).values():
                proof.require(isinstance(group, dict) and type(group.get('pid')) is int
                              and group['pid'] > 1, 'RECOVERY_GROUP_PID_INVALID')
                pids.append(group['pid'])
    folder = out / 'laptop'
    if folder.exists():
        proof.require(not folder.is_symlink() and folder.resolve() == folder, 'RECOVERY_LOG_PATH_INVALID')
        pids.extend(proof.launch_pids(folder))
    pids = sorted(set(pids))
    proof.absent(pids)
    return binding, pids
