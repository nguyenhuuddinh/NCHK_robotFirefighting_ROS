"""Inspect/signal one runner-owned process group; no process-name matching.

Exit 0 means live members, 3 means absent/zombie-only, 2 means invalid/unknown.
The caller still waits on its child to preserve the real exit status.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time


def group_members(pgid):
    members = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            # comm may contain spaces/parentheses; fields after its last ')' are fixed.
            raw = (entry / 'stat').read_text()
            fields = raw[raw.rfind(')') + 2:].split()
            if int(fields[2]) != pgid:
                continue
            members.append(dict(pid=int(entry.name), ppid=int(fields[1]),
                                pgid=int(fields[2]), sid=int(fields[3]),
                                state=fields[0], start_ticks=int(fields[19])))
        except (FileNotFoundError, ProcessLookupError):
            continue  # Exited while collecting this snapshot.
    return sorted(members, key=lambda item: item['pid'])


def inspect(pgid):
    if pgid <= 1 or pgid == os.getpgrp():
        raise ValueError('Refusing invalid/self process group')
    members = group_members(pgid)
    # start_group uses setsid, so PGID and SID must both equal the saved PID.
    if any(item['sid'] != pgid for item in members):
        raise ValueError('Group is not the runner-created setsid session')
    live = [item for item in members if item['state'] not in ('Z', 'X')]
    return members, live


def leader_exited(pid):
    """A shell may wait only when its child is gone or waiting to be reaped."""
    if pid <= 1 or pid == os.getpgrp():
        raise ValueError('Refusing invalid/self process group')
    try:
        raw = Path(f'/proc/{pid}/stat').read_text()
    except (FileNotFoundError, ProcessLookupError):
        return True
    fields = raw[raw.rfind(')') + 2:].split()
    if int(fields[2]) != pid or int(fields[3]) != pid:
        raise ValueError('Leader no longer belongs to the owned setsid session')
    return fields[0] in ('Z', 'X')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', type=int, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--phase', default='poll')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--signal', choices=('SIGINT', 'SIGTERM', 'SIGKILL'))
    mode.add_argument('--leader-exited', action='store_true')
    parser.add_argument('--quiet', action='store_true')
    parser.add_argument('--leader-only', action='store_true', help='Initial SIGINT via live session leader; absent leader falls back to owned group')
    args = parser.parse_args()
    if args.leader_only and args.signal != 'SIGINT':
        parser.error('--leader-only is only valid for initial SIGINT')
    record = dict(wall_ns=time.time_ns(), mono_ns=time.monotonic_ns(),
                  label=args.label, pgid=args.pid, phase=args.phase)
    rc = 2
    try:
        if args.leader_exited:
            return 0 if leader_exited(args.pid) else 3
        members, live = inspect(args.pid)
        record.update(members=members, live_pids=[item['pid'] for item in live])
        rc = 0 if live else 3
        record['result'] = 'LIVE' if live else ('ZOMBIE_ONLY' if members else 'ABSENT')
        if args.signal:
            record['requested_signal'] = args.signal
            if live:
                try:
                    if args.leader_only and any(m['pid'] == args.pid for m in live):
                        try:
                            os.kill(args.pid, signal.SIGINT)
                            record['signal_scope'] = 'LEADER'
                        except ProcessLookupError:
                            # The leader can disappear after inspection while
                            # descendants still need cleanup. Revalidate SID
                            # before falling back; leader loss is not group exit.
                            members, live = inspect(args.pid)
                            record.update(members=members, live_pids=[m['pid'] for m in live])
                            if not live:
                                raise
                            os.killpg(args.pid, signal.SIGINT)
                            record['signal_scope'] = 'GROUP'
                            record['leader_fallback'] = 'LEADER_EXITED_DURING_SIGNAL'
                    else:
                        os.killpg(args.pid, getattr(signal, args.signal))
                        record['signal_scope'] = 'GROUP'
                        if args.leader_only:
                            record['leader_fallback'] = 'LEADER_ABSENT_OR_ZOMBIE'
                    record['result'] = 'SIGNAL_SENT'
                    record['signal_rc'] = 0
                except ProcessLookupError:
                    record['result'] = 'GROUP_EXITED_BEFORE_SIGNAL'
                    record['signal_rc'] = 1
                    rc = 3
            else:
                record['signal_rc'] = None  # No syscall / no signal delivery claim.
    except (OSError, ValueError, IndexError) as exc:
        record.update(result='INSPECTION_OR_SIGNAL_ERROR', error=str(exc))
        rc = 2
    if not args.quiet or rc == 2:
        with (args.out / 'cleanup_groups.jsonl').open('a') as stream:
            stream.write(json.dumps(record, sort_keys=True) + '\n')
    if args.signal:
        # Keep the old evidence vocabulary only for signals actually accepted by
        # kill/killpg. Acceptance cannot prove every snapshot member received it.
        field = 'SIGNAL' if record['result'] == 'SIGNAL_SENT' else 'REQUEST'
        with (args.out / 'cleanup_signals.log').open('a') as stream:
            stream.write(f"WALL_NS={record['wall_ns']} LABEL={args.label} PID={args.pid} "
                         f"{field}={args.signal} RESULT={record['result']} "
                         f"SCOPE={record.get('signal_scope', 'NONE')} SYSCALL_RC={record.get('signal_rc', 'NA')}\n")
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
