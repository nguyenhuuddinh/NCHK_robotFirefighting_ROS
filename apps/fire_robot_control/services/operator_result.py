"""Bound runtime outcome separately from the unchanged owned STOP log proof."""
import json
import os
from pathlib import Path
import re

SCOPE = 'APP_OPERATOR_TERMINAL_RESULT_NOT_STOP_PROOF'


def fault_code(value):
    if value is None:
        return None
    return value if isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9_:/.-]{1,200}', value) else 'RUNTIME_FAULT_DETAIL_REDACTED'


def write_result(path, bootstrap, *, fault, rc, stop_confirmed):
    raw = Path('/proc/self/stat').read_text()
    birth = int(raw[raw.rfind(')') + 2:].split()[19])
    row = dict(schema_version=1, scope=SCOPE, session=bootstrap.session,
        packet=bootstrap.packet, generation=bootstrap.generation,
        pid=os.getpid(), process_start_ticks=birth, process_exit_code=rc,
        stop_confirmed=stop_confirmed, process_cleanup_ok=rc == 0 and stop_confirmed is True,
        runtime_fault=fault_code(fault), runtime_status='FAILED' if fault is not None or rc != 0 else 'SUCCEEDED')
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(row, stream, allow_nan=False)
        stream.write('\n')


def read_result(path, process, bootstrap):
    from .recovery_probe import read_object, require
    row, digest = read_object(path)
    require(set(row) == {'schema_version', 'scope', 'session', 'packet', 'generation', 'pid',
                        'process_start_ticks', 'process_exit_code', 'stop_confirmed', 'process_cleanup_ok',
                        'runtime_fault', 'runtime_status'}, 'APP_OPERATOR_RESULT_SCHEMA_INVALID')
    require(process.done and process.rc == 0 and not process.forced and process.error is None
            and process.report().get('group_exit_confirmed') is True,
            'APP_OPERATOR_PROCESS_UNCLEAN')
    require(type(row.get('schema_version')) is int and row['schema_version'] == 1 and row.get('scope') == SCOPE
            and row.get('session') == bootstrap.session and row.get('packet') == bootstrap.packet
            and row.get('generation') == bootstrap.generation
            and type(row.get('pid')) is int and row['pid'] == process.pid
            and type(row.get('process_start_ticks')) is int and row['process_start_ticks'] == process.leader_birth
            and type(row.get('process_exit_code')) is int and row['process_exit_code'] == 0
            and row.get('process_cleanup_ok') is True and row.get('stop_confirmed') is True,
            'APP_OPERATOR_RESULT_BINDING_INVALID')
    fault = row.get('runtime_fault')
    require((fault is None or isinstance(fault, str) and fault_code(fault) == fault)
            and row.get('runtime_status') == ('FAILED' if fault is not None else 'SUCCEEDED'),
            'APP_OPERATOR_RUNTIME_RESULT_INVALID')
    return row, digest
