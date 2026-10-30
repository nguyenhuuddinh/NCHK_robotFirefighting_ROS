"""Separate startup failure from telemetry STOP and audited process cleanup."""
from . import recovery_probe as proof


def cleanup_proven(local, remote):
    try:
        proof.require(local.get("clean") is True and local.get("stop_confirmed") is True
                      and local.get("group_cleanup_clean") is True and local.get("errors") == [],
                      "RECOVERY_LOCAL_CLEANUP_MISSING")
        proof.groups_clean(local.get("processes"))
        proof.child_clean(local.get("child_log_audit"))
        proof.remote_cleanup(remote.get("receipt"))
        proof.require(remote.get("confirmed") is True or remote.get("cleanup_confirmed") is True,
                      "RECOVERY_PI_CLEANUP_MISSING")
        return True
    except (ValueError, TypeError, AttributeError):
        return False


def outcome(row, session, packet, *, remote_binding=None):
    if not isinstance(row, dict) or row.get("session") != session or row.get("packet") != packet:
        return False, False
    local, remote = row.get("local"), row.get("remote")
    if not isinstance(local, dict) or not isinstance(remote, dict):
        return False, False
    if remote_binding is not None:
        expected = {k: v for k, v in remote_binding.items() if k != "secret"}
        if remote.get("binding") != expected:
            return False, False
    return local.get("stop_confirmed") is True, cleanup_proven(local, remote)


def primary_error(folder, fallback=None):
    """Only read owned structured failures; never return arbitrary SSH output."""
    from pathlib import Path
    from .recovery_probe import read_object
    for name in ("failure.json", "startup.json"):
        try:
            row, _ = read_object(Path(folder) / name)
            error = row.get("error")
            if isinstance(error, str) and error:
                if error.endswith('OWNED_CHILD_EXIT:operator'):
                    detail = operator_ui_error(Path(folder))
                    if detail is not None:
                        return detail
                return error[:400]
        except (OSError, ValueError, TypeError):
            continue
    if fallback == 'RUNNER_EXIT_2':
        try:
            row, _ = read_object(Path(folder) / 'receipt.json')
            local = row.get('local', {})
            runtime = local.get('runtime_result', {})
            from .operator_result import SCOPE, fault_code
            if (row.get('profile') == 'R5_AUTOMATIC' and local.get('runtime_failure') is True
                    and runtime.get('scope') == SCOPE
                    and runtime.get('session') == row.get('session') and runtime.get('packet') == row.get('packet')
                    and isinstance(runtime.get('runtime_fault'), str)
                    and fault_code(runtime['runtime_fault']) == runtime['runtime_fault']):
                return 'OPERATOR_RUNTIME_FAULT:' + runtime['runtime_fault']
            nav = local.get('processes', {}).get('nav', {})
            audit = local.get('child_log_audit', {}).get('roles', {}).get('nav', {})
            if (row.get('profile') == 'R5_AUTOMATIC'
                    and local.get('stop_confirmed') is True
                    and set(local.get('errors', [])) == {'PROCESS_UNCLEAN:nav', 'CHILD_LOG_AUDIT_FAILED'}
                    and nav.get('forced') is True and nav.get('group_exit_confirmed') is True
                    and audit.get('internal_signal_fallback') is True):
                return 'NAV2_SHUTDOWN_TIMEOUT'
        except (OSError, ValueError, TypeError, AttributeError):
            pass
    return fallback


def startup_result(stage, error):
    # Startup success is historical and independent of a later STOP/cleanup error.
    ready = stage in ('READY_GATE2', 'READY_OBSERVE', 'MAPPING')
    cancelled = isinstance(error, str) and any(word in error for word in
        ('OPERATOR_STOP_DURING_STARTUP', 'USER_OR_FRONTEND_STOP'))
    status = ('READY' if ready else 'CANCELLED' if cancelled else
              'FAILED' if error else 'NOT_READY')
    return dict(stage=stage, status=status)


def operator_ui_error(folder):
    """Read only a bound, allowlisted liveness diagnostic; never STOP proof."""
    from .recovery_probe import read_object
    try:
        diagnostic, _ = read_object(folder / 'operator_ui.json')
        receipt, _ = read_object(folder / 'receipt.json')
        allowed = {'FRONTEND_LEASE_EXPIRED', 'FRONTEND_DISCONNECTED',
                   'FRONTEND_MESSAGE_OVERFLOW', 'FRONTEND_CLOCK_INVALID'}
        if (diagnostic.get('schema_version') == 1
                and diagnostic.get('scope') == 'UI_LIVENESS_ONLY_NOT_STOP_PROOF'
                and diagnostic.get('session') == receipt.get('session')
                and diagnostic.get('packet') == receipt.get('packet')
                and isinstance(diagnostic.get('session'), str)
                and isinstance(diagnostic.get('packet'), str)
                and diagnostic.get('reason') in allowed):
            return 'OPERATOR_UI:' + diagnostic['reason']
    except (OSError, ValueError, TypeError):
        return None
    return None


def fault_kind(stopped, clean, started):
    if started and stopped is not True:
        return "STOP_UNCONFIRMED"
    return None if clean else "CLEANUP_REQUIRED"
