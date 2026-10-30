# Frozen pure function from app6734 for FAIL-before/PASS-after replay.
def receipt_result(row, session, packet, *, remote_binding=None):
    """Process exit and a ready line are never physical STOP evidence."""
    if not isinstance(row, dict) or row.get("session") != session:
        return False, False
    if row.get("packet") != packet:
        return False, False
    local, remote = row.get("local", {}), row.get("remote", {})
    if not isinstance(local, dict) or not isinstance(remote, dict):
        return False, False
    if remote_binding is not None:
        received = remote.get("binding")
        expected = {k: v for k, v in remote_binding.items() if k != "secret"}
        if received != expected:
            return False, False
    stopped = local.get("stop_confirmed") is True
    clean = (local.get("clean") is True and remote.get("confirmed") is True
             and row.get("rc") == 0)
    return stopped, clean
