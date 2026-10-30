"""R5 peer transport with an additional, independent app cleanup verdict."""
import json
import shlex
import subprocess

from operational_peer import SshPi, PeerTransportError, same_binding, strict_json
from .recovery_probe import remote_cleanup


class AppPeer(SshPi):
    def request(self, command):
        if not getattr(self, "_stopping", False):
            return super().request(command)
        if command not in ("STATE", "STOP"):
            raise ValueError("APP_DRAIN_STOP_ONLY")
        # The R5 startup request intentionally rejects any non-DONE error.
        # During app cleanup, retain the startup error while accepting only
        # the bound, unsealed app drain state. No startup admission uses this.
        remote = shlex.join(["python3", self.remote_script, "--root", self.bind["root"],
            "--binding", json.dumps(self.bind, separators=(",", ":")), "--command", command])
        try:
            result = self.run(["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
                "-o", "ConnectTimeout=3", "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
                "pi@10.0.0.1", remote], text=True, capture_output=True, timeout=9, check=False)
        except (OSError, subprocess.SubprocessError) as exc:
            raise PeerTransportError("PI_CONTROL_TRANSPORT:" + type(exc).__name__) from None
        if result.returncode != 0:
            if result.returncode == 255 or "PI_MAILBOX_TIMEOUT:" in getattr(result, "stderr", ""):
                raise PeerTransportError("PI_CONTROL_TRANSPORT:" + command)
            raise RuntimeError("PI_CONTROL_SSH_FAILED:" + command)
        reply = strict_json(result.stdout)
        if not same_binding(reply.get("binding"), self.bind):
            raise ValueError("PI_CONTROL_BINDING_CHANGED")
        state = reply.get("state")
        if not isinstance(state, dict) or state.get("session") != self.bind["session"]:
            raise ValueError("PI_CONTROL_STATE_INVALID")
        if state.get("request_error"):
            raise RuntimeError("PI_REQUEST_REJECTED")
        app_drain = (state.get("app_prearm_drain") is True and state.get("sealed") is False
            and state.get("phase") in ("WAIT_LAPTOP_DRAIN", "STOPPING")
            and state.get("error") == "PI_SEAL_HEALTH_TIMEOUT")
        if state.get("error") and state.get("phase") != "DONE" and not app_drain:
            raise RuntimeError("PI_RUNNER_ERROR:" + str(state["error"])[:160])
        return state

    def stop(self, **kwargs):
        self._stopping = True
        try:
            row = super().stop(**kwargs)
        finally:
            self._stopping = False
        # The R5 verdict is preserved, including original runner_error.
        # A DONE carrying a startup error can still have audited clean groups.
        row["cleanup_confirmed"] = False
        try:
            remote_cleanup(row.get("receipt"))
            row["cleanup_confirmed"] = True
        except (ValueError, TypeError, AttributeError):
            pass
        return row
