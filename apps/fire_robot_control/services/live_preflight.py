"""Read-only local packet/install/network checks, run before opening SSH."""
import sys
from pathlib import Path


def check_existing_runner_lock(path):
    """Inspect the R5 lock without creating/replacing it or retaining it."""
    import os
    from .recovery_probe import lock_existing
    try:
        fd = lock_existing(path)
    except FileNotFoundError:
        return
    except BlockingIOError:
        raise ValueError("EXISTING_LAPTOP_SESSION_LOCKED") from None
    else:
        os.close(fd)


def main():
    from operational_entry import packet_digest
    from operational_preflight import preflight, validate_gate2_profile
    from r5_install_check import inventory
    root = Path(sys.argv[1])
    if packet_digest(root) != sys.argv[2]:
        raise ValueError("CANDIDATE_PACKET_CHANGED")
    preflight(root, "laptop")
    # Mapping and STOP-only recovery do not launch YOLO and must remain
    # available even if the app-specific weights are absent or damaged.
    kind = sys.argv[3] if len(sys.argv) > 3 else 'fire'
    if kind not in ('fire', 'mapping', 'verify_stop'):
        raise ValueError('INVALID_SESSION_KIND')
    if kind == 'fire':
        from .app_model import validate_model
        validate_model()
    validate_gate2_profile(root)
    # Preserve the original static inventory gate, including the installed
    # ROS source/config bytes, patched Nav2 binary and pinned YOLO model.
    result = inventory(root, "laptop")
    if result.get("errors") or result.get("status") != "STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED":
        reasons = ",".join(str(item.get("check", "unknown")) for item in result.get("errors", []))
        raise ValueError("LAPTOP_STATIC_CHECK_FAILED:" + reasons)
    check_existing_runner_lock(Path.home() / ".cache/fire_robot_r5_laptop.lock")
    print("LOCAL_PREFLIGHT_OK")


if __name__ == "__main__":
    main()
