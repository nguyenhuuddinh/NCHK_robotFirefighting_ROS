"""Check the versioned Pi static-install receipt before starting a session."""
import json
from pathlib import Path, PurePosixPath
import re

from .frontend_lease import _object, _nonfinite


def validate_pi_receipt(path, packet):
    path = Path(path)
    if not path.is_file() or path.stat().st_size > 1024 * 1024:
        raise ValueError("PI_STATIC_RECEIPT_MISSING")
    try:
        row = json.loads(path.read_text(), object_pairs_hook=_object, parse_constant=_nonfinite)
        checks = row["checks"]
        sources = checks["installed_sources"]
        if (row["schema_version"] != 1 or row["role"] != "pi" or row["host_machine"] != "aarch64"
                or row["live"] is not False or row["runtime_verified"] is not False
                or row["status"] != "STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED"
                or row["errors"] != [] or checks["source_packet_sha256"] != packet
                or not isinstance(sources, list) or len(sources) != 103):
            raise ValueError("PI_STATIC_RECEIPT_MISMATCH")
        stage = PurePosixPath("/home/pi/fire_robot_desktop") / packet[:12] / "ros/ros2_ws"
        baseline = PurePosixPath("/home/pi/NCHK_robotFirefighting_ROS_baseline/ros2_ws")
        roots = tuple(parent / item for parent in (stage, baseline) for item in ("install", "build"))
        seen = set()
        for item in sources:
            name, digest = item["path"], item["sha256"]
            if not isinstance(name, str) or not isinstance(digest, str) or not re.fullmatch("[0-9a-f]{64}", digest):
                raise ValueError("PI_STATIC_SOURCE_INVALID")
            p = PurePosixPath(name)
            if not p.is_absolute() or ".." in p.parts or not any(p.is_relative_to(root) for root in roots) or name in seen:
                raise ValueError("PI_STATIC_SOURCE_PATH_INVALID")
            seen.add(name)
    except (KeyError, TypeError, json.JSONDecodeError) as exc:
        raise ValueError("PI_STATIC_RECEIPT_INVALID") from exc
    return row
