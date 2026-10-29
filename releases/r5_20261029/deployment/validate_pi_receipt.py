#!/usr/bin/env python3
"""Validate the source-only Pi receipt for the R5 best_29_10_2026 model-only release."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path, PurePosixPath


PACKET_SHA256 = "0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0"
ALLOWED_ROOTS = (
    PurePosixPath("/home/pi/r5_staging/0c54d9b79765/ros/ros2_ws/install"),
    PurePosixPath("/home/pi/r5_staging/0c54d9b79765/ros/ros2_ws/build"),
    PurePosixPath("/home/pi/NCHK_robotFirefighting_ROS_baseline/ros2_ws/install"),
    PurePosixPath("/home/pi/NCHK_robotFirefighting_ROS_baseline/ros2_ws/build"),
)
EXPECTED_SOURCE_COUNT = 103
SHA256_RE = re.compile(r"[0-9a-f]{64}")


class ReceiptError(ValueError):
    pass


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise ReceiptError(reason)


def validate(path: Path) -> None:
    receipt = json.loads(path.read_text(encoding="utf-8"))
    require(receipt.get("schema_version") == 1, "SCHEMA_VERSION")
    require(receipt.get("role") == "pi", "ROLE")
    require(receipt.get("host_machine") == "aarch64", "HOST_MACHINE")
    require(receipt.get("live") is False, "LIVE_MUST_BE_FALSE")
    require(receipt.get("runtime_verified") is False, "RUNTIME_MUST_BE_UNVERIFIED")
    require(
        receipt.get("status") == "STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED",
        "STATUS",
    )
    require(receipt.get("errors") == [], "RECEIPT_ERRORS")

    checks = receipt.get("checks")
    require(isinstance(checks, dict), "CHECKS")
    require(checks.get("source_packet_sha256") == PACKET_SHA256, "SOURCE_PACKET_MISMATCH")
    rows = checks.get("installed_sources")
    require(isinstance(rows, list), "INSTALLED_SOURCES")
    require(len(rows) == EXPECTED_SOURCE_COUNT, "INSTALLED_SOURCE_COUNT")

    paths: list[str] = []
    for index, row in enumerate(rows):
        require(isinstance(row, dict), f"SOURCE_ROW_{index}")
        raw_path = row.get("path")
        digest = row.get("sha256")
        require(isinstance(raw_path, str), f"SOURCE_PATH_{index}")
        require(isinstance(digest, str) and SHA256_RE.fullmatch(digest) is not None,
                f"SOURCE_SHA256_{index}")
        installed = PurePosixPath(raw_path)
        require(installed.is_absolute(), f"SOURCE_ABSOLUTE_{index}")
        require(
            any(installed.is_relative_to(root) for root in ALLOWED_ROOTS),
            f"SOURCE_STAGE_{index}",
        )
        paths.append(raw_path)
    require(len(paths) == len(set(paths)), "DUPLICATE_SOURCE_PATH")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("receipt", type=Path)
    args = parser.parse_args()
    try:
        validate(args.receipt)
    except (OSError, json.JSONDecodeError, ReceiptError) as exc:
        print(f"R5_MODEL_29102026_PI_RECEIPT_INVALID={exc}")
        return 2
    print("R5_MODEL_29102026_PI_TOOLS_READY=1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
