"""Build a source-pinned desktop candidate and offline-only artifact bundle."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from .candidate_build import build
from .supervisor import verify_packet


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_artifact(folder):
    folder = Path(folder).resolve(strict=True)
    row = json.loads((folder / "build_receipt.json").read_text())
    packet = verify_packet(folder / "ros")
    binary = folder / "bin/fire_robot_control"
    archive = folder / "source.tar.gz"
    if (row.get("schema_version") != 1 or row.get("packet") != packet
            or row.get("binary_sha256") != sha(binary) or row.get("archive_sha256") != sha(archive)
            or row.get("runtime_verified") is not False or not os.access(binary, os.X_OK)):
        raise ValueError("DESKTOP_ARTIFACT_MISMATCH")
    return row


def package(source, app, artifacts):
    app, artifacts = Path(app).resolve(strict=True), Path(artifacts)
    if artifacts.resolve().is_relative_to(Path(source).resolve(strict=True)):
        raise ValueError("APP_ARTIFACTS_INSIDE_IMMUTABLE_R5")
    artifacts.mkdir(mode=0o700, parents=True, exist_ok=True)
    temp = Path(tempfile.mkdtemp(prefix=".building-", dir=artifacts))
    try:
        candidate = temp / "ros"
        packet = build(source, app, candidate)
        target = artifacts / packet
        if target.exists():
            verify_artifact(target)
            return target
        builddir = temp / "native_build"
        configure = ["cmake", "-S", str(candidate / "desktop/native_rviz"), "-B", str(builddir),
            "-DPython3_EXECUTABLE=/usr/bin/python3", "-DPYTHON_EXECUTABLE=/usr/bin/python3",
            "-DCMAKE_BUILD_TYPE=RelWithDebInfo"]
        subprocess.run(configure, check=True)
        subprocess.run(["cmake", "--build", str(builddir), "--parallel", "2", "--target",
                        "fire_robot_control_preview", "fire_robot_frontend_test"], check=True)
        subprocess.run(["ctest", "--test-dir", str(builddir), "--output-on-failure"], check=True)
        (temp / "bin").mkdir()
        binary = temp / "bin/fire_robot_control"
        shutil.copy2(builddir / "fire_robot_control_preview", binary)
        archive = temp / "source.tar.gz"
        from .packet_archive import export_packet
        export_packet(candidate, archive)
        shutil.rmtree(builddir)
        (temp / "build_receipt.json").write_text(json.dumps(dict(schema_version=1, packet=packet,
            binary_sha256=sha(binary), archive_sha256=sha(archive), native_ctest_passed=True,
            runtime_verified=False, pi_static_receipt="PENDING"), indent=2) + "\n")
        verify_artifact(temp)
        temp.rename(target)
        return target
    finally:
        if temp.exists():
            shutil.rmtree(temp)


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("app", type=Path)
    parser.add_argument("artifacts", type=Path)
    args = parser.parse_args()
    result = package(args.source, args.app, args.artifacts)
    print("CANDIDATE_ARTIFACT=" + str(result))
    print("PI_STATIC_RECEIPT=PENDING; ROBOT_RUNTIME=NOT_VERIFIED")


if __name__ == "__main__":
    main()
