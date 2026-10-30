"""Strict app source archive, independent of the immutable R5 packet CLI."""
import argparse
import hashlib
import io
from pathlib import Path, PurePosixPath
import re
import tarfile

MANIFEST = "operational_packet.sha256"
sha = lambda data: hashlib.sha256(data).hexdigest()


def parse_manifest(raw):
    entries = {}
    for line in raw.decode().splitlines():
        digest, name = line.split("  ", 1)
        p = PurePosixPath(name)
        if (not re.fullmatch("[0-9a-f]{64}", digest) or not name or p.is_absolute()
                or str(p) != name or "\\" in name or any(x in (".", "..", "__pycache__", ".git",
                    ".pytest_cache", "build", "install", "log") for x in p.parts)
                or name in entries or not (name.startswith(("scripts/", "ros2_ws/src/", "desktop/"))
                    or name == "desktop_parent.json")):
            raise ValueError("INVALID_APP_PACKET_ENTRY")
        entries[name] = digest
    if not entries:
        raise ValueError("EMPTY_APP_PACKET")
    return entries


def source_packet(root):
    root = Path(root).resolve(strict=True)
    raw = (root / MANIFEST).read_bytes()
    entries = parse_manifest(raw)
    for name, digest in entries.items():
        p = root / name
        if (not p.is_file() or not p.resolve().is_relative_to(root)
                or any(q.is_symlink() for q in [p, *p.parents] if q.is_relative_to(root) and q != root)
                or sha(p.read_bytes()) != digest):
            raise ValueError("APP_PACKET_SOURCE_CHANGED:" + name)
    return raw, entries


def verify_archive(path, expected):
    if not re.fullmatch("[0-9a-f]{64}", expected):
        raise ValueError("EXPECTED_PACKET_REQUIRED")
    with tarfile.open(path, "r:gz") as tar:
        rows = tar.getmembers()
        names = [r.name for r in rows]
        if len(set(names)) != len(names) or any(not r.isfile() for r in rows) or MANIFEST not in names:
            raise ValueError("INVALID_APP_ARCHIVE_MEMBERS")
        raw = tar.extractfile(MANIFEST).read()
        if sha(raw) != expected:
            raise ValueError("APP_ARCHIVE_MANIFEST_MISMATCH")
        entries = parse_manifest(raw)
        if set(names) != set(entries) | {MANIFEST}:
            raise ValueError("APP_ARCHIVE_MEMBERS_MISMATCH")
        for name, digest in entries.items():
            if sha(tar.extractfile(name).read()) != digest:
                raise ValueError("APP_ARCHIVE_FILE_CHANGED:" + name)
    return dict(packet_sha256=expected, source_pins=len(entries))


def export_packet(root, target):
    root = Path(root)
    raw, entries = source_packet(root)
    with Path(target).open("xb") as stream:
        with tarfile.open(fileobj=stream, mode="w:gz") as tar:
            for name in [MANIFEST, *sorted(entries)]:
                data = raw if name == MANIFEST else (root / name).read_bytes()
                if name != MANIFEST and sha(data) != entries[name]:
                    raise ValueError("APP_SOURCE_CHANGED_DURING_EXPORT")
                item = tarfile.TarInfo(name)
                item.size, item.mode, item.mtime = len(data), 0o644, 0
                tar.addfile(item, io.BytesIO(data))
    return verify_archive(target, sha(raw))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--export", type=Path)
    mode.add_argument("--verify-archive", type=Path)
    p.add_argument("--expected-packet")
    a = p.parse_args()
    if a.export:
        print(export_packet(a.root, a.export))
    elif a.verify_archive:
        print(verify_archive(a.verify_archive, a.expected_packet or ""))
    else:
        raw, entries = source_packet(a.root)
        print(dict(packet_sha256=sha(raw), source_pins=len(entries)))


if __name__ == "__main__":
    main()
