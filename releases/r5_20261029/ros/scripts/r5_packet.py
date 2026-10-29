#!/usr/bin/env python3
"""Verify/export a pinned source-only R5 packet. No build/install/network I/O."""
import argparse
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import re
import tarfile

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = 'operational_packet.sha256'


def digest(data):
    return hashlib.sha256(data).hexdigest()


def parse_manifest(raw):
    entries = {}
    for line in raw.decode('utf-8').splitlines():
        sha, name = line.split('  ', 1)
        path = PurePosixPath(name)
        if (re.fullmatch('[0-9a-f]{64}', sha) is None or not name or path.is_absolute()
                or any(p in ('.', '..') for p in path.parts) or str(path) != name
                or '\\' in name or name == MANIFEST or name in entries
                or not (name.startswith('scripts/') or name.startswith('ros2_ws/src/'))
                or any(p in ('__pycache__', '.pytest_cache', '.git', 'build', 'install', 'log') for p in path.parts)):
            raise ValueError('INVALID_SOURCE_PACKET_ENTRY:' + name)
        entries[name] = sha
    if not entries:
        raise ValueError('EMPTY_SOURCE_PACKET')
    return entries


def source_packet(root=ROOT):
    root = Path(root).resolve()
    raw = (root / MANIFEST).read_bytes()
    entries = parse_manifest(raw)
    for name, sha in entries.items():
        path = root / name
        # Even an in-root symlink is omitted from transfers: all archive entries
        # must be the actual regular source bytes reviewed by the manifest.
        if any(p.is_symlink() for p in [path, *path.parents] if p != root and p.is_relative_to(root)):
            raise ValueError('SOURCE_SYMLINK_REJECTED:' + name)
        if not path.is_file() or digest(path.read_bytes()) != sha:
            raise ValueError('SOURCE_PACKET_CHANGED:' + name)
    return raw, entries


def export_packet(destination, root=ROOT):
    root = Path(root)
    raw, entries = source_packet(root)
    destination = Path(destination)
    # Exclusive create: never replace an existing reviewed artifact.
    with destination.open('xb') as stream:
        with tarfile.open(fileobj=stream, mode='w:gz') as archive:
            for name in [MANIFEST, *sorted(entries)]:
                data = raw if name == MANIFEST else (root / name).read_bytes()
                if name != MANIFEST and digest(data) != entries[name]:
                    raise ValueError('SOURCE_CHANGED_DURING_EXPORT:' + name)
                row = tarfile.TarInfo(name)
                row.size = len(data)
                row.mode = 0o644
                row.mtime = 0
                archive.addfile(row, io.BytesIO(data))
    return verify_archive(destination, digest(raw))


def verify_archive(path, expected_packet):
    if re.fullmatch('[0-9a-f]{64}', expected_packet) is None:
        raise ValueError('EXPECTED_PACKET_SHA256_REQUIRED')
    with tarfile.open(path, mode='r:gz') as archive:
        rows = archive.getmembers()
        names = [row.name for row in rows]
        if len(names) != len(set(names)) or any(not row.isfile() for row in rows):
            raise ValueError('NON_REGULAR_OR_DUPLICATE_ARCHIVE_ENTRY')
        if MANIFEST not in names:
            raise ValueError('ARCHIVE_MANIFEST_MISSING')
        raw = archive.extractfile(MANIFEST).read()
        if digest(raw) != expected_packet:
            raise ValueError('ARCHIVE_PACKET_SHA256_MISMATCH')
        entries = parse_manifest(raw)
        if set(names) != set(entries) | {MANIFEST}:
            raise ValueError('ARCHIVE_MEMBERS_NOT_EXACTLY_PINNED')
        for name, sha in entries.items():
            if digest(archive.extractfile(name).read()) != sha:
                raise ValueError('ARCHIVE_FILE_CHANGED:' + name)
    return dict(source_pins=len(entries), packet_sha256=expected_packet,
                archive=str(path), archive_sha256=digest(Path(path).read_bytes()),
                source_only=True, runtime_ready=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--export', type=Path)
    group.add_argument('--verify-archive', type=Path)
    parser.add_argument('--expected-packet')
    args = parser.parse_args(argv)
    if args.export:
        row = export_packet(args.export)
    elif args.verify_archive:
        row = verify_archive(args.verify_archive, args.expected_packet or '')
    else:
        raw, entries = source_packet()
        row = dict(source_pins=len(entries), packet_sha256=digest(raw), source_only=True, runtime_ready=False)
    print(json.dumps(row, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
