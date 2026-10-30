"""Read-only map catalog; validates a map before it can be selected.

This module never starts ROS, reads a serial device, connects to Pi, or writes
maps. A caller must revalidate the selection just before starting a session.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re

import yaml


MAX_YAML_BYTES = 64 * 1024
MAX_IMAGE_BYTES = 64 * 1024 * 1024
MAX_PIXELS = 40_000_000
MAP_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}\.yaml\Z")


class InvalidMap(ValueError):
    """The supplied path or map pair cannot be used safely."""


class UniqueKeyLoader(yaml.SafeLoader):
    """Reject ambiguous YAML maps instead of silently choosing the last key."""

    def construct_mapping(self, node, deep=False):
        self.flatten_mapping(node)
        result = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            try:
                repeated = key in result
            except TypeError as exc:
                raise yaml.YAMLError("Unhashable YAML key") from exc
            if repeated:
                raise yaml.YAMLError("Duplicate YAML key")
            result[key] = self.construct_object(value_node, deep=deep)
        return result


@dataclass(frozen=True)
class MapRecord:
    name: str
    yaml_path: str
    image_path: str
    yaml_sha256: str
    image_sha256: str
    width: int
    height: int
    resolution: float
    origin: tuple[float, float, float]
    occupied_thresh: float
    free_thresh: float
    negate: int
    mode: str


def _number(value, field):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise InvalidMap(f"{field}: expected finite number")
    return float(value)


def _read_bounded(path, limit):
    try:
        with path.open("rb") as stream:
            data = stream.read(limit + 1)
    except OSError as exc:
        raise InvalidMap(f"Cannot read {path}: {exc.strerror or type(exc).__name__}") from exc
    if not data or len(data) > limit:
        raise InvalidMap(f"File empty or too large: {path}")
    return data


def _pgm_size(data):
    """Check binary PGM header and complete 8-bit/16-bit raster."""
    index = 0

    def token():
        nonlocal index
        while index < len(data):
            if data[index] in b" \t\r\n":
                index += 1
            elif data[index] == ord("#"):
                end = data.find(b"\n", index)
                if end < 0:
                    raise InvalidMap("Truncated PGM comment")
                index = end + 1
            else:
                break
        start = index
        while index < len(data) and data[index] not in b" \t\r\n#":
            index += 1
        if start == index:
            raise InvalidMap("Truncated PGM header")
        return data[start:index]

    if token() != b"P5":
        raise InvalidMap("Expected a binary P5 PGM image")
    try:
        width, height, maximum = (int(token()) for _ in range(3))
    except (ValueError, OverflowError) as exc:
        raise InvalidMap("Invalid PGM dimensions") from exc
    if not (0 < width <= MAX_PIXELS and 0 < height <= MAX_PIXELS and
            width * height <= MAX_PIXELS and 0 < maximum <= 65535):
        raise InvalidMap("PGM dimensions or maximum out of range")
    # P5 raster follows a single whitespace separator. Do not skip arbitrary
    # whitespace here: a valid first pixel may itself be byte 0x0a or 0x20.
    if index >= len(data) or data[index] not in b" \t\r\n":
        raise InvalidMap("PGM header missing raster separator")
    if data[index:index + 2] == b"\r\n":
        index += 2
    else:
        index += 1
    expected = width * height * (1 if maximum < 256 else 2)
    if len(data) - index != expected:
        raise InvalidMap("PGM raster size does not match header")
    return width, height


def validate_map(path, roots):
    roots = tuple(Path(root).resolve(strict=True) for root in roots)
    if not roots:
        raise InvalidMap("No allowed map directories configured")
    requested = Path(path)
    if not MAP_NAME.fullmatch(requested.name):
        raise InvalidMap("Map filename must be a simple .yaml name")
    try:
        resolved = requested.resolve(strict=True)
    except OSError as exc:
        raise InvalidMap("Map YAML not found") from exc
    root = next((root for root in roots if resolved.is_relative_to(root)), None)
    if root is None or not resolved.is_file():
        raise InvalidMap("Map YAML is outside configured map directories")
    raw = _read_bounded(resolved, MAX_YAML_BYTES)
    try:
        value = yaml.load(raw, Loader=UniqueKeyLoader)
    except (yaml.YAMLError, UnicodeDecodeError, RecursionError) as exc:
        raise InvalidMap("Invalid map YAML") from exc
    if not isinstance(value, dict):
        raise InvalidMap("Map YAML must be a mapping")
    image = value.get("image")
    if not isinstance(image, str) or not image or Path(image).is_absolute():
        raise InvalidMap("Map image must be a relative PGM path")
    try:
        image_path = (resolved.parent / image).resolve(strict=True)
    except OSError as exc:
        raise InvalidMap("Map image not found") from exc
    if not image_path.is_relative_to(root) or not image_path.is_file() or image_path.suffix.lower() != ".pgm":
        raise InvalidMap("Map image is outside the map directory or is not PGM")
    resolution = _number(value.get("resolution"), "resolution")
    if resolution <= 0:
        raise InvalidMap("Resolution must be positive")
    origin = value.get("origin")
    if not isinstance(origin, list) or len(origin) != 3:
        raise InvalidMap("Origin must contain three numbers")
    origin = tuple(_number(item, "origin") for item in origin)
    occupied = _number(value.get("occupied_thresh"), "occupied_thresh")
    free = _number(value.get("free_thresh"), "free_thresh")
    if not 0 <= free < occupied <= 1:
        raise InvalidMap("Map thresholds are inconsistent")
    if value.get("mode", "trinary") not in ("trinary", "scale", "raw"):
        raise InvalidMap("Unsupported map mode")
    if type(value.get("negate")) is not int or value["negate"] not in (0, 1):
        raise InvalidMap("Invalid negate value")
    image_data = _read_bounded(image_path, MAX_IMAGE_BYTES)
    width, height = _pgm_size(image_data)
    return MapRecord(
        name=resolved.stem, yaml_path=str(resolved), image_path=str(image_path),
        yaml_sha256=hashlib.sha256(raw).hexdigest(),
        image_sha256=hashlib.sha256(image_data).hexdigest(),
        width=width, height=height, resolution=resolution, origin=origin,
        occupied_thresh=occupied, free_thresh=free, negate=value["negate"],
        mode=value.get("mode", "trinary"),
    )


def catalog(roots):
    """Return all valid maps and separately report invalid ones, with paths."""
    roots = tuple(Path(root).resolve(strict=True) for root in roots)
    valid, invalid, seen = [], [], set()
    for root in roots:
        for path in sorted(root.glob("*.yaml")):
            try:
                record = validate_map(path, roots)
                if record.yaml_path not in seen:
                    seen.add(record.yaml_path)
                    valid.append(record)
            except InvalidMap as exc:
                invalid.append({"path": str(path), "reason": str(exc)})
    return valid, invalid


def still_matches(record, roots):
    """Check the selected pair again before a caller starts its runner."""
    try:
        return validate_map(record.yaml_path, roots) == record
    except (InvalidMap, OSError):
        return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("roots", nargs="+", type=Path)
    args = parser.parse_args(argv)
    try:
        valid, invalid = catalog(args.roots)
    except OSError as exc:
        parser.exit(2, f"Map directory error: {exc}\n")
    print(json.dumps({"maps": [asdict(item) for item in valid], "invalid": invalid},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
