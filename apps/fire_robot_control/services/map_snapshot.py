"""Pin a selected map pair inside a new private session before Nav2 starts."""
import hashlib
import os
from pathlib import Path
import yaml
from .map_catalog import MAX_IMAGE_BYTES, MAX_YAML_BYTES, UniqueKeyLoader, validate_map


def snapshot_map(record, roots, destination):
    current = validate_map(record.yaml_path, roots)
    if current != record:
        raise ValueError("SELECTED_MAP_CHANGED")
    yaml_bytes = Path(record.yaml_path).read_bytes()
    image_bytes = Path(record.image_path).read_bytes()
    if (len(yaml_bytes) > MAX_YAML_BYTES or len(image_bytes) > MAX_IMAGE_BYTES
            or hashlib.sha256(yaml_bytes).hexdigest() != record.yaml_sha256
            or hashlib.sha256(image_bytes).hexdigest() != record.image_sha256):
        raise ValueError("SELECTED_MAP_CHANGED_DURING_COPY")
    # The original YAML may reference a nested relative image. Rewrite only
    # that reference in the private snapshot; leave the source pair intact.
    data = yaml.load(yaml_bytes, Loader=UniqueKeyLoader)
    data["image"] = "image.pgm"
    folder = Path(destination)
    folder.mkdir(mode=0o700)
    image = folder / "image.pgm"
    target = folder / "selected.yaml"
    for path, content in ((image, image_bytes), (target, yaml.safe_dump(data, sort_keys=False).encode())):
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o400)
        with os.fdopen(fd, "wb") as stream:
            stream.write(content)
    copied = validate_map(target, [folder])
    if (copied.image_sha256 != record.image_sha256 or
        (copied.width, copied.height, copied.resolution, copied.origin, copied.occupied_thresh,
         copied.free_thresh, copied.negate, copied.mode) !=
        (record.width, record.height, record.resolution, record.origin, record.occupied_thresh,
         record.free_thresh, record.negate, record.mode)):
        raise ValueError("MAP_SNAPSHOT_MISMATCH")
    return target
