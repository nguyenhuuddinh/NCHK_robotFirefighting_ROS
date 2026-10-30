"""Save a validated map pair without overwriting an existing map.

No ROS or device access occurs on import. The session manager must supply
proof that the active SLAM session is owned and no Nav2 session is active.
"""
from __future__ import annotations

import os
from pathlib import Path
import re
import subprocess
import tempfile

from .map_catalog import InvalidMap, validate_map


NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}\Z")


class MapSaveError(RuntimeError):
    pass


def ros_map_saver(prefix):
    """Invoke the existing ROS map saver using a structured argument list."""
    args = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(prefix),
            "--ros-args", "-p", "save_map_timeout:=10.0", "-p",
            "map_subscribe_transient_local:=true"]
    try:
        result = subprocess.run(args, text=True, capture_output=True, timeout=25, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise MapSaveError("Không chạy được hoặc quá hạn lưu map") from exc
    if result.returncode:
        raise MapSaveError("Lưu map thất bại: " + result.stderr[-600:])


def save_map_pair(root, name, *, assert_owned_mapping, generate):
    """Generate in a private directory and link a complete verified pair.

    `assert_owned_mapping` must raise if SLAM isn't the current owned session
    or Nav2 is present. `generate(prefix)` must create prefix.yaml and
    prefix.pgm, with the YAML's image reference pointing to prefix.pgm.
    The caller must hold its session-transition lock for the entire call and
    update selected-map state only after this function returns.
    """
    if not isinstance(name, str) or NAME.fullmatch(name) is None:
        raise MapSaveError("Tên map chỉ dùng chữ, số, dấu _ và -")
    root = Path(root).resolve(strict=True)
    if not root.is_dir():
        raise MapSaveError("Thư mục map không hợp lệ")
    if not callable(assert_owned_mapping) or not callable(generate):
        raise TypeError("Mapping proof and map generator are required")
    assert_owned_mapping()
    final_yaml = root / (name + ".yaml")
    final_image = root / (name + ".pgm")
    if final_yaml.exists() or final_yaml.is_symlink() or final_image.exists() or final_image.is_symlink():
        raise MapSaveError("Map đã tồn tại; không ghi đè")
    with tempfile.TemporaryDirectory(prefix=".fire_robot_save_", dir=root) as temp:
        stage = Path(temp)
        prefix = stage / name
        generate(prefix)
        assert_owned_mapping()
        try:
            staged = validate_map(prefix.with_suffix(".yaml"), [stage])
        except InvalidMap as exc:
            raise MapSaveError("Map tạo ra không đầy đủ hoặc không hợp lệ") from exc
        if Path(staged.image_path) != prefix.with_suffix(".pgm"):
            raise MapSaveError("YAML không trỏ tới ảnh vừa lưu")
        linked = []
        try:
            # Exclusive hard links in the same filesystem avoid silently
            # replacing a file created between the existence check and save.
            for source, target in ((prefix.with_suffix(".pgm"), final_image),
                                   (prefix.with_suffix(".yaml"), final_yaml)):
                assert_owned_mapping()
                source_info = source.lstat()
                os.link(source, target, follow_symlinks=False)
                linked.append((target, source_info.st_dev, source_info.st_ino))
                info = target.lstat()
                if (info.st_dev, info.st_ino) != (source_info.st_dev, source_info.st_ino):
                    raise MapSaveError("File map bị thay thế trong lúc lưu")
            result = validate_map(final_yaml, [root])
            if (result.yaml_sha256 != staged.yaml_sha256 or
                    result.image_sha256 != staged.image_sha256):
                raise MapSaveError("File map thay đổi trong lúc lưu")
            assert_owned_mapping()
            return result
        except (OSError, InvalidMap, MapSaveError) as exc:
            # Roll back only files that still have the exact identity we linked.
            for target, device, inode in reversed(linked):
                try:
                    info = target.lstat()
                    if (info.st_dev, info.st_ino) == (device, inode):
                        target.unlink()
                except FileNotFoundError:
                    pass
            if isinstance(exc, MapSaveError):
                raise
            raise MapSaveError("Không thể hoàn tất cặp map; kiểm tra file và dung lượng") from exc
