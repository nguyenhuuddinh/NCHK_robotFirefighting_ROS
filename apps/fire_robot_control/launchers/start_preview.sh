#!/usr/bin/env bash
# The visible application is a non-operational isolated preview.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1

app_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${FIRE_ROBOT_PREVIEW_BUILD_DIR:-${TMPDIR:-/tmp}/fire_robot_control_preview_${UID}}"
# Rebuild incrementally so source changes are visible even when a previous
# preview binary already exists in the temporary build directory.
bash "$app_dir/launchers/build_preview.sh"
source /opt/ros/humble/setup.bash
exec "$build_dir/fire_robot_control_preview" --preview "$app_dir/native_rviz/preview.rviz"
