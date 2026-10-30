#!/usr/bin/env bash
# Build only the isolated RViz preview; this script does not run the robot.
set -eo pipefail

app_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${FIRE_ROBOT_PREVIEW_BUILD_DIR:-${TMPDIR:-/tmp}/fire_robot_control_preview_${UID}}"
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  printf '%s\n' 'Không tìm thấy ROS 2 Humble tại /opt/ros/humble.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
cmake -S "$app_dir/native_rviz" -B "$build_dir" \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$build_dir" --parallel 2
printf 'PREVIEW_BINARY=%s\n' "$build_dir/fire_robot_control_preview"
