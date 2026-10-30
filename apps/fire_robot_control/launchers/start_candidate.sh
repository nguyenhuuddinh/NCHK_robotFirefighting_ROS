#!/usr/bin/env bash
# The app opens idle. Starting a session requires a matching Pi static receipt.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
app_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
repo_dir="$(cd -- "$app_dir/../.." && pwd)"
artifact="${1:-}"
if [[ -z "$artifact" ]]; then
  printf '%s\n' 'Cần đường dẫn đến artifact từ prepare_candidate.sh.' >&2
  exit 2
fi
artifact="$(cd -- "$artifact" && pwd)"
source /opt/ros/humble/setup.bash
source /home/huudinh/.codex/worktrees/laptop-baseline/NCHK_robotFirefighting_ROS/ros2_ws/install/local_setup.bash
source "$repo_dir/.codex_qa_backups/r5_operational_candidate_20261026/ros/ros2_ws/install/local_setup.bash"
export PYTHONPATH="$artifact/ros/desktop:$artifact/ros/scripts:${PYTHONPATH:-}"
binary_sha="$(/usr/bin/python3 -c 'import sys; from services.package_candidate import verify_artifact; print(verify_artifact(sys.argv[1])["binary_sha256"])' "$artifact")"
state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/fire_robot_control"
map_root="$repo_dir/ros2_ws/src/fire_robot_navigation/maps"
exec /usr/bin/python3 -m services.supervisor --candidate "$artifact/ros" \
  --binary "$artifact/bin/fire_robot_control" --binary-sha256 "$binary_sha" \
  --state-dir "$state_dir" --map-root "$map_root" --maps "$map_root" \
  "$repo_dir/.codex_qa_backups/r5_operational_candidate_20261026/ros/ros2_ws/src/fire_robot_navigation/maps"
