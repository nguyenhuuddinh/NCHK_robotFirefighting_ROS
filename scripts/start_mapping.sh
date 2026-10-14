#!/usr/bin/env bash
set -Ee -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "$SCRIPT_DIR/../ros2_ws" && pwd)"
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY

exec python3 "$SCRIPT_DIR/fire_robot_session.py" mapping "$@"
