#!/usr/bin/env bash
# User-run automatic profile, explicit map, no synthetic operator keys.
set -Eeo pipefail
umask 077
pairing=${1:-}
host=${2:-10.0.0.1}
map_name=${3:-fire_room_9_10_2026}
[[ "$host" == 10.0.0.1 || "$host" == 10.42.0.185 ]] || exit 2
[[ "$pairing" =~ ^/home/pi/qa_logs/r5_automatic_[0-9]{8}_[0-9]{6}/pairing.json$ ]] || {
  echo 'Usage: run_laptop.sh /home/pi/qa_logs/r5_automatic_YYYYMMDD_HHMMSS/pairing.json [Pi_IP] [fire_room_9_10_2026|fire_room_TN1]'; exit 2;
}
tools=$(cd -- "$(dirname -- "$0")" && pwd)
(cd "$tools" && sha256sum -c --quiet release_tools.sha256)
case "$map_name" in
  fire_room_9_10_2026|fire_room_9_10_2026.yaml|fire_room_9_10_2026.pgm)
    map_path='/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/.codex_qa_backups/r5_operational_candidate_20261026/ros/ros2_ws/src/fire_robot_navigation/maps/fire_room_9_10_2026.yaml'
    ;;
  fire_room_TN1|fire_room_TN1.yaml|fire_room_TN1.pgm)
    map_path='/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/ros2_ws/src/fire_robot_navigation/maps/fire_room_TN1.yaml'
    ;;
  *) echo "Unknown map: $map_name"; exit 2 ;;
esac
[[ -f "$map_path" ]] || { echo "Map not found: $map_path"; exit 2; }
printf 'R5_SELECTED_MAP=%s\n' "$map_path"
mkdir -p "$HOME/qa_logs"
R5_PAIRING_LOCAL=$(mktemp "$HOME/qa_logs/r5_pairing_XXXXXX")
scp -q -o ConnectTimeout=8 "pi@$host:$pairing" "$R5_PAIRING_LOCAL"
chmod 600 "$R5_PAIRING_LOCAL"
source /opt/ros/humble/setup.bash
source /home/huudinh/.codex/worktrees/laptop-baseline/NCHK_robotFirefighting_ROS/ros2_ws/install/local_setup.bash
source '/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/.codex_qa_backups/r5_operational_candidate_20261026/ros/ros2_ws/install/local_setup.bash'
R5_LAPTOP_OUT="$HOME/qa_logs/r5_automatic_laptop_$(date +%Y%m%d_%H%M%S)"
printf 'LOG_LAPTOP=%s\n' "$R5_LAPTOP_OUT"
python3 '/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/.codex_qa_backups/r5_operational_candidate_20261026/ros/scripts/r5_laptop_session.py' \
  --run-automatic --automatic-token R5_AUTOMATIC_FIRE_LIVE \
  --map "$map_path" \
  --pairing "$R5_PAIRING_LOCAL" \
  --remote-script /home/pi/r5_staging/0c54d9b79765/ros/scripts/operational_peer.py \
  --out "$R5_LAPTOP_OUT"
