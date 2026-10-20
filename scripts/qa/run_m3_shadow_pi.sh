#!/usr/bin/env bash
# User-run only, on Pi. About 7 minutes, no goal/arm/actuator commands.
set -e
tools=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
wt=/home/pi/NCHK_robotFirefighting_ROS_wireless
budget=${BUDGET_FILE:?Set BUDGET_FILE to the persistent shadow ledger}
initialize=${INITIALIZE_SHADOW_BUDGET:-false}
test -d "$wt/ros2_ws" || { printf 'STOP: run this on Pi\n'; exit 2; }
case "$initialize" in true|false) ;; *) printf 'STOP: invalid initialization flag\n'; exit 2;; esac
cd "$wt"
test -z "$(git status --porcelain)" || { printf 'STOP: Pi worktree changed\n'; exit 2; }
(cd "$tools" && sha256sum --quiet -c tools.sha256)
sha256sum --quiet -c "$tools/runtime_sources.sha256"
pattern='[r]os2 launch|[s]erial_bridge_node|[c]amsense_x1_node|[b]t_navigator|[c]ontroller_server|[p]lanner_server|[s]afety_watchdog|[n]ative_mjpeg_camera|[f]ire_mission_supervisor|[m]3_shadow_observer.py|[r]viz2'
if pgrep -af "$pattern"; then printf 'STOP: old robot processes still running\n'; exit 2; fi
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
nav_prefix=$(ros2 pkg prefix fire_robot_navigation)
test -x "$nav_prefix/lib/fire_robot_navigation/bt_navigator_snapshot" || {
  printf 'STOP: build/install snapshot navigator on Pi first\n'; exit 2;
}
for item in 'fire_robot_navigation nav2.launch.py' 'fire_robot_navigation nav2_snapshot_shadow.launch.py' 'fire_robot_bringup robot_m3_session_shadow.launch.py' 'fire_robot_bringup robot_wireless_io_nav_static.launch.py' 'fire_robot_mission mission_session_shadow.launch.py'; do
  read -r package launch_file <<< "$item"
  prefix=$(ros2 pkg prefix "$package")
  cmp "$wt/ros2_ws/src/$package/launch/$launch_file" "$prefix/share/$package/launch/$launch_file"
done
python3 - "$budget" "$initialize" <<'PY'
import hashlib,importlib,json,sys
from pathlib import Path
source=Path('/home/pi/NCHK_robotFirefighting_ROS_wireless/ros2_ws/src/fire_robot_mission/fire_robot_mission')
for name in ('mission_supervisor','global_stop_evidence','shadow_session_runtime','shadow_budget','close_owner_evidence','stop_metadata_executor'):
    loaded=Path(importlib.import_module('fire_robot_mission.'+name).__file__)
    if hashlib.sha256(loaded.read_bytes()).digest()!=hashlib.sha256((source/(name+'.py')).read_bytes()).digest():
        raise SystemExit('STOP: stale Python overlay '+str(loaded))
path=Path(sys.argv[1])
if not path.is_absolute() or not path.parent.is_dir() or path.is_symlink():
    raise SystemExit('STOP: choose persistent absolute budget path in existing directory')
if path.exists():
    state=json.loads(path.read_text())
    if state.get('pending') is not None:
        raise SystemExit('STOP: pending ledger needs explicit recovery, do not delete it')
    if state.get('used',{}).get('sessions',3)>1:
        raise SystemExit('STOP: fewer than two reservations remain; use verified encounter reset, not file deletion')
elif sys.argv[2]!='true':
    raise SystemExit('STOP: missing ledger needs explicit first-use initialization')
PY
out="/home/pi/qa_logs/qa_m3_runtime_$(date +%Y%m%d_%H%M%S)"
mkdir -p /home/pi/qa_logs
mkdir -m 700 "$out"
printf 'LOG_PI=%s\nDURATION: about 7 minutes; observer starts after 65s.\n' "$out"
printf 'START=%s\nHEAD=%s\nBUDGET_FILE=%s\nINITIALIZE=%s\n' "$(date -Is)" "$(git rev-parse HEAD)" "$budget" "$initialize" > "$out/receipt.txt"
cp "$tools/runtime_sources.sha256" "$out/runtime_sources.sha256"
sha256sum "$nav_prefix/lib/fire_robot_navigation/bt_navigator_snapshot" > "$out/native_binary.sha256"
guard=
observer=
cleanup() {
  if [ -n "$observer" ] && kill -0 "$observer" 2>/dev/null; then kill -INT "$observer" 2>/dev/null || true; fi
  if [ -n "$guard" ] && kill -0 "$guard" 2>/dev/null; then kill -INT "$guard" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
cd ros2_ws
timeout --signal=INT --kill-after=15s 420s ros2 launch fire_robot_bringup robot_m3_session_shadow.launch.py \
  "shadow_budget_file:=$budget" "initialize_shadow_budget:=$initialize" camera_fps:=10 \
  > "$out/launch.log" 2>&1 &
guard=$!
sleep 65
if ! kill -0 "$guard" 2>/dev/null; then
  printf 'STOP: launch exited before observer\n' | tee -a "$out/receipt.txt"
  exit 2
fi
printf 'OBSERVER_START=%s\n' "$(date -Is)" >> "$out/receipt.txt"
timeout --signal=INT --kill-after=5s 310s python3 "$tools/m3_shadow_observer.py" \
  --seconds 300 --output "$out/runtime.json" > "$out/observer.log" 2>&1 &
observer=$!
observer_rc=0
wait "$observer" || observer_rc=$?
observer=
launch_rc=0
wait "$guard" || launch_rc=$?
guard=
printf 'OBSERVER_RC=%s\nLAUNCH_RC=%s\nEND=%s\n' "$observer_rc" "$launch_rc" "$(date -Is)" >> "$out/receipt.txt"
sleep 3
pgrep -af "$pattern" > "$out/processes_after.txt" || true
if [ -s "$out/processes_after.txt" ]; then
  printf 'CLEANUP=REMAINING_PROCESSES\n' >> "$out/receipt.txt"
else
  printf 'CLEANUP=NO_MATCHING_PROCESSES\n' >> "$out/receipt.txt"
fi
cd "$out"
sha256sum receipt.txt launch.log observer.log processes_after.txt runtime_sources.sha256 native_binary.sha256 > hashes.sha256
if [ -f runtime.json ]; then sha256sum runtime.json >> hashes.sha256; fi
printf 'DONE\n' > DONE
cat receipt.txt observer.log
printf 'LOG_PI=%s\n' "$out"
test "$observer_rc" -eq 0 && test "$launch_rc" -eq 124 && test ! -s processes_after.txt
