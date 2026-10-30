#!/usr/bin/env bash
# Offline source/binary package only. No SSH, clock or robot runtime.
set -eo pipefail
app_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
repo_dir="$(cd -- "$app_dir/../.." && pwd)"
source /opt/ros/humble/setup.bash
export PYTHONPATH="$app_dir:${PYTHONPATH:-}"
build_log="$(mktemp "${TMPDIR:-/tmp}/fire-robot-package-XXXXXX.log")"
trap 'rm -f -- "$build_log"' EXIT
/usr/bin/python3 -m services.package_candidate \
  "$repo_dir/releases/r5_20261029/ros" "$app_dir" "$app_dir/artifacts" | tee "$build_log"
artifact="$(sed -n 's/^CANDIDATE_ARTIFACT=//p' "$build_log" | tail -n 1)"
packet="${artifact##*/}"
[[ "$packet" =~ ^[0-9a-f]{64}$ && "$artifact" == "$app_dir/artifacts/$packet" ]] || exit 2
printf '%s\n' "CANDIDATE_READY=$artifact" "CURRENT_UNCHANGED; select only after QA and matching Pi receipt"
