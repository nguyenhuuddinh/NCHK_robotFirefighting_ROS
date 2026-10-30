#!/usr/bin/env bash
# Desktop wrapper: show launch errors without requiring a terminal.
set -eo pipefail
app_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
marker="$app_dir/artifacts/current.txt"
log_file="$(mktemp "${TMPDIR:-/tmp}/fire-robot-launch-XXXXXX.log")"
if [[ ! -r "$marker" ]]; then
  zenity --error --title='Fire Robot' --text='Chưa có gói app; hãy chạy prepare_candidate.sh.'
  exit 2
fi
IFS= read -r packet < "$marker"
if [[ ! "$packet" =~ ^[0-9a-f]{64}$ ]]; then
  zenity --error --title='Fire Robot' --text='Mã phiên bản app không hợp lệ.'
  exit 2
fi
if ! bash "$app_dir/launchers/start_candidate.sh" "$app_dir/artifacts/$packet" >"$log_file" 2>&1; then
  zenity --error --title='Fire Robot' --text="Không mở được app hoặc phiên cần kiểm tra. Log: $log_file"
  exit 2
fi
