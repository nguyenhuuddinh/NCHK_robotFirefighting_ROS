#!/usr/bin/env bash
# Open idle; live actions require an explicit click and matching Pi receipt.
set -eo pipefail
app_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -gt 0 ]]; then
  exec bash "$app_dir/launchers/start_candidate.sh" "$1"
fi
exec bash "$app_dir/launchers/start_candidate_desktop.sh"
