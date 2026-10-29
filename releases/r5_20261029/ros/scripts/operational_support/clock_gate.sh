#!/usr/bin/env bash
# No NTP mutation here; wait for measured agreement under the original50ms gate.
set -Eeo pipefail
tools=$(cd -- "$(dirname -- "$0")" && pwd)
exec python3 -u "$tools/clock_settle.py" "$@"
