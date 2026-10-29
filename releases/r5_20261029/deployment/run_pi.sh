#!/usr/bin/env bash
set -Eeo pipefail
host=${1:-10.0.0.1}
[[ "$host" == 10.0.0.1 || "$host" == 10.42.0.185 ]] || exit 2
tools=$(cd -- "$(dirname -- "$0")" && pwd)
(cd "$tools" && sha256sum -c --quiet release_tools.sha256)
remote=$(cat "$tools/pi_command.sh")
# Allocate a TTY so Ctrl-C follows the owned Pi session as in the manual runbook.
ssh -t "pi@$host" bash -lc "$(python3 -c 'import shlex,sys; print(shlex.quote(sys.argv[1]))' "$remote")"
