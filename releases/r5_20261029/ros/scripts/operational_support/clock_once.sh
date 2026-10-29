#!/usr/bin/env bash
# Called by laptop before UART/observers/Pi robot launch; read-only assessment.
set -Eeo pipefail
export LC_ALL=C
tools=$(cd -- "$(dirname -- "$0")" && pwd)
out=$1
peer_ip=$2
[[ $peer_ip == 10.0.0.253 ]] || { echo 'STOP: reviewed Pi NTP server expects laptop WiFi IP 10.0.0.253'; exit 2; }
mkdir -m 700 "$out"
printf 'CLOCK_GATE_BEGIN=%s\n' "$(date -Ins)"
remote=(ssh -o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=3 -o ServerAliveCountMax=2 pi@10.0.0.1)
{ date -Ins; timedatectl show -p NTP -p NTPSynchronized; chronyc -n tracking; } > "$out/laptop_clock.log" 2>&1
timeout --signal=TERM --kill-after=3s 10s "${remote[@]}" 'LC_ALL=C; export LC_ALL; date -Ins; timedatectl show -p NTP -p NTPSynchronized; chronyc -n tracking' > "$out/pi_clock.log" 2>&1
timeout --signal=TERM --kill-after=3s 10s "${remote[@]}" 'LC_ALL=C chronyc -n sources' > "$out/pi_sources_after.log" 2>&1
timeout --signal=TERM --kill-after=3s 45s python3 "$tools/probe_clock.py" --peer 10.0.0.1 --out "$out/clock_pair.json" > "$out/probe.log" 2>&1
python3 "$tools/assess_clock.py" "$out" > "$out/alignment_assessment.log" 2>&1
python3 "$tools/verify_clock_freshness.py" "$out" > "$out/freshness.log" 2>&1
printf 'CLOCK_GATE_OK=1\n'
