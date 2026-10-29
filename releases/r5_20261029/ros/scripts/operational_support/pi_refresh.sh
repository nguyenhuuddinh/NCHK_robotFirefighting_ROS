#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C
printf 'PI_HELPER_ENTERED=%s\n' "$(date -Ins)"
# Runs on Pi before ROS. One measured cache recovery, then strict fresh proof.
[[ $(hostname) == ubuntu ]] || { printf 'STOP: unexpected Pi hostname\n'; exit 2; }
[[ $(cat /sys/class/net/eth0/operstate) != up ]] || { printf 'STOP: Pi LAN still active\n'; exit 2; }
if pgrep -af '[r]os2 launch|[s]erial_bridge_node|[n]ative_mjpeg_camera|[b]t_navigator|[f]ire_mission_supervisor|[y]olo_observer|[c]ontroller_server|[p]lanner_server|[s]afety_watchdog'; then
 printf 'STOP: robot processes are running; no NTP action.\n'; exit 2
fi
grep -Eq '^server[[:space:]]+10\.0\.0\.253([[:space:]]|$)' /etc/chrony/sources.d/90-fire-robot-local.sources || { printf 'STOP: reviewed WiFi NTP source not configured\n'; exit 2; }
systemctl is-active --quiet chrony || { printf 'STOP: existing chrony daemon inactive\n'; exit 2; }
printf 'NTP_BEFORE=%s\n' "$(date -Ins)"
tracking_before=$(chronyc -n tracking)
printf '%s\n' "$tracking_before"
before=$(chronyc -n sources)
printf '%s\n' "$before"
# A selected source can retain an old sample (run150153). Require a NEW
# reference after one bounded burst; selection alone is not readiness.
tools=$(cd -- "$(dirname -- "$0")" && pwd)
sample_dir=$(mktemp -d /tmp/fireqa-ntp-sample.XXXXXX)
trap 'rm -rf -- "$sample_dir"' EXIT
printf '%s\n' "$tracking_before" > "$sample_dir/before.log"
# Both configured endpoints belong to the same laptop. The disconnected LAN can
# remain the best cached candidate and wait indefinitely for a new LAN sample.
# chrony4.2 reset sources is supported (verified Pi help); selectopts is not.
# WiFi recovery requires a stranded accepted reference, advancing replies and
# exact test-C rejection. A recent-good -> rejected transition is covered too.
# Never reset an advancing/selected source or samples while ROS is running.
if [[ -n ${SSH_CONNECTION:-} && ${SSH_CONNECTION%% *} != 10.0.0.253 ]]; then
 printf 'STOP: Pi NTP expects laptop WiFi IP10.0.0.253; actual peer=%s\n' "${SSH_CONNECTION%% *}"
 exit 2
fi
printf 'NTP_ACTION=BOUNDED_WIFI_REFRESH\n'
printf 'Uses existing chrony only; no configuration, manual date step, or service restart.\n'
sudo -v
printf '\nSOURCE_DIAGNOSTICS_BEFORE\n'
sudo chronyc -n selectdata -v
sudo chronyc -n ntpdata 10.0.0.253 | tee "$sample_dir/ntpdata_before.log"
cache_reset_done=0
if grep -Eq '^Reference ID[[:space:]]*:.*\(10\.42\.0\.1\)' <<< "$tracking_before"; then
 # Keep chrony configuration files and daemon. Drop old measurements
 # once, become temporarily unsynchronised, then require fresh WiFi selection.
 printf 'NTP_CACHE_RESET=LAN_REFERENCE_WITH_ETH0_DOWN_ONCE\n'
 sudo chronyc reset sources
 cache_reset_done=1
else
 printf 'NTP_CACHE_RESET=NONE_NOT_OLD_LAN_REFERENCE\n'
fi
sudo chronyc online 10.0.0.253
sudo chronyc burst 4/8 10.0.0.253
sudo chronyc reselect
# Bounded observation, not repeated mutation or an unlimited waitsync.
deadline=$((SECONDS+45))
wifi_recovery_due=$((SECONDS+8))
while :; do
 sources=$(chronyc -n sources)
 printf '\nNTP_SNAPSHOT=%s\n%s\n' "$(date -Ins)" "$sources"
 if grep -Eq '^\^\*[[:space:]]+10\.0\.0\.253[[:space:]]' <<< "$sources"; then
   { date -Ins; chronyc -n tracking; } > "$sample_dir/after.log"
   cat "$sample_dir/after.log"
   if python3 "$tools/fresh_ntp_sample.py" "$sample_dir/before.log" "$sample_dir/after.log"; then
   sudo chronyc -n selectdata -v
   sudo chronyc -n ntpdata 10.0.0.253
   printf 'WIFI_NTP_SOURCE_SELECTED=1 NEW_WIFI_REFERENCE=1\n'
   exit 0
   fi
 fi
 if ((cache_reset_done==0 && SECONDS>=wifi_recovery_due)); then
   { date -Ins; chronyc -n tracking; } > "$sample_dir/current_tracking.log"
   printf '%s\n' "$sources" > "$sample_dir/current_sources.log"
   sudo chronyc -n ntpdata 10.0.0.253 > "$sample_dir/ntpdata_after.log"
   cat "$sample_dir/ntpdata_after.log"
   if python3 "$tools/ntp_cache_policy.py" "$sample_dir/current_tracking.log" \
      "$sample_dir/current_sources.log" "$sample_dir/ntpdata_before.log" "$sample_dir/ntpdata_after.log" \
      "$sample_dir/before.log"; then
     # Recheck at mutation time; the earlier preflight is not authority to
     # reset samples if a robot or a wired connection has appeared meanwhile.
     [[ $(cat /sys/class/net/eth0/operstate) != up ]] || { printf 'STOP: LAN became active before cache recovery\n'; exit 2; }
     if pgrep -af '[r]os2 launch|[s]erial_bridge_node|[n]ative_mjpeg_camera|[b]t_navigator|[f]ire_mission_supervisor|[y]olo_observer|[c]ontroller_server|[p]lanner_server|[s]afety_watchdog'; then
       printf 'STOP: robot appeared before cache recovery\n'; exit 2
     fi
     printf 'NTP_CACHE_RESET=STRANDED_WIFI_DELAY_FILTER_ONCE\n'
     sudo chronyc reset sources
     cache_reset_done=1
     sudo chronyc online 10.0.0.253
     sudo chronyc burst 4/8 10.0.0.253
     sudo chronyc reselect
   fi
 fi
 (( SECONDS < deadline )) || break
 sleep 2
done
sudo chronyc -n selectdata -v
sudo chronyc -n ntpdata 10.0.0.253
chronyc -n tracking
printf 'STOP: fresh advancing WiFi reference not proven; preserve diagnostics, no automatic repeat.\n'
exit 2
