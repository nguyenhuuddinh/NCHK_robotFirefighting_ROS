# R5 support port — NOT an operational launcher

These ten helpers are byte-identical to frozen R4 packet
`qa_r4d_servo60_tools_20261026`; see provenance.json. No old QA was repeated.
Import/syntax/hash checks do not prove new startup/teardown integration.

Clock: clock_gate.sh → clock_settle.py → clock_once.sh → probe/assess/freshness.
The read-only laptop gate requires two fresh passing observations >=5s apart,
<=120s, original50ms bounds, both NTP flags and selected10.0.0.253 source.
Pi: pi_refresh.sh → fresh_ntp_sample.py / ntp_cache_policy.py; existing chrony
only, pre-ROS checks, at most one diagnosed cache reset. This helper may invoke
sudo chronyc; it must never be called during a running robot session. No helper
was executed against a clock, service, device or network during this port.

cleanup_group.py inspects one runner-owned setsid group; the caller must still
save PID/SID ownership, select leader-only initial SIGINT for ros2 launch, bound
grace/fallback, wait/reap and report actual child exits. A launch leader RC0 is
insufficient for CLEAN_PASS. Never use process-name kill as a replacement.

Remaining: wire these helpers into a dedicated R5 Pi/laptop entrypoint and
verify ordering/limits/ownership with fake subprocess I/O. The legacy
scripts/fire_robot_session.py remains a demo, not the R5 operator launcher.
Do not run this directory as a robot procedure.
