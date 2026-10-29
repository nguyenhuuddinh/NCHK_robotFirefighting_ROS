# R5 paired entrypoints — observe-only candidate, not an operational release

`r5_pi_session.py` and `r5_laptop_session.py` default to printing JSON plans.
Imports/plan mode start no process, ROS context, device, SSH or NTP helper.
The explicitly selected `--run-observe` branch is implemented but has not been
run on either machine. Do not use the legacy auto3pulse demo as this entrypoint.
All live/protocol/provisional/close-stationary/operational-actor flags stay false;
there is no motion/pump runtime acceptance or released supervised live profile.

The source packet is `../operational_packet.sha256`. Runtime preflight verifies
its bytes, installed package modules/configs, reviewed WiFi route/domain/QoS,
and absence of existing robot processes. It changes no network or ROS settings.
Baseline packages not built in this candidate must resolve to byte-matching
accepted installations; missing/mismatched install is blocked before NTP/ROS.
Pi needs both the mission health executable and the R5.3 bringup read-only
BOOT/session identity publisher installed. Build both staging overlays on Pi;
baseline6d8cf0f lacks serial_session_identity. The mission-only install was
found incomplete by the real Pi static receipt, before any robot launch.
`nav2_msgs` is a message dependency, not a Nav2 runtime on Pi.

Pi sequence: reviewed existing chrony refresh before ROS, private new session
(binding session/secret/packet/host-boot/runner-PID/startticks), wait <=360s for
laptop, own rawSerialV2 I/O/safety/camera15FPS process and read-only health node.
The pairing file is private0600, directory0700. The operator must provide that
exact pairing JSON to the laptop; no automatic remote run discovery is used.
Startup/teardown commands go through `operational_peer.py` over bounded SSH to
the exact session mailbox; no recurring SSH probe runs after READY_OBSERVE.

Laptop: original50ms/2samples >=5s/120s clock gate -> Pi I/O health -> decoded
raw camera -> Nav2/RViz/manual pose -> current TF/lifecycle/action ready -> YOLO
first decoded annotated frames -> all10 mission consumers -> operator fresh
READY_PRESS_A report while unarmed/livefalse. Consumer process existence alone
is insufficient. Limits: Pi I/O60s, camera30s, pose185s, YOLO180s, consumers30s.
A silent camera can request one prearm Pi-I/O retry. The old generation must
exit with complete clean child audit before restart; observer pins reset. Retry
is sealed before Nav2 starts and never occurs after an operator/ARM exists.
Stage deadlines are checked before late readiness; original I/P budgets unchanged.

STATE startup transport reads retry at most3 times. A lost START/RETRY/SEAL ACK
is recovered only from matching Pi state, never by blindly replaying a mutation.
Wrong binding/protocol errors are fatal, not convergence. Runtime sensor loss
uses existing ROS freshness/identity/watchdog policies; no auto-restart of an
armed stack is introduced.

Stop: operator SIGINT once -> wait bounded fresh zero/OFF/home/release proof ->
owner groups -> YOLO/Nav2/RViz/health -> request Pi STOP -> verify the exact Pi
runner's final owned-group and child-log receipt. Local SSH exit cannot prove
Pi process exit. Missing/forced/crashed/unknown proof stays failure. Normal Q
operator exit is not classified as a runtime child crash.

StreamingOwnedLog keeps all early errors across arbitrarily long normal log
files, bounded64KiB line buffer/4096child identities. It validates the same fresh
owned inode and rejects replacement/truncation. It never accepts a clean tail
that omits an earlier failure. Legacy OwnedLog.text still has its1MiB cap; the
new entrypoints use the streaming subclass. A live launch with a failed child
is detected without waiting for its leader to exit.

The laptop observe window begins after READY, capped at8h. The independent Pi
cap starts after NTP refresh and includes its wait/start/runtime period. These
limits are orchestration bounds, not robot movement or spray duration controls.
External model/binary installation, Pi compatibility, DDS/camera/runtime and
G3/USB/JPEG/WAIT_POSE cleanup acceptance still need R5.4/R5.5 review. No user
hardware test command is released by this file.

Offline inspection only (after sourcing the accepted ROS environment):

```bash
python3 r5_pi_session.py
python3 r5_laptop_session.py
```

## Static packet/install stage added after QA125

The source packet now includes the exact default map YAML+PGM from accepted
R4; the QA125 source snapshot omitted these assets. See `R5_INSTALL.md`,
`r5_packet.py` and `r5_install_check.py` before using run-observe. The checker
only reads files/metadata; it never calls the network/process preflight or
launches a node. See the current project Pi-install report for actual copy/
build/compat status. All observe/live flags and entrypoint behavior remain as
QA125. Runtime commands are still withheld pending both receipts.
