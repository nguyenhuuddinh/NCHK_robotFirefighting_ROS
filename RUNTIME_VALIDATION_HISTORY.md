# Runtime validation history — laptop baseline

## BASELINE-C1-N8R2-20261020-01

- Base code: `da0dc7077e86ecad6e0be30c4c5a63f7dc698d8c` plus C1 patch.
- Branch: `codex/laptop-baseline-recovery`; candidate not committed at recording.
- Target: ESP32-S3 **N8R2**, environment `freenove_esp32_s3_wroom_n8r2`.
- Placement: Nav2 + YOLO on laptop; Pi I/O/TF/safety; camera configured15FPS.
- Firmware source/image not modified by C1; no firmware upload performed.

### Change and reason

Accept firmware BOOT identity and USB recovery diagnostics (17/22 fields), with
strict schema/CRC/sequence/size validation. Diagnostic frames do not count as
STATE freshness and cannot authorize active CMD/FIRE/PUMP. Retain the baseline
serial reconnect/bootstrap and mission/perception/navigation behavior.

Add optional diagnostics, default OFF: no collector when disabled; no additional
timer/thread/queue. When enabled, two latest slots coalesce bursts, with a total
capture limit of8events per bridge process lifetime, including reconnects.
Logging uses the existing telemetry callback after sensor publication, with a
node teardown guard. BOOT repeats per USB session; it is not proof of reboot.

### Validation

- User ran `verify_offline.sh` on2026-10-20,17:50:13–17:51:58 +07:00.
- **134 passed in104.87s**, pytest exit0. Serial devices were mocked.
- Raw log review:4/4log hashes and198/198source hashes match the candidate.
- Three module imports resolve to the baseline worktree source.
- Regression scope: existing serial protocol/bridge suites plus N8R2 diagnostic
  schema, sequence/reconnect, coalescing/cap, optional OFF and health/admission guards.
- Static diff check passes; source outside the selected bringup change is intact.

Local evidence directory:
`.codex_qa_backups/qa_baseline_c1_n8r2_20261020_173805/`

Run:
`runs/20261020_175012/` (`pytest.log`, `receipt.txt`, `imports.log`,
`runtime_sources.sha256`, `hashes.sha256`).

### Acceptance boundary and next step

**OFFLINE PASS.** Laptop/Pi build, deployed imports, USB/WiFi runtime and odom
latency are **NOT_TESTED/NOT_MEASURED** for C1. This result does not certify that
USB faults are fixed or every robot behavior has passed on hardware.

Next: user approves reviewed C1 commit; commit and push the baseline branch,
then prepare separate baseline worktrees/builds on laptop/Pi with backups and
matching SHA. Keep current firmware N8R2 and camera15FPS; runtime static measurement
comes after build/import verification. Update this history after each gate.

## BASELINE-C1-N8R2-BUILD-20261020-02

Code under test: `ba3fe623027c9b58cee235db8fff2caa0a257bee`.
The user ran isolated build/import gates on both hosts; raw logs were reviewed.

- Laptop: 6 packages built in5.08s (18:38:47–18:38:56 +07), RC0.
- Pi: 5 packages built in53.5s (18:38:58–18:39:57 +07), RC0.
- **BUILD/IMPORT/INSTALL PASS** on both;53/53laptop and43/43Pi log hashes match,
  with198/198source hashes on each. Package prefixes, import bytes and installed
  launch/config resources resolve to their baseline worktrees.
- Pi native LiDAR/serial artifacts verified; show-args confirms camera15FPS,
  diagnosticsdefaultOFF and the N8R2 by-id path.
- Laptop model SHA256:758c18bcfb82e115871059b0498246eff5777a4dff9da455edc93b3fdece8374;
  YOLO venv/CUDA RTX3050 verified; no inference or robot node was started.
- Source remained intact; firmware N8R2 was not uploaded/modified by this gate.

Evidence: laptop `.codex_qa_backups/qa_baseline_build_tools_20261020/runs/qa_baseline_build_laptop_20261020_183847`;
Pi `/home/pi/qa_logs/qa_baseline_build_pi_20261020_183858`, copied locally to
`.codex_qa_backups/qa_baseline_build_pi_20261020_183858`.
Review: `qa_baseline_build_tools_20261020/build_gate_review.json`.

USB/WiFi runtime, actual camera FPS, odom latency and recovery remain unmeasured
for C1. Next: prepare a bounded static WiFi comparison with Nav2/YOLO laptop,
Pi I/O+safety, camera15FPS and a confirmed map before Initial Pose. No goal/arm.
This Markdown update records new evidence and does not change tested runtime code.


## BASELINE-C1-STATIC-V3-20261020-03

Code tested: `ba3fe623027c9b58cee235db8fff2caa0a257bee`, before the C2 candidate.
User ran staged static WiFi on2026-10-20,20:23:53–20:25:04+07.
**INCOMPLETE: NAV2_WARMUP, zero steady measurement seconds.**

- First failure recorded: `PI_HEALTH_QUERY_FAILED`, SSH command timeout RC124
  after8s at20:24:49. Runner then cleaned both hosts; no leftover matching processes.
-97/97outer log hashes match (including copied Pi files);41/41Pi hashes checked
  separately match. Do not add those as138independent files. Source before/after RC0.
- Before Nav2, laptop windows included odom10Hz and JPEG15FPS. After Nav2 startup
  network traffic rose and multiple streams degraded; no RViz/YOLO had started yet.
  Whole-run startup averages must not be presented as steady sensor rates.
- UART96lines records a real CDC TX stall/recovery; native USB kernel disconnect
  aligns with it and odom returns after~2.7s on both observers. Uptime increases;
  this is not evidence of an ESP CPU reboot. Recovery is demonstrated for this event,
  not a guarantee for every future event.
- Camera separately disconnected/re-enumerated at20:24:11–13; user confirms no
  touch/unplug. Camera retried and resumed; keep physical/host/cable causes open.
- Ping laptop59/54 avg134.430max1597.177ms; Pi52/48 avg212.000max1583.852ms.
- Controller protection fault appears after shutdown signal; it did not trigger the
  first failure. Capture as a shutdown defect requiring separate investigation.

Evidence: packet `qa_baseline_combined_v3_tools_20261020`,
`runs/qa_baseline_combined_laptop_20261020_202353`, and its `runtime_review`.
This is not a full-stack/10minute acceptance or a YOLO inference test.

## BASELINE-C2-ASYNC-CANDIDATE-20261020-04

Parent code: `ba3fe623027c9b58cee235db8fff2caa0a257bee` plus uncommitted C2.
Review target remains ESP32-S3 N8R2, Nav2/YOLO laptop, configured camera15FPS.

C2 ports the scoped idea from historical `bf5d3d8`: per-process
`RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS` only for serial bridge and
odom-to-TF in `robot.launch.py`. Both measured RMWs are `rmw_fastrtps_cpp`.
It aims to reduce synchronous DDS send blocking in telemetry callbacks; it does
not fix WiFi bandwidth/discovery, guarantee nonblocking publication or cure USB.
No change to QoS/topic/protocol/watchdog/health threshold/firmware/model/FPS.
One original runtime file changed;197other baseline source files remain identical.
Add launch-contract regression for target environments, scope and safety include.

**IMPLEMENTED; OFFLINE_QA_NOT_RUN; BUILD_NOT_RUN; PI_NOT_DEPLOYED; RUNTIME_NOT_RUN.**
No new performance PASS is claimed. Existing validation applies only to original C1.
Packet `qa_baseline_c2_async_20261020` contains launch backup, candidate manifest
and one user-run offline gate. Next: user runs it, review raw/hash/diff; then user
approves commit+push, isolated Pi build/install verification and targeted runtime
comparison with matching source manifest. Do not rerun old C1 tools against dirty C2.

## BASELINE-C2-ASYNC-OFFLINE-20261020-05

Parent HEAD: `ba3fe623027c9b58cee235db8fff2caa0a257bee` plus reviewed C2 candidate.
User ran the offline gate at2026-10-20T20:34:40+07:00: **11 passed in0.18s**, RC0.
Raw evidence reviewed:7/7 log SHA256 and199/199 candidate source SHA256 match;
the tested manifest matches the reviewed candidate. Diff check passes. Assistant
reviewed evidence/source only and did not execute tests, imports or hardware.

Scope: launch-contract tests for the two process environments, absence of a global
DDS override, safety launch forwarding and existing camera launch checks. These
results do not measure runtime odom latency, USB recovery or inference performance.
The earlier NOT_RUN candidate entry records its state before this gate.

Runtime change remains4 launch lines: ASYNCHRONOUS only on Pi serial bridge and
odom-to-TF processes.197 other original sources are unchanged. Nav2 and YOLO run
on the laptop; camera15FPS and ESP32-S3 N8R2 firmware remain unchanged. The V3
8second timeout was an SSH health query from laptop to Pi while laptop Nav2 was
starting; it does not mean Nav2 ran on Pi or prove the cause of the network delay.

**C2 OFFLINE PASS; NOT_COMMITTED; NOT_PUSHED; BUILD_NOT_RUN; PI_NOT_DEPLOYED;
RUNTIME_NOT_RUN.** No claim of reduced odom delay or resolved USB/camera faults.
Evidence: `qa_baseline_c2_async_20261020/runs/20261020_203440` and
`qa_baseline_c2_async_20261020/offline_gate_review.json` in the local QA backups.

Next: user approves C2 commit and push on `codex/laptop-baseline-recovery`; then
prepare user-run Pi backup/update to exact SHA and build/import/install gate.
Only after that gate passes, prepare a targeted C2 runtime with its new manifest.
Do not reuse an unchanged C1 source-pinned runner or relax safety/freshness guards.
Keep camera USB disconnect and controller shutdown fault as separate open issues.

NEXT_STEP=USER_APPROVE_C2_COMMIT_PUSH_THEN_PREPARE_PI_BUILD

## BASELINE-C2-PI-BUILD-20261020-06

Code: `59283c550ece98acd4c8d78174ae016bf81ce199`, branch baseline recovery.
User ran exact-SHA backup/update/build gate on Pi over LAN20:48:14–20:48:29+07.
**BUILD/IMPORT/INSTALL PASS**, RC0, COMPLETE, DONE present; incremental bringup
build6.75s total (package5.58s). Assistant fetched/reviewed logs without running
builds, imports, ROS nodes or serial hardware.

25/25 log SHA256 match; runtime manifest matches reviewed199source candidate;
user-run source checks before/after pass. Installed package prefixes, resource
bytes and imports resolve to the Pi baseline worktree. Serial/LiDAR native binary
hashes remain unchanged. Camera default15FPS and recovery diagnostics defaultOFF.
Tracked source backup/Git bundle verification is recorded in the receipt at
`/home/pi/qa_backups/pi_baseline_before_c2_20261020_204814`.
Status after gate is clean; no matching robot process remains. Closing the SSH
connection is normal helper completion in this run, not evidence of WiFi failure.

Evidence: local `qa_baseline_c2_build_20261020_204814`; review JSON in
`qa_baseline_c2_pi_build_tools_20261020/build_review.json`.
C2 is now built/deployed on Pi; **RUNTIME_NOT_RUN**. No claim that USB/WiFi faults
are resolved or odom delay improved. Firmware ESP32-S3 N8R2 unchanged.

Next: user runs the offline tool gate for packet
`qa_baseline_c2_runtime_tools_20261020`. It inherits the reviewed V3 runner logic,
with C2 SHA/source pins199 and updated count/labels/README; no new collectors or
relaxed guards. After raw/hash review of that gate, run one paired static WiFi
capture with laptop Nav2/YOLO, camera15FPS and UART,600s steady/12–15min overall.
Do not rebuild unchanged Nav2/YOLO or reflash ESP. Record partial/blocked coverage
honestly and retain camera USB/controller shutdown issues separately.

NEXT_STEP=USER_RUN_C2_RUNTIME_TOOLS_OFFLINE_QA_AND_SEND_LOG

## BASELINE-C2-RUNTIME-TOOLS-OFFLINE-20261020-07

Runtime code remains59283c550ece98acd4c8d78174ae016bf81ce199.
User ran packet `qa_baseline_c2_runtime_tools_20261020` offline gate at20:52:03+07:
**22passed0.14s, RC0**. Review4/4raw-log and29/29tool SHA256; tested tools manifest
and OFFLINE_QA_OK marker match current immutable packet.199/199laptop source hashes
match C2. Copied packet via LAN to `/home/pi/qa_baseline_c2_runtime_tools_20261020`;
remote read-only checks confirm tool/marker/source hashes, pinned HEAD and clean tree.
Assistant performed review/copy/hash checks only; no tests, build, ROS nodes or UART.

Evidence: packet `offline_runs/20261020_205203`, `offline_review.json` and
`deployment_review.json`. The immutable README retains its preparation snapshot;
current release state is in these review records and canonical checkpoint.
**TOOLS OFFLINE PASS; COPY VERIFIED; C2 RUNTIME_NOT_RUN.** This does not prove
reduced latency or resolved USB/WiFi faults.

Next: user runs one paired static WiFi capture after connecting laptop to Pi AP and
unplugging LAN. Keep N8R2 nativeUSB→Pi/COM→laptop, camera15FPS and existing firmware.
Motor/pump powerOFF, no goal/arm, user Initial Pose on map9_10. Staged startup kept;
600s steady, roughly12–15min total including startup/cleanup/copy. Keep WiFi until
both runners finish/copy, send LOG_LAPTOP; review raw/hash and actual coverage before
claiming runtime acceptance. On early STOP preserve logs without automatic rerun.

NEXT_STEP=USER_RUN_ONE_C2_PAIRED_STATIC_WIFI_CAPTURE

## BASELINE-C2-STATIC-RUNTIME-20261020-09

Code59283c550ece98acd4c8d78174ae016bf81ce199. User ran paired static WiFi;
laptop21:06:17–21:07:29+07, Pi `qa_baseline_combined_20261020_210605` copied locally.
**INCOMPLETE / NAV2_WARMUP / SSH_HEALTH_TIMEOUT / steady0s. C2 did not resolve the
startup blocker.** Nav2 ran on laptop; RViz and YOLO never reached launch.

Raw hashes97/97outer match (including nested copied files);41/41Pi and3/3UART
checked separately also match. Do not add those as independent counts. Both
source manifests199files remain correct; cleanup both hosts clean. First failure
SSH health request21:07:05→13 returns124 after8s. Pi RC0 means honoring laptop STOP,
not overall acceptance. Nonzero/invalid command0, forbidden command publishers0.

Before Nav2, Pi windows~10Hz odom/15FPS; laptop has9.96Hz/14.94FPS window.
After Nav2, both observers degrade (e.g.laptop3.78Hz/4.38FPS, Pi3.39Hz/4.98FPS).
Whole-capture odom header age reaches1143ms laptop/1508ms Pi, with later Pi samples
including post-trigger time. Pre-trigger Pi also reaches~1003ms. Inter-host age has
clock offset uncertainty; not pure one-way network delay. Whole-startup rate is
not a steady-state rate. CPU total laptop2.72–6.40%, Pi after startup28.61–49.09%.
Laptop TX24,896,338B in~10.05s after Nav2 (~2.48MB/s); traffic type unclassified.
Ping loss laptop6.67%/Pi9.43%, RTT max3333ms/1941ms. No global CPU saturation proof.

UART77lines, no new CDC stall/ROM boot observed; no ESP USB disconnect in kernel.
The first USB diagnostic recoveries1/recovery_ms3954648 is historical, not a new
recovery in this run (received at esp_ms6486189). BOOT uptime6486288→6525109ms is
continuous. Bridge serial reopen21:07:15 occurs after first STOP; root not traced.
Camera USB1-1.2 resets at21:06:36–37, separate from ESP. Planner abort/context-invalid
errors follow shutdown SIGINT21:07:21.538; not the first trigger. Keep these open.

Evidence: `qa_baseline_c2_runtime_tools_20261020/runs/qa_baseline_combined_laptop_20261020_210617`,
structured `runtime_review_20261020_210617.json`; detailed canonical report
`C2_RUNTIME_RESULT_AND_C3_NETWORK_PLAN_2026-10-20.md`.
Assistant only reviewed files; no new runtime, test, firmware or network changes.

Next: prepare bounded60–90s UDP/RTPS startup capture and allowlisted DDS process-env
recording, with safe cleanup and packet/drop/coverage accounting. Determine discovery
versus user data before selecting an initial-peer/discovery or publication patch.
Do not rerun unchanged600s scenario or reduce FPS/controller/watchdog blindly.
C3 currently has a task brief only; capture tools not implemented/QA/deployed yet.

NEXT_STEP=PREPARE_TARGETED_C3_UDP_RTPS_CAPTURE_90S

## C3-NETWORK-TOOLS-PREPARED-20261020-10

User requested runnable C3 commands. Prepared local QA packet
`qa_c3_udp_rtps_startup_tools_20261020` with33pinned tool inputs, inherited from C2.
Production remains59283c5; no production source/firmware/config patch.
Pi I/O/camera15FPS/safety and UART/observer retained. Laptop launches only Nav2
shadow; no RViz/YOLO/pose/goal/arm. Passive UDP capture runs on laptop only, bounded
90s/200000packets/128bytes snapshot/ring8x4MB, no promiscuous mode. Owned root
capture session handles STOP/signals/timeout and is reaped before log hashing;
tcpdump drops to operator user for capture files. Pi cap240s is fallback cleanup.
SSH8s/safety/freshness guards unchanged. Pi saves only allowlisted DDS process env.

Assistant statically reviewed source, Python/shell syntax and hashes only;
**C3 OFFLINE_QA_NOT_RUN; HARDWARE_NOT_RUN.** User gate keeps22existing regressions
and9new cases for limits, owned cleanup, env privacy and worker stop/start-failure/
deadline. User must obtain RC0 and marker before copying/running the packet.
Commands and exact flow recorded in canonical checkpoint and C3 task brief.
No build, firmware upload or unchanged600s rerun required. Capture quality/drop,
rotation, truncated headers and actual coverage must be reviewed before protocol
claims or choosing a DDS configuration fix.

NEXT_STEP=USER_RUN_C3_OFFLINE_GATE_THEN_COPY_AND_ONE_NETWORK_CAPTURE


## C3-DISCOVERY-CAPTURE-RESULT-20261020-11

Runtime remains C2 `59283c550ece98acd4c8d78174ae016bf81ce199`. C3 tool gate user
31 PASS/0.14s; 4 log and 33 tool hashes verified. Capture run214023: 108 outer,
42 nested Pi and 3 UART hashes verified independently; 199 sources unchanged,
cleanup both hosts clean. Runner RC0 but packet cap200000 reached after16.976s;
not a90s capture, readiness or steady runtime PASS. Zero tcpdump kernel drops
is not zero WiFi loss. Strict SEDP/SPDP discovery94.23% packets/86.60% original
frame bytes; including builtin liveliness94.81%/86.98%. Mostly unicast; observed
GUID prefixes11 laptop/9 Pi (not ROS node counts). Heartbeat145954 and ACKNACK25666.
CPU laptop1.87–4.17%; odom gap2229.9ms, Pi2011.4ms, JPEG also degraded. No new
UART CDC stall/USB detach/ROM boot; two serial reopens with continuous ESP uptime.
Nonzero/forbidden commands0. C2 actual ASYNCHRONOUS env confirmed on both Pi nodes.

Evidence: `qa_c3_udp_rtps_startup_tools_20261020/runs/qa_baseline_combined_laptop_20261020_214023`,
`pcap_review_20261020_214023.json`, `runtime_review_20261020_214023.json`.
C4 selected to test fewer laptop Nav2 participants, rather than reducing camera
FPS/controller rates or changing radio/watchdog. Assistant analysis only.

## C4-COMPOSITION-CANDIDATE-20261020-12

User authorized implementation for WiFi optimization while preserving vehicle
behaviors. Candidate on branch `codex/laptop-baseline-recovery`, parent C2
`59283c550ece98acd4c8d78174ae016bf81ce199`; **NOT_COMMITTED, NOT_PUSHED**.

Added opt-in `nav2_composed_shadow.launch.py`: exactly10 baseline Nav2 components
in `component_container_isolated`, with a separate executor per component.
Keeps names, lifecycle membership, parameter order/YAML rewrites, no_reverse BTs,
TF remaps and controller→cmd_vel_nav→smoother/behavior→mission shadow output.
No extra waypoint node. Read-only preflight checks map/YAML with baseline validator,
registered plugins/libraries/executable; rejects production command overrides.
Container exit propagates launch shutdown, including optional RViz. No respawn.
Direct rclcpp_components runtime dependency and strict installed allowlist updated.

Demo gains opt-in `--nav2-composed`; default still selects original shadow launch.
Existing --live requirement, cancel exact-goal, stop/alignment/IR/pump admission and
session cleanup remain source-identical outside profile selection. Mission,
perception, safety, Pi C2 I/O, camera15FPS/model, firmware N8R2 and DDS/AP settings
unchanged. This preserves baseline da0dc707 behavior; deferred M3 expansion is
not introduced. Shared container failures affect all Nav2 nodes; independent Pi
watchdogs remain required. Fallback is a clean session using the previous profile.

QA prepared: compare all10 components' evaluated parameters/remaps against unchanged
baseline launch, one isolated container/exit event linkage, shadow override denial,
map/plugin validation, default/opt-in CLI and unchanged live permission. Existing
launch/params/artifact mutation/session gate regressions included. Installed-artifact
validation is deferred until user build/install (new file absent in current install),
not relaxed. Assistant performed static source/XML syntax and diff/hash review only.
**OFFLINE_QA_NOT_RUN; BUILD_NOT_RUN; RUNTIME_NOT_RUN.** No latency improvement claim.

Packet: `qa_c4_composed_candidate_20261020`; source candidate manifest pins201 files.
Next user runs verify_offline.sh, assistant reviews raw/hash/diff, then requests
commit+push approval with full body. After approval/build, compare matched15s startup
traffic/participant/odom/TF/JPEG/SSH with C3 before readiness+YOLO/longer static run.
Do not run motion or flash firmware as part of this gate.

NEXT_STEP=USER_RUN_C4_COMPOSITION_OFFLINE_QA


## C4-COMPOSITION-OFFLINE-PASS-20261020-13

User ran `qa_c4_composed_candidate_20261020/runs/20261020_221008_cL1qdg`.
START2026-10-20T22:10:08+07:00, END22:10:09, RC0:
**58 passed, 1 deselected in 0.86s; C4_COMPOSITION_OFFLINE_QA_OK=1**.
Assistant verified11/11 raw-log hashes,3/3 pinned tool hashes and201/201 current
candidate source hashes. Both source_before/source_after logs show201OK. Current
tracked diff exactly matches saved tracked.patch and both new launch/test files
match the tested copies. No runtime source changes after PASS; no repeated test.
Evidence: packet `offline_gate_review.json`. Parent59283c5, branchbaseline recovery.

Covered offline: all10 components' evaluated parameter/remap parity, isolated
container/exit event shutdown, shadow override denial, map/plugin preflight,
opt-in/default CLI/live permission and existing launch/params/allowlist mutations/
mission admission regressions. The one deselected check is installed-artifacts
allowlist: existing install lacks the C4 launch until user builds navigation.
The validator remains strict; run it after build and verify source/share bytes.

**OFFLINE_PASS_REVIEWED; BUILD_NOT_RUN; RUNTIME_NOT_RUN; NOT_COMMITTED; NOT_PUSHED**.
No WiFi/odom/TF/JPEG improvement or live vehicle behavior acceptance inferred.
N8R2 firmware, Pi C2 I/O/safety, camera15FPS, model and baseline mission unchanged.
Prepared full commit-body draft; wait for user's commit authorization, then commit
and push per standing preference. After that: user builds laptop navigation only,
installed check/hash review, then bounded static capture matched15s to C3 before
readiness+YOLO/longer static. No Pi build, flash, goal or arm for this offline gate.

NEXT_STEP=USER_APPROVE_C4_COMMIT_AND_PUSH_THEN_LAPTOP_BUILD


## C4-COMMIT-PUSH-AUTHORIZED-20261020-14

User explicitly authorized commit and push after offline58PASS review. The C4
candidate source manifest remains201/201 unchanged from the tested run221008.
Commit scope: composed Nav2 opt-in profile, direct dependency/install allowlist,
parity and CLI regressions, demo selection/docs and this validation history.
Commit body records the C3 motivation, exact offline evidence, preserved baseline
behavior/15FPS/N8R2/Pi safety, shared-container tradeoff and remaining gates.
At commit preparation: offline PASS reviewed; build/install/runtime NOT_RUN.
Push completion and commit SHA must be checked against the remote, then recorded
in canonical checkpoint; do not claim remote success from a local commit alone.

NEXT_STEP=VERIFY_C4_REMOTE_COMMIT_THEN_USER_BUILD_LAPTOP_NAVIGATION
