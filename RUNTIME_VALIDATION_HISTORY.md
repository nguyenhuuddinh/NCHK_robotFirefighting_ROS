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


## C4-BUILD-TOOLS-AND-TRACKED-MAP-ALLOWLIST-20261020-15

C4 commit2ab8806 push was confirmed against remote SHA; all9 blobs/body and201
committed candidate hashes matched. User requested the next build step.
While preparing installed-artifact validation, static review found a pre-existing
baseline fixture mismatch: tracked maps/fire_room_demo.pgm and .yaml are installed
by CMake, but absent from the test's required/optional lists. Original C4 offline
gate correctly deferred installed-artifact validation; it did not detect this.

QA-only follow-up adds these exact2 tracked assets to both required fixture lists;
no wildcard/validator relaxation and unknown installed artifacts remain rejected.
Production C4 source/parameters/BTs/Pi/mission/model/15FPS/N8R2 unchanged.
This QA test edit is NOT_TESTED/NOT_COMMITTED and must be distinguished from
commit2ab8806's previously verified58PASS. Do not rewrite that gate's raw evidence.

Prepared user-run packet qa_c4_build_tools_20261020: clean baseline build environment,
only fire_robot_navigation build (300s deadline), source/share byte verification,
read-only installed component/library availability, show-args introspection,
then the full artifact/invariants file (includes the deferred test and mutation
fixtures), with pinned201sources and structured stage/RC/hash logs. No ROS nodes,
serial, camera, GPU inference, Pi build or firmware upload. Assistant static syntax/
diff/hash review only; BUILD_AND_INSTALL_GATE_NOT_RUN. Failed artifact checks
must preserve the files for review, never silently delete unexpected maps.

NEXT_STEP=USER_RUN_C4_LAPTOP_BUILD_INSTALL_GATE


## C4-BUILD-PASS-AND-CAPTURE-TOOLS-20261020-16

User build gate run qa_c4_build_laptop_20261020_222047_D0DCyU:
START22:20:47/END22:21:12+07 on2026-10-20, RC0, C4_LAPTOP_BUILD_INSTALL_OK=1.
fire_robot_navigation built0.72s / total0.98s. User installed-invariants file:
**6 passed in0.10s**, including the previously deferred installed-artifact check
and exact tracked-demo-map fixture/mutation follow-up. Assistant verified25raw
log hashes,7tool hashes,201source hashes before/after,14installed resource bytes,
10registered component manifests/libraries, correct baseline prefix/share and
installed profile import. show-args rviz/use_sim_time false. No matching robot
processes before/after. No runtime node/container, Pi, serial or inference launched.
The QA-only map fix is now PASS_REVIEWED but remains NOT_COMMITTED; runtime C4
commit2ab8806 unchanged. Evidence: qa_c4_build_tools_20261020/build_gate_review.json.

User resumed after quota interruption. Rechecked raw/hash/source and finished
qa_c4_composed_capture_tools_20261020 from immutable C3 basis.37pinned tool inputs;
27/33 C3 inputs unchanged. New identity checks: PiC2sha59283c5/199sources versus
laptopC4sha2ab8806/201sources with the tested QA-only file pinned; no arbitrary dirty
or staged changes permitted. Installed C4 bytes/plugins verified before opening UART.
Laptop launches composed shadow only; CPU sampler recognizes container and launch
request wall/monotonic timestamp is recorded. Pi runner/capture backend/caps/safety/
observer/UART/delays unchanged. No goal/arm/RViz/YOLO/Pi build/ESP upload.

Tool tests cover fake per-host source/SHA/status checks and container sampling,
plus inherited31regressions for owned cleanup, capture limits, UART no-write and
safety stop/RPC timeout evidence. Assistant reviewed diff/hash/Python+shell syntax
only. **CAPTURE_TOOLS_OFFLINE_NOT_RUN; C4_RUNTIME_NOT_RUN**. Do not reuse C3's QA
marker as C4 PASS. User must run new verify_tools.sh; assistant reviews raw/hash
before copy/run. Build and58-test gate need no repeat.

After tools PASS review: one paired static WiFi capture (max90s capture,200kpacket,
128byte snapshot,ring32MB; full session about2–4min, Pi240sfallback), native USBPi/
COMlaptop/15FPS/brightemptycamera, AP connected/both EthernetDOWN. Compare matched
15sstartup with C3, not total90s versus C3~17s. Report coverage/caps/drop/truncation,
participant/SEDP/HB/ACK, odomTF/JPEG both hosts, SSH/CPU/clock bounds/UART/kernel.
RC0 only capture completion, not readiness/steady/live behavior acceptance. QA-only
follow-up commit remains subject to user authorization (then commit+push).

NEXT_STEP=USER_RUN_C4_CAPTURE_TOOLS_OFFLINE_QA


## C4-CAPTURE-TOOLS-OFFLINE-PASS-20261020-17

User ran qa_c4_composed_capture_tools_20261020/offline_runs/20261020_223518:
**42 passed in0.16s**, RC0, END2026-10-20T22:35:19+07:00.
Assistant verified4/4 raw-log hashes,37/37 immutable tool hashes, exact tools_used
manifest and OFFLINE_QA_OK digest. All201 laptop source hashes still match the
built/tested C4 plus exact QA-only mapallowlist follow-up. No source/tool edits
after this PASS. Evidence: packet offline_gate_review.json. Pi199-source identity
is pinned but actual Pi validation is deferred to operator-run preflight; assistant
did not SSH or launch anything in this review. Runtime C4 remains NOT_RUN.

Next: user copies this exact packet+QA marker to Pi over AP10.0.0.1, keeps native
USBPi/COMlaptop,15FPS/brightemptycamera/motor+pumpOFF and both EthernetDOWN.
Two laptop terminals: B copies packet and sudo -v (tcpdump only), A opens ssh -t
and runs Pi runner; B uses exact LOG_PI from A within180s. Runner starts PiI/O,
UART/odomTF/JPEG/CPU/WiFi observers then only composed Nav2laptop; no RViz/YOLO/
pose/goal/arm. Max90s UDP capture/200kpacket/128byte/ring32MB; about2–4min full
session with owned cleanup/autocopy and Pi240sfallback. Keep WiFi until logs copy.
After LOG_LAPTOP review outer/nestedPi/UART hashes and matched15sstartup against
C3. RC0 marks capture completion only; not readiness/steady or motion acceptance.
No repeated build/58-test gate/firmware upload needed. QA-only follow-up commit
still pending user authorization; no commit/push performed in this tools review.

NEXT_STEP=COPY_C4_TOOLS_AND_USER_RUN_ONE_PAIRED_STATIC_WIFI_CAPTURE


## C4-STARTUP-CAPTURE-RESULT-20261020-18

User pairedWiFi capture223923/Pi223859 completedRC0,copyRC0. Assistant reviewed99
outer/42nestedPi/3UART hashes independently (nested included in99),37tool/201laptop
sources, Pi199/laptop201beforeafter checks. Laptop2ab8806/Pi59283c5, actualASYNC2Pi
processes, bothEthernetDOWN/AP/NTPyes. No nonzero/forbidden commands; finalmatching
processes0. No assistant runtime/test/build execution or source changes.

C4capture68190packets/89.251s first-to-last,90stimeoutRC124;68476receivedfilter,
0kerneldrops. Same15s launchlogcreation anchors: C3 185308packets→C4 39485(-78.69%);
observed laptopGUIDprefix11→3,Pi9→9; discovery+builtin originalframebytes27003910→
3683264(-86.36%); HB136509→6550(-95.20%); repeatedsamepeerSEDP10147→167.
Fragment attribution uses observed initial src/dst/IPid/protocol within2s and valid
UDPoffset;16C4/9C3fragments remainunknown,128bytesnapshot may hide mixedsubmessages.
Totaloriginalframebytes30703392→31509067(+2.62%): not allWiFibytes reduced. Userwriter
traffic increases; do not infer allJPEG or RFpacketloss/airtime from captures.

Laptop gaps fullyinside15s: odom2229.9→710.4ms,TF1609.7→552.4ms,JPEG2225.0→1320.4ms.
PiC4full123.26s maxodom169.2ms/JPEG162.4ms/TF402.3ms. Many5sbins10Hz/15FPS butlaptop
stilldips andsourceage~880msodom/JPEG,976msTF. Wholeobserver130.77s includesstartup
silence;steady0, notfullreadiness. CPUlaptop2.32–4.18%/Pi29.79–65.08%, container37
threads/~84.4MiB. SSH44RC0/max2s;pingloss1.53%/2.4%,max~2.04/2.14s. iwmissingboth
hosts;clockboundRTT102–107ms, noonewaylatency/radioretryclaim.

UART264lines. ROMPOWERON/nativeUSBre-enumeration coincides withCOMopen22:39:26,
beforePiI/O/Nav2; onebridgeopen/sameknownELF, nonewstall/recovery/USBdetach later.
ThisdoesnotcertifyUSBintermittentfixed. Nav2loaded10components/localizationactive;
navigationwaitingmapTFwithoutInitialPose asdesigned. NoREADY/YOLO/mission/liveproof.
ContainerSIGINT22:41:39.255 butTFwaitthrough22:41:49.195, Nav2launchRC143/TERMfallback;
cleanupfinallyclean doesnotprovegracefulshutdown. Preserve that limitation.

Evidence: qa_c4_composed_capture_tools_20261020/runtime_review_20261020_223923.json,
matched15s/fragment/anchor outputs and offline analysis scripts; canonical
C4_WIFI_STARTUP_RESULT_2026-10-20.md. Discoveryreduction supported in one startup,
residualWiFigaps/readiness/cleanup stillopen. Nextprepare boundedstaticreadiness,
userInitialPosemap9_10/minimalRViz/no goal orarm; freshlifecycle/TF/safety andowned
cleanup, onlythenYOLO/longerstatic. Newgateplanonly, notimplemented/tested. No same
capture repeat/build/flash. QA-onlymapallowlistfix6PASS stillpendingcommitapproval.

NEXT_STEP=PREPARE_BOUNDED_C4_INITIAL_POSE_READINESS_AND_GRACEFUL_CLEANUP_GATE_NO_GOAL


## C4-PERFORMANCE-LEDGER-AND-POSE-TOOLS-20261020-19

User explicitly requested durable metric history for comparing code revisions and
future report writing. Canonical PERFORMANCE_COMPARISON_LEDGER.md/.json now holds
C1/C2/C3/C4 status, full runtimeSHAs/toolIDs/runIDs, exact matched15s metric values,
conditions/methods/raw paths and limitations. REPORT_EVIDENCE_INDEX.md explains
how to trace hashes/analysis and separate requirements, offline/build, startup,
pose-readiness, YOLO and live behavior evidence. docs/codex/rawQA remain local
Gitignored; do not claim pushed. This tracked history is the commit/run linkage.

For durable report reference, C3/C4 matched15s: laptopGUID11→3/Pi9→9; packets
185308→39485(-78.69%); discovery+builtinframebytes27003910→3683264(-86.36%);
HB136509→6550(-95.20%); ACK22866→5990; totalframebytes30703392→31509067(+2.62%).
Laptop monotonicgapodom2229.8555→710.3643ms/TF1609.7366→552.3951/JPEG2224.9947→
1320.4450ms. One run/profile, not statistically repeated; same15s launchloganchor,
RTPSentity+initialfragmentattribution,128Bsnaplimitations, no RFairtime/topicpayload
inference. RawC3run214023/C4run223923 immutable. New20spost-pose row must be separate,
NOT_RUN until evidence arrives; never overwrite earlierblocked/failed results.

User requested continued development of the bounded C4 pose/readiness gate.
Prepared qa_c4_pose_readiness_tools_20261020,41pinned inputs derived from immutable
C4capturebasis. Runtime201laptop/199Pi source hashes unchanged. New driver reuses
one Combined observer/Probe with boundedservicefutures, waits explicitRVizstage
then180s for userpose and5sstablefreshTF/lifecycle/safety/JPEG, measures20s only
after proof. No autopose/goal/arm/YOLO/UDPcapture. ExistingProbe report thresholds
unchanged; JPEG adds>=10FPS andgap/age/current<=500ms. Startupmetrics reset at
measurement start; safetyviolations retained, posechanges/lifecycleexits recorded.

Cleanup now stops ownedNav2 before PiI/O, thenRViz andobservers. ExistingINT10s/
TERM2s/KILL strategy retained but records eachsignal/PID/label/time andNav2exitRC;
finalpolicy neverclaimsgracefulPASS ifTERMorKILL used. Requires verifiedcopiedPi
manifest/sourceRC/cleanup andreadiness/JPEG proofs. Pi runner fallback480s needed
for warmup+manualpose180s+measure20s+cleanup; watchdog/TF/command guards notchanged.
Includes inheritedtoolregressions plus fakeclock/posechanges/no-pose/telemetry/
owned-signals/corrupt-copy tests. No actual signals/ROS/ports/SSH in offlinegate.

Assistant staticdiff/Python+shellsyntax/hash review only.
**NEW_TOOLS_OFFLINE_NOT_RUN; POSE_READINESS_RUNTIME_NOT_RUN; NO_NEW_COMMIT_PUSH**.
User runs new verify_tools.sh then sends LOG_QA; assistant reviews raw/hash before
copy/run. Existingruntime C4 stays2ab8806/Pi59283c5/N8R2/15FPS/model unchanged;
QAmapallowlistfix6PASS andhistory remain dirty awaiting commit approval. No need
rebuild/flash or repeat58source tests/startup capture. After measuredreadiness+
SIGINTcleanupPASS, onlythenYOLOlast/longerstatic beforelivebehavioracceptance.

NEXT_STEP=USER_RUN_C4_POSE_READINESS_TOOLS_OFFLINE_QA


## C4-POSE-TOOLS-OFFLINE-PASS-20261020-20

User run qa_c4_pose_readiness_tools_20261020/offline_runs/20261020_231436:
**60 passed in0.23s**,RC0,END2026-10-20T23:14:36+07:00. Assistant verified4rawlog
hashes,41toolinputs,exacttestedmanifest+OFFLINE_QA_OK marker and201currentlaptop
sourcehashes. No tool/runtime edits afterPASS. Evidence: packet offline_gate_review.json.
Pi actualsource199/SHA59283c5 validated by operator-run preflight, not assumed from
laptopQA. C4laptop2ab8806/PiC2/N8R2/model/15FPS unchanged. QA-onlymapallowlist6PASS
andhistory remain dirty; no newcommit/push here. Gate status: TOOLS_OFFLINE_PASS,
MANUAL_POSE_RUNTIME_NOT_RUN. Ledger/index now records toolstage separately without
filling newruntime metrics or mixing post-pose20s with startup15s.

Next user copy exactpacket+marker toPi overAP, bothEthernetDOWN, nativeUSBPi/COMlap,
noSerialMonitor/oldROS,motor+pumpOFF,brightemptycamera. TerminalA laptopSSH-tPi
run_pi.sh, B exactLOG_PI run_laptop.sh within180s. PiI/O30s→Nav2laptop20s→minimalRViz;
user2DPoseEstimate correctmap9_10/location/orientation when prompted, nogoal/arm.
Waitdeadline180s then20smeasurement only after5sstablefreshpose/lifecycle/TF/safety/
JPEG. Do not change pose duringmeasurement. Normalfullsession3–5min, up to~7min
if posewait/cleanup/copy take time, PiIOfallback480s. KeepWiFi untilbothfinish.
Nav2shutdown beforePi, recordownedINT/TERM/KILL. Only fullfinalverdict andRC0 plus
raw/hash/cleanup review can pass thisstaticgate; notYOLO/motion/actuatoracceptance.
No sudo/tcpdump laptop, noPi build/ESPflash/repeated58source tests needed.
Send LOG_LAPTOP/receipt. Updateledger/report/history afterrun includingBLOCKED;
ifready+cleanupPASS thenYOLOlast/longerstatic, otherwise diagnose failed proof.

NEXT_STEP=COPY_POSE_TOOLS_AND_USER_RUN_ONE_STATIC_MANUAL_INITIAL_POSE_GATE


## C4-POSE-RUNTIME-BLOCKED-AND-C5-SHUTDOWN-CANDIDATE-20261020-21

User capture qa_c4_pose_readiness_tools_20261020/run232003 finished FINAL_VERDICT
BLOCKED_READINESS_OR_TELEMETRY, FINAL_EXIT_RC2. Verified99outer/43nestedPi/3UART
hashes (nested already inouter),41testedtools and201laptop source atcapture;
Pi199/C2 manifests source before/after correct. No raw/tool edits. RuntimeC4
laptop2ab8806f826f4ad7fa1f0bb4842da5deb46bb207/Pi59283c550ece98acd4c8d78174ae016bf81ce199.
Manualpose acknowledged; all8lifecycle active start/end; no observed exits/posechanges.
20.020859s postpose from23:21:37.090334+07: countsodom191/TF364/scan101/mapTF86/
zerocmd191; forbidden/nonzero0. Three checks fail only: gapsodom257.514306ms,
TF256.983625ms vs250; scan525.647864ms vs500. Ageodom149.666974/TF219.701880/
scan378.797156ms withinexistinggates. JPEG278/13.885518FPS,gap257.493668ms,
age186.883013ms; allJPEGchecksPASS. Stationaryamclcount0 has preservedpriorposeproof;
10controllerRPCtimeouts accumulatedbeforemeasure, notnewfailuresduring20s.

Additionalblocker: Nav2SIGINT23:21:57.559, lastpreshutdown~8.526slater, TERM10.031s
afterINT, attemptedKILLlogged; launchRC143. Final emptyprocesslists bothhosts and
RVizRC0 do not provegracefulshutdown. First-failurepolicytelemetry hides thissecond
blocker. Pi overlappingbins remain nearodom104–111/TF55–100/scan201–205/JPEG74–84ms
gaps while one laptop5sbin stallsacrosstopics. Transport/receiversidehypothesis,
notproofRFvsDDSvsCPUscheduler. Fullcapturepingloss1.626%lap/7.759%Pi, different
scope from20s. Headerage notpureonewaylatency,clockRTTbound131ms. USBPOWERON/detach
alignsCOMopenbeforeIO; no newdetach/stallduringmeasure, notlongtermUSBacceptance.

Canonical C4_POSE_READINESS_RESULT_2026-10-20.md, performanceMD/JSON separatepostpose
row/runtime_runs, reportindex/checkpoints updated. PriorC3/C4startup15s preserved,
notdirectlycomparable. runtime_review_20261020_232003.json keepsrawpaths/SHA/limits.

Static inspection found installedHumble ComponentManagerIsolated::cancel_executor
waitswhile!is_spinning withoutcontextguard. Aftershutdown spinhasreturned, wait
canneverfinish. Upstreamros2/rclcpp#2083/#2085 andHumbleheadercontextguard support
targetedbackport. No capturedstack proves thisissolecause ofNav2RC143. Prepared
C5 localbinary/privateApacheheader withguardrclcpp::ok(context), samepercomponent
SingleThreadedExecutors/manager. Composedprofileonly pointsatlocalbinary andchecks
itsinstallation. Exactly10components/params/routes/BT/lifecycle/mission unchanged.
No /optROS/Pi/N8R2/camera15FPS/model/QoS/watchdog/TFthreshold edits. CMakebuilddeps
and4realC++localcontexttests: empty, executorsalreadyreturned, immediatecontext
shutdown, liveunloadwhileothercomponentruns. CTest30s boundsregressionhang; no
detachedthreadhidingfailure. UserQA builds onlynavigation, thenlocalC++/Python
parity/installed tests. Loopbackdummynodesonly,no robotlaunch/ports/commands.

Packetqa_c5_isolated_shutdown_candidate_20261020 pins204sources/6toolinputs,
expectedstatus/headerbeforeSHA. C5candidate7filespluspreservedQAmapfix/historydirty;
assistantstaticdiff/syntax/hashonly. C5BUILD_QA_RUNTIME_NOT_RUN, NO_COMMIT_PUSH.
C5 fixesashutdowncandidate, notresidualWiFigaps. Do notrepeattheoldC4runtimepacket
withchangedsource. NextuseronelocalQAcommand, reviewLOG_QA, thencommit/pushonly
onuserapproval beforepreparingnewruntimegate.

NEXT_STEP=USER_RUN_C5_SHUTDOWN_LOCAL_BUILD_AND_REGRESSION


## C5-LOCAL-BUILD-SHUTDOWN-QA-PASS-REVIEWED-20261020-22

User ran qa_c5_isolated_shutdown_candidate_20261020/verify_offline.sh,
runqa_c5_shutdown_20261020_233644_oJ6zWj,23:36:44–23:36:56+07,RC0/stageCOMPLETE.
Build fire_robot_navigation7.67s/colcon7.85s withrclcpp/components16.0.19,
nav2_bringup1.1.20/rmw_fastrtps_cpp6.2.10. FourrealC++dummy localcontext testsPASS:
EmptyManager10ms; ShutdownAfterAllExecutorsAlreadyReturned8ms;
ShutdownImmediatelyAfterExecutorCreation5ms;
LiveExecutorUnloadsAndRemainingExecutorStillRuns16ms. gTest39ms/CTest0.20s,
30stesttimeout nothit. Pythonlaunchparity/installed/session checks59PASS0.94s,
no deselection. Broadlintconfiguredbutnotrun; do notclaimfull-lintPASS.

Assistant verified29rawhashes/6toolinputs/204currentcandidate sources, testedtool
manifestexact, statusbefore/afterunchanged/processlistsbothempty. Installedlocal
binarySHA2562b233273d573bf5649f49c9b47f829a2de6b704d2c317842fbce85c62bc24c8f
matchesrawreceiptfile. InstalledbasisheaderSHAverified. No source/toole editsafter
PASS; onlyreviewdocs/historyupdated. Derivedreview offline_gate_review_20261020_233644.json,
raw_verification_20261020_233644.json andsavedgTestXML atpacketroot. Rawimmutable.
Canonical C5_ISOLATED_SHUTDOWN_QA_RESULT_2026-10-20.md/performanceledger/index updated.

Status C5LOCAL_BUILD_AND_SHUTDOWN_QA_PASS_REVIEWED, parent2ab8806 +dirtycandidate;
C5NOT_COMMITTED/NOT_PUSHED. Tests uselocaldummyROS/loopbackonly, no fullNav2stack,
Pi/USB/camera/YOLO/goal/actuator. Thealreadyreturnedexecutor case nowcompletes,
notproofofallpluginshutdownsorallschedulerinterleavings. C5realNav2SIGINTshutdown
andWiFi performanceNOT_RUN. C4pose232003remainsBLOCKED:3gapoverruns andTERM/RC143.
NeverlabelitPASSbecauseC5unittestsPASS. N8R2/PiC2/camera15FPS/mission/model/QoS/
watchdog unchanged; QAmapallowlistfix6PASS preserved andcoveredbycurrent59tests.

Reviewedcandidate/diff/hash readyforusercommitapproval. Whenuserauthorizescommit,
commitANDpushwithfullbodyplus thishistory; include7C5files,existingQAmapfix and
historyonly, notmainphase3dirtyfiles/localrawlogs. ThenpinnewSHAandprepareone
boundedstaticmanualpose20s/readiness+SIGINTcleanupcapture, PiI/O keptaliveuntil
Nav2stop. Same telemetrythresholds. CurrentC4packetpin201mustnotbeusedwithC5source204.
No repeatbuild/QA/firmwareflash orrobotrunneededbeforeapproval.

NEXT_STEP=C5_REVIEWED_COMMIT_PUSH_APPROVAL_THEN_PREPARE_RUNTIME


## C5-RUNTIME-TOOLS-PREPARED-AND-ACCEPTANCE-ROADMAP-20261020-23

User requested preparation after C5 push and asked which C milestone ends the
roadmap. Confirmed C5commitfdea296746161c4233364b7de95fdca0adf105b1 already pushed,
source204 matches4C+++59Python QA; PiC259283c5/N8R2/15FPS/model unchanged.
Prepared qa_c5_pose_runtime_tools_20261020 from immutable41input C4pose basis.
New46pinnedinputs, no inheritedPASSmarker. Verify204laptop/199Pi Gitblob hashes,
204currentlaptopfile hashes and exacttestedlocalbinary, Pythonparse/shellsyntax/
diffonly. Assistant didnotrunpytest/build/ROS/SSH/serial/hardware.

Tools pinC5identity andbinaryhash beforeCOMopen; no moredirtyQAfilewhitelist,
onlydocumentationhistorypermitted. Probe/PhaseGate/thresholds/Pirunner/UART/
observer/commoncleanup identical toC4. StartupQAmeasurementconstant now20s,
matchingPhaseGate; actualreadinesstimecontrolunchanged. Existing20saftermanualpose,
5sstableready,180sposewait and480sPifallback. Nav2SIGINTbeforePiSTOP, same10sINT/
2sTERM/KILLtimeouts. Finalpolicy unchanged; newgate_findings.json exposesindependent
telemetry/JPEGfailchecks andNav2shutdownPASS/BLOCKED/INCOMPLETE, signalsareattempts.
Regressions coverbinarysymlink/missing/tampered/nonexec/manifestpath,oldC4SHAreject,
telemetryfailwithshutdownstatusvisible andincompleteevidencenotgracefulPASS.

Newpacket OFFLINE_QA_NOT_RUN/RUNTIME_NOT_RUN. C5sourcePASS/C4tools60PASS donotcertify
newtools. Userrunsverify_tools.sh onlaptop(noPi/build/realportsneeded), sendsLOG_QA;
reviewraw/hashbeforepairedWiFicapture. No runtimeorfirmwareedits. Newrunbook and
BASELINE_ACCEPTANCE_ROADMAP_2026-10-20.md define nofixedlastC: R1statictelemetry+
cleanup, then R2YOLO/timedstatic, R3baselinegoals/cancel, R4baselinefirebehavior.
R2–R4plansonly,hardwareopt-in/armseparate; C6onlyforconcreteremainingdefect.
Useracceptsshortjitter/recovery butnonewperformancegateapproved; donotrelaxguards
orretroactivelyPASSC4. README/index/checkpoints/ledgercurrentupdated toavoid stale
"currentC1" pointers. docs/rawlocalignored, keepseparatebackup; historyconnects
nextcommit. AftertoolsQA recordPASS/BLOCKED+NEXT_STEP, thenprovideoperatorcommands.

NEXT_STEP=USER_RUN_C5_RUNTIME_TOOLS_OFFLINE_QA


## C5-RUNTIME-TOOLS-OFFLINE-QA-PASS-REVIEWED-20261020-24

User run qa_c5_pose_runtime_tools_20261020/offline_runs/20261020_235552,
72passed0.25s,RC0,END23:55:52+07. Assistantverified4rawlogs/46toolinputs/
204currentlaptopsourcehashes andpinnedC5binary; tools_usedexactcurrentmanifest,
OFFLINE_QA_OK markerhashcorrect. No tool/runtimechangesafterPASS. Evidence:
packet offline_gate_review_20261020_235552.json. Pi199sources/C2HEAD verifiedby
operatorpreflightonPi, notassumedfromlaptopQA. Tooltests usefakefiles/Git/time/
signals/serial; no realROS/network/hardware. C5NAV2_RUNTIME_NOT_RUN remains.

Updatedcanonicalrunbook/ledgerMD+JSON/reportindex/roadmap/checkpoint including
exact72PASS andlimits; immutablepacketREADMEkeepsprepared-timecontext. Nextuser
copyexactpacket+QAmarkertoPiAP, nativeESPUSBPi/COMlaptop, bothEthernetDOWN,
oldROS/RViz/SerialMonitorclosed,motor/pumpOFF,brightemptycamera. A laptopSSH-tPi
run_pi;B exactLOG_PI run_laptopwithin180s. PiIO30s/Nav2laptop20s/minimalRViz,
manualInitialPosemap9_10/correctpositionorientation,5sstablethen20smeasure;
posewait180s,fullsessionusually3–5minupto~7min,keepAPthroughcopy. No goal/arm/YOLO/
newbuild/flash/repeatedsourceQA. StopNav2beforePi,signalsattemptslogged,independent
findingsdonotrelaxfinalpolicy. SendLOG_LAPTOP/verdict/exit;reviewraw/hashandsave
PASS/BLOCKEDmetrics before nextaction. C4blockedgap/TERMhistoricunchanged.
Laptoppushedfdea296/Pi59283c5/N8R2/15FPS/model unchanged,historyonlydirtyallowed.

NEXT_STEP=COPY_C5_TOOLS_AND_USER_RUN_ONE_STATIC_POSE_SHUTDOWN_GATE


## C5-RUNTIME-BLOCKED-CHILD-CRASH-CORRECTION-20261021-25

User qa_c5_pose_runtime_tools_20261020/runqa_baseline_combined_laptop_20261020_235849,
START23:58:49Oct20/END00:00:45Oct21+07,finalBLOCKED_READINESS_OR_TELEMETRY/RC2.
LaptoppushedC5fdea296/PiC259283c5/N8R2/15FPS/model unchanged. Verified100outer/
43nestedPi/3UART/46tool/204laptopsource/1binaryhash; nestedalreadyinouter.
Testedtoolmanifest/markerexact,sourcebeforeafterlap204/Pi199correctSHA,copyRC0,
sourcecheckRC0,processafterbothclean,nonzeroinvalid/forbidden0bothhosts. Rawuntouched.

Manualpose/AMCL/mapTFproof/8lifecycleactivebeforeafter20.000026s,00:00:11.774012–
31.774038Oct21+07. Noobservedexit/posechange. Countsodom184/TF343/scan93/mapTF75/
zerocmd183;JPEG263/13.149983FPS. Sixfailingchecks:odomgap579.938014/TF443.328932/
scan1293.551711ms,ages890.333359/983.636973/1182.460606ms. JPEGgap743.870484ms,
age1005.321896msfail500ms;rate/currentPASS. Endcurrentrecoversbutmaxfailretained.
Pi5sbinsodom49–50/gap151–156ms,TF100/gap54–58ms,scan26/gap199–201ms,JPEG75/
gap76–82mswhilelapstallsacrosstopics. CPU10smeanslap3.96–4.35%total/Nav2~17–18%
onecore,Pi31.65–34.36%total;notproofnoCPUspikes. Samplermissesreadiness_driver
perprocCPU. Pingfullcapturelap106/106/0lossavg106.451max2064.033ms;Pi98/92/6.122%
lossavg74.539max1407.335ms;not20s-only. iwmissing/noRFtracedata. RPCerrors7controller/
1safetyaccumulatedbeforemeasurement,nogrowthduring. NoUSBdetach/CDCstallinrun;
UARTstartupwatchdogbeforemeasurement,notUSBreset. NoWiFirootcauseproven.

CRITICALCORRECTION: rawgate_findings.nav2_shutdown=PASSisfalseacceptancefromlaunchRC0.
ContainerPID169827afterSIGINT loggedMagickSIGSEGVsignal11thenchildexit-6. NoTERM/
KILLattempt,cleanupwithin~5.517sdoesnotprovegraceful;processdied. Bondbt_navigator
error/reset/transitioncollisionsoccurafterSIGINT;pollingdoesnotproveanysubsecond
lifecyclehistory. LastDestroyingbehavior/Magicktextdoesnotidentifycausalplugin.
NoexactC5executablecorefoundin/var/crash,oldunrelatedreportsnotopened. Rootcause
teardownUNDETERMINEDwithoutbacktrace. PreserveC5local4C+++59PythonPASSbutC5actual
shutdownBLOCKED_CONTAINER_CRASH. Do notretroactivelymarkC4orC5runtimePASS.

CanonicalC5_STATIC_RUNTIME_RESULT_2026-10-21.md/performanceMDJSON/reportindex/
checkpoint/historyupdated,one-runC4C5postposecomparisondistinctstartup15s.
Derivedruntime_review_20261020_235849.jsoncontainsCPU/bins/errors/correction.
Preparedseparateqa_c5_shutdown_evidence_fix_20261021:4pinnedinputs,6puretextcases,
parserrequirescleancontainerchildexit+INT/nonforcedlaunchRC;fatal/deathblocks,
missingchildproofINCOMPLETE. NEW_QA_TOOLS_NOT_TESTED; assistantparse/syntaxonly.
Noeditimmutable46inputpacket/raw/runtimesource/Pifirmware. NotC6runtimepatchyet.
Userrunonlynewverify_tools,sendLOG_QA,thenreviewandprepareboundedteardownbacktrace
withchildexitproof. No uninstrumentedrobotrepeat/build/flash/72toolretest/YOLO/
goals/actuators. ResidualWiFigapseparateopenissue;R1notcomplete.

NEXT_STEP=USER_QA_CHILD_EXIT_EVIDENCE_FIX_THEN_PREPARE_TARGETED_TEARDOWN_BACKTRACE


## CHILD-EXIT-PARSER-PASS-AND-GDB-TOOLS-PREPARED-20261021-26

User parserqa_c5_shutdown_evidence_fix_20261021/offline_runs/20261021_001326_xPlhnC:
6PASS0.01s/RC0,END00:13:26+07;4raw/4toolhashes andexacttestedmanifestverified.
No parserchangesafterPASS. CorrectedC5runtime remainsBLOCKED_CONTAINER_CRASH;
launchRC0cannotcertifychildcleanexit. NoC5runtimeorfirmwarefixmadehere.

Prepared qa_c5_teardown_gdb_tools_20261021,55pinnedinputsderivedC5pose46+parserbasis.
NewtoolsQANOTRUN/runtimeNOTRUN. Source204lapfdea296/Pi19959283c5/binarypinnedand
unchanged,N8R2/15FPS/model/mission/guardsunchanged. UsedinstalledHumbleros2launch
CLIlaunchprefix+regexfilterforonlyfire_robot_component_container_isolated; no
sourcelaunchchanges. OwnchildGDB—notattach. GDBpassINT/TERM,stopfatalSEGV/ABRT/
BUS/ILL/FPEbeforehandler,savefirstsignal/time/allthreadbt30/sharedlibsmax2MiB,
thenquitstoppedinferior. Verifiedtraceframe#0beforemarkingbacktracecaptured.
Inferiorexitseparatefromgdb/launchRC,wrapperstoresdebuggerreceipt,330sdeadline;
externalownedcleanup10sINT/2sTERM/KILLandPi480sremain. No globalptracepolicy/
sudo/cores/networksymbols. RuntimeCAPTRACE isdiagnostic,neverperformanceor
staticreadinessPASS. No-reproducecleanexitunderGDBnotfixedproof. Originalgate
retainedinsummary forsafety/copy/source; debuggerstate separateanderrorsvisible.

TestsfakeGDBeventscoverfataltrace/normalexit/INT/missingframes/timeouts/prefixscope,
plusverified6parsercasesandinheritedgatecontrols. UserverifyalsoGDBPythonAPIcheck
withoutinferior. AssistantonlyPythoncompile/shellsyntax/diff/hash,noGDB/pytest/
build/ROS/SSH/serialrun. Useroneofflinecommand,sendLOG_QA;reviewbeforecopyandpaired
AP/manualpose/20sobserve/teardowncapture. Debuggersignal/timingperturbationdocumented,
noWiFigapcausalclaim. C5crashrootcauseawaitsbacktrace,nospeculativepluginfix orC6.
Updatecanonicalplan/ledger/index/checkpointswith6PASSand55newtoolNOT_RUN.

NEXT_STEP=USER_RUN_TEARDOWN_GDB_TOOLS_OFFLINE_QA


## GDB-TOOLS-OFFLINE-PASS-REVIEWED-20261021-27

User qa_c5_teardown_gdb_tools_20261021/offline_runs/20261021_002311:
71PASS0.24s/RC0/END00:23:11+07,GDB_PYTHON_API_OK1(capabilitywithoutinferior).
Verified5rawlogs/55toolinputs/204currentlaptopsources/1binaryhash,exacttested
manifestandOFFLINE_QA_OKmarker. No tool/runtimeeditsafterPASS. Evidencepacket
 offline_gate_review_20261021_002311.json. Pi199sources/C2HEADactualpreflightby
operatornotassumedfromlaptopQA. GDBbacktraceruntimeNOTRUN. C5runtime235849still
BLOCKEDtelemetry+childSIGSEGV/-6,rawshutdownPASSfalsecorrectionpreserved.

CanonicalC5_TEARDOWN_GDB_RUN_2026-10-21.md providescopyandtwo-terminalcommands;
plan/ledger/index/checkpoint/historyupdated. UserAPPi10.0.0.1,bothLANdown,
nativeESPUSBPi/COMlaptop,oldROS/RViz/Monitorclosed,motorpumpOFF,brightemptycamera.
Copyexact55inputs+marker, A laptopSSH-tPi run_pi,B exactLOG_PIwithin180srun_laptop.
PiIO30s/Nav2underGDB20s/minimalRViz/manualposemap9_10/correctlocationorientation,
5sstablethen20sobserve,INTNav2beforePiSTOP,backtraceandautocopy. Usually3–5min,
upto~7minpose/copy,keepAPuntilbothdone. No goal/arm/YOLO/build/flash/QAloop.

FINAL_VERDICTDIAGNOSTIC_/FINAL_EXIT_RC2bydesign evenondiagnosticcleanexit; not
performanceorgracefulacceptance. Fataltracecapturedreviewstackbeforefix;no
reproductionnotfixedproof,norepeatblindly. GDBchangesignal/timing,ownnewchildonly,
noattach/sudo/globalptrace/symbolfetch/core. Capturedeadlines330sGDB/Pi480s and
10sINT/2sTERM/KILLcleanupunchanged. SendLOG_LAPTOP/verdict/exit,thenraw/hash/
stackreviewandnextstep. C5fdea296/Pi59283c5/N8R2/15FPS/model unchanged,historyonly
dirtyallowed. NoC6codepatchyet orprovencrashrootcause;R1incomplete/R2–R4notrun.

NEXT_STEP=COPY_GDB_TOOLS_AND_USER_RUN_ONE_INSTRUMENTED_TEARDOWN_CAPTURE


## GDB-FATAL-BACKTRACE-REVIEWED-AND-LOCAL-AB-PREPARED-20261021-28


## 21/10 — C5 GDB runtime 002615: fatal backtrace captured, đã review

[Báo cáo stack và bước tiếp](docs/codex/current/C5_TEARDOWN_BACKTRACE_RESULT_2026-10-21.md).
Laptop fdea296 / Pi59283c5, ESP32-S3 N8R2, camera15FPS. 105 raw ngoài /43 Pi /
3 UART (nested đã nằm trong outer),55 tool/204 source/binary khớp. Cleanup sạch.
SIGINT attempt00:28:01.782324; GDB capture update00:28:07.473882, SIGSEGV thread1
trong LifecycleNode destructor qua NodeInstanceWrapper/map clear/container dtor.
Không TERM/KILL; GDB RC86 đúng thiết kế, trace đủ. FINAL_EXIT_RC2 diagnostic,
không runtime PASS và không lỗi thu log. Component executor threads không còn
trong snapshot; không loại trừ race/hỏng heap xảy ra trước đó.

Disassembly offset0x22179 tương ứng weak control block release/virtual call;
Spin/Wait libs vắng trong sharedlib list. Giả thuyết plugin bị unload trước base
node weak references, chưa chứng minh root cause. Chưa sửa runtime, không quy lỗi
cho WiFi/ESP/Magick. Pi kernel không entries; UART watchdog đầu phiên rồi recovery
trước đo. Odom gap733.303ms/age594.394;TF717.451/588.775;scan1061.254/971.416;
JPEG242/20.024535s=12.0852FPS,gap867.007/age613.826ms;zero nonzero commands.
Các số dưới GDB chỉ lưu truy vết, không dùng so hiệu năng C4/C5 thường.

Soạn packet riêng qa_c5_plugin_lifetime_ab_20261021,10 pinned input, syntax-only,
BUILD/TEST/PROBE NOT_RUN. Một lệnh user chạy kiểm verdict+compile1job+2case GDB trên
installed BehaviorServer INACTIVE: A không giữ lib; B giữ Spin/Wait owner đến sau
node destructor rồi trả owner. Không robot/Pi/serial/activate/goal/actuator; ROS
localhost tạm thời, domain/firmware/model/guard/15FPS không đổi. A không reproduce
phải INCONCLUSIVE; không gọi clean B là fix. Sau raw A/B review mới runtime patch.
Lệnh trong báo cáo; thường1–3min, cap configure180s/build180s/case40s. Chưa commit.

**NEXT_STEP=USER_RUN_ONE_LOCAL_BEHAVIOR_PLUGIN_LIFETIME_AB**


## LOCAL-PLUGIN-LIFETIME-AB-INCONCLUSIVE-REVIEWED-20261021-29


## 21/10 00:43:15 — local lifetime A/B INCONCLUSIVE, đã review

[Chi tiết](docs/codex/current/C5_PLUGIN_LIFETIME_AB_RESULT_2026-10-21.md). Build probe thành công,
2 unit checks PASS; A/B đều configure INACTIVE, cleanup và EXIT0, không timeout.
RC2=`INCONCLUSIVE_CONTROL_DID_NOT_REPRODUCE`, không phải bản vá đạt. 24 raw/10tool/
204 source/binary/4 installed library hash và exact tested manifest khớp.
Spin/Wait vẫn mapped sau node destroy và lease release ở cả A/B; chưa tái hiện
unload thấy trong full container. Probe direct-links behavior_server_core và
không activate, khác runtime dynamically loaded/multi-node ACTIVE. Chưa chứng
minh library retention là fix; không lặp bài này, không áp patch suy đoán.
Runtime/firmware/15FPS/model/guards không đổi; C5 crash và WiFi/odom còn mở.
Bước tiếp rà offline dynamic loader/ownership/active teardown; chưa có lệnh user
mới hoặc runtime patch/commit/push. Sau fix được QA mới static telemetry acceptance.

**NEXT_STEP=OFFLINE_REVIEW_DYNAMIC_COMPONENT_LOADING_AND_ACTIVE_TEARDOWN_BEFORE_NEXT_PROBE**


## DYNAMIC-COMPONENT-OWNERSHIP-REVIEW-AND-PROBE-PREPARED-20261021-30


## 21/10 — đã rà dynamic ownership và soạn probe đúng manager C5

[Source review và lệnh local](docs/codex/current/C5_DYNAMIC_COMPONENT_REVIEW_AND_PROBE_2026-10-21.md).
Không thấy lý do bỏ join hoặc đổi thứ tự member loader/wrapper từ stack hiện có.
Bài direct-link004315 khác đường nạp/factory/executor/ACTIVE của container thật.
Packet riêng qa_c5_dynamic_component_ab_20261021:11 pinned input, nguyên header C5
fdea296, factory/resource thật, executor/destructor C5. Không direct link component
hay Spin/Wait (ELF check bắt buộc). Bốn điều kiện INACTIVE/ACTIVE x control/retain,
đúng1case/process mỗi điều kiện; không retry. GDB có registers/mappings/control
block reads khi fatal. Scope local một component, chưa full multi-node/signalrace.

STATIC/SYNTAX ONLY: AST/bash/header exact/204sourcehash đạt; build/5unitcases/probe
NOT_RUN. Không sửa runtime/Pi/firmware/model/15FPS/mission/QoS/guard. Domain hiện tại
được giữ cho child, localhost tạm thời; không Pi/AP/USB/camera/goal/arm/actuatorAPI.
ACTIVE trong probe chỉ local lifecycle, cmd_vel namespace QA. User1command thường
1–3min,cap~9min configure/build/4case40s. Sau rawreview mới cân nhắc targetedpatch;
cleancontrol vẫn INCONCLUSIVE, không gọi retain sạch là robotPASS. Chưa commit/push.

**NEXT_STEP=USER_RUN_ONE_LOCAL_DYNAMIC_COMPONENT_LIFETIME_AB**


## DYNAMIC-PROBE-V1-BUILD-BLOCKED-AND-V2-INTERFACE-FIX-20261021-31


## 21/10 00:53:40 — dynamic V1 build FAIL; đã soạn V2 sửa công cụ

[Build failure và lệnh V2](docs/codex/current/C5_DYNAMIC_PROBE_BUILD_FIX_2026-10-21.md). User run005340:
5 unit checks PASS, compiler thiếu smclib/statemap.hpp, bốn case GDB NOT_RUN,
runnerRC2/sourcecheck0. Header có sẵn; CMake probe thiếu transitive includes do
assistant dùng header nội bộ Nav2. Không phải lỗi firmware hoặc Nav2 runtime.
12 raw/11 input/204 source hash khớp. V1/raw giữ nguyên, không ghi đè.

V2 riêng qa_c5_dynamic_component_ab_v2_20261021:11 inputs. Bỏ header/cast Nav2,
configure/activate và xác nhận state qua dịch vụ chuẩn ChangeState/GetState từ
client local. Manager C5/resource/factory/dynamic load/executor/destructor/4case
và guard giữ nguyên; không cần thêm smclib hoặc link component. Wait3s/RPC5s nằm
trong process cap40s. Runner ghi LAST_STAGE/STOP/error-tail/BLOCKED result nếu chưa
hoàn tất. V2 STATIC/SYNTAX ONLY; compile/tests/probe NOT_RUN, không lấy5PASS V1
làm PASS V2. Runtime204/fdea296/Pi59283c5/N8R2/15FPS/model/mission không sửa.

User một lệnh laptop V2, không cần Pi/AP/USB/camera/goal/actuator; chỉ probe build
1job +4fixedcases, thường1–3min/cap~9min. GửiLOG_QA/verdict, review trước patch;
không chạy V1 cũ hoặc loop. Chưa commit/push. C5crash và R1telemetry còn mở.

**NEXT_STEP=USER_RUN_FIXED_V2_LOCAL_DYNAMIC_COMPONENT_PROBE**


## DYNAMIC-LIFETIME-FAULT-CONFIRMED-AND-C6-CANDIDATE-WRITTEN-20261021-32


## 21/10 00:59:05 — V2 tái hiện lifetime fault; C6 candidate đã viết

[Báo cáo bằng chứng + bản vá + lệnh QA](docs/codex/current/C6_BEHAVIOR_LIBRARY_LIFETIME_CANDIDATE_2026-10-21.md).
V2 run005905: buildOK,5unitchecksPASS; INACTIVE/ACTIVE control đềuSIGSEGV đúng
LifecycleNode destructor, cả2retainEXIT0. RC0/fourcasesđủ/khôngtimeout;34raw/11tool/
204source/binary/4installedlib hash vàtestedmanifestkhớp. RAX vtable thuộc vùng
Spin trước unload, saufatalkhôngđọcđược: INACTIVE0x7f95fdf0db88,ACTIVE0x7616508c0b88;
ELFvirtual0x3eb88/fileoffset0x3db88 (RELROLOADdelta0x1000). Casegiữlibmappedqua
managerdestroy rồiunmapsaulease. Đãxácnhậncơchếtronglocalreproducer,fullcontainer
002615phùhợpcùnginstruction/stack;chưafullruntimefixed hoặcWiFi/odomPASS.

Áp C6candidate5file navigation, parentfdea296/branchbaseline: newretainedmanager
đọcIDs/types từloadNodeOptions, dùngpluginlibregistry/mỗiresolvedlibrary1owner,
khônghardcodeSpin/Wait/path hoặc tạo behaviorinstancephụ. Retainerbasekhaitrước
C5managerbase đểmanager/nodewrappershủytrước, rồi mớiunloadlib. ChỉBehaviorServer,
khôngđổirequest/remap/params/timer/thread/loop/scheduling/join/signals. Dependency
pluginlib/interfaceheadersnav2_core+smclibimportedheadertarget, khônglinkserver/
pluginvàkhôngsửa/opt/ros.7C++testsmới parameter/default/type/error/dedup/parity.

C6STATIC_REVIEW_ONLY: chưa compile/test/regression/commit/push. Packet riêng
qa_c6_behavior_library_lifetime_candidate_20261021 pin206sources/expectedstatus.
Onecommanduser buildnav1job,4C5+7newC++ tests/launchinstalledparity,2realcomponent
INACTIVE/ACTIVE C6actualheader/hookkhôngexternallease, ELFdirectlinkcheck/hash/
cleanup. Khôngreruncontrolđãtáihiện. ROSlocalhost/domainhiệntạigiữ,khôngPi/AP/COM/
camera/goals/actuatorAPI;namespaceQA. FirmwareN8R2/Pi59283c5/camera15/model/mission/
QoS/watchdog/thresholds khôngđổi. UserLOG_QA→raw/diffreview→userduyệtcommit+push
→mớimộtfullstaticWiFi/pose/shutdowngate;R1vẫnchưađạt/R2–R4chưachạy.

**NEXT_STEP=USER_RUN_C6_LOCAL_BUILD_AND_REAL_COMPONENT_LIFETIME_QA**


## OLD-C5-PROBE-PREFLIGHT-BLOCKED-ON-C6-CANDIDATE-20261021-33


## 21/10 01:13:17 — chạy nhầm probe C5 V2 sau khi áp C6

Run `qa_dynamic_lifetime_ab_20261021_011317_Kj90SF` BLOCKED_PREFLIGHT/RC2 ngay
trong cùng giây, chưa build hoặc chạy A/B. Manifest C5 cũ khác đúng3file C6 đã
sửa: CMakeLists.txt/package.xml/component_container_isolated.cpp. Không phải C6
QA FAIL hoặc lỗi runtime mới. Không reset source, không sửa manifest C5 để ép qua.
7 raw/11 old tool hash khớp; candidate C6 hiện tại206 source/15tool hash và exact
expectedstatus vẫn khớp. C6 build/tests/regression vẫn NOT_RUN. Dòng raw cũ
runtime_patch_applied=false chỉ nói probe không áp patch, không phủ nhận source
candidate C6 đã tồn tại; giữ raw và giải thích bằng review dẫn xuất.

Lệnh hiện hành trên laptop:

```bash
bash '/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/.codex_qa_backups/qa_c6_behavior_library_lifetime_candidate_20261021/verify_offline.sh'
```

Đóng launch/RViz/probe cũ, không cần Pi/AP/COM. Gửi LOG_QA và
C6_LOCAL_BUILD_AND_LIFETIME_QA_OK hoặc STOP. Chưa commit/push hoặc chạy robot.

**NEXT_STEP=USER_RUN_C6_LOCAL_BUILD_AND_REAL_COMPONENT_LIFETIME_QA**


## C6V1-BUILD-DEPENDENCY-BLOCK-AND-V2-METADATA-ONLY-FIX-20261021-34


## 21/10 01:14:59 — C6V1 build chặn; V2 bỏ implementation header không cần

[Chi tiết lỗi và lệnh C6V2](docs/codex/current/C6_BUILD_HEADER_FIX_2026-10-21.md).
Run011459 RC2/BUILD_NAVIGATION: thiếu include rcl_lifecycle/rcl_lifecycle.h trong
chuỗi nav2_core/behavior.hpp. C++tests/parity/componentregression NOT_RUN.
17raw/15tool/206source hash khớp. Lỗi dependency do assistant thêm, không phải
firmware/mạng; header ROS có sẵn, không cần cài lại hay sửa /opt/ros.

V2packet qa_c6_behavior_library_lifetime_v2_candidate_20261021 giữ V1/raw. Sửa3file
CMake/package/header: forwarddeclare đúng nav2_core::Behavior vì chỉ dùng API
pluginlib registry/path/load/destructor, khôngcreateinstances/sizeof/typeid/delete
Behavior. Đã đọc pluginlib5.1.3 administrative methods/member declarations không
cần completeBehavior. Bỏ implementationheader/includeDirs và nav2_core/smclib build
deps; nav2_core runtime registry giữ. RegressionCMake tương ứng, khôngdirectlink
server/plugin. Logicretainer/baseorder/params/tests/mission/QoS/guard không đổi.

V2STATIC/SYNTAX ONLY: diff/AST/bash/XML/source206 kiểm; compile/tests/regression
NOT_RUN. Không dùng A/B005905 làm PASS candidate. User1lệnhV2 build1job+4C5/7newC++/
parity/2realcomponent; childlocalhost/domaingiữ, khôngPi/AP/USB/camera/goal/actuator.
GửiLOG_QA/QAmarker→rawreview→userduyệtcommitpush→fullstaticruntime, chưadeploy/flash.

**NEXT_STEP=USER_RUN_C6_V2_LOCAL_BUILD_AND_REAL_COMPONENT_QA**


## C6V2-INCOMPLETE-TYPE-FAIL-C6V3-STANDARD-DEPS-AND-STATUS-20261021-35


## 21/10 01:18:32 — C6V2 compile fail; trạng thái R1 và dữ liệu được chốt rõ

[Trạng thái thực tế, dữ liệu, R1–R4 và lệnh V3](docs/codex/current/DEVELOPMENT_STATUS_AND_REMAINING_GATES_2026-10-21.md).
Run011832 BUILD_NAVIGATION/RC2: incomplete Behavior vì pluginlib virtualisClassLoaded
instantiate RTTI. Forward-declaration assumption của assistant là sai; tests/
regressions chưa chạy.17raw/16tool/206source verified. V1/V2 compile failures là
lỗi build/code integration assistant, không3lỗirobotmới hoặc evidence mất dữ liệu.

C6V3 khôi phục complete header, dùng ament_target_dependencies nav2_core+smclib
chuẩn thay copy include dirs/incomplete type. Regression CMake cùngdependency;
logicownership/order/tests/params/mission không đổi. Link có thể thêm thư viện
Nav2, không tạo node/behavior/timer/thread phụ; không khẳng định overhead0. ELF
vẫn chặn directSpin/Wait/behaviorcore. V3STATIC/SYNTAX_ONLY; build/testNOT_RUN.

R1 đang xử lýshutdown+telemetry; dữ liệuđãnhậnnhưngchưaổnđịnh. NonGDBC5last20s:
odomgap579.938ms/TF443.329/scan1293.552;JPEG13.15FPS nhưnggap743.870ms vàtuổi1005ms.
8active/pose vàzerostreamcóproof, chưa nghiệmthu nonzero command/goal/IR/actuator.
USBquiettrongcửasổkhôngchứngminhổnlâudài. R2YOLO/R3goal-cancel/R4firebehavior
vẫnchưađạt;khôngcamkếtcònlạichỉ3lầntest. Userjittertolerancetracked, khôngnớiguard.

V3packet qa_c6_behavior_library_lifetime_v3_candidate_20261021 mộtlệnhlocalQA cùng
scope,khôngnhánhdiagnostic/hardwaremới. SaureviewQA/userduyệtcommitpushmớifull
staticWiFi/pose/shutdown. Pi59283c5/N8R2/15FPS/model/baseline giữ;chưadeploy/flash.

**NEXT_STEP=USER_RUN_C6_V3_STANDARD_DEPENDENCY_LOCAL_QA**


## C6V3-LOCAL-QA-PASS-REVIEWED-AND-POSTPOSE-LATENCY-CLARIFIED-20261021-36


## 21/10 01:26:19 — C6V3 localQA PASS, raw/source đã review

[Báo cáo QA và giải thích startup/postpose](docs/codex/current/C6_LOCAL_QA_RESULT_2026-10-21.md).
Run012619SKeNUQ01:26:19–01:26:45+07/RC0. Buildnav10.6s/colcon10.8s;4C5C++37ms+
7newC++31msPASS,CTest0.38s;59PythonPASS0.87s. ActualcandidateINACTIVE/ACTIVE đều
EXIT0,retained2libs/active/destroymarkers đủ,no timeout. Khôngexternallease hoặc
directlinkbehaviorcore/Spin/Wait. Processbefore/afterrỗng.43raw/16tool/206source/
2binaryhashkhớp;exacttestedmanifest/status/sourcebefore-after/currentkhớp.
Source/toolgiữnguyênsauPASS. FullNav2/WiFi/USB/YOLO/goals/actuatorNOT_ACCEPTED.

Userhỏi gapchỉstartup? Đínhrõ sốC5nonGDB20s đượcđosaupose/readiness:
Nav2loganchor23:59:32.408;measurement00:00:11.774–31.774,~39.37s saulaunchanchor.
Odomgap579.938/TF443.329/scan1293.552ms,JPEG13.15FPSgap743.870/age1005.322ms vẫn
xuấthiệntrongcửasổnày. Cuốilượt since-last odom~90/scan100/TF0.28ms đãhồiphục;
khôngngắtliêntục nhưngchưachứngminhchờstartupxonglàluônổn. Headeragekhôngđồng
nghĩaonewayWiFidelay. StartupDDSburstvàpostposespikesđượctáchphạmvi,khôngtrộnsố.

C6LOCAL_QA_PASS_REVIEWED; chưa commit/push/deploy/flash. Đãsoạnbody/allowlist6file
5navigation+history. Userduyệtcommitthìpushluôncùnghistory, rồi pinSHApacket mới
mộtfullstaticWiFi/pose/shutdowngate;khônglặpQA/probeC5. R1fullruntimepending;
R2YOLO/R3goalcancel/R4baselinefire chưađạt. Pi59283c5/N8R2/15FPS/model/missiongiữ.

**NEXT_STEP=C6_REVIEWED_COMMIT_PUSH_APPROVAL_THEN_ONE_STATIC_WIFI_RUNTIME**


## C6-STATIC-WIFI-TOOLS-PREPARED-120S-AND-CHILD-EXIT-PROOF-20261021-37

## 21/10 — C6 đã push; chuẩn bị một lượt R1 static có giới hạn

C6 commit/push hoàn tất: `4e2fe97c5c9dd92a22973c8a3e1f7f0d6e06fae5`,
parent fdea296, branch codex/laptop-baseline-recovery, remote SHA verified.
Local candidate QA012619SKeNUQ đạt build10.6s/11C++/59Python/2componentEXIT0;
không dùng localQA thay full10component/WiFi runtime. Source sau commit giữ nguyên.

Packet mới `.codex_qa_backups/qa_c6_static_wifi_tools_20261021` ở repo chính;
[flow/scope/lệnh hiện hành](docs/codex/current/C6_STATIC_WIFI_RUNTIME_2026-10-21.md).
Kế thừa tools C5 nonGDB có kiểm hash và parser shutdown đã sửa. Pin206source
laptop4e2fe97,199sourcePi59283c5, container binary từ C6build đãQA. Khôngpull/build
Pi, khôngflash. Raw cũ giữ nguyên. Tool/docs localGitignored, phải sao lưu riêng.

Một observer mỗi máy, không decodeJPEG/pcapbus. COM5s→PiI/O30s→Nav2laptop20s→RViz
pose operator; đợipose<=180s/ready5s→đo120s. Cùng lượt lưu20s đầu để soC4/C5,
không resetstats/latchedsafety ở20s; startup tách riêng, sample5s/host10s/ping1Hz.
Giữ threshold/QoS/watchdog/safety/staticroute/N8R2/15FPS/model/missionbaseline.
Logodom/TF/scan/JPEG/IMU, lifecycle, DDSendpoints, network/CPU, kernelUSB và UART.
Khôngbyte-levelRTPS, khôngnghiệmthuYOLO/goal/motion/pump trongR1.

Final gate kế thừa source/overlay/Pimanifest/latchedsafety/cleanup, yêu cầu120s
và20s cùngstart, captureUART/clock/host/kernel đầy đủ, actualcleancontainerchild
exit sauINT. LaunchRC0/processrỗng khôngthaycleanchildproof; childcrash/forced
TERM/KILL/missingproof chặn. Findings telemetry/shutdown riêng để khôngche lỗi.
C6 sửa lifetime shutdown; chưa có bằng chứng giảm jitter hoặc dữ liệu đã ổn định.

STATIC_DIFF/AST/BASH/SOURCE+BINARY_HASH_REVIEW_ONLY. **TOOLS_OFFLINE_QA_NOT_RUN;
C6_FULL_RUNTIME_NOT_RUN.** Không chạy test/build/ROS/SSH/serial bởi assistant.
User chạy một lệnh verify_tools.sh (fakefiles/time/UART/ROS/signals, <=60s), gửi
LOG_QA→reviewraw/hash→copy tools và một pairedWiFi/manualpose/120s/shutdown.
Thông thường4–6phút; deadlinePiI/O480s/observer420s, có thể8–9phút với pose/copy.
R1 vẫnpending; R2YOLO/R3goal-cancel/R4baselinefire theo sau kết quả thực tế.

**NEXT_STEP=USER_RUN_C6_STATIC_WIFI_TOOLS_OFFLINE_QA**


## C6-STATIC-WIFI-TOOLS-97PASS-REVIEWED-20261021-38

## 21/10 02:21:42 — QA công cụ C6 đạt, chờ một lượt thực tế qua WiFi

User chạy packet `qa_c6_static_wifi_tools_20261021`, run `20261021_022141`:
**97 passed in0.39s, RC0**. Assistant chỉ đọc và review; không chạy lại tests.
4file raw khớp SHA256.55input công cụ/manifest exacttested/OFFLINE_QA_OK khớp;
manifest b00fa7920f4b565f79c0d03562aa6ffac9e75eca59dac990426d7a101cf8bf4f.
206source laptop khớp working tree và commit4e2fe97; binary bf286522… đúng bản
C6 đãbuild. Pi199source vẫn pin59283c5; không kiểmPi live trong review offline.
Code/tools giữ nguyên sauQA, chỉ nối Markdownhistory; chưa commit mục37–38.

Đạt **OFFLINE_TOOLS_QA_PASS_REVIEWED**, không phải fullNav2/WiFi/robotPASS.
[Runbook ba khối lệnh, đều gõ laptop](docs/codex/current/C6_STATIC_WIFI_RUN_2026-10-21.md).
Copy packet+marker qua AP; A tựSSH Pi chạyI/O, B local chạyNav2+minimalRViz và
COMcapture. User tựInitialPose/map9_10, khônggoal/arm. Mộtlượt120s sau readiness,
20s đầu cùng lượt đểsoC4/C5, startup tách riêng. Khởi động theo từng bước để hạn
chế tải đột ngột; không thêmpcapbus hoặcJPEGdecode. Giữ camera15FPS vàESP N8R2.

Không cần pull/buildPi hoặc flashESP. Runtime C6 chưa chạy; chưa kết luận jitter
hoặc shutdownfull10component đãhết lỗi. Giữ motor/bơmOFF, SerialMonitor/ROS cũ
đóng, Ethernet hai máyDOWN, nativeESPUSB→Pi/COM→laptop. C6khôngđổi hànhvibaseline.
Thường4–6phút, cóthể8–9 với pose/copy; giữ AP tới khi hai terminal dừng/copyxong.
GửiLOG_LAPTOP/FINAL_VERDICT/FINAL_EXIT_RC. Rawreview→cập nhật metrics/status/
ledger/reportindex; R1pending,R2YOLO/R3goal-cancel/R4fire theo kết quả thực tế.

**NEXT_STEP=USER_RUN_ONE_C6_PAIRED_STATIC_WIFI_120S_AND_SHUTDOWN_CAPTURE**


## C6-STATIC-WIFI-120S-BLOCKED-TELEMETRY-AND-FORCED-SHUTDOWN-20261021-39

## 21/10 09:06:00 — C6 đã đo đủ120s; R1 chưa đạt

[Báo cáo, bảng so sánh và lệnh snapshot tiếp theo](docs/codex/current/C6_STATIC_WIFI_RESULT_2026-10-21.md).
Run qa_baseline_combined_laptop_20261021_090600, Pi copy folderqa_baseline_combined_20261012_145507
nhưng receipt/log thật là21/10. Laptop4e2fe97,Pi59283c5,N8R2,camera15FPS,map9_10.
104outer/43Pi/3UARTrawhash/55toolhash verified; exact tested toolsmanifest và206/199
source manifests khớp. Sourcebefore/after/Pi sourcecheck thànhcông, binaryC6pin khớp.
Assistant đọc log/phân tích dữ liệu; không chạy lại tests/build/ROS/SSH/serial.

Đo09:07:37.682464–09:09:37.684168+07,120.001962s; first20.000190s cùngstart.
Bắt đầu ~51.634s sau Nav2startmarker. 8lifecycleactive, operatorpose/safety đúng,
nonzerocmd0/forbiddenpublishers0 cảhai, cleanup sạch, RUNNER_RC0/COPY0.
FinalBLOCKED_READINESS_OR_TELEMETRY/RC2; không phải thiếupose hoặc bàiđobịcắt.

Toàn120s laptop: odom1130msg/gap912.720ms/age1090.641ms; TF2150/gap810.924/
age1187.062;scan586/gap1790.124/age1277.828;JPEG1634/13.616FPS/gap1142.397/
age1027.536ms. JPEG5gap>500ms. Cuối đo odom24ms/TF0.13/scan188ms đãphụchồi.
Sau20s vẫn có spikes đến+114s; không gọi chỉstartup. So20s đầuC6 với20sC5
lưu riêng, n=1/khácđiềukiện nên không quy nhân quả tăng/giảm doC6.

23wholebins≈115s tươngứng:Pi odomgapmax108.557/TF106.318/scan203.271ms,
PiJPEG có1gap502.085ms; laptop trễ rõ hơn. Ping cùngcửađo laptop→Pi mean76.20/
max1754ms; ngược mean55.64/max1431ms. Wholepingloss gồmstartup/cleanup1.333%/
6.912%, không phảiDDSloss. CPUbusy10s tổngmáy laptop2.53–3.58%,Pi30.80–32.66%:
chưa có bão hòaCPU kéo dài; không loại trừ transient/one-thread/kernel stall.
Ưu tiên nhánh WiFi/driver/receive laptop; chưa chốt radio/powersave/DDS rootcause.
Cảhai thiếuiw nên không có actualPS/MCS/stationretry. IdlePSA/Bcũ khôngcải thiệnrõ.

UART413lines/COMPLETE, USBactivity/IDLE→MANUAL/PumpOFF; khôngCDCstall/reset/panic
thêm trong log. InbandBOOT1lần firmware0343c56…/uptime581508ms, không phải reset
trongđo. Pi kernel khôngESPdisconnect; cameraUSBreset09:06:22.623 trướcmeasurement,
chưa xácnhận nguyênnhân. PiNTP=no, SSHclockbounds rộng; wall-minus-monotonic netdelta
tronghostsamples≈0.0015ms laptop/0.0158msPi, không có bằngchứng netclocksteptrongđo.
TênfolderOct12 khôngthểdùng suyra đồnghồ đanglệch9ngày trongmeasurement. Headerage
khôngonewayWiFi/acquisitionage; mapTFfuturestamp không tựchứngminhclockfault.

Shutdown: retainedSpin/Wait2libs cólog. INT09:09:38.315;cleanup qua cácnode,
cuối bt_navigatorCleaningup09:09:41.864; runnerTERM09:09:48.351 sau10.04s,
NAV2_EXIT_RC143, khôngcleanchildexitline. KhôngSIGSEGV/abort quan sát trướcTERM,
nhưng không nghiệmthu C6fullteardown hoặc khẳngđịnh crashđãfix. Chưa stackđiểmkẹt;
khôngtự tăngtimeout/bỏBTplugin/đổiwatchdog đểépPASS. RàheadersHumbleon_cleanup
chuỗiNavigator/BtActionServer; chưa xácđịnhdòngreset nào chậm/kẹt.

Bước tiếp: user lấy snapshot WiFi hiện tại bằnginspect_wifi.sh đãQA (khôngROS/
scan/reconnect/mutation), copyPi snapshot vềlaptop. Assistant tiếp tục rà BTcleanup
offline; cần bằngchứngđịnhvịtrướcpatch. Không lặp full120s hoặc chạyGDBtoànbài
chưachọnmụcđích. R1BLOCKED,R2YOLO/R3goal-cancel/R4fire chưa nghiệmthu.
Tools/code giữ nguyên; chỉ docs/history+derivedanalysis cậpnhật, chưacommit mục39.

**NEXT_STEP=READ_CURRENT_WIFI_CONFIGURATION_AND_REVIEW_BT_NAVIGATOR_CLEANUP_OFFLINE**


## PRACTICAL-ACCEPTANCE-AND-WIFI-REBOOT-SNAPSHOT-20261021-40

## 21/10 — user chấp nhận jitter vừa phải có phục hồi; chuyển trọng tâm sang chức năng

User báo đã tắt Pi qua đêm và mở lại buổi sáng. Không dùng việc restart bình thường
để kết luận ESP lỗi. User nhắc rõ: chấp nhận độ trễ vừa phải/ngắn nếu tự hồi phục;
không cần không-trễ, không muốn debug kéo dài để đạt sự hoàn hảo.

[Quyết định nghiệm thu thực dụng và phạm vi R2](docs/codex/current/PRACTICAL_ACCEPTANCE_UPDATE_2026-10-21.md).
Assistant đã để ngưỡng thống kê nghiêm ngặt chặn bước tiếp quá lâu. Chốt lại: các
ngưỡng đo cũ giữ để so lịch sử, không tự chặn chuẩn bị perception đứng yên chỉ vì
một max-gap vượt. Guard runtime/watchdog/E-stop/stale/arm/ownership không đổi.
User chưa phê duyệt số ms tối đa cho mọi hành vi; không tự đặt ngưỡng mới hoặc
coi gap1790ms đã được nghiệm thu cho xe đang chạy.

Snapshot laptop092911/Pi092914:2file mỗi máy/4hash khớp. Vẫn WiFi AP10.0.0.1↔
10.0.0.253, EthernetDOWN, bg/kênh11/2462MHz, driver/kernel như20/10. Laptop
profilePS0default/runtimebuson, PiPS2disable/busauto; iw thiếu cảhai. Không có thay
đổi cấu hình lớn được quan sát, chưa có actualradioPS/MCS/retry hoặc rootcause.
Không cài/đổi AP/channel/powersave hoặc lặp ping riêng để tiếp tục lúc này.

C6run090600 giữ FINAL_VERDICTBLOCKED/RC2 gốc. Đã có đủ120s,8active/pose/safety,
nhận telemetry/camera13.616FPS và hồi phục; thêm đánh giá
READY_TO_PREPARE_R2_STATIC_WITH_KNOWN_LIMITATIONS. Không ghi lại robotPASS vào raw.
Jitter và shutdown cầnSIGTERM ởbt_navigator vẫn là lỗi mở; C6fullgraceful chưa đạt.
Không cần R1 hoàn hảo mới chuẩn bị R2 đứng yên có giới hạn/owned-process cleanup.

R2: reuse observer/hash/metrics, source laptop4e2fe97/Pi59283c5/N8R2/15FPS,
Nav2+YOLOlaptop. Sourceperception.launch.py giữ modelbest_final/GPUvenv/threshold,
chỉdetections/annotated/alignmentpreview, không mission/actuatorpublisher. Viewer
nhẹ và user xác nhận SHOW/HIDE sau khi sắp cảnh; một cửa sổ dự kiến5phút,
đo YOLO+telemetry+CPU/WiFi+USB/UART cùng lượt. R2runner chưa viết/test/chạy.
Ghi functional/recovery/scenecoverage riêng, không test lặp tất cả build cũ.
Goal/cancel rồi căn/IR/phun theo baseline và phần hardware được cấp quyền sau.
Nhánh shutdown rà riêng, không dùng nó kéo dài vô hạn test đứng yên.

Assistant chỉ review/config/data và sửa MD; code/tools/model/runtimeguards giữ
nguyên. Không test/build/ROS/SSH/serial hoặc commit/push lần này. History40dirty.

**NEXT_STEP=PREPARE_ONE_R2_STATIONARY_YOLO_FUNCTIONAL_CAPTURE_USING_BASELINE_MODEL_AND_GUARDS**


## R2-STATIONARY-YOLO-VIEWER-TOOLS-PREPARED-20261021-41

## 21/10 — chuẩn bị một lượt R2 có viewer và nhãn cảnh do operator xác nhận

User: “ok chuẩn bị yolo”. Theo tiêu chí thực dụng mục40, giữ recoveredjitter làm
metrics/giới hạn, không ép mọi max-gap đạt mới kiểm chức năng đứng yên.
[Scope, lệnh QA và flow R2](docs/codex/current/R2_STATIONARY_YOLO_PREPARATION_2026-10-21.md).

Packet qa_r2_yolo_static_tools_20261021 mới,54input, manifest
e38b336f9c81316b02cdb0d7d375915fd4527bda233a1c9700bd5daca2a195f3.
Kế thừa C6testedbasis; không sửa packet/rawC6. Pin laptop4e2fe97/206source/binary
bf286522…, Pi59283c5/199source, N8R2/camera15FPS/map9_10/modelbest_final758c18bc…/
GPUvenv/configYOLOgiữ. Không sửa ROSsource/firmware/model/QoS/watchdog/stale/arm.

COM5s→PiI/O30s→Nav2laptop20s→RViz/operatorpose→YOLOcuối/warmup<=120s. Mộtwindow
300s, tail<=15s chỉ chứng minh phục hồi cuối; không gộp làm đẹp metrics mainwindow.
Viewer trên cùng observerlaptop, decode/display<=5FPS,16raw+1annotated,5snapshots
max; không node/inference/DDSsubscriptionphụ. Box/khungcăn là annotatedbaseline,
rawfallback cóWAIT/STALE rõ, windowchưa map không tựthoát; Q/Esc ghiincomplete.

User sắp cảnh trước, bấm1EMPTY/2SHOW/3HIDE/4SHOW/5HIDE, giữ15s mỗi cảnh; chưa bấm
UNLABELLED. Không yêu cầu thao tác đúng thời điểm countdown tự chuyển nhãn. Scene
không được gated theo detector đã thấy để tránh bias. Chỉ tính distinctfreshsource
sau floor, agehiệntại−20..300ms theo baselinealignment, bỏ2ssettle. SHOW>=3confirm;
EMPTY/HIDE>=3fresh tail5s và khôngconfirmởtail. Thiếucảnh/data ghiINCOMPLETE,
không tựcoimodelhỏng. Ảnh điệnthoại không nghiệmthuIR/bơm.

Ghi YOLOpayload/age/scenes, telemetry/CPU/GPU/WiFi/USB/UART nhưC6; kếtquảchức năng/
jitter/safety/shutdown riêng. R2FUNCTIONAL_PASS_WITH_KNOWN_LIMITATIONS cho scope
đứngyên, khôngđổirawC6BLOCKED hoặcgọiNav2TERM graceful. Nonzero/forbiddenowner,
thiếusafety/source/copy/hash/cleanup hoặcchưa hồi phục cuối khôngPASS.

PiI/O900s/observer900s/UART1000s giới hạn QAphiên, không đổiwatchdogrobot. Thường
7–10phút tổng, cóthể12–14phút với pose/warmup/copy. No-goal/no-arm/motorpumpOFF.

STATIC_DIFF/AST/BASH/PINNED_SOURCE_BINARY_REVIEW_ONLY. **TOOLS_QA_NOT_RUN;
R2_RUNTIME_NOT_RUN.** Tests fakeROS/OpenCV/clock/UART/file không đụngGPU/GUIthật.
Assistant không chạy tests/build/ROS/SSH/serial. User một lệnhverify_tools.sh<=60s,
gửiLOG_QA→reviewraw/hash→copyvàpairedruntime. Chưa commit/pushhistories mới.

**NEXT_STEP=USER_RUN_R2_STATIONARY_YOLO_TOOLS_OFFLINE_QA**


## R2-TOOLS-58PASS-REVIEWED-AND-VIEWER-SCENE-INSTRUCTIONS-20261021-42

## 21/10 10:05:22 — QA công cụR2 đạt, sẵn sàng một lượt pairedYOLO đứng yên

Run qa_r2_yolo_static_tools_20261021/offline_runs/20261021_100522:
**58passed/0.20s,RC0**.4rawhash/54toolhash, exacttestedmanifest/QAmarker khớp.
206source laptop khớp working tree/commit4e2fe97, binaryC6 vàmodelbest_final758c18bc…
đúngpin. Pi199manifest giữ59283c5; không livePi trong reviewoffline. Chỉhistorydirty;
code/tool/model khôngđổi sauQA. Assistant chỉđọc/hash, khôngruntests/build/ROS/GUI/serial.
[Runbook copy/A/B và giải thích thao tác](docs/codex/current/R2_STATIONARY_YOLO_RUN_2026-10-21.md).

User hỏi viewer5FPS và các phím1–5. Ghi rõ: viewer làcửa sổ xemảnh cậpnhật<=5/s,
camera vẫnconfig15FPS, tham sốYOLO khôngbịép5. Nhịp thực tếcamera/inference đo
riêng; khônghứa15inferences/s. Annotated/khungcăn do baselineYOLO vẽ, khôngđổi.
User sắp cảnh xong, click cửa sổảnh (khôngterminal), bấmphím đượcnhắc:1EMPTY,
2SHOW1,3HIDE1,4SHOW2,5HIDE2,giữ15s mỗi cảnh. Thời gian chưa bấmUNLABELLED.
Sau5cảnh giữkhônglửa chờđủ300s; khôngQ/Esc/đóngwindowvìsẽdừngsớmINCOMPLETE.
Mẫuảnh/videođiệnthoại, khôngcầnlửathật/IR. WAIT/STALE thìđợiframemới.

OFFLINE_R2_TOOLS_QA_PASS_REVIEWED, **R2_RUNTIME_NOT_RUN**. Không lấy58fake tests
thaynhận diệnthật. Flow COM5s→PiI/O30s→Nav2laptop20s→pose→YOLOwarmupcuối<=120s→
đochính300s/recoverytail<=15s. Tổngthường7–10phút,cóthể12–14vớiwait/copy.
NativeUSB→Pi/COM→laptop,WiFiAP/LANDOWN,motorpumpOFF/no-goal/no-arm. Khôngbuild/
pull/flashlại. Runtime guards/model/15FPS/N8R2/map9_10/Baselinegiữ.

GửiLOG_LAPTOP/FINAL_VERDICT/FINAL_EXIT_RC. Review scene evidence/raw/hash rồi
cập nhậtfunctional/jitter/recovery/shutdown riêng; C6TERMvẫnlỗimở,khôngfullNav2PASS.
Khôngtựlặptoànbài hoặcđổingưỡngkhioperatorlàmthiếucảnh. Goal/cancel/IR/phun chưa
nghiệmthu. Trackedhistory42 nối, chưacommit; sourceC6đãpush,tool/docs/rawlocalignored.

**NEXT_STEP=USER_RUN_ONE_R2_PAIRED_STATIONARY_YOLO_CAPTURE_WITH_VIEWER**


## R2-RUNTIME-FUNCTIONAL-PASS-WITH-LIMITATIONS-20261021-43

Run101514: **R2_FUNCTIONAL_PASS_WITH_KNOWN_LIMITATIONS, RC0**, main300.007753s
10:18:09.362700–10:23:09.369558+07. Laptop4e2fe97/Pi59283c5/N8R2/model758c18bc…/
camera15FPS/map9_10 giữ.119outer/43Pi/3UART/54toolhash/exacttestedmanifest/
206source laptop/199Pimanifest/binary xác minh; raw và inputs đã test giữ nguyên.

Đếm độc lập5cảnh: EMPTY125/0confirmed; SHOW1129/129; HIDE1160/0; SHOW2160/148;
HIDE2121/0. Ba cảnh khônglửa tailconfirmed0;5JPEG gốc phù hợp nhãn đã mở. Không
lấy số này thành recallbenchmark.3325ok/20camera_stale trong3345resultsmain;
inference mean16.271/max24.11ms,processing mean17.400ms (không endtoend).
JPEG11.073FPS/gap3407.243ms,odomgap1174.817/TF1160.021/scan1874.074ms; cuối đo
current lại. PiTF618.607/JPEG1683.619ms nên không quy mọi gap choDDS/WiFi.

Safetyzero/8active/pose đạt, không observed lifecycleexit. KernelPi noentries,
UART931 COMPLETE: watchdog10:15:17.784→validRX10:15:34.420 hồi MANUAL trước main;
không gọi BOOT/reset/USBfault trong main.3335libjpegwarnings nhưng60stats
maxdecode_errors0/inference_errors0, nguyên nhân chưa rõ. YOLOEXIT0; Nav2INT10s→
TERM/143, dòng cuối smoother_serverCleaningup; cleanup hai máy sạch. Jitter và
shutdown vẫn knownlimits, C6strictBLOCKED không đổi; chưa goal/motor/IR/phunPASS.

User yêu cầu viewer mượt hơn trêni5gen12/RTX3050 6GB. Soạn candidate riêng cap5→15,
AST/diff một dòng; GUI_NOT_RUN/nextpacketpending. Annotatedbaselinecap10, không
sửa model/threshold hoặc hứa15FPS box. Không lặp R2 vì displaycap. Docs có result,
JSONmetrics, sceneproof, kếhoạchR3 và checkpoint. Chưa chạy tests/ROS/GUI/SSH/
serial bởi assistant; chưa commit/push lịch sử mới, runtime source không đổi.

**NEXT_STEP=PREPARE_R3_GOAL_CANCEL_PLAN_AND_USE_VIEWER15_NEXT_SESSION_NO_R2_REPEAT**


## R3-NORMAL-GOAL-REARM-CANDIDATE-PREPARED-NOT-APPLIED-20261021-44

User hỏi Nav2forcedshutdown: R2 đo active, không observed crash giữa bài; khi kếtthúc
SIGINT10s chưaexit thìTERM/143. Teardownlimit riêng chưa sửa/đạt bởiR3.

Rà baseline4e2fe97 LiveNavStopCore thấy normalgoalSUCCEEDED/operatorCANCELED không
đủ proof fullfirecycle nên releasedgateway không nhậngoal mới. Đây là findingtừ
source, không suy từR2 khônggoal. Soạn candidate mộtgateway trong packet riêng
qa_r3_goal_rearm_candidate_20261021: terminal4/5 exactUUID khi inputvalid→zero/
stationaryrelease cũ→1s liên tụcIDLE/nofire/off/zero→UUID mới với toànadmission cũ.
ProofIDLE hết hạn bằngmissiontimeout0.35; xửlý callbackreordering trong đúng hạn,
không phátmotion trong lúcđợi. ABORTED/fault/fire/missingarm/stale/ownerconflict
không được xóa latch bằng normalterminal. Firecycleproof cũ giữ, khôngframeworkM3.

Thêm4scalar và2stateJSONfields; không node/thread/timer/inference mới. Không đổi
model/15FPS/N8R2/topic/QoS/firmware/guard. Chưa áp worktree/install/build/Pi; viewer15
candidate chưaGUI/integrate. Scriptdemo cũ không navigation-only, chưa dùng R3 thật.
7toolinputhash/206baselinesource pin; verify tạo overlay package tronglog, kiểm
candidateimportpath rồi gateway/cancel/diagnosticsregression+newcases deadline60s.
AST/bashsyntax/diff chỉ; **OFFLINE_QA_NOT_RUN; RUNTIME_NOT_RUN**. Assistant không
chạytests/build/ROS/SSH/serial/GUI. User một lệnhverify_offline.sh→LOG_QA→review
trướcapply/commitpush/buildlaptop. Chưa cấp quyền motor/goal/arm hoặc nghiệmthuR3.
Docs có finding/candidate/nextstep; rawR2 vàtested54toolinputs giữ nguyên.

**NEXT_STEP=USER_RUN_ONE_R3_NORMAL_GOAL_REARM_OFFLINE_QA**


## R3-NORMAL-GOAL-REARM-111PASS-REVIEWED-APPLIED-UNCOMMITTED-20261021-45

Userrun qa_r3_rearm_20261021_105614_jFrbag: **111passed/0.26s**, EXIT_RC0,
LAST_STAGEOFFLINE_COMPLETED,10:56:14–15+07.7rawloghash/7toolinputs/exacttestedmanifest/
206sourcebeforeafter, candidateimportpath và27overlaypackagefiles đãxác minh.
Reviewdiffkhông finding mới buộc đổi candidate. Không chạy lạiQA bởiassistant.

Đãáp đúng gatewaySHA11cf5424e7b7ea55741cccee9022ca018225064626d748f4a996eb638117148b
và test_normal_goal_rearm.py đúngbyteđãtest vào baselineworktree.205file nguồnkhác
khôngđổi; base206/toolmanifest/raw giữ; manifestpostapply207artifactriêng. Cóbackup
modulecũ tạiqa_r3_goal_rearm_candidate_20261021/before_apply_20261021_105614.
Core terminalexactUUID4/5→zero/stationaryrelease→1scontinuousIDLE/off/nofire/zero→
newUUID với admissioncũ. Proofhết hạn0.35s, xửlýcallbackreordering tronghạn. Mấtarm/
stale/owner/actuator/fire/faultlatch/ABORTED không tựclear, firecycleproof cũgiữ.
4scalar/timer20Hzcũ, không node/thread/inference/topic/QoS/guard/FW/model/FPS mới.

gitdiff--check sạch; trackedruntime/testapplied+historydirty, HEAD vẫn4e2fe97.
Chưa commit/pushmới/build/ROS/goal/motor/arm; primarydirty giữ. Pi59283c5/N8R2 và
camera15FPS/model giữ; patchlaptop không cầnPi pull. Viewer15candidate riêng chưa
GUI/integrate. R2functionalPASSknownjitter/TERM khôngđổi.111tests khôngrobotR3PASS,
khôngsửaNav2shutdown hoặcABORTEDrecovery. Docsresult/checkpoint/commitbody đầyđủ.

**NEXT_STEP=USER_AUTHORIZE_COMMIT_PUSH_THEN_LAPTOP_BUILD_AND_PREPARE_R3_RUNTIME**


## R3-COMMIT-PUSH-AUTHORIZED-AND-REVIEWED-20261021-46

User: "ok push đi". Commit này đóng gói gatewaynormalgoal vàtest đúngcandidate
111PASS/0.26s (run105614_jFrbag), kèm các journal37–45 đãreview nhưng chưacommit.
Base4e2fe97, branchcodex/laptop-baseline-recovery.207sourcepostapplyhash vàexact
candidate/test khớp trướccommit; gitdiff--check sạch. Chỉ3file: gateway,testnormalgoal,
RUNTIME_VALIDATION_HISTORY; primaryworktree vàPi khôngđổi. Body ghi scope,QA,limits.

SUCCEEDED/CANCELED exactUUID→zero/stationaryrelease→continuousIDLE/off/nofire1s→
newUUID cóadmissioncũ. ABORTED/fault/fire/arm/stale/ownership khôngtựclear. Giữ
camera15FPS/N8R2/model/QoS/guard; no node/thread/worker mới. BUILD_NOT_RUN và
R3_HARDWARE_RUNTIME_NOT_RUN. Nav2TERM khi shutdown vàWiFijitter củaR2 cònknownlimits;
không lấy111tests thành acceptancechuyểnđộng. Viewer15candidate chưaintegrate.

SHAcommit và kếtquả push thực tế ghi ở docs/codex/current/R3_COMMIT_PUSH_2026-10-21.md
(localGitignored; đốichiếu gitlog/remote). Không ghi SHA tự thamchiếu vào commit.
Saupush chuẩn bị mộtlệnh build/importlaptop cho user, rồi boundednavigation-only
runtimevớioperator; patchlaptop không cầnPi pull/build hoặcflash. KhônglặpQAđãđạt.

**NEXT_STEP=USER_LAPTOP_BUILD_IMPORT_THEN_PREPARE_BOUNDED_R3_NAVIGATION_RUNTIME**
