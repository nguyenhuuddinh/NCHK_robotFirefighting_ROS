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


## R3-LAPTOP-MISSION-BUILD-PASS-REVIEWED-20261021-47

Userrunqa_r3_mission_build_laptop_20261021_111156_8rWrl0,11:11:56–59+07:
**R3_LAPTOP_MISSION_BUILD_IMPORT_OK1,RC0**,package0.64s/colcon0.92s.22rawhash,
5exacttestedtoolinputs và207sourcebeforeafter/current khớp. Installedamentprefix/
gatewayimport đúngbaseline, SHA11cf5424… đúng111PASScandidate,entrypointexecutable/
3resourceskhớp. Gitstatusbeforeafter sạch,processbeforeafter trống; noROSinit/coreexec.
Assistantchỉđọc/hash, khôngbuild/test/ROS/SSH/serial/GPU. Khônglặp111QA.

HEAD61bf6ea đãpush/remoteverified, Pi59283c5/N8R2/model/camera15/guards giữ. R3hardware
chưa chạy; runtime runner chưa soạn. Trialspec3UUID: G1SUCCEEDED→G2operatorCANCELED→
G3SUCCEEDED vớizero/stationaryrelease/normalclear giữa goal. Khôngảnhlửa/IR/phun.
R2static route/observer không dùngnguyênchoR3live; phảikiểm /cmd_vel_raw vàbounds/
owner thaystaticnonzeroforbidden. Demo cũ bật thêmalignment/IR/pump khôngphùhợp.

Đã hỏioperator quaasync: sàntrống+ngườitrựccôngtắc/ngắtnguồnmotor, kêbánh haychưa
sẵn sàng. Chưa cólựa chọn lúcghi; khôngsuybuild/staticpermission thành quyềnarm/motor.
Sànthật mớiđánhgiáarrival; kêbánh chỉcommand/cancel, khôngfakepose/odom/actuatorOFF.
AGENTS yêu cầu hardwareopt-in. Maintrial dựkiến<=300s; setup/outerdeadline phảiinrõ
trongrunner. Giữ fault/stale/ABORTED stop, không restartcore đểxóalatch. KnownR2jitter/
TERMshutdown vàviewer15candidate chưaruntime vẫnriêng. Docsresult/spec/checkpoint
đãcậpnhật; trackedhistory47nối, chưacommit. Không cầnPi pull/flash hoặcQA/buildlặp.

**NEXT_STEP=CONFIRM_OPERATOR_MOTION_SETUP_AND_PREPARE_BOUNDED_R3_NAVIGATION_RUNTIME**


## R3-FLOOR-OPERATOR-CONFIRMED-TOOLS-PREPARED-NOT-QA-20261021-48

User chọn sàntrống vàtrực côngtắc/ngắtnguồnmotor. Soạnpacket
qa_r3_navigation_tools_20261021,38inputspinned; source61bf6ea/Pi59283c5/N8R2/camera15/
model/map giữ. Laptop207/Pi199manifest+C6binary. Không hỏi lại cùngsetup.
R3core111PASS/buildimport đãđạt khônglặp; packet mới **OFFLINE_QA_NOT_RUN**,
HARDWARE_RUNTIME_NOT_RUN. Userverify_tools.sh<=60s: purepolicy/fakeclock/cancel/
OpenCVmock/filefixture+boundarychecks; khôngROSinit/GPU/GUI/USB/SSH/hardware.

R3main<=300s từoperatorA,3UUIDG1SUCCESS→V/stopclear→G2active→Cexactcancel→CANCELED/
stopclear→G3SUCCESS→V/stopproof. Stagegoal90s,driver900s gồmsetup,cleanup/copyriêng.
UsertựRVizgoal/InitialPose, khôngfakepose/odom/actuatorOFF. Chỉsupervisor/gateway;
khôngalignment/IR/pump, khôngdemoold. ToolchỉBoolarm/exactCancelGoal, khôngTwist/
ActionSendGoal. Startup readiness3s rồiA; noautorearmkhierror. Arm/cancel/stop/proof
UUID vàterminal/motion/odomdisplacement>=.10m/physicalarrivalV phảiđủ trướcPASS.

ĐổiR2staticrule trong tool sanglive /cmd_vel_raw/boundedTwist0.10/.8/đúngowner;
guardrobot khôngđổi. Driver cleanupFalsearm trongcontextcònhợplệ, trướcmission/
Nav2/Pi; ownedgroupkillonly. GiữdisarmedngắnsauDONE đểtránh expectedexit/healthrace.
Viewer15 cacheảnhmới, khôngdecode/inferencephụ; annotatedbaselinecap10. Metrics/
clock/CPU/GPU/WiFi/USB/UART dùnghelperR2 hashverified, scope/startup/main/shutdownriêng.
Childcrash khôngẩn bởilaunchRC0; TERMknownlimit, chưafixNav2teardown/faultrecovery.

AssistantchỉAST/bashsyntax/hash/diff/datareview, khôngtests/ROS/hardware. Docsrunbook
vàcheckpoints cậpnhật; baentrypointcontext đượcviếtgọn, bảncũgiữ tại
qa_r3_context_before_floor_tools_20261021. Trackedhistory48nối chưacommit; primarydirty
giữ. GửiLOG_QA→reviewraw/exactmanifest→mộtfloortrialđãopt-in, khôngPi pull/flash.

**NEXT_STEP=USER_RUN_ONE_R3_NAVIGATION_TOOLS_OFFLINE_QA**


## R3-FLOOR-TOOLS-OFFLINE-QA-PASS-REVIEWED-20261021-49

User chạy qa_r3_tools_20261021_114930_43q5vL, 11:49:30–31+07:00:
**64 passed / 0.35s, RC=0, LAST_STAGE=COMPLETE**. Assistant đọc và kiểm hash:
5 file raw, 38 input đúng bản đã test, marker OFFLINE_QA_OK khớp SHA256 manifest
6129ac10d400441b32b7df7f7c7be0876ffce0709888c3bb7d93e5d664a3dd5b;
207 source laptop hiện tại, binary C6 và model best_final đều khớp.
HEAD laptop61bf6ea, Pi59283c5. Không sửa 38 input đã test; preparation.json/README
trong packet giữ trạng thái lịch sử lúc chuẩn bị, kết quả mới ghi riêng tại
`offline_gate_review_20261021_114930.json` và hồ sơ này.

Đây là QA policy/cancel/GUI giả lập và parser chứng cứ; **R3_HARDWARE_RUNTIME_NOT_RUN**.
Không coi 64 test là xác nhận xe chạy sàn. Build/import đã đạt, không cần build/pull
Pi, nạp firmware hoặc lặp test. Setup sàn trống và operator trực công tắc/ngắt
nguồn motor đã được user xác nhận. Runbook hiện hành cung cấp copy + terminal A
SSH Pi + terminal B laptop, giữ WiFi AP tới khi cleanup/copy log xong.

Một lượt R3: tự đặt Initial Pose, chờ READY, A arm trong camera; G1 tới đích/V/stop,
G2 đang chạy/C hủy đúng UUID/stop, G3 tới đích/V/stop. Goal gần 0.3–0.5m; chỉ đặt
sau R3_ARMED và WAIT_Gn tương ứng. Main <=300s từ A, stage <=90s, driver <=900s
bao gồm setup; cleanup/copy riêng. Camera không mẫu lửa; không alignment/IR/pump.
Q/Esc rút arm; bất thường dùng công tắc motor. Không tự clear fault/ABORTED/stale.

Nav2/YOLO laptop, Pi I/O/rawSerialV2/safety/camera15FPS, N8R2, model/map/guards giữ.
Thu log odom/TF/scan/JPEG, CPU/GPU/WiFi/DDS graph, kernelUSB và COM/UART, action UUID/
cancel/arrival/stop. Jitter và TERM shutdown còn là known limitations, không kết
luận đã fix. Assistant không chạy test/ROS/GUI/hardware. Tracked history49 chờ
commit lần sau; không có commit/push mới trong bước review này.

**NEXT_STEP=USER_RUN_ONE_R3_FLOOR_TRIAL**


## R3-FLOOR-PREFLIGHT-BLOCKED-NO-MOTION-20261021-50

Userrun qa_baseline_combined_laptop_20261021_220912: **RUNNER_RC=1,
LAST_STAGE=PREFLIGHT, R3_BLOCKED_OR_INCOMPLETE**. 18 raw file SHA256 khớp;
38 input packet QA64PASS vẫn nguyên bản. Lỗi đầu 22:09:12+07 tại run_laptop.sh:125:
kiểm gộp kiến trúc x86_64, DISPLAY có giá trị và cổng COM/debug by-id tồn tại.
Lượt kết thúc 22:09:13. Chưa gọi preflight laptop, chưa mở UART, Nav2/YOLO/mission
hoặc arm; **floor trial NOT_STARTED**, không coi đây là lỗi robot đang chạy.

User nghi quên cắm debug. Tại lúc review đường dẫn by-id
usb-1a86_USB_Single_Serial_5972047168-if00 không tồn tại. Thiếu dây debug phù hợp
với lỗi, nhưng log gộp chưa chứng minh chỉ một điều kiện hỏng; cần user kiểm DISPLAY
trong terminal local. Không dùng DISPLAY của shell tool để suy terminal user.
UART_CAPTURE_INCOMPLETE là thiếu capture chưa mở, không bằng chứng ESP reset.
STOP invalid host role invalid là lỗi kiểm source trong cleanup trước khi source_role
được khởi tạo, không kết luận source sai HEAD. Không sửa firmware hay guard.

Laptop cleanup trống. Pi log220811 không được copy do runner chưa handshake/release;
Pi cleanup chưa review, không nói toàn hệ thống đã sạch. Giữ terminal A cho script
Pi tự hết WAIT_LAPTOP <=180s và cleanup; chờ prompt trước khi bắt đầu A mới.
Cắm COM/debug ESP vào laptop, giữ nativeUSB→Pi, đóng SerialMonitor, kiểm by-id và
DISPLAY trên terminal desktop laptop. Sau đó chạy A mới và B với LOG_PI mới theo
runbook; không dùng log220811 đã hết cửa sổ chờ. Không build/QA/flash lại.

Kết quả QA64PASS vẫn đúng scope offline, R3 hardware chưa nghiệm thu. Không thay
38 input đã test trong bước phân tích này. Báo cáo JSON runtime_preflight_review_
20261021_220912.json giữ lỗi đầu/giới hạn kết luận. Assistant chỉ đọc/hash/stat,
không mở serial/ROS/GUI/SSH hay chạy tests/hardware.

**NEXT_STEP=CONNECT_COM_DEBUG_VERIFY_LOCAL_GUI_THEN_START_NEW_A_B_SESSION**


## R3-NO-READY-ROOT-CAUSE-V2-CANDIDATE-20261021-51

Run qa_baseline_combined_laptop_20261021_221702: 22:17:03–22:22:28+07,
RUNNER_RC0 nhưng CONTROL_EXIT_RC2 / R3_BLOCKED_OR_INCOMPLETE. 118 outer raw hashes
(bao gồm nested), 43 Pi và 3 UART khớp. User không thấy READY nên không đặt goal:
đúng quy trình; WAIT_ARM suốt, armed=false, measurement0s, completed_goals=[],
không có R3_ARMED. Pose/readiness Nav2 đã đạt trước YOLO/mission. Không đổ lỗi thao tác.

Hai blocker xác định trong công cụ: status_seen yêu cầu bản tin action trước goal
đầu, nhưng action_status0 trong lượt này; và yêu cầu full ready3s liên tục trong
khi actuator_off dùng freshness350ms, telemetry ACT264 mẫu median gap1000.095254ms
(max5486.146698ms). Các mẫu đều pumpOFF/servoIDLE/OFF; không thể lấy một cửa sổ
350ms mỗi giây để chứng minh3s liên tục. 64QA cũ không cover startup/cadence thật.

UART đã hoạt động COMPLETE645 dòng, không thiếu debug. Đã đọc raw: 2 TXstalls,
available0/offset0/len64 rồi63, task_gap20ms; soft reconnect và watchdog1000ms
MANUAL→EMERGENCY→MANUAL. Pi kernel ESP disconnect22:19:39.123 và22:21:56.067,
re-enumerate/recover; ESP uptime16368→154089→291248ms tăng, không kết luận các
BOOT identity lặp là MCU reset. BootPOWERON ban đầu22:17:06 giữ riêng. Không suy
cáp hay USB chính là nguyên nhân của 2 lỗi READY trong tool.

Sau timeout/control cleanup, Nav2 nhận SIGINT22:22:08.753, rồi mất bond/teardown,
containercrash-6 lúc22:22:17.529 (MagickSIGSEGV); parentlaunchRC0 không che crash.
Đây là lỗi shutdown còn mở, không chứng minh Nav2 chết trướcREADY. Laptop/Pi
cleanup trống, sourceafterchecks0, UART/model/source đúng; chưa motion acceptance.

Chuẩn bị packet V2 riêng qa_r3_navigation_v2_tools_20261021, không sửa packet V1.
Không bắt status sample trước goal khi unique server/cancelservice đã sẵn sàng,
mission/gateway không active, graph/odom/zero/health đúng. Gateway robot vẫn cần
status/UUID thật trước output. Standby3s cho topology/health/idle và OFF history
<=1.5s; history KHÔNG cho arm. A chỉ tạo intent<=2s, grant duy nhất khi có OFF mới,
tổng receive_age_ms + tuổi receipt<=350ms, mọi precondition vẫn đạt. ON/stale/mất
input/goalactive/Q/clock lỗi hủy intent; expiry cầnA mới, không tựarm.

Thêm pure/fake-clock regression sparse1Hz, no initialstatus, inputloss/ON/stale,
requestexpiry/repeatedkey/clockjump, real12ACT-event timingfixture; adapter không
ROSinit. Sửa preflight công cụ để báo riêng DISPLAY/COM/arch. Assistant chỉhash/
AST/bashsyntax/source/datareview; **V2_OFFLINE_QA_NOT_RUN**. Source61bf6ea/Pi59283c5/
N8R2/FW/model/camera15FPS/watchdogs/robotfreshness không đổi. Chưa commit/push.

Cùng mismatch ACTcadence tồn tại trong gateway normal_cycle_rearm: freshOFF350ms
liên tục1s. Phải xử lý đúng hợp đồng telemetry/proof trước full3goal; chưa cho
runtime V2. Không fakeOFF hoặc lặp bản tin cũ để tươi giả, không nới watchdog.
Bước hiện tại: user một lệnh verify_tools.sh V2, deadline60s; reviewQA rồi chọn
fix cadence tại nguồn hoặc proof tương thích nhưng vẫn fresh khi admission.
Không chạy lại sàn ngay, không QA/build lại phần đã đạt nếu không có thay đổi.

**NEXT_STEP=USER_RUN_ONE_V2_READINESS_OFFLINE_QA_THEN_REVIEW_ACT_CADENCE_CONTRACT**


## R3-V2-QA-PASS-ACT-SOURCE-PHASE-CANDIDATE-20261021-52

User QA run qa_r3_tools_20261021_223828_OrUZHw,22:38:28–29+07:
**94 passed / 0.37s, RC0, LAST_STAGE=COMPLETE**. 5 raw SHA256,42 tool inputs đúng
bản đã test, marker manifest và207 source laptop hiện tại khớp. V2 readiness QA
PASS; chưa V2 hardware, không lặp94test khi không sửa packet. Source61bf6ea giữ.

Rà nguồn ACT: firmware TaskSerialComm20ms, IRdivider8/10→tick2,12,22,32,...;
ACTdivider18/25→tick7,32,57,82,... Alternate ACT trùng IR, sau IRenqueue khiến
ACT fail điều kiện emptyring. Hợp với medianACT1s ởrun221702. STATE5ticks vàENV25
còn có thể cần2/3tick đểdrain. Chọn sửa lịch nguồn, giữ nguyên350ms freshness và
1s normalclear trong gateway; không tăng timeout hoặc lặp payload cũ giảtươi.

Candidate riêng qa_r3_act_cadence_candidate_20261021: TxScheduler thêm ACTdue ở
tick8,18,28,...5Hz; để đủ thời gian STATE2chunk+ENV1chunk drain, tránh IRtick2.
TaskSerialComm chỉ thay dividerACT thành schedulerflag; cùng ACTframeformat và
actuatorStatusSnapshot mới mỗi lần. Giữ STATE10Hz, ENV2Hz, IRphase/priority,
2slotqueue, ACTbest-effort FIFO>=64/emptyring/deadline150ms, CDCrecovery, allmotor/
pump watchdogs/pins/owners. Không tác vụ/heap/dependency mới; thêmcounter/flag nhỏ.
5Hz là lịch cấp slot, chưa phải xác nhận FPS nhận trên phần cứng hay fix USBstall.

Snapshot64 file firmware dirty hiện tại,79 input packet đượcpin,overlay4 file:
SerialComm.h/TaskSerialComm.cpp và native testmain/newTelemetryCadenceHarness.h.
Firmware gốc không sửa, FWELF gốc0343c56f… khớp identity ESPrun221702; không coi
ELFhash chứng minh toàn bộ trạng thái source đãbuild khi thiếu provenance cũ.
Ros207source/350msguard giữ. Không apply/commit/push/build/test/upload bởiassistant.

Một user-run command tạo shadowfirmware riêng, copycache đúngnative/N8R2(~4MB),
chạy48nativecũ+4targetedrate/phase/realqueue/FIFO/offline/wrap, buildN8R2singlejob,
compile traceC++actualscheduler+queue/fakeUSB cho legacy/candidate/blocked, replay
qua LiveNavStopCore không sửa và normalgoalregression. Expected model legacy10ACT/
10s vsnew50ACT/10s, STATE/IRkhônggiảm; source/receivecompletiontimes córecord.
Core phải chứng minhlegacy/stale/ON/lostarm/blockedUSB khôngrearm, newcadence đủ
fresh350ms/clear1s mới nhận UUIDmới; không gửiROSgoal/Twist/arm hoặc chạmGPIO.

**CANDIDATE_QA_BUILD_NOT_RUN; NOT_APPLIED; NOT_UPLOADED.** Tổngdeadline900s,
native120s/MCUbuild600s/coreQA60s; originalsourcehash/statusbeforeafter và rawlogs
đượcgiữ. AssistantchỉAST/bashsyntax/hash/diff/datareview, khôngthực thi test/hardware.
Sau LOG_QA phảireviewrawtraces/build/newELF rồi mớiapply/commit/upload và cậpnhật
runtimefirmwarepin; không dùng SHA0343 cũ cho firmwaremới. Nav2teardowncrash và
USBstalls vẫnopen, khôngcoifixcadence đồngnghĩakhắcphục2vấnđềđó.

**NEXT_STEP=USER_RUN_ONE_ISOLATED_ACT_CADENCE_QA_AND_N8R2_BUILD**


## R3-ACT-QA-BUILD-PASS-APPLIED-UPLOAD-PENDING-20261021-53

Userrun qa_r3_act_cadence_20261021_225330_LKHO7F,22:53:30–55+07: RC0/COMPLETE,
sourceafterFW0/ROS0. **Native52/52PASS0.511s; gateway54PASS0.23s; N8R2build
SUCCESS23.023s**. 22rawhash/79exacttoolinput/65shadowfiles/64FWoriginalfiles/
207ROScurrentfiles/2candidatebinaryhash verified. Originalstatusbeforeafter khớp,
overlay4file byte-identical với shadow đãbuild. Buildlog không có warning/error.
N8R2 RAM33564/327680bytes,flash387233/3342336bytes; không gọi đây là đoCPU.

Trace mô hình scheduler + TXqueue thực/fakeUSB:10s legacyACT10/candidateACT50,
maxgap1000→200ms. STATE100 cảhai,IR41 cảhai,healthyexpired0. BlockedUSB ACT0,
không fakefresh. Gateway không sửa chấp nhậnnewcadence sau normalclear1s,
350ms freshness cònnguyên; oldcadence/stale/ON/armfault/blockedUSB khôngrearm.
Đây là offline modeledtraffic, **actualACTcadenceNOT_MEASURED**, USBstall/teardown
chưa fix. Không lặp94V2QA hoặc52+54test sauapply byteidentical.

Đã backup3oldsource +oldELF/bin/bootloader/partitions vàstatus vào
qa_r3_act_cadence_candidate_20261021/before_apply_20261021_225330, manifest hashes.
Áp đúng4file đãtest vào /home/huudinh/Documents/esp_projects/robot_mainboard:
lib/app/TaskSerialComm.cpp,lib/service/SerialComm.h,test/test_liveness/test_main.cpp,
newtest/test_liveness/TelemetryCadenceHarness.h. Giữbyte củauser ngoài delta,
không resetdirtyworktree. Applied65buildinputs khớp shadow; ROS207source giữ.
**APPLIED_OFFLINE_PASS; UPLOAD_NOT_RUN; NEW_FIRMWARE_RUNTIME_NOT_RUN.**

ReviewedELF f6496e95f6657bf922f561ba748826ffd225e5a9f5671e8efaa64566072bf0ed;
BIN5e8e1c6f3ae544ceab8cb32516d3c0a326b7435919619f872f5812bc4931f43d.
Rootfirmware .pio artifact vẫn0343… cũ; upload dùng đúngshadowartifactrun225330,
không dùngIDEroot trước khi rebuild. Córelease_20261021_225330/upload_reviewed_
n8r2.sh --upload; pin65applied/65shadow/5binary inclboot files,5releaseinputs.
Cờ--upload rõràng, đúngby-idWCH COM5972047168, x86laptop, khôngoldROSsession.
User dừnglaunchPi/laptop,đóngmonitor,tắtnguồn motor/bơm trước nạp; không hỏi lại
setupfloor đãxácnhận. DùngPIO -t nobuild -t upload (đãđọc builder cài đặt),
max180s; loghash/receipt/expectedELF. AssistantkhôngchạyPIO/tests/serial/flash.

Chưa commit/push mới. Đãghi QA/apply/rollback/runbook, fwrootACT_CADENCE_VALIDATION_
2026-10-21.md. Sau userLOG_UPLOAD phảireviewuploadwritehash/postimagehash rồi
kiểmbootidentityELFf649… vàACTfreshness khiđứngyên trướcREADY/full3goal. Không
đánhdấu uploaded/runtimePASS từkếtquảcompile. Model/map/camera15FPS/ROSguards,
GPIO/pumpservoowner/watchdogs khôngđổi. Nav2crash-6saucleanup vàUSBTXstalls giữopen.

**NEXT_STEP=USER_UPLOAD_REVIEWED_N8R2_ARTIFACT_THEN_SEND_LOG_UPLOAD**


## R3-ACT-UPLOAD-VERIFIED-STATIC-CAPTURE-PREPARED-20261021-54

Userupload qa_r3_act_upload_20261021_230155_JG2nWD:23:01:55–23:02:00+07,
RC0/COMPLETE,BINARY_AFTER_RC0.11rawhash/5releaseinputs/5currentbinary/65applied
source verified; toolsused exact release manifest. PIO5.501s, N8R2 environment,
esptool chipESP32-S3rev0.2/PSRAM2MB,MAC28:37:2f:1f:73:c8 đúngESP đãdùng.
**App387600bytes@0x10000 write/hashverified, hardresetRTS.** Nobuild uploadlog
chỉghi ứngdụng, khôngnói bootloader/partition đượcghi lại; cácfilepin vẫnkhớp.
ExpectedELFf6496e95f6657bf922f561ba748826ffd225e5a9f5671e8efaa64566072bf0ed.
**UPLOAD_VERIFIED; RUNTIME_BOOT_IDENTITY_PENDING; ACT_PHYSICAL_RATE_NOT_MEASURED.**
Không đánhdấuREADY/3goal hoặcUSBfix từesptoolSUCCESS.

Chuẩn bịpacket qa_r3_act_static_tools_20261021,11pinnedinputs. Mộtlệnhlaptop
--static tựcopy đúngfilepin sangPi, nhậnCOMUART0, settle5s, SSHPiI/O+readonlyprobe.
PiROS59283c5/laptop61bf6ea sourcepins; sourceFW65khớp đãapply. 20swarmup+60sđo,
camera15FPS nhưngkhôngsubscriberJPEGquaWiFi, inputwatchdog unusedtopic
/fire_robot_static/cmd_vel_raw. KhôngNav2/YOLO/RViz/pose/arm/goal/pump command.
Probechỉ3subscriptionACT/odom/cmd_vel, khôngpublisher/client; ACTraw/sourceESPms/
receipttime/odomcmdcounts/graph+PiBOOTlog/kernelUSB vàlaptopUARTđượcgiữ.
Source/payloadfreshness khôngrewrite. Nonzero/invalidCMD,ACTON/servoactive hoặc
controlpublisher dừngcapture; motor/bơm tiếp tục tắt. GiữnativeESP→Pi,COM→laptop,
APWiFi/LANdown vàđóngoldlaunch/monitor nhưtrước.

Thường~2phút inclsetup/cleanup/copy; remoteSSHtimeout150s vàcopy45s riêng.
CAPTURE_RUNNER_COMPLETE chỉlàcollectiondone, **khôngphải physicalacceptance**.
ReviewrawELF/ACTmedian5Hz/gaps/fresh350ms/continuousOFF1s vàrecovery trướcR3.
Đây làPi-sideACTsourcecheck, khôngloadedWiFi/DDS/JPEG/Nav2benchmark. Không mởrộng
thêmsoak dài hoặc lặp94/52/54QA/build saubytekhôngđổi. Utilitymớiread-onlykhông
bắt thêmregressiongate; assistantAST/bashsyntax/hash/adapterboundaryreview, không
thực thiROS/tests/hardware. Reusecommon/check_sources/UARThelperđãreview, không
ngụy tạoOFFLINE_QAmarker hoặc gọipreflight live-floor củaR3.

Journal/status/fwrootMD cậpnhật; khôngcommit/push mới. Firmwarecontrols/guards/
GPIO/model/map giữ. Rootfirmware .pio artifact cũ0343… còn, release/appJSONstatus
lúcchuẩnbị làimmutablehistory; upload_review230155 làstatusmới. Nav2teardowncrash
và2USBTXstallsrun221702 cònopen. Bướctiếp usermộtstaticcapture, gửiLOG_LAPTOP;
chưaarm hay chạyfloortrial ngay.

**NEXT_STEP=USER_RUN_ONE_STATIC_ACT_BOOT_IDENTITY_CAPTURE_THEN_SEND_LOG_LAPTOP**


## R3-ACT-STATIC-CADENCE-PASS-RECOVERY-TAIL-INCOMPLETE-20261021-55

Run `qa_act_static_laptop_20261021_231804_GPK8Nm`,23:18:04–23:19:43+07.
Collection RC0; verified46outer/27Pi/3UART/11tool SHA256 entries (nested sets overlap).
Pi199 source pins before/after unchanged; laptop207 source check and FW65 verify
recorded successful. Both process-after lists empty. UART156lines,COMPLETE.
BOOT observed ELF `f6496e95f6657bf922f561ba748826ffd225e5a9f5671e8efaa64566072bf0ed`
confirms the reviewed N8R2 image is running, not merely uploaded.

| 60-second measured window | Result |
| --- | --- |
| ACT count / whole-window rate | 285 / 4.75Hz |
| ACT active receipt-span rate / median gap | 4.9999Hz / 200.000ms |
| ACT max received-pair gap (raw receipt timestamps) | 202.195ms |
| ESP source delta / repeated ESP timestamps | 199–201ms,median200 / 0 |
| Pi residence age at publication | median28.0,max36.7ms; not end-to-end acquisition age |
| Continuous fresh commanded-OFF interval under350ms residence+receipt guard | 57.116s |
| ACT tail silence at capture end | 3211.114ms; total residence+elapsed3245.514ms |
| Odom count / received-pair gap / tail silence | 570 / max105.692ms / 3060.901ms |
| CMD stream / nonzero or invalid observed | 600 / 0 |

All ACT payloads commanded pumpOFF/servoPWMoff/IDLE; forbidden control publishers0,
invalidACT0,omitted0. ACT means actuator-command telemetry, not a command to turn
on actuators and not physical pump/nozzle feedback. No Nav2/YOLO/RViz/arm/goal;
this is Pi-side ACT cadence, not loaded WiFi DDS/JPEG or floor acceptance.

**ACT_CADENCE_PASS; END_TELEMETRY_STALE; USB_RECOVERY_INCOMPLETE_AT_CAPTURE_END.**
Received-pair maxima exclude silence after the final message; reporting202ms alone
would conceal the3.211s end outage. Capture summary RC0 is completion, not PASS.
USB disconnect23:19:35.387→enumerated36.725; UART TXstall reason1,offset0,len65,
avail0,taskgap20/max21ms→soft reconnect→watchdog EMERGENCY→MANUAL. Pi reopened
23:19:37.439, while capture ended37.156:0.283s too early for returnedACT/odom.
TF correctly stopped on stale odom before USB detach. Cause of TXstall is still
unresolved; ordering does not establish cable/CPU/firmware root cause. UART reacquired
hostactivity is not evidence that Pi again published ACT/odom. Cleanup SIGINT has
some childexit-2; no claim every child exited0. Nav2 shutdown crash remains open
from the earlier floor attempt; Nav2 was absent here.

Prepared separate `qa_r3_act_recovery_tail_tools_20261021` (11pinnedinputs).
Original capture packet/raw unchanged. Same read-only3subs/0pubs/0clients, same
PiI/O/staticunusedinput and UART. Only logger changed:20swarmup+60smain, up to15s
extra if freshACT/odom have not held3s at the boundary, with end_ages and separate
recovery_tail counts.95s probe hard limit, remote150/copy45 bounds unchanged.
RC0 remains collection-only, exhausted tail explicit. No fault injection; if no
fault occurs, record NO_FAULT_OBSERVED rather than claim fault recovery proven.
AST/bash syntax/hash inspected, **new utility runtime NOT_RUN**, no new project
QA/build/flash. Existing94V2/52native/54gateway results not rerun. No robot/FW/
safety/GPIO/model/map/15FPScamera changes. No commit/push this turn.

NEXT_STEP=ONE_BOUNDED_STATIC_RECOVERY_TAIL_CAPTURE_BEFORE_FLOOR_NO_REBUILD_NO_REFLASH


## R3-ACT-STATIC-STABLE-V2-FLOOR-RELEASE-20261021-56

Run `qa_act_static_laptop_20261021_233009_YvBJWl`,23:30:09–23:31:50+07.
Verified46outer/27Pi/3UART/11tool SHA256 entries (nested sets overlap). Collection
RC0, Pi/laptop sourceafter0, current207laptop/65firmware source pins verified by
hash only; Pi199before/after verification successful. Both cleanup process lists
empty. UART182lines COMPLETE; BOOT ELFf6496e95f6657bf922f561ba748826ffd225e5a9f5671e8efaa64566072bf0ed.

| 60-second main window | Result |
| --- | --- |
| ACT | 300 / 5.00Hz; median receipt gap199.996ms,max205.014ms |
| ESP source delta | 199–201ms,median200ms; no duplicate timestamps |
| Pi residence at ACT publication | median12.2,max18.3ms; not end-to-end latency |
| Odom | 600 /10.00Hz; max received-pair gap104.273ms |
| CMD | 600 /10Hz, no nonzero/invalid observed |
| End freshness | ACT receipt20.716ms,totalresidence+elapsed37.716ms; odom68.201ms |
| Collection freshness continuity | 72.221s including warmup, fullmain covered |
| USB detach / UART TXstall during capture | 0 /0 |

All ACT command payloads pumpOFF/servoPWMoff/IDLE; forbidden controlpublishers0,
invalidACT0,omitted0. Ends DURATION_COMPLETE_WITH_FRESH_TAIL; no extension needed
beyond one cmd callback at nominal80s. **PASS_STATIC_ACT_CADENCE_AND_END_FRESHNESS;
NO_FAULT_OBSERVED.** No fault means recovery from prior TXstall is not demonstrated.
Do not compare maxgap205ms with the previous3.211s tail as the same metric.

UART logs POWERON at23:30:14, before Pi bridge open23:30:32; cause not established.
No additional boot/reset observed in main. Watchdog at23:31:45.931 follows launch
SIGINT and ended measurement, not a main-window outage. Some Pi children exit-2
on requested SIGINT; empty process lists do not mean universal exit0.
No Nav2/YOLO/arm/goal in this run. Not loaded WiFi/DDS/JPEG, physical pump/nozzle,
or floor acceptance. USBstall and Nav2teardowncrash remain open historical items.

Reviewed unchanged V2 tools42pinnedfiles, QAraw5 and OFFLINE_QA_OK manifest marker.
Prior94PASS0.37s retained; no repeatQA/build/flash. Readiness fixes and5HzACT now
have source-side evidence; release ONE user-operated bounded R3floor trial using
`qa_r3_navigation_v2_tools_20261021`. Its pinned README/QAmetadata remain historical
candidate snapshots; external current runbook is authoritative release record.
Nav2/YOLOlaptop61bf6ea, PiI/O59283c5, N8R2ELFf649, map9_10, camera15FPS unchanged.
Already approved emptyfloor/operator motor cutoff; no new confirmation needed.
A only after READY, goals only after R3_ARMED. G1SUCCESS+V→G2active+C exactcancel/
stop→G3SUCCESS+V. No fire/alignment/IR/pump, no tools generating goals or Twist.
Main<=300s fromA,stage90s; setup/pose/cleanup separate, typically7–12minutes total.
Q/Esc withdraw; abnormalmovement physicalmotorcutoff. Do not clear fault latches or
restart to forcePASS. Rawreview checks goalUUID/motion/stop/telemetry/USB/shutdown
separately. No commit/push this turn.

NEXT_STEP=ONE_V2_R3_BOUNDED_FLOOR_TRIAL_NO_REBUILD_OR_REFLASH


## R3-V2-STARTUP-AND-POST-ARM-ABORT-20261021-57

Run `qa_baseline_combined_laptop_20261021_234121`:120outer/43Pi/3UART raw hash
entries verified,42testedinputs/QAmarker unchanged. Sourceafter successful both
hosts, cleanup empty; UART444lines COMPLETE, Pi kernel noentries. N8R2 BOOTELFf649…
observed in Pi log. **BLOCKED_POST_ARM_PERCEPTION_SOURCE_STALE_NO_GOAL.**
A accepted23:45:08.973, stopped23:45:11.270 after2.299s,0goal intents/action goals/
completedgoals. At23:45:11.268 mission health becameBLOCKED/SOURCE_TIMESTAMP_STALE:
camera-source age411.6ms > existing300ms, detection receiveage60.8ms; odomfreshtrue,
odomage80.8ms. All8lifecycle active and safetyfresh in final sample. Trial.update
withdraws arm immediately on livehealth loss. User had insufficient time to setgoal;
no evidence of wrongA or main300s expiry. Mission camera-source age is distinct
from detection-message receiveage and odomgap; do not treat it as YOLOcompute411ms.

| Startup event (local +07) | Observation |
| --- | --- |
| Laptop start23:41:22 | Preflight/COM then staged startup |
| Deliberate QA waits | COM5s +PiI/O30s +Nav2wait20s =55s |
| Nav2 start23:42:05.133 | Localization active23:42:08.430,about3.3s |
| RViz pose stage23:42:25.463 | Operator initialpose23:43:10.474 |
| Navigation lifecycle active23:43:16.544 | About6.1s after pose |
| Pose/readiness stage completed23:43:24.328 | Includes fresh-data/stability checks |
| YOLO launch log created23:43:25.620 | Modelready23:44:50.880,about85.3s |
| First READY23:44:54.568 | MISSION_READY filemtime, matching untimestampedR3_READY stdout |
| Operator A23:45:08.973 | Runtime2.299s then perceptionhealthfault |

Source audit: perception.launch._validate_paths synchronously imports torch and
ultralytics in a separate Python process before launching both nodes. Real YOLO
process imports them again in _load_model, then model/CUDA initialization and
2warmup predictions. Approx41s prechild startup +44s realprocess initialization
inferred from sampler/procstartticks and firstalignmentlog; exact import/model/
CUDA breakdown NOT_MEASURED. Weights loaded by realprocess; do not call this
2modelcopies. No claim entire Nav2 startup is85s. READY means current standby
conditions hold3s (pose/lifecycle/safety/mission nofire/freshinputs/owners/zero/
ACT); it is not permanent data health. Prior READY state can be lost later.

Resource sample: aggregateCPU mean6.22%,max10.54%; MemAvailable min6.876GiB;
GPUutil6–47%,VRAMmax1017/6144MiB. No evidence of global memory/GPU exhaustion;
aggregate samples cannot exclude a hot thread or brief pause. YOLO steady5s
processingmeans17.5/18.3/18.5ms; firstwindowmax585.8ms includes first-usecost.
Camera noframe1.07s at23:44:57.880 then resumed; sourceage also exceeded300ms
beforearm. JPEGextraneous-marker warnings observed but decode_errors0; don't
infer badcable/USB/DDS solely from warning. Exactcamera-age origin not isolated.

Nav2 SIGINT23:45:16.471 follows aborted measurement; teardownagainSIGSEGV/exit-6,
parentlaunchRC0 is notchildsuccess. Separate open issue, not cause of stop2.3s
afterA. No actualgoal/navigationrun was measured; cannot claimR3functionalPASS.
No sourcechanges/tests/build/ROS/hardware this review; user requested explanation.

Next: offline optimize redundantYOLOstartup import and log perstage timing, then
trace camera timestamp-age path (Pi source→WiFi/DDS→YOLO→mission) before retrying
floor. Replace fixed waits only with proven readiness checks, retain sequencing
unless evidence supports change. Do not raise300ms guard or ignore livehealth
loss to forcePASS. Preserve original42testedV2files/rawlogs and all sourceguards.

NEXT_STEP=OFFLINE_OPTIMIZE_YOLO_STARTUP_AND_TRACE_CAMERA_SOURCE_AGE_BEFORE_ANOTHER_FLOOR_TRIAL


## YOLO-STARTUP-SINGLE-IMPORT-CANDIDATE-20261022-58

User authorized nextstep and asked fixedwaits vsreadiness and WiFiPi vsROS2.
5/30/20s are configured QAsettles (55s total). Pose/readiness59s andYOLO85s are
observed, not sleeps; READYrequires stableinputs3s. Timeouts are upperbounds,
not waits to be fully consumed. Keep staggeredstartup and existingsettles fornow.

Reverified run234121120outer/43Pi/3UART raw. Ping237tx/229rx,3.37553%loss,
RTT0.881/254.262/4303.901ms min/avg/max, substantial link/host jitter even forICMP.
Cannot call this inherent PiAP limit or pureWiFi/RF delay: endpoint scheduling,
driver/powersave/competition notisolated. OnPi JPEGmaxgap1456.538ms/maxheaderage
1386.643ms vs laptop2153.948/2282.837ms. Hostaggregatewindows differ229s/224s,
not perframematched; do not subtractmaxima as one-waynetworklatency. Piobserver
still includes localDDS/executor, not physicalcamera-only timing. Around23:44:58,
Pi already recordsJPEGgap991.933ms/age991.646ms andlaptopgap1012.803/age1349.735ms.
SSHclockboundsbefore[-48.170,+83.605]ms/after[-64.530,+88.963]ms prevent precise
one-wayallocation. Evidence supports BOTH source/local and addedcommunication
pathissues; notjustWiFi,notjustNav2heavy. NoUSBkernelentries inthatcapture.

Prepared isolated candidate `qa_r3_yolo_startup_candidate_20261022`,28pinnedinputs,
package-onlycopy,4changedcode/testfiles; baseline is NOT modified/APPLIED.
- launch/perception.launch.py keeps model/config/interpreter pathchecks, removes
  subprocess torch/ultralytics import; actualnode stillimports/checksdeps once.
- yolo_observer.py logs begin/done of dependencyimport,devicecheck,modelload,
  GPUwarmup,overallobserverinit. Originalmodel/CUDAfail behavior retained.
- Samefile bounded3keystats callback-entrysourceage,post-inferencesourceage and
  totalcallback(inclannotation), count/sum/min/max,logeveryexisting5s. Noimages
  retained/newthreads/nodes/dependencies; O(1), no perframe extra listallocation
  once initialized. Signedages retained as clockevidence; nofakefresh/retimestamp.
- Launchtests replace obsoleteimportsubprocesstimeout check with validpath/no
  childprocess andmissingpath/notexecutable checks. Newfakeinitializationtests
  check missingdeps/CUDAfail/modelonce/warmup2 andboundedstatssigned/nonfinite.
  Existingperception/aim/alignment helpertests included, excludeslint.

Model/conf/FP16/image_size/warmup2/FPS15/sourceJSON/topics/QoS and300msmission
freshness unchanged. NoFP/behaviorlogic tweak. Onmissingdeps realYOLO exitsbefore
validdetections; alignmentpreview maystart earlier butownsno control, missing
missiondetection stillblocksarm. Parentlaunch mayremain withpreview; existing
runnerhealth/timeouts retained. Noactualsavedstartupduration measured orpromised.
Diagnostics overheadnotmeasured. Sourceage entry includes capture/Pi/network/
DDS/executor/clock, notpureWiFi. Floorstillblockeduntil freshpipelineevidence.

Assistant onlyAST/bash-n/hash/diffinspection, no projectimports/tests/ROS/GPU/
serial/SSH/hardware. Useronecommandrun_offline.sh: cleanenv,timeout120s tests,
180soverall,207baselinepins+perceptionbeforeandtoolsbefore/after. Importpath
assertsshadowpackageandnorealtorch; fakeinitialization doesnotcreateROSNode.
Noapply/build/flash/commit/push, OFFLINE_QA_NOT_RUN/RUNTIME_NOT_RUN.
AfterQArawreview applysamebytes, userbuildonlyperception, prepareonestationary
pairedWifi measurementwithnewstartup/latencystats beforefloorretry. No more
full3goalattempts merelytolookforREADY. USBstall/Nav2teardownissuesremainopen.

NEXT_STEP=USER_RUN_ONE_OFFLINE_GATE_THEN_REVIEW_CANDIDATE_BEFORE_APPLY_BUILD_STATIC_WIFI


## YOLO-STARTUP-QA40-APPLIED-BUILD-PENDING-20261022-59

Userrun `qa_yolo_startup_20261022_001242_Fb4LcT`:00:12:42–43+07, COMPLETE/RC0,
**40 passed in0.39s**. Reviewed10rawhash/28exacttestedtoolinputs,207baselinepins/
20perceptionbeforefiles; toolsused exactmanifest,sourceafter0,toolsafter0,
gitstatusunchanged0. Shadowimport points tocandidate, torch/ultralytics not
imported bymodule. Fakeinitialization covers dependency/CUDAfailure,warmup2,
modelonce; signedboundedstats/pathchecks included. No projecttests repeatedby
assistant; no actualGPUstartup orWiFiage measurement yet.

Appliedbyteidentical4files tobaseline laptopworktreeHEAD61bf6ea afterrechecking
source/status andtestedmanifest. Backup3oldfiles+filesJSON+gitstatus (5filehash
manifestverified) atqa_r3_yolo_startup_candidate_20261022/before_apply_20261022_001242.
Originalhistorydirty preserved, no otherusercode changed. Files:
- launch/perception.launch.py SHAa10fc08ebcab3699dc151bb0aa7f7f5eabe57354767d567ba7da9df76ae5c23b
- fire_robot_perception/yolo_observer.py SHA072ec994b0aab2bf396b556f322c20bb600d407a5b8e92c896c674101f9b76df
- test/test_perception_launch.py SHA82c4531841d463643ad0bf36c01c99a966c2dc2a713886bc8ad6d4568a151dec
- newtest/test_startup_latency.py SHA3d05565bbc245c788db9982f782c6b3a8fcbd251e4b0f3cb9462cb49279628fd

**QA_PASS_REVIEWED; APPLIED; BUILD_INSTALL_NOT_RUN; RUNTIME_IMPROVEMENT_NOT_MEASURED.**
Onlyperceptionstartupanddiagnostics changed. No mission/Nav2/FW/guard/model/
FPS15/QoS/sourceJSON/stamp changes. Missingdependencies now failrealYOLObefore
validobservations, noCPUfallback; alignmentpreviewearlierownsnooutputs. Existing
runnerreadinessfailclosed. Do not promise85s→44s fromstaticcodechange alone.
Keep55sQAsettles asbefore; removingthem needs separateevidence.

Prepared `qa_r3_yolo_startup_build_tools_20261022`,5pinnedinputs. Useronecommand
buildonlyfire_robot_perception --symlink-install --executor sequential,deadline
180s;installedimport/hash/launchverification30s,overall300s.207updatedruntimepins
and21perceptionpins sourcebefore/after,gitstatusbefore/after andunchangedNav2native
binary enforced. Importverification usesamentselectedinstalledmodule pathwithin
baseline,checkssourcehash/LatencyWindow/startup/co_names; noNodeinit/realtorch/
model/GPU/serial/robotlaunch. BuildtoolAST/bashsyntax/hashinspected, NOT_RUN.
No repeat40QA,Piupdate/firmwareflash required. Sourcepins inoldR3V2correctly
rejectnewperception;do notsilentlyreuseoldruntimepacket. Prepareseparatepaired
stationarycapture withnewpins onlyafterbuildreview. RawV2/tools remainimmutable.

Rollbacksourceonly ifneeded:restorethe3backedupfilesandremoveonlynewtest identified
asbeforeabsent, thenuserrebuildperception; do notgitresetdirtyworktree. No automatic
rollbackperformed. No commit/push thisturn. ExistingUSBstall/Nav2teardowncrash and
camera-sourceage>300ms remainopen; nofloorretryuntilpipelineevidence.

NEXT_STEP=USER_BUILD_ONLY_LAPTOP_PERCEPTION_THEN_REVIEW_INSTALL_HASHES


## YOLO-STARTUP-BUILD-INSTALL-VERIFIED-STATIC-PREPARED-20261022-60

Userbuild `qa_yolo_startup_build_20261022_001936_J7L03e`,00:19:36–38+07:
COMPLETE/RC0,BUILD_RC0,sourceafter0/toolsafter0/gitstatusunchanged0.
Reviewed13raw/5buildtool hashentries,207currentruntime/21perceptionsourcepins.
Colconperceptionpackage0.66s,total0.98s. Installedamentmodule andlaunch hashes
072ec994b0aab2bf396b556f322c20bb600d407a5b8e92c896c674101f9b76df /
a10fc08ebcab3699dc151bb0aa7f7f5eabe57354767d567ba7da9df76ae5c23b matchappliedQA40.
Heavydependencies notimported byverification; noNodeinit/GPU/model wasrun.
Nav2nativebinarybf286522… byteidentical before/after. **BUILD_INSTALL_IMPORT_PASS;
GPU_STARTUP_AND_WIFI_IMPROVEMENT_NOT_MEASURED.** BaselineHEAD61bf6ea+reviewed
perceptiondelta, Pi59283c5 unchanged, ESPN8R2ELFf649 expected. No newcommit/push.

Prepared `qa_yolo_startup_static_wifi_tools_20261022`,30pinnedinputs. Forkedonly
usedR2readonlyacquisitionhelpers fromverifiedmanifest, oldR2/R3tools/raw remain
unchanged. Not using/forgingoldOFFLINE_QAmarker; this is diagnosticlogger, not
projectQAacceptance. Exactdirtyperceptionallowlist plusupdated207runtime and21
packagehashes guardlaptop; Pi199sourcepins remain. Buildinstall preflight repeats
onlyreadhash/import verification, no rebuild. No newprojecttests demandedfor
reversibleloggingutility; assistantAST/bash-n/hash/dependency/boundaryreviewonly.
**NEW_CAPTURE_UTILITY_RUNTIME_NOT_RUN.**

Nav2composedshadow+YOLO+laptopviewer15FPScap,PiI/O/camera15FPS/rawserial/safety,
mapfire_room_9_10_2026. NativeUSB→Pi,COM→laptop, APWiFi/LANdown; motor/pumpOFF,
cameraEMPTY, closeoldlaunch/monitor. UserplacesInitialPoseonly, noA/arm/goals.
Pi usesunusedstaticCMDinput, noCMDsource; observerrejectsnonzeroCMD/forbidden
commandorarm publisher/goal/ACTnonOFF. Recorder hasno publishers/actionclients;
onlyread-onlyGetState/GetParametersRPCs. Nav2shadow route notrealCMD; no livegateway
launched. Mission_supervisor observeonlyrecordshealth. BestEffortmission/ACTsubs
matchsourceQoS. Readonlydriverallowsstaleperceptionhealth tobe loggedthroughout
staticmeasurement; this neverweakensrealrobothealthguard orauthorizescontrol.

KeepQAsettles5+30+20s andpose-readinessworkflow forcomparison; once realYOLOok+
JPEGarrivefor3s,collect120s. Modelstartupstageprogressprinted every5s. Pose185s,
YOLOfirstdata150s,driver600s,Pi900s bounded;usually4–7min inclmanualpose/start/
cleanup,notguaranteed. ExistingSSHhealth8s/timeoutsretained. Matchedwindow marker
sentfromlaptoptoPi viaSSH; recordactualmarkerreceipttimes, notassumesimultaneous.

AdditionalJPEGperframe metadata stamp/frame_id/bytes/receiptwall+mono onbothhosts
permitsmatching samepublishedframe instead ofsubtractingindependentmaxima; noJPEG
payloadstored. YOLOresults/sourceage/processing/inference andmissionhealthchanges,
ACTcommandstate,kernelUSB/UART,ping/clockbounds/hostCPU/GPU preserved. Sourceage
includesclockdifference/capture/localexecutor/network; no"pureWiFi" claim. Whole
comparison notcontrolledA/B: thisstationaryprofilehasno livegateway/arm/goal; file
pins/durations/workloadscope recorded. Nav2teardownchildcrashmustbereviewed separately,
parentlaunch0 notchildsuccess. CollectionRC0/finalCAPTURE_ONLY neverphysicalPASS.

Afterrawreview, compareYOLOdependency/model/warmupstage timing withold85.3s,
matchsource-stamps acrossPi/laptop, inspectYOLOentryage vsafterinference andmission
healthage before floorretry. Currentcodefix removesduplicateimport, notyetconfirmed
camera-agefix; don'traise300ms missionfreshness orchangeAP/model/FPS toforcePASS.
NoPiupdate/flash/QA40repeat. Source/hardwarecontrols unchangedthisturn.

NEXT_STEP=USER_ONE_PAIRED_STATIC_WIFI_CAPTURE_THEN_REVIEW_TIMING_AND_MATCHED_FRAMES


## STATIC-TOOL-PREFLIGHT-STALE-HEAD-FIX-20261022-61

Userrun `qa_baseline_combined_laptop_20261022_003107`,00:31:07–08+07,stoppedat
PREFLIGHT common.shline88. Verified17raw/30V1tool SHA entries. Laptopreceipt
RUNNER_RC1/FINAL_EXIT_RC2,sourceafter1,cleanupempty. source_after sayswrongHEAD
actual61bf6ea…; **actualbaselinewas correct**. V1check_sources copiedR2's old
expected4e2fe97… even thoughexpected_runmetadata/207sourcepins correctlydescribed
61bf6ea. This is assistanttoolpreparationbug; AST/syntax/hashinspectiondidnot
catchsemanticmismatch. **BLOCKED_TOOL_PREFLIGHT_NO_ROBOT_RUNTIME**; no UART,
Nav2/YOLO/observermain ormeasurement started by laptoprunner; noPiLAPTOP_READY
release orcopiedPi capture. Cannot assessstartup/WiFiage fromthisattempt. Existing
QA40PASS/build0.66s/installhashes remainvalid, no evidence ofrobotregression here.

PreservedV1packet/raw untouched. PreparedseparateV2packet
`qa_yolo_startup_static_wifi_v2_tools_20261022`,32pinnedinputs:
- check_sources exactlaptopHEAD61bf6ea,Pi59283c5 retained. AssertPROFILESexpected
  matchesexpected_runmetadataforbothroles; wrongHEADstillrejects andmessagegives
  actual+expected.207runtime/21package/dirtyallowlist retained, no"ignoreHEAD" bypass.
- commoncreatesreceiptbeforefirstcheckandcapturesstderrinsource_before, avoiding
  emptyreasonfileontheoriginalfailure. Otherstaticguards/configurationunchanged.
- prepare_copy.sh checksmanifest andlocalreadonlyidentitybeforeSSH,stagesonlypinned
  inputs preservingmaps/subdirs,thenremotehash+PisourcecheckbeforeanyROSlaunch.
  No oldQAmarker reusedorforged; thispreflightstep is notclaimingnewprojecttestPASS.
- Capturedrivers,viewer,sourcepins,model,FPS15,5+30+20ssettles,120smain remainbyte
  identicaltoV1. Robot/FWbaselinecode untouched; no build/flash/QA40repeat.

AssistantAST/literalmetadata/datahash/bash-n/referenceinspectiononly, no project
code/tests/ROS/SSH/hardware executed. NewV2runtimeNOT_RUN. Userfastprepare_copy
willactuallyexecute checkerandfailbeforelaunchifidentity/config inconsistency
remains. OldPiA maystillwait180sforhandshake; Ctrl-Cthatoldsessionifstillopen,
thenusefreshV2AandnewLOG_PI. No broadprocesskill/resetESP neededfromthisfailure.

Lệnh hiện hành: docs/codex/current/YOLO_STATIC_WIFI_V2_PREFLIGHT_FIX_2026-10-22.md.
AfterpreparecopyOK useV2A/B foronepreviouslyauthorizedstationarycapture, motor/
pumpOFF,APWiFi/LANdown/native+COM,cameraEMPTY,userInitialPoseonly,noA/goal.
No floorretryuntilcapturecontainsactualdata. No commit/push thisturn.

NEXT_STEP=USER_FAST_PREFLIGHT_AND_COPY_THEN_NEW_STATIC_A_B_NO_REBUILD_NO_REFLASH


## STATIC-V2-SCP-DOT-COMPATIBILITY-FIX-20261022-62

Userpastedprepare_copy output:32toolhashesOK, laptopHEAD61bf6ea correct and207
runtimepins/sourceidentitypassed withreviewedperceptiondirtydelta. Then
`scp: error: unexpected filename: .`. NoSTATIC_V2_SOURCE_PREFLIGHT_AND_COPY_OK,
no completedremotehash/Pisourcecheck; no newROS/YOLO/UARTmeasurement started by
prepare_copy. **LOCAL_SOURCE_PREFLIGHT_PASS; COPY_BLOCKED; RUNTIME_NOT_RUN.**
This is toolcopycompatibilitybug inassistantpreparedscript, notrobot/WiFi failure.
Source staging/. gets rejected asdotdirectory byreceiver; no needdisableSSH/scp
filenamechecking oruseunsafe-Tflag. Existingbuild/QA40/installresult remainsvalid.

Preservedpreviousprepare_copy.sh and32inputmanifest withreviewJSON in
qa_yolo_startup_scp_fix_20261022,3filehashmanifest. UpdatedsameV2packetpath only
prepare_copy.sh anditsmanifest. Nowmkdirremote/maps, copyexplicit30pinnedroot
regularfiles+tools.sha256, then2mapfiles toexactmapsdirectory. No sourcename'.',
recursivecopy,temporaryfolderorunsafeoption. Unexpectednewnestedpathfailsclearly.
No robot/FW/driver/sourcepins/guard/FPS/map/model/runtimebehavior changes. Same
32pinnedinputs; before/aftercopytoolSHA saved. Assistantbash-n/hash/dataargument-
partitioninspectiononly, noexecutingscp/SSH/projectchecks/tests/ROS/hardware.
ActualremotecompatibilityNOT_RUN; userrerunthesameprepare_copycommand, thenstart
newV2A/B onlyafterSTATIC_V2_SOURCE_PREFLIGHT_AND_COPY_OK=1. Don'trepeatbuild,
flash orQA40. IfoldAisstillwaitingCtrl-CitbeforefreshA; userInitialPoseonly,
motor/pumpOFF/cameraEMPTY/native+COM/APWiFi/LANdown asbefore.

NEXT_STEP=USER_RERUN_SAME_PREPARE_COPY_THEN_STATIC_A_B_AFTER_COPY_OK


## YOLO-STATIC-WIFI-MEASURED-20261022-63

User run `qa_baseline_combined_laptop_20261022_004908`, main 120.0004 s.
**STARTUP_IMPROVED_COLLECTION_COMPLETE_STEADY_WIFI_FRESHNESS_BLOCKED**.
Verified 112 laptop / 44 Pi / 3 UART / 32 tool manifest entries (overlap);
207 laptop and 199 Pi source pins unchanged before/after. HEAD laptop 61bf6ea
plus reviewed 4-file startup delta, Pi 59283c5; N8R2 ELF f6496e95… .

YOLO launch→ready 85.26→43.97 s in these two runs (not controlled benchmark);
import 41.216 s, GPU warmup 2.241 s. Exact main processing mean 18.03 ms,
inference 16.90 ms, 1363 ok + 8 camera_stale. Valid ok source age p95 242.38 ms,
max 2481.36 ms; 59 >300 ms. Mission 8 recovered stale episodes, period 14.8–15.1 s,
28.50/120 s blocked, final health OK. Main laptop odom gap max 1313.21 ms,
TF 1315.36 ms, scan 2281.06 ms, JPEG 2808.49 ms / 11.38 FPS.

Matched 1365 common frames: Pi receive age median/p95 18.76/29.67 ms,
laptop 24.92/210.38 ms. Clock uncertainty prevents exact one-way RF inference.
Pi-only82 is observer difference, not exact packet loss. Pi steady 134.43 s differs
from laptop120 s; don't compare their full-window counts as identical scope.
ACT in common120 s: Pi600/maxgap274.54ms, laptop576/maxgap1320.18ms, all commanded OFF.
Ping whole run 274/266, loss2.92%, mean346.87ms/max3576.13ms. No aggregate resource
exhaustion observed; short single-thread/executor blocks remain possible.

UART556 COMPLETE, no main TX stall/reset/USBdetach; startup POWERON+reenumeration
00:49:15 before I/O/main cause unknown. Nav2 still SIGTERM after INT10s/RC143,
no crash-6 observed; both cleanup empty. No goals/arm, CMD nonzero0, forbiddenpub0.
Capture RC0 is not motion acceptance. Raw summary YOLO age with zero stamp is invalid;
review excludes camera_stale from numeric age, retains states. 5s latency windows
include boundary/pre-main callbacks; do not call callback584ms main inference time.

Full Vietnamese report: `docs/codex/current/YOLO_STATIC_WIFI_RESULT_2026-10-22_004908.md`.
Analysis and matched-frame JSONL in `qa_yolo_startup_static_wifi_v2_tools_20261022`.
Prepared read-only `qa_wifi_periodic_inspection_20261022/inspect.sh`: cached AP info,
driver/power-save/AP fields, ping20packets; no scans/settings/secrets/ROS/hardware.
Assistant shell syntax/hash/data review only; new inspector actual execution NOT_RUN.
No robot/FW/guard/model/FPS changes or commit/push. Preserve old pinned captures;
future runtime needs a fresh reviewed source/history snapshot.

NEXT_STEP=READ_ONLY_WIFI_DRIVER_POWERSAVE_AND_AP_CONFIGURATION_INSPECTION_BEFORE_NEXT_RUNTIME


## WIFI-READONLY-CLOCK-MISMATCH-20261022-64

User `qa_wifi_readonly_20261022_120619_ym1naF`, laptop 12:06:19–12:06:41+07,
collection RC0, 5 raw hashes + 3 input hashes verified. No ROS/hardware runtime.
**IDLE_PING_GOOD_GROSS_CLOCK_MISMATCH_BLOCKS_NEXT_ROS_CAPTURE**.

Idle ping20/20, loss0%, mean9.737ms, max79.746ms (19.025s). Laptop wlp8s0
10.0.0.253, enp7s0DOWN; Pi wlan0 10.0.0.1, eth0DOWN. AP FireRobot_AP2.4GHz
channel11. Laptop driver rtw89_8852be/kernel6.8.0-138, NMpowersave default;
PCI runtime control on/active; rtw89 core disable_ps_mode=N and three PCI disable
flags=N. These do not prove actual 802.11 PS ON. Pi brcmfmac/kernel5.15.0-1110,
Hotspot modeap/bg/channel11/powersave disable; SDIO runtime auto/unsupported.
Both iw missing; hostapd files inaccessible/not present, AP managed by NM.
Cached signal100 and rate65Mbps are not measured throughput/airtime; Pi AP
rate0/signal0 is not proof weak signal. Cached neighbors do not establish congestion.

Critical new observation: laptop `2026-10-22T12:06:19.708+07`, Pi
`2026-10-12T14:52:22.881+07`, approximately 854036.83s apart (sequential snapshots,
not exact offset). Current NTP status and reboot cause were not collected. Do not
assert NTP disabled or clock loss root cause. Correct current clock state before
another distributed TF/source-age measurement. This does NOT explain away run004908,
which had near-zero clock bounds and real steady-state arrival gaps. Short unloaded
ping does NOT overturn loaded120s2.92%loss/346.87msmean/3576msmax result.

Prepared qa_pair_clock_check_20261022: readonly timedatectl flags +5 existing
SSH clock probes; conservative ±50ms measurement interval budget, not a change of
300ms mission/350ms ACT guard. Too-wide bound INCONCLUSIVE, no usable probe BLOCKED.
NTPflag=yes alone is insufficient. No claim holdover/reboot-safe time solved.
User first closes ROS/motorpumpOFF, reconnects LAN laptop↔Pi and laptop Internet,
enables existing Pi network-time service with explicit `sudo timedatectl set-ntp true`,
waits for sync then runs check.sh10.42.0.185. After WiFi return/LANdown, check.sh
10.0.0.1 before new runtime packet. No manually forced date, packages, WiFi changes,
firmware/build, model/FPS/guard change. Inspector result actual reviewed; new clock
recovery/checker NOT_RUN by assistant (shell syntax/AST/hash/data-only verification).

Report/runbook: docs/codex/current/WIFI_CLOCK_INSPECTION_2026-10-22_120619.md.
No commit/push. Keep old pinned runtime tools unchanged; next capture needs a new
reviewed history/source snapshot after this journal entry.

NEXT_STEP=RESTORE_PI_NETWORK_TIME_WITH_LAN_INTERNET_THEN_VERIFY_PAIR_CLOCKS_BEFORE_WIFI_RUNTIME


## EXISTING-CHRONY-WIFI-DISCOVERED-20261022-65

User recalled automatic WiFi clock sync and explicitly authorized SSH over LAN.
Assistant performed read-only LAN SSH and local daemon inspection (sandbox socket
access initially denied; retried through approved escalation). Saved pi_lan.log
and laptop.log + evidence.sha256. No clock/config/service/network mutation.
**EXISTING_WIFI_NTP_CONFIRMED_BY_JOURNAL_CURRENT_LAN_SYNC_OK_WIFI_ONLY_RECHECK_PENDING**.

Laptop chrony enabled/active, UDP123 listens, existing
/etc/chrony/conf.d/90-fire-robot-server.conf allows10.0.0.0/24 and10.42.0.0/24,
localstratum8 for offline relative laptop time. Pi chrony enabled/active,
/etc/chrony/sources.d/90-fire-robot-local.sources already contains10.0.0.253 and
10.42.0.1, bothiburst/prefer/minpoll3/maxpoll5. No new NTP setup required.

Pi journal proves it selected WiFi10.0.0.253 at oldOct12 14:53:05, reported
854038.898972s error, then stepped to Oct22 12:07:04. This occurred AFTER readonly
inspection120619 and BEFORE LANonline13:12:34/selected13:12:38. Thus entry64's
large offset observation was real but transient; suggesting LAN as necessary
recovery was incomplete and overlooked existing config. Intermittent no selectable
sources events at12:07:20/12:08:00/13:00:27 recorded, cause not established.
Current13:13LAN selected10.42.0.1, NTP/NTPSynchronizedyes; Pi reported offset~6us
is a chrony estimate, not independent exact offset or WiFi verification.

User-dirty scripts/fire_robot_pi_session.py has synchronize() before start(),
3SSHsamples/100ms budget, authorized set_clock ifneeded; script demo code is not a
WiFi-connect hook and current standalone QA runners don't invoke it. Read only,
not modified or launched because it also owns ROS startup. No need a secondserver.

Prepared qa_existing_chrony_review_20261022/check_wifi.sh (3 pinned tool inputs):
user disconnectsLAN/reconnectsFireRobot_AP, waits30–60s, runs one read-only check.
Require configured laptopIP10.0.0.253/routewlp8s0/LANdownboth, Pi selected^*10.0.0.253,
NTPflagsyes and5SSHsample bestbound inside±50ms. Budget unchanged fromentry64,
not robot300ms/350ms guard change. WAIT/inconclusive saved distinctly. Newactual
WiFi-only run NOT_RUN; assistant shellsyntax/AST/hash only, no tests/ROS/hardware.
No RTC/UTC guarantee offline; laptop local fallback gives relative coherence.
Journal evidence does not show why initial boot time was old or why sources were
intermittently unselectable; not proof of a driver fault. Existing004908nearzero
clockbounds and8stale events remain valid. No commit/push or robot/FW changes.

Next new ROS packet must gate clocks before starting nodes, inspect exact chrony
source, and refresh reviewed history/source pins; never repin old raw captures.
Report: docs/codex/current/EXISTING_WIFI_NTP_REVIEW_2026-10-22.md.

NEXT_STEP=USER_RECONNECT_PI_WIFI_LAN_DOWN_THEN_VERIFY_EXISTING_CHRONY_SOURCE_AND_CLOCK_BOUND


## WIFI-NTP-HANDOVER-SSH-MEASUREMENT-20261022-66

User run qa_existing_ntp_wifi_20261022_132330_osNqfZ,13:23:30–36+07,RC2;
9 raw + 3 V1tool hashes verified. Same date and NTPflagsyes both hosts; Pi source
WiFi10.0.0.253 has Reach1/LastRx22s/estimate+24ms±49ms, but selected^*LAN10.42.0.1
retains old sample46s ago. Both LANdown/route/IP preflights passed. This is stale
source reference/handover pending, not proof physicalLANstillup or new10dayoffset.
WiFi source estimate is not independent exact-clock proof.

Fresh-SSH whole-invocation clock probe: RTT463.859ms, interval[-14.645,+449.214]ms.
Includes authentication/interpreter startup; interval valid but too broad to prove
±50ms or an actual464msclock error. V1 mislabeled the workflow as near-instantclock
verification without separating handshake; own tool limitation, not measuredWiFiRTT.
**WIFI_NTP_RESPONDED_SOURCE_HANDOVER_PENDING_CLOCK_OFFSET_NOT_ESTABLISHED**.

Prepared separate qa_ntp_wifi_persistent_v2_tools_20261022 (4 inputs), preserving
V1/raw: open oneSSHstream, awaitREADY then8id-bound request/responses, fourtimestamps
client-send/peer-receive/peer-send/client-receive give interval without symmetric
latency assumption. Handshake excluded; executor/transport delay still included.
Wall-vs-monotonic jumps checked on both hosts; protocol error cannot PASS. Snapshot
only, no UTC/reboot/loadedjitter guarantee. Clock alignment reported separately from
source selection/NTPflags; budget±50ms unchanged, robot300ms/350msguards unchanged.
Readonlychronyqueries up to60s toobservehandover; never reselect/offline/restartdaemon,
changeclock/network/driver/firmware, or startROS. User closeslaunch/motorpumpOFF,
WiFiPi/LANdown, runscheck_wifi.sh once, sendsLOG_CLOCK. ActualV2NOT_RUN byassistant;
AST oflocal/remote Python andbash-n/hash reviewed only. No projecttests/build/runtime.
No commit/push. NextROS packet must gate clocks and use freshsource/historypins.

Report: docs/codex/current/WIFI_NTP_HANDOVER_RESULT_2026-10-22_132330.md.
NEXT_STEP=USER_RUN_ONE_PERSISTENT_SSH_WIFI_CLOCK_CHECK_BEFORE_NEW_ROS_PACKET


## NTP-WIFI-ALIGNMENT-PASS-DAEMON-REFRESH-20261022-67

User qa_ntp_wifi_v2_20261022_133556_kiCVxH,13:35:56–13:37:03+07,runnerRC2,
probeRC0. Verified 11 raw entries/4V2toolinputs. 8valid fourtimestamp probes;
bestPi−laptop[+0.624544,+2.480787]ms,steadySSHRTT1.888910ms,bootstrap787.512ms
excluded. Math recomputed from rawdata, all8valid, no newprojecttest execution.
**CLOCK_ALIGNMENT_PASS_ONGOING_WIFI_NTP_SELECTION_NOT_CONFIRMED**.

Clockalignment budget±50ms met, but no ^* in sources table (all^-). Tracking RefID
10.42.0.1 is historical, RefTime06:22:47UTC vs06:37:02UTC snapshot, ~855s old.
Do NOT call LAN currently selected based only onRefID. WiFi10.0.0.253Reach3,
LastRx54s,sourceestimate−13ms±39ms. NTPflagsyes both, insufficient forfreshNTP
proof. No ten-dayoffset now. Driver/network/selectionrootcause still unproven.

Prepared qa_ntp_wifi_refresh_tools_20261022 (5inputs). Explicit one-time maintenance
of existing daemon while bothROSstopped/motorpumpOFF/WiFiPi/LANdown: capture
sudochronycselectdata/ntpdata, onlineWiFipeer,burst4/8onlythatpeer,reselectonce,
observe45s. IfWiFialready^*, skip mutations. NoofflineLANfallback/configfile edit,
serviceinstall/restart,manualsetdate/makestep,ROS/serial/firmwareoperation. Existing
chrony may discipline time peritscurrentpolicy, soROSidleguard applies. Interactive
SSH/sudoTTY, passwordnothandled/storedbytools. Thenreusebyte-identicalV2probeand
assess (actually8probeverifiednow), requiringbothalignment andWiFiselectedsource.
If stillblocked, saveactualselection/NTPtestdiagnostics; noautomaticrepeat/forcingPASS.

Primarydocs chrony4.2 chronyc.html confirm online/burst/reselect andsource/reference
semantics. Newwrapper onlybash-n/AST/hashreview, actualrefreshNOT_RUN; userexecutes.
Robot300ms/350msguards,model,FPS15,FWN8R2 unchanged; no commit/push. No fullROSretry
forclock alone. Afterclock+NTPselectionPASS, newstaticpacketclockgate+freshsourcepins
theninvestigate00490815sjitter. EarlierRAW/resultsnotmodified.
Report: docs/codex/current/NTP_WIFI_ALIGNMENT_AND_REFRESH_2026-10-22_133556.md.
NEXT_STEP=USER_REFRESH_EXISTING_WIFI_NTP_ONCE_AND_VERIFY_WITH_PERSISTENT_SSH_BEFORE_ROS


## NTP-REFRESH-INTERACTIVE-TIMEOUT-FIX-20261022-68

User qa_wifi_ntp_refresh_20261022_135126_7g0axL,13:51:26.845–13:53:11.889+07,
RC137 after105.043s atPI_NTP_REFRESH. Verified4rawhashes/5pre-fixinputhashes.
pi_refresh.log0bytes, no result.json/pi_clock/probe. **BLOCKED_INTERACTIVE_SSH_WRAPPER_NO_NTP_VERIFICATION**.
No receivedPihelperoutput; cannot establish actualremoteactions. Do notcallESPbad,
NTPrefreshfailed, or erase133556CLOCK_ALIGNMENTPASS0.625–2.481ms.

Found concrete assistantwrapperbug: `timeout100s ssh-t ... | tee` lacks--foreground,
which is required for interactivecommandTTYreads in this invocation. LocalGNU
`timeout--help` confirms. GenericlocalPTY stand-in (noSSH/NTP/ROS/projectimports/
hardware) reproduced blockedread withoutforeground/timedout124; withforeground,
readprobe_input succeeds/RC0. Firstharnessassert expectedexact137, got124; corrected
assert toacceptdocumentedtimeouttermination124or137 while requiringnoreadcompletion,
preservedinitial_expectation.json. This provesTTYdefect, notexacthistoricalsignalstate.

Saved5oldinputs+manifest inqa_ntp_refresh_tty_fix_20261022/before. Patchedsameuser
packet qa_ntp_wifi_refresh_tools_20261022: timeout--foreground--verbose keeps100s/
killafter5; SSH-v-Etransportlog, begin/end/SSH/TEEcodes; helperfirstentrymarker.
NTPcommands/guards/probe/assess unchanged (probe/assess byteidentical). 5inputmanifest
updated. GenericPTYactualPASS, shellsyntax/AST/hashreview; newactualSSH/NTPNOT_RUN.
NoOSconfig/clock/robot/FW/guard/model/FPSchanges orcommitpush. Userrerunthesamecommand
onFireRobot_AP/LANdown/ROSclosed/motorpumpOFF; PisudopromptTTYpasswordnotstored.
If blockedagain, transport/entrymarkers distinguishconnection/remote/sudophases;
noautomaticrepeat. ToolfixREADME/reportupdated, oldraw/manifests preserved.
Report: docs/codex/current/NTP_REFRESH_TTY_FIX_2026-10-22_135126.md.
NEXT_STEP=USER_RERUN_SAME_WIFI_NTP_REFRESH_COMMAND_WITH_TTY_FIX_AND_TRANSPORT_LOGS


## WIFI-NTP-REFRESH-PASS-CAMERA-NEXT-20261022-69

User qa_wifi_ntp_refresh_20261022_140306_EsASKW,14:03:06–25+07,19.630s;
11raw/5toolhashes verified, interactiveSSH/TEE/probe/runnerRC0,PHASECOMPLETED.
**PASS_WIFI_TIME_SNAPSHOT_NO_MORE_STANDALONE_CLOCK_RETRY**. Pi selected^*10.0.0.253
withfreshRef07:03:18/20UTC, NTPflagsyesboth.8validpersistentSSHprobes;
Pi−laptopbestbound[+3.028266,+4.809837]ms,RTT1.818527ms,within±50msbudget.
TTYfixactualSSHpassed; sourcebeforeall^?/selectdataSstale, onlineWiFi/burst4of8/
reselect restoredWiFiselect. Tests1111111101→1111111111, peerDelay189.4→34.8ms;
chronyestimates/cumulativeTX/RX notexactRFpacketlossorcontrolledABcausality.
No configfile/manualdate/firmware/ROS/guardchange; lastingholdover/rebootnotproven.
Stop clock-only retests. Existing0049088stale/120s andNav2forcedshutdown remainopen.

Offline source review found nativecameraunpinRMWpublicationmode; serial/TFexplicit
ASYNC. NativeV4L2pipe alreadybufsize0, queue1 andBEST_EFFORTdepth1, so do not claim
buffered65536readbug. ActualcameraRMWenvironment wasn'tcaptured byold2-targetsampler;
notprovenSYNCHRONOUSinthatrun. HumbleFastDDSprimarydocs describe defaultSYNC and
ASYNCenqueue/thread behavior; XMLoverride1 canoverrideenv, futurecapturemustcheck.
OldidlePSreconnectAB20/10hadnosignificantbenefit, no newpower-save/APtuning now.

Prepared qa_camera_dds_async_candidate_20261022 shadowfire_robot_bringup,4filedelta:
robot/sensors declare/forward native_camera_publication_mode defaultASYNC andSYNC
rollback, onlynativecameraNode env scoped; camera3monotonic_nsreads/frame +2O1max
countersinexisting10sstats; testsbackend/scope/forward/bothmodes/stamp/JPEG/CameraInfo/
budgetdrop/reset + existingC2serialTF/safetywiring. Unbufferedcapture,750000Bps,FPS15,
QoSdepth1/model/mission300ms/ACT350ms/FWN8R2 unchanged. ASYNCmaystillblockunderresource
pressure, notclaimjitterfixed. Noheavydeps/newapp-timers/topics/threads/decode; middlewareASYNCusesabackgroundthread andmustbemeasured.

Candidate notapplied, noactualQA/build/ROS. AssistantAST/bash-n/datahash/source review
only. User runoneboundedofflineQA (camera+C2launch regressions, noROSinit/GPU/device),
LOG_QA forreview beforeapply/commitpush/deploy. Freeze207runtimepins, bringuporiginal
hashes, revieweddirtyidentity andhistorySHA afterentry69; don'trepinoldrawcapture.
No commit/push. Report: docs/codex/current/WIFI_NTP_PASS_AND_CAMERA_NEXT_2026-10-22.md.
NEXT_STEP=USER_RUN_ONE_OFFLINE_SHADOW_CAMERA_ASYNC_QA_THEN_REVIEW_BEFORE_APPLY_COMMIT_DEPLOY


## CAMERA-ASYNC-OFFLINE-PASS-APPLIED-20261022-70

User qa_camera_async_20261022_142547_4Ut1Pj,14:25:47–48+07,15PASS0.42s,
runnerRC0/PHASECOMPLETED/sourceafter0/statusunchanged0/toolsafter0.
Verified10raw/38pinnedinput/207runtime/30originalbringup/historySHA hashes;
shadowimportnativecamera path verified. No blocking scoped-diff findings.
**OFFLINE_CAMERA_QA_PASS_TESTED_BYTES_APPLIED_PI_RUNTIME_NOT_RUN**.

Applied exact4testedfiles to laptop-baseline worktree (HEAD61bf6ea retained):
launch/robot.launch.py,launch/sensors.launch.py,fire_robot_bringup/native_mjpeg_camera.py,
test/test_camera_launch.py. Guardedoldhash/head/status/historySHA checks beforecopy,
backup4oldfiles+history atqa_camera_dds_async_candidate_20261022/before_apply_20261022_142547,
verifynewfiles equalshadow andallotherpackage/runtimefiles unchanged. Newsourcepins
andapplication_review.json saved separately; immutabletest/rawinputmanifests unchanged.
Preserveprior4perceptiondirtyfiles andprimarycheckoutuserdirtyscripts/maps. No gitstage/
commit/push, Piupdate/build, hardware/runtime or firmware change byassistant.

Change is nativecamera-scoped FastDDSASYNC, default15FPS/budget750000/QoSdepth1/latest
queue1/stamp/JPEG unchanged. Rootlaunchargument native_camera_publication_mode forwards
ASYNC/SYNC rollback. O1image/pairpublishmaxdurations appendtoexisting10sstats/reset,
noappthread/timer/topic/decode dependency. MiddlewareASYNC hasbackgroundthread;
CPU/RAM/queuing/blockingmuststillbemeasured. Safetywatchdog/300msmission/350msACT/
Nav2andYOLOlaptop/ESP32-S3N8R2rawserial unchanged. XMLQoSoverride1 couldoverrideenv,
futurecapturemust verifyactualcameraRMW/XMLenvironment; notclaimjitterfixed.

ExistingYOLOstartup4filedelta alsoawaitscommit (offline40PASS/build0.66s reviewed,
measured85.26→43.97s inonecomparison). Readyfor2logicalcommits: perceptionstartup
4files, thenbringupcamera4files+trackedvalidationhistory (entries47–70). Commitonly
whenuserexplicitlyauthorizes, thenpushfullbodies peruserworkflow; notautocommitfromQA.
Draftcommitbodies andverifiedchanges saved inpacket forconcrete review.
Aftercommitpush: Piupdatebaselinewithbackup/sourcechecks, userbuildbringuponly,
thenoneclock-gatedstaticWiFicapturewithCOM/odom/TF/JPEG/DDS/CPU/publishmetrics.
No extraoffline15testrepeat or standaloneclockloop required. NTP140306snapshotPASS
3.028–4.810ms,sourcefresh10.0.0.253 remainslastverified; futurepreflightmustrecheck.

Report: docs/codex/current/CAMERA_ASYNC_QA_APPLIED_2026-10-22_142547.md.
NEXT_STEP=USER_AUTHORIZE_REVIEWED_COMMITS_AND_PUSH_THEN_PI_UPDATE_BUILD_STATIC_CAPTURE


## AUTHORIZED-TWO-COMMIT-RELEASE-20261022-71

User explicitly authorized: "Commit và push hai bản". Reviewed exact source
bytes against prior perception and camera QA; index was empty and only the
nine reviewed paths were dirty in the baseline worktree. Primary user scripts,
maps, and firmware are outside this release.

Perception commit: dbab2caf56e0c1a9ac87de97f67d6d57913a491e
- Four perception source/test files; 40 offline tests passed, user build/install
  checked, one startup observation 85.26 -> 43.97 s. Loaded stale episodes remain.

This accompanying bringup commit contains the four tested camera files and the
tracked validation history entries 47-71. Camera gate: 15 passed in 0.42 s, exact
bytes applied, no runtime acceptance claim. Full commit bodies describe scope,
QA, limits, rollback, and next steps. Backup/source snapshots and commit receipts
are saved in qa_camera_dds_async_candidate_20261022/release_two_commits_20261022.

Push verification is a separate step, recorded with both final SHAs in the
current release ledger. Do not treat commit creation or offline PASS as a Pi
build, loaded WiFi PASS, clean Nav2 teardown, or motion acceptance. No firmware,
model, camera 15 FPS, watchdog, or freshness guard changes.

NEXT_STEP=VERIFY_PUSH_THEN_USER_PI_BASELINE_UPDATE_BUILD_AND_CLOCK_GATED_STATIC_CAPTURE


## CAMERA-ASYNC-STATIC-UDP-REVIEW-20261022-78

Runtime under test: 0edb7cbdb6cab459f9442b922c5906a40c05137f on both hosts;
Nav2/YOLO laptop, Pi I/O/TF/safety, ESP32-S3 N8R2, JPEG640x480 configured15FPS.
User run qa_baseline_combined_laptop_20261022_162728 completed120s main capture.
Raw manifests verified:134outer,51nestedPi,3nestedUART (nested included in outer).

Laptop main120.0104s: JPEG12.7156FPS/max-gap2313.27ms; odom9.6242Hz/max-gap1417.17ms;
odomTFmax-gap861.67ms; scan5.0829Hz/max-gap1013.70ms. YOLO valid-result inference
mean16.4998ms/p9518.27ms; input-agep95317.30/max2280.34ms,77valid-result ages>300ms,
1536ok/4camera_stale by main receipt window. Mission health BLOCKED17.1014s in
10episodes, returnedOK. This is collection completion, not motion acceptance.

Pi observer summary135.1296s includes the laptop measurement tail: odom9.9978Hz,
max-gap110.02ms, max-age14.84ms. Pi JPEG13.1873FPS/max-gap1461.31ms. Do not compare
these maxima as identically bounded windows.1501same stamp/frame_id/length JPEGs
matched in one120s source-stamp window: localPiagep9541.37ms, laptop214.62ms;
intersection only, not exact frame/packet loss or pureRF delay. Both clocks were
measured at endpoints only: Pi-laptop before+12.43..+13.74ms, after-0.91..+0.82ms.

Camera process verified ASYNCHRONOUS/rmw_fastrtps_cpp. Image publish call duration
still reached1206.584ms inside main10s bins; reader queue drops present, budget
drops0/restarts0. Socket backpressure is a targeted hypothesis, not proved sole
cause. Aggregate sparseCPUmean laptop6.54%/Pi37.83% does not exclude thread stalls.
Whole-run ping272sent/264received/loss2.94%, mean189.88/max2752.03ms.

UART511lines COMPLETE; one startup1000ms command watchdog transition recovered
MANUAL->EMERGENCY->MANUAL. No TX-stall/reset evidence in captured UART. Pi kernel
camera reset during startup; no ESP detach in captured kernel. Nav2 child still
crashed during cleanup (SIGSEGV text, exit-6); parentlaunchRC0/emptyprocesses_after
do not prove clean teardown. No goal/arm/nonzero commands/forbidden publishers.

Compared descriptively with004908: JPEG11.38->12.72FPS, maxgap2.808->2.313s;
odommaxgap1.313->1.417s, TF1.315->0.862s, scan2.281->1.014s; missionblocked28.5->17.1s.
One run per revision with different conditions, not controlled causalA/B evidence.
Raw and derived stats retained for reporting, including failures and units.

Evidence: qa_camera_async_static_wifi_v4_tools_20261022/{runs/qa_baseline_combined_laptop_20261022_162728,
runtime_review_20261022_162728.json,matched_frames_20261022_162728.jsonl,analyze_162728.py,
review_162728.sha256}; paths relative to primary repo .codex_qa_backups/.
Report: docs/codex/current/CAMERA_ASYNC_RESULT_AND_UDP_CANDIDATE_2026-10-22_162728.md.


## CAMERA-UDP-NONBLOCKING-OFFLINE-APPLIED-20261022-79

User ran qa_camera_udp_20261022_171008_bUeA4Q,17:10:08.748–17:10:10.914+07;
27tests PASS0.41s, runnerRC0/PHASECOMPLETED/sourceafter0/statusunchanged0/toolsafter0.
Verified13raw hashes (including checkerbinary),42candidate/tool input hashes,
207runtime source pins and30originalbringup pins before/after; all unchanged.
Shadow imports resolve to tested candidate. C++compilelog empty, checkerRC0:
FAST_DDS_XML_PROFILE_OK1/NO_PARTICIPANT_CREATED1 against FastDDS2.6.11/RMW6.2.10.
Named/default participant both select custom nonblockingUDP; socketbuffers/
maxMessageSize/interface defaults retained. No ROS participant/network/USB test.

Applied exact7tested files plus this Markdown history to laptop-baseline worktree,
guarded by HEAD0edb7cb and clean status, original/runtime/tool hashes and backups.
Delta: camera_transport.py, native_camera_wifi_nonblocking.xml, robot/sensors
launch, setup.py, test_camera_launch.py and test_camera_transport.py. All other
tracked runtime/package bytes retained. Primary dirty user scripts/maps untouched.

Opt-in native_camera_transport:=wifi_nonblocking uses custom FastDDSUDPv4 with
non_blocking_send=true only for nativecamera child. Defaultsystem restores normal
transports. Reject conflicting RMW/XMLQoS/env, error on missing installedXML;
setup installsXML. Full socket may discard datagrams/JPEGs; DDS can still block
elsewhere. Disabling builtins removes cameraSHM, so Pi-local camera readers also
useUDP; measureFPS/drop/CPU before calling this an improvement. No new application
thread/timer or JPEGdecode/re-encode. Camera15FPS/model/bytes/stamps/depth1/budget
750000, mission/IR/pump/controlQoS/odom/TF/safety/watchdog/firmwareN8R2 unchanged.

State: OFFLINE_QA_PASS_APPLIED_NOT_COMMITTED_PI_RUNTIME_NOT_TESTED. No commit/push,
Pi sync/build/upload or physical actuation by this step. Performance after this
patch is NOT_MEASURED; do not attach162728metrics to the new UDP configuration.
Nav2 teardown remains a separate open finding. Baseline behavior code exists;
R3floor goal/cancel/newgoal and R4fullfire/IR/pump integration still need acceptance.

Evidence: qa_camera_udp_nonblocking_candidate_20261022/runs/qa_camera_udp_20261022_171008_bUeA4Q;
review/application backups and commit-body draft: qa_camera_udp_nonblocking_release_20261022/.
Next: user authorizes commit+push; Pi fast-forward/buildbringup/XMLverification;
one paired120s staticWiFi capture with opt-in, comparepublishstall/ages/FPS/
missionblocked andodomTF to162728. Rollbacksystem if worse; do not weaken guards.
NEXT_STEP=USER_AUTHORIZE_REVIEWED_CAMERA_UDP_COMMIT_AND_PUSH_THEN_PI_BUILD


## AUTHORIZED-CAMERA-UDP-RELEASE-20261022-80

User explicitly authorized commit and push: "ok push đi b" after reviewing gate79.
This commit contains exactly the seven tested bringup source/config/test files
and RUNTIME_VALIDATION_HISTORY.md entries78-80. The camera transport remains opt-in;
system remains default/rollback. Camera15FPS, mission behavior, safety/freshness,
YOLO model, N8R2 firmware and laptop/Pi placement retain their reviewed contracts.

Pre-commit checks verify exact applied bytes, parent0edb7cb, eight dirty paths,
empty index, and unchanged207existing plus3new runtime pins. Prior27testPASS0.41s
and FastDDSXMLcheckerPASS remain the applicable offline gate; not repeated.
Primary userdirty scripts/maps are excluded. Full commit body records the actual
120s pre-patch metrics, mechanism, packet-loss tradeoff, rollback, QA and limits.

The final commit SHA and remote verification are recorded after push in
qa_camera_udp_nonblocking_release_20261022/commit_receipt.json, push.log and
push_verification.json, and linked from the current docs release ledger.
Pi is not updated by commit/push. Next: Pi fast-forward to the pushed SHA,
user-run bringup build/install/XML check, then one paired120s static WiFi capture
with native_camera_transport:=wifi_nonblocking. UDP performance remains
NOT_MEASURED; Nav2 cleanup crash and R3/R4 hardware acceptance remain open.
NEXT_STEP=VERIFY_PUSH_THEN_PI_UPDATE_AND_USER_BRINGUP_BUILD


## NAV2-87 — 2026-10-22: child costmap config and scoped navigation library lifetime

Parent:66d87b43c1f3af394fa52fee30a7a01c5f9bb257; source-only fix applied after QA.
Evidence:qa_production_lifetime_20261022_195129_kuPTs5 (43rawhashes verified).
17Python tests PASS0.63s,5C++unitPASS; production container compiled; default/restored
C6 controls SIGSEGV at unmapped layers vtable, both production-manager cases exit0
and layers/RPP unmap after complete manager destruction. No external library lease
in patched probe. Global child parameter matching includes namespace/wildcard/last
file precedence; invalid plugin configuration fails before component construction.

Change: pass complete rewritten YAML to composed container, retain configured
controller and costmap layer/filter libraries via pluginlib until whole component
manager teardown. Preserve existing C6 behavior retention, C5 executor shutdown,
command routes, parameter values and safety guards. No periodic work is added.

STATUS=OFFLINE_REPRODUCER_PASS_SOURCE_APPLIED_NOT_INSTALLED
FULL_PACKAGE_BUILD=NOT_RUN
FULL_NAV2_ACTIVE_RUNTIME_SHUTDOWN=NOT_RUN
USB_CDC_ODOM_STALL=OPEN_NOT_FIXED
FIRMWARE_MODIFIED=NO
PI_MODIFIED=NO
COMMIT_PUSH=NOT_AUTHORIZED_YET
NEXT_STEP=REVIEW_APPLIED_DIFF_AUTHORIZE_COMMIT_PUSH_THEN_USER_PACKAGE_BUILD
