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
