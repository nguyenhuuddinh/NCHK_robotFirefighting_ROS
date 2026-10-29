# IR standoff preview (laptop, observe-only)

The Pi bridge publishes raw ADS1115 counts at about 5 Hz on
`/fire_sensors/ir_raw`. This optional laptop node subscribes to that topic,
`/yolo/detections`, and `/yolo/alignment_preview`. It publishes only JSON
`std_msgs/msg/String` on `/fire_mission/ir_range_preview` (Best Effort). It is
not included in any motion launch and cannot publish Twist, pump or servo
commands. The separate opt-in supervised pump controller described below can
consume this topic; existing mission supervisors do not.

## Approach shadow (no wheel motion)

`ir_approach_shadow.launch.py` is a separate laptop-only diagnostic. It reads
the IR-only `ir_band` from the preview (not the vision-gated `zone`), `/scan`,
`/odom` and `/cmd_vel`, and publishes only
`/fire_mission/ir_approach_shadow` (`std_msgs/String`). A `STEP_CANDIDATE`
means *only* that the current sensor snapshot would support considering one
5 cm step after future live handoff/collision-control work. It is not an
admission proof and has no Twist or pump publisher. `SPRAY_RANGE`, `TOO_CLOSE`,
`UNKNOWN`, stale data, a moving robot, a nonzero command, another raw motion
owner, or inadequate front LiDAR coverage all give `HOLD` and zero suggested
distance. A valid LiDAR scan must cover both sides of the forward cone, have
at least ten finite returns there, and leave at least 45 cm plus the 5 cm
step between the robot and the nearest front return. These conservative
criteria can reject clear open space; never loosen them just to get a green
preview without checking the scanner geometry and hardware setup.

With the Pi, YOLO and IR preview already running, start just this diagnostic
on the laptop after sourcing the ROS environment:

```bash
ros2 launch fire_robot_mission ir_approach_shadow.launch.py
```

In another sourced terminal:

```bash
timeout 8s ros2 topic echo /fire_mission/ir_approach_shadow \
  std_msgs/msg/String --once --full-length \
  --qos-reliability best_effort
ros2 topic info /cmd_vel_raw --verbose
```

Keep motor and pump power off for the first run. Do not start a second Pi,
YOLO or IR preview launch. Use `--full-length` when echoing LaserScan arrays:
the ROS CLI otherwise truncates the printed `ranges`. An observed full scan
contained 400 beams, but only 3 finite returns among 45 front-sector beams.
That is insufficient for a real approach test; inspect LiDAR mounting and
the intended test scene before changing any safety rule.

## Separate live one-step IR jog (opt-in, floor only)

`ir_approach_live.launch.py` defaults to disabled and only publishes String
telemetry. With both `enable_live_ir_jog:=true` and
`allow_provisional_ir:=true`, a fresh false-then-true operator heartbeat on
`/fire_mission/ir_approach_arm` permits at most one 5 cm forward step when
the stable IR-only band is `APPROACH` or `TOO_FAR`. `TOO_CLOSE` holds by
default; a separate `enable_reverse_jog:=true` allows one short reverse step
only when the rear LiDAR sector is clear. `SPRAY_RANGE`
and `UNKNOWN` hold position. This node never publishes pump/servo commands
and never automatically repeats a step. It refuses a concurrent Nav2 status
publisher, pump publisher, raw motion owner or invalid SafetyGate graph.
It must see fresh IR, stationary odometry, a fresh zero safety command and
adequate finite LiDAR returns in the travel direction before owning
`/cmd_vel_raw`. During motion it continuously rechecks arm, IR, odometry,
scan, and ROS graph, then publishes zero until odometry confirms a stop.
On the LAN, DDS may temporarily hide the Pi endpoint's node name. The live
node accepts that only after it first verified the exact expected node name,
type and QoS and pinned its GID; a new GID or extra publisher still blocks or
stops motion. During the zero-command `ACQUIRING` phase only, a transient
graph-discovery gap can clear within 0.75 s; the node never sends a nonzero
command until the topology is valid again. A gap during motion stops at once.
The gate check permits one separately verified `rosbag2_recorder` subscriber
to `/cmd_vel_raw` for diagnostics. It still requires the pinned SafetyGate
subscriber, rejects any other subscriber and keeps the sole raw publisher
rule. The recorder cannot send commands; it only reads the topic.
Restart the live node after a Pi endpoint is restarted.
The Pi and S3 watchdogs remain unchanged. LiDAR is only an obstacle check,
not a distance sensor for the low flame; low obstacles can still be missed.
The operator must put the robot on an open floor and stay ready to cut motor
power before any enabled test. Never run this on a table.

### Experimental post-Nav2 handoff (not yet physically validated)

`integrated_handoff:=true` is a separate opt-in on the same launch. It
requires the canceled Nav2 goal UUID, completed live alignment for that UUID,
fresh mission/gateway/guard proof, camera still centered, exactly the expected
ROS publishers, IR `zone` in `APPROACH`/`TOO_FAR`, and an operator false-then-
true IR arm. Only the `ir_approach_live` node may temporarily own
`/cmd_vel_raw`; the cancel guard recognizes that exact owner, checks a
bounded forward command, and acknowledges that owner before any nonzero
forward command is sent. It restores `CANCEL_CONFIRMED_STOPPED` only after
the IR node reports completion and the SafetyGate and odometry are zero.
Each movement is at most 5 cm, 0.06 m/s and 1 second. Successful partial
progress of at least 1 cm can start another step after a fresh handoff;
the whole attempt stops after 20 steps or 30 cm. `SPRAY_RANGE` stops the
approach; `TOO_CLOSE`, sensor disagreement, lost proof, or no measured
progress aborts. Reverse remains off. These are hard software bounds, **not**
a verified safe distance to flame. The LiDAR can miss a low obstacle.

The full `scripts/start_demo.sh` now explicitly selects
`continuous_approach:=true speed_m_s:=0.08` after that same canceled-goal
handoff. Unlike the legacy 5 cm jog, it keeps one `/cmd_vel_raw` owner and
commands 0.08 m/s without intermediate step releases. Hard limits are 30 cm
of odometry travel **per leg**, and a one-second
no-progress check. After a leg stops and releases `/cmd_vel_raw`, fresh
same-goal stationary/cancel/centered proof can start the next leg. The local
approach has a separate 2 m total cap; reaching that cap stops the robot for
operator review. If fire moves off-center or temporarily loses confirmation
during a leg, IR commands zero,
releases ownership after a stationary hold, and hands the same goal back to
live alignment. Once centered, IR resumes only if the fresh IR band is
`APPROACH`/`TOO_FAR`; if it is already `SPRAY_RANGE`, no more IR motion is
needed.
Each 20 Hz tick rechecks the arm, exact ROS owners, centered-fire proof,
fresh IR, odometry, zero angular command, and front LiDAR clearance (at least
0.45 m + 0.08 m); any failure sends zero. The first trustworthy filtered
`SPRAY_RANGE` candidate sends zero immediately, even before five IR samples
confirm it. The node keeps zero until odometry and SafetyGate are stationary
and the preview confirms `zone=SPRAY_RANGE` without blockers. If confirmation
does not arrive within 1.5 s, it releases raw motion ownership and waits for
new IR samples and a fresh same-goal centered handoff; it does **not** label
the unconfirmed candidate as spray or authorize the pump. Stale/invalid IR
during acquisition or motion also sends zero immediately, requires stationary
zero-command proof, and releases the raw owner. The preview's
`ir_sample_seq` increases only for accepted ADC messages. Considering a new
alignment attempt requires at least two *new* accepted ADC samples after the
stop and a fresh, stable IR-only band; an off-center fire may still have
`zone=UNKNOWN` during this step. If the preview node restarts and its sample
counter resets, the recovery gate starts a new sample-counting epoch. A
republished old preview cannot restart anything. Forward motion
requires the separate, stricter centered-camera, stable-zone and exact-goal
handoff proof. If the band is then `APPROACH`/`TOO_FAR`, the same canceled
goal can be admitted again; if it is confirmed `SPRAY_RANGE`, no further
forward motion is needed.
A temporary YOLO `FIRE_CANDIDATE` during an IR gap keeps only the exact
canceled-goal identity, never motion permission. A different/active goal,
lost arm, foreign raw owner, unsafe scan/odom or coasting more than 3 cm past
a 30 cm leg still fails closed. The pump still needs the independent
guard/gateway and stationary proofs. IR is still transmitted from S3 at
about 5 Hz; S3 firmware and the Pi bridge have not been changed here.
Use `./scripts/start_demo.sh --ir-only` for the first supervised floor trial:
it starts the same live navigation/alignment/IR chain but no pump node or
pump arm. Only after measuring the actual stopping distance at 0.08 m/s
should the full pumping flow be tried. If both camera/IR evidence disappears,
LiDAR proof fails, or the robot stops making progress, it still fails closed;
this code does not implement an unobserved autonomous obstacle detour.

Keep `pump_demo_controller` stopped while IR approaches: its publisher on
`/pump_cmd` is an explicit IR blocker. Start the pump stage only after IR
releases `/cmd_vel_raw`, the guard returns to `CANCEL_CONFIRMED_STOPPED`, the
gateway reports `HANDOFF_READY`, the robot is stationary, and the IR preview
reports `SPRAY_RANGE` with no blockers. The full session script sequences
this handoff and bounded repeat pulses after post-spray review. The
integrated mode has unit tests but has **not**
yet been run on the real robot; do not enable it for a public demo until a
separate no-pump motion trial passes.

Adjust IR boundaries in `config/ir_range_preview.yaml` and restart only the
IR preview launch for changes to take effect. Its thresholds remain
provisional. `config/ir_range_demo_trial.yaml` is a *separate temporary*
trial file with the same current thresholds; check the running node's actual
parameters because an already-running process keeps its previous values.
The flame can change size, so no R threshold is a calibrated distance.
Do not treat this as a validated distance calibration or as automatic pump
permission. Supply the trial file using the `params_file` launch argument.

The `ir_band` field is a tentative classification of the three IR channels.
Each channel is median-filtered separately. The left/center/right baselines
are checked separately; the right channel then selects a provisional band.
The `zone` field stays `UNKNOWN` until the IR band is stable, the current
YOLO frame detects and confirms fire, and alignment reports `CENTERED`.
Missing/stale/invalid inputs, a disagreeing sensor, or an unstable transition
return `UNKNOWN`. The median-filtered R-eye `ir_band` bands are contiguous:
at most 850 is `TOO_CLOSE`, 851-1100 is `SPRAY_RANGE`, 1101-1850 is
`APPROACH`, and above 1850 is `TOO_FAR`. `ir_band` is the provisional raw
candidate; `zone` remains `UNKNOWN` until confirmation. Entry into
`SPRAY_RANGE` needs five consecutive IR samples (other bands need three).
After confirmed spray is lost above R=1100, re-entry is blocked until R<=1050
for five consecutive samples. This 50-count re-entry margin never keeps
`SPRAY_RANGE` active above R=1100. `IR_BAND_NOT_STABLE` or
`IR_SPRAY_REENTRY_HYSTERESIS` blocks pump admission and the legacy one-step
IR jog while in transition. In continuous mode, a trustworthy transition
only triggers an early **stop**; it is never pump permission.
`validated_for_actuation` is **always false**,
including when the field reads `SPRAY_RANGE`. `UNKNOWN` must turn off an
active pump; a new pump pulse still requires its separate safety proof and
operator arm.

The YAML defaults are only for the user's unchanged 2026-10-08 candle/sensor
setup: no flame L/C/R about 26060/26140/25825; in the latest 8-second
samples, R was 951-994 at 40 cm, 1173-1270 at 60 cm, 1581-3885 at 75 cm,
and 3961-7612 at 90 cm. An earlier 90 cm trial gave R about 1367, so these
bands cannot consistently separate physical distances as the flame changes.
These values are **not** a calibrated distance
sensor, guaranteed spray range, or a safe autonomous approach policy. Do not
interpret the band as centimetres or reuse these thresholds after changing
potentiometers, candle, mounting, power supply, or ambient lighting.

Build on the laptop:

```bash
cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --packages-select fire_robot_mission --symlink-install
source install/setup.bash
```

With the Pi bridge running and the laptop connected to the same ROS domain,
start the optional observer in a laptop terminal:

```bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
ros2 launch fire_robot_mission ir_range_preview.launch.py use_sim_time:=false
```

In another laptop terminal, source ROS and this workspace again and inspect:

```bash
timeout 8s ros2 topic echo /fire_mission/ir_range_preview \
  std_msgs/msg/String --full-length --qos-reliability best_effort
ros2 topic info /fire_mission/ir_range_preview --verbose
```

Without YOLO, `ir_band` may respond to IR, but `zone` must remain `UNKNOWN`
with `YOLO_MISSING_OR_STALE`/`ALIGNMENT_MISSING_OR_STALE`. This lets the IR
transport be checked without a flame or any motion. Stop the observer with
Ctrl+C after the test. Leave motor-driver and pump power disconnected for
this first check.

## Stationary candle comparison, when the operator is ready

Keep motor-driver and pump power off. Do not run Nav2, live alignment, or arm
heartbeats. Work over the existing LAN; do not change laptop WiFi. On the Pi,
keep the current `robot.launch.py` if it already has both
`/serial_bridge_node` and `/native_mjpeg_camera`. Do **not** start a second
copy. If the camera was disabled, stop that Pi launch with Ctrl+C and run:

```bash
cd ~/NCHK_robotFirefighting_ROS/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
ros2 launch fire_robot_bringup robot.launch.py \
  use_sim_time:=false use_camera:=true \
  serial_port:=/dev/serial/by-id/usb-Espressif_Systems_Freenove_ESP32-S3_WROOM_N8R2__8MB_Flash___2MB_PSRAM__28372F1F73C8-if00
```

On the laptop, start perception in one terminal:

```bash
cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
ros2 launch fire_robot_perception perception.launch.py use_sim_time:=false
```

Start the IR preview in a second laptop terminal after sourcing the same ROS
environment:

```bash
ros2 launch fire_robot_mission ir_range_preview.launch.py use_sim_time:=false
```

To see the image, run `ros2 run rqt_image_view rqt_image_view` in another
laptop terminal and select `/yolo/image_annotated/compressed`. Keep the flame
centered in the image. In one more sourced laptop terminal, collect each
condition separately, placing the flame before starting the command:

```bash
timeout 12s ros2 topic echo /fire_mission/ir_range_preview \
  std_msgs/msg/String --full-length --qos-reliability best_effort \
  | tee /tmp/fire_ir_preview_60cm.txt
```

Repeat at 40 cm with a different output file. At 40 cm the provisional
`zone` may say `SPRAY_RANGE`; 60 cm may now read `APPROACH` because the spray
maximum was conservatively reduced to 1100. If `zone` says `UNKNOWN`, inspect
`blockers` first.
Neither result grants permission to move or spray. Send both logs for review
before changing thresholds or connecting this observer to behavior logic.

## Supervised pump demo integration (not armed by default)

The separate `pump_demo_controller` is **not** part of the IR preview or
default mission launches. It has two explicit opt-ins:
`enable_pump_demo:=true` and `allow_provisional_ir:=true`. The latter is a
supervised-demo exception to the unvalidated IR calibration; it does not turn
`validated_for_actuation` into true. The controller also requires an operator
heartbeat on `/fire_mission/pump_demo_arm`, fresh `SPRAY_RANGE` with no
blockers, confirmed fire at image centre, exact live cancel/alignment proof,
the canceled UUID still being Nav2's newest goal, no raw motion owner,
fresh zero `/cmd_vel`, stationary `/odom`, and exclusive
`/pump_cmd` ownership. The gateway's fresh `HANDOFF_READY` and guard's fresh
`CANCEL_CONFIRMED_STOPPED` must still match the alignment goal UUID while
the pump is on. Those conditions must remain true throughout the
pulse. One qualifying pulse defaults to 5 seconds and is capped at 5 seconds;
set `pulse_s:=<seconds>` on `pump_demo.launch.py` to choose a shorter pulse.
`UNKNOWN`, lost arm,
motion, stale input or graph conflict publishes OFF and latches the attempt.
It does not automatically restart. Servo GPIO38 is reserved but does not
move in this phase.
After starting the controller, the operator must publish an explicit
`pump_demo_arm=false` before sending the sustained `true` heartbeat. A
leftover `true` publisher from a previous run cannot arm a restarted node.

The Pi serial bridge stops refreshing an abandoned ON after 0.7 seconds.
ESP32-S3 independently requires fresh PUMP frames, an online serial host and
an allowed motor-watchdog state, and caps continuous output at 5 seconds
until it receives OFF. S3 GPIO47 drives only TB6612FNG PWMA, with AIN1=3V3,
AIN2=GND and STBY=3V3 as confirmed by the operator. Software compilation and
simulation do **not** prove pump wiring, flow rate or safe aiming. Keep pump
power disconnected while checking ROS graph and OFF-state telemetry; flash
and energize the actuator only during a separate, supervised bench test.
