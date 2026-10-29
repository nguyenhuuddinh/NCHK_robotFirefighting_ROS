# R5 source packet and local install checks — observe candidate only

No operational release, automatic deployment, firmware image or hardware test.
`r5_packet.py` verifies/exports only manifest-pinned regular source files. Build,
install, logs, Git internals, caches and unpinned files are excluded. Keep the
archive SHA256 and packet SHA256 in the handoff; they identify different things.
Do not copy the laptop build/install directories or its symlinks to the Pi.

The accepted underlay must already exist on each host. The default reviewed
paths are:

- laptop: `/home/huudinh/.codex/worktrees/laptop-baseline/NCHK_robotFirefighting_ROS/ros2_ws/install`
- Pi: `/home/pi/NCHK_robotFirefighting_ROS_baseline/ros2_ws/install`

Extract into a NEW dedicated staging folder; never into either Git checkout or
firmware project. Source `/opt/ros/humble/setup.bash`, the host's accepted
underlay, then build the selected staging overlays with `--symlink-install`:
**Pi: fire_robot_bringup + fire_robot_mission; laptop: fire_robot_navigation + fire_robot_perception + fire_robot_mission**.
The laptop navigation overlay is required for actual action-server status
refresh before the first goal. The R5 runner opts in at 5 Hz; the standalone
composition defaults OFF. Refresh uses the existing action-server publisher,
including its real active/canceled UUIDs; it creates no goal or cancel request.
The Pi bringup overlay is necessary for the R5.3 read-only BOOT/session identity
publisher; baseline6d8cf0f lacks that module. Laptop's candidate bringup overlay
has already been built where needed. Do not build the whole workspace or
enable the ignored camsense_x1 package. Pi safety, description and installed
Lidar remain its reviewed underlay. Missing/mismatched dependencies block;
no dependency installation is performed by this packet. This corrects the
QA39 mission-only Pi instructions based on the actual Pi install receipt.

After a successful local build, source that staging workspace's
`install/local_setup.bash`, then run:

```bash
python3 scripts/r5_packet.py
python3 scripts/r5_install_check.py --role laptop
# On Pi instead: --role pi
```

Run these commands from the extracted `ros` root; they start no ROS context,
process backend, SSH, NTP, camera, motor or pump. The checker prints JSON to
stdout and returns 0 only for `STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED`; any
missing/mismatched file returns 2. Save stdout to a new local receipt file.
It does not change networking, services, package installations or the manifest.

Checks: all role module/launch/config bytes and observe flags; setuptools
entrypoint targets plus executable wrappers/interpreters; URDF/BT included
assets; message module presence; host and ELF architecture; executable mode;
Nav2 component registrations/libraries on laptop; Lidar/RSP on Pi. The default
map YAML+PGM is the exact R4 fire_room_9_10_2026 pair. Laptop external model hash
is pinned, confidence is0.15/3-of-5, and the existing YOLO interpreter and
torch/ultralytics venv metadata must exist. Pi does not check/load the model or
Nav2 runtime. No torch import, CUDA probe, dynamic-library load, model inference,
ROS graph, device access or physical firmware identity is certified by this
static inventory. Those remain explicit runtime gates.

Both host receipts must name the SAME packet SHA before any paired observe
session. A laptop PASS alone does not attest the Pi. `--run-observe` is not run
by these checks. The detailed R5.5 runbook is in the project documentation;
its runtime stage remains gated on Pi compatibility and explicit user action.
