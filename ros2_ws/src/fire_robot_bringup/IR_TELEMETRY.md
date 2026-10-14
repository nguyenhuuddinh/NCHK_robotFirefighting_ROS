# ADS1115 IR telemetry (observe-only)

Current hardware: ESP32-S3 reads three 3.3 V IR analog outputs through ADS1115
on the shared IMU I2C bus. The old WROOM sensor/actuator architecture in some
historical documents is not used for these three values.

The ESP32-S3 IR task samples left/center/right at 10 Hz. Its serial task copies
the completed snapshot under `stateMutex` without accessing I2C, and offers a
lower-priority frame at 5 Hz only when the USB TX queue is empty. `STATE`
(odom/IMU, 10 Hz) is queued first. Under serial load, IR may be dropped.

Protocol V2 frame (normal `@payload*CRC16\n` envelope):

```
IR,2,<shared_seq>,<sample_age_ms>,<left>,<center>,<right>,<valid>
```

Counts are unsigned ADS1115 single-ended readings (0–32767). `valid=1` only
for a complete snapshot no older than 300 ms. Invalid frames carry zero counts.
The Pi rejects malformed, stale-valid, out-of-range, or bad-CRC frames. It
publishes JSON `std_msgs/msg/String` on `/fire_sensors/ir_raw` with
`schema_version`, `valid`, `left`, `center`, `right`, `sample_age_ms`, and
`receive_age_ms`. Invalid counts are JSON `null`; an old frame or old serial
session is not republished. The topic uses Best Effort QoS.

This is raw observation, **not** a distance estimate or permission to move or
spray. Flame size and orientation varied strongly in the user's 20–90 cm
samples; no 20/40/60/75/90 cm threshold is committed here. The pump, servo,
motion, and watchdog paths are unchanged.

Hardware verification after flashing the new N8R2 firmware and starting the Pi
bridge (no motor or pump command):

```
ros2 topic hz /odom
ros2 topic hz /imu/data
ros2 topic hz /fire_sensors/ir_raw
ros2 topic echo /fire_sensors/ir_raw std_msgs/msg/String --once \
  --qos-reliability best_effort
```

Expected: odom and IMU remain about 10 Hz, IR about 5 Hz, and IR `valid=true`
with counts that respond to flame presence/distance. Disconnecting ADS1115
should produce `valid=false` with null counts; stopping the bridge should
stop the topic instead of replaying a fresh valid sample. Verify the Pi's
actual `/dev/serial/by-id/` path on the Pi separately. The source defaults
match the confirmed N8R2 path, but Pi's installed package must be rebuilt
before those defaults take effect.
