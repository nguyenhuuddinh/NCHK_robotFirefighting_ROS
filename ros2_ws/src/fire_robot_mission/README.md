# fire_robot_mission

Package laptop-only kết hợp telemetry Nav2 và YOLO ở chế độ **observe-only**.
Package không publish `Twist`, không tạo Nav2 action client và không publish
`/pump_cmd`.

## Luồng dữ liệu

```text
/yolo/detections -----------+
/yolo/alignment_preview ----+
/navigate_to_pose/... ------+--> fire_mission_supervisor
/odom ----------------------+        |
/cmd_vel_nav ---------------+        +--> /fire_mission/state (JSON)
/cmd_vel_raw ---------------+
```

`/fire_mission/state` chỉ mô tả trạng thái, freshness, hướng căn lửa dự kiến,
mức đồng thuận/xung đột với lệnh Nav2 và các blocker an toàn. Topic này không
được nối vào chuỗi điều khiển.

## Chạy

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY

ros2 launch fire_robot_mission mission_preview.launch.py \
  use_sim_time:=false
```

Diagnostic state machine:

```bash
ros2 run fire_robot_mission mission_preview_diagnostics \
  --seconds 30 \
  --output /tmp/fire_mission_preview.json
```

Đo offset clock laptop và Pi qua LAN, hoàn toàn read-only:

```bash
ros2 run fire_robot_mission clock_sync_diagnostics \
  --peer pi@10.42.0.185 \
  --samples 10 \
  --output /tmp/fire_clock_sync.json
```

## Giới hạn an toàn

- Một detection không đủ để tạo tracking intent; mặc định phải giữ liên tục
  `0,50 s`.
- Timestamp camera cũ hơn `300 ms` hoặc đi trước laptop quá `20 ms` tạo
  blocker.
- Mất target sau khi đã acquire tạo `TARGET_LOST` và intent
  `REQUEST_SAFE_STOP`.
- Đây chưa phải quyền điều khiển. Không có command arbiter, collision checker
  hoặc interlock bơm trong package này.
