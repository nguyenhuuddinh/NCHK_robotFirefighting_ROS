# Hướng dẫn và trạng thái tinh chỉnh Nav2 cho Fire Robot

Tài liệu này mô tả **source và runtime đang hoạt động tới ngày 2026-09-17**.
Lịch sử Rotation Shim + DWB và các lần hiệu chuẩn trước được giữ tại
`docs/codex/HANDOFF_NAV2_TUNING_2026-09-10.md`; không dùng phần lịch sử đó để
suy ra controller đang active.

## 1. Trạng thái active hiện tại

Nav2 chạy trên laptop. Raspberry Pi chỉ chạy Lidar, serial bridge,
`odom -> base_link`, SafetyGate và rosbridge.

Luồng command:

```text
controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_raw
behavior_server ----------------------------------------> /cmd_vel_raw
/cmd_vel_raw -> SafetyGate tren Pi -> /cmd_vel -> ESP32-S3 -> motor
```

`FollowPath` hiện dùng trực tiếp:

```yaml
plugin: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
```

Không có Rotation Shim bọc ngoài và không dùng DWB trong runtime hiện tại.

| Nhóm | Tham số active | Giá trị |
| --- | --- | ---: |
| Controller | `controller_frequency` | `10 Hz` |
| RPP | `desired_linear_vel` | `0.10 m/s` |
| RPP | `min/max_lookahead_dist` | `0.15/0.25 m` |
| RPP | `use_rotate_to_heading` | `true` |
| RPP | `rotate_to_heading_min_angle` | `0.174533 rad` = `10 deg` |
| RPP | `rotate_to_heading_angular_vel` | `1.0 rad/s` |
| RPP | `max_allowed_time_to_collision_up_to_carrot` | `1.0 s` |
| Goal checker | `xy_goal_tolerance` | `0.08 m` |
| Goal checker | `yaw_goal_tolerance` | `0.15 rad` = `8.59 deg` |
| Planner | plugin | NavFn |
| Planner | `use_astar` | `false` (Dijkstra) |
| Planner | `tolerance` | `0.5 m` |
| BT replan | `RateController` | `2 Hz` |
| Local costmap | update | `5 Hz` |
| Global costmap | update/publish | `2/2 Hz` |
| Velocity smoother | output | `20 Hz` |

`smoother_server` có cấu hình `SimpleSmoother`, nhưng Behavior Tree hiện tại
không gọi action `SmoothPath`. Đường từ `ComputePathToPose` được chuyển thẳng
sang `FollowPath`.

## 2. Bằng chứng bài test ngày 2026-09-16

Các lần goal từ log Nav2 lúc 20:44-20:47:

| Goal | Thời gian | Kết quả | Bằng chứng chính |
| --- | ---: | --- | --- |
| Phía trước | `20.86 s` | `SUCCEEDED` | 0 collision, 0 clear/recovery |
| Phía sau lần 1 | `30.72 s` | `FAILED` | 14 RPP collision, 10 local-costmap clear, recovery Spin cũng collision |
| Cùng goal phía sau lần 2 | `24.78 s` | `SUCCEEDED` | 1 collision rồi clear/retry |

Trong cửa sổ test không có lỗi TF/extrapolation, dữ liệu quá cũ,
`Failed to make progress` hay planner mất path. Nguyên nhân trực tiếp của lần
`FAILED` là:

```text
RegulatedPurePursuitController detected collision ahead!
Collision Ahead - Exiting Spin
```

Điểm bắt đầu do BT ghi nhận giữa hai lần goal phía sau lệch khoảng `0.18 m`.
Phải dùng rosbag để tách chuyển động thân xe thật, trượt bánh, sai số odom và
dịch pose AMCL.

### Baseline rosbag đứng yên lúc 22:40-22:50 ngày 2026-09-16

Bag `/tmp/fire_robot_nav2_diag_20260916_YkiCLu/behind_goal_final` dài
`631.31 s`, có `50.647` message. Trong bag này người vận hành chưa gửi goal,
nên `/plan`, `/cmd_vel_nav` và action feedback đều có `0` message. Bag chỉ được
dùng làm baseline đứng yên, không dùng để kết luận nguyên nhân pha quay.

Kết quả baseline:

- `/odom`: dịch chuyển đầu-cuối `0.000000 m`, yaw đổi `0.0000 deg`;
- IMU `angular_velocity.z`: mean `0.000047 rad/s`, độ lệch chuẩn
  `0.000643 rad/s`, biên lớn nhất `0.002 rad/s`;
- `3.280/3.280` scan không có điểm nào lọt vào bán kính robot `0.17 m` hoặc
  vùng cận `0.20 m`; điểm scan gần tâm `base_link` nhất là `0.4207 m`;
- `1.052/1.052` local-costmap frame không có lethal cell nằm trong footprint;
- lethal cell gần tâm footprint nhất là `0.2577 m`, tức còn khoảng `0.0877 m`
  ngoài `robot_radius=0.17 m`.

Kết luận giới hạn: bag không ủng hộ giả thuyết LiDAR luôn tự thấy thân xe khi
đứng yên. Vẫn cần bag có goal phía sau để kiểm tra collision phát sinh khi
quay, TF/costmap biến đổi theo thời gian và footprint sweep của controller.

### Dry-run planner sau baseline

Tại pose `map -> base_link` xấp xỉ `(0.297, 0.213, -179.74 deg)`:

- goal `0.50 m` đúng phía sau tại `(0.797, 0.215)` bị planner từ chối vì nằm
  ngoài global costmap (`mx=46`, trong khi `size_x=45`);
- goal `0.30 m` phía sau tại `(0.597, 0.214)` được NavFn báo `SUCCEEDED`, nhưng
  path thực tế kết thúc tại khoảng `(0.347, 0.064)`, lệch goal yêu cầu khoảng
  `0.329 m`.

Đây là bằng chứng trực tiếp rằng `GridBased.tolerance=0.5 m` đang cho phép
planner thay đích bằng một cell khác quá xa khi goal nằm sát biên/vùng cost
cao. Không dùng kết quả `SUCCEEDED` của planner như bằng chứng xe sẽ tới đúng
goal. Candidate sau khi hoàn thành bag baseline phải giảm tolerance về cỡ một
cell đến goal tolerance (`0.05-0.08 m`) và chạy lại dry-run trước khi chạy xe.

### Rosbag goal 0,50 m phía sau lúc 23:12-23:16 ngày 2026-09-16

Sau khi đưa xe ra giữa map, dry-run planner kết thúc đúng goal
`(0.0847, -0.0838)` rồi mới cho phép chạy. Bag:
`/tmp/fire_robot_nav2_diag_20260916_YkiCLu/behind_goal_motion`.

Kết quả action:

- initial heading error: `180.00 deg`;
- `SUCCEEDED` sau `35.95 s`, `0 recovery`;
- sai số cuối: `0.0527 m`, `7.98 deg`, nằm trong tolerance
  `0.08 m / 8.59 deg`;
- lệnh chỉ-quay tích lũy `32.10 s`, lệnh tiến chỉ `3.80 s`;
- phải chờ `23.02 s` mới có lệnh tiến đầu tiên;
- lệnh quay đổi dấu 5 lần.

Bằng chứng drift trong pha lệnh chỉ-quay:

- trước lệnh tiến đầu tiên, `/cmd_vel_raw` và `/cmd_vel` đều giữ `Vx=0`;
- odom vẫn báo dịch tịnh tiến ròng `0.111 m`, quãng đường tích lũy `0.202 m`,
  `|Vx|` cực đại `0.021 m/s`;
- pose map từ feedback dịch ròng `0.098 m`; odom và localization cùng xác nhận
  xe không quay quanh một tâm cố định;
- yaw mới đổi khoảng `123.42 deg` thì path tangent đổi hướng và controller bắt
  đầu xen kẽ quay/chạy, thay vì hoàn tất một lần quay gần 180 độ.

Bằng chứng planner/costmap không ổn định:

- có `67` global plan trong action, tương ứng replan xấp xỉ `2 Hz`;
- chiều dài path dao động `0.093-1.087 m`, trong khi goal ban đầu chỉ cách
  khoảng `0.55 m`;
- độ lệch ngang lớn nhất của path so với dây cung là `0.179 m`;
- khoảng giây 20-21, path tangent nhảy từ khoảng `-17 deg` sang `+77 deg`;
  RPP vì thế đảo chiều quay, đây không phải RPP tự chọn goal khác;
- global costmap đổi trung vị `277` cell/frame, cực đại `659` cell/frame;
- đường thẳng robot-goal không chứa cell inscribed/lethal, nhưng cost dọc đường
  dao động `124-173`; costmap/inflation vẫn đủ mạnh để thay gradient NavFn.

Bằng chứng loại trừ collision giả trong lượt này:

- `187` scan frame, khoảng cách nhỏ nhất tới tâm `base_link` là `0.410 m`;
- `0` scan frame có điểm trong bán kính `0.17 m` hoặc vùng cận `0.20 m`;
- `0/60` local-costmap frame có inscribed/lethal cell trong footprint;
- `0 recovery`, không có collision abort.

Command chain không cắt lệnh tiến: `cmd_vel_nav`, `cmd_vel_raw` và `cmd_vel`
đều đạt `0.10 m/s`. RPP có lúc yêu cầu `1.0 rad/s`, nhưng sau smoother và
SafetyGate cực đại quan sát được là `0.702 rad/s`; odom/IMU cùng đo cực đại
xấp xỉ `0.80 rad/s`. Trong phần lớn pha quay, yaw thực tế thấp hơn nhiều do
tải skid-steer và drift tịnh tiến.

`SimpleGoalChecker` chỉ kiểm tra pose, không kiểm tra robot đã dừng. Ngay sau
`SUCCEEDED`, `/cmd_vel` còn lệnh quay tới `0.417 rad/s` trong khoảng `0.147 s`;
odom tiếp tục quay thêm `5.28 deg` trong 5 giây sau result, dù chỉ dịch
`0.002 m`. Điều này giải thích trường hợp UI báo reached nhưng thân xe vẫn còn
quay/đầu xe vượt khỏi yaw tolerance sau đó. Candidate cần A/B là
`nav2_controller::StoppedGoalChecker` với ngưỡng vận tốc dừng phù hợp robot
chậm; không chỉ tiếp tục nới yaw tolerance.

Kết luận của bag: RPP đã thực hiện đúng việc bám path và kết thúc đúng goal.
Vấn đề ưu tiên là quay tại chỗ không thuần túy, làm start pose/costmap đổi; sau
đó NavFn/Dijkstra replan sang các tiếp tuyến rất khác nhau. Tăng replan lên
5-10 Hz sẽ khuếch đại đổi path. MPPI cũng không thể sửa drift quay hoặc costmap
đầu vào; giữ RPP làm baseline cho tới khi sửa hai nguồn này.

## 3. Vì sao path có đoạn cong và xe chỉnh lái giật

Các nguyên nhân đã xác nhận từ source:

1. NavFn/Dijkstra tạo path theo gradient costmap; nó không được thiết kế để
   luôn cho đường thẳng trong vùng trống.
2. `SimpleSmoother` chưa được gọi trong BT, nên RPP bám path thô.
3. Planner tính lại đường ở 2 Hz. Điểm bắt đầu path và gradient costmap thay
   đổi sau mỗi 0,5 giây có thể làm carrot của RPP dịch chuyển.
4. RPP hiện chỉ có ngưỡng quay `10 deg`, không có cặp ngưỡng engage/disengage
   riêng. Path mới có thể làm sai số góc dao động quanh ngưỡng và tạo chuyển
   đổi quay/chạy liên tục.
5. `planner.tolerance=0.5 m` quá lớn so với goal tolerance `0.08 m`; nó không
   được xem là nguyên nhân duy nhất của đoạn cong nhưng cần giảm trong candidate
   tiếp theo.

Không thu nhỏ collision envelope, inflation hoặc tắt collision detection để
che triệu chứng. Nếu scan tự phản xạ lên thân robot, phải lọc đúng hình học
vùng thân xe.

## 4. Hành vi điều khiển mục tiêu

Sai số khi chạy phải tính với **tiếp tuyến path phía trước**, không phải luôn
với đường thẳng nối robot tới goal.

```text
Nhan goal
  |
  +-- lech huong path lon: Vx=0, quay tai cho
  |                         thoat khi sai so < 5-8 deg
  |
  +-- bam path:
  |     <= 15 deg : chay binh thuong
  |      15-30 deg: giam Vx, hieu chinh mem
  |      > 30 deg : neu keo dai 0.3-0.5 s thi dung va can lai huong
  |
  +-- gap vat can: dung an toan, cap nhat costmap/replan
  |
  +-- vao 0.08 m: dung tien, can yaw <= 0.15 rad, sau do reached
```

Cần hysteresis: ví dụ kích hoạt quay tại `25-30 deg`, chỉ giao lại controller
khi còn `5-8 deg`. Dùng cùng một ngưỡng nhỏ để bật và tắt dễ gây rung mode.

## 5. Planner và tần số replanning

`2 Hz` nghĩa là BT yêu cầu global path mới tối đa mỗi `0.5 s`. Đây đã là mức
hợp lý với Lidar khoảng `5.2 Hz` và global costmap `2 Hz`.

Không nên tăng planner lên 5-10 Hz ở trạng thái hiện tại vì:

- global costmap chỉ đổi ở 2 Hz, nên nhiều lần plan sẽ dùng lại cùng dữ liệu;
- thay path liên tục làm điểm carrot/controller dao động mạnh hơn;
- không chữa được collision giả ở local costmap;
- tăng CPU và tăng khả năng controller loop 10 Hz bị miss.

Nếu muốn phản ứng vật cản nhanh hơn, thứ tự đúng là:

1. Xác nhận `/scan` và local/global costmap không có vật cản giả.
2. Giữ local costmap 5 Hz để dừng/né gần robot.
3. Giữ global replan 2 Hz làm baseline.
4. Chỉ A/B `3 Hz` sau khi bag chứng minh global path chậm hơn costmap; không
   tăng thẳng lên 5-10 Hz.

Planner candidate cho vùng trống:

- `ThetaStarPlanner`: ưu tiên đoạn thẳng, phù hợp mong muốn hiện tại.
- `SmacPlanner2D`: A* cost-aware, có smoother nội bộ và cân bằng khoảng cách
  vật cản.
- NavFn: giữ làm baseline để so sánh, nhưng thường tạo đường cong rộng hơn.

## 6. RPP hay MPPI

### RPP

Ưu điểm:

- nhẹ, đơn giản, dễ giải thích;
- bám path tốt khi path sạch và costmap đúng;
- collision prediction rõ ràng.

Hạn chế trong robot hiện tại:

- chủ yếu bám global path, không phải local trajectory optimizer mạnh;
- khi thấy collision thường dừng/abort để BT replan hoặc recovery;
- một ngưỡng quay `10 deg` không tạo được hysteresis mong muốn;
- nhạy với path gấp khúc và path đổi ở 2 Hz.

### MPPI

Ưu điểm:

- lấy mẫu và chấm điểm nhiều quỹ đạo dự đoán;
- có critic cho obstacle cost, path align/follow, goal và goal angle;
- phù hợp hơn cho chuyển động mượt và né vật cản động trên laptop.

Hạn chế:

- nhiều tham số hơn và cần rosbag/A-B test;
- không thể sửa scan, footprint hoặc costmap sai;
- chất lượng vẫn bị giới hạn bởi odom/IMU 10 Hz và ma sát skid-steer.

### Quyết định hiện tại

Không chuyển production sang MPPI trước khi xác định nguồn `collision ahead`.
Giữ RPP làm baseline có thể tái hiện. Sau khi costmap sạch, candidate khuyên
dùng để A/B là:

```text
Theta* hoặc Smac2D -> SmoothPath -> Rotation Shim -> MPPI
```

Rotation Shim xử lý pha quay đầu có hysteresis; MPPI chỉ nhận điều khiển sau
khi xe đã gần đúng hướng và xử lý bám đường/né vật cản.

## 7. Hành vi với vật cản động

1. Local safety/controller dừng trước va chạm.
2. Chờ ngắn `0.5-1.0 s` để phân biệt vật đi ngang.
3. Global costmap và planner cập nhật đường vòng ở 2 Hz.
4. Nếu có hành lang đủ rộng, controller bám quỹ đạo vòng.
5. Nếu không có đường sau khoảng `3-5 s`, tiếp tục đứng yên hoặc abort an toàn.
6. Không Spin khi footprint simulation đã báo collision.
7. Không tự chạy lùi khi chưa có xác nhận vùng sau robot an toàn.

## 8. Rosbag bắt buộc cho bài test tiếp theo

Ghi từ lúc xe đứng yên ít nhất 5 giây, sau đó gửi một goal phía sau ở vùng
trống, để chạy tới `SUCCEEDED/FAILED`, rồi giữ thêm 5 giây trước khi dừng bag.

Các topic tối thiểu:

```text
/scan
/odom
/imu/data
/tf
/tf_static
/amcl_pose
/particle_cloud
/plan
/cmd_vel_nav
/cmd_vel_raw
/cmd_vel
/local_costmap/costmap
/local_costmap/costmap_raw
/local_costmap/published_footprint
/global_costmap/costmap
/global_costmap/costmap_raw
/lookahead_point
/curvature_lookahead_point
/navigate_to_pose/_action/status
/navigate_to_pose/_action/feedback
/navigate_to_pose/_action/result
```

Không dùng `ros2 bag record -a`: camera hoặc topic nặng không liên quan có thể
làm thay đổi chính bài test đang đo.

Bag phải trả lời được:

- scan gần nhất tới `base_link` là bao nhiêu và có nằm sát footprint không;
- cell nào của local costmap làm RPP/Spin báo collision;
- `/cmd_vel_nav` có thực sự đạt `Wz=1.0 rad/s` hay bị cắt về zero liên tục;
- SafetyGate có truyền đúng `/cmd_vel_raw -> /cmd_vel` không;
- yaw odom/IMU đáp ứng bao nhiêu so với command;
- AMCL có dịch pose khi robot chỉ quay tại chỗ không;
- path mới ở 2 Hz thay đổi hình học bao nhiêu.

## 9. Quy tắc an toàn khi tuning

- Pump/relay luôn OFF.
- Không chạy teleop đồng thời với Nav2.
- Luôn có người đứng cạnh nút Cancel/cắt nguồn.
- Chỉ test obstacle cao tới mặt phẳng quét Lidar.
- Không vô hiệu hóa SafetyGate, watchdog hoặc collision checking.
- Không thêm recovery chạy lùi khi chưa có cảm biến xác nhận vùng sau.

## 10. Contract test của baseline hiện hành

Sau khi RPP đã được thử trên xe thật và được giữ làm baseline, test tham số
được cập nhật để kiểm tra đúng plugin RPP, giới hạn tốc độ, no-reverse và
collision detection. Đây chỉ là đồng bộ kiểm thử với cấu hình đang chạy, không
đổi controller hay tham số motor. Kế hoạch A/B tiếp theo nằm ở
`NAV2_FUTURE_WORK.md`.

## 11. Rosbag goal xa + UART ngày 2026-09-17

### 11.1. Phạm vi và kết quả

Bag tạm thời:

```text
/tmp/fire_robot_nav2_far_goal_20260917_qILvLU/rosbag
/tmp/fire_robot_nav2_far_goal_20260917_qILvLU/uart_debug.log
```

Goal thử nghiệm được chọn theo pose/map hiện tại, gần vùng operator đánh dấu:

```text
requested: x=0.06 m, y=-1.15 m, yaw=-90 deg
NavFn endpoint: x=0.01 m, y=-1.10 m
```

Kết quả:

- `SUCCEEDED` sau `32.59 s`;
- feedback cuối: còn `0.010 m`, yaw AMCL khoảng `-96.5 deg`;
- `6 recoveries`, tập trung từ giây `27.65` tới `28.61`;
- RPP đạt tối đa `Vx=0.10 m/s`, `Wz=0.719 rad/s`; sau velocity smoother/
  SafetyGate đạt `Wz=1.0 rad/s`;
- AMCL đi ròng `1.011 m`, đổi yaw `-90.2 deg`;
- odom firmware đi ròng `1.094 m`, đổi yaw `-98.3 deg`.

### 11.2. Đường cong không do RPP tự tạo

Trong action có `58` global plan, tương đương `1.78 Hz`, đúng với baseline
planner `2 Hz`.

- path đầu dài `1.265 m`, lệch ngang tối đa `0.284 m`;
- cực đại quan sát được là `0.288 m`;
- tiếp tuyến đầu của các plan đầu khoảng `-160..-170 deg`, trong khi hướng
  thẳng tới endpoint chỉ khoảng `-93 deg`;
- đường thẳng robot-goal có cost `253` trong toàn bộ `41/41` costmap frame;
- chính cell goal yêu cầu cũng có cost `253`, nên NavFn dùng
  `tolerance=0.5 m` để chọn endpoint khác cách khoảng `0.071 m`;
- global costmap đổi trung vị `351` cell/frame, cực đại `820` cell/frame.

Do đó RPP đang bám một global path vòng hợp lệ theo costmap. Không tăng planner
lên 5-10 Hz: đầu vào chỉ cập nhật khoảng 2 Hz và đang đổi mạnh; tăng replanning
sẽ làm carrot/path đổi thường xuyên hơn. Candidate sau khi costmap sạch là giảm
`GridBased.tolerance` về `0.05-0.08 m` và A/B Theta*/Smac2D + SmoothPath.

### 11.3. Sáu recovery là collision thật trong costmap

Timeline behavior tree/rosout:

```text
27.62 s  RPP collision ahead
27.76 s  RPP collision ahead
27.96 s  RPP collision ahead
28.16 s  RPP collision ahead
28.18 s  recovery Spin bắt đầu
28.58 s  Spin hủy: Collision Ahead
28.61 s  Wait bắt đầu
30.61 s  Wait thành công
32.56 s  controller reached goal
```

Tại giây 27-28.5:

- local costmap có `7-10` cell cost `253` bên trong footprint tròn `0.17 m`
  đang dùng tại thời điểm ghi bag;
- LiDAR có điểm gần nhất cách tâm `base_link` khoảng `0.188-0.201 m`, ở phía
  trước-trái, xấp xỉ `(x=0.16 m, y=0.10 m)`;
- sau Wait, footprint không còn cell `253`; robot mới quay tiếp và reached.

Collision checker hoạt động đúng. Trước test tiếp theo phải loại trừ chân
operator, dây/cơ cấu lọt vào mặt phẳng quét hoặc vật thật trong khoảng 20 cm.
Không thu nhỏ footprint, inflation hoặc tắt collision detection để che điểm
LiDAR gần này.

### 11.4. UART xác nhận lỗi quay ở tầng bánh xe

UART debug 5 Hz ghi `Tgt/Act/PWM/encoder` cho bốn bánh. Khi đi thẳng, tỷ lệ tốc
độ thực/mục tiêu trung bình khá đều:

```text
FL 1.01, RL 1.03, FR 1.09, RR 1.11
```

Khi quay tại chỗ, độ lệch tăng mạnh:

```text
FL median 1.14, có 5 mẫu stall
RL median 1.33
FR median 1.00, có 4 mẫu stall
RR median 2.00, có 3 mẫu stall và 2 mẫu sai dấu
```

Ở đoạn chỉnh yaw cuối, target bánh trái chỉ khoảng `-0.7 rad/s`: FL nhiều mẫu
gần `0`, còn RL đạt `-1.8..-2.4 rad/s`; hai bánh phải khoảng `+1.2..+1.6
rad/s`. Đây là vùng tốc độ quá thấp và không cân bằng cho ma sát skid-steer.

Phát hiện mismatch bắt buộc xử lý trước khi tuning Nav2 sâu hơn:

```text
URDF + tài liệu đo xe thật: TRACK_WIDTH = 0.180 m
firmware RobotConfig.h:     TRACK_WIDTH_M = 0.086 m
```

Firmware dùng giá trị này cho cả target bánh và `wz_encoder`. Git history cho
thấy `0.086` có từ lần thêm cấu hình đầu tiên, chưa có bằng chứng hiệu chuẩn
360 độ. Với `0.086 m`, lệnh quay `0.55 rad/s` chỉ tạo target khoảng `0.7
rad/s/bánh`, đúng vùng bánh bị stall/overshoot.

Không sửa âm thầm vì đổi track width sẽ tăng đáng kể target bánh khi quay.
Bước đúng là xác nhận kích thước/effective track, build/flash có kiểm soát rồi
test quay tại chỗ ở `0.3`, `0.5`, `0.8 rad/s` với auto-stop và UART.

### 11.5. Quyết định về tần số

Giữ `/odom` và `/imu/data` ở `10 Hz` trong baseline hiện tại:

- `Task_Motion` trên S3 đã chạy `50 Hz`;
- packet `STATE` thật sự chỉ phát `10 Hz`; tăng publisher ROS mà không tăng
  firmware chỉ lặp dữ liệu cũ;
- controller Nav2 chạy `10 Hz` và xe chỉ chạy `0.10 m/s`;
- dữ liệu chỉ ra lỗi hình học/ma sát bánh khi quay, không chỉ ra thiếu sample.

Chỉ A/B telemetry `20 Hz` sau khi sửa quay tại chỗ. Khi đó phải đổi chu kỳ
`STATE` firmware từ 100 ms xuống 50 ms, kiểm tra serial drop/read failure,
state age và soak ít nhất 10 phút trước khi tăng controller frequency.

## 12. Hình học đo lại và footprint polygon ngày 2026-09-17

Số đo được user xác nhận:

```text
wheel width:       0.027 m (cả bốn bánh)
outer-to-outer:    0.200 m (trục trước và sau)
inner-to-inner:    0.146 m (suy ra từ hai bánh rộng 0.027 m)
mechanical track:  0.173 m = 0.200 - 0.027
wheelbase:         0.135 m (cả bên trái và bên phải)
wheel radius:      giữ 0.034 m
chassis:           giữ 0.255 x 0.150 m
```

`TRACK_WIDTH_M=0.173` là seed cơ khí cho firmware, chưa phải kết quả hiệu chuẩn
track hiệu dụng. Skid-steer còn trượt ngang, nên phải chạy quay CW/CCW có UART
và góc ground truth trước khi xem giá trị này là final.

Nav2 không còn dùng hình tròn `robot_radius=0.17 m`. Bao lồi vật lý được suy ra:

- chassis: `X=+/-0.1275`, `Y=+/-0.0750 m`;
- tâm bánh: `X=+/-0.0675`, `Y=+/-0.0865 m`;
- biên bánh: `X=+/-0.1015`, `Y=+/-0.1000 m`.

Local và global costmap dùng cùng footprint bát giác:

```text
[[ 0.1275,  0.0750], [ 0.1015,  0.1000],
 [-0.1015,  0.1000], [-0.1275,  0.0750],
 [-0.1275, -0.0750], [-0.1015, -0.1000],
 [ 0.1015, -0.1000], [ 0.1275, -0.0750]]
```

`footprint_padding=0.015 m` tạo biên an toàn 15 mm. So với hình tròn cũ, vùng
giữ chỗ danh nghĩa ở hai hông giảm từ 170 mm xuống khoảng 115 mm sau padding,
nhưng không làm nhỏ hơn bao vật lý đã đo. Inflation và collision detection giữ
nguyên.

Kết quả verify source candidate:

- Xacro parse: PASS.
- Geometry invariant: PASS; 8 đỉnh, local/global giống nhau, không còn
  `robot_radius`, padding đúng `0.015 m`.
- ROS build `fire_robot_description` + `fire_robot_navigation`: PASS.
- Launch tests với `ROS_LOG_DIR=/tmp`: 3/3 PASS.
- Nav2 invariants liên quan (bỏ allowlist map đã biết lệch): 5/5 PASS.
- Firmware native tests: 28/28 PASS.
- Firmware ESP32-S3 build: PASS; RAM 10.2%, flash 5.9%.

Ở thời điểm candidate geometry, full `colcon test` còn đỏ vì test params lịch
sử bắt Rotation Shim+DWB, allowlist map chưa khớp và xmllint không tải được
schema mạng. Đây là **ghi nhận lịch sử**, không phải trạng thái test hiện nay;
xem kết quả verify của commit chốt baseline và lộ trình `NAV2_FUTURE_WORK.md`.

### 12.1. Kết quả flash và quay độc lập

Firmware geometry đã flash thành công. Dữ liệu nằm tại:

```text
docs/codex/artifacts/fire_robot_geometry_cal_20260917_bDzJAN/
```

Rosbag đo được:

- thẳng `0.08 m/s`: `0.2610 m/3.000 s`, yaw `+1.89 deg`, `dy=+0.0040 m`;
- CCW `+0.5 rad/s`: `+57.58 deg`, gyro trung bình `+0.3444 rad/s`;
- CW `-0.5 rad/s`: `-59.99 deg`, gyro trung bình `-0.3664 rad/s`.

Hai chiều quay lệch độ lớn `2.41 deg` (`4.1%`) và kết thúc ở khoảng
`-0.5 deg`. Geometry mới không có lỗi dấu/scale lớn, nhưng chưa đủ để chốt
effective track vì odom yaw fuse IMU với `alpha=0.995` và chưa có ground truth
360 độ.

UART cho thấy lỗi riêng ở wheel tracking khi chạy lùi. Với target magnitude
`1.3 rad/s`, trung bình CCW là
`FL/RL/FR/RR=-0.708/-2.246/+1.769/+1.754`; CW là
`+1.492/+1.462/-1.077/-2.354 rad/s`. Bánh trước chạy lùi yếu trong khi bánh
sau cùng phía overshoot. Phải xử lý cơ khí/encoder/FF/PID ở một pass riêng;
không đổi footprint, track, planner Hz hoặc controller plugin để che hiện tượng
này.

SafetyGate giữ mẫu nonzero cuối tới timeout 1 giây nếu zero BEST_EFFORT đơn lẻ
bị mất. Script hiệu chuẩn phải phát zero burst hữu hạn sau mỗi pha.

## 13. Goal thẳng kiểm chứng sau geometry — 2026-09-17

Goal thẳng `0.40 m` sau khi chuyển sang track `0.173 m`, wheelbase `0.135 m`
và footprint polygon đã `SUCCEEDED` trong `4.85 s`, recovery `0`. Không có lỗi
collision, progress, TF hoặc controller loop. Odom tiến `0.334 m`, đổi yaw
`-3.38 deg`; feedback cuối còn `0.0373 m`.

Dry-run trước đó đã chặn candidate `0.70 m` do footprint có 6 cell cost `253`.
Do đó kết quả goal thật không phải đánh đổi bằng cách bỏ qua collision hoặc thu
nhỏ footprint.

Planner không cho đường thẳng tuyệt đối: path đầu dài `0.4423 m` so với direct
`0.3897 m`, lệch ngang cực đại `0.0582 m`. RPP bám path này với
`vx <= 0.10 m/s`, `wz=-0.370..+0.358 rad/s` và có 4 lần đổi dấu correction
đáng kể. Khi nhìn xe chỉnh trái-phải, phải kiểm tra `/plan` trước khi quy toàn
bộ cho controller.

Các `FollowPath` status `6` trong bag trùng với thời điểm replan và log
`Passing new path to controller`; goal con cuối status `4`, còn outer
`NavigateToPose` liên tục chạy rồi `SUCCEEDED`. Đây là thay subordinate goal
khi cập nhật path, không phải recovery/abort của toàn navigation task.

UART cho thấy median `actual/target` khi `|target| >= 1 rad/s` là
`FL/RL/FR/RR=1.091/1.106/1.035/1.032`; sai số lớn chủ yếu tại lúc target đổi
nhanh giữa tiến và xoay. Planner/path curvature và wheel transient là hai biến
độc lập cần A/B riêng.

AMCL chỉ có 10 sample trong bag dài 121 giây. Sample gần kết thúc còn sai số
XY `0.0822 m`, nên không thay thế được action feedback cuối `0.0373 m` khi
đánh giá goal checker. Yaw sample cuối lệch khoảng `5.3 deg`, nằm trong
`yaw_goal_tolerance=8.59 deg`.

Không tăng planner Hz chỉ để làm thẳng path, không tăng odom/IMU Hz để che
wheel transient, và không chuyển MPPI trước khi có baseline goal phía sau cùng
geometry này. Không thay FF/PID trong cùng pass. Chi tiết và bag/UART:

```text
docs/codex/artifacts/fire_robot_nav2_forward_geometry_20260917_lR8Kqs/
```

Bài tiếp theo phải tiếp tục dùng dry-run footprint, timeout hữu hạn, zero burst
và chỉ được gửi goal sau khi operator xác nhận đang đứng cạnh xe và pump OFF.

## 14. Goal phía sau kiểm chứng sau geometry — 2026-09-17

Goal cách khoảng `0.50 m` đúng phía sau, yaw theo hướng di chuyển mới, đã
`SUCCEEDED` trong `21.270 s`, recovery `0`. Sai số cuối `0.0786 m / 3.39 deg`
nằm trong tolerance `0.08 m / 8.59 deg`. Không có collision, progress, TF hoặc
controller-loop warning trong action. Pump OFF toàn phiên; cuối bài command và
actual bốn bánh đều zero.

Kết quả này tốt hơn baseline phía sau trước geometry, nhưng chuyển động chưa
mượt:

- RPP phát rotate-only `17.0 s`, drive `4.2 s`;
- xe chờ `8.521 s` mới tiến, khi đó yaw vẫn lệch `25.32 deg` so với dây cung
  tới goal;
- sau đó có khoảng tám burst tiến xen kẽ quay tại chỗ và `wz` đổi dấu đáng kể
  chín lần;
- cross-track RMS/max là `0.0629/0.1068 m`;
- trong các mẫu tiến, heading error median `18.66 deg`, p95 `47.16 deg`.

### 14.1. Global path là nguồn gây đổi hướng

Planner tạo `41` path ở `1.888 Hz`. Path đầu dài `0.4990 m` so với direct
`0.4772 m`, lệch ngang `0.0473 m`; trong toàn action lệch ngang cực đại đạt
`0.1727 m`. Tiếp tuyến đầu nhảy hơn `10 deg` trong `12/40` lần replan, cực đại
`77.63 deg`. Ngay sau burst tiến đầu, tangent đổi `-7.4 -> +40.1 -> +69.4
deg`, nên RPP dừng tiến và đổi chiều quay để bám path mới.

Direct corridor không có cell unknown/inscribed/lethal. Cost inflation trên
đường thẳng dao động `0..165`; có lúc path lệch `0.173 m` chỉ để giảm max cost
từ `107` xuống `106`. Đây là bằng chứng NavFn/potential field cùng path chưa
smooth, không phải obstacle thật bắt buộc phải né. `SimpleSmoother` đã được
cấu hình nhưng BT active vẫn đưa thẳng output `ComputePathToPose` sang
`FollowPath`.

Không tăng planner Hz: `1.888 Hz` đã sát cấu hình và chính mỗi path mới đang
làm tangent/carrot thay đổi. Candidate A/B ít biến nhất là thêm action
`SmoothPath` vào BT, giữ nguyên planner, RPP và toàn bộ rate, rồi lặp cùng goal
và so sánh tangent jump, rotate/drive phases, `wz` sign changes, cross-track.

### 14.2. Rotational gain vật lý còn thấp

UART trong pha quay đầu `8.4 s` đo:

```text
mean |cmd_wz|:    0.620 rad/s
cmd_wz integral: -300.44 deg
IMU/odom yaw:     -155.84 deg
odom-center drift: 0.036 m
```

Wheel median `abs(actual/target)` trong pha đó là
`FL/RL/FR/RR=1.053/0.842/0.842/1.333`. Các bánh có phản hồi, gần như không
stall, nhưng thân xe quay chỉ khoảng 52% tích phân lệnh do skid/trượt và bất
cân bằng bánh. Đây là biến riêng với global path. Sau A/B smoothing, cần một
pass CW/CCW riêng ở đúng dải `wz` Nav2 để đo effective rotational gain theo
tốc độ; không đổi FF/PID, geometry và planner trong cùng pass.

Chưa chuyển MPPI: MPPI không làm thẳng global path đầu vào và không sửa được
skid vật lý. Artifact chi tiết:

```text
docs/codex/artifacts/fire_robot_nav2_behind_geometry_20260917_xvUEMv/
```
