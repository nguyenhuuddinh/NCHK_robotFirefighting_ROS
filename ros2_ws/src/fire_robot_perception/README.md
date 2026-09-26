# fire_robot_perception

Package YOLO chạy trên laptop. Giai đoạn hiện tại là **observe-only**:

- Subscribe `/image_raw/compressed` (`sensor_msgs/CompressedImage`).
- Publish `/yolo/detections` (`std_msgs/String`, JSON).
- Publish `/yolo/image_annotated/compressed`
  (`sensor_msgs/CompressedImage`) khi có subscriber.
- Publish `/yolo/alignment_preview` (`std_msgs/String`, JSON) từ node căn hướng
  observe-only; trường vận tốc góc chỉ là giá trị đề xuất để đánh giá.
- Không tạo publisher điều khiển chuyển động hoặc cơ cấu chữa cháy.

## Môi trường GPU

Venv hiện tại:

```bash
source /opt/ros/humble/setup.bash
source /home/huudinh/.venvs/fire_robot_yolo/bin/activate
```

Nếu cần tạo lại venv, dùng Python 3.10 với `--system-site-packages`, sau đó cài
`requirements-gpu.txt`. Không cài Torch/Ultralytics lên Raspberry Pi.

## Build

Dùng Python của venv để console script được tạo với đúng interpreter:

```bash
cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/ros2_ws"
source /opt/ros/humble/setup.bash
/home/huudinh/.venvs/fire_robot_yolo/bin/python -m colcon build \
  --symlink-install --packages-select fire_robot_perception
source install/setup.bash
```

Kiểm tra dòng shebang:

```bash
head -n 1 install/fire_robot_perception/lib/fire_robot_perception/yolo_observer
```

Kết quả phải trỏ tới
`/home/huudinh/.venvs/fire_robot_yolo/bin/python`, không phải
`/usr/bin/python3`.

## Chạy với camera Pi

Pi phát camera trước. Trên laptop kết nối WiFi của Pi:

```bash
cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY

ros2 launch fire_robot_perception perception.launch.py \
  use_sim_time:=false \
  model_path:="/media/huudinh/New Volume/esp32/Fire_smoke_person_detection/best_final.pt"
```

Node dùng Best Effort, Volatile, depth 1 và không giữ queue ảnh riêng. Baseline
model là `best_final.pt`, chạy observe-only inference FP16 trên GPU, tắt CLAHE,
confidence 0,25 và xác nhận lửa khi có ít nhất 3/5 frame. Model final phải qua
lại các bài exact-center, target chuyển động và hard-negative trước khi được
phép tham gia handoff điều khiển. Trạng thái xác nhận chỉ là telemetry.

## Quan sát

```bash
ros2 node info /yolo_observer
ros2 topic info /image_raw/compressed --verbose
ros2 topic info /yolo/detections --verbose
ros2 topic hz /yolo/detections
ros2 topic echo /yolo/detections --once
```

Mở ảnh annotated:

```bash
XDG_CONFIG_HOME=/tmp/fire_robot_rqt_yolo \
ros2 run rqt_image_view rqt_image_view -v \
  /yolo/image_annotated/compressed
```

Nếu dùng RViz Image display, chọn:

```text
Topic: /yolo/image_annotated/compressed
History: Keep Last
Depth: 1
Reliability: Best Effort
Durability: Volatile
```

Ảnh annotated chỉ được JPEG-encode khi có subscriber và bị giới hạn 10 FPS để
không tốn CPU/băng thông vô ích.

Overlay observe-only gồm:

- dấu cộng màu cyan tại tâm camera;
- khung cyan biểu diễn vùng chết `aim_deadband_x/y`;
- dấu thoi vàng tại tâm lửa trung vị khi đã xác nhận 3/5 frame;
- dòng trạng thái `aim`, sai số chuẩn hóa, confidence, tuổi frame và processing
  latency;
- dòng `filter=EMA` cho biết tâm vàng đang dùng EMA, mặc định alpha `0,60`.

`aim=CENTERED` chỉ là kết quả phân loại trên ảnh. Nó không phải lệnh quay xe.

Xác nhận có lửa và lọc vị trí là hai bước độc lập: vote `3/5` vẫn quyết định
target có hợp lệ hay không, còn `center_filter` chỉ làm mượt tọa độ. Mặc định
dùng `ema` với `ema_alpha: 0.60` để bám nhanh hơn median 5 frame. Khi frame hiện
tại không có lửa, output lập tức là `NO_TARGET` và EMA bị reset; lần bắt lại sẽ
khởi tạo từ candidate mới, không kéo theo tọa độ cũ. Có thể đặt
`center_filter: "median"` trong `config/yolo_params.yaml` để chạy đối chứng.

## Căn hướng observe-only

`perception.launch.py` còn chạy `fire_alignment_observer`. Node này chỉ đọc
`/yolo/detections` và publish JSON telemetry trên `/yolo/alignment_preview`;
không publish message điều khiển robot.

Luồng tính preview ngang:

```text
fire confirmed + detected ở frame hiện tại
  -> smoothed_center.x
  -> deadband + hysteresis
  -> P controller liên tục ở biên deadband
  -> giới hạn tốc độ góc
  -> giới hạn tốc độ thay đổi (slew-rate)
  -> /yolo/alignment_preview
```

Thông số mặc định trong `config/yolo_params.yaml`:

```yaml
deadband_enter_x: 0.10
deadband_exit_x: 0.13
proportional_gain: 1.0
max_angular_speed_rad_s: 0.50
max_angular_acceleration_rad_s2: 0.80
max_source_age_ms: 300.0
input_timeout_s: 0.50
```

Quy ước ROS `base_link`: `angular_z > 0` là quay trái. Vì vậy target bên trái
ảnh tạo preview dương, target bên phải tạo preview âm. Khi target vào
`deadband_enter_x`, preview về 0 ngay; hysteresis giữ trạng thái CENTERED cho
tới khi target vượt `deadband_exit_x`. Khi mất target, inference lỗi, ảnh quá
cũ hoặc telemetry timeout, preview về 0 và controller reset ngay.

Kiểm tra topic:

```bash
ros2 node info /fire_alignment_observer
ros2 topic info /yolo/alignment_preview --verbose
ros2 topic hz /yolo/alignment_preview
ros2 topic echo /yolo/alignment_preview --once
```

Chạy report trong lúc đưa hình lửa trái/phải rồi giữ ở center:

```bash
ros2 run fire_robot_perception alignment_preview_diagnostics \
  --seconds 30 \
  --progress-period 1 \
  --output /tmp/fire_alignment_preview.json
```

`target_wz` là đầu ra P-controller sau giới hạn tốc độ; `preview_wz` là giá trị
sau slew-rate. Cả hai chỉ là telemetry. Chưa được nối topic này vào bộ điều
khiển xe, Nav2, `/cmd_vel`, `/fire_target` hoặc `/pump_cmd`.

Report còn phải có các bộ đếm dưới đây bằng 0:

```text
observe_only_violations
safety_contract_violations
direction_sign_violations
stop_nonzero_violations
speed_limit_violations
```

## JSON detection

Mỗi message có timestamp/frame gốc, tuổi frame, thời gian inference, danh sách
box và trạng thái xác nhận lửa. Tâm box được đưa ra theo pixel và chuẩn hóa
`[-1, 1]`, nhưng node chưa publish target điều khiển.

Quy ước tâm chuẩn hóa:

```text
x < 0: trái      x = 0: giữa      x > 0: phải
y < 0: trên      y = 0: giữa      y > 0: dưới
```

Các trạng thái có thể gặp:

- `ok`: frame được decode và inference thành công.
- `decode_error`: JPEG không giải mã được; evidence cũ bị xóa.
- `inference_error`: model/GPU lỗi; evidence cũ bị xóa.
- `camera_stale`: không có frame quá timeout; evidence cũ bị xóa.

## Diagnostic tâm lửa

CLI dưới đây chỉ subscribe `/yolo/detections`, không có publisher. Đặt ảnh lửa
trên điện thoại tại một vị trí cố định trong 30 giây rồi chạy:

```bash
ros2 run fire_robot_perception yolo_target_diagnostics \
  --seconds 30 \
  --output /tmp/yolo_target_center.json
```

Report chứa tỷ lệ phát hiện/xác nhận, confidence, độ rung tâm, phân bố trạng
thái `LEFT/RIGHT/UP/DOWN/CENTERED`, FPS telemetry và latency. Nên tạo report
riêng cho `center`, `left`, `right`, `up`, `down` và bài che/mở target.

Không dùng `/fire_target` để thử bước này vì Pi đang chuyển topic đó xuống
ESP32 như một lệnh actuator.

## Test offline

```bash
cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
/home/huudinh/.venvs/fire_robot_yolo/bin/python -m colcon test \
  --packages-select fire_robot_perception
/home/huudinh/.venvs/fire_robot_yolo/bin/python -m colcon test-result \
  --verbose --test-result-base build/fire_robot_perception
```
