# Đo TF riêng, bảo toàn R5

Bộ chẩn đoán này nằm ngoài services/native_rviz, không nằm trong packet app
hiện tại và không sửa R5. Dữ liệu là fixture giả trên một host, **không phải
nghiệm thu robot hoặc chứng minh nguyên nhân duy nhất của lỗi WiFi thật**.

## Bảo vệ phạm vi

- Chỉ chấp nhận source R5 packet0c54, kiểm364 hash trước/sau và ở từng worker.
- Worker bắt buộc ROS_DOMAIN_ID=213, ROS_LOCALHOST_ONLY=1; từ chối domain0.
  Đây chỉ là biến trong tiến trình chẩn đoán, không đổi cấu hình R5/mạng.
- Không mở serial/camera/Lidar hoặc chạy launch/runner/watchdog/mission R5.
  Dữ liệu odom/scan/identity/OFF/cmd-zero do fixture tạo trong domain213.
  Tên node/topic mô phỏng contract để thử class health thật; không phải
  telemetry firmware thật. Không có publisher pump/target/navigation goal.
- Phát hiện R5/robot đang chạy bằng snapshot tiến trình: từ chối trước chạy,
  kiểm lại mỗi0,5s và kết thúc fixture nếu R5 bắt đầu. Không chiếm khóa R5.
  Công cụ vẫn tiêu thụ CPU khi chạy; không chạy song song với R5.
- Chỉ gửi tín hiệu tới subprocess do chính bộ đo tạo, không pkill/kill ROS
  khác. Hủy bằng Ctrl+C hoặc SIGTERM sẽ chờ thu dọn worker.
- Không ghi source/overlay/install hay file khóa phiên. Không ghi bytecode
  vào module R5, không đổi artifacts/current.txt hoặc receipt vận hành.
- Output phải là thư mục mới nằm ngoài source R5 đã chọn; không ghi đè run.
  Dùng /tmp hoặc /home/pi/fire_robot_diagnostics để giữ tách biệt.

## Các điểm đo

1. Class OdomToTfBroadcaster nguyên byte của R5, dữ liệu odom giả10Hz,
   TF20Hz/lead100ms. Ghi source age ngay trước sendTransform trong process
   riêng; gọi method gốc giữ nguyên payload.
2. Class StartupHealthNode nguyên byte của R5, rolepi. Chỉ bọc set_transform
   trong RAM của process chẩn đoán để ghi lúc callback nhận, rồi gọi method
   thật. Không monkeypatch node đang vận hành hoặc sửa file R5.
3. Ghi tuổi TF mà sample()/lookup thực nhìn thấy, thời gian spin/sample/
   ghi health JSON. Listener độc lập xử lý TF liên tục làm đối chứng.

`callback` giữ đúng vòng R5: rclpy.spin_once một callback rồi sample/ghi file.
`timer` là biến thể **chỉ ở process chẩn đoán**: vẫn spin liên tục nhưng gọi
sample/ghi file khoảng5Hz. Không phải bản sửa đã triển khai cho R5.

`readers=early` tạo thêm reader TF/scan/actuator nhỏ, **không phải toàn bộ
RViz/Qt hoặc đường WiFi laptop–Pi**. `delay-ms` thêm xử lý giả có kiểm soát
vào worker health; chỉ dùng positive control, không suy ra Pi thật có đúng
độ trễ đó. Nhãn injected_delay_ms được ghi trong mỗi kết quả.

TF stamp lặp lại khi TF20Hz/odom10Hz, nên không ghép stamp như ID duy nhất
để tuyên bố latency từng message. Công cụ báo tuổi nguồn ở mỗi điểm và phân
vị; loại2s đầu discovery khỏi thống kê, giữ đầy đủ event raw.

## Chạy trên laptop, không kết nối robot

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B apps/fire_robot_control/diagnostics/tf_startup/run_probe.py \
  --source releases/r5_20261029/ros --output /tmp/tf_diag_new_run \
  --duration 10 --matrix
```

Ba case: callback không thêm reader; callback thêm reader; sample5Hz thêm
reader. Có thể chọn một case bằng --sampling callback|timer --readers none|early.
Duration5..25s/case, tối đa3case; worker chờ discovery và thu dọn có giới hạn.
Có worker lỗi/không đủ dữ liệu/trace tràn thì báo fail, không kết luận PASS.

Kiểm dương tính bộ đo, dùng output mới:

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B apps/fire_robot_control/diagnostics/tf_startup/run_probe.py \
  --source releases/r5_20261029/ros --output /tmp/tf_diag_positive_control \
  --duration 8 --matrix --delay-ms 25
```

Pure guard/unit QA (không ROS/network):

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B -m unittest discover \
  -s apps/fire_robot_control/diagnostics/tf_startup -p 'test_*.py'
```

Trên Pi chỉ dùng bản copy tool vào fire_robot_diagnostics và source stage R5
có packet0c54; source Humble/baseline/overlay cũ như bình thường. Không copy
đè vào R5. Phép synthetic Pi vẫn chỉ localhost, không phải test robot.

## Đọc kết quả

Mỗi case có fixture/fast/health.json (event raw), observer_health.json,
summary.json, log riêng. Root có result.json, r5_unchanged và SHA256SUMS.

- Nguồn/fast listener mới, health callback cũ: thấy backlog/lịch xử lý tại
  đường health trong fixture. Chưa tự quy kết nguyên nhân phần cứng.
- Nguồn đã cũ: cần xem publisher/odom/tải trước listener.
- Cả hai listener cùng cũ nhưng nguồn mới: cần đo DDS/CPU nhận.
- Callback mới nhưng lookup cũ: cần rà buffer/chuỗi frame/thời điểm sample.

So cùng điều kiện và duration; không dùng số ca offline thay runtime PASS.
Lượt đo nguồn thật sau này cần runner/packet chẩn đoán riêng có kiểm chứng,
không gắn vào R5 đang hoạt động mà mặc định không có overhead.
