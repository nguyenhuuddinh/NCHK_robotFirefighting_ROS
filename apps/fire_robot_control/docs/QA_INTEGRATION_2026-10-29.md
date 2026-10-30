# QA và gói tích hợp Fire Robot Control — 29/10/2026

> **Hiện hành:** [bản sửa popup map 69dfe](MAP_SELECTION_FIX_2026-10-29.md)
> đã QA và sync Pi/source413/installed103. App chọn được map cũ ở IDLE mà
> không tạo lại danh sách mỗi STATUS. Các mốc 750008 bên dưới là lịch sử.

> **Cập nhật sau QA:** đã đồng bộ Pi qua LAN cho packet `750008b09dfe…`, build
> hai package, source 413/413 và file cài Pi 103/103 khớp hash; receipt đã lưu.
> [Báo cáo sync hiện hành](PI_STATIC_SYNC_2026-10-29.md). Chưa runtime robot.
> Các trạng thái “chưa SSH/sync”, “PENDING” trong phần QA bên dưới ghi lại
> thời điểm trước bước sync này, không ghi đè kết quả mới.

> **Đã sửa F1–F3 của QA trước Pi:** gói `750008b09dfe…`; 19 ca hồi quy sửa lỗi,
> 124 unittest, 17 tích hợp và 1 target CTest PASS. Source/binary/archive và
> static laptop 95 file PASS. [Báo cáo sửa lỗi hiện hành](QA_FIXES_BEFORE_PI_2026-10-29.md).
> Sau đó đã đồng bộ Pi theo báo cáo bên trên; nghiệm thu phần cứng còn thiếu.

## Phạm vi đã làm

- Giao diện Qt/RViz native có camera trong cùng cửa sổ. Giao diện điều khiển
  dùng `services/supervisor.py` độc lập; mở app không tự khởi động robot.
- Quét map tạo phiên R5 Pi + SLAM, mở khung bàn phím teleop; Lưu map kiểm tra
  phiên SLAM, tạo YAML/PGM trong staging và không ghi đè. Start dùng map chọn
  đã được xác thực và sao chép riêng cho phiên.
- Chữa cháy lấy `PI_PAIRING` từ chính phiên Pi, kiểm packet/binding, gọi runner
  laptop với RViz ở app và operator hiện có ở chế độ không mở camera OpenCV
  riêng. `SetInitialPose` chỉ bật ở giai đoạn POSE; `SetGoal` cần report
  operator mới cho biết đã armed và không có hold/cycle.
- X đi tới operator; Q/Esc/đóng hoặc mất heartbeat yêu cầu STOP. Supervisor giữ
  PTY Pi trong khi laptop drain; nếu không có chứng cứ dừng/cleanup, phiên sau
  bị chặn. Các nguồn `/pump_cmd`, watchdog và node điều khiển gốc không bị đổi.
- Đóng gói bản ứng viên từ source R5 `0c54…` sang packet mới; hai bản source
  R5 đã niêm phong được kiểm hash và giữ nguyên. Không dùng receipt Pi cũ cho
  packet ứng viên mới.

## Bằng chứng QA offline

- **124 test Python PASS**: state machine, bấm lặp/sai thứ tự, save trùng và
  lỗi/gián đoạn, map thay đổi, pairing sai, supervisor, receipt, watchdog UI,
  PTY/process giả, gói source và sync giả. Trong đó có ba bài dùng SSH/SCP giả;
  chúng không kiểm kết nối Pi thật.
- **1 target CTest Qt PASS**: heartbeat trên UI thread, X/Q, mất kết nối, FD
  riêng, JSON lỗi/trùng khóa và dừng ở thời điểm kết nối.
- **17 case tích hợp riêng PASS:** chín case socket → node operator R5 với bus
  giả; hai case client Qt → supervisor → preflight; ba case cửa sổ RViz thật
  ở domain 211 localhost (gồm luồng quét/lưu/map mới/chữa cháy bằng supervisor
  giả và khóa công cụ Initial Pose/Goal); một case kiểm tham số trên node ROS
  giả localhost; một case mở/đóng binary ứng viên đã đóng gói; một case chạy
  entrypoint runner được sinh ra với toàn bộ I/O giả. Các nhóm này không phải
  test robot thật.
- Static inventory laptop của gói ứng viên: 95 source/config đã đối chiếu,
  `STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED`. Không gộp kết quả này thành PASS
  của phần cứng hoặc toàn bộ 86 ca trong kế hoạch QA gốc.

Khi kiểm thử đã tìm và sửa: heartbeat bị từ chối do lấy timestamp trước `recv`,
callback map của `rclpy` Humble chỉ nhận message, teleop cần nhận Ctrl-C dưới
dạng byte trước khi phải escalated signal, map cần snapshot khi start, và
phiên thất bại ngay trước SSH không được hiển thị là đã xác nhận robot dừng.
Quá trình này cũng bổ sung khóa phiên laptop chung, kiểm lại SLAM khi lưu và
giữ Pi nếu process local chưa chứng minh đã thoát.

## Chưa được xác nhận

- Chưa SSH/NTP, đồng bộ Pi, chạy SLAM/Nav2/YOLO, camera thật, motor, servo,
  bơm hoặc thử STOP vật lý trong lượt này. Prompt SSH/sudo, luồng pairing và
  cleanup từ runner thật qua WiFi vẫn cần thử trên Pi.
- Receipt Pi cho packet ứng viên **PENDING**; chưa nghiệm thu TF, QoS thực tế,
  tốc độ/độ trễ camera, phục hồi kết nối, load CPU hay cách người dùng vận hành
  trong điều kiện thật. Đã giữ QoS/logic watchdog gốc; các sai khác lịch sử của
  repo không tự sửa trong phạm vi này.
- Observer giữ nhận dạng publisher trong phiên và báo dữ liệu cũ/không hợp lệ;
  trường hợp publisher đổi danh tính ngoài luồng phục hồi đã xử lý vẫn cần
  kiểm thực tế. Không coi ảnh, trạng thái giả hoặc process exit là STOP proof.
- Khi cleanup hoặc STOP chưa được xác nhận, `pending.json` được giữ để chặn
  mở phiên mới. Cần xem receipt/log và kiểm tra robot trước khi xử lý phục hồi;
  không xóa marker chỉ để bỏ qua lỗi.

## Tệp và cách mở

Gói hiện tại: `artifacts/<packet-trong-current.txt>`; `build_receipt.json` ghi
hash binary/archive và `runtime_verified=false`. Các gói cũ hơn trong
`artifacts/` là bản trung gian, không dùng để sync hoặc chạy.

Mở preview giả: `bash apps/fire_robot_control/launchers/start_preview.sh`.
Mở ứng viên bằng `launchers/fire-robot-control.desktop` hoặc:

```bash
app='/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/apps/fire_robot_control'
packet=$(cat "$app/artifacts/current.txt")
bash "$app/launchers/start_candidate.sh" "$app/artifacts/$packet"
```

Preview và ứng viên là hai launcher khác nhau. Bản ứng viên mở ở trạng thái chờ;
nếu kiểm tra laptop đạt nhưng chưa có receipt Pi, app báo lỗi trước khi mở SSH.

## Bước tiếp theo với Pi

Đây là lệnh cho bước đồng bộ **khi người dùng sẵn sàng thực hiện**; chưa chạy
trong đợt QA này. Pi cần kết nối LAN `10.42.0.185`, baseline đúng và không có
tiến trình robot cũ. Script chỉ chuyển source, build và kiểm hash tĩnh; không
launch ROS, NTP hay flash firmware.

```bash
app='/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/apps/fire_robot_control'
packet=$(cat "$app/artifacts/current.txt")
bash "$app/launchers/sync_pi_candidate.sh" "$app/artifacts/$packet" 10.42.0.185
```

Cần giữ `LOG_SYNC` và thấy `PI_STATIC_RECEIPT_VALID=1`. Sau đó rút LAN, dùng
WiFi profile hiện tại (Pi `10.0.0.1`, laptop `10.0.0.253`) và kiểm thử có giám
sát từng bước: quét/lưu map, dừng, chọn map, khởi động, Initial Pose, X, Q và
đối chiếu STOP/cleanup với tình trạng thật. App có thể hỏi mật khẩu Pi cho
SSH/sudo qua giao diện; prompt này chưa được kiểm thử với Pi thật.

## Tái chạy QA offline

```bash
cd '/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS'
bash apps/fire_robot_control/launchers/build_preview.sh
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD/apps/fire_robot_control:${PYTHONPATH:-}" /usr/bin/python3 -m unittest discover -s apps/fire_robot_control/tests -q
ctest --test-dir /tmp/fire_robot_control_preview_$(id -u) --output-on-failure
PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/releases/r5_20261029/ros/ros2_ws/src/fire_robot_bringup:${PYTHONPATH:-}" /usr/bin/python3 -m pytest -q -p no:cacheprovider apps/fire_robot_control/tests/qa_full_operator.py apps/fire_robot_control/tests/qa_supervisor_qt.py apps/fire_robot_control/tests/qa_supervisor_gui.py apps/fire_robot_control/tests/qa_candidate_smoke.py apps/fire_robot_control/tests/qa_candidate_runner.py apps/fire_robot_control/tests/qa_operator_ros_args.py
```

Qt/PTY QA cần quyền tạo Unix socket và PTY cục bộ; RViz GUI QA cần màn hình X11.
Không dùng các bài fake làm bằng chứng cho vận hành thật.
