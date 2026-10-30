# Phản hồi khi bấm nút — 29/10/2026

User phản ánh bấm nút nhưng khó biết đã nhận thao tác, không thấy nút lún.
Nguyên nhân: stylesheet tự vẽ border QPushButton làm mất phần vẽ nút native,
nhưng chưa khai báo trạng thái `:pressed`/`:hover`/focus.

Bản mới `c93a42c964d98528b0db017a536f45b6b898fc8253431c9c5119e3547ce8a910`:

- Nút thường có viền dưới tạo cảm giác nổi; khi giữ chuột, nền tối hơn, viền
  trên và nội dung dịch xuống tạo cảm giác lún. Hover/focus dùng màu đỏ PTIT.
- Bấm nhanh hoặc Space: tô màu ngắn 180 ms bằng QTimer, không chặn event loop.
- Gửi Quét map/Fire/Kiểm tra lại/Lưu: phản hồi ngay bằng nhãn chờ và thông báo
  dưới cửa sổ. Nút thao tác được khóa trong lúc chờ trạng thái phù hợp.
- STATUS cập nhật nhãn Đang mở quét map/Đang khởi động/Đang lưu map/Đang dừng
  phiên. Nhãn trở lại bình thường theo trạng thái server. X giữ cooldown cũ.
- Phản hồi “Đã gửi yêu cầu” chỉ xác nhận gửi local, không thay thế kết quả
  session/receipt hoặc xác nhận STOP. Hiệu ứng không phát thêm lệnh robot.

## Phạm vi source

So với artifact 1c55: thêm `native_rviz/button_style.hpp`, sửa `main.cpp`,
`session_ui.cpp`, `session_ui.hpp`. Không có file Python/runner, launch, node,
topic/QoS, TF/watchdog, firmware hoặc model nào khác. Manifest mới 416 pins.
Parent R5 vẫn khớp 364/364 hash; bộ lệnh cũ và các artifact cũ giữ nguyên.

Ảnh dưới là render widget Qt riêng để kiểm tra các trạng thái; không phải
ảnh đang điều khiển robot:

![Các trạng thái nút Qt](qa_evidence/button_feedback_20261029/button_feedback.png)

## QA

38 ca Qt/IPC trước đóng gói PASS / 7,04 s:

- 10 ca input/render mới: giữ chuột thay đổi pixel, không chạy action trước
  thả, thả chuột chỉ một action, kích thước giữ nguyên, click nhanh có màu,
  màu tự hết, event loop vẫn chạy, nút disabled bỏ qua chuột/Space, Space chỉ
  tạo một action.
- 4 ca phản hồi mới trong SessionUi: nhãn gửi tức thì, nhãn STOP tức thì,
  nhãn đang lưu và nhãn đang dừng theo STATUS.
- 16 ca SessionUi cũ: khóa Pose/Goal sau STOP/mất IPC, receipt cũ, retry.
- 3 ca popup chọn map bằng chuột, 3 ca GUI supervisor giả, 2 ca Qt client với
  preflight từ chối localhost trước SSH.

Binary đóng gói: thêm 1 PASS / 2,22 s mở/đóng IDLE với state tạm; tổng 39 ca.
CTest native: 1 target PASS / 0,41 s, kiểm IPC.
Tất cả QA GUI/ROS dùng dữ liệu giả hoặc localhost domain 211, không khởi động
robot. Bộ unittest Python 141 ca đã PASS ở bản 1c55, không chạy lại toàn bộ
vì không đổi backend. Không tuyên bố đã phủ mọi thao tác phần cứng.

Lệnh build/verify chính:

```bash
source /opt/ros/humble/setup.bash
cmake -S apps/fire_robot_control/tests/pre_sync_ui_review -B /tmp/fire_robot_pre_sync_ui -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/fire_robot_pre_sync_ui --parallel 2
bash apps/fire_robot_control/launchers/build_preview.sh
export PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/releases/r5_20261029/ros/ros2_ws/src/fire_robot_bringup:${PYTHONPATH:-}"
/usr/bin/python3 -m pytest -q -p no:cacheprovider apps/fire_robot_control/tests/qa_button_feedback.py apps/fire_robot_control/tests/qa_pre_sync_ui.py apps/fire_robot_control/tests/qa_map_selection.py apps/fire_robot_control/tests/qa_supervisor_gui.py apps/fire_robot_control/tests/qa_supervisor_qt.py
```

Render QA đã xem trực tiếp: viền/nền/độ lún khác nhau rõ, cùng tông trắng–đỏ–xám.

## Đồng bộ và mở lại app

- Packet: `c93a42c964d98528b0db017a536f45b6b898fc8253431c9c5119e3547ce8a910`.
- Binary SHA: `676906bc07cd46433eec96b385493297e441d7d60d4bfa7c3711b354caf548cc`.
- Archive SHA: `bfa8c18505fd851e9fcce8d82bdb3cbca5b96a3e634d0963cbc91e1a0c8c9259`.
- Pi stage: `/home/pi/fire_robot_desktop/c93a42c964d9/ros`.
- Sync log: `/home/huudinh/.local/state/fire_robot_control/sync_logs/sync_cCDck3`.

Script sync LAN RC0, build 2 package / 22,4 s, source416 và installed103
khớp hash; `PI_STATIC_HASH_PASS=1`, `PI_STATIC_RECEIPT_VALID=1`. Receipt đúng
packet đã lưu để app kiểm tra. Không chạy robot/NTP/flash.

Đã xác minh app cũ đúng PID140218/packet1c55/state của user, không có robot
process trên laptop và có receipt phiên lỗi đã kết thúc. Đóng qua SIGTERM
chuẩn của supervisor, RC2 do BLOCKED là dự kiến. Không xóa pending, SHA vẫn
`dc2f488644d4240cd843414f72b0841c20eb29802fc5b98c194e8ace46a0e318`.
App mới đã mở ở BLOCKED, GUI log `/tmp/fire-ui-wt6pydr7/gui.log`, để người dùng
thử nút Kiểm tra lại. Chưa bấm quét lại hoặc điều khiển phần cứng.

Bằng chứng build/test/sync, ảnh Qt, receipt và SHA256SUMS nằm trong
`qa_evidence/button_feedback_20261029/`.
