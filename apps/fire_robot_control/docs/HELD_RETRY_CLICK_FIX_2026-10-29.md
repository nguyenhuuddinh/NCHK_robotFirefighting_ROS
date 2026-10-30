# Sửa mất thao tác Kiểm tra lại khi giữ chuột — 29/10/2026

User bấm Kiểm tra lại rồi nút vẫn giữ nguyên, không thấy kết quả. Pending
phiên `6f9e9217-1f7f-4561-beb8-2fa58a95cf61` vẫn nguyên và không có archive
thành công. Bản trước không lưu các lần audit thất bại nên không thể kết luận
từ log cũ rằng đúng lần bấm của user đã tới backend hay chưa.

Đã tái hiện được lỗi có thể hủy lần bấm **trước khi gửi**:

1. STATUS tới khoảng mỗi 100 ms.
2. Trong BLOCKED, `SessionUi::update()` gọi `lockCommands()`, tạm disable
   Scan rồi enable lại vì nút này đang đóng vai Kiểm tra lại.
3. Nếu người dùng đang giữ chuột, Qt hủy trạng thái pressed khi nút bị
   disable. Thả chuột không phát `clicked`, không gửi RECHECK.

Bộ QA trước dùng `QPushButton::click()` tức thì nên bỏ sót. Hiệu ứng pressed
đã sửa trong c93 không giải quyết việc nút bị toggle giữa chừng.

## Thay đổi

Chỉ sửa `native_rviz/session_ui.cpp/.hpp` so với c93:

- `lockCommands(bool disableScan=true)` giữ nguyên cách khóa mặc định khi
  STOP, mất IPC hoặc đóng app. Riêng STATUS BLOCKED không toggle nút retry.
- Pose/Goal và các nút điều khiển khác vẫn khóa. Quyết định enable cuối cùng
  vẫn xét closing/disconnected. CHECKING vẫn khóa nút retry.
- GUI log ghi `UI_RECHECK_REQUEST_SENT` khi gửi thành công và
  `UI_SESSION_STATUS phase=... stage=... error_present=...` khi nhận thay đổi.
  Không log argument, password, pairing hoặc ảnh camera.

Không đổi Python/backend, audit/STOP proof, runner/launch/node ROS, TF,
watchdog, topic/QoS, model hoặc firmware. Không xóa pending để vượt khóa.

## QA

- Trước sửa: regression giữ chuột qua 4 STATUS rồi thả **1 FAIL**, không có
  RECHECK. Bằng chứng `fire_robot_held_retry_before.log`.
- Sau sửa: **39 PASS / 7,44 s** gồm đúng ca trên, các ca feedback/popup,
  STOP/lost IPC/Pose/Goal và supervisor/GUI giả.
- Phép audit độc lập dùng đúng dịch vụ của artifact c93 và receipt/Pi thật:
  PASS chỉ đọc, lỗi TF cũ giữ `PI_SEAL_HEALTH_TIMEOUT`. Không archive marker
  thật và không tạo phiên robot.
- Binary mới: thêm 1 PASS / 2,24 s mở/đóng IDLE ở state tạm; tổng 40 ca.
  CTest native: 1 target PASS / 0,41 s, kiểm IPC.

Regression mới nằm trong `tests/pre_sync_ui_review/main.cpp` dùng
`QTest::mousePress`, nhận bốn STATUS, rồi `QTest::mouseRelease`. Socket đích
là server giả; không gửi yêu cầu tới robot. ROS test localhost domain 211.

```bash
source /opt/ros/humble/setup.bash
cmake -S apps/fire_robot_control/tests/pre_sync_ui_review -B /tmp/fire_robot_pre_sync_ui -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/fire_robot_pre_sync_ui --parallel 2
export PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/releases/r5_20261029/ros/ros2_ws/src/fire_robot_bringup:${PYTHONPATH:-}"
/usr/bin/python3 -m pytest -q -p no:cacheprovider apps/fire_robot_control/tests/qa_pre_sync_ui.py -k held_retry_survives_status_refresh
```

## Gói

- Packet: `cc4a3574a0d24ea47c6be5ef51d803120279ca99d65d364ebce10e4de9f4b72e`, 416 pins.
- Binary: `2394144da0833abe9ab29fc01fba2bac8b9b965b262c18cbb5d45a19f733fde8`.
- Archive: `ffa844080737b04603d4912f86d2c7b8f491a4ee8c32089f2d55e4a38fda042b`.

Các artifact cũ giữ nguyên. Người dùng bấm Kiểm tra lại để audit và về IDLE;
bản thân việc kiểm tra không khởi động robot. Sau đó vẫn phải bấm Quét map
riêng, dùng WiFi Pi/rút LAN theo profile R5 khi thực sự quét.

## Đồng bộ và xác minh thao tác thật của người dùng

Sync LAN RC0, build hai package 23,2 s, source416 và installed103 hash đúng;
`PI_STATIC_HASH_PASS=1`, `PI_STATIC_RECEIPT_VALID=1`. Log:
`/home/huudinh/.local/state/fire_robot_control/sync_logs/sync_rk3b3a`.
Pi stage `/home/pi/fire_robot_desktop/cc4a3574a0d2/ros`, receipt mới đã lưu ở
laptop theo packet để app kiểm tra. Không đổi bộ lệnh cũ.

Đóng chuẩn app c93 đã BLOCKED, xác minh PID143202 và không có tiến trình
robot trước khi gửi SIGTERM; RC2 của trạng thái BLOCKED là dự kiến. Mở bản
cc4 tại GUI log `/tmp/fire-ui-euqfnhgn/gui.log` và giữ nguyên marker.

Sau đó user xác nhận đã bấm lại. Đọc log thật thấy lần lượt:

```text
UI_RECHECK_REQUEST_SENT
UI_SESSION_STATUS phase=CHECKING stage=CHECKING error_present=0
UI_SESSION_STATUS phase=IDLE stage=IDLE error_present=0
```

Audit đã lưu tại
`~/.local/state/fire_robot_control/recoveries/051de08ce5ca4beaafcf1b62d1d1d860/`.
Pending gốc chuyển vào cùng folder, không còn pending ở root; receipt audit
ghi đúng phiên lỗi và Pi processes_absent=true. Kết quả này xác minh trọn
luồng người dùng bấm nút → IPC → audit laptop/Pi → archive → IDLE, **chỉ đọc
và phục hồi**, không phải quét map thành công hoặc nghiệm thu STOP vật lý.

Bằng chứng trước/sau sửa, test/build/sync, receipt, log thao tác thật và
SHA256SUMS: `qa_evidence/held_retry_20261029/`.
