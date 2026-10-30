# Phục hồi phiên chữa cháy và hiển thị kết quả kiểm tra — 29/10/2026

User đã bấm Kiểm tra lại; GUI có `UI_RECHECK_REQUEST_SENT` nhưng vẫn BLOCKED.
Phiên cũ `c3213541-6851-4136-a191-968b92801d67` dùng `R5_AUTOMATIC`, trong khi
hàm phục hồi chỉ nhận `DESKTOP_MAPPING`, nên trả `RECOVERY_MAPPING_ONLY` ngay.
Audit có thể xong trước STATUS 100 ms; nếu chỉ nhìn nhãn nút thì khó nhận ra
lần bấm được xử lý. Khẳng định trước đó rằng chỉ cần kiểm tra lại để mở khóa
chưa bao phủ phiên chữa cháy.

## Thay đổi

- `services/recovery.py`: hỗ trợ đúng hai profile DESKTOP_MAPPING/R5_AUTOMATIC.
  Giữ đủ kiểm tra STOP, binding phiên/packet, process group/child sạch, không
  force/crash, PID/PGID/SID không còn sống, khóa runner và kiểm tra mới trên Pi.
  Loại profile lạ vẫn bị chặn. Thông báo lỗi dùng mã và giải thích tiếng Việt,
  không phản chiếu nội dung tùy ý từ SSH/file hoặc dữ liệu xác thực.
- `services/supervisor.py`: STATUS có recovery_attempt/recovery_result; kết quả
  CHECKING/SUCCEEDED/FAILED/CANCELLED giữ đến lần kiểm tra tiếp theo hoặc khi
  người dùng mở phiên mới. Bấm lặp khi đang kiểm tra không tạo worker khác.
  STOP/CLOSE, lỗi archive hoặc thiếu bằng chứng vẫn giữ BLOCKED/pending.
- `native_rviz/session_ui.cpp`: hiển thị “Kiểm tra lần N: ĐANG KIỂM TRA /
  THÀNH CÔNG / CHƯA ĐẠT / ĐÃ HỦY”, cùng lý do. Log GUI ghi attempt/result,
  kể cả khi worker xong quá nhanh để GUI nhìn thấy CHECKING. Nút giữ chuột
  qua STATUS tiếp tục hoạt động, không tái tạo lỗi mất click cũ.

Chỉ ba file production thay đổi so với cc4a3574. Không thay node/runner ROS,
TF, firmware, điều kiện startup, STOP/watchdog hoặc threshold. Lỗi khởi động
SEAL_PI cũ giữ nguyên trong receipt và có thể tái diễn. Phục hồi chỉ mở quyền
bắt đầu phiên mới sau kiểm tra; không tự chạy lại quét/chữa cháy.

## QA

- Regression receipt chữa cháy thật trên artifact cc4a3574: 1 ERROR do
  RECOVERY_MAPPING_ONLY. Cùng test trên bản mới PASS.
- Python unittest: 146 PASS / 5,037 s; gồm profile chữa cháy, STOP thiếu,
  local/group/child không sạch, Nav child crash/thiếu, forced group, profile
  lạ, marker thay đổi, Pi đổi boot/mất kết nối, lock bận, lỗi quyền/archive,
  callback cũ, bấm liên tục, STOP/CLOSE trong audit, kết quả thất bại nhanh.
- Qt/RViz/IPC: 42 PASS / 7,45 s, localhost domain211, tiến trình giả; ba ca
  mới kiểm lý do thất bại hiển thị, số lần giữ qua STATUS và kết quả thành công.
- CTest native IPC: 1 target PASS / 0,41 s.
- Smoke binary đóng gói: 1 PASS / 2,29 s; tổng 189 ca Python/Qt/smoke, cộng 1 target CTest.
- Lượt Python/build đầu trong sandbox bị chặn Unix socket (EPERM), không phải
  lỗi code; đã chạy lại ngoài sandbox đúng phạm vi QA và đạt như trên.

Lệnh verify:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/releases/r5_20261029/ros/ros2_ws/src/fire_robot_bringup:${PYTHONPATH:-}"
/usr/bin/python3 -m unittest discover -s apps/fire_robot_control/tests -p 'test_*.py'
cmake --build /tmp/fire_robot_pre_sync_ui --parallel 2
/usr/bin/python3 -m pytest -q -p no:cacheprovider apps/fire_robot_control/tests/qa_pre_sync_ui.py apps/fire_robot_control/tests/qa_map_selection.py apps/fire_robot_control/tests/qa_button_feedback.py apps/fire_robot_control/tests/qa_supervisor_gui.py apps/fire_robot_control/tests/qa_supervisor_qt.py
/usr/bin/python3 -m pytest -q -p no:cacheprovider apps/fire_robot_control/tests/qa_candidate_smoke.py
```

## Bằng chứng phiên cũ và giới hạn

Đã chạy riêng audit chỉ đọc với laptop/Pi thật qua LAN trên phiên
c3213541: PASS, Pi processes_absent=true, binding/boot/cleanup đúng. Kết quả
ở `qa_evidence/fire_retry_20261029/actual_read_only_audit.json`. Pending giữ
nguyên SHA256 `89a80dc576beb164dbf16fc9c74d95173e9964334adfe19bd3e95c74ef5defa6`.
Không gọi archive trong lần audit độc lập, không khởi động ROS, không gửi
lệnh actuator hoặc xác nhận STOP vật lý mới.

Hai receipt không chứa trường xác thực đã lưu trong failed_fire/ để regression
không phụ thuộc trạng thái robot ở những lần QA sau. Log khởi động lỗi gốc vẫn
ở ~/.local/state/fire_robot_control/session_156cf4387fab4e109fb0707cc7420e96.

## Gói và cập nhật

Packet `6734c2846c465773d1a4e6c96b9ca3f0ed193d06c07cc013bc0353c1f7b2ae83`,
416 source pins, được chọn trong artifacts/current.txt. Receipt build, binary /
archive SHA và kết quả sync được lưu cùng thư mục bằng chứng. Các gói cũ và
bộ lệnh cũ trên Pi được giữ nguyên. Chưa nghiệm thu quét/chữa cháy phần cứng
bằng bản sửa này.

Sau mở bản mới, user bấm Kiểm tra lại một lần. Nếu đạt, app về IDLE, ghi
THÀNH CÔNG và chuyển pending vào lịch sử. User chọn Quét map hoặc chọn map
rồi Khởi động chữa cháy bằng thao tác riêng. Nếu không đạt, lý do vẫn hiện
cạnh CHƯA ĐẠT để xử lý rồi kiểm tra lại.

Đồng bộ LAN RC0: hai package build22,3 s, source416/installed103 hash PASS,
PI_STATIC_HASH_PASS=1, PI_STATIC_RECEIPT_VALID=1. Log sync:
`/home/huudinh/.local/state/fire_robot_control/sync_logs/sync_ATwZOE`.
Pi stage `/home/pi/fire_robot_desktop/6734c2846c46/ros`, receipt đã lưu theo
packet trên laptop. Receipt build runtime_verified=false vẫn nguyên; receipt
Pi static là bằng chứng riêng, không thay thế runtime.

Đã mở app6734 sau khi xác minh supervisor.lock rảnh và không có tiến trình
robot. GUI log `/tmp/fire-ui-jgr9h5mq/gui.log`, app bắt đầu BLOCKED, pending
nguyên SHA như trên. Chưa có thao tác Kiểm tra lại của user trên binary mới;
audit độc lập đã PASS nhưng chưa gọi archive trên phiên thật. App không tự
chạy robot. Không commit/push hoặc sửa ai/ trong lượt này.
