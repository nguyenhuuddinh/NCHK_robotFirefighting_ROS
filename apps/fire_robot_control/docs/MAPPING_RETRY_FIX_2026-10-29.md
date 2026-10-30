# Cho quét lại sau phiên mapping lỗi — 29/10/2026

User yêu cầu chỉ sửa khả năng quét lại; không sửa TF/ESP32. Bản
`1c55c4db6f75d1ec96ef00518c03d1f73c992367be89757ac3c997a812e16e03`
đã build, QA và sync Pi qua LAN. Đây không phải bằng chứng quét map thành công.
Lỗi `PI_SEAL_HEALTH_TIMEOUT` của phiên 19:50:12 vẫn giữ nguyên.

## Hành vi

1. Khi phiên lỗi kết thúc ở BLOCKED, nút Quét map đổi thành **Kiểm tra lại**.
2. Nút này chạy kiểm tra chỉ đọc ở thread riêng: receipt STOP cũ, group/child
   cleanup, tiến trình và khóa chung trên laptop/Pi. UI vẫn gửi heartbeat.
3. Khi đang kiểm tra, Scan/Fire/Save/Select/Pose/Goal bị khóa. Bấm nhiều lần
   không tạo nhiều audit. STOP/đóng app hủy việc mở khóa; chờ audit kết thúc.
4. Đạt kiểm tra: ghi audit vào `~/.local/state/fire_robot_control/recoveries/`,
   chuyển nguyên file pending cũ vào lịch sử rồi về IDLE. Receipt và log lỗi
   gốc không bị sửa. Dòng STOP của phiên cũ được bỏ khỏi trạng thái phiên mới.
5. Người dùng phải bấm **Quét map** lần nữa để chạy phiên mới. Pairing mới và
   toàn bộ preflight/TF/STOP/watchdog R5 vẫn áp dụng.

Hiện phục hồi có kiểm chứng cho **DESKTOP_MAPPING** với STOP đã xác nhận và
cleanup từng group/child sạch. Thiếu receipt, chưa xác nhận STOP, forced
cleanup, còn tiến trình, reboot Pi làm đổi boot ID hoặc không tới được Pi:
giữ BLOCKED và cho bấm kiểm tra lại. Không xóa pending thủ công để ép chạy.
Việc rút/cắm ESP32 không thay thế kiểm tra cleanup hoặc đảm bảo hết lỗi TF.

## Ranh giới và thay đổi source

So với bản 69dfe, manifest chỉ khác:

- Thêm `services/recovery.py`, `services/recovery_probe.py`.
- Sửa `services/supervisor.py`, `native_rviz/session_ui.cpp`.

Không đổi runner R5/launch/config ROS, TF threshold, topic/QoS, watchdog,
firmware, model, mạng hay lệnh vận hành cũ. Hai parent R5 local vẫn khớp đủ
364/364 hash, packet `0c54d9b79765…`. App vẫn tự điều phối các launch/node cũ;
các tham số hiển thị RViz/camera đã được tích hợp từ các bản app trước.

Audit không import ROS, không gửi signal, không chạy node/NTP hoặc chạm
thiết bị. SSH chỉ dùng hai tuyến trực tiếp đã cấu hình: WiFi Pi hoặc LAN Pi,
không đi qua gateway tới router có cùng IP. Pairing secret chỉ được đọc nội
bộ trên Pi để đối chiếu; không trả về, không copy và không đặt trong argv.

Pending cũ có packet 69dfe vẫn được kiểm tra bằng receipt/binding gốc, dù app
mới có packet 1c55. Marker đổi trong lúc kiểm tra, kết quả quá 5 giây, lỗi
ghi audit hoặc lỗi archive đều không mở khóa. Audit không sửa chính sách
`SshPi.stop()` hay biến RC=2 cũ thành thành công.

## QA

| Nhóm | Kết quả | Phạm vi |
| --- | --- | --- |
| Python unittest | 141 PASS / 5,066 s | Gồm 17 ca mới; process/SSH giả, lỗi file, race marker, cancel/close, packet cũ |
| Qt/fault/popup | 26 PASS / 1,97 s | 4 ca mới cho retry; nút thật gửi một RECHECK, không SCAN/FIRE; công cụ vẫn khóa |
| Tích hợp operator/supervisor/runner | 16 PASS / 14,22 s | Localhost domain 211, I/O giả, không runner robot thật |
| Binary đóng gói | 1 PASS / 2,21 s | Mở IDLE/đóng với state tạm, không tự chạy robot |
| CTest native | 1 target PASS / 0,41 s | Build artifact |
| Receipt phiên thật + Pi LAN | PASS chỉ đọc | STOP receipt, group/child sạch, không còn tiến trình; giữ lỗi TF gốc |
| Controller với proof thật | PASS | State tạm đi BLOCKED→CHECKING→IDLE, archive thành công, factory cấm tạo phiên; pending thật giữ nguyên |
| Laptop static | 95 file PASS, errors=[] | Không nghiệm thu runtime |
| Pi static | 415 source + 103 installed PASS | Build hai package 22,4 s; rehash độc lập, baseline sạch |

Lệnh tái kiểm Python và GUI (GUI cần màn hình, socket/PTY localhost):

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/releases/r5_20261029/ros/ros2_ws/src/fire_robot_bringup:${PYTHONPATH:-}"
/usr/bin/python3 -m unittest discover -s apps/fire_robot_control/tests -p 'test_*.py'
cmake -S apps/fire_robot_control/tests/pre_sync_ui_review -B /tmp/fire_robot_pre_sync_ui -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/fire_robot_pre_sync_ui --parallel 2
/usr/bin/python3 -m pytest -q -p no:cacheprovider apps/fire_robot_control/tests/qa_pre_sync_ui.py apps/fire_robot_control/tests/qa_map_selection.py apps/fire_robot_control/tests/qa_pre_sync_faults.py
```

Lượt static laptop đầu dùng lệnh QA ghi đè PYTHONPATH nên thiếu
`ament_index_python`; đã sửa lệnh để giữ môi trường ROS và kiểm lại 95 PASS.
Không phải lỗi source/runtime. Không tính lần rerun thành ca mới.

## Gói và đồng bộ

- Packet: `1c55c4db6f75d1ec96ef00518c03d1f73c992367be89757ac3c997a812e16e03`.
- Binary SHA: `16680e64162f61ce0911fbe920a0479dbdc3edf135deb433603b431d375b95fa`.
- Archive SHA: `19de075d3c0158635af913b9c9563a8ea2d46ec13d528c9c083aefdd44685131`.
- Pi stage: `/home/pi/fire_robot_desktop/1c55c4db6f75/ros`.
- Sync log: `/home/huudinh/.local/state/fire_robot_control/sync_logs/sync_QOEcDb`.
- Receipt local: `~/.local/state/fire_robot_control/pi_receipts/1c55c4db6f75d1ec96ef00518c03d1f73c992367be89757ac3c997a812e16e03.json`.
- SHA receipt local/Pi cùng `65a69f86d7a063e8cd20eaffa5bf40a96e296a6ed7ab5ad1143c652e43a06dc5`.

Sync RC=0, `PI_STATIC_HASH_PASS=1`, `PI_STATIC_RECEIPT_VALID=1`. Các stage,
artifact và lệnh terminal cũ giữ nguyên. Receipt build giữ PENDING tại thời
điểm build; receipt Pi thực tế lưu riêng cho app kiểm tra.
Log build/test/sync, receipt và SHA256SUMS lưu tại
`qa_evidence/mapping_retry_20261029/`.

Đã mở app mới bằng launcher ứng viên, GUI log `/tmp/fire-ui-t9oeksn2/gui.log`.
Giữ nguyên pending phiên lỗi thật để người dùng bấm Kiểm tra lại. Chưa tự
bấm quét lại, chạy ROS/NTP, flash hay nghiệm thu dừng/motor/bơm vật lý.

## Người dùng thử

Kết nối Pi rồi bấm **Kiểm tra lại**, chờ nút về **Quét map**. Có thể kiểm tra
qua LAN; trước khi quét thật phải rút LAN, kết nối WiFi Pi theo profile R5.
Nếu cần rút/cắm ESP32, thực hiện khi phiên cũ đã kết thúc và xe đã dừng, rồi
bấm Quét map để tạo phiên mới. Nếu lỗi TF tái diễn, app vẫn giữ nguyên cơ chế
dừng và có thể yêu cầu kiểm tra lại; lượt sửa này không giải quyết TF.
