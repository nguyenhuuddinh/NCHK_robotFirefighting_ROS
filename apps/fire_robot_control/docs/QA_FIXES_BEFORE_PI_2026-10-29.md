# Sửa ba finding trước đồng bộ Pi — 29/10/2026

> **Bước sau đã hoàn tất:** [đồng bộ Pi 750008 qua LAN](PI_STATIC_SYNC_2026-10-29.md),
> receipt 103 file PASS. Các trạng thái PENDING/chưa SSH bên dưới là tại thời
> điểm sửa và QA; chưa có nghiệm thu runtime robot.

**F1, F2 và F3 đã sửa và qua QA offline.** Gói đang chọn là
`750008b09dfeed1835cfcd0b52849e0d26f0185910c4f9646b1bda7b9d6717cf`.
Chưa SSH, đồng bộ Pi, chạy NTP/robot hoặc flash. Receipt Pi của gói mới còn
PENDING; đây chưa phải nghiệm thu phần cứng.

## Thay đổi và kết quả

| Finding | Sửa trong source app | Kiểm chứng |
| --- | --- | --- |
| F1: kẹt khi file phiên lỗi | `services/supervisor.py`: chỉ nhận worker sau khi ghi marker thành công; bỏ worker chưa start nếu start thất bại. FINISHED luôn thu hồi worker trước khi xóa marker. Lỗi xóa marker báo BLOCKED, giữ marker, vẫn đóng được app. | Lỗi ENOSPC/EACCES, start RuntimeError/OSError, lỗi start kèm lỗi xóa marker và mở lại app có marker đều qua. Không có FINISHED/STOP proof giả để vượt gate. |
| F2: công cụ RViz còn mở sau STOP/lost | `native_rviz/session_ui.cpp/.hpp`: latch STOP/lost, kiểm cùng session và phase, dùng chung điều kiện khi thêm/chọn công cụ. Chỉ NEW_SESSION sau thao tác start mới hợp lệ mới bỏ latch STOP; mất IPC giữ khóa đến khi mở lại app. | POSE/Goal sau STOP, refresh/report đến muộn, STOP trước ack, NEW_SESSION không được yêu cầu, sai session, phase MAPPING và report sau mất IPC đều không mở công cụ. Hai lần start mới hợp lệ mở lại POSE được. |
| F3: proof cũ hiện trong phiên mới | Supervisor reset `last_stop` khi nhận phiên mới; UI chỉ hiện kết quả dừng ở IDLE/BLOCKED khi không có phiên hiện tại. Receipt/log cũ vẫn giữ nguyên. | Controller qua FINISHED → SCAN không mang proof cũ; label Qt không hiển thị proof cũ trong PREFLIGHT. |

Chỉ ba file production trên khác artifact e05. Source node/config ROS, firmware,
topic/message/QoS, watchdog và cách xác minh STOP của R5 không đổi. Hai parent
R5 local vẫn khớp 364/364 hash; artifact e05 cũng được giữ nguyên để tra cứu.
Không sửa trực tiếp gói cũ để tạo kết quả mới.

## QA đã chạy

| Nhóm | Kết quả | Phạm vi |
| --- | --- | --- |
| Unittest hiện có | 124 PASS, 5,233 s | Fake session/process, socket/PTY, map, pairing, STOP adapter, sync giả |
| Hồi quy trước Pi | 19 PASS, 0,58 s | 7 ca Python về lifecycle/proof + 12 trường hợp công cụ/label Qt; gồm 6 ca từng FAIL ở lần review trước |
| Tích hợp | 17 PASS | 16 ca nguồn/operator/runner/GUI fake hoặc localhost (13,37 s) và 1 ca mở/đóng binary mới ở IDLE (2,28 s) |
| CTest Qt | 1 target PASS, 0,41 s | Cả build native hiện tại và build gói mới; không tính hai lần chạy thành hai ca |
| Static laptop | PASS, 95 source/config, errors=[] | `STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED` |
| Gói và launcher | PASS | 413 pins; tái dựng cùng packet; binary/archive đúng receipt; archive verifier, Bash syntax và desktop-file-validate |

Probe Qt dùng kết nối Unix socket giả và ROS domain 211 localhost. Nó kiểm
danh sách công cụ/label, không click canvas hoặc phát thông điệp Pose/Goal.
Các test fake không xác nhận STOP vật lý, TF/QoS, camera hay khả năng hoạt
động của SSH/sudo trên Pi thật. Không cộng rerun thành ca mới.

Log và receipt kiểm tra trong `qa_evidence/pre_pi_fixes_20261029/`. Bằng chứng
6 FAIL trước sửa vẫn giữ ở `qa_evidence/pre_pi_recheck_20261029/`.

## Nhận dạng gói

- Packet: `750008b09dfeed1835cfcd0b52849e0d26f0185910c4f9646b1bda7b9d6717cf`.
- Archive SHA-256: `75c96c3340c1e8d4fd618dea5ef411a2a6ecdf52f7f3d97117a78986d97c6b69`.
- Binary SHA-256: `8991b83b152498059a6bd83ad2d2e6151cac077e26e0a9b569e429deeb27e0ad`.
- `artifacts/current.txt` đã chọn gói mới; launcher desktop dùng marker này.
- Receipt build: `native_ctest_passed=true`, `runtime_verified=false`,
  `pi_static_receipt=PENDING`. Receipt Pi phải thuộc đúng packet mới.

## Đồng bộ app có ảnh hưởng lệnh cũ trên Pi không?

Đã review `launchers/sync_pi_candidate.sh`: script chỉ đặt source/build app
trong `/home/pi/fire_robot_desktop/<12-ký-tự-packet>/ros`, tải archive vào
thư mục `incoming` của app và giữ receipt/log riêng. Baseline cũ chỉ được đọc
commit/status và source môi trường; script không ghi đè source/install của
baseline, không sửa `.bashrc`, systemd, mạng hoặc firmware. Script từ chối
build khi phát hiện phiên robot đang chạy.

Vì vậy việc cài gói app vào thư mục riêng không thay thế lệnh/bộ R5 cũ.
Muốn dùng lệnh cũ, mở terminal mới, source đúng bộ cũ và dùng cùng phiên bản
laptop/Pi với pairing mới của phiên đó. Không lấy pairing/receipt của app mới
ghép với runner cũ. Chạy một phiên vận hành tại một thời điểm; app và bộ R5
dùng chung khóa phiên và các topic robot.

Đây là kết luận từ source/script và hash local; chưa thử trước/sau đồng bộ
trên Pi thật trong lượt này. Chưa chạy lệnh sync. Bước tiếp theo là đồng bộ
packet mới qua LAN, xác minh receipt Pi 103 file, rồi nghiệm thu có giám sát
qua WiFi theo [hướng dẫn tích hợp](QA_INTEGRATION_2026-10-29.md).

## Chạy lại hồi quy sửa lỗi

```bash
source /opt/ros/humble/setup.bash
cmake -S apps/fire_robot_control/tests/pre_sync_ui_review \
  -B /tmp/fire_robot_pre_sync_ui \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/fire_robot_pre_sync_ui --parallel 2
PYTHONPATH="$PWD/apps/fire_robot_control:${PYTHONPATH:-}" /usr/bin/python3 -m pytest \
  -q -p no:cacheprovider --tb=short \
  apps/fire_robot_control/tests/qa_pre_sync_faults.py \
  apps/fire_robot_control/tests/qa_pre_sync_ui.py
```

Cần quyền socket/PTY cục bộ và X11 cho test Qt/RViz. Cách chạy các nhóm cũ
nằm trong hướng dẫn tích hợp; chưa dùng chúng làm bằng chứng robot thật.
