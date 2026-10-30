# QA lần hai trước đồng bộ Pi — 29/10/2026

> **Đã sửa sau review:** F1–F3 và sáu ca FAIL bên dưới đã được xử lý trong gói
> `750008b09dfe…`. [Kết quả sửa và QA mới](QA_FIXES_BEFORE_PI_2026-10-29.md).
> Nội dung bên dưới giữ làm lịch sử lỗi của e05; chưa đồng bộ Pi.

**Kết luận: còn 3 finding P2, cần sửa và đóng gói lại trước khi tiếp tục
đồng bộ/nghiệm thu app.** Chỉ thêm ca QA và tài liệu trong lượt này; chưa sửa
production, chưa đổi gói đang chọn, chưa SSH/sync/NTP/flash hoặc chạy robot.

Gói được review: `e05ef3e3a18eb289cdfa4e63a8280b15b225815721874dfaf129ea83c2f4ff3d`.
Review source hiện tại và đối chiếu source trong artifact; dựng lại manifest
từ source cho ra đúng cùng packet, 413 file. ROS node/config gốc nguyên byte;
chỉ hai script của parent được adapter thay trong gói mới: runner laptop và
parser manifest/archive. Parent R5 vẫn đủ 364 hash.

## Finding

### F1 — P2: lỗi file đánh dấu phiên làm supervisor giữ một worker không thể hoàn tất

- File: `services/supervisor.py:118` và `:181`; vòng chờ teardown ở `:332`.
- Trước `start()`, controller gán `self.session` rồi ghi `pending.json`. Nếu
  ghi thất bại (đã inject `ENOSPC`), exception chỉ cập nhật error; worker chưa
  chạy nhưng vẫn được giữ. CLOSE chuyển STOPPING và chờ FINISHED mãi.
- Ở cuối phiên, nếu unlink marker thất bại (đã inject `EACCES`), `tick()` ném
  exception trước khi bỏ tham chiếu worker đã hoàn tất. Vòng finally tiếp tục
  chờ một FINISHED đã bị lấy khỏi inbox. App không đóng bình thường được.
- Ảnh hưởng: kẹt app/không mở phiên tiếp; ca đầu chưa khởi động robot. Không
  dùng finding này để kết luận watchdog hoặc STOP vật lý bị hỏng.
- Đề xuất: rollback việc nhận worker nếu chưa start và ghi marker lỗi; khi
  FINISHED phải luôn thu hồi worker. Lỗi xóa marker giữ trạng thái BLOCKED,
  giữ bằng chứng và hiển thị lỗi nhưng vẫn cho đóng app. Không xóa marker chỉ
  để bỏ qua lỗi dừng chưa xác nhận.
- Tái hiện: hai test đầu trong `tests/qa_pre_sync_faults.py` đều FAIL trên gói
  được review.

### F2 — P2: công cụ lệnh RViz chưa khóa bền vững sau STOP/mất kết nối giao diện

- File: `native_rviz/session_ui.cpp:92`, `:115`, `:211`;
  `native_rviz/main.cpp:448` gọi cập nhật quyền Goal mỗi 100 ms.
- Khi đang POSE, bấm STOP đổi phase nhưng không đổi stage. Điều kiện công cụ
  chỉ xét stage POSE nên SetInitialPose vẫn tồn tại ngay sau STOP.
- Handler mất kết nối không lưu một cờ khóa. SetInitialPose vẫn còn nếu
  stage cũ là POSE. SetGoal bị gỡ ban đầu, nhưng một lần refresh còn đọc report
  operator hợp lệ cũ có thể thêm lại Goal vì phase vẫn RUNNING. Tuổi report
  tối đa 500 ms giới hạn cửa sổ này trong luồng bình thường; không được coi đó
  là khóa ngay tại thời điểm ngắt.
- Đã tái hiện trên class SessionUi/ToolManager thật của Qt/RViz, ROS domain
  211 localhost, signal lost giả lập và refresh quyền như main. Chỉ đếm công
  cụ; không phát `/initialpose` hoặc `/goal_pose`, không có robot/mission.
- Ảnh hưởng: UI còn cho phép chọn công cụ phát lệnh sau khi đã yêu cầu dừng
  hoặc mất supervisor. Chưa chứng minh robot nhận/chạy theo lệnh đó; các gate
  operator/watchdog phía sau vẫn giữ nguyên.
- Đề xuất: latch STOP/disconnected ở UI, dùng chung cho cả toolChanged và
  tools(); POSE cần phase phù hợp và kết nối hợp lệ. Refresh report không
  được bỏ khóa; chỉ phiên mới được xác thực mới khôi phục quyền.
- Tái hiện: ba tham số của `tests/qa_pre_sync_ui.py` đều FAIL.

### F3 — P2: xác nhận dừng phiên cũ được trình bày trong phiên mới

- File: `services/supervisor.py:120`; `native_rviz/session_ui.cpp:174`.
- Sau một FINISHED có STOP/cleanup confirmed rồi SCAN/FIRE phiên mới,
  `last_stop` vẫn là receipt trước. UI ghép dòng “Dừng robot: ĐÃ XÁC NHẬN ·
  Cleanup: SẠCH” vào trạng thái phiên mới mà không ghi đây là lịch sử.
- Đã tái hiện tại controller và kiểm label trên class Qt thật. Đây là lỗi
  trình bày/ghép phiên; không có receipt giả mới hoặc bypass gate vận hành.
- Đề xuất: reset kết quả hiển thị khi nhận phiên mới, hoặc bind session và
  trình bày rõ thành lịch sử tách khỏi trạng thái hiện tại. File receipt cũ
  vẫn cần giữ trong log để tra cứu.
- Tái hiện: test cuối `tests/qa_pre_sync_faults.py` FAIL.

## Kết quả đã chạy lại

| Kiểm tra | Kết quả lần này | Giới hạn |
| --- | --- | --- |
| Bộ unittest hiện có | 124 PASS / 4,998 s | Chủ yếu fake/offline; chưa bao phủ các lỗi mới ở trên |
| Tích hợp hiện có | 17 PASS / 15,83 s | Socket/operator giả, Qt, RViz localhost, packaged idle, generated runner I/O giả |
| CTest Qt hiện có | 1 target PASS / 0,41 s | IPC/lease/parser; không phải kiểm robot |
| Hồi quy mới | **6 FAIL / 0,70 s** | 2 ca F1, 3 ca F2, 1 ca F3; đã tái hiện đúng hành vi lỗi |
| Build native | PASS | CMake incremental; có thông báo thiếu freeimage tùy chọn, không chặn build |
| Static inventory laptop | PASS, 95 source/config, errors=[] | `runtime_verified=false`, không SSH hoặc đo runtime |
| Source/binary/archive | PASS | 413 pins, binary/archive hash đúng receipt, source tái dựng cùng packet |
| Launcher | Bash syntax + desktop-file-validate PASS | Sync chỉ dùng SSH/SCP giả trong bộ 124; chưa kiểm Pi thật |

Không cộng rerun thành ca mới. Các ca mới FAIL là bằng chứng còn lỗi, không
được thay bằng con số PASS của bộ cũ. Chi tiết log, hash và fault probe trong
`qa_evidence/pre_pi_recheck_20261029/`.

## Lệnh tái hiện ca mới

Chạy từ repo trên laptop. Hai file Python đầu chỉ dùng fake session; probe Qt
bị chặn nếu không ở domain 211 localhost, không có code phát Pose/Goal.

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

Để đóng finding, sửa đúng supervisor/UI, chạy 6 ca trên thành PASS, chạy lại
các nhóm có liên quan rồi đóng gói packet mới và kiểm manifest/binary/archive.
Không sửa trực tiếp artifact e05 hoặc parent đã niêm phong. Sau đó mới dùng
packet mới để đồng bộ Pi và lấy receipt tương ứng.

## Phần vẫn cần Pi/robot thực tế

Receipt Pi mới còn PENDING. SSH key/host trust, prompt mật khẩu/sudo, scp,
pairing qua WiFi, SLAM/Nav2/YOLO, TF/QoS thực tế, camera latency/FPS và STOP
vật lý chưa được xác nhận trong lượt này. Ba finding trên độc lập với những
giới hạn đó. Không flash hoặc đổi firmware/domain/mạng/watchdog để xử lý chúng.
