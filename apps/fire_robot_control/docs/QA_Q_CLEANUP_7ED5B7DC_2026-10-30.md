# QA log khi nhấn Q — phiên 7ed5b7dc, 30/10/2026

**Kết luận:** Q được nhận đúng, STOP được telemetry xác nhận sau 0,40224 s.
Phiên bị BLOCKED vì Nav2 không hoàn tất shutdown trong ngân sách, không phải do
mất heartbeat giao diện trong lượt Q này. Có thêm lỗi ở nhánh phục hồi của app:
chưa nhận trường hợp STOP đã xác nhận nhưng Nav2 phải dừng cưỡng bức.

Chỉ phân tích và replay chỉ đọc; không sửa code vận hành, chạy lại robot, sync,
đổi cấu hình hoặc xóa pending. 364 file R5 gốc, source app, map người dùng,
current và pending giữ nguyên hash trong lượt QA này.

## Định danh và nguồn bằng chứng

- Thư mục thật: `/home/huudinh/.local/state/fire_robot_control/session_7ed5b7dc66dc47139594fc447a516aa8`.
- Session: `4817d475-2433-4da1-a698-c060f0d7bceb`.
- Packet: `d5209520c8f894a0996f3e3841ba2709b945192f1cf38087c3995907d1a2e222`.
- Receipt Pi gắn đúng session riêng `b42631c8-7407-4e24-b51b-5897f41f80b0`, cùng packet;
  root `/home/pi/qa_logs/r5_automatic_20261030_094447`.
- Bản sao 14 log/receipt chọn lọc và SHA256:
  [evidence](qa_evidence/q_cleanup_7ed5b7dc_20261030/result.json).
  Không sao chép pairing secret hoặc runtime rebind nonce.

## Findings, theo ảnh hưởng vận hành

### F1 — P1: Q không kết thúc phiên sạch vì Nav2 vượt cả hai thời hạn shutdown

Bằng chứng trực tiếp:

- `raw_laptop/laptop/operator.log:289–290`: `OPERATOR_Q`; STOP cause Q,
  `confirmed=true`, `FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED`, blockers rỗng.
- `raw_laptop/laptop/mission.log:49`: actor drain confirmed sau 0,42880 s.
  Mission 9/9 con và YOLO 2/2 con thoát sạch.
- `raw_laptop/laptop/nav.log:1561–1607`: SIGINT đi vào rclcpp; `bt_navigator`
  đang preshutdown/cleanup thì lifecycle manager còn phát lệnh deactivate/cleanup.
  Xuất hiện lỗi transition 4 và 2 từ trạng thái `cleaningup`.
- `nav.log:1617`: launch hết hạn **5 s** và gửi SIGTERM cho container.
- `cleanup.json/processes/nav`: runner tiếp tục hết hạn **8,013 s** từ SIGINT,
  gửi SIGTERM cho cả group. Leader rc `-15`, `forced=true`; toàn group sau đó
  `GROUP_EXITED`, `live_pids=[]`.
- Child audit không có dòng terminal cho container PID 359312: started 1,
  terminal 0, cộng `SIGINT_TO_SIGTERM`. Không được coi mất dòng terminal là
  bằng chứng child vẫn sống; snapshot `/proc` hiện tại xác nhận PID này đã vắng.

Liên hệ source (bản R5 bất biến được app đóng gói nguyên byte):

- `releases/r5_20261029/ros/ros2_ws/src/fire_robot_navigation/launch/nav2_composed_shadow.launch.py:126–131`
  không override thời hạn launch. Thư viện Humble cài trên máy,
  `/opt/ros/humble/lib/python3.10/site-packages/launch/actions/execute_local.py:90–93`,
  đặt mặc định SIGINT→SIGTERM 5 s, tiếp đến SIGKILL 5 s.
- `releases/r5_20261029/ros/scripts/operational_owned_process.py:29,75–85`:
  outer budget SIGINT 8 s, SIGTERM 3 s; khi quá hạn thì `forced=true`.
- `releases/r5_20261029/ros/scripts/operational_stack.py:62–70` ghi
  `PROCESS_UNCLEAN:nav` và `CHILD_LOG_AUDIT_FAILED`.
- `releases/r5_20261029/ros/scripts/r5_laptop_session.py:192` trả rc2 khi
  local cleanup không sạch. `services/session_outcome.py:5–16` giữ cleanup false.

**Mức chắc chắn:** chứng minh được chuỗi timeout/escalation và lý do BLOCKED.
Log cho thấy các đường lifecycle shutdown chồng nhau, nhưng chưa có stack trace
để kết luận deadlock, chỉ ra thread cụ thể hoặc khẳng định tăng timeout sẽ đủ.
Sau SIGTERM đầu tiên Nav2 vẫn tiếp tục log cleanup; không thể gọi nó đã đứng
hoàn toàn tại mốc 5 s. Lỗi “CRITICAL FAILURE ... heartbeat” đầu tiên trong đoạn
này xảy ra **sau SIGINT**; không dùng thông báo đó để quy lỗi mất kết nối trước Q.

Hướng sửa đề nghị: chuẩn bị đường shutdown Nav2 riêng cho app, teardown lifecycle
có thứ tự khi ROS còn hoạt động, sau đó mới đóng launch; phối hợp hai ngân sách
thời gian có giới hạn. Giữ nguyên bản R5 và kiểm chứng thực tế việc thoát child.
Không đổi forced rc thành clean hoặc bỏ child audit.

### F2 — P1: nhánh phục hồi chưa phủ confirmed STOP + forced Nav2

`services/stop_recovery_runner.py:19–35` yêu cầu đồng thời:

- `stop_confirmed is False`;
- đúng bộ bốn lỗi gồm STOP unconfirmed và operator unclean;
- operator rc2.

Phiên thật lại là STOP true, operator rc0, chỉ hai lỗi Nav2/child audit. Vì vậy
`old_groups()` từ chối trước mọi SSH với `RECOVERY_FORCED_NAV_PROOF_INVALID`.
Nhánh “Kiểm tra lại” chỉ đọc cũng từ chối với
`RECOVERY_LOCAL_CLEANUP_MISSING` tại `services/recovery.py:42–45` vì cleanup cũ
thực sự false. Hai kết quả đã replay bằng **receipt thật, không sửa fixture**.

Guard giữ BLOCKED là đúng khi thiếu bằng chứng; lỗi là chưa có đường phục hồi
có kiểm chứng cho tổ hợp trạng thái này. Bấm đi bấm lại không thay đổi receipt
cũ nên sẽ không tự mở khóa. Đây là dự đoán theo source/replay, không phải tuyên
bố đã bấm nút thật trong lượt QA này.

Hướng sửa đề nghị: nhận diện riêng tổ hợp confirmed STOP/forced Nav2, vẫn xác minh
đầy đủ group + PID con vắng, đúng session/packet/boot và Pi idle. Chỉ archive pending
sau audit hợp lệ hoặc phiên STOP mới theo workflow được chọn; giữ receipt lỗi cũ.
Không áp dụng ngoại lệ này cho mapping hoặc lỗi child không biết.

### F3 — P2: thông báo rc2 che lý do Nav2 và ghi lại startup thành FAILED

`services/live_session.py:388` fallback `RUNNER_EXIT_2`; sau đó dòng 558–560 đặt
`startup_result.status=FAILED` vì có lỗi cuối phiên. Trong khi
`automatic_start.json` chứng minh đã READY_GATE2, và Nav2 ghi nhiều goal succeeded
trước Q (goal cuối 09:52:35,404).

Đây là lỗi phân loại/thông báo: startup thành công nhưng teardown thất bại.
Nên hiển thị nguyên nhân có cấu trúc như Nav2 shutdown timeout/forced child, và
giữ kết quả startup riêng với kết quả STOP/cleanup. Không dùng rc2 để suy ra
robot chưa dừng hoặc startup chưa bao giờ đạt.

## Timeline chính — giờ Việt Nam

| Thời điểm | Sự kiện |
| --- | --- |
| 09:52:35,404 | Nav2 goal cuối ghi succeeded |
| Khoảng 09:52:44,9 | Nhận Q; supervisor heartbeat age 81 ms |
| 09:52:45,301 | STOP confirmed sau 0,402 s |
| 09:52:46,086 | Actor drain confirmed; cuối cùng 9/9 mission child clean |
| 09:52:47,591 | SIGINT tới Nav2 container |
| 09:52:48,497 và 09:52:50,138 | bt_navigator bị yêu cầu transition khi đang cleaningup |
| Khoảng 09:52:52,6 | Launch hết hạn 5 s, SIGTERM child |
| 09:52:54,589 | Dòng cuối Nav2: bt_navigator Destroying |
| Khoảng 09:52:55,6 | Outer runner hết hạn 8 s, SIGTERM cả group; rc -15 |
| Sau cleanup | STOP true; cleanup false; app BLOCKED/CLEANUP_REQUIRED |

Hai mốc SIGTERM có dấu “khoảng”: log tín hiệu launch không có wall time trên
chính dòng đó; mốc outer suy ra từ monotonic signal và cặp wall/mono gần nhất.
Thứ tự và độ dài 8,013 s lấy trực tiếp từ receipt.

## QA đã chạy và giới hạn

[replay.py](qa_evidence/q_cleanup_7ed5b7dc_20261030/replay.py) thực hiện 9 kiểm tra
chẩn đoán, đều đạt: parser child audit khớp receipt thật; outcome `(True, False)`;
fault CLEANUP_REQUIRED; STOP thiếu và session sai vẫn bị từ chối; hai đường
phục hồi tái hiện chính xác mã từ chối; các group/PID Nav2 cũ hiện vắng;
364 source R5 khớp manifest. Hash source app/mapping/map/current/pending trước
và sau replay không đổi. Đây là xác nhận chẩn đoán, không phải 9 ca nghiệm thu
robot hoặc bằng chứng mọi lỗi đã hết.

Lệnh chạy lại, không khởi động ROS hoặc điều khiển phần cứng:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 apps/fire_robot_control/docs/qa_evidence/q_cleanup_7ed5b7dc_20261030/replay.py
```

SSH read-only tới `pi@10.0.0.1` với BatchMode/ConnectTimeout 7 s trong lượt này
bị connection timeout, chưa đọc thêm log Pi trực tiếp. Receipt Pi đã lưu của
đúng phiên ghi hai group exited, 8/8 child clean, confirmed/cleanup_confirmed
true, runner_error null. Phân biệt receipt lịch sử này với xác minh Pi hiện tại.

Log còn ghi hai lần phục hồi `EXACT_GOAL_HANDOFF_TIMEOUT` trước Q, đều đến trạng
thái recovered, không restart Nav2. Chúng là vấn đề riêng cần QA tiếp nếu phân
tích vận hành chữa cháy; không phải nguyên nhân trực tiếp của rc2 cleanup cuối.
Chưa phân tích riêng camera stale trong ảnh 09:52:15, và không quy nó cho YOLO
hoặc mạng khi chưa có bằng chứng. Không có thay đổi topic/QoS/TF/watchdog trong
lượt này; không lặp full QA offline cũ hoặc chạy robot để tái hiện lỗi đã rõ.
