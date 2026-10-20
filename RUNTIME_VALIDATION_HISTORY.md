# Nhật ký thay đổi và kiểm thử runtime

Mỗi bản code phải có mô tả thay đổi, bằng chứng kiểm thử và giới hạn sử dụng.
**Commit đã tạo hoặc push không đồng nghĩa robot đã chạy ổn.**

## Cách cập nhật sau mỗi lượt test

1. Ghi đúng commit CODE được chạy, máy chạy, cấu hình/điều kiện và ngày nhận kết quả.
   Nếu cây làm việc có sửa code, ghi thêm hash hoặc patch; không gán PASS cho HEAD sạch.
2. Ghi lệnh/tool, kết quả, nguồn bằng chứng (output người vận hành hay raw log đã kiểm),
   đường dẫn log và kiểm hash nếu có. Không cộng số test của các suite chạy chồng nhau.
3. Cập nhật riêng OFFLINE, BUILD, STATIC RUNTIME và MOTION/ACTUATOR.
   Dùng PASS, FAIL, BLOCKED, INCOMPLETE hoặc NOT_TESTED; luôn ghi phạm vi của PASS.
4. Giữ lịch sử lỗi cũ. Thêm kết quả mới thay vì xóa lần FAIL/BLOCKED hoặc đổi nhãn
   một lượt chưa đủ bằng chứng thành PASS. Liên kết commit sửa với lỗi đã quan sát.
5. Ghi vấn đề còn mở, bước tiếp theo và bản code đã nghiệm thu gần nhất cho đúng phạm vi.
   Bản chỉ đổi tài liệu phải trỏ về commit code được test, không tự nhận PASS runtime.
6. Trước commit runtime, body phải nêu thay đổi, kiểm thử, hạn chế và mã bản ghi này.
   Sau khi có SHA, cập nhật mục lục cục bộ `docs/codex/COMMIT_TEST_LEDGER.md` và checkpoint.
   File này được theo dõi trong Git; mục lục cục bộ nằm trong docs/ đang gitignore.

## Bảng trạng thái hiện tại — ghi nhận ngày 2026-10-20

| Bản code | Offline | Build Pi | Runtime Pi của M3 | Kết luận sử dụng |
| --- | --- | --- | --- | --- |
| `c1022014bd327105d06b10b6edc392145a0dbeb3` | PASS 1078 Python + 1 CTest trong Gate A | PASS Gate B | BLOCKED Gate C | Chưa nghiệm thu M3; baseline đang có trên Pi |
| `M3-CLOCK-20261020-01` (commit chứa bản ghi này) | PASS 530 test | NOT_TESTED | NOT_TESTED | Candidate sửa lấy mẫu giờ; chờ build/runtime Pi |

Bảng chỉ tổng hợp các mốc M3 liên quan, không đánh giá lại mọi commit lịch sử.
Chưa có bản M3 nào được xác nhận ổn toàn bộ từ các phép thử được ghi ở đây.

## M3-CLOCK-20261020-01 — lấy mẫu giờ cho stop evidence

- Commit cha: `c1022014bd327105d06b10b6edc392145a0dbeb3`.
- Commit code: commit thêm mục này, nhận diện bằng trailer
  `Validation-Record: M3-CLOCK-20261020-01` trong body Git.
- Ngày nhận kết quả QA: 2026-10-20.
- Trạng thái: **OFFLINE_PASS / PI_BUILD_NOT_TESTED / PI_RUNTIME_NOT_TESTED**.

### Đã thay đổi gì và vì sao

Supervisor cũ đọc giờ ROS rồi monotonic riêng rẽ. Khi bị tạm ngừng giữa hai lần
đọc, cặp giờ có thể lệch quá 20 ms dù hai đồng hồ thực tế vẫn nhất quán. Có test
fake-clock tái hiện đường lỗi phần mềm này; chưa chứng minh đây chính là nguyên
nhân của lần lỗi Pi đã ghi.

Thêm cách đọc monotonic → ROS → monotonic. Nhận mẫu khi khoảng đọc không quá
1 ms; thử lại toàn bộ cặp tối đa 3 lần. Giá trị invalid hoặc đi lùi chặn ngay,
kể cả so với lần đọc trước và giữa các lần retry. Nếu hết lượt vẫn không có mẫu
đủ tin cậy, giữ lỗi `GLOBAL_CLOCK_SAMPLE_UNCERTAIN` và xóa zero hold.

Dùng cặp giờ ở callback stop, admission, kiểm graph, close barrier và kiểm tra
sau publish. Header trạng thái vẫn lấy trước công việc graph/evaluate. Bổ sung
counters, mẫu cuối và cặp clock đầu tiên gây fault vào shadow state hiện có.
Không có node/timer/topic mới; không nội suy ROS hoặc thay timestamp nguồn.

Giữ freshness 350 ms, skew/clock guard 20 ms hoặc cấu hình chặt hơn, watchdog,
STOP, source identity, quota/ledger và yêu cầu operator recovery. Chưa đo CPU.

### Bằng chứng offline

Người vận hành chạy trên laptop:

```bash
bash '/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/.codex_qa_backups/qa_m3_stop_clock_candidate_20261020/verify_offline.sh'
```

Kết quả: **530 passed in 1.74s**, marker `M3_STOP_CLOCK_OFFLINE_QA_OK=1`.
11 file test bao gồm tái hiện lỗi cũ, retry, clock jump thật, rollback, invalid,
hết retry, dữ liệu DDS 400 ms cũ, callback, close proof và replay nhiều phiên.
15/15 hash source/test/script khớp manifest đã được chấp nhận.
Đây là output user cung cấp; assistant kiểm hash/diff, không tự chạy test/build.
Không thay code/test sau lượt PASS này.

Hồ sơ cục bộ: `.codex_qa_backups/qa_m3_stop_clock_candidate_20261020/`:
`user_gate_result.json`, `accepted_offline_manifest/`, `FINAL_REVIEW.md`,
`before/`, `before.sha256`, `candidate.patch`, `commit_sources.sha256`.
Các hồ sơ/log này đang gitignore, không có trong bản clone GitHub.

Hash bốn file code/test được QA (file Markdown này không đổi hành vi runtime):

```text
1352c210d2ddc229f7dbf29060fac0ceb25e55666f520b81953a4ad99352791b  ros2_ws/src/fire_robot_mission/fire_robot_mission/stop_clock_sampling.py
400b4bb3261135776c5d6832a6423e15ebfd733448c01c630d45e37becf418a0  ros2_ws/src/fire_robot_mission/fire_robot_mission/mission_supervisor.py
7468fb3a5a9bf06a54e12ce7d235abe26abf39e0d40395a8ad12c53cb7a51e68  ros2_ws/src/fire_robot_mission/fire_robot_mission/global_stop_evidence.py
fe6e1fbe550ba70734dff74077b026430e5a387514035dea3a13d401a9372513  ros2_ws/src/fire_robot_mission/test/test_stop_clock_sampling.py
```

### Còn lỗi/chưa kiểm gì

- Chưa build/import bản này trên Pi, chưa chạy Gate C bằng bản này.
- Chưa xác định dứt điểm clock fault của Pi có phải do cặp đọc bị lệch hay không.
- RViz trên laptop vẫn có sự kiện SIGSEGV (`exit -11`); chưa có bản sửa.
- Ảnh đầu vào YOLO từng có khoảng trống; chưa kết luận tải Pi hay WiFi/camera.
- Readiness V2 giữ 64 thay đổi đầu, chưa lưu trạng thái cuối rõ khi bão hòa;
  cần sửa chẩn đoán trước lần nghiệm thu tiếp theo.
- Chưa nghiệm thu motor/bơm hoặc đổi quyết định Nav2 chạy ở máy nào.

### Bước tiếp theo

Chốt commit, xác minh blob code khớp bản QA. Sau user duyệt push mới chuẩn bị
backup/checkout/build Pi. Chuẩn bị đủ final-readiness/clock diagnostics và rà RViz
trước phép đo runtime có giới hạn. Giữ ledger hiện có; không tự chạy lại Gate C.

## c1022014bd327105d06b10b6edc392145a0dbeb3 — baseline M3

Thêm bounded multi-session shadow runtime. Gate A user PASS 1078 Python tests
và 1 native CTest; Gate B Pi build/import/install đạt với 96 source hash khớp.
Log Gate B: `/home/pi/qa_logs/qa_m3_gate_b_20261020_141720`.

Gate C gần nhất: `/home/pi/qa_logs/qa_m3_runtime_v2_20261020_153210`.
13/13 file log khớp hash; ledger không đổi và cleanup sạch. Cả 8 lifecycle Nav2
cuối lượt active nhưng mission giữ `GLOBAL_CLOCK_DISCONTINUITY`; readiness
BLOCKED, chưa bắt đầu measurement. Vì thế **M3_RUNTIME_BLOCKED**, không gọi bản
baseline này là robot đã chạy ổn. Camera/RViz cần theo dõi riêng.

Bản sửa M3-CLOCK-20261020-01 xử lý nguy cơ lấy mẫu giờ; kết quả runtime còn chờ.
