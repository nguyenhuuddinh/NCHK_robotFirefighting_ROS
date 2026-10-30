# STOP sau runtime fault và chẩn đoán Nav2

Lượt này xử lý F2 đã chứng minh ở phiên `825c1487…`; bổ sung quan sát cho F1/F3.
Không tuyên bố đã sửa nguyên nhân sâu khiến Nav2 mất bond. Không chạy robot,
motor, bơm, NTP, flash hoặc sửa release R5 trong lượt kiểm thử này.

## Lỗi đã xác nhận và thay đổi

Phiên 825c có verifier STOP Q confirmed sau 0,403 s, các nhóm tiến trình thoát
sạch, nhưng operator rc2 vì còn fault `NAV2_BACKEND_LOST`. StackCleanup bỏ qua
receipt của process rc2 nên app báo STOP_UNCONFIRMED. Bằng chứng gốc, replay và
timeline nằm trong [báo cáo phân tích](QA_NAV2_BOND_STOP_825C_2026-10-30.md).

App mới tách hai kết quả:

- **Kết quả tiến trình:** rc0 chỉ khi verifier STOP confirmed và toàn bộ teardown
  node/channel/ROS thành công. Mọi exception, thiếu STOP hoặc ghi kết quả thất
  bại vẫn rc2. Không có đường chấp nhận tùy tiện operator rc2 cũ.
- **Kết quả vận hành phiên:** runtime fault được lưu vào `operator_result.json`.
  Runner giữ **session rc2** và nguyên nhân runtime ngay cả khi STOP/cleanup đạt.
  UI có thể về IDLE sau khi đủ proof dừng/dọn, đồng thời giữ lỗi phiên trong log.

Kết quả riêng này bắt buộc khớp session, packet, generation, PID và thời điểm tạo
process trong `/proc`; file độc quyền mode 0600, không ghi đè file/symlink. Đây
**không phải bằng chứng STOP**. StackCleanup/StreamingOwnedLog gốc vẫn kiểm tra
process rc0, group exit, đúng inode/log, một receipt đầy đủ, child audit sạch;
metadata không hợp lệ không được biến cleanup thành đạt.

Không sửa receipt/pending của phiên 825c bằng kết quả mô phỏng. Phiên cũ vẫn cần
xác minh STOP mới trên robot trước khi mở phiên khác.

## Quan sát bổ sung cho F1/F3

- Health entry **chỉ luồng chữa cháy laptop** bọc đúng lần `sample/report` vốn có,
  ghi thời gian query và các toán hạng localization: nguồn/giá trị pose, pose sau
  Initial Pose, map TF, action, lifecycle, identity. Không thêm query hoặc đổi
  quyết định readiness.
- Wrapper Nav2 đọc `/proc` của tối đa 32 hậu duệ cùng nhóm: CPU, RAM, threads,
  trạng thái tiến trình, áp lực CPU/memory/I/O và khoảng cách giữa mẫu.
- Log tối đa 1 Hz, mỗi file 2 MiB và một bản xoay; lỗi ghi diagnostic vô hiệu hóa
  diagnostic, không tạo proof hoặc bỏ guard. Có thêm đo thời gian `spin_once`.
- UI phân biệt đang phục hồi Nav2/định vị với lỗi chung. Không tự restart Nav2,
  không tăng bond timeout hoặc các ngưỡng freshness/STOP/watchdog.

## QA và ca lỗi giữ mở

Đã đóng gói, đồng bộ tĩnh Pi và chọn packet
`5054a43e5371bc1ce31c08a8b23f6dc79d1e2a122f0fa2f940c800ab23e77890`.
Gói có **440 source pins**, giữ nguyên **364/364 file R5**. Pi build hai package
8,96 s, **103/103 file cài Pi** và **95/95 file cài laptop** đúng hash.
Không tự mở app hoặc chạy robot. `pending.json` cũ SHA `cae6999a…` giữ nguyên.
Chuẩn bị xác minh STOP cho chính pending 825c đã qua kiểm tra **chỉ đọc** trên
laptop/Pi qua LAN (`old_pending_prepare.json`): đủ điều kiện tạo lượt STOP mới,
nhưng chưa tạo lượt đó, chưa xóa pending và chưa xác nhận dừng vật lý mới.

Kết quả cuối trên laptop:

- **278 unittest PASS**, 15,620 s: [log](qa_evidence/stop_result_fix_20261030/unittest_final.log).
- **188 pytest tích hợp PASS**, 107,46 s; một warning thư viện Python cũ, không
  failure: [log](qa_evidence/stop_result_fix_20261030/pytest_full.log).
- **Qt CTest 1/1 PASS** trong gói; **smoke binary 1/1 PASS** sau đóng gói.
- Operator thật trong artifact trên localhost 211, không telemetry: Q trả rc2,
  STOP false, metadata failed, socket/helper/group đã thoát. Không được tự tạo
  STOP proof từ việc process đã biến mất.
- Nav2 thật cô lập với health dừng đúng stage: goal SUCCEEDED, child 1/1 sạch,
  health rc0, không process sống; shutdown 1,818 s, có 5 mẫu performance và
  4 mẫu health diagnostic. Đây là kiểm thử localhost với đầu vào giả.

26 ca delta operator/runner đã chạy trước nằm trong các bộ trên; không cộng
trùng thành số ca mới. Log thô, receipt cài, selection và kiểm hash nằm trong
[thư mục bằng chứng](qa_evidence/stop_result_fix_20261030/).

1. Unit/integration kiểm tra runtime fault + STOP tốt, STOP thiếu, teardown lỗi,
   file không ghi được, metadata sai, proof log thiếu và process/PID thật với
   ROS giả. Fault vẫn làm cả phiên rc2; không telemetry vẫn không được confirmed.
2. Regression chọn/lưu map, mapping, operator X/Q, IPC/Qt, recovery và guard cũ.
3. Nav2 thật với TF/odom/scan **giả trên localhost domain 216**: goal SUCCEEDED;
   dừng health cùng stage với Nav2 như R5; child 1/1 sạch, không process sống,
   health rc0, có log CPU/RAM và localization blockers.
4. **P1 còn mở — Q trong recovery nội bộ Nav2.** Ca âm tính SIGSTOP
   **container Nav2 sở hữu riêng của bài QA**, giữ 6 giây
   rồi SIGCONT; hai manager mất bond và tự reset. Q trong recovery có thể dọn
   không sạch. Bản source mới gặp native exit -6/SIGSEGV; bản e2 cũ cũng dọn
   không sạch do SIGINT→SIGTERM. Cả hai bài đều **FAIL cleanup**, audit vẫn chặn.
   Đây là một edge case có trước; khác dạng kết thúc giữa hai lượt không đủ
   chứng minh hồi quy hoặc nguyên nhân sâu trên robot. Chưa sửa Nav2/R5 bằng
   phỏng đoán, chưa nghiệm thu Q trong mọi trạng thái phục hồi.

Lần probe đầu giữ health sống qua teardown Nav2 không khớp thứ tự R5 và gặp
fallback. Giữ nguyên log đó, sau đó chỉnh fixture dừng health đúng stage và chạy
ca bình thường đạt; không gộp lần lỗi vào PASS. Probe cũng đã sửa để trả lỗi khi
child audit không sạch, kể cả `ros2 launch` trả rc0.

## Phạm vi bảo toàn và nghiệm thu tiếp

Các file quét/lưu map, model `best_minireplay.pt`, dữ liệu map TN1 và toàn bộ
364 file release R5 được kiểm hash. Không sửa topic/QoS, TF/safety policy,
firmware, tốc độ mapping hoặc lệnh terminal R5. Diagnostic chỉ ở laptop.

Sau khi gói được kiểm và đồng bộ tĩnh đúng phiên bản, cần một lượt có giám sát:
xác minh STOP cũ, chọn map, khởi động, đặt Initial Pose, theo dõi phục hồi; Q rồi
đối chiếu receipt STOP, child audit và log diagnostic mới. Không dùng kết quả
localhost hoặc hash cài đặt để kết luận xe/bơm đã dừng thật.

Để dùng bản mới: đóng app cũ, chuyển laptop sang WiFi Pi rồi mở lại **Fire Robot
Control**. Khi bắt đầu phiên robot, rút LAN theo preflight WiFi hiện có; USB
Internet có thể giữ. Nếu app còn BLOCKED vì phiên 825c, dùng **Xác minh dừng trên
robot**; **Kiểm tra lại** chỉ đọc, không thay thế bằng chứng STOP mới. Chỉ khi app
về IDLE mới chọn map và khởi động. Lưu log của lần tiếp theo để so điều kiện
localization và thời gian native query, thay vì tăng timeout theo phỏng đoán.

SSH WiFi `10.0.0.1` trong lượt này timeout vì laptop đang ở WiFi khác; đồng bộ
đạt qua LAN `10.42.0.185`. Không đổi network/clock để ép preflight đạt. Mọi QA ROS
cô lập dùng domain riêng và localhost; domain/QoS vận hành không đổi.
