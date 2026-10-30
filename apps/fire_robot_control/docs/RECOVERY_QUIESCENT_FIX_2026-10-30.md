# Phục hồi bằng STOP mới sau cleanup Pi lỗi

Ngày 30/10/2026, giờ Việt Nam. Gói mới:
`f63e38d71f5e767960bf84ef151cfdaf72b680f329bf3b080dcf27828dca38bd`.

## Vấn đề và hành vi mới

Phiên Pi cũ có TF node thoát lỗi sau SIGINT, dù đã kết thúc đủ 8/8 child.
RECHECK và cả hai gate của Xác minh dừng đều phụ thuộc cleanup cũ thành công,
nên không thể lấy proof mới trong cùng boot. File DONE cũ có tồn tại và được
giữ nguyên; bản sửa không đổi kết quả FAIL đó thành PASS.

**Xác minh dừng** nay có thể đi tiếp khi lịch sử Pi lỗi hoặc thiếu DONE, sau
khi có bằng chứng mới rằng runner, group và child đã ghi nhận không còn sống,
không có tiến trình robot/I/O khác, lock Pi rảnh, đúng pairing/boot và boot
không đổi trong lúc kiểm. Proof này mang scope
`QUIESCENT_ONLY_REQUIRES_NEW_STOP`; hoàn toàn chưa có STOP claim.

Cả gate đầu và audit cuối đều hiểu proof hiện trạng này. Chỉ sau khi phiên
STOP mới xác nhận telemetry và cleanup mới đầy đủ, audit mới archive pending
và cho IDLE. Không tự chạy lại scan/chữa cháy. **Kiểm tra lại vẫn chỉ audit
receipt lịch sử bằng điều kiện cũ**, không mở I/O hoặc phát STOP mới.

Nếu thiếu receipt laptop cuối, các phiên mới có journal sở hữu riêng gồm
session/packet, public binding Pi, stage và PID/start ticks của runner laptop.
Phục hồi kiểm journal, receipt nào còn tồn tại, các PID launch trong log,
tiến trình/lock hiện tại rồi mới cho lấy STOP mới. Journal không chứa secret.
Hồ sơ cũ không có journal mà cũng mất receipt vẫn bị từ chối với lý do rõ;
không suy đoán ownership hoặc xóa khóa để chạy.

## Phạm vi code

- `services/recovery_probe.py`: nhánh quiescence cùng boot, đọc PID launch
  có giới hạn, kiểm owner/type/symlink, mở rộng inventory I/O. Không có lệnh
  kill, ROS publish, đổi giờ hay start node trong probe.
- `services/recovery.py`: kiểm nonce/binding/boot/scope/PID của loại proof
  mới; giữ nguyên validator cho strict RECHECK và proof STOP mới. Thêm câu
  giải thích lỗi lịch sử, thiếu ownership hoặc log vượt giới hạn.
- `services/stop_recovery_runner.py`: fallback journal chỉ khi thiếu receipt;
  đối chiếu binding lại sau round trip, kiểm I/O/robot process ở cả hai gate.
- `services/recovery_ownership.py`: journal và kiểm ownership khi thiếu file;
  không chấp nhận receipt còn lại mâu thuẫn hoặc spawn dở chưa có PID.
- `services/live_session.py`: ghi journal sau pairing và trước/sau spawn.
  Không đổi nhánh launch, timeout, STOP hoặc xử lý quét/lưu map.

Source R5 364 file, node TF nguyên gốc, watchdog, topic/QoS, deadline, model
`best_minireplay.pt`, native RViz/Esc/Q, mapping runner/teleop/map saver giữ
nguyên. Không xử lý ngoại lệ ROS của TF bằng cách nuốt RuntimeError. Căn
nguyên nội bộ lỗi chuyển đổi ở rclpy vẫn cần tái hiện riêng nếu tái diễn.

## QA và bằng chứng

[Evidence](qa_evidence/recovery_quiescent_fix_20261030/).

- **306 unittest PASS**, 16,008 s, trên source cuối. Gồm 15 ca quiescence/
  ownership: Pi child lỗi, thiếu DONE, group sống, owner không xác định, lock
  bận, boot đổi, DONE sai danh tính, PID hỏng, RPC replay/sai scope/STOP claim
  giả, journal thiếu/khởi động dở/sai binding/symlink, child PID trong log,
  thiếu hai receipt, fresh STOP âm/dương, archive và không ghi secret.
- **289 pytest PASS**, 143,58 s, một warning Python đã có. Bao gồm 32 ca
  timeout/recovery, 48 ca dữ liệu STOP mới/trễ/mất và Q/X/lặp, Qt/IPC, Esc/Q,
  chọn/lưu map, camera/map display, auth, view lifecycle và worker transport.
  Sau full run chỉ thêm pattern chặn `ir_range_preview` đang sống và hai ca
  unittest ownership; toàn bộ unittest cuối và delta ownership đã chạy lại.
- 32 ca timeout matrix giờ kiểm Pi TF lỗi cùng boot và thiếu DONE: RECHECK
  vẫn từ chối, prepare có thể qua, proof mới đạt thì audit/archive thành công.
  Các case thiếu ownership, forced local ngoài profile hỗ trợ hoặc thiếu
  STOP/cleanup mới vẫn bị chặn. Các kết quả này là mô phỏng có assertion,
  không phải nghiệm thu chuyển động robot.
- **Pi thật chỉ đọc**, qua LAN, đúng phiên
  `/home/pi/qa_logs/r5_automatic_20261030_153737`: proof mới hợp lệ, kiểm
  **11 PID**, process absent, common lock rảnh, cùng boot. Không tạo STOP mới,
  không mở node và không xóa pending. `pi_read_only_quiescent.json` có proof.
- Native build/CTest **PASS**. Smoke binary đóng gói mở/đóng IDLE: **1 PASS**,
  2,72 s. Operator đóng gói localhost: Q khi không có telemetry phải ra rc2,
  STOP chưa xác nhận, không còn child/socket — **PASS** đúng thiết kế.
- Inventory laptop **95 file đúng hash**, model app đúng pin. Gói source
  **442 file**, gồm 364 file R5 nguyên byte và journal app mới.
- Native không đổi nên các fixture đã build ở lượt Esc vẫn cùng source;
  gói mới vẫn build lại binary/CTest riêng. Không dùng QA source thay cho
  kiểm gói đóng cuối.

Các log QA cũ trong báo cáo timeout là baseline trước sửa. Test matrix hiện
hành đã đổi kỳ vọng ở các nhánh được sửa, không sửa guard STOP mới để làm
test đạt. Các lỗi fixture của lượt trước vẫn giữ trong evidence lịch sử.

## Đồng bộ và cách thử

Đã sync LAN `10.42.0.185`, build hai package tại thư mục riêng
`/home/pi/fire_robot_desktop/f63e38d71f5e/ros`, xác minh **103 file Pi** và
receipt hợp lệ; đã chọn current sau kiểm hash/lock/process. Xem `selection.json`
và `pi_sync.log` trong evidence. Không tự mở/đóng app người dùng, không chạy
robot, motor/bơm, NTP, flash hoặc commit/push trong lượt sửa này. Pending cũ
giữ nguyên để thao tác Xác minh dừng thật lấy proof mới.

1. Đóng app cũ. Sau sync, rút LAN và nối WiFi Pi như profile vận hành hiện có.
2. Mở app mới, bấm **Xác minh dừng trên robot** một lần. Đây là thao tác I/O
   và STOP thật, cần người dùng giám sát xe/bơm; không đặt goal hoặc bấm chạy
   chữa cháy cùng lúc. Nhập mật khẩu trong app nếu được hỏi.
3. Chờ audit thành công và **IDLE**. Sau đó mới bấm Quét map hoặc chọn map rồi
   Khởi động chữa cháy. Agent chưa thực hiện bước này trên phần cứng.
4. Nếu vẫn BLOCKED, giữ nguyên log mới để đối chiếu. Mất kết nối, group còn
   sống, hồ sơ ownership không đầy đủ, thiếu telemetry STOP hoặc cleanup mới
   lại lỗi vẫn phải bị chặn; bấm xác minh không phải kill-all hoặc bỏ guard.

Chưa nghiệm thu vòng timeout → phục hồi → khởi động lại trên robot thật. Gói
không tuyên bố luôn mở khóa được mọi lỗi hoặc đã đủ điều kiện bảo vệ đồ án.
