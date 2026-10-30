# Sửa Q/cleanup khi Nav2 đang tự reset

Lỗi đã có repro: packet 5054 Q đơn hoặc Q lặp đúng đầu Nav2 bond reset làm
container crash -6, dù hai SHUTDOWN RPC success. Child audit đã chặn đúng.
[QA trước sửa](QA_Q_DURING_NAV2_RECOVERY_2026-10-30.md) giữ nguyên log lỗi.

## Thay đổi app

Chỉ sửa `services/nav_session.py` và thêm `services/nav_teardown.py`:

1. Sau khi operator/mission đã đi qua STOP, helper đọc GetState của tám node
   Nav2. Chỉ gửi SHUTDOWN khi các nhóm ổn định ACTIVE; nhóm đã FINALIZED được
   bỏ qua để không chuyển trạng thái lần nữa. Trạng thái thiếu, transitioning,
   unconfigured hoặc inactive giữ chờ trong **cùng hạn RPC cũ**.
2. Quan sát theo lô tối đa 5 Hz, mỗi lô chờ tối đa 250 ms. Cần trạng thái hợp lệ
   ổn định tối thiểu 0,5 s; mất mẫu quá 350 ms xóa khoảng ổn định đã tích lũy.
   Đây là điều phối teardown, không phải readiness hoặc bằng chứng STOP vật lý.
3. Sau phản hồi SHUTDOWN success, kiểm tra từng node thực sự FINALIZED trước khi
   chuyển sang nhóm tiếp theo rồi SIGINT. ACK của manager không đủ để báo dọn sạch.
4. Thiếu phản hồi, không hồi phục hoặc không finalized trả lỗi; giữ nguyên
   fallback SIGINT/SIGTERM/SIGKILL, process-group ownership và child audit.

Không dời lệnh STOP/false-arm. Helper riêng vẫn bị outer owner giới hạn.
Các budget **RPC 12 s, launch INT 8 s, TERM 3 s, outer INT 29 s** nguyên giá trị.
Không tăng bond timeout, nới freshness/STOP/watchdog, đổi QoS hoặc sửa native R5.

Nguyên nhân bộ nhớ sâu của SIGSEGV chưa có backtrace. Bản sửa điều phối app tránh
window gây crash đã tái hiện; không suy rộng thành sửa mọi lỗi native Nav2.

## Kiểm thử sau sửa

Log/receipt thô: [nav_teardown_fix_20261030](qa_evidence/nav_teardown_fix_20261030/).

**291 unittest PASS**, gồm 13 ca mới: không shutdown trong reset, cần trạng thái
ổn định, mất mẫu/clock bất thường, missing/stuck service, false ACK, ACK nhưng
node không finalized và không chuyển trạng thái lại các node đã finalized.

Nav2 thật với TF/odom/scan giả, localhost domain 216, không debugger:

| Ca | Thời gian teardown | Kết quả |
| --- | --- | --- |
| Q đơn ngay đầu reset sau pause 6 s | 6,691 s | Child 1/1 sạch |
| Q ba lần ngay đầu reset | 6,698 s | Child 1/1 sạch, một helper |
| Q sau resume 2 s, trong recovery | 4,782 s | Child 1/1 sạch |
| Q khi goal đang chạy | 2,823 s | Child 1/1 sạch |
| Q sớm khi stack đang khởi động | 4,880 s | Child 1/1 sạch |
| Q sau hai manager phục hồi ACTIVE | 3,678 s | Child 1/1 sạch |

Sáu ca đều không process sống sót, không signal escalation, không warning
`No transition matching`. Mỗi ca có đủ GetState FINALIZED cho cả hai nhóm trước
SIGINT. [Tổng hợp native](qa_evidence/nav_teardown_fix_20261030/native_cases.json).
Các số đo này chỉ về thu dọn Nav2 trên laptop, không chứng minh thời gian dừng
motor/bơm thật.

**236 kiểm thử tích hợp PASS**, 137,86 s: giao diện/IPC, quét/lưu/chọn map,
STOP/recovery, 48 ca Q/X khi fault với telemetry liên tục/chậm/mất. Một warning
Python thư viện cũ, không failure. [Log đầy đủ](qa_evidence/nav_teardown_fix_20261030/pytest_full.log).
291 unittest mất 15,696 s; [log](qa_evidence/nav_teardown_fix_20261030/unittest_final.log).

Đã đóng gói và chọn packet
`49c68b77300f62e6a0b27dd02e50096765e64c1d43030860821e91906964f6fd`,
**441 source pins**, parent app `5054a43e…`. Qt CTest 1/1 và smoke binary 1/1
PASS. Operator trong gói chạy trên localhost 211 không có telemetry vẫn trả
rc2/STOP false, dọn socket/helper sạch; không tự tạo proof.

Chạy lại ca Q ba lần ngay đầu reset **từ artifact đã đóng gói** cũng PASS:
một helper, đủ tám FINALIZED, child 1/1 sạch, không process sống hoặc fallback.
Đây là kiểm gói cho một ca đã có trong ma trận, không cộng thành ca độc lập mới.

Đã sync tĩnh qua LAN Pi `10.42.0.185`, build hai package **9,03 s**. Receipt
**103/103 installed Pi**, **95/95 installed laptop**, source hashes đúng.
`selection.json`, `pi_static_receipt.json`, `laptop_inventory.json` và
`protected_package.json` ở thư mục bằng chứng. Không tự mở app hoặc chạy robot.

## Phạm vi bảo toàn

364 file release R5 `0c54d9b7…` và bản sao candidate phải khớp từng hash. Không
sửa lệnh R5 terminal, Pi I/O/watchdog, firmware, map TN1, model
`best_minireplay.pt`, phần quét/lưu map, startup/recovery policy hoặc GUI.

Pending cũ không được sửa/xóa bằng QA localhost. Lượt này không chạy robot,
motor/bơm, NTP hoặc flash. Đã cài tĩnh; còn một lượt nghiệm thu có giám sát trên
WiFi Pi: xác minh STOP cũ, khởi động với map, theo dõi recovery và Q, đối chiếu
receipt/child audit. Không lấy build/hash hoặc localhost PASS thay kết quả đó.

Để dùng bản mới, đóng app cũ và mở lại **Fire Robot Control**. Khi thử phiên robot,
chuyển sang WiFi Pi/rút LAN theo preflight hiện có. Nếu còn BLOCKED từ phiên cũ,
dùng **Xác minh dừng trên robot**; Kiểm tra lại chỉ đọc không thay thế STOP proof
mới. Chỉ bắt đầu phiên mới khi app đã về IDLE.
