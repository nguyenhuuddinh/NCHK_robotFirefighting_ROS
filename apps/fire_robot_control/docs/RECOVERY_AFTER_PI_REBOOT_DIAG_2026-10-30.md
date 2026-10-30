> **Mới nhất — a387d27e đã sửa và sync:** Nhánh phục hồi sau Pi reboot đã
> qua 205 unittest / 158 ca tích hợp / CTest; chuẩn bị chỉ đọc trên Pi thật đạt.
> R5/pending giữ nguyên. App mới đã mở BLOCKED, chờ user xác minh STOP mới;
> chưa I/O hoặc nghiệm thu robot. [Báo cáo và bằng chứng](PI_REBOOT_RECOVERY_FIX_2026-10-30.md).

# Kiểm tra click và phục hồi sau Pi reboot — 30/10/2026

**App đã nhận click; đường phục hồi bị chặn bởi boot Pi đã đổi.** Chưa sửa
code, xóa pending hoặc chạy I/O/STOP/actuator trong lượt chẩn đoán này. R5 giữ
nguyên. User đã thử 6 lần RECHECK và 1 lần xác minh dừng trên app 59cddc99.

## Bằng chứng

- Log thật từ file descriptor của supervisor 273082:
  `/tmp/fire-ui-wwbf8ve4/gui.log`. Có 6 UI_RECHECK_REQUEST_SENT, recovery attempt
  tới 6/FAILED, sau đó STARTING/PREFLIGHT→CHECKING_OLD_SESSION→BLOCKED.
- Hồ sơ mapping 6734 cũ: robot_stop_confirmed=false. Kiểm tra lại chỉ đọc sẽ
  luôn dừng ở RECOVERY_STOP_NOT_CONFIRMED, không tạo xác nhận mới.
- Lần user chọn xác minh mới có receipt 1839e853 trong
  `session_69047655ad6e40f88658514d67667bb6`: session_was_started=false,
  primary_error=ValueError:CHECKING_OLD_SESSION. Chưa khởi động Pi I/O.
- Đã gọi lại **prepare chỉ đọc** bằng đúng source artifact 59cddc99, không
  gọi worker live: lỗi gốc là
  `RECOVERY_PI_CHECK_FAILED:RECOVERY_PI_BOOT_CHANGED`.
- Lúc chẩn đoán, WiFi Pi đúng route 10.0.0.1→wlp8s0/src 10.0.0.253; LAN DOWN.
  Vì vậy lần chặn này không phải do chưa nối WiFi Pi. Laptop có thêm USB
  network phục vụ Internet; không thay cấu hình mạng.

## Hai finding thuộc app

1. **Nhánh xác minh sau reboot chưa được hỗ trợ đầy đủ.**
   `services/stop_recovery_runner.py:40` gọi remote_proof của hồ sơ cũ;
   `services/recovery_probe.py:147` yêu cầu boot hiện tại bằng boot của phiên
   cũ. Pi đã reboot thì prepare bị từ chối trước khi lấy STOP proof mới.
   Audit mới cũng kiểm remote_proof(binding cũ) nên chỉ bỏ check ở prepare
   chưa giải quyết đúng cả luồng. Đây là thiếu nhánh phục hồi; không phải
   bằng chứng R5 hoặc ESP32 đã hỏng. Không bỏ identity/lock/process check
   hoặc tự coi reboot là xác nhận motor/bơm dừng.
2. **Lỗi gốc bị che trên UI.** `services/live_session.py` chỉ giữ text của
   RuntimeError; ValueError từ prepare bị đổi thành tên exception + stage.
   failure_message cũng rút mã outer RECOVERY_PI_CHECK_FAILED, nên boot-change
   chưa được diễn giải cụ thể. Cần bảo toàn mã RECOVERY được allowlist, không
   đưa raw SSH/binding/secret ra màn hình.

Hướng sửa cần kiểm riêng: cho phép chứng minh Pi ở boot mới đang quiescent,
không có owner/group robot và lock rảnh; sau đó chạy một phiên STOP-only mới
có giám sát để lấy đủ telemetry/cleanup hiện tại. Chỉ archive pending sau
proof mới gắn đúng session/packet/current boot. Giữ lịch sử thất bại và R5
nguyên; không xóa pending hoặc nới gate để làm hết BLOCKED.

Cần QA same-boot, reboot, boot đổi giữa hai lần probe, foreign binding/PID
reuse, lock bận, thiếu DONE/cleanup, mạng lỗi và STOP proof mới thiếu; không
coi kịch bản reboot đã được QA PASS từ bộ test laptop trước. Chưa triển khai
hướng sửa hoặc tự thử phần cứng trong lượt này.

## Đính chính đường log evidence sync

File app_launch_verified.json trong evidence sync cũ ghi gui_log
`fire-ui-kpzf6sca` vì helper duyệt nhiều wrapper log và ghi đè bằng một log
BLOCKED cũ. PID/candidate supervisor đã kiểm thuộc app mới, nhưng đường GUI
log trong file đó không phải log app mới. Đợt này lấy trực tiếp từ fd của
supervisor, dùng wwbf8ve4 và lưu các event thật. Không sửa checksum/raw đã
niêm phong để che nhầm lẫn; dùng addendum này và evidence mới khi đối chiếu
click. Log thật vẫn có trạng thái BLOCKED/attempt0 trước các thao tác user.
Kết quả sync/source/installed/hash không dựa vào đường GUI log nhầm đó.

[Evidence mới](qa_evidence/recovery_pi_reboot_20261030/) có event, mã lỗi,
receipt summary và SHA256SUMS. Pending thật vẫn giữ hash a13a0874… như trước.
