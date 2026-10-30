# Phiên quét 19:50:12 bị khóa — 29/10/2026

**Chưa quét map thành công.** Gói app `69dfe74e…`, laptop session
`6f9e9217-1f7f-4561-beb8-2fa58a95cf61`, thư mục
`~/.local/state/fire_robot_control/session_3321e626802943239bbd6501321c8b72`.
Pi session `68cc420d-a712-4f3c-84d1-bc9042d27405`, log
`/home/pi/qa_logs/r5_automatic_20261029_195012`.

User bấm Quét map qua WiFi, sau đó gửi ảnh BLOCKED. Lượt phân tích này chỉ
đọc log local và sáu file log/receipt đúng phiên Pi qua LAN; không chạy lại
ROS, NTP, thiết bị, không sửa source runtime, không đổi packet hoặc xóa khóa.

## Nguyên nhân quan sát được

1. Clock gate đạt `CLOCK_SETTLED`, 13,36 s, hai lượt đo trước ROS trong budget
   50 ms. Không quy lỗi lần này cho việc WiFi Pi thiếu Internet.
2. Laptop nhận I/O hợp lệ rồi yêu cầu SEAL. Lúc yêu cầu, Pi ghi
   `io=false`, blocker `odom_tf`. Sau 20 s không đạt, Pi trả
   `PI_SEAL_HEALTH_TIMEOUT`, chuyển STOP và kết thúc.
3. Snapshot cuối Pi có blocker `odom_tf` và `scan`. TF ghi stamp age khoảng
   4910 ms, source age khoảng 5010 ms, vượt cửa 300 ms đang giữ nguyên.
   `max_stable_s=0` trên Pi. Trong khi đó laptop từng đạt I/O ổn tối đa
   8,516 s; snapshot cuối laptop thấy TF source age khoảng 125 ms.
4. Pi log cho thấy bridge còn HEALTHY, có odom/TF publish tiến triển. Chưa có
   bằng chứng đủ để kết luận hỏng Lidar/encoder hoặc nguyên nhân gốc làm
   observer Pi thấy TF cũ. Cần kiểm tiếp đường nhận TF, thời gian dưới tải và
   sự khác nhau giữa observer hai host; không tự tăng timeout hay nới độ mới.
5. Runner laptop thất bại ở chờ SEAL. Registry local chỉ có operator STOP-only
   và startup_health; chưa khởi động SLAM hoặc bàn phím teleop. App hiện mã
   hậu quả `PI_RUNNER_EXITED_DURING_SESSION`, che mã nguyên nhân cụ thể trong
   `laptop/failure.json`.

Camera raw đã tới bộ kiểm tra laptop (387 frame hợp lệ), nhưng ảnh trống sau
phiên không chứng minh lỗi camera là nguyên nhân SEAL. Các cảnh báo JPEG trong
log chưa được xác định là nguyên nhân TF timeout.

## Dừng và cleanup là hai kết quả khác nhau

- Operator STOP: `FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED`, khoảng 0,430 s,
  `motion_authorized=false`; không phải kiểm chứng dừng vật lý bằng quan sát.
- Local drain: clean=true, hai group thoát, không forced, return code 0.
- Pi DONE: group đã thoát, tám launch child có terminal audit sạch, không
  forced. Kiểm tra read-only sau phiên qua LAN không thấy robot process trong
  danh sách kiểm của runner.
- Tuy vậy `SshPi.stop()` hiện yêu cầu DONE không có runner error mới cho
  `confirmed=true`. `PI_SEAL_HEALTH_TIMEOUT` được giữ trên DONE nên
  remote.confirmed=false, dù các group/child đã dọn sạch. Runner laptop rc=2,
  desktop cleanup_clean=false; controller chuyển BLOCKED và giữ pending.json.
- Receipt còn ghi một lỗi transport STATE; DONE đã được tải độc lập để đối
  chiếu. Không coi exit SSH hoặc snapshot process riêng là toàn bộ STOP proof.

## Hiện có thể bấm quét lại không?

Khi phiên trước kết thúc sạch và controller về IDLE, nút Quét map bật lại;
app tạo session/pairing mới, không dùng lại phiên cũ.

Ở BLOCKED, hiện chưa có lệnh/nút phục hồi để kiểm tra rồi cho thử lại. Đóng
mở app cũng vẫn khóa vì pending marker được giữ. Đây là phần UX phục hồi
chưa được tích hợp; không được hướng dẫn user xóa marker để vượt nó.

Thiết kế cần bổ sung là “Kiểm tra & thử lại”: đối chiếu đúng phiên/packet,
STOP proof, local group và Pi DONE/child cleanup, kiểm tra không còn phiên
cũ/phiên khác sở hữu runtime rồi mới thu hồi marker có lưu lịch sử. Kiểm tra
không đủ hoặc mất kết nối phải giữ BLOCKED. Nút này không tự lái, không tự
phun, không chạy lại vô hạn và không bỏ qua lỗi freshness/startup.

Đồng thời cần hiển thị mã `PI_SEAL_HEALTH_TIMEOUT` cùng blocker TF trong UI,
tách kết quả khởi động thất bại khỏi tình trạng dọn tiến trình. Chưa sửa các
chính sách này trong lượt phân tích ảnh/log; chưa tuyên bố đã có nút phục hồi.

## Bằng chứng

File gốc laptop được sao lưu trong
`qa_evidence/mapping_failure_195012_20261029/laptop_session/`; sáu file Pi nằm
trong `pi/`. Không copy pairing/secret. SHA256SUMS khóa các byte đã đọc.
Các QA offline trước đó giữ nguyên phạm vi, không dùng để nâng phiên runtime
này thành PASS. Trạng thái hiện tại: mapping chưa đạt, pending marker giữ,
nguyên nhân TF freshness chưa chốt, retry BLOCKED chưa có giao diện.
