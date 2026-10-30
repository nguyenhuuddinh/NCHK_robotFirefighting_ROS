# Lượt xác minh 06:31:23 — chưa đạt

Packet app vẫn `fe47caf63090`; chưa sửa source/build/sync trong lượt này.
R5 giữ nguyên. User cho phép tự nhập mật khẩu và thử STOP; không lái hoặc
chạy chữa cháy. Không lưu thông tin xác thực vào evidence.

Ảnh đầu của user ứng với Pi `r5_automatic_20261030_062825`: error
`PI_NTP_REFRESH_FAILED`, không có generation/I/O. Desktop tương ứng là
`session_f3e676bc641c4a0fb1bfd6537353b9c1`.

Agent bấm xác minh tiếp theo. Đã quan sát hộp Xác thực Pi có ký tự được che;
khi thử gửi input thì hộp đã đóng, guard từ chối trước khi phát bất kỳ phím
nào. Không gọi đó là agent nhập mật khẩu thành công. Phiên thực tế đã qua
xác thực/đồng bộ giờ: clock settle14,06s, hai probe trong budget50ms.

Laptop: `session_4789b7d2d1834d99aade5e7198366673`, session
`6c33ec4b-ba0b-4101-b065-32b4da8e279c`.
Pi: `/home/pi/qa_logs/r5_automatic_20261030_063123`, session
`49dab2d4-6d78-4a89-94b5-53fe5a43ec31`.

Quan sát:

- UI đi qua READING_PAIRING → STARTING_STOP_CHECK → WAIT_PI_IO → STOPPING →
  BLOCKED. Laptop lỗi `InterruptedError:MAPPING_STOP_DURING_IO` sau khoảng
  6,5s chờ I/O, chưa phải timeout65s. Log hiện chưa lưu nguyên nhân đầu tiên
  đặt stop_event; chưa kết luận là user/heartbeat/SSH.
- Laptop health không thấy nguồn Pi, STOP monitor hết hạn không confirmed,
  tiến trình monitor sau đó cần SIGKILL. Kết quả local/remote đều không sạch.
- Pi đã mở8child; bridge HEALTHYepoch1, odom/scan/camera phát; health I/O có
  lúc ổn định19,92s. State chính dừng cập nhật ở IO_STARTING; runner PID8600
  biến mất, không error.json/DONE.json, còn hai nhóm I/O và health. Chưa đủ
  trace xác định thời điểm/cơ chế runner mất. Kernel log không thấy OOM hoặc
  USBdisconnect mới trong khoảng thử. Không quy lỗi cho ESP32/TF từ lỗi này.

Agent đã dọn riêng hai nhóm app bằng SIGINT sau khi đối chiếu boot, packet,
runner đã mất, command và start_ticks: startup_health PID9121/birth1187948;
launch Pi PID9120/birth1187945, SIGINT tới leader. Cả hai hết live members;
kiểm lại không có robot process. Nhật ký riêng:
`/home/pi/qa_logs/app_owned_cleanup_20261030_063123/manual_group_cleanup.json`.
Đây chỉ là chứng cứ process exit; không tạo DONE hoặc STOP proof thay runner.
Không xóa pending và không biến lượt lỗi thành PASS.

Mạng kiểm chỉ đọc: LANdown, WiFiwlp8s0 10.0.0.253; unicast10.0.0.1 quaWiFi.
Laptop còn USB mạng điện thoại172.20.10.3; route mặc định239.255.0.1 đi USB.
Đây là dấu hiệu cần kiểm DDS khi có nhiều interface, **chưa chứng minh DDS
thực tế chỉ dùng route đó hoặc đây là nguyên nhân dừng phiên**. SSH thành
công không phải chứng cứ các ROS endpoint đã discovery.

Tiếp theo: đối chiếu launcher/environment/lifetime với R5; bổ sung trace
stopcause/SSHexit/HUP và kiểm lifecycle bằng endpoint giả trước khi lặp robot.
Giữ trình tự launch/node/gate R5, sửa lớp app có bằng chứng; không nới watchdog,
freshness hoặc xóa khóa để vượt lỗi. LAN có thể dùng đọc log/sync, profile
runtime app hiện vẫn yêu cầu rút LAN và chạy qua WiFi.

[Evidence](qa_evidence/verification_063123_20261030/) có receipt/log đã lọc,
clock proof và snapshot mạng; không có pairing riêng hoặc mật khẩu.
