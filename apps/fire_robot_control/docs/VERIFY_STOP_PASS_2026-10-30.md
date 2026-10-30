# Xác minh STOP trên Pi — 30/10/2026

Lượt xác minh STOP qua WiFi đã đạt với app `abf4aae31db8`. App tự đối chiếu
chứng cứ, archive hồ sơ cũ và chuyển BLOCKED → IDLE. Không lái xe, chạy SLAM,
Nav2, YOLO hay nhiệm vụ chữa cháy trong lượt này.

Laptop: `~/.local/state/fire_robot_control/session_d75ddc85b35147f9a27472a98f225a34`,
session `89a94981-55c7-495d-967c-da9873de2a1c`; Pi:
`/home/pi/qa_logs/r5_automatic_20261030_065257`, session
`4a3b973d-d250-4641-8acd-e44af655a42d`.

`OPERATOR_STOP` xác nhận `FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED`, thời gian
0,420s, không blockers/fault, không motion_authorized/pulse_replayed. Receipt
laptop rc0, STOP confirmed, local clean, remote confirmed và cleanup confirmed;
desktop receipt STOP/cleanup true, Pi SSH rc0. Log transport ghi một lần
`PI_CONTROL_TRANSPORT:STATE` trong quá trình chờ, nhưng đã nhận DONE đúng phiên
sau đó; không coi kết nối luôn ổn định. Pi DONE error=null, 8/8 child thoát
sạch, không ép dừng. Sau lượt thử không còn process robot trên Pi.

Đây là bằng chứng telemetry và cleanup, không chứng minh quan sát vật lý động
cơ/bơm. Cần user quan sát xe dừng. Bản đồ, lưu map, điều khiển và chữa cháy
chưa được nghiệm thu bởi lượt này. Relay OPERATOR/nút X còn finding đã ghi
trong `STOP_MONITOR_AND_AUTH_RUNTIME_FIX_2026-10-30.md`; không dùng nhiệm vụ
chữa cháy như đã đạt.

Thay đổi app so với fe47: ghi cause STOP/heartbeat/SSH rc không chứa password,
giữ ngưỡng heartbeat cũ; xử lý SIGHUP của runner Pi đi qua STOP/drain hiện có,
không giả receipt. Ca SIGHUP chỉ đã thử bằng runner/process giả, không cố tình
ngắt SSH trên robot. Lỗi phiên 063123 trước đó chưa đủ bằng chứng chốt nguyên
nhân; xem `VERIFICATION_063123_DIAG_2026-10-30.md`.

QA: 222 unittest, 65 ca tích hợp runner/STOP/cleanup và CTest 1/1 đạt. Pi
build 2 package, kiểm 427 source và 103 installed; 24316 file của 51 bộ Pi cũ
nguyên hash, 1569 file R5 laptop nguyên hash. Gói app dùng parent R5 có 364
source nguyên byte; không thay đổi lệnh R5, firmware, watchdog hoặc ROS topic.
Lần này không lặp toàn bộ 161 ca UI vốn không bị sửa.

Kết nối SSH dùng WiFi Pi. Laptop có thêm USB mạng điện thoại; kiểm định tuyến
multicast ưu tiên USB nhưng chưa đủ kết luận nguyên nhân phiên 063123. Lượt
xác minh mới đã nhận dữ liệu qua ROS; chưa kết luận mạng luôn ổn định.

Tiếp theo: trên app IDLE bấm Quét map, chờ MAPPING/map/Lidar rồi bấm Dừng phiên
và đợi IDLE/STOP/cleanup sạch. Thử Quét lần hai, sau đó mới lái/quét/lưu map
có người giám sát. Nếu lỗi giữ trạng thái/log, không xóa pending bằng tay.
