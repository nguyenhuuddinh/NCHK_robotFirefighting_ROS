# Phân tích lỗi operator thoát khi khởi động chữa cháy — 30/10/2026

Phạm vi: chỉ đọc log laptop và Pi, xem metadata model cục bộ và tái hiện lease bằng Unix socket giả. Không sửa code, khởi động robot, xóa pending, đổi cấu hình R5 hoặc xác nhận STOP bằng suy đoán.

Phiên laptop `9296b1f2-65e7-44ba-a3a2-aa690abd9ea2`, log `~/.local/state/fire_robot_control/session_769d835ae51448bebbd50f4bb45f81e7`; Pi `/home/pi/qa_logs/r5_automatic_20261030_080833`; packet app `e6fa1893279ac0bcb43dd752b9b8dc3dff3e34b0fde3458bc57e68aa4361efca`.

## Trình tự và kết quả

1. Bản đồ đã nạp; AMCL ghi `initialPoseReceived` và đặt pose. Startup tới POSE, YOLO, sau đó CONSUMERS. YOLO đã load và warmup thành công.
2. Operator đã chạy, log `WAIT_ARM: WAIT_VALID_GRAPH_AND_DATA` và phát arm proof; khác lỗi lần trước khi supervisor làm rơi thông báo OPERATOR. Mốc OPERATOR_GRAPH liên tiếp 66589.156958656 và 66589.620974623 monotonic, cách 0,464016 s; Q bắt đầu STOP khoảng 1793322592.580 UTC epoch dựa trên thời điểm report và elapsed.
3. Operator ghi `OPERATOR_STOP`, cause Q, `STOP_UNCONFIRMED` sau 4,172 s. Thiếu bằng chứng nguồn STOP, cmd zero mới, odom stationary mới, identity, actuator OFF/home và dữ liệu ACT mới. Log ghi `motion_authorized=false`, nhưng không thay thế quan sát vật lý.
4. Runner thấy operator exit code 2 và báo `OWNED_CHILD_EXIT:operator`. Khi thu dọn sau đó, Nav2 phải SIGTERM; `fire_pulse_controller` báo `ACTUATOR_OR_MOTION_STOP_UNCONFIRMED` và exit 2 sau tín hiệu dừng. Không coi exit này là nguyên nhân khởi phát trước khi operator thoát.
5. Pi báo remote groups và 8 child đã thoát sạch, DONE error null. Laptop receipt STOP false và cleanup false. Pending session vẫn có, BLOCKED là đúng theo bằng chứng hiện tại.

## Giả thuyết và giới hạn

Kênh operator có lease 350 ms. Loop operator trên laptop thực hiện xử lý ROS và poll socket trong cùng thread. Tái hiện bằng FrontendLease và OperatorChannel thật: heartbeat ban đầu hợp lệ, sau đó 4 heartbeat được xếp hàng đều đặn trong khi bên nhận không poll 463 ms; lần poll tiếp theo trả `Q`, `FRONTEND_LEASE_EXPIRED`. Kiểm thử này chứng minh cơ chế có thể xảy ra ngay cả khi bên gửi vẫn gửi, không phải bằng chứng trực tiếp rằng nó đã xảy ra trong lượt thật. Log lượt thật chưa ghi thời điểm UI gửi/nhận heartbeat, thời gian từng tick hoặc lease.reason, nên không phân biệt được hoàn toàn operator xử lý chậm, GUI ngừng phát, lỗi kênh hoặc Q có nguồn khác. Không nới lease, giả heartbeat hay tắt watchdog.

Pi log còn nhiều cảnh báo `/odom stale` 301–393 ms với ngưỡng TF 300 ms; IO/TF chập chờn có thể ảnh hưởng xác minh nhưng không chứng minh nguyên nhân Q/thiếu STOP. Log báo serial bridge HEALTHY epoch 1 ở các mẫu đã xem, không đủ cơ sở kết luận ESP32 lỗi. YOLO có thông báo ảnh JPEG corrupt; không có bằng chứng nó làm operator Q hay model load thất bại. Lỗi camera/transport cũng còn cần nghiên cứu riêng.

Nút xác minh STOP của bản hiện tại: kiểm tra chỉ đọc `old_groups` với receipt của phiên này trả `RECOVERY_CHILD_NOT_CLEAN`, vì child `fire_pulse_controller` thoát code 2. Không xóa khóa hay sửa receipt. Cần thiết kế recovery riêng cho child này với chứng minh process đã thoát và lượt STOP/OFF/home mới, không đánh dấu lịch sử thành công; không kết luận rằng nhấn nút hiện tại sẽ khôi phục được.

## Model thực tế

Log `yolo.log`: `/media/huudinh/New Volume/esp32/Fire_smoke_person_detection/best_29_10_2026.pt`, SHA-256 `28d3b25f70ccd0ea958a2abfa11f35804f5a82c2f711e4786d5bfa6cce46e17fc0`. Metadata nạp cục bộ xác nhận kiến trúc `yolo11l.yaml`, task detect, classes `{0: fire}`. Lượt này chạy NVIDIA RTX 3050 6GB Laptop GPU FP16, conf 0,15; node YOLO chỉ quan sát. Việc load thành công không phải đánh giá độ chính xác hoặc độ ổn định camera.
