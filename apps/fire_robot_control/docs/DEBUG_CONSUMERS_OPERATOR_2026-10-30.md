# Điều tra timeout CONSUMERS và STOP chưa xác nhận — 30/10/2026

Phiên laptop `34446a3e-23e3-4999-a6fe-01720c42a6e7`, packet `71c302e930d057910a6475dea6c8aa9e5881802231421632e0842b707bdc55e1`, log `~/.local/state/fire_robot_control/session_cf380142f8ce4603af5d27ae5214295a`. Map `TN1_30_10_2026` giữ nguyên. User xác nhận quan sát xe và bơm đã dừng.

## Bằng chứng thực

- Startup qua POSE, YOLO; ở CONSUMERS thiếu `operator_present`, hết 180 s và báo `STARTUP_TIMEOUT:CONSUMERS`. Có lúc IO/camera/pose/YOLO đạt cùng lúc, nhưng operator không được xác nhận. Không thể khẳng định mọi tín hiệu đó liên tục đạt suốt lượt.
- Log operator có `OPERATOR_GRAPH` lúc 1793321175.132 s và chỉ báo `OPERATOR_STOP` lúc 1793321350.501 s, `cause=Q`, `confirmed=false`, `elapsed_s=175.369789`, `motion_authorized=false`. Log không phân biệt nguồn Q hay ghi dấu từng heartbeat, vì vậy không kết luận user đã bấm Q hay chứng minh trực tiếp mọi bước của giả thuyết bằng log thực.
- Desktop receipt: STOP không xác nhận và cleanup không sạch. Pi remote receipt xác nhận nhóm và 8/8 child thoát, nhưng có `PI_CONTROL_TRANSPORT:STATE`. Laptop nav phải SIGTERM; child audit thiếu terminal event của component container. Bằng chứng nhóm Pi thoát và user quan sát xe đứng yên không thay cho STOP telemetry.
- Đọc `/proc` sau đó thấy 7 PID nhóm và một PID nav child được ghi nhận vắng mặt; kiểm tra mẫu tiến trình robot không thấy tiến trình. Pi không thấy các tiến trình I/O/runner qua SSH. Không có bất kỳ lệnh chuyển động nào được chạy trong điều tra.

## Lỗi app đã tái hiện và sửa trong source

1. `LiveSession` phát `OPERATOR` khi socket operator xuất hiện nhưng `SessionController.tick()` không có nhánh chuyển tiếp event. Tái hiện xác định: gửi event đúng vào controller thì GUI nhận 0 event. GUI do đó không có thông tin để tạo kênh heartbeat operator. Đây là lỗi cụ thể và giải thích hợp lý cho tình trạng operator_present bị thiếu; log thực thiếu dấu event/lease nên mức chứng minh đến đây là code + tái hiện, không khẳng định đủ mọi nguyên nhân cùng lúc.
2. `OperatorAdapter` khi nhận Q do hết hạn trước heartbeat đầu tiên chỉ tick node một lần; những tick sau bị chặn vì lease chưa active, khiến STOP verifier không tiến lên. Đã tái hiện độc lập với FrontendLease thật, rồi kiểm tra node operator R5 trên bus giả sau sửa. Hành vi này phù hợp với khoảng 175 s giữa bắt đầu STOP và báo cáo cuối, nhưng log thực không ghi nguyên nhân lease nên không gán chắc sự kiện đó cho lỗi này.
3. Controller đã lọc ràng buộc session, generation, nonce, đường dẫn socket và phase trước khi chuyển event; gỡ event nếu FINISHED cùng batch. Adapter giữ trạng thái dừng sau Q, tiếp tục chạy verifier, không chuyển tiếp lệnh khác. Không nới lease/heartbeat, không sửa R5, QoS, watchdog hoặc logic dừng motor.

## Kết quả QA và trạng thái triển khai

- Hai ca tái hiện đều FAIL trước sửa và PASS sau sửa. 232 unittest PASS; 102 QA hồi quy GUI, Qt, mapping, operator/bus giả, recovery và runner PASS; thêm 12 ca kiểm gói và phục hồi sau siết điều kiện PASS. Test xuyên suốt supervisor → Qt AppClient → socket operator thật xác nhận heartbeat và Q; không dùng robot.
- Gói app cuối `e6fa1893279ac0bcb43dd752b9b8dc3dff3e34b0fde3458bc57e68aa4361efca` build, CTest và smoke idle PASS; đã đồng bộ Pi vào stage riêng, xác minh 427 source pin, 103 installed pin và receipt tĩnh; laptop 95 installed pin. R5 parent 364/364 hash PASS. Chưa runtime hardware.
- Đã chọn gói mới và mở app ở BLOCKED; pending.json vẫn nguyên byte của phiên cũ. Không chạy phiên STOP hoặc tác vụ robot mới.

## Phục hồi sau cleanup Nav2 bị cưỡng chế

Bản cũ từ chối `forced=true` nên cả nút xác minh STOP cũng không qua preflight. Bản sửa nhận diện riêng đúng mẫu R5_AUTOMATIC với bảy nhóm đã thoát, Nav2 SIGINT→SIGTERM, child audit khớp PID/label/signal, các child khác sạch và lỗi chỉ gồm bốn mã đã xác định. Kiểm tra group PID và child PID vắng mặt, quét thêm process container, khóa runtime và Pi cùng boot/cleanup. Những trường hợp khác, dữ liệu thiếu, process còn tồn tại hoặc PID tái sử dụng vẫn bị từ chối.

Đường mới chỉ cho phép chuẩn bị lượt STOP-only; **không sửa receipt cũ, không tự gỡ pending và không coi STOP cũ thành công**. Một lượt STOP mới vẫn phải độc lập xác nhận zero/OFF/home, cleanup sạch, audit laptop/Pi và kiểm tra lại PID trước khi archive. Kiểm tra chỉ đọc trên phiên và Pi thực với mã mới đạt, chưa chạy lượt STOP-only thực. Test giả lập gồm âm tính (PID còn sống, sai tín hiệu, thiếu audit, sai profile/nhóm, lỗi khác) và test mới STOP không xác nhận vẫn giữ khóa.
