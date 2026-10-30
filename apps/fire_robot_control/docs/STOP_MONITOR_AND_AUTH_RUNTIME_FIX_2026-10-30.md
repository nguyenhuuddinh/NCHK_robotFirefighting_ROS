# Sửa xác nhận STOP và hộp xác thực — 30/10/2026

**Bản cuối `fe47caf63090` đã QA, đồng bộ Pi và mở ở BLOCKED.**
218 unittest, 161 ca tích hợp và CTest đạt. Chưa nghiệm thu STOP trên phần
cứng bằng bản cuối vì lượt agent bấm xác minh chưa được nhập mật khẩu sudo;
Pi hết hạn trước khi mở I/O. Giữ nguyên pending, không mở khóa bằng tay.

## Nguyên nhân từ lượt user thực tế

Laptop `session_40de6d2e7a434ac9ae8efd8e429d3a44`, session
`026a9e4f-37d0-4db9-9384-8f07928fb978`, packet `857b9eb3…`.
Pi `/home/pi/qa_logs/r5_automatic_20261030_050642`, session
`2d9ba06e-4ec1-41be-85f4-3d283e228e85`.

- Pi I/O cuối hợp lệ, ổn định 3,3076 s; laptop I/O hợp lệ, ổn định 1,8036 s.
  Không có `error.json` trên Pi. DONE rc0, 8/8 child sạch, không ép dừng.
- `laptop/operator.log` có đúng một `OPERATOR_STOP`: confirmed=true,
  `FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED`, elapsed 0,4213 s, không fault/blocker,
  không cấp quyền chạy hoặc phát lại phun. Đây là xác nhận telemetry, không
  thay thế quan sát động cơ/bơm thực tế.
- Bộ đọc log R5 chỉ nhận logger `fire_robot_operator`. App cố ý dùng logger
  riêng `fire_robot_app_stop_monitor` để không giả thành nguồn operator của
  R5. Vì chưa có bộ đọc riêng, cleanup bỏ sót dòng STOP thành công và ghi
  `OPERATOR_STOP_UNCONFIRMED`. Replay trên parser cũ nhận 0 receipt → FAIL.
- Khi laptop dọn xong và gửi Pi STOP, SSH Pi có thể thoát trước lúc laptop
  nhận xong DONE/ghi receipt. LiveSession coi mọi Pi exit trong vòng lặp này
  là lỗi khởi động, gây thêm `PI_RUNNER_EXITED_DURING_SESSION` dù Pi DONE sạch.

Vì vậy không thể dùng thông báo tổng quát trong ảnh để kết luận lượt này
lỗi ESP32/TF. Dữ liệu lần này chỉ ra lỗi tiếp nhận kết quả tại lớp app.
Các lỗi TF/USB ở những phiên khác không được coi là đã sửa.

## Thay đổi trong app

1. `services/monitor_log.py`: parser dành riêng cho log monitor thuộc tiến
   trình app. Giữ nguyên kiểm tra inode, nhóm tiến trình, rc0 và yêu cầu
   đúng một receipt đầy đủ của R5. Không đổi tên ROS node, không sửa parser R5.
   Monitor không được nhận receipt mang tên operator R5.
2. `mapping_runner.py` và `desktop_hooks.py`: lắp parser ngay khi khởi tạo
   STOP monitor, trước mọi lần scan. Áp dụng cho quét, xác minh và monitor
   trước khi app chữa cháy chuyển sang operator thật.
3. `live_session.py`: đọc status gắn đúng session trước khi phân loại Pi exit.
   Chỉ hoãn lỗi exit khi SSH rc0 và laptop đã ghi STOPPING. Quyết định cuối
   vẫn bắt buộc receipt STOP/cleanup/binding và process exit hợp lệ; status
   STOPPING riêng lẻ không cho phép IDLE. Nhãn xác minh dùng STARTING_STOP_CHECK.
4. `candidate_build.py`: entrypoint chữa cháy **được sinh riêng cho app** gắn
   session vào startup status và đánh dấu STOPPING trước drain. Giữ failure.json
   riêng để không mất lỗi khởi động. Lỗi ghi status vẫn tiếp tục STOP monitor.
5. `native_rviz/session_ui.{cpp,hpp}`: hộp xác thực dùng `open()` bất đồng bộ,
   vẫn modal với cửa sổ app và che mật khẩu. Không giữ hàm nhận socket trong
   `getText()/exec()`. Hộp gắn session, không mở trùng; tự đóng khi worker kết
   thúc/STOP/mất kết nối/đổi phiên. Đóng tự động không gửi thêm AUTH hoặc STOP
   sang phiên đã hết. Hủy của người dùng vẫn gửi STOP.

Không sửa emergency stop, watchdog, QoS/topic, TF300ms, clock50ms, firmware,
GPIO, serial, model YOLO hoặc mã nguồn/lệnh R5.

## Lượt agent đã bấm trên app thật

Agent dùng cửa sổ X11 có tên được kiểm tra, quan sát ảnh trước khi bấm nút
**Xác minh dừng trên robot** của bản trung gian `d5b1cdaf…`.
App nhận: PREFLIGHT → CHECKING_OLD_SESSION → STARTING_PI → hộp Xác thực Pi.
Đã gửi yêu cầu user nhập mật khẩu trực tiếp, không đọc/lưu/đoán mật khẩu.

Lượt này là laptop `session_6936561b67f44f8ebb0589bd90b686ab`, session
`561da7b6-e972-4d56-81fd-4e2d5d09e282`; Pi log `r5_automatic_20261030_052047`.
Không nhận được xác thực trong thời hạn. Pi ghi `PI_NTP_REFRESH_FAILED`,
chỉ có app_environment/error/pairing; không generation, không mở I/O.

Qua đó thấy thêm lỗi UI: worker đã ghi receipt kết thúc nhưng hộp xác thực
cũ còn giữ hàm nhận IPC, màn hình vẫn STARTING_PI. Đã đóng hộp hết hạn, xác
nhận không còn Pi runtime rồi sửa UI ở bước 5. Không gọi lượt này là STOP PASS.
Bản cuối đã được mở lại; chưa tự chạy lại một lượt cần nhập mật khẩu khi user
chưa sẵn sàng. Common lock rảnh và pending cũ còn nguyên.

## QA cuối và giới hạn

| Bộ kiểm | Kết quả |
| --- | --- |
| Python unittest | 218 PASS, 8,090 s; 211 cũ + 7 method mới |
| Tích hợp Qt/RViz/runner | 161 PASS, 55,35 s; 160 cũ + ca timeout xác thực |
| Native CTest | 1/1 PASS; cả build gói và frontend suite |
| Parser STOP cũ | Replay logger monitor: FAIL trước sửa; parser app nhận đủ |
| Qt timeout cũ | Probe dùng UI trước sửa: FAIL sau 8,03 s; bản mới PASS |
| Xác thực Qt thật | Accept/cancel/timeout qua worker-controller-AppClient-SessionUi; PTY giả, không SSH |
| Pi static | 427 source/103 installed PASS; laptop 95 installed PASS |
| STOP trên phần cứng bằng bản cuối | PENDING mật khẩu sudo; chưa kiểm chứng |
| Quét/lưu map và chữa cháy thực tế | NOT_RUN trong lượt này |

Ca mới có process thật trên laptop với log giả: thiếu log, receipt trùng,
receipt chưa hoàn chỉnh, confirmed=false, logger lạ, exit rc2 đều giữ FAIL;
receipt hợp lệ duy nhất mới PASS. Kiểm Pi exit đúng/sai session, sai stage,
rc khác0; replay LiveSession trọn vòng với Pi exit trong drain và receipt
đến sau, kể cả receipt thiếu vẫn bị khóa. Không dùng process exit làm STOP.

Các lần QA đầu còn lỗi fixture: status giả thiếu session; AUTH giả thiếu
session. Đã cập nhật fixture theo giao thức thật rồi chạy lại, giữ log FAIL
ban đầu. Không nới kiểm tra production để cho qua test. Probe Qt đã rebuild
từ UI mới trước lần 161 ca cuối. Cảnh báo xacro/Python3.11 hiện có không làm
test fail; không coi bộ này là mọi trường hợp phần cứng đều PASS.

**Finding còn mở ngoài sửa lần này:** relay OPERATOR từ live worker qua
supervisor đã được ghi ở báo cáo auth trước vẫn chưa sửa; chưa nghiệm thu nút
X/operator hoặc nhiệm vụ chữa cháy. Không dùng kết quả STOP monitor làm bằng
chứng cho đường operator thật. Chưa nghiệm thu clock/TF/camera khi full tải.

## Gói và bảo toàn R5

Packet cuối:
`fe47caf630901bdb7e075ac6840340e98de597736f62ba1a243f8fc06bac6b32`.
Stage Pi `/home/pi/fire_robot_desktop/fe47caf63090/ros`, build2package thành công.
Binary SHA `eb3197a9264d92376b7b3b2688838536704e2ed09c8d511c3857b8c20c650d19`.
Archive SHA `c8018683542477a785d12a5173bdd4e8da542e59e2e6a442d7991c2df8b37dd4`.
Receipt Pi SHA `e9f7a46404cacbb03f1e17c6a34d49c64abe8b36221851b00240f2025cca4461`.

364 source của parent R5 `0c54…` còn nguyên trong manifest427. Pi kiểm trước
có49bộ/23262file; sau thêm bản trung gian và bản cuối có51bộ/24316file.
So sánh trước → sau không đổi file/baseline của49bộ cũ; so tiếp bản trung
gian → cuối giữ23789file/50bộ. Baseline Git sạch, không robot process lúc kiểm.
R5 terminal0c54/2484/dfac giữ nguyên. Không sửa ai/, flash/commit/push.

Laptop giữ8842file được bảo vệ; chỉ selector current.txt thay đổi có chủ đích.
Pending giữ SHA `a13a087426d353a234743217da9bbd7fff1857c8b78ce353db5c83453f0ce676`.
Hai bản d5b1 và fe47 tồn tại riêng; không sửa gói đã niêm phong. Build receipt
vẫn ghi runtime=false; không sửa receipt build để tuyên bố phần cứng PASS.

App cuối PID307869, log launcher `/tmp/fire-ui-nwx45y2e/gui.log`.
Chỉ archive pending bằng audit sau khi nhận STOP mới hợp lệ, không xóa tay.

## Tiếp tục nghiệm thu

Giữ WiFiPi, không chạy R5 terminal cùng app. Khi user có mặt, bấm xác minh,
nhập mật khẩu pi trong hộp Xác thực Pi ngay khi hỏi. Chờ STOP telemetry,
local/Pi child audit, đối chiếu boot/session rồi mới được IDLE.
Quan sát robot dừng; nếu đạt, thử Quét map không lái → Dừng phiên → Quét lại.
Chỉ sau đó nghiệm thu lưu/chọn map và các bước chữa cháy còn mở.

[Evidence](qa_evidence/stop_monitor_auth_runtime_fix_20261030/) gồm log FAIL/PASS,
XML, diff, hashes và receipt đã lọc; không chứa pairing riêng hoặc password.
Lệnh chạy QA đầy đủ ở run_release_qa_final.sh, cần ROS overlays và socket/X11
cho Qt; fixture dùng ROS_DOMAIN_ID211/localhost1, không ROS robot. Các script
activate/sync trong evidence là thao tác triển khai, không dùng làm test.
