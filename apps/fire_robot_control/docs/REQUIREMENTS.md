# Yêu cầu đã thống nhất

Ngày: 29/10/2026. Trạng thái: **đặc tả để lập kế hoạch; chưa implement**.

## 1. Yêu cầu user và truy vết QA

| ID | Yêu cầu đã chốt | Tiêu chí quan sát được | QA liên quan |
| --- | --- | --- | --- |
| R01 | App nằm ngay trong repo, tại `apps/fire_robot_control/` | Source app không nằm trong QA backup; code ROS cũ không bị chép đè | ENV01, ENV02 |
| R02 | Giao diện trắng–đỏ–xám theo tinh thần màu PTIT | Nền sáng, chữ rõ, khung xám, điểm nhấn đỏ; không tự nhận màu/logo chuẩn chính thức | UI01, UI02 |
| R03 | Map và camera cùng một cửa sổ, nhận ra RViz quen thuộc | Có RViz thật, Displays/Views/tool Pose/Goal; không nghiệm thu bằng ảnh render giả | RV01–RV04 |
| R04 | Nối WiFi Pi trước rồi mở app bằng chuột | Có launcher desktop sau cài đặt ban đầu; không cần Internet | ENV03, ENV04, ENV07 |
| R05 | Nhấn Quét map để khởi động các phần cần thiết | Hiện tiến độ, chỉ báo sẵn sàng sau dữ liệu thật; không chạy Nav2 đồng thời | PROC01, PROC03, MAP01 |
| R06 | Lái khi quét vẫn bằng terminal được | App tự mở terminal có TTY; teleop giữ luồng/watchdog đã có | MAP02, MAP03 |
| R07 | Lưu map bằng nút trên app | Nhập tên, lưu khi SLAM còn sống, xác minh đủ YAML/PGM rồi báo thành công | MAP04–MAP08 |
| R08 | Chọn map vừa lưu hoặc map có sẵn | Có danh sách, thấy đường dẫn nếu trùng tên, dùng đúng cặp file đã chọn | MAP09–MAP12 |
| R09 | Khởi động chữa cháy bằng nút | Tự quản lý pairing/runner Pi–laptop, Nav2/YOLO/camera/operator; không nhập lệnh/pairing thủ công | PROC04, PROC05, LIVE02 |
| R10 | Chuyển quét → chữa cháy có kiểm tra | Dừng teleop/SLAM, xác nhận nhả quyền và cleanup trước khi bắt đầu phiên mới | PROC06, PROC07, MAP13 |
| R11 | Giữ Initial Pose và đặt goal bằng RViz | Chờ người dùng đặt Pose, báo lý do chờ; chỉ thông báo sẵn sàng theo report thật | RV05, OBS11, LIVE02 |
| R12 | Dừng tác vụ tương đương X; dừng phiên tương đương Q/Esc | Nút đi vào operator hiện có; có phản hồi và trạng thái STOP xác nhận/không xác nhận | CTRL01–CTRL08 |
| R13 | Hiển thị hành vi, YOLO, IR, bơm/servo, Nav2, kết nối | Giá trị lấy từ source/report; phân biệt mất dữ liệu với không có lửa | OBS01–OBS12 |
| R14 | Giữ logic robot và các guard hiện hành | Không thêm owner motor/bơm, không đổi QoS/watchdog/network/firmware để làm UI | ENV08, CTRL09, LIVE01 |
| R15 | UI dễ đọc và không phủ kín camera bằng mã lỗi | Một hành vi/lý do chính; chi tiết mở rộng; trạng thái vẫn hiểu khi không phân biệt màu | UI02–UI05 |
| R16 | Giữ hành vi phục hồi R5 | Mất serial giữ Nav2/định vị/camera theo runner, không tự phát lại goal/P hoặc bắt Pose lại | REC01–REC05 |
| R17 | Ưu tiên cách làm ít ảnh hưởng runtime | Tái sử dụng runner/proof, làm và kiểm theo mốc nhỏ; log được truy vết | PROC08–PROC12, PERF01–PERF03 |
| R18 | Lượt hiện tại chỉ viết plan/tài liệu QA | Không tạo source app hoặc thay đổi runtime; mọi ca QA app còn NOT_RUN | DOC01–DOC04 |

`R03`, `R09`, `R12` là mục tiêu cuối. Có app mở được hoặc launcher thay terminal
nhưng camera/RViz còn hai cửa sổ chỉ được ghi **mốc trung gian**, không gọi hoàn tất.

## 2. Workflow người dùng

### 2.1. Đã có map

1. Laptop đã nối AP Pi. Mở Fire Robot; app không tự kích hoạt robot lúc mở.
2. Chọn map từ danh sách và kiểm tra tên/đường dẫn/preview nếu có.
3. Nhấn Khởi động chữa cháy; app thực hiện quy trình runner đã được kiểm.
4. Đặt Initial Pose trong RViz theo map đã chọn; chờ báo AUTO sẵn sàng.
5. Đặt goal. Robot giữ nguyên automatic R5, kể cả xử lý lửa khi chưa có goal.
6. X/Dừng tác vụ hủy tác vụ hiện tại; Q/Esc/Dừng phiên kết thúc qua STOP drain.

### 2.2. Quét map mới

1. Nhấn Quét map; app mở phiên thiết bị cần thiết, SLAM, RViz và terminal teleop.
2. Người vận hành lái bằng terminal; Quét map không phải tự khám phá phòng.
3. Nhấn Lưu map khi SLAM còn hoạt động, nhập tên; chờ xác minh hoàn tất.
4. Map vừa lưu tự được chọn; vẫn cho đổi sang map cũ trước khi khởi động.
5. Nhấn Khởi động chữa cháy: app kết thúc phiên mapping có kiểm chứng, sau đó
   mở phiên R5 mới với map đã chọn. Không đóng SLAM trước khi save hoàn tất.
6. Nếu không xác nhận dừng/cleanup, giữ trạng thái lỗi rõ ràng và chặn mở trùng.

Đổi map giữa hai phiên là workflow bình thường. Không hot-swap map trong phiên
Nav2 đang chạy ở phiên bản đầu; kết thúc phiên cũ, mở phiên mới, đặt Pose mới.

## 3. Bố cục và màu

- Thanh trên: tình trạng Pi, Quét map, Lưu map, Chọn bản đồ, Khởi động chữa cháy.
- Vùng trái khoảng60–70%: RViz thật, map/scan/TF robot và công cụ Pose/Goal.
- Vùng phải khoảng30–40%: camera YOLO giữ tỉ lệ ảnh; hành vi chính, một lý do
  chờ, IR, bơm/servo và mục Chi tiết/Nhật ký.
- Thanh dưới: camera, serial, IR, odom/TF, Lidar; X và Q/Esc luôn thấy.
- Palette đề xuất cho UI, không phải brand guideline: nền `#FFFFFF`, khung
  `#F2F3F5`, chữ `#252525`, đỏ nhấn `#B62025`, đỏ dừng `#971B20`.
- Map/camera giữ màu dữ liệu gốc. Trạng thái có chữ/biểu tượng ngoài màu;
  xanh/vàng được dùng tiết chế cho kết nối/cảnh báo.
- Kiểm tối thiểu1366×768,1920×1080, scale100/125/150%; splitter điều chỉnh được.
- Không lấy ADC, FPS15 hay đường xanh trong ảnh minh họa làm dữ liệu mặc định.
- Không tự thêm logo PTIT nếu chưa có asset được xác định nguồn sử dụng.

## 4. Những gì phải giữ từ robot

- Pi: thiết bị/gateway; laptop: Nav2, SLAM, YOLO, RViz và điều phối.
- Automatic: không cần A/F/T/I/P; xử lý lửa vẫn được khi chưa có goal.
- Confidence0.15; IR xác nhận3 sequence mới, giới hạn tuổi theo profile500ms.
- Không tái bật cap IR0.30m/15s trong automatic.
- Trước phun mất YOLO3s: giữ dừng/chờ rồi trả Nav2 theo policy thật.
- Mỗi lượt6s từ phản hồi PWM ON ESP; tối đa3 lượt liên tiếp. Reset quota do
  căn/tiến thực sự hoàn tất theo source, không do UI chuyển màn hình.
- Sau OFF xét YOLO/IR ngay, ON mới đợi home/ready; PWM là telemetry lệnh điện,
  không phải bằng chứng nước chảy hay góc vòi thực tế.
- Trong ON, YOLO/IR clear không tự cắt pulse; Q/STOP và lỗi link/actuator thật
  vẫn có hiệu lực. Không áp dụng quy tắc này để bỏ qua X theo handler hiện có.
- Thay serial epoch: dừng/phục hồi đúng runner, không tự reset Nav2/Pose.

## 5. Ngoài scope

Không viết lại mission, Nav2, perception hoặc firmware. Không tự khám phá map,
thêm joystick web, cho phép bật bơm trực tiếp, tự đổi mạng/IP/domain/serial/pin,
tự cập nhật software, tải CDN/font/model khi mở app, hoặc đồng bộ/flash mỗi lần
chọn map. Không di chuyển candidate/snapshot đã phát hành trong task UI.

Việc thay đổi nhỏ ở adapter operator/runner có thể cần để ghép cửa sổ và nút;
phải được khoanh scope, kiểm hồi quy và có packet mới nếu đụng source được pin.
Không diễn giải “giữ node” thành lời hứa không cần tích hợp bất kỳ source nào.
