# Phục hồi cùng Goal sau khi Nav2 tạm ngừng phát vận tốc

Gói cuối: `62c78ddf92eacb5d800f0b8137eeb1223302ad6e2ee9f557bddf0558e5ad0e4b`.
Evidence: `qa_evidence/nav_wait_fix_20261031/` (local, không đưa raw vào Git).
Phân tích phiên thật: [QA trước sửa](QA_NAV2_STALL_RECOVERY_2026-10-31.md).

## Lỗi đã tái hiện và cách sửa

Trong recovery Wait, publisher Nav2 có thể còn tồn tại nhưng không phát Twist.
Gateway R5 dừng, xác minh zero rồi nhả raw. Khi lệnh mới trở lại cùng UUID,
app trước đây vẫn chờ terminal/Goal mới, còn UI tiếp tục báo Goal active.
Cùng phép thử hộp đen trên gói 33f9f7db FAIL: x vẫn 0,036 m sau 5 giây lệnh
mới. Bản sửa được kiểm lại với cùng phép thử, không gửi Goal hoặc reset Nav2.

Sửa riêng trong wrapper app:

- `services/app_nav_recovery.py`: chỉ mở lane khi lần STOP đầu tiên có đúng
  blocker vận tốc Nav2 quá hạn, với lệnh trước đó hữu hạn/đúng giới hạn,
  cùng Goal active và đầy đủ nguồn/ARM/identity/phiên hợp lệ. Lỗi odom, health,
  topology, quyền raw hoặc command sai không được đổi thành command-gap.
- R5 vẫn STOP và nhả publisher. Lệnh vận tốc mới phải đến sau lần giữ dừng;
  cần đủ STOP/odom đứng yên, actuator OFF hiện hành, không owner raw/pump,
  ARM và mọi preflight tốt liên tục 1 giây. Callback gián đoạn không được
  cộng vào khoảng ổn định. Không dùng lệnh cache hoặc gia hạn timeout.
- Khi nhận lại quyền, reset riêng `ever_forwarded` để R5 xác minh discovery
  và operator ACK cho GID publisher raw mới. Kiểm lại lease và STOP/OFF
  tới ngay trước lệnh di chuyển đầu tiên. Không gửi lại NavigateToPose,
  không tạo kết quả hủy giả, không tiêu thụ UUID thành Goal mới.
- `services/app_task_recovery.py`: lease gắn với instance operator, số lần
  X, runtime generation, boot/serial identity và GID nguồn đã pin. X/Q,
  Goal thay đổi/terminal, phiên đóng/fault, nguồn hoặc identity thay đổi
  không được dùng ticket cũ để tự tiếp tục. Lane pause identity cũ không
  được bỏ qua proof của command-gap khi hai nhánh trùng nhau.
- Trạng thái giữ dừng cũ được bỏ khỏi UI sau khi lane terminal/X đã xác
  minh đủ điều kiện nhận Goal mới. Đây là lỗi hồi quy được QA cũ bắt trong
  lúc sửa; không bỏ kiểm tra chỉ để mở nút Goal.
- `native_rviz/navigation_recovery_label.hpp`, `main.cpp`: phân biệt chờ
  lệnh mới, xác minh phục hồi và giữ dừng cần X/Q. Giữ khóa công cụ Goal
  trong lúc phục hồi. Log mới `APP_NAV_STOP` ghi blocker khi bắt đầu STOP;
  `APP_NAV_RECOVERY` ghi chuyển trạng thái, không ghi từng tick hoặc mật khẩu.

Mất tín hiệu thật hoặc lỗi cứng vẫn cần xử lý đúng nguyên nhân. Lane này
không phải nút bỏ mọi khóa. Nếu Nav2 đã ABORTED/CANCELED/SUCCEEDED, xử lý
qua terminal thật và X/Goal mới như trước; không tự phát lại Goal đó.

## Bảo toàn R5 và mapping

364 file source R5 nguyên hash; parent manifest vẫn
`0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0`.
Mapping runner vẫn SHA
`7d769dce6f0170a7ee8c0ad798009593755202023befefef64f5659a990a2503`.
Không đổi lệnh R5 CLI, watchdog, deadline an toàn, topic/QoS, TF, firmware,
model `best_minireplay.pt`, tốc độ quét 0,1/0,8 hoặc tham số Nav2 của mốc
33f9f7db (global/local inflation 0,25/0,30 m, scaling 5/3).

## QA và đóng gói

Kết quả hợp nhất cuối: **924 PASS, 2 SKIP, 0 FAIL**, 926 test ID duy nhất,
gồm **112 ca phục hồi mới**; xem `summary.json`. Chuẩn hóa prefix classname
do pytest đổi root khi nhận file /tmp trước khi hợp nhất, không cộng rerun.
Hai SKIP là nhánh không có Goal trong fixture bắt đầu không có Goal.
Các XML/log thất bại trước sửa và lỗi setup được giữ nguyên trong evidence.

Lượt rộng: 913 PASS/7 FAIL/2 SKIP, trong đó 7 FAIL do chưa tạo thư mục cha
health/evidence. Lượt acceptance trung gian bắt 6 regression khóa UI sau X
và 8 ca IPC bị sandbox chặn. Sau sửa thật và chuẩn bị đúng môi trường:
**334 PASS/2 SKIP** cho các nhóm bị ảnh hưởng trên source cuối; thêm 4 ca
health, 4 ca event-status/Wait, 3 ca kiểm log chuyển trạng thái và smoke gói
đều PASS. Số cuối ở trên đã loại trùng các ca này.

- Ca mới dùng node app thật trên bus trong bộ nhớ, cấm `rclpy.init`: Wait
  im lặng 1,25/5/30 giây, lặp Wait, zero keepalive, cùng UUID không phát lại
  action status, dữ liệu cache, proof không liên tục, trễ query, X/Q trong
  các bước chờ/xác minh/tạo lại publisher/đã chạy; X xong nhận Goal mới.
- Proof âm: đổi phiên/epoch/generation, Goal khác/trùng/terminal, lửa,
  actuator ON/stale/sai contract, cmd không zero, odom chuyển động/stale,
  ARM false/stale, health/contract sai, nguồn thiếu hoặc xung đột raw.
  Có kiểm ticket đã bị thu hồi không sống lại khi dữ liệu cũ trở lại.
- Hồi quy cũ gồm X/lửa/IR/phun/retirement, transport hủy, Goal đổi trước X,
  hết timeout, cleanup, map/lưu/chọn map, Esc/Goal và Q, UI/IPC/auth và
  heartbeat. Bộ rộng được chạy, sau các sửa cuối chạy lại các nhánh bị ảnh
  hưởng. Lỗi thiếu thư mục evidence/health và IPC bị sandbox chặn được chạy
  lại trong môi trường phù hợp; không sửa production để che lỗi môi trường.
- Native build/CTest đạt. Artifact có 451 pin, nguồn R5 cũ nằm nguyên trong
  gói. Static laptop kiểm 95 installed file; Pi kiểm 103 installed file,
  build hai package trong stage app riêng. Chi tiết lựa chọn bản và receipt
  nằm trong `selection.json`.

Đã đồng bộ qua LAN `10.42.0.185`, Pi build hai package trong 9,00 s tại
`/home/pi/fire_robot_desktop/62c78ddf92ea/ros`. Receipt đúng packet, không
ROS launch/NTP/flash/actuator. Sau khi xác minh không có pending và giữ
common lock laptop, đã đổi `artifacts/current.txt` từ 33f9f7db sang 62c78ddf.
78 file runtime app trong services/config/native khớp artifact; ba source
Qt fixture cũng khớp. Không sửa gói cũ hoặc tag khôi phục, không tự mở lại
app đang dùng. Cửa sổ cũ phải đóng rồi mở lại mới nhận bản này.

Không chạy xe/bơm hoặc thử vật cản thật trong lượt này. Việc dừng đầu tiên
của phiên 00:59 rất khớp command gap trong Wait nhưng raw cũ thiếu trace
gateway từng tick; bản sửa xử lý cơ chế đã tái hiện, chưa chứng minh mọi
nguyên nhân đứng xe ngoài thực tế đã hết.

## Thử có giám sát sau khi mở app mới

Đóng app cũ và mở lại để dùng đúng gói. Chọn map, khởi động, đặt Initial Pose
và chờ READY. Thử một Goal có lối đi rõ; khi Nav2 tự dừng/chờ vì vật cản,
UI phải báo giữ dừng/chờ lệnh mới. Nếu đường thông và Nav2 phát lại lệnh
hợp lệ, cùng Goal có thể tự tiếp tục sau khi proof mới đạt. Không đặt người
hoặc tay sát xe trong khoảng chờ vì hành vi này là tự tiếp tục.

Bấm X trong lúc chờ phải hủy tác vụ, xe không tự chạy lại Goal vừa hủy;
chỉ đặt Goal mới khi app cho phép. Q cuối lượt phải STOP và cleanup sạch.
Nếu vẫn đứng, giữ log `APP_NAV_STOP`/`APP_NAV_RECOVERY` cùng nav/operator/Pi
để phân biệt controller không phát lệnh, preflight giữ dừng và lỗi driver.
Không xóa pending hoặc bỏ guard để ép khởi động.
