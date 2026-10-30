# QA offline — giao diện và điều phối phiên

> Báo cáo lịch sử của giai đoạn trước tích hợp. Trạng thái hiện tại và kết quả
> mới xem [QA tích hợp](QA_INTEGRATION_2026-10-29.md).

## Phạm vi và kết quả

Đã chạy **98 test Python PASS** cho code app; **1 target CTest Qt PASS**;
**9 case PASS** đưa X/Q qua Unix socket vào node operator R5 trên bus mô phỏng;
**1 case PASS** kiểm tham số ROS trên node giả chỉ localhost; và **47 + 15 test
hồi quy R5 PASS** cho pairing/sequence và STOP/teardown.
Đây là các nhóm kết quả riêng, không tính là nghiệm thu robot hoặc toàn bộ QA
plan. Không có ROS/DDS kết nối robot, SSH, NTP, đồng bộ Pi, flash hoặc actuator
thật.

- Chính sách phiên kiểm tra bấm lặp, save lỗi, dừng khi save, callback cũ,
  pairing sai/lặp/muộn, Pi mất, map bị sửa, X/Q đồng thời và cleanup degraded.
  Có thêm 100 chuỗi giả lập, mỗi chuỗi 100 sự kiện, seed cố định để tái hiện.
- PTY dùng child Python giả; kiểm tra controlling TTY, output chia mảnh,
  nhập không echo trong fixture và giữ child sống cho đến khi caller xử lý.
  Chưa kiểm được prompt SSH/sudo thực, cơ chế an toàn password hoặc remote PID.
- Thành phần tải pairing kiểm tra đường dẫn/packet/ownership do caller cung cấp,
  giữ file `0600` trong thư mục `0700`, từ chối file thay thế hoặc symlink; đã
  thử bằng child giả trên PTY. `assert_owned_pi` là callback do adapter cung cấp,
  chưa tự chứng minh PID/remote receipt. Chưa thử `scp` thật hoặc nối vào runner
  laptop.
- Qt client và Python server được nối thực qua Unix socket: thử X/Q, treo vòng
  lặp Qt, đóng kết nối và nonce sai. Heartbeat đến từ Qt event loop; khi mất,
  socket/lease yêu cầu Q. Kết quả được đưa vào **lõi** operator R5 thật nhưng
  chưa qua node ROS và process runner thật.
- Một nhóm bài khác đưa socket vào **node** operator R5 và các node trên bus mô
  phỏng: Q trong standby, Nav, quay, IR, WAIT_P, pulse và review; X hủy goal
  giả; runner ứng viên cũng chạy với node này và ROS giả. Khi Q, test chờ report
  xác nhận STOP và kiểm tra publisher giả đã nhả.
- Bộ chạy operator ứng viên dùng node/STOP verifier hiện có, chặn tick trước
  heartbeat, đối chiếu session/packet với bootstrap và chạy finally khi có lỗi.
  Việc xác thực manifest/hash vẫn thuộc runner; bootstrap riêng không chứng minh
  packet. Chưa là một entrypoint được đóng gói, chưa được gọi bởi runner hay GUI
  thực.
- Bootstrap operator được truyền bằng file descriptor ẩn danh, chỉ đọc, thay vì
  đặt nonce trên argv. Đã thử process giả và client Qt nhận FD, kể cả đường dẫn
  tiếng Việt có dấu cách; backend ứng viên dùng lại R5
  `OwnedProcess`/registry. Test drain xác nhận receipt giả đúng schema và từ
  chối trường hợp process thoát sạch nhưng không có receipt. Đây vẫn là dữ liệu
  giả, chưa chứng minh STOP thực.
- Node `rclpy` giả trong domain 211 chỉ localhost đã nhận đúng các tham số được
  tạo từ profile R5 (`show_gui=false`, automatic và gate2); không khởi động
  operator thật hay publisher điều khiển.
- `map_catalog` từ chối YAML có khóa trùng để tránh chọn map với dữ liệu mơ hồ.

## Lỗi phát hiện và giới hạn

Khi QA, adapter từng nhận lệnh khác X/Q, yêu cầu shutdown nhưng vẫn đưa lệnh
không hợp lệ vào tick đó. Đã xóa lệnh trước khi tick, thêm test cho A/P/Esc và
chạy lại bộ test đạt. Lỗi này nằm trong code ứng viên, chưa từng bật trên robot.
Một fixture bus mô phỏng ban đầu sai timestamp do số thực; sau khi đồng bộ với
clock giả của harness, các case đạt. Đây là sửa fixture, không phải sửa runtime.

Legacy mapping có thể tự mở Pi detached, còn R5 quản lý Pi bằng phiên/receipt;
chưa có cơ chế chuyển ownership an toàn giữa hai luồng. Runner R5 còn tự mở
RViz/OpenCV, và integration candidate chưa có manifest/receipt tương ứng. Do đó
chưa bật Scan/Save/Start/X/Q trong preview. Packet `0c54…` có Pi receipt PENDING;
không dùng kết quả offline để coi là Pi đã sẵn sàng hoặc STOP đã xác nhận.
PTY của Pi phải thuộc một supervisor sống độc lập với UI: nếu UI tự giữ PTY và
bị crash, đóng terminal có thể làm Pi rời phiên trước khi laptop drain. Cơ chế
supervisor này chưa được nối và phải kiểm trước khi cho chạy thật.

## Tái chạy

```bash
cd '/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS'
bash apps/fire_robot_control/launchers/build_preview.sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=apps/fire_robot_control /usr/bin/python3 -m unittest discover -s apps/fire_robot_control/tests -v
ctest --test-dir /tmp/fire_robot_control_preview_$(id -u) --output-on-failure
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="apps/fire_robot_control:releases/r5_20261029/ros/ros2_ws/src/fire_robot_bringup:${PYTHONPATH:-}" /usr/bin/python3 -m pytest -q -p no:cacheprovider apps/fire_robot_control/tests/qa_full_operator.py
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="apps/fire_robot_control:${PYTHONPATH:-}" /usr/bin/python3 -m pytest -q -p no:cacheprovider apps/fire_robot_control/tests/qa_operator_ros_args.py
```

Kiểm thử socket/PTY cần hệ thống cho phép Unix socket và pseudo-terminal cục bộ.
Nếu Qt fixture chưa build, bốn test liên thông Qt/Python sẽ báo skipped; phải
build trước và kiểm output để xác nhận chúng thực sự chạy.
