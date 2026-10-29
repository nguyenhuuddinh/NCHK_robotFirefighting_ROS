cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS"

python3 scripts/stop_laptop_nodes.py
Các file này dùng ROS 2 Humble trong `ros2_ws/install` hiện có. Trên Pi phải
có cùng source ROS và firmware S3 tương thích. SSH mặc định `pi@10.42.0.185`;
qua AP đặt `FIRE_PI_SSH_HOST=pi@10.0.0.1`. Script không tự đổi mạng laptop.

**Demo cập nhật 2026-10-17:** `start_demo.sh` tự kiểm tra/đồng bộ giờ Pi theo
laptop **trước khi mở ROS**, mở Pi launch một lần, và Ctrl-C dừng launch của
phiên trên **cả laptop và Pi**. Nếu giờ đã khớp trong ngưỡng sai số 100 ms,
không cần chỉnh; nếu cần chỉnh có thể hỏi mật khẩu sudo Pi, không lưu mật khẩu.
Không đổi timezone hoặc cấu hình NTP. Xem
[quy trình và kiểm thử](../docs/codex/DEMO_TIME_CLEANUP_2026-10-17.md).

Demo từ chối khi Pi còn launch/node cũ: dừng đúng phiên cũ trước khi chạy.
Không dùng `FIRE_AUTO_START_PI=0` với demo mới, vì demo cần quản lý Pi launch
để đảm bảo cleanup. Log Pi có đường dẫn riêng trong event `PI_SESSION_STARTED`.
Chờ `PI_SESSION_STOPPED` và `SESSION_STOPPED` trước khi đóng terminal/rút mạng;
`PI_CLEANUP_UNCONFIRMED` yêu cầu kiểm tra Pi, không có nghĩa đã dừng thành công.

Mapping vẫn dùng cơ chế cũ: có thể reuse Pi stack, `FIRE_AUTO_START_PI=0` để
tự mở Pi; Ctrl-C mapping không dừng Pi hoặc đồng bộ giờ. Lệnh Pi thủ công dưới
đây chỉ dành cho workflow đó hoặc chẩn đoán riêng, không mở đồng thời demo:

ssh -o HostKeyAlias=ubuntu.local pi@10.0.0.1

cd ~/NCHK_robotFirefighting_ROS/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY

ros2 launch fire_robot_bringup robot.launch.py \
  use_sim_time:=false \
  use_camera:=true \
  camera_backend:=native_mjpeg \
  camera_fps:=15 \
  camera_max_bytes_per_sec:=750000

```bash
cd ~/NCHK_robotFirefighting_ROS/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
ros2 launch fire_robot_bringup robot.launch.py use_sim_time:=false \
  use_camera:=true camera_backend:=native_mjpeg camera_fps:=15 \
  camera_max_bytes_per_sec:=750000
```

## Dọn phiên cũ trên laptop

Nếu một phiên cũ đã thoát nhưng Nav2/YOLO/mission trên **laptop** còn chạy,
trước hết Cancel goal và xác nhận xe đã dừng. Sau đó chạy trên laptop (không
chạy trong shell SSH của Pi):

```bash
cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS"
python3 scripts/stop_laptop_nodes.py --dry-run  # Xem tiến trình sẽ dừng
python3 scripts/stop_laptop_nodes.py            # SIGINT, rồi SIGTERM nếu kẹt
```

Script chỉ chọn tiến trình ROS của robot do người dùng hiện tại chạy trên
laptop; không SSH sang Pi. `ros2 node list` vẫn thấy node Pi sau khi dọn laptop.

## 1. Quét và lưu map

Trên **laptop**, xe ở sàn thoáng và người vận hành đứng cạnh công tắc nguồn
động cơ:

```bash
cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS"
./scripts/start_mapping.sh
```

source /opt/ros/humble/setup.bash
source "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/ros2_ws/install/setup.bash"

export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -r cmd_vel:=/cmd_vel_raw \
  -p speed:=0.05 \
  -p turn:=0.5

File tự source ROS, kiểm tra `/scan`, `/odom`, `/imu/data`, mở SLAM + RViz,
chờ map thật rồi vào `teleop_twist_keyboard` ngay terminal đó. Tốc độ khởi
đầu 0,08 m/s, xoay 0,5 rad/s; phím tăng tốc của teleop vẫn có tác dụng nên
không dùng khi chỗ chạy hẹp. Giữ terminal này mở trong lúc quét.

Khi map đã đủ, **mở terminal laptop thứ hai**:

```bash
cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS"
./scripts/start_mapping.sh save fire_room_demo
```

`save` gọi `map_saver_cli` và lưu `fire_room_demo.yaml/.pgm` vào
`ros2_ws/src/fire_robot_navigation/maps/`; nếu tên đã tồn tại thì dừng, không
ghi đè. Có thể bỏ tên để dùng thời gian hiện tại. Tên map vừa lưu được ghi
vào `scripts/selected_map.txt` để buổi demo tự dùng. Sau khi xác nhận `.yaml`
và `.pgm` đã tạo, Ctrl-C terminal teleop/SLAM. Không chạy SLAM và Nav2 cùng lúc.

## 2. Thử Nav2 → YOLO → dừng/hủy → căn tâm (không IR/bơm)

Kết thúc phiên demo cũ bằng Cancel goal trong RViz rồi Ctrl-C terminal cũ.
Đưa lửa ra khỏi camera. Trên laptop chạy:

```bash
cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS"
./scripts/start_demo.sh --align-only
```

File chờ `[READY]` rồi tự cấp đúng bốn arm cho Nav2, hủy goal và xoay thân;
không khởi động `ir_approach_live`, `pump_demo_controller` hay arm IR/bơm.
Đặt goal ngắn trên RViz, chờ xe thật sự chạy, sau đó đưa lửa vào camera.
Chờ `[OK] Xe đã dừng và căn tâm`, kiểm tra thực tế, rồi Ctrl-C để kết thúc.
Vẫn phải đứng cạnh chỗ ngắt nguồn động cơ; lỗi dừng sẽ thu hồi các arm.
Nếu YOLO mất lửa khi đang xoay, bộ căn tâm phát vận tốc góc **0** và chờ
mission xác nhận `FIRE_TRACKING` lại; lúc đó tiếp tục cùng goal, không cần
đặt goal khác. Nếu detection còn lửa nhưng preview báo `NO_TARGET`, mission
có thể là `WAITING_FOR_ALIGNMENT`; lúc này xe cũng phát zero và chờ preview
hợp lệ. Không còn mốc 45/47 giây; vẫn kiểm tra arm, scan, odom, nguồn lệnh,
goal, tổng góc xoay và phản hồi odom khi chờ/xoay. Nếu mất điều kiện an toàn,
file in `[WARN]`, thu hồi arm,
giữ terminal/RViz để xem log; không tự xoay mù hay phun.

## 3. Nav2 → YOLO → dừng/hủy → căn tâm → IR → phun

Để nghiệm thu IR trước **mà tuyệt đối không mở node bơm**, chạy trên laptop:

```bash
cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS"
./scripts/start_demo.sh --ir-only
```

Đây vẫn là chế độ **xe chạy thật**, cần người đứng cạnh công tắc nguồn động
cơ và lửa/vật cản được bố trí an toàn. Chờ `[READY]`, đặt goal ngắn, rồi
đưa lửa vào camera khi xe đang chạy. Chờ `[OK] IR đã dừng trong vùng phun`;
nếu `[WARN]` thì xe dừng và không phun. Ctrl-C để đóng phiên trước lượt mới.
Chỉ dùng chế độ đầy đủ dưới đây sau khi lượt không bơm đạt và kiểm tra quãng
dừng thực tế tại 0,08 m/s.
Trước `[READY]`, script giữ lệnh arm IR `false` cho đến khi chính node IR báo
đã nhận, rồi chuyển sang heartbeat `true` và đợi node xác nhận đủ chuỗi
`false → true` cùng đúng một publisher. Nếu không xác nhận được, phiên dừng
trước khi đặt goal. Nếu arm mất kéo dài sau khi căn tâm/bàn giao, script thu
hồi arm và báo lỗi thay vì chờ im lặng; không tự tiến hay phun khi arm trở lại.

Giữ ngọn lửa **ngoài camera** khi mở file. Xe phải ở sàn thoáng, người vận
hành cạnh chỗ ngắt nguồn động cơ/bơm, vòi hướng nơi hứng an toàn và bơm được
cấp nước trước khi chạy file này. Đây là chế độ chuyển động/phun **thật**.

```bash
cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS"
./scripts/start_demo.sh
```

File tự dùng map cuối cùng lưu bằng `start_mapping.sh save`. Nếu muốn dùng
map khác: `./scripts/start_demo.sh --map /duong/dan/map.yaml`.

File tự mở Nav2 **shadow** + RViz, YOLO + camera annotated, supervisor,
gateway, cancel guard, alignment, IR, hậu kiểm; chờ trạng thái sẵn sàng rồi
tự phát năm heartbeat arm với QoS Reliable/Volatile ở 5 Hz. File đã chọn
`--live`; không cần thêm cờ hoặc chạy các lệnh arm thủ công nữa.

Trong RViz, đặt **2D Pose Estimate** theo đúng vị trí/hướng xe nếu AMCL chưa
định vị (lần khởi động Nav2 mới thường cần bước này). Chỉ sau dòng
`[READY]` và khi vật cản đã được kiểm tra, đặt goal ngắn bằng `Nav2 Goal`.
Chờ xe thật sự chạy rồi đưa ngọn lửa nhỏ vào camera. Sau khi Nav2 hủy goal
và xe căn tâm, IR tiến liên tục 0,08 m/s, tối đa 30 cm mỗi chặng, chỉ khi
cổng an toàn cho phép. IR bắt đầu báo vùng phun thì xe dừng trước, đợi xác
nhận đủ mẫu khi đã đứng yên; mất tín hiệu, gặp cản, quá giới hạn hay chưa
xác nhận vùng phun đều dừng và không phun. Bơm chỉ được khởi động sau khi xe đã dừng,
IR báo vùng phun, quyền `/cmd_vel_raw` đã nhả và proof của gateway/guard khớp
goal; `pump_demo_controller` vẫn tự kiểm tra lửa, odom, cmd và publisher. Nếu
hậu kiểm xác nhận vẫn còn lửa **và** vẫn đúng tâm/vùng phun, script cho tối đa
3 lượt phun 5 giây. Nếu hậu kiểm yêu cầu căn/tiến lại, script nhả publisher
bơm trước, chờ bàn giao cùng UUID và thử phục hồi có giới hạn 90 giây; không
đủ bằng chứng thì dừng và yêu cầu xử lý thủ công. Dữ liệu không chắc không
được xem là đã hết lửa.

Lượt Nav2 **không có lửa**: goal SUCCEEDED, CANCELED (kể cả hủy bằng RViz)
hoặc ABORTED đều có thể đặt goal khác mà không cần qua IR/bơm. Gateway chỉ
nhận UUID mới sau khi goal cũ kết thúc, xe và `/cmd_vel` bằng 0, bơm/servo
OFF và không còn publisher lệnh chuyển động/bơm. Nếu lửa đã được xác nhận
nhưng tắt **trước lần phun đầu**, mission đợi camera và IR cùng không còn
bằng chứng lửa trong 1,5 giây, Nav2 đã hủy đúng UUID, xe đứng yên và cơ cấu
OFF; các node căn tâm/IR/guard mới nhả lượt cũ. Goal mới tới sát bản tin
kết thúc goal cũ phải đợi guard nhả UUID và cửa sổ dừng an toàn; không cần
đặt lại goal sau khi đủ điều kiện. Nếu node bơm từng giữ quyền trong lượt
đó, không còn coi là nhánh "chưa phun" dù hiện tại publisher đã biến mất.
Nếu IR/camera mất kết nối,
không coi đó là “hết lửa” để tự cấp goal mới. Nếu đã từng phun, chỉ nhánh
hậu kiểm sau phun được phép kết thúc lượt chữa cháy.

Khi YOLO xác nhận lửa và đã căn tâm nhưng IR còn `NO_FLAME_SIGNATURE`, chế
độ demo tích hợp xem đây là vùng tiếp cận xa, **không phải** xác nhận khoảng
cách thật. Xe có thể tiến 0,08 m/s qua từng chặng tối đa 30 cm và kiểm tra
lại Lidar/odom/camera/IR liên tục; file demo không đặt trần tổng 2 m nữa.
Chỉ `SPRAY_RANGE` IR đã ổn định mới có thể mở bơm. Vùng xa này cần thử thật
trong hành lang thoáng và vẫn có người ngắt nguồn: IR không thấy lửa cũng có
thể do che khuất hoặc hiệu chuẩn sai. Vật cản, mất dữ liệu, vượt chặng hoặc
không xác nhận vùng phun đều giữ bơm OFF.

Nếu IR dừng lỗi trong phiên đầy đủ (ngoại trừ đang xác minh lửa tắt trước
phun), script **thu hồi toàn bộ arm**, ghi sự
kiện `IR_APPROACH_FAILED` với lý do, blocker lúc bắt đầu dừng và trạng thái
IR/camera vào `events.jsonl`, nhưng giữ RViz và các node quan sát mở cho tới
khi người vận hành Ctrl-C. Nó không tự tiến lại, tự tránh vật cản hay phun khi
camera/IR bị che. Giới hạn 30 cm mỗi chặng vẫn áp dụng: chạm giới hạn
thì dừng, nhả quyền, đo lại và tự tiếp tục khi proof còn đúng, không phải
xác nhận đã tới vùng phun. Mất target đã có frame IR mới thì dừng và chờ
YOLO xác nhận lại trước khi căn tâm; mất sensor/heartbeat vẫn là lỗi an toàn.
Để thử một
goal khác sau lỗi cảm biến/an toàn, cần xác nhận xe đứng yên, Ctrl-C phiên cũ
rồi khởi động phiên mới; nhiều goal liên tiếp sau lỗi chưa được nghiệm thu
trên xe thật. Riêng lửa tắt trước phun chỉ được giải phóng goal khi đủ proof
camera + IR + trạng thái Nav2 + cơ cấu OFF như trên. Khi YOLO mất tracking,
các node giữ lệnh chuyển động bằng 0 và script vẫn giữ arm cho cùng goal;
không còn thu hồi arm chỉ vì chờ quá 8 giây. Nếu YOLO xác nhận lại lửa lệch
tâm, bộ căn tâm tiếp tục xoay sau khi các cổng an toàn còn hợp lệ. Nếu không
có proof lửa tắt hoặc tracking mới, xe tiếp tục đứng yên; lỗi heartbeat,
telemetry, Lidar, odom hoặc quyền lệnh vẫn dừng theo nhánh an toàn riêng.
Nếu state `STOPPING` kéo dài quá 4 giây mà thiếu bằng chứng xe đứng yên và
nhả `/cmd_vel_raw`, script ghi `IR_STOP_PROOF_TIMEOUT`, thu hồi arm và yêu cầu
kiểm tra xe; tuyệt đối không xem timeout này là đã dừng để được phun.
Nếu telemetry của node IR live mất quá 1 giây trước lúc phun, script ghi
`IR_TELEMETRY_LOST` và cũng thu hồi arm. Cả hai trường hợp đều không tự reset
goal hoặc tự phun khi kết nối trở lại.

Sau khi hậu kiểm báo hết lửa, chờ dòng `[READY] Hết lửa` rồi mới đặt goal
khác. Nếu một cổng không đạt hoặc launch/heartbeat chết, file dừng phiên và
thu hồi các arm; Pi watchdog/S3 watchdog vẫn độc lập. **Ctrl-C** tại terminal
demo sẽ dừng các launch do file mở. Script không tự giết các launch cũ: nếu
thấy node trùng, hãy Ctrl-C đúng terminal cũ, xác nhận xe đứng yên, rồi chạy
lại. Demo mới dừng Pi `robot.launch.py` của phiên sau khi dọn laptop. Lượt
demo kế tiếp sẽ kiểm tra giờ rồi mở lại Pi. Nếu SSH mất, supervisor Pi tự
cleanup khi thấy EOF hoặc quá 35 giây không có heartbeat; đây là cơ chế dọn
phiên, không thay thế watchdog motor/bơm. Nếu chưa nhận xác nhận cleanup,
terminal báo lỗi và không tuyên bố Pi đã dừng.

Để điều tra một lượt, ghi bag **trước khi đặt goal**; `--include-hidden-topics`
là bắt buộc để thu `/navigate_to_pose/_action/status`. Ghi thêm các topic
arm giúp phân biệt mất heartbeat với kẹt bàn giao. Dừng bag bằng Ctrl-C và
gửi đường dẫn `BAG_DIR` cùng `fire_robot_session_*/events.jsonl`:

```bash
source /opt/ros/humble/setup.bash
source "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
BAG_DIR="/tmp/fire_full_$(date +%Y%m%d_%H%M%S)"
echo "BAG_DIR=$BAG_DIR"
ros2 bag record --include-hidden-topics --polling-interval 50 -o "$BAG_DIR" \
  /yolo/detections /yolo/alignment_preview \
  /fire_mission/state /fire_mission/live_nav_stop_state \
  /fire_mission/nav2_cancel_guard_state /fire_mission/live_alignment_state \
  /fire_mission/ir_range_preview /fire_mission/ir_approach_live_state \
  /fire_mission/pump_demo_state /fire_mission/post_spray_review \
  /fire_mission/live_nav_arm /fire_mission/nav_cancel_arm \
  /fire_mission/live_alignment_arm /fire_mission/ir_approach_arm \
  /fire_mission/pump_demo_arm /fire_sensors/ir_raw \
  /fire_sensors/actuator_state /navigate_to_pose/_action/status \
  /scan /odom /cmd_vel_raw /cmd_vel /pump_cmd
```

Log mỗi node và nhật ký sự kiện JSONL nằm dưới thư mục
`/tmp/fire_robot_session_*` được in lúc khởi động. Đây là luồng tích hợp thử
nghiệm: unit test không thay thế bài chạy phần cứng có người giám sát.
