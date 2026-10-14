cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS"

python3 scripts/stop_laptop_nodes.py
Các file này dùng ROS 2 Humble trong `ros2_ws/install` hiện có. Trên Pi phải
có cùng source ROS và firmware S3 tương thích; script tự kiểm tra Pi qua LAN và
chỉ yêu cầu Pi chạy `robot.launch.py` nếu chưa có. SSH mặc định:
`pi@10.42.0.185`; nếu địa chỉ khác, đặt `FIRE_PI_SSH_HOST=pi@<IP>` trước khi
chạy. Nếu không muốn script tự mở Pi, đặt `FIRE_AUTO_START_PI=0` và tự chạy trên
Pi:
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
3 lượt phun 5 giây. Nếu hậu kiểm đòi xoay/tiến lại hoặc dữ liệu không chắc,
phiên sẽ dừng và yêu cầu xử lý thủ công; chưa tự xoay/tiến lại sau khi phun.

Nếu IR dừng lỗi trong phiên đầy đủ, script **thu hồi toàn bộ arm**, ghi sự
kiện `IR_APPROACH_FAILED` với lý do, blocker lúc bắt đầu dừng và trạng thái
IR/camera vào `events.jsonl`, nhưng giữ RViz và các node quan sát mở cho tới
khi người vận hành Ctrl-C. Nó không tự tiến lại, tự tránh vật cản hay phun khi
camera/IR bị che. Giới hạn 30 cm mỗi chặng vẫn áp dụng: chạm giới hạn
thì dừng, nhả quyền, đo lại và tự tiếp tục khi proof còn đúng, không phải
xác nhận đã tới vùng phun. Mất target đã có frame IR mới thì dừng và chờ
YOLO xác nhận lại trước khi căn tâm; mất sensor/heartbeat vẫn là lỗi an toàn.
Để thử một
goal khác sau lỗi, cần xác nhận xe đứng yên, Ctrl-C phiên cũ rồi khởi động
phiên mới; tự phục hồi và nhiều goal liên tiếp sau lỗi chưa được nghiệm thu.
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
lại. Pi `robot.launch.py` được giữ chạy qua hai bước; dừng riêng trên Pi khi
kết thúc buổi demo.

Log mỗi node và nhật ký sự kiện JSONL nằm dưới thư mục
`/tmp/fire_robot_session_*` được in lúc khởi động. Đây là luồng tích hợp thử
nghiệm: unit test không thay thế bài chạy phần cứng có người giám sát.
