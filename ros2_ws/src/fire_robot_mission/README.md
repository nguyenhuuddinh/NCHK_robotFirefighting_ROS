# fire_robot_mission

Package laptop-only kết hợp telemetry Nav2 và YOLO. Supervisor vẫn hoàn toàn
**observe-only**. Node handoff và arbiter tùy chọn chỉ publish `Twist` lên
các topic shadow bị cô lập. Riêng `live_nav_stop_gateway` mới có thể publish
`/cmd_vel_raw` khi operator bật opt-in và duy trì heartbeat; mặc định nó
**không** làm xe chạy. Riêng `pump_demo_controller` có thể publish
`/pump_cmd` cho bài phun giám sát, nhưng chỉ trong launch tách biệt với hai
opt-in và arm riêng; mặc định không tạo publisher này. Node cancel guard
riêng có thể gửi `CancelGoal` cho đúng một Nav2 goal khi operator bật opt-in.

## Luồng dữ liệu

```text
/yolo/detections -----------+
/yolo/alignment_preview ----+
/navigate_to_pose/... ------+--> fire_mission_supervisor
/odom ----------------------+        |
/cmd_vel_nav ---------------+        +--> /fire_mission/state (JSON)
/cmd_vel_raw ---------------+
/fire_mission/post_spray_review -+ (chỉ đóng chu trình)
/fire_sensors/actuator_state ---+ (telemetry lệnh bơm/vòi OFF)
```

`/fire_mission/state` chỉ mô tả trạng thái, freshness, hướng căn lửa dự kiến,
mức đồng thuận/xung đột với lệnh Nav2 và các blocker an toàn. Topic này không
được nối vào chuỗi điều khiển.

Shadow handoff thêm một tầng thử admission độc lập:

```text
/fire_mission/state --------------------+
/yolo/alignment_preview ----------------+--> alignment_handoff_shadow
/fire_mission/alignment_arm_shadow -----+             |
                                                      +--> /fire_mission/alignment_cmd_vel_shadow
                                                      +--> /fire_mission/alignment_handoff_state
```

Topic `alignment_cmd_vel_shadow` không được SafetyGate trên Pi subscribe và
không thể làm xe chạy. Node fail-fast nếu output bị cấu hình/remap thành
`/cmd_vel`, `/cmd_vel_raw`, `/cmd_vel_nav` hoặc ra ngoài namespace
`/fire_mission/*_shadow`.

Để kiểm thử chuỗi bàn giao sau khi Nav2 đã **thực sự** chạy, có cổng opt-in
`require_live_cancel_proof` (mặc định `false`). Khi bật trong
`config/mission_params.yaml` trước lúc launch, handoff shadow chỉ báo `active`
nếu đồng thời nhận bản tin mới từ gateway `HANDOFF_READY` (đã nhả publisher
`/cmd_vel_raw` sau khi phát hiện lửa) và cancel guard
`CANCEL_CONFIRMED_STOPPED` (đã hủy đúng một UUID, status `CANCELED`, xe đứng
yên), mỗi topic có đúng một publisher. Bản tin thiếu, quá hạn, sai safety
contract, hoặc có publisher trùng đều khiến output shadow bằng zero. Điều này
**không** cấp quyền điều khiển xe: output vẫn chỉ là topic `_shadow`, chưa có
node nào lấy lệnh đó đưa vào SafetyGate. Để kiểm thử phần cứng vẫn cần quy
trình opt-in riêng và người vận hành trực tiếp giám sát.

Trong bài kiểm tra tích hợp, launch
`mission_motion_arbiter_shadow.launch.py` nhận đối số
`require_live_cancel_proof:=true` để bật cổng này mà không sửa YAML. Đối số
chỉ thay đổi admission của handoff shadow, không bật gateway hoặc guard thật.
Diagnostic subscriber-only `handoff_sequence_diagnostics` đọc năm topic
state của mission, gateway, guard, handoff và arbiter. Nó ghi các mốc từ
Nav2 đang chạy đến arbiter shadow chọn alignment. `PASS_TELEMETRY_ONLY` chỉ
nghĩa là chuỗi telemetry đủ và nhất quán; **không** chứng minh thời gian phanh
hay việc thân xe đã dừng trong thế giới thật. `INCOMPLETE` nghĩa là thiếu
mốc, `FAIL` nghĩa là JSON/hợp đồng/thứ tự sai. Phải mở diagnostic trước khi
đặt goal mới để thấy đủ chuỗi.

```bash
ros2 run fire_robot_mission handoff_sequence_diagnostics \
  --seconds 90 --progress-period 1 \
  --output /tmp/fire_handoff_sequence_lan.json
```

Arbiter dry-run thêm phép kiểm tra quyền sở hữu độc quyền:

```text
/fire_mission/state -------------------------+
/fire_mission/alignment_handoff_state -------+
/fire_mission/nav2_cmd_vel_shadow -----------+--> motion_arbiter_shadow
/fire_mission/alignment_cmd_vel_shadow ------+             |
                                                           +--> /fire_mission/cmd_vel_arbiter_shadow
                                                           +--> /fire_mission/motion_arbiter_state
```

Nav2 xin quyền bằng `navigation.goal_active`; alignment xin quyền bằng
`handoff.active`. Hai yêu cầu cùng xuất hiện, state/command quá hạn, contract
sai hoặc target bị mất đều đưa arbiter về `SAFE_STOP` với lệnh zero. Output
vẫn chỉ là dry-run và không nối vào `/cmd_vel_raw`.

Nguồn Nav2 của arbiter là đầu ra **sau** `velocity_smoother` từ launch
`nav2_motion_shadow.launch.py`, không phải `/cmd_vel_nav` thô. Wrapper này ép
cả velocity smoother và Nav2 behavior server vào
`/fire_mission/nav2_cmd_vel_shadow`; nó không cho phép override output về
chuỗi điều khiển robot.

Cancel-intent shadow đọc độc lập mission state và Nav2 action status. Nó chỉ
publish JSON lên `/fire_mission/nav2_cancel_intent_shadow`: `REQUEST_CANCEL`
nghĩa là **đề xuất** hủy goal theo UUID, không phải đã gửi yêu cầu hủy. Goal
đang `CANCELING` được báo riêng; mission quá hạn, mất publisher Nav2, hai nguồn
không khớp hoặc có nhiều goal active đều `BLOCKED`. Action status của ROS 2
được phát theo sự kiện đổi trạng thái, không có heartbeat định kỳ; vì vậy tuổi
status chỉ là telemetry, không dùng làm timeout. Khi mất publisher, node bỏ
cache status và cần nhận lại status trước khi đề xuất hủy. Node không tạo
action client.

Cancel guard là bước **khác** với shadow: dùng dịch vụ chuẩn
`/navigate_to_pose/_action/cancel_goal` của ROS 2 để hủy đúng UUID. Request
có UUID khác zero và timestamp zero, tuyệt đối không dùng request cancel-all.
Mặc định `enable_real_cancel=false`. Khi opt-in, nó còn cần heartbeat Bool
`/fire_mission/nav_cancel_arm` mới, cancel-intent mới và khớp UUID với action
status. Đây là thử nghiệm đầu tiên nên chỉ cho phép `Nav2` phát lệnh vào
`/fire_mission/nav2_cmd_vel_shadow` và không có publisher `/cmd_vel_raw`.
Gate kiểm tra đúng ba endpoint shadow hiện có: một `velocity_smoother` và hai
endpoint `behavior_server`, đều là `geometry_msgs/Twist` với QoS Reliable.
Thiếu endpoint, thừa endpoint hoặc xuất hiện publisher lạ đều bị chặn; không
chỉ kiểm tra số lượng.
Node không phát Twist hoặc lệnh bơm. Một goal UUID chỉ được gửi tối đa một yêu
cầu hủy; chỉ sau chu trình thành công mới nhận goal UUID mới, còn lỗi an toàn
vẫn khóa và cần kiểm tra thủ công. Sau khi service chấp nhận, guard
chỉ báo `CANCEL_CONFIRMED_STOPPED` khi status của UUID là `CANCELED`, không có
goal khác active, `/cmd_vel` mới và zero, odom mới và đứng yên liên tục ít nhất
`0,50 s`. Trạng thái này **chưa cấp quyền alignment hay phun**.

## Gateway dừng Nav2 thật và bàn giao quyền xuất lệnh

`mission_live_nav_stop.launch.py` chạy **riêng** với Nav2 đã cô lập qua
`nav2_motion_shadow.launch.py`, Pi SafetyGate, và supervisor/cancel-intent.
Không chạy đồng thời Nav2 launch thường hoặc teleop/dashboard nào publish
`/cmd_vel_raw`. Gateway chỉ nhận vận tốc đầu ra sau `velocity_smoother`, giới
hạn `|linear.x| <= 0,10 m/s`, `|angular.z| <= 0,8 rad/s`, và cấm mọi thành phần
Twist khác zero. Trước khi xuất lệnh, nó đòi đúng fingerprint ba publisher
Nav2 shadow, đúng một SafetyGate `/cmd_vel`, một publisher cho mỗi
mission/odom/heartbeat/status, dữ liệu mới và mission `KEEP_NAV2`.

Gateway chỉ tạo **một** publisher `/cmd_vel_raw` khi
`enable_live_output:=true` **và** nhận Bool `true` mới trên
`/fire_mission/live_nav_arm` (heartbeat riêng, không phải cancel arm). Nếu
mission báo lửa được xác nhận, mất heartbeat/dữ liệu, phát hiện publisher lạ,
hoặc goal không còn thuộc Nav2, nó chốt STOP và liên tục phát zero. Nó chỉ
hủy publisher của chính mình sau khi thấy odom đứng yên **và** `/cmd_vel` do
SafetyGate phát mới bằng zero trong tối thiểu 0,50 s. Sau đó nó không resume
goal cũ; chỉ nhận goal UUID khác sau khi chu trình dập lửa được xác nhận kết
thúc an toàn. Guard có thể thấy
`/cmd_vel_raw` không còn publisher để hủy goal đúng UUID. Trạng thái
`HANDOFF_READY` chỉ là JSON telemetry sau khi guard báo
`CANCEL_CONFIRMED_STOPPED` và supervisor báo đủ điều kiện; tự nó không điều
khiển alignment hoặc bơm.
Không restart gateway khi Nav2 goal cũ còn active; nếu guard không hoàn tất
cancel, operator phải xử lý goal đó và xác nhận xe đứng yên trước buổi thử mới.

Mặc định dùng lệnh sau để **quan sát, không chạy xe**:

```bash
ros2 launch fire_robot_mission mission_live_nav_stop.launch.py \
  use_sim_time:=false
ros2 topic echo --full-length /fire_mission/live_nav_stop_state \
  std_msgs/msg/String --once
ros2 run fire_robot_mission live_nav_stop_diagnostics \
  --seconds 20 --output /tmp/fire_live_nav_stop_observe.json
```

Chỉ dùng `enable_live_output:=true` trong buổi thử có operator cạnh xe,
emergency stop sẵn sàng, đường chạy trống, Pi/ESP32 watchdog đã kiểm tra và
đã hoàn tất preflight ROS graph. Arm `live_nav_arm` có thể khiến xe chạy khi
Nav2 goal còn active; không bật để chỉ quan sát. Nếu process laptop chết,
SafetyGate Pi vẫn dừng sau timeout hiện tại 1000 ms; đây là fallback, không
phải bằng chứng xe đã dừng tức thì. Chưa nghiệm thu chuyển động thật.

## One-shot body alignment sau Nav2 cancel

`mission_live_alignment.launch.py` chạy **riêng**, mặc định
`enable_live_alignment:=false`, không được include trong launch mission hay
Nav2 hiện hữu. Khi tắt, node chỉ phát JSON
`/fire_mission/live_alignment_state`, không tạo publisher `/cmd_vel_raw`.
Chỉ khi opt-in **và** có heartbeat riêng
`/fire_mission/live_alignment_arm`, handoff shadow đang active với
`require_live_cancel_proof=true`, gateway đã nhả publisher, guard xác nhận
đúng UUID `CANCELED`, Nav2 status không có goal active, SafetyGate và odom
zero, LiDAR `/scan` mới/quét gần 360°/không có điểm dưới 0,30 m, node mới
có thể nhận độc quyền `/cmd_vel_raw`.

Sau khi node tạo publisher, guard và handoff shadow cũ sẽ không còn proof
`active`: đây là hệ quả dự kiến của việc raw owner mới xuất hiện. Node chỉ
dùng proof trước lúc takeover, sau đó tự kiểm tra liên tục operator arm,
mission/preview (mất lửa thì chỉ xuất zero), LiDAR, odom, SafetyGate,
status Nav2 và nguồn publisher. Nó chỉ phát `linear.x=0`, giới hạn mặc định
`|angular.z|<=0,60 rad/s`, gia tốc `0,40 rad/s²`. Không còn mốc kết thúc
theo 15/45/47 giây khi vẫn cùng goal: mất target thì giữ zero, chờ mission
xác nhận lại rồi xoay tiếp. Guard yêu cầu telemetry mới, đúng UUID và
đúng raw owner; thiếu proof sẽ dừng an toàn. Lệnh thực còn qua giới hạn gia tốc, nên
có thể thấp hơn trần và chưa đủ để bánh xoay nếu YOLO chập chờn liên tục.
Để không quay mù vô hạn khi YOLO bám nhầm, tổng góc xoay đo từ odom vẫn
giới hạn một vòng (2π rad) mỗi lần căn; odom không phản hồi lệnh xoay cũng
là lỗi an toàn.
Mất target: phát zero trong lúc
mission xác nhận lại. Cùng goal Nav2 đã thu nhận, mission cần ít nhất 0,20 s
và hai bản tin YOLO mới có lửa đã xác nhận trước khi trở lại `FIRE_TRACKING`
(hoặc trạng thái căn giữa). Trong `FIRE_CANDIDATE` node không xoay hoặc bàn
giao IR; chỉ tiếp tục khi mission đã xác nhận lại. Lửa mới/goal khác cần
0,50 s và ít nhất hai bản tin YOLO mới. Odom vượt
`0,80 rad/s` sẽ dừng. Tới `CENTERED`, mất dữ liệu/heartbeat, gần
vật cản, có nguồn lệnh lạ hoặc lệnh xoay mà odom không tiến triển thì phát zero, chờ odom
đứng yên và SafetyGate zero 0,50 s rồi nhả publisher. IR báo lệch tâm có
thể bàn giao trở lại để căn tiếp cùng goal sau proof đứng yên/không trùng owner.
Không có output cho bơm, không thay đổi watchdog hay cancel guard hiện tại.

Lệnh kiểm tra **mặc định tắt**, chỉ được chạy khi `/cmd_vel_raw` đang zero
publisher và robot đứng yên:

```bash
ros2 launch fire_robot_mission mission_live_alignment.launch.py \
  use_sim_time:=false
ros2 param get /live_alignment_takeover enable_live_alignment
ros2 topic info /cmd_vel_raw --verbose
```

Bài bật motor `enable_live_alignment:=true` **chưa nghiệm thu**. Không chạy
chỉ dựa vào unit test; cần operator kiểm tra khu vực trống, emergency stop,
scan clearance và toàn bộ chuỗi live Nav2→cancel trên xe trước khi cấp
heartbeat live alignment.

Khi operator thử xoay thật, có thể thu report gọn bằng
`ros2 run fire_robot_mission live_alignment_diagnostics --seconds 30
--output /tmp/fire_live_alignment_lan.json`. Công cụ này chỉ subscribe
`/fire_mission/live_alignment_state` và `/odom`, kiểm tra zero ngoài pha
`TURNING`, giới hạn command góc và ghi yaw odom quan sát được; report
**không** tự chứng nhận chuyển động vật lý an toàn. Không chạy heartbeat
hoặc motor chỉ để tạo report.

Sau khi căn tâm, IR tự tiến có chế độ bàn giao thực nghiệm riêng
`ir_approach_live.launch.py integrated_handoff:=true`; xem điều kiện,
giới hạn và thứ tự chuyển sang bơm trong [IR_RANGE_PREVIEW.md](IR_RANGE_PREVIEW.md).
Chế độ này mặc định tắt và chưa nghiệm thu trên xe; không chạy song song
với publisher `/pump_cmd`.

Khi YOLO mất mục tiêu giữa lượt căn tâm, live alignment giữ cùng quyền
`/cmd_vel_raw` nhưng chỉ gửi vận tốc **0**. `FIRE_CANDIDATE` và
`WAITING_FOR_ALIGNMENT` (detection còn lửa nhưng preview tạm `NO_TARGET`)
không cho xoay; khi mission lại `FIRE_TRACKING` với preview hợp lệ thì tiếp
tục cùng goal. Chờ lửa không tạo thêm state điều khiển, không có giới hạn
thời gian căn tâm theo đồng hồ, và không cho phép IR/bơm nhận bàn giao trước khi
`TARGET_CENTERED` cùng proof đứng yên/nhả quyền. Mất arm, scan, odom,
quyền xuất lệnh hoặc xuất hiện goal Nav2 mới vẫn dừng an toàn.

## Chạy

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY

ros2 launch fire_robot_mission mission_preview.launch.py \
  use_sim_time:=false
```

Diagnostic state machine:

```bash
ros2 run fire_robot_mission mission_preview_diagnostics \
  --seconds 30 \
  --output /tmp/fire_mission_preview.json
```

Chạy supervisor kèm shadow handoff:

```bash
ros2 launch fire_robot_mission mission_handoff_shadow.launch.py \
  use_sim_time:=false
```

Heartbeat arm chỉ kích hoạt phép tính shadow và phải được giữ liên tục. Dừng
lệnh bằng `Ctrl-C` thì gate tự về zero sau tối đa `0,40 s`:

```bash
ros2 topic pub --rate 5 \
  /fire_mission/alignment_arm_shadow \
  std_msgs/msg/Bool '{data: true}'
```

Diagnostic shadow ở terminal khác:

```bash
ros2 run fire_robot_mission handoff_shadow_diagnostics \
  --seconds 30 \
  --progress-period 1 \
  --output /tmp/fire_alignment_handoff_shadow.json
```

Để kiểm tra hướng sau một lần Nav2 đã dừng và goal đã cancel, giữ hình lửa
demo lần lượt bên trái, giữa và bên phải trong cùng một lượt handoff shadow.
Report đếm `active_direction_counts` (`LEFT`, `CENTERED`, `RIGHT`),
`active_direction_transitions` và đặt `directional_coverage_complete=true`
chỉ khi cả ba vùng đã có frame active. Hai bộ đếm
`direction_sign_violations` và `active_alignment_state_violations` phải bằng
zero; `shadow_contract_violations`, `linear_motion_violations` và
`nonzero_while_inactive_violations` cũng phải bằng zero. Đây chỉ là kiểm tra
telemetry: lệnh góc vẫn trên topic `_shadow` và không xoay bánh. Không lấy
trực tiếp output shadow để publish `/cmd_vel_raw`: handoff hiện yêu cầu odom
đứng yên, còn cancel guard cố ý khóa khi có publisher raw mới.

Chạy đầy đủ supervisor + handoff + arbiter dry-run:

```bash
ros2 launch fire_robot_mission mission_motion_arbiter_shadow.launch.py \
  use_sim_time:=false
```

Chạy Nav2 với output bị cô lập để test owner `NAVIGATION` (terminal riêng):

```bash
ros2 launch fire_robot_navigation nav2_motion_shadow.launch.py \
  use_sim_time:=false \
  map:=/absolute/path/to/map.yaml \
  rviz:=true
```

Trong chế độ này, Nav2 chỉ publish lệnh cuối lên
`/fire_mission/nav2_cmd_vel_shadow`. Xe không nhận các lệnh này.

Diagnostic quyền sở hữu:

```bash
ros2 run fire_robot_mission motion_arbiter_shadow_diagnostics \
  --seconds 35 \
  --progress-period 1 \
  --output /tmp/fire_motion_arbiter_shadow.json
```

Diagnostic cancel-intent (terminal khác, chạy khi launch shadow đang mở):

```bash
ros2 run fire_robot_mission nav2_cancel_intent_diagnostics \
  --seconds 35 \
  --output /tmp/fire_nav2_cancel_intent_shadow.json
```

Thử guard an toàn trước tiên ở chế độ mặc định tắt:

Trên laptop, mở từng terminal bằng cùng môi trường ROS trước các lệnh bên
dưới:

```bash
cd "/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
```

Terminal A (giữ launch chạy):

```bash
ros2 launch fire_robot_mission nav2_cancel_guard.launch.py \
  use_sim_time:=false
```

Terminal B (chỉ đọc state):

```bash
ros2 run fire_robot_mission nav2_cancel_guard_diagnostics \
  --seconds 15 --output /tmp/fire_nav2_cancel_guard_disarmed.json
```

Khi đã xác nhận đúng một `Nav2` shadow đang chạy, `/cmd_vel_raw` có zero
publisher, Pi SafetyGate đang phát `/cmd_vel` zero, odom mới và xe đứng yên,
mở guard ở terminal riêng với `enable_real_cancel:=true`; **không chạy**
`nav2.launch.py` thường trong bài này. Tắt guard mặc định ở terminal A trước
khi mở guard mới. Mỗi lệnh sau đây chạy ở **một terminal riêng**:

```bash
ros2 topic info /cmd_vel_raw --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic info /fire_mission/nav2_cmd_vel_shadow --verbose
ros2 topic hz /odom
```

Chỉ tiếp tục khi `/cmd_vel_raw` có `Publisher count: 0`, `/cmd_vel` có đúng
một publisher SafetyGate, topic Nav2 shadow có đúng ba endpoint nêu trên,
`/odom` đang cập nhật, và thân xe đứng yên. Lệnh
`ros2 topic hz /odom` chạy liên tục; dừng nó bằng `Ctrl+C` trước bước sau.

```bash
ros2 launch fire_robot_mission nav2_cancel_guard.launch.py \
  use_sim_time:=false enable_real_cancel:=true
```

```bash
ros2 topic pub --rate 5 /fire_mission/nav_cancel_arm \
  std_msgs/msg/Bool '{data: true}'
```

```bash
ros2 run fire_robot_mission nav2_cancel_guard_diagnostics \
  --seconds 35 --output /tmp/fire_nav2_cancel_guard_lan.json
```

Giữ arm bằng một terminal riêng. Đặt goal Nav2 shadow khi chưa có lửa, sau đó
đưa hình lửa demo vào camera. Guard gửi đúng một request, chờ status `CANCELED`
và xác minh đứng yên. Dừng arm bằng `Ctrl+C` sau khi đã lấy report. Nếu
guard báo `FAILED` hoặc không có `/cmd_vel`/odom tươi, dừng thử và kiểm tra;
không khởi động lại node để spam yêu cầu. Khi gateway đang cho xe chạy thật,
`/cmd_vel_raw` có publisher nên guard cố ý từ chối. Chỉ sau khi gateway dừng
và nhả publisher, guard mới có thể hủy goal theo điều kiện cũ. Bài này vẫn
cần test an toàn riêng trên xe; không nới điều kiện guard.

Kịch bản: để Nav2 chạy goal trên launch `nav2_motion_shadow.launch.py`, quan
sát `NO_CANCEL_NEEDED` khi chưa có lửa; đưa lửa vào và giữ quá 0,5 s để thấy
`REQUEST_CANCEL`, UUID và `FIRE_ACQUIRED`. Goal Nav2 **không bị hủy thật** ở
bước này; dừng goal bằng RViz hoặc thao tác operator sau khi test. Không nối
kết quả shadow vào `/cmd_vel_raw` hay bơm.

Đo offset clock laptop và Pi qua LAN, hoàn toàn read-only:

```bash
ros2 run fire_robot_mission clock_sync_diagnostics \
  --peer pi@10.42.0.185 \
  --samples 10 \
  --output /tmp/fire_clock_sync.json
```

## Giới hạn an toàn

### Đánh giá sau phun cho dashboard (chỉ quan sát)

`ros2 launch fire_robot_mission post_spray_review.launch.py` chỉ đọc trạng
thái bơm, YOLO, IR, alignment và mission rồi xuất JSON
`/fire_mission/post_spray_review` (`std_msgs/msg/String`, Best Effort).
Mỗi lượt chỉ được xét khi chính node đã thấy đi từ `FIRING` sang
`COMPLETE`/bơm OFF; chờ ảnh bớt bị nước che 1 giây và đòi bằng chứng ổn
định thêm 1 giây. `NO_FIRE_EVIDENCE` là *không còn bằng chứng lửa ở hai
nguồn cảm biến*, không phải chứng nhận dập lửa vật lý.
Sau một kết luận `RESPRAY_READY_SHADOW`, node có thể quan sát lượt phun kế
tiếp nếu bơm đã OFF ít nhất 0,5 giây. Nó đếm `pulse_index`, mặc định gợi ý
tối đa 3 lượt (`max_pulses` trong launch, hợp lệ 1–5), và báo
`RETRY_LIMIT_REACHED` thay vì tiếp tục gợi ý phun. Nếu bơm bật lại khi chưa
được kết luận cho phép hoặc sau khi đạt giới hạn, node báo
`UNEXPECTED_PUMP_RESTART`/`ABORTED`; đây chỉ là telemetry, **không chặn được
bơm thật**. Mỗi instance bộ bơm chỉ phát tối đa một xung cho một goal UUID.
`REALIGN_REQUIRED`, `RESPRAY_READY_SHADOW` và `APPROACH_REQUIRED` là
trạng thái đọc-only; node review **không** xuất `/pump_cmd`, Twist hay Nav2
action. Session full dùng các trạng thái này để tắt pump owner, đợi actuator
OFF, rồi căn/tiến lại cùng canceled goal; chỉ tạo pump owner mới sau proof
đứng yên, đúng tâm và đúng vùng phun. Tối đa 3 xung/goal. Chưa xác nhận chuỗi
này trên xe thật; mất proof hoặc vượt giới hạn thì dừng, không tự phun.

### Lượt dập tiếp theo không restart launch

Giữ các launch hiện tại chạy. Sau khi guard xác nhận cancel đúng UUID,
alignment kết thúc `TARGET_CENTERED`, bơm đã OFF (hết xung hoặc dừng sớm do
mất bằng chứng lửa), post-spray review xác nhận `NO_FIRE_EVIDENCE` bằng camera
**và** IR, mission về `IDLE` không còn lửa, xe đứng yên, `/cmd_vel` zero và
servo đã về idle, người vận hành có thể đặt một goal mới trong RViz. Gateway,
guard, alignment và bơm chỉ reset quyền cho UUID mới, không dùng lại UUID cũ.
Nếu còn lửa hoặc bất kỳ guard/sensor/actuator nào báo lỗi, không tự reset.
Supervisor chỉ giữ `TARGET_LOST` cho lửa đã acquire khi có **một Nav2 goal
UUID hợp lệ**. Thấy lửa trước khi đặt goal không tạo chu trình handoff giả.
Sau phun, supervisor chỉ trả về `IDLE` khi nhận `NO_FIRE_EVIDENCE` mới từ
đúng node review, cùng UUID đã acquire, có ít nhất một lượt phun, status goal
đó là `CANCELED`, camera mới không còn lửa, odom đứng yên và telemetry bridge
báo lệnh bơm OFF/vòi idle. Telemetry này không chứng minh bơm đã tắt vật lý.
Review khác UUID, quá hạn, thiếu dữ liệu hoặc bơm
còn ON giữ nguyên `TARGET_LOST`. Cần chạy `post_spray_review.launch.py` cho
luồng nhiều goal; chỉ thấy lửa biến mất không đủ để reset. Đây là reset
**telemetry**, không bỏ qua điều kiện rearm riêng của gateway/guard/bơm.
Các launch đang chạy bằng phiên bản code cũ phải được restart **một lần sau
khi build**; từ đó các lượt thành công không cần restart nữa. Đây là logic
chưa được nghiệm thu trên xe; bài chạy đầu vẫn cần giám sát và ngắt nguồn bơm
nếu có hành vi bất thường.

- Một detection không đủ để tạo tracking intent; mặc định phải giữ liên tục
  `0,50 s`.
- Timestamp camera cũ hơn `300 ms` hoặc đi trước laptop quá `20 ms` tạo
  blocker.
- Mất target sau khi đã acquire với goal UUID tạo `TARGET_LOST` và intent
  `REQUEST_SAFE_STOP`; không có goal UUID thì trả về `IDLE` khi hết lửa.
- Arbiter alignment vẫn chỉ là dry-run. Gateway Nav2 mới có thể sở hữu
  `/cmd_vel_raw` khi được bật rõ ràng; nó không thay thế SafetyGate và không
  cấp quyền alignment/bơm.
- Cancel-intent vẫn là JSON telemetry. Guard tách riêng chỉ gọi service
  `CancelGoal` đúng UUID khi được opt-in và có heartbeat arm; guard mặc định
  tắt, một lần hủy mỗi lần chạy, chỉ cho Nav2 shadow, chưa điều khiển xe.
- Chưa có collision checker mới cho tự tiếp cận. Bộ bơm demo tách biệt, mặc
  định tắt, chỉ phun tối đa 5 s sau khi alignment thật hoàn tất, Nav2 goal
  tương ứng là goal gần nhất đã hủy, IR/YOLO ổn định và xe đứng yên. Mất điều
  kiện thì OFF và khóa lần thử. Xem `IR_RANGE_PREVIEW.md` để biết giới hạn
  hiệu chuẩn và cách kiểm tra an toàn.
- Shadow gate yêu cầu đồng thời: mission input mới, alignment input mới,
  heartbeat arm mới, supervisor `eligible`, odom mới, robot đứng yên, Nav2
  không active và không có nguồn chuyển động khác.
- `linear.x` luôn bằng zero; `angular.z` shadow bị giới hạn tuyệt đối ở
  `0,35 rad/s`; mất bất kỳ điều kiện nào thì output shadow về zero.
- Điều kiện `robot stationary` chỉ phù hợp để kiểm tra admission trước handoff.
  Trước khi điều khiển thật phải có command arbiter duy nhất và state ownership
  riêng cho giai đoạn robot đang quay; không được nối trực tiếp shadow topic
  vào SafetyGate.
- Arbiter cho phép đúng một owner: `NAVIGATION`, `ALIGNMENT`, hoặc không owner
  ở `IDLE`/`SAFE_STOP`; trạng thái xung đột luôn xuất zero.
