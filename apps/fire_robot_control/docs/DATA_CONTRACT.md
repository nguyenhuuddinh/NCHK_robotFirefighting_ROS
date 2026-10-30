# Hợp đồng dữ liệu, nút bấm và vòng đời phiên

> Tài liệu thiết kế ban đầu; trạng thái triển khai mới nhất, điểm còn giới hạn
> và kết quả QA được cập nhật tại [QA tích hợp](QA_INTEGRATION_2026-10-29.md).

Trạng thái tại lúc lập tài liệu: khảo sát source0c54 và thiết kế dự kiến.
Ký hiệu đường dẫn C/S được định nghĩa trong [SOURCE_AUDIT](SOURCE_AUDIT.md).

## 1. Ranh giới quyền

```text
Người dùng
  → Qt frontend: chọn map, hiển thị, thao tác Pose/Goal qua RViz
  → quản lý phiên: gọi runner được cấu hình, nhận pairing/progress/receipt
  → sự kiện X/Q: chuyển vào operator backend hiện có
  → node/owner/guard hiện tại: quyết định cancel, motor, pump và release

Telemetry/report hiện có → observer UI chỉ đọc → trình bày tiếng Việt
```

Observer UI không tạo publisher/subscriber điều khiển trên `/pump_cmd`; không
tạo publisher `/cmd_vel`, `/cmd_vel_raw`, `/cmd_vel_nav`, `/fire_target`, arm,
stationary request hoặc cycle control. Không thêm action cancel client khác.
Tool RViz Pose/Goal vẫn là thao tác có chủ đích của người dùng, được phân biệt
với observer. Preview/replay không bật các tool có thể gửi lệnh.

**Lý do cấm cả reader `/pump_cmd`:** actor kiểm đúng2 subscriber theo tên/type/
QoS. Bật `ros2 topic echo /pump_cmd` hoặc reader UI trong phiên cũng có thể làm
chặn bơm. Dùng `/fire_mission/r4d_state.published_pump_request` để quan sát.

## 2. Topic/report đã thấy trong code

BE = Best Effort; REL = Reliable; V = Volatile; TL = Transient Local.
Tần suất là timer/config/luồng source, **không phải số đo thực tế**. Hàng ghi
“kiểm runtime” không được điền con số minh họa thành giá trị đã xác minh.

| Nội dung UI | Topic / file | Type / QoS đọc | Nguồn và nhịp source | Xác định cũ/không hợp lệ |
| --- | --- | --- | --- | --- |
| Hành vi/chờ/phục hồi | `/fire_mission/operator_state` | `std_msgs/msg/String`, BE/V/depth1 | `fire_robot_operator`, vòng50ms≈20Hz, thực tế phụ thuộc callback | Validate schema/scope, endpoint và tuổi nhận; đọc holds/fault/readiness, không tự cấp quyền |
| Camera YOLO | `/yolo/image_annotated/compressed` | `sensor_msgs/msg/CompressedImage`, BE/V/depth1 | `yolo_observer`, cap config10FPS, chỉ khi có subscriber | Giải mã được, stamp/frame hợp lệ; giữ ảnh cũ phải dán nhãn stale và tuổi |
| Confidence/vị trí tâm | `/yolo/detections` | `std_msgs/msg/String`, BE/V/depth1 | `yolo_observer`, theo inference + stale/error report | `state`, `source.stamp/age_ms`, `fire`, tuổi nhận; error/stale không là no-fire |
| IR ADC raw | `/fire_sensors/ir_raw` | `std_msgs/msg/String`, BE/V; publisher depth10 | `serial_bridge_node`, theo IR frame, timer bridge50ms; cadence MCU không chứng nhận ở lượt này | `valid`, giá trị, `sample_age_ms`, `receive_age_ms` và tuổi nhận, phân biệt raw/preview |
| IR band/filter | `/fire_mission/ir_range_preview` | `std_msgs/msg/String`, BE/V/depth1 | `ir_range_preview`, heartbeat5Hz + publish khi IR update theo YAML | `ir_band`, `filtered_counts`, `ir_receive_age_ms`, blockers; heartbeat không phải mẫu mới |
| Xác nhận IR/phun | `/fire_mission/r4d_state` | `std_msgs/msg/String`, BE/V/depth1 | `fire_pulse_controller`, timer100ms≈10Hz | `stationary_confirmation`, context, source age/report; không tự đếm frame trên UI |
| Yêu cầu bơm | `/fire_mission/r4d_state` | Như trên | `published_pump_request`, `pump_publisher_registered` | Null nghĩa chưa có yêu cầu/publisher; khác với OFF được báo thật |
| Phản hồi ESP/servo | `/fire_sensors/actuator_state` hoặc `r4d_state.actuator` | `std_msgs/msg/String`, BE/V; bridge depth10 | `serial_bridge_node`, theo ACT frame; timer20Hz không là ACT20Hz | `receive_age_ms` + tuổi nhận, `esp_ms` tiến; repeated snapshot không chứng minh home mới |
| Identity serial | `/fire_sensors/serial_session_identity` | `std_msgs/msg/String`, BE/V; bridge depth10 | `serial_bridge_node`, snapshot tối đa5Hz | instance/epoch/ELF, `state_age_ms`, valid/ready/healthy; bound runtime500ms |
| Mission/handoff chi tiết | `/fire_mission/state`, `/fire_mission/live_nav_stop_state`, `/fire_mission/nav2_cancel_guard_state`, `/fire_mission/live_alignment_state`, `/fire_mission/ir_approach_live_state`, `/fire_mission/post_spray_review` | `std_msgs/msg/String`, BE/V/depth1 | Các node tên tương ứng trong JSON_INPUTS; mission10Hz, phần khác theo config | Ưu tiên tổng hợp operator; mở thêm reader chỉ khi thiếu field, đọc đúng source/GID |
| Goal status | `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray`, REL/TL/depth1 ở observer | `bt_navigator`, action status, kiểm runtime | UUID + status + thời điểm nhận; status terminal cũ không thành goal đang chạy |
| Map | `/map` | `nav_msgs/msg/OccupancyGrid`, REL/TL/depth1 theo RViz config | SLAM khi mapping, map_server khi navigation; không giả map có heartbeat | Validate map/file đang chọn, frame và publisher; map tĩnh không stale chỉ vì không publish lại |
| Robot/odom | `/odom` | `nav_msgs/msg/Odometry`, BE/V; bridge depth10 | `serial_bridge_node`, theo STATE; không lấy timer20Hz làm odom thực | Stamp/tuổi nhận, finite pose/twist; STOP proof do backend, UI không thay thế |
| TF | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage`, dùng QoS chuẩn tf2 đang cài; xác minh endpoint khi tích hợp | SLAM/AMCL, odom broadcaster, robot_state_publisher | Dùng tf2 và health/profile hiện hành; không làm TF mới từ odom cũ; TF static không hết hạn kiểu sensor |
| Lidar | `/scan` | `sensor_msgs/msg/LaserScan`, BE/V/depth1 trong RViz | `scan_qos_relay`; tần suất đo từ scan thật | Header stamp/tuổi nhận; mốc0.5s có trong startup health, không suy an toàn khoảng trống từ Hz |
| Pose người dùng đặt | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped`, REL/V/depth5 trong config | Tool SetInitialPose RViz | Chỉ thao tác thật, gắn phiên/map; app không tự giả Pose để startup qua gate |
| Điểm đích chọn trên RViz | `/goal_pose` | `geometry_msgs/msg/PoseStamped`, REL/V/depth1 trong config | Tool SetGoal RViz | Là điểm vừa chọn, chưa phải bằng chứng action đã nhận/chạy/thành công |
| Startup/STOP/cleanup | `startup.json`, `health.json`, `receipt.json`, `cleanup.json`, `remote_cleanup.json` và owned log | File JSON local riêng phiên, không phải topic | Runner và startup_health | Binding/session, schema, atomic replace, freshness, error/forced/child proof riêng |

Nguồn code: `operational_session.py:JSON_INPUTS/EXPECTED/tick`,
`operational_session_core.py:report`, `operational_view.py:status_lines`,
`operational_contract.py`, `operational_frame.py`, `operational_identity.py`,
`pulse_controller.py`, `pulse_session.py`, `startup_health.py`,
`mission_supervisor.py:sensor_qos/action_status_qos`, `ir_range_preview.py`,
`serial_bridge_node.py:telemetry_publish_callback`, `yolo_observer.py` và các
YAML/config RViz tương ứng. Không sửa QoS publisher để khớp observer mới.

## 3. Mapping field quan trọng

| Widget | Field có thật / quy tắc |
| --- | --- |
| Hành vi chính | `operator.task.stage/outcome/reason`; phase/reason/fault ở mục chi tiết |
| Đang chờ phục hồi | `input_hold`, blockers, `runtime_rebind.held/reason`, `navigation_only`; không tự restart Nav2 |
| Ready | `startup_readiness`, execution/automatic + runner stage; “runner đã mở” không đồng nghĩa AUTO armed/goal-ready |
| IR | `ir_counts` là filtered counts; raw từ topic IR riêng; ghi đúng nhãn, không trộn hai nguồn |
| IR đủ mẫu | `stationary_proof.sample_count`, `range_confirm_samples/ir_confirm_required`; lượt sau lấy `spray_sequence.confirmation_sample_count/confirmation_required` |
| Quota | `spray_sequence.consecutive_count/max_pulses` và reason; không tích số lượt từ animation UI |
| Đồng hồ phun | `pulse_clock`, `pulse_on_ack_at`, `pulse_remaining_s` ở actor/sequence; không bắt đầu từ click Start hoặc ý định ON |
| Pump request | Actor `published_pump_request`: true/false/null phân biệt; không dùng `state==FIRING` làm bằng chứng request thực |
| PWM phản hồi | ACT `pump_pwm_commanded_on`, `pump_pwm_ready`; nhãn “ESP32: PWM ON/OFF” |
| Servo | ACT `servo_phase`, `servo_pwm_commanded_on`, `servo_commanded_angle_deg`, `home_command_completed_count`; đây là telemetry lệnh, không sensor góc vật lý |
| Nhả quyền nhận goal mới | Report task/phase/intents/handback; không dùng “YOLO không thấy lửa” làm ready |
| Dừng | `shutdown.requested/done/confirmed/blockers/reason`; receipt cleanup local/remote hiển thị riêng |

YOLO `source.age_ms`, processing/inference ms và thời gian từ ảnh tới hiển thị
là các đại lượng khác nhau; nhãn phải ghi rõ. Confidence detection hiện tại
khác ngưỡng cấu hình0.15. ADC không được đổi thành mét khi chưa có calibration.

## 4. Quy tắc freshness cho UI

1. Mỗi giá trị lưu payload, thời điểm nhận local monotonic, source stamp/age
   nếu có, nguồn/phiên và trạng thái hợp lệ. Không thay null bằng0 hoặc OFF.
2. Dữ liệu chứa tuổi từ source: cộng thời gian local trôi qua kể từ nhận, không
   reset “dữ liệu mới” chỉ vì có heartbeat/GUI redraw. Không trừ monotonic Pi
   với monotonic laptop; timestamp ROS/wall có kiểm đồng hồ riêng.
3. `operational_frame.FrameProof` đang kiểm source ảnh≤300ms, receive≤350ms;
   `SessionView` hiển thị đen sau0.5s không có decoded image mới. UI phải tách
   “có ảnh để nhìn” với “ảnh đủ mới cho điều khiển” và giữ kết quả backend.
4. IR runtime500ms cho điều kiện liên quan; identity500ms; ACT/odom/TF theo
   validator/profile source hiện có. Chép ngưỡng vào metadata chỉ để giải thích,
   không tạo gate điều khiển mới trong observer.
5. Đề xuất tuổi nhận0.5s để cảnh báo luồng operator UI bị gián đoạn; đây là
   ngưỡng trình bày chưa nghiệm thu, không gia hạn bound backend. Report JSON
   operator hiện không có source stamp tổng quát nên không gọi tuổi nhận đó là
   end-to-end latency hoặc bằng chứng message hoàn toàn mới.
6. Schema/type sai, JSON quá lớn/không hợp lệ, NaN/Inf, tuổi âm bất hợp lệ,
   sequence/stamp lùi, source đổi: hiện Không hợp lệ/Chờ dữ liệu, lưu lý do;
   không đổi thành Không có lửa hoặc Đã dừng.
7. Map và TF static có tuổi dữ liệu khác stream động. TF odom có deliberate
   offset100ms theo profile health; dùng validator hiện có, không âm thầm sửa.
8. Khi replay, đóng nhãn REPLAY/DỮ LIỆU GIẢ, khóa mọi runner/live command;
   clock replay không được dùng như BOOT/current proof của robot.

## 5. Hợp đồng các nút

| Nút | Tiền điều kiện và tác dụng | Khi chưa đủ điều kiện |
| --- | --- | --- |
| Kiểm tra Pi | Read-only bounded check khi người dùng yêu cầu; phân biệt WiFi/SSH/DDS/backend readiness | Hiện nguyên nhân; không tự đổi mạng, kill node hoặc chỉnh giờ |
| Quét map | Session lease còn trống, profile mapping hợp lệ → start runner/TTY đúng scope | Không mở thêm khi đang quét/chữa cháy/cleanup chưa xác nhận |
| Lưu map | SLAM thực sự hoạt động, không Nav2, tên hợp lệ, không overwrite → save async | Giữ SLAM; lỗi/partial file không chọn map mới |
| Chọn map | Validate YAML+image; khóa snapshot selection lúc start | Trong phiên chỉ ghi lựa chọn cho phiên sau hoặc khóa rõ; không đổi map ngầm |
| Khởi động chữa cháy | Map hợp lệ, phiên cũ dừng xong, profile/receipt đúng → runner Pi→pairing→laptop | Readiness thiếu hiện blocker, không fake gate hoặc chạy lệnh cũ |
| Dừng tác vụ X | Enqueue đúng một intent X vào operator hiện tại; ACK và thực thi theo report | Không nối X vào SIGTERM hoặc tự cancel action; offline/không phiên hiển thị phù hợp |
| Dừng phiên Q/Esc/đóng app | Ưu tiên yêu cầu Q, giữ UI/telemetry qua STOP drain, sau đó runner cleanup local→Pi | Timeout hiện UNCONFIRMED/degraded; không báo thành công từ process exit |

## 6. Adapter X/Q dự kiến — chưa tồn tại

Không thêm topic DDS lệnh để tránh phá kiểm endpoint. Đề xuất local Unix socket
riêng người dùng/phiên (`0600`, thư mục private), nối frontend với operator.
Envelope phải có version, current session/nonce, frontend generation, request ID,
sequence và thời hạn; chỉ chấp nhận X/Q. Đây là thiết kế mới, chưa phải API có sẵn.

- Operator xử lý intent trên chính loop hiện có; kết quả trả về từ feedback và
  report. ACK nhận request không phải STOP proof.
- Q ưu tiên, idempotent; không replay event qua reconnect/epoch hoặc gửi A/F/T/I/P.
- Frontend đóng/mất kết nối/không phát lease từ UI loop còn đáp ứng phải dẫn
  đến request_shutdown ở backend; không tắt safety để chạy headless.
- Nhịp lease/timeout cần chốt ở PoC từ guard callback hiện tại350ms và fault
  injection; không tự gia hạn guard hoặc biến backend heartbeat thành frontend
  còn sống. Backend vẫn phải dừng khi UI loop treo dù process còn chạy.
- Esc/nút dừng có đường ưu tiên khi phiên hoạt động, kể cả lúc nhập liệu;
  X/Q ngoài ô nhập dùng shortcut, chữ X/Q đang gõ tên map không thành lệnh.
  Window-close, Ctrl-C của runner và lỗi UI có đường STOP đã kiểm. Không yêu
  cầu xác nhận thêm trước nút dừng; form Save không khóa mất nút STOP.

Thay `operational_session` để nhận adapter/lease là thay source được pin:
phải tạo/review packet mới và xác minh deployment tương thích; không sửa manifest
cho qua check hoặc giả profile UI không liên quan packet.

## 7. Phần đã có và phần còn thiếu

Đã có: report operator/actor, compressed annotation, TF/map/scan, runner
`--map`, selector910/TN1, owned cleanup và STOP proof.

Còn thiếu: catalog mọi map mới, mapping cùng profile R5 được quản lý thống nhất,
API UI X/Q/lease, external frontend ownership thay RViz/OpenCV windows cũ,
desktop shortcut, renderer native đã kiểm và bằng chứng performance.

RobotModel/path xanh/goal marker trong ảnh chưa đủ nguồn từ config hiện tại:
trước thêm phải tìm model/topic thật. GoalStatusArray không chứa tọa độ goal;
`/goal_pose` chỉ chứng minh điểm đã chọn. Không đoán `/plan` hay coi marker là
goal đã được Nav2 nhận. Chức năng cơ bản có thể dùng TF để chỉ vị trí/hướng robot.
