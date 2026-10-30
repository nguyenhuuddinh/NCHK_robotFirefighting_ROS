# Đối chiếu R5 terminal đã qua startup với app — 29/10/2026

Tiếp tục QA theo yêu cầu user, chưa sửa source runtime. Chỉ so source/launcher,
đọc các log đã có, resolve module trong shell Pi riêng và mở một app IDLE
trong ROS localhost domain211 để đọc graph. Không chạy lại robot.

## Kết luận có thể xác nhận

- R5 terminal lượt laptop21:17:14 / Pi21:16:40 thực sự tới READY_GATE2 và có
  automatic_start.ready=true. Đây là startup PASS, không phải giả định.
- App dùng cùng code Pi/TF/serial/health và cùng gate seal20s. Phần tích hợp
  đổi thứ tự/lượng tác vụ trước gate; chưa có A/B chứng minh delta nào làm
  TF trễ. Lỗi phối hợp STOP và gộp lỗi cleanup của app đã xác nhận riêng.
- R5 qua startup không chứng minh observer TF ổn định suốt phiên: snapshot
  Pi cuối lượt R5 có TF source_age4,062s. Snapshot này cách yêu cầu seal
  khoảng629,787s, sau khi Nav2/YOLO đã hoạt động; không được coi là dữ liệu
  tại startup hoặc so trực tiếp tải máy với app trước startup.

## 1. Code/gate thực tế được tái sử dụng

So364 file R5 packet0c54 với artifact app6734: chỉ
scripts/r5_laptop_session.py và scripts/r5_packet.py khác. Toàn bộ source
node ROS giữ byte. So riêng r5_pi_session.py, operational_peer.py,
operational_preflight.py, startup_health.py và odom_to_tf_broadcaster.py:
đều giống byte.

Diff runner tích hợp chỉ thêm bootstrap FD/session app, thay operator bằng
backend app, thay RViz process bằng display_guard, thêm STOP-only monitor
trước thu dọn khi thiếu operator, kiểm local group trước remote STOP và bổ
sung binding vào receipt. Không thay nhánh SEAL_PI, ngưỡng TF300ms hoặc
poll STATE2s của luồng chữa cháy.

Nguồn: [diff runner](qa_evidence/r5_app_parity_20261029/runner_integration.diff).
Gate dùng chung: releases/r5_20261029/ros/scripts/r5_pi_session.py:51 và
scripts/operational_entry.py:134/170. Trong app chữa cháy, nguyên block
StartupSequence/STATE/read_health vẫn được runner R5 gọi.

## 2. Finding P2 — App nhận dữ liệu ROS trước gate, R5 mở RViz sau gate

Code app native_rviz/main.cpp:561–574 tạo ROS context, Observer và RViz
frame ngay khi mở app. operational.rviz bật Map, LaserScan, TF từ đầu.
services/desktop_hooks.py:68 thay lệnh RViz muộn của R5 bằng display_guard;
RViz thật đã được supervisor mở trước user bấm khởi động.

R5 gốc scripts/operational_entry.py:170 chỉ phát START_NAV/START_RVIZ khi
pi_sealed/io/camera cùng đạt; r5_laptop_session.py:133 mới local.start RViz.

**Đã kiểm chứng binary app thật ở IDLE**, state tạm/localhost/domain211:

| Topic | Subscriber trước khi bấm khởi động |
| --- | --- |
| /tf, /tf_static | transform_listener_impl_* (Reliable) |
| /scan | fire_robot_control_preview (Best Effort) |
| /map | fire_robot_control_preview (Reliable) |
| /yolo/image_annotated/compressed | fire_robot_ui_observer (Best Effort) |
| /fire_sensors/actuator_state | fire_robot_ui_observer (Best Effort) |

Trong IDLE fire mặc định **không có subscriber raw camera của UI**;
annotated camera chưa có publisher YOLO trước gate, nên không quy camera
annotated thành lưu lượng ảnh startup. Khi chuyển sang mapping, Observer
đổi sang raw camera ở NEW_SESSION trước Pi START.

Phép kiểm không tạo publisher /cmd_vel, /cmd_vel_raw, /pump_cmd, /fire_target
hoặc /map; không tạo pending, supervisor đóng RC0. `depth=0` từ metadata
graph không dùng để suy ra độ sâu history thực; xem source QoS khi cần.

Đây là khác biệt thực về thời điểm tạo DDS reader, không phải bằng chứng
độc lập rằng nó gây TF lag trên Pi. Cần A/B hoặc trace trước khi quy kết.

## 3. Finding P2 — Mapping runner có nhịp kiểm tra khác R5

- R5/chữa cháy: r5_laptop_session.py:107 đặt peer_next=monotonic()+2s sau
  response STATE, ngoài thời gian SSH.
- App mapping: services/mapping_runner.py:160–170, sau SEAL chờ0,25s rồi
  gọi STATE lại bằng SSH đồng bộ. Chu kỳ thực bằng thời gian SSH +0,25s,
  **không khẳng định tần số thật4Hz**. Nhịp có thể dày hơn R5.
- Mapping khởi tạo mapping_monitor STOP-only trước Pi START
  (mapping_runner.py:139), thêm subscriber/ROS participant trước gate.
- Local IO1s, seal20s và điều kiện RUNNING_OBSERVE/sealed/health.io vẫn có.

Các điểm này chỉ thuộc mapping hoặc lớp app; không được dùng riêng chúng
để giải thích lượt app chữa cháy vốn giữ poll STATE2s của R5.

## 4. Thời điểm qua gate khác nhau dù dùng cùng code

| Quan sát | R5 terminal21:16:40 | App chữa cháy21:59:26 | App quét22:01:56 |
| --- | --- | --- | --- |
| Pi chờ từ yêu cầu tới seal | 0s trong cùng tick | 15,523s | Không seal trong20s |
| Snapshot seal ghi Pi IO | true | false, odom_tf | false từ yêu cầu; cuối vẫn odom_tf |
| Kết quả laptop startup | READY_GATE2 | Timeout SEAL_PI | PI_SEAL_HEALTH_TIMEOUT |
| TF nguồn ở snapshot Pi cuối | 4061,945ms | 178,491ms | 4764,675ms |
| Snapshot cuối cách yêu cầu seal | 629,787s | 28,088s | 20,689s |

Pi seal chỉ cần io=true tại một lần kiểm tra, còn laptop lại lấy health.io
hiện tại qua STATE. Các cửa sổ tốt ngắn và thời điểm poll có thể làm hai
lượt có kết quả khác nhau; cần lịch sử mẫu để chốt. R5 request có
seal_health_at_request.io=false nhưng seal_confirmed_at bằng request_at:
health được đọc lần nữa trong _try_seal và có thể đổi giữa hai lần đọc.
Không suy ra mọi dữ liệu request đồng thời, không sử dụng snapshot cuối
R5 để khẳng định trạng thái lúc seal.

R5 có40 SERIAL_QUARANTINE được ghi trong log dài hơn10phút, app chữa cháy
0, app quét3. **Không so trực tiếp các số đếm này như tỷ lệ lỗi**, vì thời
lượng/tải các lượt khác nhau. Chúng chứng minh hiện tượng serial timeout
cũng có trong đường R5, chưa chứng minh nguyên nhân vật lý hoặc MCU reset.

## 5. Môi trường/overlay: đã kiểm gì, chưa kiểm gì

Hai wrapper laptop source cùng Humble, laptop-baseline và install candidate.
Pi source cùng baseline nhưng overlay stage khác (R5 r5_staging/0c54, app
fire_robot_desktop/6734). Trong hai shell Pi riêng chỉ resolve/import metadata,
không init ROS context, đã đối chiếu byte module thực:

- rclpy, tf2_ros;
- fire_robot_mission.startup_health;
- fire_robot_bringup.odom_to_tf_broadcaster;
- fire_robot_bringup.serial_bridge_node.

Cả5 SHA khớp. Các biến ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, RMW_IMPLEMENTATION,
RMW_FASTRTPS_USE_QOS_FROM_XML, FASTRTPS_DEFAULT_PROFILES_FILE,
FASTDDS_DEFAULT_PROFILES_FILE, RMW_FASTRTPS_PUBLICATION_MODE cũng bằng nhau
ở phép resolve hiện tại. DOMAIN unset dùng mặc định0; LOCALHOST=0.

Hai lần runtime lịch sử không lưu đầy đủ môi trường cả laptop/Pi. Supervisor
app đã thoát, không đọc lại được environ. Không dùng shell QA hiện tại để
khẳng định môi trường runtime quá khứ giống tuyệt đối. Chưa thấy bằng chứng
mismatch module/domain/XML gây hai lỗi đã xem; vẫn cần ghi whitelist khi đo.

## Hướng xử lý sau QA, chưa áp dụng

1. Đo latency của TF publisher, callback listener, buffer lookup, thời gian
   spin/sample/ghi file của observer Pi cùng lúc với số reader và tải CPU.
   Source startup_health.py:302 spin_once một callback rồi sample/ghi JSON;
   dấu hiệu backlog cần đo, không chỉ đổi ngưỡng freshness hoặc20s timeout.
2. Đo A/B cùng node/gate R5: app hiện tại so với UI trì hoãn ROS readers tới
   đúng phase. Nếu áp delta này, tắt display đơn thuần chưa chắc bỏ reader
   /tf vì listener thuộc RViz transformation manager. Phải kiểm lại graph.
3. So nhịp STATE mapping và thời điểm STOP-only monitor. Giữ điều kiện STOP/
   watchdog, không chạy chồng hoặc mở khóa bằng cách xóa receipt.
4. Tách startup failure, STOP proof và process cleanup trong app như báo cáo
   QA trước. Hướng sửa app này có bằng chứng độc lập với giả thuyết TF.

Chưa chạy A/B robot hay sửa instrumentation. Lượt này hoàn tất đối chiếu
source/runtime graph và log sẵn có; chưa tuyên bố tìm được nguyên nhân duy
nhất của độ trễ TF. Không tự flash/build/sync hoặc thử actuator.

## Bằng chứng

[10 file và SHA256SUMS](qa_evidence/r5_app_parity_20261029/SHA256SUMS): R5
startup/automatic_start/pi_seal, receipt summary, ba snapshot Pi kèm event
serial/TF đã lọc, module resolution, graph IDLE và script kiểm tra, diff
runner và comparison.json. Không copy pairing/password/envelope xác thực.

[Báo cáo hai lượt app lỗi](QA_STARTUP_BLOCK_215926_220156_2026-10-29.md).
