# QA xe đứng dù đường Nav2 vẫn cập nhật — 31/10/2026

Phạm vi: chỉ phân tích log và chạy mô phỏng trong bộ nhớ; không sửa production,
không đổi cấu hình, không khởi động ROS/robot, không flash hoặc đồng bộ bản mới.
Đã SSH chỉ đọc log phiên cũ trên Pi theo quyền user. Thời gian bên dưới UTC+07.

## Kết luận

Có lỗi tích hợp/phục hồi xác định được: gateway có thể nhả publisher vận tốc
sau một khoảng ngắt lệnh Nav2, sau đó không nhận lại cùng Goal dù lệnh mới đã
trở lại. Operator vẫn báo `NAVIGATION / NAV2_ACTIVE_GOAL` vì Goal cấp cao còn
active. Đây không phải đủ bằng chứng để quy lỗi firmware, motor hoặc chỉ do
vật cản đặt khó. Dừng ban đầu do dự đoán va chạm và không tự đi tiếp sau khi
đường thay đổi là hai sự kiện khác nhau.

## Phiên và bằng chứng

- Laptop: `session_b54372eaaf8644eeb06be87d0fed4601`, operator session
  `f1d474a5-8707-4c47-bb7b-e2f90dd68701`.
- Packet đang chạy và đang chọn:
  `33f9f7db817fb044da8278bdcd7cf8df18b421290bddf424ab4539f5feed1750`.
- Pi: `/home/pi/qa_logs/r5_automatic_20261031_005002`, generation 1.
- Cấu hình: global inflation 0,25 m, local 0,30 m; scaling 5,0/3,0.
- Hai ảnh user chụp 01:00:10 và 01:00:45 nằm trong khoảng gateway đã nhả quyền.
- Evidence local: `qa_evidence/nav_stall_0100_20261031/`. Không sao chép pairing
  secret. Bản sao laptop và Pi được kiểm SHA256; raw log giữ ngoài Git.

| Giờ | Sự kiện xác nhận trong log |
| --- | --- |
| 00:59:12,608 và 12,838 | RPP báo `detected collision ahead`, hủy FollowPath con |
| 00:59:12,858 | Nav2 chạy recovery Spin |
| 00:59:14,458 | Spin báo `Collision Ahead - Exiting Spin` |
| 00:59:14,488–16,489 | Nav2 chạy recovery Wait 2 giây |
| 00:59:15,378 | Gateway ghi `raw publisher released after STOP` |
| 00:59:15,405 | Operator graph: raw_count=0, raw_owner=null, graph vẫn valid |
| 00:59:16,452 | SafetyGate Pi báo không nhận raw command 1063 ms, tiếp tục STOP |
| 00:59:16,538 | Controller nhận lại FollowPath, tiếp tục nhận đường mới |
| 00:59:41,538; 01:00:22,879; 01:00:47,999 | Controller báo `Failed to make progress` |
| 01:00:51,062 | User bấm X, app nhận `NAV_GOAL_CANCEL_REQUESTED` |
| 01:00:52,243 | Operator trở lại `NAV2_READY_NEW_GOAL` |
| 01:01:06,419 | Q xác nhận zero/OFF/home/release trong khoảng 0,408 s |

Giữa gateway release và X là khoảng 95,683 s; log có 149 lần truyền đường mới
vào controller, không có acquire raw publisher mới trong khoảng này. Pi vẫn
ghi serial `HEALTHY epoch=1`, không tăng reconnect/read/tx failure trong đoạn
này. Scan và odom/TF vẫn tiến; không thấy bằng chứng reset Nav2/container khi
bị đứng. Một số lỗi planner và cảnh báo tick/control rate có xuất hiện, được
giữ trong raw; không dùng chúng để suy riêng ra reset hoặc USB lỗi.

Cuối phiên receipt rc=0, STOP và cleanup sạch, pending không còn. Điều này
chỉ chứng minh đóng phiên đúng theo contract, không có nghĩa Goal trước đã
thực hiện thành công. `operator_result.runtime_status=SUCCEEDED` không phải
result NavigateToPose của từng Goal.

## Findings

### P1 — Ngắt vận tốc trong recovery có thể làm mất quyền đến hết Goal

`releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission/fire_robot_mission/live_nav_stop_gateway.py:431`
coi vận tốc shadow quá 0,35 s là không còn hợp lệ. Tại dòng 568–588, đã từng
forward mà gặp blocker thì latch STOP; dòng 596–606 xác nhận đứng/zero rồi
nhả publisher. Nhánh released dòng 553–566 chỉ có các lane rearm đã định nghĩa.

`services/app_task_recovery.py:138` giữ chính sách gốc này: rearm thông thường
cần terminal thật 4/5/6, proof idle rồi Goal mới. Nó không nhận lại cùng Goal
đang EXECUTING sau command gap. App có lane X/terminal để phục hồi có kiểm
chứng, nhưng lane đó không tự chạy khi chỉ bỏ vật cản ra.

Nav2 Humble Wait không phát vận tốc trong `onCycleUpdate`; velocity smoother
cũng ngừng publish sau input timeout khi đã zero. Đây là trạng thái hợp lệ
của Nav2 nhưng không khớp giả định lệnh vận tốc phải liên tục của gateway.
Nguồn chính thức: [Wait Humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_behaviors/plugins/wait.cpp),
[Velocity smoother Humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_velocity_smoother/src/velocity_smoother.cpp).

Timeline thực tế release đúng trong Wait, sau đó Pi mất raw heartbeat; mô
phỏng cùng cơ chế tái hiện đúng trạng thái đứng. **Trigger đầu tiên ở runtime
có thể khẳng định tới mức rất phù hợp command gap trong Wait, chưa có record
per-tick của gateway để chốt đúng blocker đầu tiên.** Không loại bỏ tuyệt đối
mission health/freshness khác chỉ từ log tổng hợp.

Hướng sửa sau khi user yêu cầu: tách dừng tạm do Nav2 recovery khỏi lỗi cứng;
chỉ phục hồi cùng Goal khi có proof mới đúng UUID/session/identity, operator
còn ARM, nguồn/lifecycle/ownership hợp lệ và STOP/OFF cần thiết. Hoặc giữ
zero heartbeat trong một lane Wait được xác minh rõ. Không phát lại vận tốc
cache, không bỏ freshness/watchdog, không tự chạy lại sau X/Q hoặc sau đổi
identity. Giữ source/lệnh R5 cũ bằng wrapper riêng của app như các sửa trước.

### P2 — Giao diện báo Goal active nhưng che mất việc gateway đã nhả quyền

`releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission/fire_robot_mission/operational_session_core.py:754`
chọn phase/reason theo UUID Goal active. Nó không kết hợp kết quả đó với
gateway release khi đặt nhãn `NAV2_ACTIVE_GOAL`. Sau 00:59:15 graph vẫn hợp
lệ vì zero raw publisher được phép ở trạng thái dừng, nên operator không
fault; người dùng vẫn thấy đang điều hướng.

Đề xuất: hiện riêng Goal còn active và motion bị hold/released, kèm lý do và
hướng X/Goal mới khi cần. Thông báo không được giả Goal đã thành công hoặc
đã bị hủy, cũng không được tự mở gate chỉ để sửa nhãn.

### Giới hạn quan sát — Chưa ghi đầy đủ lệnh/costmap/gateway theo thời gian

Không có trace đồng bộ `/cmd_vel_nav`, shadow/raw/cmd, costmap và toàn bộ
gateway state từng tick ở thời điểm lỗi. Vì vậy chưa kết luận được hình
học vật cản có thực sự chặn quay/đường hay có ô vật cản lưu lại; không thể
khẳng định mọi command Nav2 sau đó đều không zero. Giảm inflation trước đó
có thể làm đường gần vật cản hơn nhưng chưa chứng minh là nguyên nhân gây
latch. Không tiếp tục đổi radius/scaling để che lỗi mất quyền.

## Kiểm thử chẩn đoán, không phải sửa lỗi

`audit_nav_wait.py` trong evidence dùng generated operator và các mission
node thật của app trên bus mô phỏng. Nó cấm `rclpy.init`, không tạo DDS hoặc
đụng thiết bị. Bốn ca đều đạt **điều kiện tái hiện/đối chứng**:

1. Đang chạy → zero → ngắt shadow velocity 2 s → trả velocity mới cùng UUID:
   gateway `NAV_INPUT_OR_TOPOLOGY_LOST`, nhả raw; sau 5 s dữ liệu mới vẫn
   không tiến, operator vẫn `NAVIGATION/NAV2_ACTIVE_GOAL`, không fault.
2. Nhánh trên rồi X → terminal thật trong mô phỏng → Goal mới: chạy lại được.
3. Nhánh trên rồi Q: STOP xác nhận trong mô phỏng.
4. Cùng 2 s dừng nhưng tiếp tục gửi zero mới: không latch/nhả; cùng Goal
   chuyển về vận tốc mới thì tiến lại được.

Số 4/4 là kết quả chẩn đoán cơ chế, **không có nghĩa bug đã sửa hoặc robot
đã nghiệm thu**. Raw fixture trước đây luôn phát zero/move trên shadow topic
mỗi tick (`test/r5_full_adapter/harness.py:196`), không mô phỏng publisher
còn tồn tại nhưng ngừng gửi trong Wait. Các ca command gap đã có chủ yếu
kiểm dừng an toàn và X/new Goal; thiếu tiêu chí tự tiếp tục cùng UUID sau
Nav2 recovery. Đó là phần QA cần bổ sung khi sửa.

## Trạng thái sau lượt phân tích

- Production/config/packet R5 không thay đổi, không chọn artifact mới.
- Không chạy robot, actuator, ROS hoặc thử đặt vật cản lại.
- Chưa sửa code. Cần kiểm thử lane phục hồi cùng Goal và thông báo hold,
  với negative cases mất nguồn, sai UUID/identity, raw conflict, X/Q, lửa và
  actuator chưa OFF; không chỉ tăng timeout hoặc replay Goal.
