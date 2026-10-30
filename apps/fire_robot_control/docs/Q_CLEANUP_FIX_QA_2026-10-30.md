# Sửa cleanup khi Q và QA — 30/10/2026

Gói đã chọn và đồng bộ Pi: `e2a8152ba98beb4f792e5ea2bf848672137c8bc2cbdb275ccd17e314e12cd1f8`.
Parent app: `d5209520c8f894a0996f3e3841ba2709b945192f1cf38087c3995907d1a2e222`.
R5 gốc: `0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0`, 364 file nguyên byte.

## Lỗi có bằng chứng trước khi sửa

[QA phiên 7ed5b7dc](QA_Q_CLEANUP_7ED5B7DC_2026-10-30.md) xác nhận:

1. Q/STOP đã đạt sau 0,402 s; Nav2 gặp lifecycle transition chồng trong shutdown,
   vượt inner 5 s rồi outer 8 s, bị SIGTERM; cleanup thực sự không sạch.
2. Nhánh phục hồi cũ chỉ nhận STOP unconfirmed/operator rc2, từ chối chính receipt
   thật STOP confirmed/operator rc0 trước cả SSH.
3. `RUNNER_EXIT_2` che lý do cleanup và làm kết quả startup READY bị ghi FAILED.

Năm test hồi quy lấy từ receipt thật chạy **trước sửa**: 1 PASS, 2 FAIL, 2 ERROR
(hai lỗi là nhánh phục hồi bị từ chối và hàm phân loại startup chưa có).
Log đầu giữ tại `qa_evidence/q_cleanup_fix_20261030/regression_before.log`.
Sau sửa, 5 test đều PASS; 9 biến thể receipt sai/child còn sống vẫn bị từ chối.

## Thay đổi vận hành, chỉ cho app

- `services/nav_session.py`: app sở hữu một wrapper cùng process group với
  ros2 launch, container Nav2 và helper; không setsid/daemon con tách rời.
  Chỉ khi role nav nhận STOP, helper mới tạo ROS client gọi SHUTDOWN cho
  lifecycle manager navigation, rồi localization; sau đó gửi SIGINT cho launch.
  Trước đó StackCleanup vẫn hoàn tất operator STOP và mission drain theo thứ tự cũ.
- Helper không có publisher motor/bơm/arm. Native ROS call bị treo được chặn bởi
  helper process với ngân sách 12 s; dịch vụ chưa có hoặc từ chối vẫn quay về
  shutdown launch có giới hạn. Q lặp không tạo helper mới hoặc kéo dài ngân sách.
- `services/app_local_processes.py`: chỉ nav của app chữa cháy có outer SIGINT
  grace **29 s**, đủ cho RPC 12 s, launch 8 s và phần dự phòng. Grace TERM 3 s/KILL
  2 s, process ownership, child audit và STOP receipt gốc giữ nguyên. Đây là
  thời gian dọn tiến trình **sau STOP**, không tăng timeout dừng motor/lease/watchdog.
- `services/desktop_hooks.py`, `candidate_build.py`: nối wrapper và lớp process
  riêng trong entrypoint app được sinh. Thời hạn launch SIGINT→TERM 8 s, TERM→KILL
  3 s. Không sửa launch, container hay runner R5 gốc. Mapping vẫn dùng lớp
  LocalProcesses và SLAM command cũ; không đi qua wrapper Nav2 mới.
- `stop_recovery_runner.py`: bổ sung đúng biến thể confirmed STOP + operator rc0
  + hai lỗi Nav2/child audit đã quan sát. Vẫn kiểm profile, group, tín hiệu, rc,
  PID container, session/packet và sự vắng mặt hiện tại. Ngoại lệ chỉ cho phép
  chuẩn bị **phiên xác minh STOP mới**; không tự mở khóa hoặc sửa receipt cũ.
  Nhánh unconfirmed cũ và mọi điều kiện audit/new Pi STOP/boot giữ nguyên.
- `session_outcome.py`, `live_session.py`: hiển thị `NAV2_SHUTDOWN_TIMEOUT` cho
  tổ hợp cleanup đã biết, chỉ profile R5_AUTOMATIC; giữ kết quả startup READY độc lập với lỗi teardown.
  Startup chưa qua gate vẫn FAILED/CANCELLED theo nguyên nhân gốc.

Không đổi QoS, topic/type, TF, ROS domain vận hành, GPIO, firmware, safety watchdog,
UI lease 350 ms, STOP verifier 4 s hoặc chính sách cấp quyền motor/bơm.
Model app vẫn `best_minireplay.pt`, SHA256 `e792f896faf3fb584842898e93913cdfd0fc33b5d48d620163fadd15f02212c4`.

## Kiểm chứng Nav2 thật, không nối robot

Probe dùng ROS localhost/domain **216**, TF/scan/odom giả, command Nav2 vẫn đi
vào shadow topic. Không dùng thiết bị, goal vật lý hoặc actuator. Trong sandbox,
UDP bị lọc nhưng các tiến trình ROS giao tiếp nội bộ qua transport khả dụng:
GetState ACTIVE, goal/result và ManageLifecycleNodes nhận phản hồi thật.

| Ca | Kết quả |
| --- | --- |
| Baseline R5 đứng chờ, không goal | rc0, child clean, khoảng 1,67 s |
| Baseline sau goal SUCCEEDED | rc0, child clean, khoảng 1,52 s |
| Gọi lifecycle shutdown tuần tự trước SIGINT, sau goal | cả hai manager success; rc0, child clean, 1,80 s |
| Wrapper mới sau goal SUCCEEDED | rc0, child clean, 1,87 s |
| Wrapper mới nhận STOP trong startup | rc0, child clean, 3,88 s |
| Wrapper mới khi goal còn ACTIVE | goal chưa terminal lúc Q; rc0, child clean, 2,92 s |
| Wrapper **từ artifact cuối** sau goal | rc0, child clean, 1,77 s; group không còn live PID trước test cleanup |

Các ca wrapper không có signal escalation hoặc lifecycle transition error như
đoạn lỗi thật. Log xác nhận hai lifecycle SHUTDOWN xong trước `LAUNCH_SIGINT`.
Test cuối kiểm tra group **trước** khi bộ test tự dọn để không che tiến trình sót.

**Giới hạn:** không tái hiện được race/timeout hiếm của phiên thật chỉ bằng
baseline synthetic này; baseline cũng thoát sạch. Vì vậy kết quả chứng minh
đường shutdown có thứ tự và bounded fallback hoạt động, chưa chứng minh mọi
nguyên nhân treo Nav2 đã hết hoặc đã nghiệm thu robot qua WiFi.

## Hồi quy và các lần chạy bị từ chối

- **266 unittest PASS**, gồm 254 cũ và 12 test mới: receipt thật, biến thể sai,
  helper chỉ sinh khi Q, Q lặp, helper thất bại/treo, launch bỏ qua tín hiệu,
  outer TERM và command không hợp lệ. Không coi mock là physical STOP proof.
  Review cuối bắt thêm nguy cơ gắn nhầm nhãn Nav2 cho cleanup SLAM; đã giới hạn
  đúng profile chữa cháy và thêm test riêng. Gói 4078650f trung gian đã qua bộ
  tích hợp dưới; gói cuối e2a8152b bổ sung điều kiện profile này, chạy lại 266
  unit, CTest, **56 ca tích hợp bị ảnh hưởng và smoke gói cuối**, đều PASS. Wrapper/Nav2 binary
  không thay đổi giữa hai gói, kế thừa probe Nav2 cô lập có kiểm hash.
- **186 ca tích hợp cũ đã PASS** theo hợp các lượt sau, không cộng lượt chạy lại:
  lần đầu 165 PASS, 21 ca không đạt do khai báo nhầm domain 216 trong khi fixture
  yêu cầu 211 và thiếu thư mục output health. Đã sửa môi trường chạy, không sửa
  guard/test để bỏ điều kiện. Chạy lại 21 ca đó và smoke artifact mới: 21 PASS,
  1 FAIL ở worker health **sampled**. Chạy riêng đúng ca đó: 1 PASS/7,65 s.
- Lần sampled lỗi `DIAGNOSTIC_WORKER_FAILED`, health.ready có nhưng thiếu
  health.json, không có traceback trong health.log. Parent diagnostic có cơ chế
  kill worker nếu không thoát trong 4 s, nhưng log cũ không lưu return code từng
  worker nên chưa khẳng định chính xác worker bị kill hay nguyên nhân native.
  Giữ đây là bất ổn của bài thử cần theo dõi; không gọi lần lỗi là PASS hoặc suy
  ra do tải. Mode health vận hành app vẫn **r5**, source health không đổi.
- Native build + **CTest Qt PASS**. Lần build đầu trong sandbox bị cấm listen
  Unix socket nên CTest fail; chạy lại với quyền socket nội bộ hợp lệ đạt.
- Unittest lần đầu thiếu environment ROS và bị cấm Unix socket; không dùng làm
  verdict code. Lần đầy đủ cuối đúng môi trường 266 PASS/15,11 s.
- Một đề nghị quyền chạy probe localhost hết thời hạn auto-review trước khi
  khởi chạy. Probe sau đó thực hiện được trong sandbox, không nới điều kiện
  hoặc mở kết nối đến robot để thay thế.
- Packaged native GUI mở/đóng IDLE, auth dialog, map speed 0,1/0,8, map save,
  robot/path overlays, Q/X theo các pha, heartbeat/recovery và startup regressions
  nằm trong các bộ đã PASS. Không lặp bài phun hoặc chạy motor thật.

## Đồng bộ và bảo toàn

- Source packet cuối **436 file**; Pi stage
  `/home/pi/fire_robot_desktop/e2a8152ba98b/ros`.
- WiFi `10.0.0.1` timeout. Route chỉ đọc xác nhận laptop hiện dùng LAN trực tiếp
  `10.42.0.1 → 10.42.0.185`; SSH qua LAN thành công.
- Đồng bộ tĩnh gói cuối sang thư mục riêng: build 2 package/8,98 s; **103/103 installed
  source Pi**, receipt errors rỗng. Script từ chối nếu Pi đang chạy robot; không
  chạy node, NTP, flash hay ghi đè baseline/R5.
- Laptop static inventory: **95 installed sources**, errors rỗng. Artifact,
  binary/archive/source manifest đúng hash; chỉ chọn `current.txt` sau kiểm tra
  common laptop lock rảnh và không có tiến trình robot.
- 364 source R5 gốc và bản copy trong artifact khớp từng byte; bảy file
  mapping/map/health/model được bảo vệ khớp bản d520. Cặp map người dùng TN1
  không đổi. Hash pending giữ
  `1daf69a2057658315f9dc127f78f6162eebffb4f9fade9ef74b50afe01e4c153`;
  14 log/receipt của phiên lỗi giữ nguyên hash.
- Live preflight hiện từ chối `LAN_MUST_BE_DISCONNECTED_FOR_WIFI_PROFILE` vì
  LAN đang UP. Đây là guard R5 được giữ nguyên. Không sửa network/guard để test
  chạy tiếp, và không gọi live preflight đã PASS. Static inventory riêng đã PASS.

## Bước dùng bản mới

1. Đóng cửa sổ app cũ rồi mở lại icon để nạp packet e2a8152b.
2. Khi thử vận hành profile WiFi, rút LAN và nối laptop với WiFi Pi. Cần giữ kết
   nối internet riêng nếu muốn tiếp tục trao đổi trong lúc thử.
3. Pending cũ vẫn còn: bấm **Xác minh dừng trên robot**, chỉ tiếp tục khi xác
   minh thật thành công và app về IDLE. Không xóa pending thủ công.
4. Sau đó mới thử phiên chữa cháy có giám sát và Q. Tiêu chí: STOP confirmed,
   hai manager shutdown trước SIGINT, child/group exit clean, Pi clean, app IDLE.
   Nếu lại BLOCKED, giữ log; không bấm liên tiếp để bỏ guard.

Lượt sửa này chưa mở app vận hành, khởi động robot hoặc chạy xác minh STOP thật.
F2/F3 đã đóng ở mức replay/integration; F1 có đường shutdown riêng đã kiểm chứng
trên Nav2 thật cô lập, còn chờ delta runtime robot. Không gọi toàn dự án hết bug.

## Bằng chứng và lệnh

Thư mục [qa_evidence/q_cleanup_fix_20261030](qa_evidence/q_cleanup_fix_20261030/).
Các file chính: `unittest_final.log`, `pytest_full.log`,
`pytest_environment_corrected.log`, `health_serial.log`, `package_final.log`,
`pytest_final_delta.log`, `pi_sync_final.log`, hai install receipts,
`release_checks.json`, các thư mục probe Nav2. Log không có hậu tố final giữ
trạng thái gói trung gian hoặc lần lỗi môi trường, không gộp thành lần PASS cuối.

```bash
# Pure receipt regression; không ROS/network/robot
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=apps/fire_robot_control:apps/fire_robot_control/tests:releases/r5_20261029/ros/scripts \
  python3 -m unittest discover -s apps/fire_robot_control/tests -p test_q_cleanup_regression.py -v

# Probe Nav2 thật: phải source ROS/installed overlay và cô lập localhost216;
# chọn thư mục --out mới để giữ log cũ, không chạy chung với robot.
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=216 ROS_LOG_DIR=/tmp/fire_nav_cleanup_ros_logs \
  PYTHONPATH=apps/fire_robot_control python3 apps/fire_robot_control/tests/probe_nav_shutdown.py \
  --goal --wrapper --out /tmp/fire_nav_shutdown_new_result
```
