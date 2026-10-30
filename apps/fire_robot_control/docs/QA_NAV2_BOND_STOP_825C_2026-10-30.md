# QA phiên 825c: Nav2 tự reset lifecycle và kết quả STOP mâu thuẫn

**Kết luận:** có reset lifecycle Nav2 thật trong khi phiên đang chạy, nhưng không
có bằng chứng app giết/chạy lại tiến trình Nav2. Nav2 tự reset vì mất bond heartbeat
với `bt_navigator` và `map_server`. Khi Q sau đó, verifier xác nhận STOP sau 0,40267 s;
app tổng hợp thành STOP_UNCONFIRMED vì operator rc2 khi còn fault runtime cũ.

Lượt này chỉ đọc/replay và ghi tài liệu. Không sửa production code, thử robot,
sync, chọn packet, thay network/clock, giảm guard hay xóa pending.

## Phiên và bằng chứng

- Session folder: `/home/huudinh/.local/state/fire_robot_control/session_825c1487439e4cb082197def964e6fe9`.
- Session UUID: `a3c7ea9a-4254-4f51-9039-1df369125c66`.
- Packet app: `e2a8152ba98beb4f792e5ea2bf848672137c8bc2cbdb275ccd17e314e12cd1f8`.
- Pi root trong receipt: `/home/pi/qa_logs/r5_automatic_20261030_110002`.
- 18 log/receipt chọn lọc được sao chép và kiểm SHA256:
  [result.json](qa_evidence/nav_bond_825c_20261030/result.json).
  Không sao chép pairing secret hoặc runtime rebind nonce.

## F1 — P1: Nav2 tự reset cả navigation và localization khi đang vận hành

Trong `raw_laptop/laptop/nav.log`:

- Dòng 3: đúng **một** lần container khởi chạy, PID **378441**.
- Dòng 408, **11:05:05,658**: lifecycle manager navigation báo bt_navigator mất
  heartbeat **4000 ms**, rồi `Resetting managed nodes...` tại dòng 410.
- Dòng 420, **11:05:11,566**: localization manager báo map_server mất heartbeat
  **4000 ms**, rồi reset cả nhóm localization tại dòng 422.
- Dòng 592/639: hai manager tự nhận lại kết nối và bringup trở lại.
- Localization active lại **11:05:59,399**; navigation active lại **11:06:17,263**.
  Không có dòng container khởi chạy lần thứ hai.

App recovery chỉ retire role mission: source R5 bất biến
`releases/r5_20261029/ros/scripts/runtime_recovery_runner.py:14,44,51`.
Hai retirement receipt thật cũng chỉ chứa role mission. Nav2 wrapper PID **378415**,
container PID **378441** được giữ; cờ `nav_restart_allowed=false` nói về hành động
của runner, **không bảo đảm lifecycle manager nội bộ không tự reset**.

Đặc biệt, `APP_NAV_SHUTDOWN LIFECYCLE_BEGIN` đầu tiên chỉ có ở **nav.log:721**, sau
khi người dùng nhấn Q khoảng **11:07:00**. SIGINT/rclcpp shutdown chỉ xuất hiện
ở dòng 805, **11:07:13,039**. Reset lúc 11:05 xảy ra trước đoạn cleanup mới gần
hai phút, nên không quy reset runtime này cho helper shutdown vừa bổ sung.

**Đã chắc:** nguyên nhân trực tiếp của reset là Nav2 bond timeout; không phải app
relaunch container hoặc yêu cầu reset Nav2 trong recovery.
**Chưa chắc:** nguyên nhân sâu làm bond heartbeat trễ. Không có trace scheduling,
CPU/memory theo tiến trình hoặc DDS bond transport tại thời điểm lỗi để phân biệt
native call bị giữ, CPU/swap, DDS/backpressure hoặc yếu tố khác. Không suy ra chỉ
cần tăng bond timeout hoặc kết luận lỗi WiFi/ESP32/model từ dòng heartbeat.

Hướng QA tiếp: quan sát bounded thời gian callback/bond/DDS và scheduling trên
laptop, đối chiếu serial Pi đúng window. Dùng bài cô lập để tái hiện trước khi
thay cấu hình. Không tắt bond, watchdog hoặc logic STOP để tránh reset bề ngoài.

## F2 — P1: verifier STOP true bị tổng hợp thành STOP_UNCONFIRMED do rc của phiên fault

Bằng chứng thật:

- `operator.log:187`: fault `RUNTIME_RECOVERY_HOLD:NAV2_BACKEND_LOST` trước Q.
- `operator.log:203`: UI nhận `OPERATOR_Q`, không phải lease expired.
- `operator.log:204`, **11:07:00,733**: `OPERATOR_STOP`, `cause=Q`, `confirmed=true`,
  `FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED`, blockers rỗng, elapsed **0,402669 s**.
- `operator_ui.json`: max polling gap **15,76 ms**, max delivery delay **10,31 ms**;
  `stop_request.json`: UI heartbeat age **81,70 ms**. Không có bằng chứng lỗi UI
  liveness cũ lặp lại trong lượt Q này.
- `cleanup.json/processes/operator`: rc **2**, unforced, group đã exited.
- `cleanup.json/stop_confirmed=false` và lỗi `OPERATOR_STOP_UNCONFIRMED`,
  `PROCESS_UNCLEAN:operator`; desktop cũng báo false.

Chuỗi source đã xác minh:

1. `apps/fire_robot_control/services/operator_runner.py:109–110` đặt rc2 nếu
   `node.core.fault` còn khác None, dù verifier STOP đã confirmed.
2. Đây là điều kiện kế thừa từ R5 gốc tại
   `releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission/fire_robot_mission/operational_session.py:501`.
3. `releases/r5_20261029/ros/scripts/operational_stack.py:43` chỉ đọc receipt khi
   operator rc0. Với rc2, nó bỏ qua cả receipt STOP true rồi dòng 49 thêm
   `OPERATOR_STOP_UNCONFIRMED`.
4. App nhận kết quả tổng hợp đó, trả BLOCKED/STOP_UNCONFIRMED. Không có failure.json
   cung cấp nguyên nhân cụ thể nên UI fallback RUNNER_EXIT_2.

Đã replay **code thật** bằng fake ROS/time/channel, không tạo ROS context:

| Core fault | Verifier STOP | Operator rc | Stack chấp nhận STOP |
| --- | --- | --- | --- |
| None | true | 0 | true |
| RUNTIME_RECOVERY_HOLD:NAV2_BACKEND_LOST | true | 2 | false |

Ngoài ra, đưa nguyên receipt STOP thật vào StackCleanup: rc2 → false,
rc0 → true; không sửa nội dung receipt giữa hai lượt. Điều kiện rc là đủ tạo
mâu thuẫn; không cần giả định telemetry STOP thất bại.

Hướng sửa đề nghị: tách **kết quả vận hành phiên** khỏi **bằng chứng STOP hiện tại**
và **process cleanup**. Giữ fault/rc2 lịch sử; không đổi rc0 chỉ để có PASS. Chỉ
chấp nhận STOP khi receipt mới, đúng process/session, có đủ verifier proof và
không có lỗi ownership/forced/context/parse. Không tự xóa pending bằng kết quả
replay; physical STOP vẫn chưa được kiểm chứng mới trong lượt QA này.

## F3 — P2: phục hồi giữ mission chờ, nhưng UI không nêu rõ nguyên nhân

Timeline recovery thật:

| Giờ Việt Nam | Sự kiện |
| --- | --- |
| 11:02:48 | ARM, identity epoch 8 |
| 11:03:51 và 11:04:27 | Hai goal Nav2 SUCCEEDED |
| 11:04:43,761 | Identity epoch 9 khác epoch 8 đã arm; fault BOOT_OR_SERIAL_EPOCH_CHANGED |
| 11:04:53,810 | Rebind generation 1, `nav_restarted=false`, không yêu cầu Initial Pose mới |
| 11:05:05 / 11:05:11 | Hai lifecycle manager Nav2 tự reset vì bond timeout |
| 11:05:58,362 | Operator arm lại ở epoch 12 |
| 11:05:59,745 | Mất nguồn action status `/navigate_to_pose/_action/status` trong lúc Nav2 reset |
| 11:06:03,605 | Recovery generation 2 vì NAV2_BACKEND_LOST, retire mission, giữ operator/Nav2 |
| 11:06:17,263 | Nav2 lifecycle active lại, cùng container PID |
| Trước/đầu Q | Recovery vẫn WAIT_EXISTING_NAV, chưa tạo mission generation 2 mới |
| 11:07:00,733 | Verifier STOP Q confirmed |

`runtime_recovery.json` giữ generation 2 ở `WAIT_EXISTING_NAV`.
`runtime_recovery.py:140–144` đòi pose hiện tại, ổn định 5 s, IO, YOLO, lifecycle
và đúng identity trước khi mở mission mới. Snapshot health cuối ghi IO true,
`lifecycle_active=true`, `initial_pose_seen=true`, nhưng **localization/pose false**.
Snapshot này được chốt đầu cleanup, không phải trace đủ mọi thời điểm của window.
Chưa đủ chi tiết để kết luận thiếu AMCL pose mới, map→odom TF, action hoặc nguồn
pose cụ thể. Không khuyên đặt lại pose chỉ từ nhãn READY_GATE2.

Việc giữ mission khi chưa đủ proof là đúng. Thiếu sót là thông báo chưa cho người
dùng biết đang chờ gì: `services/operator_status.py:71–72` rút mọi fault thành
“Có lỗi — kiểm tra chi tiết”, trong khi dòng phiên còn READY_GATE2 (mốc startup).
Nên hiển thị mã runtime recovery, lifecycle/localization và blocker cụ thể;
không dùng startup READY_GATE2 làm bằng chứng hiện đang sẵn sàng nhận goal.

## Kết quả bản sửa cleanup trước và quan hệ với R5

Trong chính phiên lỗi mới, Nav2 cleanup đã sạch:

- Nav child **1/1 clean**, YOLO **2/2 clean**, không signal escalation.
- Hai mission generation được retire đều **9/9 child clean**.
- Tất cả local groups có group_exit_confirmed true, forced false.
- Hai lifecycle manager xác nhận shutdown qua helper mới, rồi container thoát rc0.
- Receipt Pi lưu tại laptop: 2 group exited, **8/8 child clean**, confirmed true.
  Có một diagnostic `PI_CONTROL_TRANSPORT:STATE`, sau đó remote exit vẫn có proof;
  không dùng riêng lỗi transport này làm nguyên nhân Nav2 reset trên laptop.

Vì vậy lỗi Q lần này khác lỗi Nav2 forced/-15 trước: lỗi tổng hợp STOP do fault
rc2, dù process cleanup đã hoàn tất. `group_cleanup_clean` hiện vẫn false vì
StackCleanup trộn điều kiện STOP với cleanup; không suy ra còn tiến trình sống
chỉ từ dòng “cleanup cần kiểm tra”.

**Có liên quan phần R5 app đang tái sử dụng:** lifecycle Nav2 và policy recovery,
điều kiện rc khi fault, cùng StackCleanup. **Không có bằng chứng R5 bị sửa hỏng:**
364 file release gốc khớp manifest `0c54…`; lượt QA này không chạm R5/app source.
Không suy rộng verdict này sang mọi candidate R5 khác đang phát triển riêng.

## Bằng chứng phụ và giới hạn

- Cùng window, app ghi native query trước publish **2,801 s / 1,637 s / 2,021 s**
  (`operator.log:110,114,117`). YOLO có callback total tối đa **2,823 s**, tuổi
  ảnh đầu callback tối đa **1,732 s** ở window 11:04:41–11:05:36. Đây là tương
  quan trễ ở nhiều tiến trình, chưa chứng minh quan hệ nhân quả với bond timeout.
- Model thật trong log vẫn **best_minireplay.pt**, RTX3050 FP16, confidence 0,15;
  không tự đổi model hoặc kết luận model gây reset.
- Có 4544 cảnh báo JPEG ở health và 3347 ở YOLO; snapshot frame proof vẫn fresh,
  decode_errors=0. Cảnh báo không đồng nghĩa mọi frame decode thất bại hoặc đã
  chứng minh network nghẽn. Health log cũ chỉ có cảnh báo JPEG, thiếu trace blocker.
- Identity cùng bridge instance/ELF, epoch đổi 8→9 rồi 12. Chưa đọc được serial
  Pi trực tiếp nên không gọi đó là số lần MCU reboot hoặc lỗi dây USB.
- Một lần xin quyền đọc Pi quá hạn auto-review trước khi chạy. Retry một lần
  với lệnh chỉ đọc pi.log đã được phép nhưng **SSH 10.0.0.1 timeout**. Không có
  log Pi mới, journal, CPU hoặc DDS trace trong lượt này; receipt Pi nói trên
  là bằng chứng đã lưu của phiên, không phải xác minh hiện tại.

## Verify đã thực hiện và phần chưa làm

[replay.py](qa_evidence/nav_bond_825c_20261030/replay.py) có **14 kiểm tra chẩn đoán
PASS**, gồm parser receipt thật, Nav một process, reset trước helper Q, hai
retirement chỉ mission, mâu thuẫn rc/STOP và fake-ROS runner, source/hash không đổi.
Đây là xác nhận phát hiện, không phải test rằng lỗi đã được sửa hoặc nghiệm thu
dừng vật lý. Source app/native, current, pending và cặp map trước/sau giữ hash.

Lệnh chạy lại không ROS/network/robot:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 apps/fire_robot_control/docs/qa_evidence/nav_bond_825c_20261030/replay.py
```

Chưa sửa code, clear fault, xác minh STOP mới, replay goal, publish command, sync,
flash hoặc chạy lại robot. Nên sửa F2 ở lớp app bằng regression receipt này;
với F1, cần chốt nguồn trễ bond trước khi sửa runtime. Không nới timeout bừa
hoặc tắt cơ chế an toàn để làm giao diện hết BLOCKED.
