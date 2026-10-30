# Sửa X, thu dọn chu trình và nhận lại Goal — 30/10/2026

Gói source cuối:
`ad0a51a757b29651c331361c10eac147cbc093de2d5a5f6f265c4d19a7f33ca3`.
Thư mục evidence: `qa_evidence/task_cancel_fix_20261030/`.

## Hành vi được sửa

- X giữ ý định hủy, ngăn automatic FIRE_WATCH tạo lại chu trình lửa cũ ở
  tick kế tiếp. Sau STOP/OFF/HOME và retirement, có thể đặt Goal mới dù lửa
  cũ còn trong ảnh. Khi tín hiệu clear ổn định, quan sát tự động có thể trở
  lại; dữ liệu trễ không được coi là clear.
- X hoặc target-lost trước khi guard gửi CancelGoal: giữ đăng ký ACTIVE và
  gửi hủy đúng Goal qua đường retirement riêng, không phụ thuộc lửa còn
  nhìn thấy. Chỉ sau kết quả terminal thật mới đi PREPARE/COMMIT.
- Goal gốc tự SUCCEEDED/ABORTED trước khi hủy không bị giả thành CANCELED.
  Dùng policy terminal-before-pulse vốn có của R5 với status thật.
- Goal khác đến trong lúc hủy: thu dọn Goal đang active sau khi Goal gốc đã
  terminal thật, đúng policy trước pulse, cùng identity và STOP. Không hủy
  mọi Goal bằng UUID zero và không tiếp tục chạy Goal cũ.
- Sau lỗi input, X xử lý cả UUID gateway đang giữ và UUID Nav2 hiện tại.
  Có lưu terminal fact đã được kiểm nguồn/identity cho trường hợp Nav2
  lược bỏ status cũ. Nếu báo cáo gateway tạm trễ khi bấm X, chỉ UUID mục
  tiêu được nhớ từ bản ghi cũ; STOP/ownership/identity vẫn phải dùng proof mới.
- Goal mới đến sớm giữ zero trong lúc xác nhận dừng liên tục rồi mới nhận
  quyền. Khoảng trống callback không được cộng dồn thành proof idle liên tục.
- UI mở công cụ Goal theo admission do app/gateway xác nhận, cùng mọi gate
  freshness/ARM/hold/fault cũ. Hiện rõ đang hủy/chờ dừng/OFF/HOME; kết quả
  CancelGoal chưa xác định có hướng dẫn Q để dừng và kiểm tra phiên.

## Nguồn lỗi và phạm vi thay đổi

Các lỗi đã QA có phần nằm trong code R5 kế thừa: guard chỉ nhận intent
FIRE_ACQUIRED, release yêu cầu hủy Goal trước đó, X không có cycle không
ngăn auto bật lại FIRE_WATCH, và normal rearm không xử lý lịch fault/UUID
đã thay. Vì người dùng yêu cầu giữ R5 đang dùng, **không sửa 364 file trong
release R5 hoặc các lệnh terminal R5**.

- `services/app_task_recovery.py`: các subclass app của session, handback,
  gateway và endpoint/cancel retirement. Operator vẫn chỉ phát ý định;
  CancelGoal thật vẫn do guard gửi, motor/pump vẫn do owner hiện có.
- `services/app_operator_source.py`: generated operator dùng AppSessionCore
  và AppHandback, giữ các sửa timing/Q trước đây.
- `services/app_mission_node.py`: wrapper cục bộ cho đúng hai executable
  gateway/guard. Không sửa file installed R5, không khởi tạo ROS khi import.
- `services/candidate_build.py`: sinh launch Gate2 riêng của app, prefix
  hai node đó và giữ exec_name cũ để audit child cleanup không nhầm python3.
- `services/desktop_hooks.py`: chỉ thay launch Gate2, giữ observe mode gốc.
  Mapping runner, teleop và map saver không đi qua wrapper này.
- `native_rviz/goal_admission.hpp`, `main.cpp`: khóa Goal theo report app
  và hiển thị trạng thái hủy. Kiểm C++ dùng đúng predicate của UI.
- Test mới/bổ sung: X/Goal, lịch hủy đua nhau, proof âm, process-prefix và
  label cleanup; fixture model bổ sung mission command thật để kiểm cả hook.

## Các điều kiện an toàn được giữ

Retirement intent phải đến từ operator GID đã pin, đúng scope/schema,
request/cycle/Goal UUID, cùng boot/epoch/ELF, control còn hạn, graph hợp lệ,
không owner raw/pump, odom đứng yên, cmd zero và actuator OFF mới. Đường gửi
còn giữ enable/ARM/service/topology của guard. Nó chỉ thay admission theo
ý định hủy khi intent nhận lửa không còn hợp lệ.

Gateway chỉ phục hồi khi có terminal thật cho đúng UUID và cùng session
hiện hành; có đủ nguồn, ARM, cmd/odom/actuator mới và khoảng dừng liên tục.
Mất nguồn, đổi identity, epoch, owner không rõ, dữ liệu sai hoặc telemetry
không trở lại không được mở quyền bằng cache. Q/STOP, watchdog, QoS, timeout
an toàn, firmware, tốc độ quét map và model không thay đổi.

`best_minireplay.pt` giữ SHA
`e792f896faf3fb584842898e93913cdfd0fc33b5d48d620163fadd15f02212c4`.
Parent R5 manifest giữ SHA
`0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0`.

## QA và giới hạn

- Broad regression: `full_pytest.xml`, **795 PASS, 2 SKIP**, 398,80 s. Có
  một warning Python/RViz đã có. Bao gồm timeout/recovery, map, UI/IPC,
  X/Q, nguồn dữ liệu, auth, cấp quyền và cleanup.
- Sau broad run, thêm kiểm nhãn child launch và ca X đúng lúc mất báo cáo
  gateway. Ca sau đã FAIL trước sửa và PASS sau sửa. Các nhóm bị ảnh hưởng
  được kiểm lại trên source cuối: **140 PASS, 2 SKIP**, 187,97 s; xem
  `final_acceptance.xml`. Gói cuối mở/đóng IDLE **1 PASS**, 2,66 s.
- Hợp nhất theo test ID, lấy kết quả mới nhất: **798 PASS, 2 SKIP, 0 FAIL**
  (800 test duy nhất). Không cộng các lần chạy lặp thành test mới.
- 54 ca proof âm/continuity: không bypass khi tag sai, phiên đóng/fault,
  source/identity đổi, raw/pump còn owner, actuator ON/stale/LOCKED,
  odom/cmd không dừng, status mâu thuẫn/future, request quá hạn hoặc thiếu
  policy terminal trước pulse. Có kiểm proof idle không sống qua khoảng
  trống scheduling.
- Native build và CTest đạt, có kiểm Goal thiếu admission, stale/future,
  hold, active cycle, fault, unarmed và runtime rebind phải khóa.
- Artifact có 446 file source, archive/binary được kiểm hash; smoke binary
  chỉ mở/đóng IDLE trong state-dir tạm và ROS localhost211. Laptop static
  **95 file** đúng hash, errors=[]; đây chưa phải kiểm chuyển động robot.
- Không đếm các log first/debug thất bại thành PASS và không xóa evidence
  cũ. Chỉ test/helper thay đổi sau broad run được ghi rõ, không gọi mọi tổ
  hợp phần cứng đã nghiệm thu.

Lệnh kiểm acceptance cục bộ (không ROS init/DDS/thiết bị trong các bus test):

```bash
source /opt/ros/humble/setup.bash
source /home/huudinh/.codex/worktrees/laptop-baseline/NCHK_robotFirefighting_ROS/ros2_ws/install/local_setup.bash
source .codex_qa_backups/r5_operational_candidate_20261026/ros/ros2_ws/install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/apps/fire_robot_control/tests:$PWD/releases/r5_20261029/ros/scripts:$PYTHONPATH"
python3 -m pytest -q -p no:cacheprovider \
  apps/fire_robot_control/tests/qa_x_cycle_audit.py \
  apps/fire_robot_control/tests/qa_goal_retry_audit.py \
  apps/fire_robot_control/tests/qa_x_gap_audit.py \
  apps/fire_robot_control/tests/qa_task_cancel_races.py \
  apps/fire_robot_control/tests/qa_task_cancel_guards.py \
  apps/fire_robot_control/tests/qa_app_mission_wrapper.py
```

## Những trường hợp vẫn phải chờ/dừng

Nếu gửi CancelGoal lỗi hoặc không rõ response, không gửi lặp mù và không
giả kết quả. UI báo lý do, Q vẫn có đường STOP/cleanup; nếu chưa có proof
STOP/cleanup thì VERIFY_STOP phải kiểm hiện trạng và lấy proof mới. Đây
không phải nút xóa mọi khóa. Mất telemetry thật, identity đổi, thiếu HOME
sau phun hoặc tiến trình còn sống vẫn bị chặn đúng thiết kế.

Chưa thử xe/bơm, Goal thật, mất WiFi/USB hoặc nguồn thật trong lượt này.
Static sync và chọn version được ghi riêng trong `selection.json` khi đạt;
không dùng bản gói tạm 9c6128/0b9a488/864bb262 thay cho gói cuối ad0a51a7.

## Đồng bộ và chọn bản cuối

Đã đồng bộ LAN `10.42.0.185`, chỉ build/hash, không ROS launch/NTP/flash:
2 package build xong 10,6 s tại
`/home/pi/fire_robot_desktop/ad0a51a757b2/ros`. Pi static **103 file** đúng
hash, receipt đúng packet, nguồn có **446 pin**. Laptop static **95 file**
đạt. Xem `pi_sync.log`, `pi_install_receipt.json`, `release_ready.json`.

Sau kiểm không có pending, common lock laptop rảnh và receipt khớp, đã chọn
`artifacts/current.txt` từ f63e38d71f5e sang ad0a51a757b2. Bản cũ giữ nguyên
để đối chiếu. Không đóng/mở lại app người dùng hoặc phát lệnh robot. Cửa sổ
app cũ nếu còn mở vẫn là tiến trình bản cũ; cần đóng rồi mở lại để dùng bản
mới. `robot_runtime_tested=false` trong `selection.json` giữ đúng thực tế.

## Kiểm thử có giám sát sau khi mở lại app

1. Đóng cửa sổ app cũ rồi mở lại. Kiểm phiên mới đúng gói ad0a51a7; không
   tái sử dụng tiến trình đang chạy gói cũ.
2. Chọn map, khởi động, đặt Initial Pose và chờ READY.
3. Đặt Goal → X → chờ hoàn tất hủy/dừng → đặt Goal khác. Lặp X hai lần và
   thử Goal kết thúc trước khi bấm X. Không cần đổi model hoặc chạy lệnh R5.
4. Kiểm X khi mới phát hiện lửa, đang xoay, IR, trước phun, sau phun/HOME;
   xe/bơm phải dừng và không tự bắt lại chu trình vừa hủy.
5. Q cuối phiên phải xác nhận STOP và cleanup sạch. Nếu không, giữ nguyên
   log để phân tích; không xóa pending hoặc bypass guard.

Các bước này cần người dùng giám sát phần cứng, chưa được agent thực hiện.
