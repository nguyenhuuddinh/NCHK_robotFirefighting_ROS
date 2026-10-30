# QA lỗi đặt lại Goal — 30/10/2026

Phạm vi: đọc log phiên mới nhất và tái hiện trên bus mô phỏng bằng node R5
thật + operator được sinh cho app. **Chưa sửa runtime, chưa build/sync/chọn
gói mới, không SSH, ROS init/DDS hoặc chạy phần cứng.** Chỉ thêm test và bằng
chứng QA. Không xác nhận robot đã được nghiệm thu mọi nhánh.

## Kết luận từ phiên thật

Phiên laptop:
`/home/huudinh/.local/state/fire_robot_control/session_efb30da3a6764becbdb8025a701ba7bf`.
Session `de56a358-935e-4853-b8e8-f6e43cb001f6`, packet
`f63e38d71f5e767960bf84ef151cfdaf72b680f329bf3b080dcf27828dca38bd`.

Đây là **khóa đường chuyển lệnh của nhiệm vụ điều hướng trong phiên**.
Nav2 vẫn nhận các Goal mới. Không có bằng chứng cả phiên đã chuyển sang
BLOCKED trong lượt này: Q cuối phiên có STOP confirmed, cleanup clean,
operator exit 0, runtime fault null; hiện không có pending.json.

Giờ Việt Nam, các dòng dưới được trích từ raw log và ghi lại trong
`qa_evidence/goal_retry_audit_20261030/timeline.json`:

| Thời gian | Bằng chứng |
| --- | --- |
| 18:27:23 → 18:27:28 | Goal 1 chạy và SUCCEEDED; gateway nhả raw, sau đó báo NAV2_READY_NEW_GOAL. |
| 18:27:31 → 18:27:52 | Goal 2 chạy và SUCCEEDED; tiếp tục có READY_NEW_GOAL. |
| 18:27:55 | Goal 3 bắt đầu, đích (-0.19, -0.11). |
| 18:28:09 trở đi | Planner nhiều lần không tạo được đường tới đích này. Chưa đủ dữ liệu để kết luận do bản đồ, costmap hay vị trí chọn. |
| 18:28:47.855 → 18:28:50.055 | Spin recovery kết thúc, Nav2 thực hiện wait từ 18:28:48.055. |
| 18:28:48.716 | Gateway nhả raw publisher sau STOP khi Goal 3 còn trong luồng recovery. |
| 18:29:18.644 | Nav2 nhận preemption, bắt đầu Goal 4 ở (1.30, -1.51). |
| 18:29:32.555 → .610 | Operator hủy Goal đang hoạt động, trở về ARMED_IDLE/WAIT_GOAL_OR_F. Gateway vẫn RELEASED_STOPPED/NAV_INPUT_OR_TOPOLOGY_LOST, raw_count=0. |
| 18:29:50.054; 18:30:06.779 | Nav2 vẫn nhận thêm Goal 5, 6; gateway vẫn giữ lý do cũ. Các Goal sau cũng được hủy. |
| 18:30:22.033 | Q: FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED. Receipt cuối xác nhận cleanup sạch. |

Gateway release trùng giai đoạn Nav2 wait; điều này phù hợp giả thuyết dòng
velocity tạm ngừng khi recovery. **Chưa chứng minh được blocker đầu tiên
chính xác**: mission.log chỉ ghi release chung, operator.log chỉ ghi khi
phase/reason đổi. Snapshot sau có blockers=[] không chứng minh thời điểm
STOP không có blocker. Không quy lỗi cho ESP32 hoặc YOLO từ các log này.

## Findings

### P1 — X không khôi phục khi Goal bị khóa khác Goal đang hủy

`releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission/fire_robot_mission/cycle_endpoint.py:279–288`:
endpoint ghi nhận navigation_cancel_uuid của operator vào nav_cancel_targets.
Chỉ khi **goal_uuid mà gateway đang giữ** thuộc tập đó, có terminal 4/5/6,
đúng session/identity/graph, đã đứng yên và OFF, nó mới chuyển lỗi transient
thành đường normal rearm. Đây là guard có kiểm chứng, không được xóa mù.

`.../operational_session_core.py:589–594`: X chỉ chọn UUID đang active.
Sau preemption, đó là Goal mới. Goal cũ của gateway không được ghi nhận là
đã yêu cầu hủy. Sau lỗi dữ liệu, `_normal_cycle_rearm` ở
`.../live_nav_stop_gateway.py:305` không nhận nếu normal_terminal còn null.
Goal mới vì vậy được Nav2 nhận nhưng gateway tiếp tục nhả quyền xuất lệnh.

Đã tái hiện trên **actual app operator + các node thật qua bus mô phỏng**:

- Lỗi input rồi preemption rồi X: Goal mới bị hủy đúng UUID, Goal cũ vẫn
  giữ latch; Goal kế tiếp không có raw publisher, không chuyển động.
- Preemption trực tiếp lúc đang chạy cũng tạo nhánh tương tự, không cần
  giả lập input gap trước đó.
- Cả terminal cũ SUCCEEDED/CANCELED/ABORTED đều được thử; có/không giữ Goal
  cũ trong status list đều tái hiện được (12 tổ hợp).
- Q sau các nhánh này vẫn xác nhận STOP trong mô phỏng; không có pump ON,
  không có hai raw owner đồng thời.

Luồng này khớp thứ tự log thật: gateway lỗi/nhả quyền trước, rồi Nav2
preemption, rồi X. Raw log không lưu UUID bị gateway giữ lúc latch, nên
không tuyên bố đã thu được đầy đủ toàn bộ chứng cứ UUID tại thời điểm đó.

### P1 — Goal lỗi tự kết thúc trước X cũng không có mục tiêu phục hồi

Cùng `operational_session_core.py:589–594`: nếu Goal đã terminal và không còn
active goal, X trả accepted/NO_ACTIVE_NAV_GOAL, navigation_cancel_uuid vẫn
null. Gateway đã STOP bởi lỗi input trước đó không có normal_terminal và
endpoint không được bổ sung target cũ. Đặt Goal mới vẫn không chạy.

Đã tái hiện trên node thật với terminal 4, 5, 6 sau nav gap. Đây là nhánh khác
của cùng khoảng trống phục hồi; không phải mọi input gap đều kẹt:
**nav gap → X khi đúng Goal cũ còn active → chờ READY → Goal mới chạy** đã đạt.

### P1 — UI cho đặt Goal dù đường xuất lệnh chưa dùng lại được

`apps/fire_robot_control/native_rviz/main.cpp:479–489` cho Goal khi armed,
không fault/hold/cycle, phase ARMED_IDLE hoặc NAVIGATION. Predicate không
phản ánh trạng thái lỗi/UUID/khả năng nhận lại quyền của gateway.
`.../operational_session_core.py:754–758` cũng đặt phase NAVIGATION khi thấy
active Goal, dù gateway vẫn RELEASED_STOPPED bởi lỗi cũ.

Đã đối chiếu predicate UI hiện tại với report của actual app node ở các ca
trên: vẫn đủ điều kiện mở công cụ. Đây là đối chiếu source/predicate và
report thật của mô phỏng, không phải lần click RViz trên robot thật.

Thêm nhánh: sau terminal sạch, nếu Goal mới tới trước khi hết khoảng xác
nhận idle liên tục, không tích lũy được proof idle vì đã có active Goal.
Ba terminal 4/5/6 đều tái hiện trên node thật. **Nhánh này có thể cứu bằng X
hủy Goal mới → đợi NAV2_READY_NEW_GOAL → đặt lại**, khác hai nhánh khóa trên.
UI hiện chưa phân biệt và chưa hướng dẫn rõ.

### P2 — Thiếu bản ghi lúc latch để xác định blocker đầu tiên

`.../live_nav_stop_gateway.py:896` chỉ ghi raw release chung. Bản tóm tắt
operator không lưu đầy đủ gateway UUID, normal_terminal, cancel targets và
blocker tại lần đổi STOP đầu tiên. Vì thế không được suy diễn rằng tăng
timeout hoặc đổi model sẽ chữa được lỗi này. Cần snapshot có giới hạn khi
latch/release/rearm thay đổi, không log dày mọi tick.

## Vì sao các lượt QA trước chưa bắt được

- `tests/qa_full_operator.py:125–147` kiểm socket X tới cancel và phiên còn
  mở; không kiểm tiếp Goal mới thực sự được gateway chuyển lệnh.
- Bộ lifecycle R5 có kiểm X → Goal mới nhưng fixture hủy đúng Goal đang
  active, có luồng velocity đều và chờ READY trước Goal tiếp theo. Đó là
  đường lành đã đạt, không đại diện chuỗi lỗi/preemption/late-X này.
- QA recovery BLOCKED, receipt và STOP khi Q kiểm một lớp khác với quyền
  chuyển lệnh Nav2 trong phiên còn hoạt động. Số lượng test trước không
  chứng minh đã phủ mọi tổ hợp thao tác.
- Giả thuyết ban đầu "input gap rồi X luôn kẹt" đã bị bác bỏ bằng actual
  node test. Bare core không tự rearm sau fault là chủ ý; phải kiểm thêm
  cycle endpoint. Không sửa code từ giả thuyết đã bị bác bỏ.

## QA mới và cách đọc kết quả

`tests/qa_goal_retry_audit.py`: 39 test, **38 passed, 1 XFAIL**. XFAIL là yêu
cầu Goal mới chạy sau lỗi/preemption/X, đang thất bại đúng lỗi cần sửa.
Nhiều test passed xác nhận **tái hiện được hành vi lỗi hiện tại và giữ dừng
an toàn**, không có nghĩa app đã hết bug hoặc đã qua acceptance.

- Control sạch: SUCCEEDED/CANCELED/ABORTED → chờ clear hold → Goal mới chạy.
- 11 lỗi bare core: nav/odom/arm/mission gap, arm false, mission health,
  nav/safety topology, status source, UUID preemption, raw conflict. Đều giữ
  zero, nhả owner; đây là kiểm an toàn, không yêu cầu tự bỏ các lỗi này.
- 12 tổ hợp node lỗi/preemption/X; terminal trước X; Goal mới quá sớm;
  healthy X và nav gap/X đúng UUID; Q sau lỗi và không pump ON.
- Kiểm code operator sinh từ source hiện tại đúng byte với generated
  operator của artifact đang chọn. Mô phỏng không chứng minh timing DDS,
  RF/WiFi, chuyển động, costmap hoặc firmware trên robot.

Lệnh tái hiện đầy đủ lưu ở
`qa_evidence/goal_retry_audit_20261030/run_qa.sh`; log pytest, JUnit, raw log
SHA và timeline được lưu cùng thư mục. Dùng overlay Humble cài sẵn chỉ để
import dependency; core/fixture được nạp rõ từ release R5 hiện hành. Không
chạy bản ROS node của overlay hoặc khởi tạo ROS context.

364 file R5 và bản sao của chúng trong gói app đều khớp manifest;
runtime app không đổi trong QA. Model, mapping/teleop, watchdog, topic/QoS,
thời hạn STOP, Esc/Q, artifact current và pending không được sửa.

## Hướng sửa đề xuất, chưa triển khai

1. App cần trạng thái điều hướng chung với gateway: đang chạy, chờ STOP,
   chờ sẵn sàng, cần phục hồi; không coi một active Goal là có quyền xuất lệnh.
2. Khi thay Goal/hủy sau lỗi, theo dõi cả UUID gateway đang giữ và UUID
   Nav2 hiện tại; chỉ phục hồi bằng terminal/STOP/source/identity/ownership
   proof đúng phiên. Nếu bằng chứng thiếu/evicted phải báo rõ và dùng luồng
   dừng phiên có kiểm chứng; không giả terminal hoặc xóa latch trực tiếp.
3. Chặn hoặc xếp lại thao tác Goal đến trước READY theo hành vi được định
   nghĩa rõ; báo kết quả X và lý do chưa nhận Goal mới ngay trên UI.
4. Snapshot transition cho first blocker/UUID và bổ sung regression end to
   end: lỗi → preemption → X → Goal kế tiếp thực sự được chuyển lệnh.
5. Thực hiện ở lớp app/candidate riêng; giữ release/lệnh R5 và quét map
   nguyên trạng. Chỉ test robot sau QA có kết quả, không bỏ guard an toàn để
   làm test chạy được.
