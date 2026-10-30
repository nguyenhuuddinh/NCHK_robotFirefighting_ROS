# QA mở rộng các nhánh kẹt và hành vi X — 30/10/2026

## Kết luận

Ngoài các lỗi Goal đã ghi trong `QA_GOAL_RETRY_2026-10-30.md`, đã tìm thấy
**hai khoảng trống khác ở ranh giới chu trình lửa**. X hiện chưa đảm bảo kết
thúc chu trình và cho dùng Goal mới ở mọi thời điểm. Không đánh dấu nghiệm
thu chỉ vì lệnh X đã được nhận hoặc có nhiều bài kiểm thử đạt.

Lượt này **chỉ thêm QA và tài liệu**, không sửa runtime, build/sync gói mới,
khởi động lại app người dùng, kết nối Pi, NTP, chạy xe/bơm hay sửa R5.
Packet đang chọn vẫn `f63e38d71f5e767960bf84ef151cfdaf72b680f329bf3b080dcf27828dca38bd`.

## Findings ưu tiên sửa

### P1 — X/target lost sớm tạo vòng chờ hủy Goal chưa từng được gửi

Files trong `releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission/fire_robot_mission/`:

- `operational_handback.py:73–98`: khi cancel_requested hoặc mất target tạo
  yêu cầu kết thúc, coordinator chuyển PREPARE ngay sau đăng ký ACTIVE.
- `cycle_release.py:87–120`: PREPARE yêu cầu không còn Goal active và có
  bằng chứng hủy đúng Goal gốc.
- `nav2_cancel_guard.py:449–455`: đường hủy khi nhận lửa chỉ được gửi khi
  intent còn hợp lệ và reason=FIRE_ACQUIRED.
- `cycle_blocked_goal.py:24–38`: đường hủy Goal khác không xử lý Goal gốc
  của chu trình; vẫn cần lịch sử hủy gốc cho các Goal khác.

Nếu target mất trước khi guard gửi CancelGoal, coordinator chờ bằng chứng
hủy nhưng guard không còn intent nhận lửa hợp lệ. **Đã xác nhận trực tiếp**:
guard `WAITING_FOR_ADMISSION`, goal_uuid=null, send_count=0,
`CANCEL_INTENT_CONTRACT_INVALID`; raw/pump owner đều đã nhả, operator không
fault, chu trình vẫn `CANCEL_WAIT_CLEAR`, các peer PREPARING. Bấm X lặp
không tạo được lịch sử hủy gốc còn thiếu.

Tái hiện 16 tổ hợp: có/không cho navigation chạy ổn trước khi xuất hiện lửa,
X hoặc tự target-lost, mất target trước/sát/sau 1 hoặc 3 tick ở thời điểm
chu trình đăng ký. Tất cả không thu dọn được trong cửa sổ kiểm 16 giây sau
thao tác. Test ma trận rộng cũng tái hiện khi X hai lần. Q cuối mỗi ca lỗi
đều lấy STOP confirmed trong mô phỏng. Đây là lỗi phục hồi chu trình, không
phải bằng chứng xe/bơm đang chạy không dừng.

Evidence: `early_True_0_X.json`, `early_guard_trace.log`, 16 file `early_*.json`,
`x_final.xml`. **Không sửa guard bằng cách giả CANCEL_CONFIRMED hoặc bỏ proof**.
Cần luồng hủy theo ý định operator/retirement, đúng UUID/session/identity/
source và STOP, độc lập việc target còn nhìn thấy hay không.

### P1 — X ở ranh giới phát hiện lửa chưa ngăn tái nhập tự động

- `operational_session_core.py:589–594`: nếu tick đó chưa có cycle, X chỉ
  yêu cầu hủy active Nav2 UUID, hoặc trả `NO_ACTIVE_NAV_GOAL`. Không đặt cơ
  chế hủy/suppression cho FIRE_WATCH ở nhánh này.
- `automatic_policy.py:124–127`: khi không còn cycle có thể bật FIRE_WATCH.
- `operational_session_core.py:747–751`: khi lửa mới đến và còn quyền fire,
  một cycle mới có thể được tạo.

Đã tái hiện hai hình thức theo thời điểm:

1. Đang ARMED_IDLE/no-goal, lửa xuất hiện sát X một lần: sau X vẫn tạo cycle
   mới và xoay tự động trong mô phỏng. Test ghi `accepted_motion_after_x=true`.
2. Goal vừa bắt đầu, X hai lần trùng chuyển trạng thái lửa: Goal được hủy
   nhưng sau đó có cycle no-goal mới, phase ALIGNING; vì vậy không mở lại
   được công cụ Goal như người dùng mong đợi.

Sáu ca điều khiển riêng (có/không Goal, X 1/2/6 lần) cho thấy lỗi phụ thuộc
lịch tick; bấm lặp nhiều hơn trong một lịch chạy không chứng minh đã sửa
race. Trường hợp Goal đã chạy ổn trước đó vượt qua ở lịch này. Q xác nhận
dừng ở tất cả các ca mô phỏng. Chưa thử chuyển động phần cứng.

Evidence: `fire_boundary_False_1.json`, `idle_True_True.json`.
Cần quy định X là hủy chu trình/ý định hiện tại, thu dọn xong rồi cho Goal
mới; không để automatic tự làm mất hiệu lực yêu cầu hủy ngay ở tick kế tiếp.
Không được tự bỏ điều kiện OFF/HOME/STOP để mở Goal sớm.

### P1 — Các lỗi Goal/UUID trước vẫn còn, chưa được sửa

Lỗi input trước terminal → Goal khác/preemption → X chỉ hủy UUID mới;
và Goal đã terminal trước X → X không có active target. Gateway còn giữ
latch gắn với UUID cũ nhưng UI vẫn cho Goal. Các bài kiểm từ lượt trước
được chạy lại; acceptance sau preemption vẫn XFAIL đúng lỗi đã biết.

X trong chu trình đã thực sự bắt đầu, X trong navigation-only và X khi chưa
có chu trình là các nhánh code khác nhau. Đây là lý do không thể dùng một
bài kiểm X thành công để kết luận mọi X đều phục hồi.

### Nhánh chờ có chủ đích — transport hủy Goal chưa xác nhận

`cycle_endpoint.py:441–460` lưu một lần gửi theo UUID. Nếu send exception,
response reject hoặc mất response, lặp X không gửi lại mù; operator còn
`NAV_CANCEL_WAIT` với lý do tương ứng. Khi dịch vụ hủy chỉ tạm vắng rồi trở
lại trước khi gửi, thao tác tiếp tục và Goal mới chạy được.

4 bài kiểm actual node xác nhận các hành vi trên. Ba ca không xác nhận
giữ Goal tool khóa; Q vẫn STOP confirmed trong mô phỏng. Đây là chờ an toàn
do kết quả action chưa xác định, **chưa kết luận cần bỏ chống gửi lặp**.
App cần hướng dẫn rõ Q → thu dọn/kiểm tra → phiên mới, hoặc thêm recovery
có proof và chính sách retry rõ ràng. X hiện chưa phải lối thoát cho chúng.

## Các nhánh đã kiểm và kết quả

| Nhóm | Phạm vi và kết quả |
| --- | --- |
| Khởi động quét map | Timeout I/O, seal, SLAM; drain laptop trước Pi, không tự chạy lại khi dữ liệu đến muộn. QA hiện có đạt. |
| Khởi động chữa cháy | PI_IO, CAMERA, SEAL_PI, POSE, YOLO, CONSUMERS; quá hạn đi STOP, không hồi sinh bằng callback muộn. QA hiện có đạt. |
| Chọn/lưu map | Tên sai/trùng, map đổi, lưu lặp, mất ownership, stop khi đang lưu, rollback file, callback cũ. QA hiện có đạt; không sửa tốc độ/quét map. |
| Bấm nút theo thứ tự khác nhau | 320 cặp thao tác SCAN/FIRE/SAVE/SELECT/X/STOP/RECHECK/VERIFY_STOP ở 5 trạng thái. Worker giả; STOP với proof sạch về IDLE, lần scan mới không bị callback cũ khóa. Không đại diện cho việc X thật đã hủy chu trình. |
| X trong chu trình đang hoạt động | X lặp khi xoay, tiến IR, xác nhận range, endpoint bơm vừa tạo, đang ON, đang HOME, sau pulse; có/không Goal gốc và lửa còn/mất. Các giai đoạn đã hoạt động đạt, 2 ca lỗi ở ranh giới bắt đầu nêu trên. |
| X khi dữ liệu trễ | 20 tổ hợp: navigation/xoay/IR/pump × vision/identity/odom/actuator/IR mất tạm ~1,2 s. Dữ liệu cùng boot hồi lại → retire → Goal mới có raw velocity; không ON mới, không hai raw owner. Đạt toàn bộ. |
| X khi dịch vụ CancelGoal lỗi | 4 ca: service down hồi được; send failed/rejected/response lost giữ chờ, Q thoát có STOP proof. |
| Q trong lỗi/gián đoạn | Bộ 48 ca cũ gồm Q/XQ/QXQ/lặp, thứ tự callback và STOP telemetry fresh/late/absent. Late có thể xác nhận sau khi hồi; absent phải STOP_UNCONFIRMED. Đạt trong QA. |
| BLOCKED và phục hồi | Receipt/ownership/boot/PID/group/lock, quiescence chỉ đọc, thiếu cleanup Pi, fresh STOP, audit/archive, dữ liệu cũ/sai phiên và lỗi ghi/xóa marker. QA hiện có đạt theo đúng kỳ vọng từng ca; không đồng nghĩa mọi BLOCKED đều phải tự mở. |
| Giao diện thật | Qt/RViz/IPC, chọn map, Esc thoát công cụ, Q dừng, mất frontend/child và view bootstrap. Kiểm localhost, không robot. Các lỗi thiết lập QA được chạy lại đúng môi trường và đạt. |

## BLOCKED nào vẫn phải giữ

Không có STOP/OFF/HOME hiện thời; identity/epoch đổi; process hoặc group còn
sống; có publisher điều khiển không rõ ownership; hồ sơ phiên không khớp;
proof stale/sai nonce/sai scope; lock bận; không liên lạc Pi; lỗi lưu bằng
chứng đều không đủ cơ sở mở phiên mới. X là hủy nhiệm vụ, không phải quyền
bỏ qua các bằng chứng này. Khi dữ liệu phục hồi, app cần cho biết đang đợi
proof nào và đưa đường phục hồi được hỗ trợ.

RECHECK kiểm lại bằng chứng cũ và tiến trình, không phải kill-all. VERIFY_STOP
chỉ được mở I/O/STOP mới sau kiểm ownership/quiescence, rồi phải audit STOP
và cleanup mới. Phiên cũ không đủ hồ sơ ownership vẫn cần xử lý riêng;
không xóa pending để giả IDLE. Các trường hợp mất kết nối thật, treo kernel,
nguồn actuator, firmware và WiFi thực tế chưa được nghiệm thu ở lượt này.

## Số liệu và giới hạn

Evidence: `qa_evidence/block_branches_audit_20261030/summary.json`.

- Hợp nhất theo **test ID duy nhất**, dùng kết quả chạy lại sau cùng:
  **725 test: 703 PASS, 19 FAIL, 1 XFAIL lỗi Goal đã biết, 2 SKIP**.
- 91 test mới: **70 PASS, 19 FAIL, 2 SKIP**. 19 FAIL gồm 17 cách tái hiện
  deadlock hủy sớm và 2 cách tái nhập lửa sau X; không phải 19 root cause.
- 2 SKIP là navigation active khi fixture khởi đầu không có Goal, không phải
  bỏ qua lỗi. 5 test controller có tổng 320 cặp thao tác, không đếm chúng
  thành 320 pytest khác nhau.
- 306 unit test hiện có đạt. Các nhóm integration hiện có và 39 ca Goal
  audit đã chạy lại. PASS trong bài characterization có thể là xác nhận
  **app vẫn giữ chờ** đúng thiết kế, không khẳng định không còn bug.
- Log thô lượt đầu giữ nguyên: 29 lỗi Unix socket do sandbox; chạy lại với
  socket cục bộ được phép thì 306 PASS. Broad integration đầu có thiếu biến
  thư mục evidence cho 21 ca Esc và 4 ca health đã có output cũ; chạy lại
  bằng thư mục mới. Lần chuẩn bị thư mục cha health thiếu cũng được giữ log,
  sau tạo đúng thư mục thì cả 4 PASS. Không sửa runtime để làm test đạt.
- `integrity_before.json`/`integrity_after.json`: **437 file runtime/R5/
  current được bảo vệ, không file nào đổi**. R5 364 file và bản sao trong
  artifact vẫn khớp manifest. Không commit/push hoặc thay pending thật.

Không thể chứng minh mọi tổ hợp thao tác, timing DDS và lỗi phần cứng bằng
một ma trận hữu hạn. Điểm mới của đợt này là kiểm **X → STOP/OFF/HOME thật
trong mô phỏng → retire → Goal mới có raw command**, thay cho chỉ kiểm X
được nhận. Giữ các test FAIL để làm tiêu chí cho bản sửa kế tiếp.

## Thứ tự sửa đề xuất

1. Chốt và thực thi semantics X thống nhất: latch hủy ngay, ngăn tái nhập
   automatic ngoài ý muốn, kết thúc task hiện tại trước khi cho task khác.
2. Hủy Goal gốc có kiểm chứng khi target đã mất hoặc cycle đã PREPARE; tránh
   coordinator chờ guard và guard chờ một intent không thể hợp lệ nữa.
3. Phục hồi theo cả UUID gateway đang giữ và UUID Nav2 hiện tại; xử lý Goal
   đến sớm, terminal trước X, preemption và lịch sử status bị lược bỏ.
4. UI phản ánh đang hủy/chờ OFF-HOME/chờ proof/cần phục hồi, mở Goal theo
   năng lực gateway thực; báo đường Q/verify cho action chưa xác nhận.
5. Sửa chỉ lớp app/candidate, giữ R5, watchdog, mapping và deadline an toàn.
   Chạy các acceptance FAIL trước, regression sau, rồi mới test có giám sát
   trên robot. Chưa có bản sửa được đóng gói hoặc đồng bộ trong lượt QA này.
