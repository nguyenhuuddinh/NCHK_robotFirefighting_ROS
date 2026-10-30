> **Cập nhật 30/10/2026:** Đây là kế hoạch lịch sử. Phiên 1 đã triển khai và
> QA offline; xem [báo cáo thực tế](SESSION1_APP_ENTRY_QA_2026-10-30.md) và
> [registry QA](SESSION1_QA_REGISTRY_2026-10-30.md). Chưa chọn artifact mới,
> chưa sync Pi; các ca PARTIAL/NOT_RUN vẫn còn.

# Kế hoạch entrypoint riêng cho app, bảo toàn R5

Ngày: 29/10/2026. Trạng thái: **PLAN_ONLY — chưa sửa code vận hành, chưa build/sync/chạy robot**.

Đây là kế hoạch tiếp nối sau hai lượt app lỗi SEAL_PI và bộ đo TF riêng,
không phải kế hoạch xây lại giao diện từ đầu. Mục tiêu tổ chức: **1 phiên code
+ QA offline, 1 phiên tích hợp Pi + nghiệm thu có giám sát**. Checkpoint trong
một phiên không phải những lần bàn giao hoặc hỏi có tiếp tục hay không.

## 1. Kết quả cần đạt

1. Giữ các lệnh R5 terminal đang có, đường dẫn, tham số, môi trường và source
   nguyên byte; không bắt người dùng đổi sang app.
2. Có một lệnh riêng mở app ở IDLE. Nhấn Quét map/Khởi động chữa cháy mới mở
   phiên tương ứng; tự lấy pairing, không cần copy đường dẫn từ Terminal1.
3. App dùng lại launch/profile, các phép kiểm readiness, operator và chuỗi
   STOP của R5; phần riêng chỉ điều phối vòng đời và giao diện.
4. Hiện rõ đang chờ gì, thời gian còn lại, lỗi gốc, xác nhận STOP và kết quả
   cleanup. Không dùng một nhãn BLOCKED để thay cho mọi loại lỗi.
5. Lỗi khởi động đã STOP và thu dọn sạch phải cho thử lại bằng thao tác rõ ràng.
   Chưa có bằng chứng dừng hoặc còn tiến trình thì tiếp tục khóa có lý do.
6. Giữ RViz thật + khung camera lớn, màu trắng/đỏ/xám; chọn map cũ hoặc vừa lưu.
   Quét map vẫn lái bằng terminal/khung bàn phím hiện có, không tự khám phá phòng.

**“Tránh BLOCKED” nghĩa là tránh khóa oan và có đường phục hồi kiểm chứng được.**
Không hứa loại bỏ mọi lỗi phần cứng/WiFi hoặc mở khóa khi STOP chưa xác nhận.

## 2. Cơ sở đã kiểm tra và phần chưa được chứng minh

| Mục | Hiện trạng source/log |
| --- | --- |
| R5 chuẩn local | Release và candidate cùng packet `0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0`, 364/364 file |
| App đang chọn | `6734c2846c465773d1a4e6c96b9ca3f0ed193d06c07cc013bc0353c1f7b2ae83`, 416 file; không đổi trong lượt lập kế hoạch |
| R5 terminal 21:17:14 | Đã qua startup/READY_GATE2; cleanup cuối phiên còn lỗi riêng, không coi toàn R5 đã PASS |
| App chữa cháy 21:59:26 | STARTUP_TIMEOUT:SEAL_PI; STOP=true, nhóm local sạch, remote confirmed=true; runner rc=2 vì startup |
| App quét 22:01:56 | PI_SEAL_HEALTH_TIMEOUT; Pi thu dọn sớm làm thiếu nguồn STOP; STOP=false, khóa là đúng |
| Retry hiện tại | Audit chỉ đọc; có thể về IDLE khi đủ proof. Chưa tự khôi phục readiness hoặc tạo proof STOP mới |
| TF | Positive control delay25ms tái hiện observer trễ4,8s; fixture Pi nhẹ không tái hiện. Chưa chốt nguyên nhân phiên robot thật |

Các khác biệt cần xử lý nằm ở lớp tích hợp:

- `services/candidate_build.py:15`: sinh bản runner đã chèn hooks, hiện ghi vào
  tên `scripts/r5_laptop_session.py` của **artifact app**, không phải R5 gốc.
- `native_rviz/main.cpp:561`: mở ROS/Observer/RViz trước khi user chạy phiên;
  R5 mở RViz sau seal. Tắt display chưa chắc loại được TF listener.
- `services/mapping_runner.py:160`: poll STATE bằng SSH +0,25s; R5 chữa cháy
  dùng SSH +2s. Không suy ra nhịp thực4Hz.
- `services/live_session.py:57`: cleanup hiện phụ thuộc cả runner rc=0;
  `run()` còn thay lỗi gốc bằng PI_RUNNER_EXITED_DURING_SESSION.
- `scripts/r5_pi_session.py:51` của R5: seal hết20s thì tự thu dọn I/O;
  lớp app chưa phối hợp đủ cửa sổ xác minh STOP khi khởi động lỗi.
- `services/recovery.py:32`: phiên cũ thiếu STOP bị từ chối đúng; không thể
  sửa hiện tượng đó bằng cách xóa pending hoặc chỉ cắm lại ESP32.

Nguồn: [QA hai lượt lỗi](QA_STARTUP_BLOCK_215926_220156_2026-10-29.md),
[đối chiếu R5/app](QA_R5_APP_PARITY_2026-10-29.md),
[phép đo TF riêng](TF_ISOLATED_DIAGNOSTIC_2026-10-29.md).

Tài liệu ROS lịch sử còn nói micro-ROS, camera comment, Reliable /cmd_vel và
watchdog500ms; R5 đã có profile raw serial/camera/QoS riêng. Kế hoạch này lấy
code R5 đã pin làm chuẩn so sánh, không tự sửa các mismatch lịch sử hoặc đổi
topic/frame/QoS/watchdog để chạy qua QA.

## 3. Ranh giới bảo toàn R5

**Không sửa/ghi đè:**

- `releases/r5_20261029/` và `.codex_qa_backups/r5_operational_candidate_20261026/`;
  cả source, manifest, các script wrapper `evidence_r5_5_model_29_10_2026/run_pi.sh`
  và `run_laptop.sh`, install/build đang dùng.
- Stage Pi `/home/pi/r5_staging/0c54d9b79765/ros`, baseline Pi/laptop, launchers
  R5 cũ, model và firmware. Không pull/build đè workspace R5.
- Source node tại `ros2_ws/src/`, ai/, cấu hình mạng/AP/domain sản xuất, GPIO,
  serial/baud, STOP/watchdog, ngưỡng TF300ms/lead100ms và các gate actuator.
- Map cũ, log/receipt cũ, pending thật chưa được kiểm chứng; output/ và tmp/
  đang có của người dùng.

Thay đổi chỉ trong `apps/fire_robot_control/`. Artifact mới là thư mục mới
theo hash. Pi dùng stage app riêng `/home/pi/fire_robot_desktop/<packet>/ros`.
Đồng bộ app không chạy lệnh robot, không sửa stage R5 hoặc tự flash.

Trước/sau từng phiên: hash đủ364 file R5 release + candidate, manifest, hai
wrapper; khi có LAN thì hash364 file stage R5 Pi. Ghi cả đường dẫn module/
install thực được resolve. Hash source không tự chứng minh runtime không bị
ảnh hưởng: phải kiểm lock, argv/env allowlist, installed sources và hồi quy CLI.

App và R5 phải dùng cơ chế common lock hiện có để **từ chối chạy chồng**.
App chỉ mở IDLE không chiếm robot, không gửi tín hiệu/STOP tới R5 đang chạy.

## 4. Kiến trúc chốt cho phần tiếp theo

### 4.1. Hai điểm khởi động độc lập

- **R5 terminal:** người dùng tiếp tục dùng hai wrapper trong runbook
  [R5 map910/TN1](../../../docs/codex/current/R5_5_MODEL_29102026_MAP910_SYNC_AND_RUN_2026-10-29.md).
- **App:** thêm `launchers/run_app.sh` làm lệnh mở giao diện, dùng launcher/
  desktop entry hiện có. Lệnh không có chế độ mặc định tự chạy robot.
  `start_candidate.sh` vẫn có thể dùng để chọn một artifact đã xác minh.

Không tạo hai bộ mission/điều khiển. Dùng lại helper R5 nguyên byte; runner
riêng của app điều phối các helper qua adapter có kiểm hash và contract.

### 4.2. Tách tên và nguồn của runner app

Giữ pattern sinh adapter có kiểm anchor đang dùng, nhưng đầu ra mang tên app
dưới `desktop/generated/`, không ghi bản tích hợp vào tên runner R5.
Mọi thay đổi sinh mã phải có diff/allowlist và test anchor chỉ xuất hiện1lần.
Không chép một bản vòng lặp R5 dài để duy trì thủ công song song.

- `app_fire_entry.py`: adapter sinh từ runner laptop R5 đã pin; thêm FD/session,
  external RViz/operator, progress và kết quả app.
- `app_pi_entry.py`: adapter Pi riêng của app, dùng lại Pi argv, preflight,
  pairing/ownership và helper R5; thêm phối hợp lỗi **trước khi cho chạy**.
- `services/packet_archive.py`: công cụ verify/export packet app, giữ các
  kiểm path/symlink/hash/no-overwrite hiện có. Thay các chỗ gọi công cụ packet
  R5 trên manifest app, kể cả script sync Pi. Không sửa `scripts/r5_packet.py`.

Mục tiêu artifact mới: toàn364 source R5 gốc đều giữ byte; app có manifest
riêng mở rộng, parent packet0c54 và diff các adapter phát sinh. Không dùng
công cụ R5 vốn chỉ nhận scripts/ros2_ws/src để xác minh archive có desktop/.
Đây không phải thao tác đổi manifest của release R5.

### 4.3. UI có sẵn, ROS view chỉ gắn khi đúng phase

Mở Qt shell/camera placeholder/status trước. Ở IDLE/PREFLIGHT/PAIRING/SEAL:
không khởi tạo RViz transformation manager/Observer sản xuất trong GUI.
Sau seal mới gắn RViz/camera vào cùng cửa sổ; khi dừng phải tháo context/
reader thật và xóa dữ liệu phiên cũ. Không chỉ đặt display Enabled=false.

Backend vẫn có health observer theo R5; STOP verifier tối thiểu được chuẩn
bị trước khi Pi START. Đây là reader phục vụ an toàn riêng, phải đo graph/
tải và không được nhân đôi operator/owner. Khi chuyển sang operator chính,
STOP verifier cũ phải kết thúc/reap và nhả quyền trước khi operator mới hoạt
động; không có STOP/receipt tới muộn của generation cũ tác động phiên mới.

Thực hiện bằng refactor nhỏ host RViz hiện có sang shell chứa view có vòng
đời riêng; không đổi framework, không ghép cửa sổ ngoài bằng window ID.
Effect START_RVIZ phải có ACK view thật cùng session/generation và timeout;
display_guard hoặc status đến muộn không được giả làm ACK. STOP trước ACK
phải hủy attach, không để view/tools của phiên đã dừng xuất hiện trở lại.

### 4.4. Dữ liệu trạng thái

Supervisor là nơi duy nhất thay state. Worker chỉ gửi event có session/
generation; không quyết định mở khóa từ widget hoặc chuỗi log tự do.

Kết quả phiên app phải tách:

- `startup_result / primary_error / stage / blockers`;
- `robot_stop_confirmed`;
- `local_groups_exited / remote_groups_exited / children_clean / forced`;
- `cleanup_clean / recovery_allowed`;
- binding packet/session/boot, thời điểm và đường dẫn log đã loại secret.

Runner rc=2 vẫn giữ nguyên và báo phiên thất bại. Nó không tự chứng minh
cleanup bẩn; ngược lại process exit0 không chứng minh robot đã dừng. Adapter
đánh giá các proof riêng, không sửa DONE/receipt gốc để biến FAIL thành PASS.
Remote error của startup được giữ ở primary_error; chỉ khi receipt/group/
child audit thực sự đạt mới xác nhận cleanup, kể cả R5 peer.confirmed=false
chỉ do còn startup error. Trường hợp force/child crash/thiếu audit vẫn FAIL.

Log/progress qua file atomic hoặc IPC có schema/binding và giới hạn kích
thước. Ghi env allowlist; không ghi raw SSH/password/pairing secret.

Giữ phase chính đang có (IDLE/STARTING/MAPPING/RUNNING/STOPPING/BLOCKED/
CHECKING), thêm fault_kind/recovery_allowed để phân loại lỗi. Không tạo một
FSM giao diện thứ hai. Bổ sung VERIFY_STOP vào allowlist protocol có cùng
lease/nonce/sequence, không thêm kênh bỏ qua xác thực. Receipt app phiên bản
mới phải có reader cho dữ liệu cũ; không đổi schema receipt R5 tại nguồn.

| Phase chính | Thao tác được phép |
| --- | --- |
| IDLE | Chọn map, Quét map; Fire khi map hợp lệ; đóng app |
| STARTING | Xem tiến độ, nhập auth khi được yêu cầu, STOP/close; Pose chỉ tại bước POSE sau ACK view |
| MAPPING | Teleop, Save, chọn map khi không đang Save, STOP/close; chuyển Fire qua dừng sạch |
| RUNNING | X/Goal theo quyền operator R5, STOP/close; không tạo Scan/Fire khác |
| STOPPING | Xem kết quả, nhấn STOP/close lặp là idempotent; không kéo dài deadline |
| BLOCKED | Chi tiết lỗi, RECHECK nếu không còn worker; VERIFY_STOP chủ động nếu điều kiện phục hồi cho phép |
| CHECKING | Xem audit/attempt, hủy/close; không Scan/Fire/Save/VERIFY_STOP chồng |

Khi RUNNING bị mất dữ liệu, quyền Goal/arm vẫn phải theo operator thực, không
chỉ theo phase coarse. Lỗi input map/lưu map không tự biến thành lỗi STOP.

## 5. Luồng chờ và phục hồi

### 5.1. Luồng chữa cháy

IDLE → kiểm map/receipt/lock/mạng → Pi pairing → clock gate → Pi I/O →
camera → SEAL_PI → mở Nav2 + RViz → Initial Pose → YOLO → consumers/operator →
READY theo R5. Không arm/gửi goal chỉ vì process đã mở.

### 5.2. Luồng quét

IDLE → cùng preflight/pairing/clock/Pi I/O và seal → SLAM + RViz/camera →
terminal teleop → lưu map có transaction → dừng sạch → chọn map mới/cũ →
chữa cháy. Khi user đang quét mà bấm chữa cháy: hướng dẫn lưu nếu cần, dừng/
xác minh rồi mới chuyển mode; không chạy SLAM và Nav2 chồng nhau.

### 5.3. Ngân sách thời gian

| Công đoạn | Quy tắc |
| --- | --- |
| R5 readiness | Giữ PI_IO60s, CAMERA30s, SEAL20s, POSE185s, YOLO180s, CONSUMERS180s |
| Quét map | Giữ các deadline mapping hiện có, đồng bộ nhịp STATE với R5; không poll SSH chồng |
| Chờ TF/camera ổn định | Trong deadline hiện tại; hiển thị lý do + thời gian. Dữ liệu chập chờn reset timer ổn định, không reset deadline tổng |
| Retry camera vốn có | Giữ tối đa1lần RETRY_PREARM đúng phase; không phát lại START/SEAL khi ACK thất lạc mà chưa đối chiếu STATE |
| STOP verification | Giữ thời hạn/điều kiện của verifier R5; không tăng tuổi dữ liệu hoặc dùng snapshot trước STOP |
| App prearm failure drain | Dự kiến tối đa30s, cấu hình riêng và test ngân sách poll/SSH/STOP; không gia hạn20s admission |
| Bấm kiểm tra lại | Audit SSH có deadline, hiển thị attempt/kết quả; không tự mở một phiên chạy robot |

Mọi RPC/blocking operation phải bị giới hạn và nằm ngoài Qt loop. STOP đi
đường ưu tiên; timeout mạng không được làm mất phản hồi nút hoặc trì hoãn
việc rút ý định điều khiển. Deadline30s phải được xác nhận bằng fault tests;
nếu nguồn chết hoặc hết hạn thì giữ STOP_UNCONFIRMED, không thêm vòng chờ vô hạn
để che lỗi. Nếu còn tiến trình do app sở hữu, supervisor tiếp tục giữ quyền
thu dọn và báo rõ, không bỏ PTY/orphan chỉ để đóng UI.

### 5.4. Phối hợp lỗi trước khi chạy, chỉ thuộc runner app

Khi seal/readiness thất bại, app Pi entry ghi lỗi và chuyển trạng thái chỉ
cho STATE/STOP; từ chối START/RETRY/SEAL/arm mới. Giữ serial/watchdog/telemetry
trong cửa sổ drain hữu hạn để laptop verifier lấy proof zero/OFF/home mới,
rồi local drain xong mới yêu cầu Pi STOP. Hết cửa sổ hoặc mất laptop: Pi tự
cleanup, mọi admission đã rút. Không dùng đường này để chờ vô hạn khi đang
chạy/đang phun; lỗi runtime vẫn đi đường dừng hiện hành.

App startup timeout không còn trực tiếp khiến Pi mất I/O trước verifier.
Watchdog/emergency-stop vẫn hoạt động trong toàn cửa sổ; phải có ca chứng
minh không xuất hiện nonzero/ON/arm mới. Đây là phối hợp vòng đời, không thay
logic dừng motor hoặc hạ yêu cầu proof. R5 CLI vẫn dùng PiController gốc.

### 5.5. Phân loại kết thúc và thao tác tiếp

| Tình trạng | Giao diện và cách tiếp tục |
| --- | --- |
| Preflight fail, chưa START Pi I/O | Báo lỗi cụ thể, về IDLE sau cleanup; không ghi “đã xác nhận dừng robot” |
| Startup fail, STOP=true, cleanup sạch | Hiện lỗi gốc + “đã dừng sạch”; cho Kiểm tra lại rồi Quét/Khởi động lần nữa |
| Đang chờ dữ liệu trong deadline | Hiện “Đang chờ TF/camera phục hồi”; không báo BLOCKED sớm |
| STOP chưa xác nhận | STOP_UNCONFIRMED; không cho mở phiên chạy mới; có chỉ dẫn xác minh dừng |
| Còn process/force/child error | CLEANUP_REQUIRED; báo thành phần chưa sạch, không kill theo tên/PID cũ |
| Thiếu kết nối khi audit | CHECK_FAILED có lý do; nút kiểm tra dùng lại được sau khi kết nối |
| Cleanup đạt nhưng ghi/archive lỗi | STORAGE_ERROR; giữ journal/pending, báo đường dẫn cần xử lý |

Không tự khởi động lại robot sau mất mạng/khởi động lại app, không tự gửi lại
goal, phun hoặc teleop. Retry mặc định là **user chủ động**, tạo session/
generation/pairing mới, tối đa1worker cho mỗi lần bấm.

### 5.6. Phiên cũ thiếu STOP: không bỏ qua

Pending phiên quét22:01:56 không thể tự chuyển thành “đã dừng” bằng bản sửa
mới hoặc reset ESP32. Giữ nguyên log/proof cũ và kết quả UNKNOWN/FAIL.

Làm đường **“Xác minh dừng trên robot”** tách khỏi **“Kiểm tra lại” chỉ đọc**:
chỉ khi user chủ động chọn thao tác có phần cứng, lock hai máy rảnh, tiến trình
cũ đã được kiểm và Pi/thiết bị đúng danh tính. Dùng I/O + STOP verifier R5 trong
phiên phục hồi mới, không Nav2/SLAM/YOLO/automatic/goal/teleop/nonzero/ON.
Đây có thể gửi OFF/home theo chuỗi STOP hiện có, phải mô tả đúng trên UI.

Chỉ proof mới đầy đủ + cleanup sạch mới cho archive pending cũ kèm quan hệ
“trạng thái hiện tại được xác minh bởi phiên phục hồi mới”; không gọi đó là
proof lịch sử của phiên cũ. Mất boot/binding, còn owner hoặc thiếu telemetry
thì từ chối và hướng dẫn xử lý có giám sát. Không âm thầm thêm bước phần
cứng vào nút audit. Ca này thuộc nghiệm thu phiên2, không tự chạy khi viết code.

## 6. Nhánh TF cần kiểm chứng, không lấy giả thuyết làm bản sửa

Phiên1 chuẩn bị entry health riêng dưới app, import class/checker R5 nguyên
byte, chỉ tách vòng spin callback khỏi nhịp sample/ghi file khoảng5Hz trong
process app. Không cài đè executable r5_startup_health hoặc sửa source node.

So hai chế độ cùng fixture/load; kiểm sample≤750ms, timer ổn định/identity/
profile không đổi và không dùng snapshot cũ để cấp quyền. Source publish,
listener độc lập, callback và lookup phải được phân biệt. Phép đo delay25ms
chỉ là positive control. Không thêm worker đo phụ nặng vào runtime mặc định.

Chỉ chọn chế độ observer mới cho artifact phát hành khi QA lỗi tải đạt và
phiên2 đối chiếu được trên Pi có tải thực. Nếu TF cũ ngay tại nguồn, ghi OPEN
ngoài lớp app; không sửa R5/ESP32 hoặc nới freshness cho “hết block”. Nếu chưa
đủ bằng chứng, artifact vẫn là candidate, không tuyên bố nghiệm thu.

## 7. File dự kiến

Tất cả đường dẫn dưới đây tính từ apps/fire_robot_control; tên mới là dự kiến.

| Nhóm | File và thay đổi |
| --- | --- |
| Sinh entry/đóng gói | services/candidate_build.py, package_candidate.py; mới services/packet_archive.py, app_startup.py, app_pi_lifecycle.py; output desktop/generated/app_fire_entry.py và app_pi_entry.py |
| TF app | mới services/app_health_entry.py; class/checker import từ R5 đã pin, mode được QA riêng |
| Điều phối | services/live_session.py, desktop_hooks.py, mapping_runner.py, supervisor.py, live_preflight.py; tái dùng session model nếu phù hợp, không tạo FSM cạnh tranh |
| Proof/phục hồi | mới services/session_outcome.py, stop_recovery_runner.py; services/recovery.py, recovery_probe.py và mapping_monitor.py nếu cần readiness/handoff; không sửa verifier R5 |
| Giao diện | native_rviz/main.cpp, observer.cpp/.hpp, session_ui.cpp/.hpp, operational.rviz, CMakeLists.txt; có thể tách RosView trong native_rviz để giảm main.cpp |
| Launcher/Pi | mới launchers/run_app.sh; chỉnh start_candidate.sh, start_candidate_desktop.sh, sync_pi_candidate.sh và desktop entry app nếu cần; prepare_candidate.sh tách build với chọn current |
| QA | tests/test_* liên quan; mới test_app_entry.py, test_app_startup.py, test_session_outcome.py, test_app_pi_lifecycle.py, test_app_health_entry.py, test_stop_recovery.py và probe graph/Qt/packet |
| Tài liệu | kế hoạch này, ma trận QA, README, hướng dẫn offline và báo cáo/evidence mới; không viết lại lịch sử PASS/FAIL cũ |

Nếu cần thay ngoài danh sách/ngoài apps để tiếp tục, ghi rõ nguyên nhân và
giữ yêu cầu không thay R5. Không tự mở rộng sang firmware/mission/Nav2 fix.

## 8. Tổ chức trong hai phiên

### Phiên1 — code và QA offline trọn luồng

1. Chụp hash/argv/profile/dirty state; tạo case hồi quy các lỗi đã có bằng
   receipt/replay đã loại secret. Các lỗi xác nhận phải FAIL trước sửa.
2. Tách entry app + packet tooling; giữ364 file R5 gốc. Test cả artifact thật,
   không chỉ mô hình thuần; CLI R5 mặc định PLAN_ONLY không khởi động ROS/SSH.
3. Làm progress/outcome, Pi prearm drain và STOP-only readiness/handoff; nối
   cả mapping/fire vào cùng contract. Tách lỗi gốc khỏi lỗi cleanup.
4. Làm Qt shell/lazy RViz, phản hồi nút, state phục hồi và đường STOP-only
   chủ động. Giữ popup map và held-click qua STATUS đã sửa.
5. QA service/Qt/fake SSH/PTY, replay hai lượt lỗi, TF loaded fixture cô lập,
   kill/loss/storage fault, package/rollback. Sửa đúng delta rồi test lại phần
   ảnh hưởng, không dừng giữa các bước để hỏi tiếp tục.
6. Build artifact mới chưa chọn current. Ghi registry case/evidence, lỗi OPEN,
   manifest/binary/archive; hiện bản GUI fake cho user xem được.

**Bàn giao phiên1:** code trong apps + candidate chạy thử offline + báo cáo
QA thực tế + một lệnh mở app và runbook offline; R5/pending thật nguyên.
Không cộng189 case parent vào kết quả mới nếu chưa map/kiểm lại ảnh hưởng.

### Phiên2 — Pi riêng và nghiệm thu

1. Có kết nối/ủy quyền phù hợp: kiểm hash R5 Pi, sync vào stage app mới,
   build/static inventory/hash/receipt; không chạy robot trong bước sync.
2. Mở candidate bằng đường dẫn tường minh, chưa tự thay app đang dùng. Sau đó
   kiểm qua WiFi Pi đúng profile; LAN chỉ phục vụ đồng bộ như workflow hiện có.
3. Có user giám sát: xác minh dừng cho pending cũ nếu cần; quét→lái→lưu→chọn
   map mới/cũ→dừng→chữa cháy→Pose→quan sát sẵn sàng→X/Q. Thử dừng lúc startup.
   Chạy motion/pump là mục tùy chọn riêng, không cần phun để nghiệm thu launcher.
4. Đo TF/health/reader/CPU đủ phân biệt giả thuyết. So R5 CLI và app tuần tự,
   không song song. Nếu lượt R5 cũ cũng lỗi, giữ nguyên và báo riêng.
5. Sửa delta chỉ thuộc app nếu phát hiện; chạy lại ca liên quan, chụp hash
   trước/sau. Chỉ chọn current/shortcut khi các gate phát hành đạt.

Mục tiêu2phiên không phải cam kết mọi lỗi USB/WiFi/TF ngoài app đều sửa được.
Thiếu robot/kết nối hoặc QA phần cứng chưa đạt phải ghi NOT_RUN/FAIL chính xác,
vẫn hoàn thành phần code/offline được phép; không phát hành để kịp số phiên.

## 9. QA, lệnh verify và tiêu chí phát hành

Xem [ma trận96 ca và cách chạy](QA_APP_ENTRY_AND_RECOVERY_2026-10-29.md).
QA bao phủ hai entry, tất cả state/action của app, race/timing/ownership,
STOP/cleanup/retry, map, giao diện, đóng gói và các nhánh lỗi đã thấy.
Không thể chứng minh mọi sự cố vật lý bằng QA offline.

Các gate bắt buộc:

- G0: hash/argv/install/lock chứng minh R5 bảo toàn; không có diff ngoài apps.
- G1: unit/replay/fault/adapter/package đạt; hai lỗi đã xác nhận được kiểm trước/
  sau; thiếu STOP luôn bị từ chối.
- G2: Qt/RViz/IPC thực + ROS giả localhost đạt; không có UI DDS reader trước
  gate hoặc sau tháo view; không thêm pump reader/owner trái contract.
- G3: Pi static sync đạt và workflow WiFi có giám sát có evidence; CLI R5 vẫn
  dùng được. “App mở được” hoặc receipt static không thay cho G3.

Release cần không còn P0/P1 thuộc app trong phạm vi, đầy đủ log/registry/packet/
rollback. Ca phần cứng chưa chạy không được ghi PASS hoặc lặng lẽ bỏ khỏi ma trận.

## 10. Rollback

- Build/sync tạo bản mới, không ghi đè artifact6734 hoặc stage R5.
- Chọn current mới là bước riêng sau QA; lưu selector cũ và receipt tương ứng.
- Rollback chỉ khi không còn phiên sở hữu/cleanup chưa xong; trả launcher app
  về artifact cũ và Pi stage khớp. Không dùng receipt của packet khác.
- R5 terminal dùng đúng wrapper/stage0c54 trước giờ, không cần rollback app để
  sửa source R5. Vẫn phải chờ common lock rảnh/không chạy chồng.
- Không xóa journal/pending/log để rollback; không có remote pkill, resetNav2,
  flash hoặc thay mạng tự động.

## 11. Trạng thái lượt lập kế hoạch

Chỉ inspect source/log/document local và ghi kế hoạch/ma trận. Trước ghi tài
liệu đã kiểm R5 release364 + candidate364 và app hiện tại416 đúng manifest.
Chưa implement entry/recovery mới, chưa chạy96 ca, chưa sync/SSH/chạy robot.
Kết quả kiểm cấu trúc tài liệu và hash sau ghi được lưu cùng kế hoạch.
