# Kế hoạch triển khai theo mốc

**Đây là kế hoạch gốc; user đã yêu cầu triển khai.** Trạng thái thực tế và các
phần còn thiếu xem [tiến độ](IMPLEMENTATION_PROGRESS.md). Mỗi mốc phải có kết quả
QA đúng phạm vi và lỗi còn mở trước khi nối sang phần có quyền chạy robot.

## Gộp công việc theo yêu cầu user

Cập nhật29/10: vẫn có **7 mốc M0–M6**, nhưng tổ chức thành **2 phiên chính**:

1. **Phiên1 — code và QA offline liên tục, M0–M5.** Thử native RViz trước;
   sau khi đạt tự tiếp tục UI/theme, report/camera, map/TTY/save, runner/pairing,
   X/Q/lease và đóng gói. Cuối phiên có app mở thử được, kết quả QA và danh sách
   phần chưa nghiệm thu bằng robot thật. Checkpoint lỗi được sửa và kiểm ngay;
   không gộp mọi QA tới cuối, không cần user xác nhận lại từng mốc thông thường.
2. **Phiên2 — nghiệm thu robot thật, M6.** Khi user cho phép và cấu hình/deployment
   phù hợp: kiểm mapping/save/select, start/fire, X/Q và logs. Đây là nghiệm thu
   có phần cứng, không phải phiên viết lại app; sửa delta phát hiện trong đúng
   scope nếu được yêu cầu và chạy lại case liên quan.

Khi user yêu cầu bắt đầu phiên code theo kế hoạch gộp, scope là **toàn M0–M5**,
không mặc định dừng sau PoC M0 để hỏi có tiếp tục không. Tự chọn kỹ thuật thường
lệ, giữ cập nhật tiến độ và bằng chứng; chỉ cần thông tin/ủy quyền mới khi thiếu
thực sự, phải đổi yêu cầu ngoài scope, hoặc bước phụ thuộc phần cứng chưa được phép.
Nếu bị chặn, làm tiếp phần độc lập có ích và báo chính xác việc còn thiếu.

Một phiên có thể tiếp tục qua các lần rút gọn ngữ cảnh; không tách lại việc đã
làm hoặc lặp QA đã đạt. Hai phiên là mục tiêu tổ chức, không bảo đảm trước mọi
dependency/OpenGL/ABI/deployment đều sẵn có hoặc mọi lỗi đều xong trong một lượt.
Gộp phiên không bỏ gate, thay behavior hoặc nới safety để chạy nhanh.

## 1. Lựa chọn kiến trúc

| Phương án | Lợi ích | Điểm cần xử lý | Quyết định đề xuất |
| --- | --- | --- | --- |
| Desktop Qt5 + RViz native | Tận dụng UI desktop, RViz/tools thật, offline, gọi runner sẵn có | Host/panel Qt C++, ownership khi runner vốn tự mở RViz, ROS/Qt event loops | **Ưu tiên làm PoC** cho mục tiêu một cửa sổ |
| Python Qt5 launcher độc lập | Nhanh cho catalog/nút/log, tái dùng Python runner | Không mặc định có Python binding RViz Humble; nhiều cửa sổ chưa đạt yêu cầu cuối | Có thể dùng cho prototype dịch vụ/mốc trung gian, không gọi là sản phẩm hoàn chỉnh |
| Web dashboard | Chạy trên browser | Cần backend lifecycle, renderer map/camera, tài nguyên offline và contract riêng; rosbridge hiện chỉ là server | Không chọn cho vòng triển khai đầu |

Qt5 phù hợp dependency RViz Humble trên máy đã khảo sát. Không chọn Qt6 chỉ
vì mới hơn, không pip-upgrade Qt/ROS để làm UI. Ngôn ngữ host RViz có thể là
C++/Qt5; việc app dùng Python runner không bắt buộc UI cũng là Python.

M0 phải quyết định **RViz native host mở rộng VisualizationFrame hay RViz panel
plugin** dựa trên resize/render/tool/lifecycle thực. Ưu tiên host native nếu đáp
ứng để app tồn tại trước startup và sau STOP; plugin là phương án thay thế cần
ghi rõ tác động ownership. Không hứa ghép hai cửa sổ ngoài process bằng reparent
window ID. Không viết lại map renderer bằng canvas chỉ để giống mockup.

## 2. Thành phần và nơi đặt code sau này

Root app: `/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/apps/fire_robot_control`.

```text
fire_robot_control/
  README.md
  docs/                  tài liệu hiện tại và ảnh tham khảo
  ui/                    MainWindow/native RViz, CameraDock, StatusDock, theme
  services/              SessionController, RunnerBackend, MapCatalog, Observer
  config/                đường dẫn runtime, map roots, RViz layout, timeout UI
  assets/                icon local, không CDN
  tests/                 fake runners, reports/replay fixtures, tests GUI/services
  native_rviz/           build metadata/adapter native nếu M0 chọn host/package
  launchers/             script mở app có môi trường đúng, desktop entry template
```

Đây là **cấu trúc dự kiến**, chưa tạo các source/thư mục code trong lượt tài liệu.
Vị trí C++ headers/sources có thể thống nhất khi kết thúc M0; không giữ hai bộ
MainWindow trùng chức năng Python/C++.

Nếu tạo ament package dưới app, tên package riêng, build/install/log riêng và
kiểm colcon discovery từ gốc repo/workspace. Không chép package cùng tên từ
release vào root ROS workspace. Marker ignore/build command phải được thử rõ,
không làm mất package đang build hoặc build cả snapshot vô tình.

## 3. Trách nhiệm các lớp

- **View:** widget/ảnh/format tiếng Việt, gửi ý định người dùng, không cấp quyền
  actuator; RViz native giữ công cụ Pose/Goal theo config đã kiểm.
- **Observer:** subscribe tối thiểu report+compressed image; latest-only,
  decode ngoài UI loop, không thêm reader pump command. Giữ source provenance.
- **MapCatalog:** tìm/validate cặp file, snapshot selection, save transaction;
  không sửa file map đã có để hợp lệ hóa nó.
- **SessionController:** chỉ quản lý phiên app và tiến trình, không suy luận
  mission. Cờ trạng thái robot chỉ đọc từ report.
- **RunnerBackend:** dùng structured argv, môi trường explicit, log/session ID;
  giao việc khởi động/dừng thật cho runner hiện hành.
- **Operator adapter:** nhận X/Q/frontend lease vào loop operator; giữ core,
  intent executor, STOP verification và quyền của sáu consumers.

QProcess có thể theo dõi wrapper/stdout asynchronously nhưng không thay bằng
chứng PID/start_ticks, process group, child log hoặc receipt Pi. Không dùng
`waitForFinished`, shell dài, SSH, graph query hoặc JPEG decode trên UI thread.

## 4. Vòng đời phiên app

Các nhãn sau thuộc **app lifecycle**, không phải state machine robot mới:

```text
IDLE → CHECKING → STARTING_MAPPING → MAPPING → SAVING → MAPPING
IDLE → CHECKING → STARTING_FIRE → WAIT_INITIAL_POSE → FIRE_RUNNING
MAPPING → STOPPING_MAPPING → CHECKING → STARTING_FIRE
MAPPING/FIRE_RUNNING/STARTING_* → STOPPING → STOPPED
bất kỳ tác vụ → ERROR hoặc STOP_UNCONFIRMED khi thiếu bằng chứng
```

- Chỉ một phiên mapping/fire; khóa app và đối chiếu lock/preflight backend.
- Trong STARTING/SAVING/STOPPING, khóa nút xung đột; Q luôn có ưu tiên.
- Không báo READY từ việc spawn thành công, chuỗi log cũ hoặc ping.
- Map/file/profile được snapshot lúc start; thay dropdown không ảnh hưởng phiên.
- Lần mở app sau crash kiểm journal/runner lock và report thực, không auto
  resume điều khiển. PID cũ phải đối chiếu birth/session, không kill theo tên.
- Tách ba kết quả: robot STOP proof, các group đã thoát, child exit sạch.
  Nav2 SIGTERM/RC2 là cleanup degraded dù STOP thành công. Không đổi thành PASS.
- Nếu STOP hoặc ownership exit chưa chứng minh: chặn phiên mới. Nếu STOP và
  các group đều đã chứng minh thoát nhưng child cleanup degraded: giữ cảnh báo,
  chỉ cho mở phiên mới sau preflight fresh của runner đạt; không khóa vĩnh viễn
  chỉ vì error log lịch sử, không tự restart Nav2 để che lỗi.

## 5. Các mốc triển khai

### M0 — thử kỹ thuật RViz native và chốt thiết kế

**Chạm dự kiến:** source PoC riêng dưới `native_rviz/`, fake fixtures/UI; ADR
trong docs. Không sửa runtime source hoặc mở Pi.

1. Ghi phiên bản distro/Qt/RViz, session display X11/Wayland, dependency sẵn có.
2. Dựng cửa sổ Qt5 dùng RViz native, giữ Displays/Views/Pose/Goal toolbar;
   gắn camera fake và status dock; kiểm font tiếng Việt, resize, close/reopen.
3. Preview pure UI không DDS. Phần RViz cần context chỉ thử với mock/test domain
   cô lập và dataset local; không để xuất command vào domain robot.
4. Chứng minh một cửa sổ, không có rviz2/OpenCV trùng; đánh giá host vs panel.
5. Chốt C++ host/package hoặc phương án native khác bằng diff/screenshot/log.

**Qua mốc khi:** RV01–RV04/UI01–UI05 đạt trong môi trường đã ghi; không có
command output; không chọn lại kiến trúc ở các mốc sau nếu chưa có lý do mới.
Nếu native không đạt, ghi blocker thật và phương án, không đưa ảnh giả thay PoC.

### M1 — giao diện và mô hình dữ liệu giả

**Chạm:** `ui/`, `assets/`, `config/theme`, Observer/DTO thuần và fake backend.

1. Bố cục trắng–đỏ–xám theo REQUIREMENTS; theme không phá property tree RViz.
2. Hiển thị task/reason, YOLO, IR, actuator, goal status, connection với
   VALID/STALE/INVALID/UNKNOWN; giá trị không có giữ dấu “—”.
3. Thêm preview/replay mode đóng nhãn rõ và cấm mọi side effect runner/IPC live.
4. Ảnh giữ tỉ lệ, queue tối đa một frame chờ, log view có giới hạn bộ nhớ.

**Qua mốc khi:** UI/OBS offline đạt; import/mở preview không SSH, không model
load/inference, không mở serial, không khởi tạo command publisher.

### M2 — quản lý tiến trình và session trên fake runner

**Chạm:** `services/SessionController`, `RunnerBackend`, profile paths, tests.

1. Source môi trường bằng wrapper local cố định, mọi dữ liệu user là argv.
2. Quản lý async runner Pi, nhận pairing từ stdout đúng phiên, kiểm private
   binding rồi gọi laptop runner; không dùng “file pairing mới nhất” glob.
3. Tái sử dụng ownership/cleanup và receipts; app không gửi tín hiệu cả process
   group lồng nhau làm bypass STOP drain của operator.
4. Test startup fail ở mỗi stage, repeated clicks, app crash, child crash,
   PID reuse, pipe/log flood, timeout và late output của phiên cũ.
5. Sau này nút Start là thao tác user cho phép chạy profile đã chọn, không
   thêm modal xác nhận thừa; setup SSH key/known host/dependency làm một lần.
   Mở app hoặc nhấn Kiểm tra Pi không tự chỉnh NTP. Runner start giữ clock gate
   hiện hành; không duplicate NTP helper ở UI.

**Qua mốc khi:** PROC01–PROC12 và ENV đạt trên fake, evidence chứng minh signal
order/local→Pi, mutation retry không replay; không có SSH thật trong gate này.

### M3 — mapping, TTY, save và map catalog

**Chạm:** app mapping adapter/wrapper, MapCatalog, tests. Nếu sửa root mapping
script, phải tách diff riêng và kiểm tools hồi quy tương ứng, không sửa demo.

1. Dùng root `mapping/save_map` làm nguồn tham khảo nghiệp vụ; không copy toàn
   bộ demo cũ. Giải quyết Pi profile rawSerial/ownership để cùng một mode manager.
2. Mở terminal với PTY/TTY thật cho teleop, process và sentinel/receipt được quản
   lý tường minh. Không dùng PID của terminal server như PID teleop. Chỉ thêm
   terminal emulator dependency khi kiểm máy chưa có backend phù hợp.
3. Với UI cuối, runner mapping không mở RViz thứ hai; UI host giữ view SLAM.
   Launcher trung gian có thể giữ cửa sổ cũ nhưng phải ghi chưa đạt R03.
4. Save chạy async khi SLAM còn sống; giữ rule tên/no overwrite; kiểm YAML
   safe-load, image resolution/origin, readable valid PGM và path resolve.
5. Timeout/disk-full/partial save không đổi lựa chọn hoặc báo thành công;
   không xóa map cũ, không tự ghi đè. Lưu xong chọn map mới.
6. Catalog hợp nhất map root/candidate đã cấu hình, hiển thị path khi trùng tên;
   wrapper app nhận generic validated map path để gọi runner `--map`.
   Selector hiện có910/TN1 chỉ là compatibility; không sửa sealed release tools.
7. Khi Start fire từ mapping: teleop zero/exit, SLAM dừng, map save hoàn tất,
   xác minh publisher/ownership và Pi session kết thúc/chuyển đúng contract;
   rồi mới tạo pairing R5 mới. Không reuse Pi stack cũ ngoài binding R5.

**Qua mốc khi:** MAP01–MAP13/PROC06–PROC07 fake/local offline đạt, chọn file có
path chứa khoảng trắng đúng, map mới không bị rơi về910. Đổi map đơn thuần không
cần flash hoặc sync Pi; thay runtime packet là trường hợp khác.

### M4 — adapter operator, X/Q và ownership frontend

**Chạm có điều kiện:** integration copy của `operational_session.py`,
`operational_view.py`, `r5_laptop_session.py`, runtime recovery/cleanup nếu cần,
cùng tests đúng phần; không sửa core mission/firmware để làm UI.

1. Thêm external-view/intent adapter theo DATA_CONTRACT. X/Q chỉ đi vào handler
   thật; headless operator vẫn nhận/giải mã ảnh và duy trì proof/heartbeat.
2. Frontend lease phản ánh UI loop còn đáp ứng, không chỉ backend còn sống;
   lỗi Qt/IPC/frontend chết phải request_shutdown. Không bỏ close→STOP.
3. Chuẩn hóa ai sở hữu view: app host tồn tại xuyên startup/STOP, runner coi
   frontend là external registered view. Không spawn stock RViz/OpenCV lần hai,
   không để runner cleanup kill cha Qt trước khi STOP/receipt hoàn tất.
4. Recovery giữ view/Pose/Nav2; kiểm không có old generation event/cycle replay.
5. Điểm điều khiển Pose/Goal chỉ hoạt động trong mode phù hợp; Esc/nút dừng có
   đường ưu tiên, X/Q ngoài ô nhập dùng shortcut. Không biến chữ X/Q khi nhập
   tên map thành lệnh; form không khóa STOP vì modal hoặc camera focus.
6. Nếu đụng file trong packet: làm integration copy/new release được review,
   tạo định danh packet/tool/receipt mới theo quy trình dự án. Không sửa bytes
   trong release0c54 đã phát hành, không sửa manifest0c54 để che delta.
   Host laptop thay đổi không cần firmware mới; Pi staging/receipt tương thích
   cần xác minh theo packet, không khẳng định luôn miễn sync vì chỉ sửa GUI.

**Qua mốc khi:** CTRL/REC fake đạt, test source GID/subscriber counts không đổi,
headless không bỏ proof, shutdown ảnh hưởng đúng phạm vi. Chỉ thêm adapter mới;
report không được giả `confirmed`, ACK, READY hoặc COMPLETE.

### M5 — tích hợp offline và đóng gói desktop

**Chạm:** launcher, desktop template, docs vận hành, tests; UI package build
nếu M0 cần, không build lại toàn ROS vô cớ.

1. Cài/resolve dependency local trước; app sử dụng khi không có Internet.
2. Local replay báo cáo/ảnh đã có; nếu ROS replay cần thiết, dùng domain212
   cô lập, chỉ topic telemetry allowlist và terminal test riêng. Không replay
   rosbag chứa cmd/pump/arm/goal vào mạng robot.
3. Kiểm resource/performance, responsive STOP, lặp đóng/mở và installer path.
4. Desktop shortcut source đúng Humble/underlay/profile, không phụ thuộc cwd;
   không cài vào home hoặc autostart trước khi bước triển khai đó được yêu cầu.
5. Ghi hướng dẫn dùng app, vị trí log, lỗi thường gặp, rollback về runner CLI.

**Qua mốc khi:** ENV/RV/PERF/OBS/PROC/CTRL/REC liên quan đạt và có evidence;
preview không biến thành live sau lỗi cấu hình. Chưa gọi hardware PASS.

### M6 — nghiệm thu với robot, tùy chọn riêng

Chỉ khi user yêu cầu/chấp nhận lượt runtime và profile/bản Pi phù hợp; việc
duyệt plan hoặc code UI không tự cấp quyền hardware. Thực hiện LIVE01–LIVE06
theo QA_PLAN, không cố tình rút USB/nguồn khi đang phun để test UI.

Các bước: read-only endpoint audit trong phiên được phép → mapping/save →
chọn map → Start/InitialPose/goal → X → Q → restart. Thử lỗi nguy hiểm dùng
fake/replay; hardware chỉ quan sát lỗi tự xảy ra hoặc kịch bản được thống nhất.
Không lặp toàn R4/Gate1 hay dùng QA app để đóng lỗi USB/Nav2 còn mở.

## 6. File được chạm theo phạm vi

| Khu vực | Phạm vi dự kiến |
| --- | --- |
| `apps/fire_robot_control/**` | Code/docs/config/assets/test app; là nơi thay đổi chính |
| `scripts/start_control_app.sh` | Tùy chọn wrapper mới, không thay start_demo bằng app |
| `scripts/fire_robot_session.py`/Pi helper | Chỉ mapping adapter khi cần, diff nhỏ, giữ root workflow cũ |
| R5 operator/runner integration copy | Chỉ bridge UI và external-view, có packet/release mới nếu thay pinned source |
| `ros2_ws/src/...` hoặc native package | Chỉ nếu M0 cần dependency/build hook rõ; tránh duplicate package |
| `docs/codex/` | Có thể thêm entry handoff ở lượt implementation, không bắt buộc lượt hiện tại |
| `ai/`, firmware, network, maps cũ | Ngoài scope sửa; map mới chỉ qua thao tác Save có chủ đích |
| `releases/r5_20261029/**` | Nguồn baseline để đọc/rollback, giữ immutable trong task UI |

## 7. Rủi ro và biện pháp

| Rủi ro | Biện pháp / dấu hiệu kiểm |
| --- | --- |
| Python Qt được giả định nhúng được RViz | PoC Qt5 native trước; không dùng ROS1 binding hoặc ảnh giả |
| Hai RViz/operator/owner chạy trùng | Session lock + runner preflight + external-view registration; test repeated clicks |
| Observer khiến bơm không nhận quyền | Cấm reader `/pump_cmd`, đối chiếu graph trước/sau ở gate runtime |
| Shell/path/map injection hoặc path có khoảng trắng | Structured argv, YAML safe-load, allowlisted map roots, test quote/metachar |
| Save xong nhưng map sai/còn thiếu | Validate cặp file và metadata, chọn atomically sau success; no overwrite |
| Mapping cũ để lại Pi ngoài binding | Unified session adapter; remote receipt và preflight trước fire |
| Frontend treo nhưng backend auto tiếp tục | Lease gắn UI event loop; request_shutdown khi mất lease; không nới guard |
| UI báo dừng từ rc0/SSH disconnect | Đọc STOP, local groups, remote receipt và child clean độc lập |
| Lỗi cleanup Nav2 sẵn có bị che | Hiện degraded/UNCONFIRMED đúng dữ kiện, giữ ledger lỗi baseline |
| Subscription/decode tăng tải WiFi/CPU | Dùng annotated laptop, latest-only, bounded log, observer tối thiểu |
| Fake/replay phát lệnh thật | Backend fake fail-closed, deny command topics, domain cô lập, không import runtime có side effect |
| Cấu hình cũ hoặc đổi packet ở chat khác | Snapshot profile/hash trước mỗi mốc, không dùng receipt parent hoặc sửa baseline |

## 8. Rollback

Giữ source/packet/CLI baseline hiện hành và profile trước khi tích hợp. Không
đổi shortcut CLI cũ thành app, không gỡ dependency ROS dùng chung. Rollback chỉ
sau STOP/cleanup đúng phiên; dừng app, bỏ profile integration, dùng lại runner
baseline đúng receipt. Không git reset/clean lên worktree người dùng, không xóa
map/log hoặc force-kill tiến trình khác. Nếu STOP/remote exit chưa chứng minh,
báo rõ giới hạn và xử lý nguyên nhân, không coi rollback filesystem là robot dừng.

## 9. Điều kiện hoàn tất sản phẩm

Tất cả R01–R18 có traceability và gate tương ứng; một cửa sổ RViz+camera thật;
nút/start/save/select/TTY hoạt động; X/Q giữ proof; offline/shortcut đạt; không
thêm owner hoặc giảm safety; log/packet/profile truy vết được; các lỗi baseline
còn mở ghi rõ. Phân biệt nghiệm thu app với nghiệm thu toàn bộ robot R5.
