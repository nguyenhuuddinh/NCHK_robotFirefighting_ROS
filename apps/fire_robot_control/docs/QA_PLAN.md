# Kế hoạch QA chi tiết

Ngày29/10/2026. **86 ca dự kiến; tất cả NOT_RUN trong lượt lập tài liệu.**
Không có code app để chạy các suite bên dưới. Kiểm cấu trúc/link/hash tài liệu
ở lượt này không phải kết quả những ca app, không cộng vào QA50/71 của robot.

## 1. Gate và thứ tự

| Gate | Môi trường | Phạm vi / điều kiện qua |
| --- | --- | --- |
| G0 | Filesystem, không ROS/network | Đọc đúng source, scope, dependency, dữ liệu và traceability |
| G1 | Fake backend, GUI local/offscreen khi phù hợp | Preview, parser, catalog, process simulation, X/Q/lease không side effect |
| G2 | Native GUI thực trên laptop, mock/replay local cô lập | RViz thật, event loops/OpenGL, một cửa sổ, nguồn giả không có quyền robot |
| G3 | Robot thật, **tùy chọn riêng khi user cho phép** | Audit graph, mapping/fire/stop theo kịch bản được xác định; không gộp với G1/G2 |

Không chuyển gate khi P0/P1 thuộc scope app còn mở. P0: có thể bypass safety/
điều khiển nhầm/không dừng; P1: sai lifecycle/map/source/STOP proof; P2: hiển thị,
usability/performance ảnh hưởng quan sát. Known issue baseline ghi riêng, không
đổi FAIL thành PASS hoặc mặc định lỗi nào cũng do UI.

Theo lịch gộp của user, G0–G2 được thực hiện xen kẽ implementation M0–M5 trong
cùng phiên code; checkpoint đạt thì tiếp tục, không dừng để xin xác nhận thường
lệ. G3 thuộc phiên nghiệm thu phần cứng riêng. Giữ86 case dự kiến, chọn đúng case
áp dụng theo delta và ghi trạng thái; gộp phiên không gộp PASS giả hoặc bỏ kiểm.

## 2. Quy tắc test và bằng chứng chung

- Fake runner phải ghi argv/env/signal/ownership/event, có kịch bản delayed,
  fail, crash, duplicate output, PID reuse. Stub SSH/ROS/serial mặc định raise
  nếu bị gọi; unit test không import module có side effect hoặc init ROS.
- Replay bắt đầu từ parser/in-memory reports. Chỉ dùng bag nếu đã inspect
  topic allowlist; không replay cmd/arm/pump/goal/cycle control. Không dùng dữ
  liệu có timestamp cũ để qua BOOT/current gate. Domain212 chỉ ở shell test
  cô lập nếu cần; không sửa profile runtime domain0 để test qua.
- Lưu Git SHA + diff/source hash + packet + profile + OS/Qt/RViz/display backend
  + dataset manifest + test version. Logs có run ID duy nhất và local monotonic
  timeline, loại secret pairing/mật khẩu/SSH private key.
- Mỗi case có nodeid/ID, status, expected/actual, evidence, duration, issue.
  PASS phải có evidence assertion/receipt; screenshot chỉ chứng minh hiển thị.
- Chọn hồi quy theo source bị sửa: runner→r5_paired/owned logs; operator→X/Q,
  presence, no-goal, stationary/reentry/stop; mapping→root mapping tools. Không
  chạy lại fullR4/Gate1 hoặc test đã đạt chỉ để tăng số lượng.
- Không chạy lệnh hardware hay thay đổi thời gian khi đọc tài liệu này. Mọi
  thao tác/runtime mô tả ở bảng là **bước test tương lai**, không đã được phép chạy.

## 3. Ma trận test

Mọi hàng có trạng thái ban đầu **NOT_RUN**. Cột bằng chứng là dữ liệu phải lưu
sau khi implement, không phải file đã tồn tại. Cách test gồm setup/inject→action
ở cột “Bước”; assert đầy đủ ở cột “Kết quả bắt buộc”.

### 3.1. Tài liệu và phạm vi — G0

| ID | Mức | Bước | Kết quả bắt buộc | Bằng chứng |
| --- | --- | --- | --- | --- |
| DOC01 | P1 | So R01–R18 với annotation/yêu cầu đã chốt; tra ID QA | Không mất terminal teleop, map mới/cũ, màu, một cửa sổ và X/Q | Requirement trace report |
| DOC02 | P1 | Đọc context/manifest mới nhất; so profile C/S/underlay/remote stage | Không gọi source root/demo cũ là R5; không dùng receipt parent | Source/profile hashes |
| DOC03 | P1 | So git diff trước/sau mỗi mốc với scope | Không sửa ai/, firmware/network/safety hoặc dữ liệu người dùng ngoài scope | Diff/path allowlist |
| DOC04 | P2 | Kiểm local links, ảnh, commands dự kiến và status QA | Không có link hỏng, lệnh giả ghi như đã chạy, hoặc PASS không evidence | Documentation review |

### 3.2. Cài đặt/môi trường — G0/G1/G2

| ID | Mức | Bước | Kết quả bắt buộc | Bằng chứng |
| --- | --- | --- | --- | --- |
| ENV01 | P1 | Clone/copy repo ở path có khoảng trắng; mở app từ cwd khác | Resolve profile/workspace đúng; không phụ thuộc cwd hoặc `shell=True` | argv/env/path assertions |
| ENV02 | P1 | Chạy colcon discovery trước/sau thêm native package, theo từng base path | Không duplicate package; không build snapshot ngoài ý muốn | Package list diff |
| ENV03 | P1 | Sau setup local, chặn Internet nhưng giữ WiFi Pi hoặc fake Pi | App/asset/font dùng offline; không tải CDN/model/dependency lúc mở | Network deny log + GUI |
| ENV04 | P2 | Mở desktop shortcut từ file manager và launcher menu | Đúng Humble/Qt/profile; không cần nhập lệnh, lỗi startup nhìn thấy | Desktop launch log |
| ENV05 | P1 | Thiếu Qt/RViz/underlay/model/config lần lượt | Báo dependency cụ thể; chưa start robot; không tự pip/apt/flash | Fake launch count0 |
| ENV06 | P1 | Candidate/remote packet/receipt sai hoặc thiếu | Chặn trước live, chỉ rõ mismatch; không fallback sang baseline cũ | Validation log |
| ENV07 | P1 | Mở app; sai WiFi/route, SSH key thiếu, known-host sai lần lượt | Mở app không actuation; check bounded/báo lỗi; không đổi mạng hoặc bỏ host-key check | Transport stub log |
| ENV08 | P0 | Quét publisher/subscriber/side effects của observer và launch profile | Không output motor/pump/arm; không thêm reader pump; safety config nguyên | Static + fake graph audit |

### 3.3. Hiển thị và thao tác — G1/G2

| ID | Mức | Bước | Kết quả bắt buộc | Bằng chứng |
| --- | --- | --- | --- | --- |
| UI01 | P2 | Mở layout ở1366×768/1920×1080 và scale100/125/150% | Nền trắng/đỏ/xám; map/camera không đè; X/Q luôn thấy | Screenshots từng viewport |
| UI02 | P2 | Feed READY/WAIT/FAULT/STALE; xem ở grayscale/keyboard focus | Có chữ/icon, tương phản đọc được; không dựa riêng màu đỏ/xanh | Visual review |
| UI03 | P2 | Feed lý do dài/UUID/tiếng Việt rồi mở Details | Một lý do chính ngắn; raw không mất; camera không phủ kín lỗi | Screenshot + source payload |
| UI04 | P1 | Nhập tên map chứa chữQ/X; dùng Esc/nút STOP khi phiên đang chạy | Gõ chữ không phát lệnh nhầm; vẫn có đường dừng ưu tiên và không bị modal khóa | Key/event trace |
| UI05 | P2 | Kéo splitter, mất camera, đổi tỉ lệ ảnh, restart app | Giữ aspect ratio; trạng thái đủ đọc; layout invalid có fallback local | Layout snapshot |

### 3.4. Quản lý phiên/tiến trình — G1; chọn lọc G2

| ID | Mức | Bước | Kết quả bắt buộc | Bằng chứng |
| --- | --- | --- | --- | --- |
| PROC01 | P1 | Double-click Start/Scan20 lần; mở app thứ2 | Chỉ một workflow/runner, không trùng owner; app thứ2 chỉ báo phiên đang có | Spawn count/lock trace |
| PROC02 | P1 | Có phiên robot ngoài app hoặc PID trong journal đã bị tái sử dụng | Không nhận quyền/kill theo PID/name; yêu cầu xử lý đúng phiên | Ownership assertions |
| PROC03 | P1 | Fake mapping stages chậm/lỗi trước và sau SLAM start | Chỉ Ready khi proof đạt; rollback tiến trình đã start đúng scope | Ordered stage trace |
| PROC04 | P1 | Fake Pi stdout in log rác rồi pairing hợp lệ; laptop start | Chọn binding đúng phiên/packet, tự truyền pairing, không cần paste | Redacted argv/binding |
| PROC05 | P1 | Inject pairing cũ, malformed, duplicate, wrong boot, output muộn | Từ chối/không start laptop nhầm; secret không xuất log UI | Negative binding fixtures |
| PROC06 | P0 | Đang mapping/teleop; nhấn Start fire, giữ STOP proof chưa đạt | Không start Nav2, không kill Pi sớm; UI hiển thị blocker | Effect order/count0 |
| PROC07 | P1 | Mapping save hoàn tất→teleop/SLAM exit→remote exit→fresh preflight | Chỉ sau đủ điều kiện mới mở R5; tối đa một `/map` owner theo mode | Timeline + fake graph |
| PROC08 | P1 | Fail từng stage CLOCK/PI_IO/CAMERA/POSE/YOLO/CONSUMERS | Chờ/báo stage thật, cleanup đúng runner; không mở node phụ để “fix” | Parameterized fault results |
| PROC09 | P0 | User Q khi startup/cleanup, lặp Q/Ctrl-C | STOP idempotent; không rút ngắn drain hoặc reset deadline mỗi nhấn | Signal/deadline trace |
| PROC10 | P1 | Parent wrapper rc0 nhưng child còn sống; SSH thoát trước Pi | Không báo stopped/clean từ rc; kiểm group và remote receipt | Local/remote fake receipts |
| PROC11 | P1 | Robot STOP confirmed, groups đã thoát nhưng Nav2 bị TERM/rc2 | Hiện cleanup degraded, không gọi PASS; fresh preflight mới quyết định phiên sau | STOP vs exit UI evidence |
| PROC12 | P1 | GUI close/crash giữa startup; runner lỗi/pipe đầy/log flood | Không treo UI hoặc quên child; journal đủ đối chiếu, bounded logs | Crash/flood trace |

### 3.5. Map và terminal — G1/G2

| ID | Mức | Bước | Kết quả bắt buộc | Bằng chứng |
| --- | --- | --- | --- | --- |
| MAP01 | P1 | Nhấn Scan với Pi fake/rawSerial profile và mock `/scan`/odom | SLAM đúng profile; không tự gọi demo cũ/Nav2 | argv/profile snapshot |
| MAP02 | P1 | Spawn terminal qua PTY; kiểm stdin TTY và teleop argv | Keyboard dùng được, `/cmd_vel_raw` và tốc độ/profile giữ hiện tại; không reader/publisher trùng | PTY/argv evidence |
| MAP03 | P0 | Đóng terminal, mất focus/key input hoặc teleop child chết | Đường dừng/watchdog hiện tại còn hiệu lực; terminal PID không làm proof robot stopped | Child/STOP transition |
| MAP04 | P1 | SLAM hoạt động→Save tên mới→map_saver trả đủ cặp | Giữ SLAM tới save hoàn tất; validate file rồi thông báo và tự chọn map | Save transaction trace |
| MAP05 | P1 | Save khi chưa SLAM hoặc đang Nav2 | Từ chối, không ghi nhầm map server hoặc start saver mù | Saver call count0 |
| MAP06 | P1 | Tên map trùng, `../`, metachar, quá dài; path repo có spaces | Không overwrite/path escape/shell execute; input sai có lý do | Filesystem/argv assertions |
| MAP07 | P1 | Saver timeout/rc khác0, disk full, chỉ có YAML hoặc PGM | Không báo saved; map đang chọn cũ giữ nguyên; partial được đánh dấu | Fault files + selection |
| MAP08 | P1 | YAML hỏng/unsafe tag, image thiếu, resolution≤0, origin sai, PGM lỗi | Loại map không hợp lệ trước start; không sửa nội dung để qua gate | Parser negative dataset |
| MAP09 | P1 | Catalog có map mới,910,TN1 và hai tên giống ở hai roots | Thấy tất cả map hợp lệ và provenance path; không tự ưu tiên nhầm | Catalog snapshot |
| MAP10 | P1 | Chọn map mới không thuộc whitelist910/TN1→Start fake | Generic validated path truyền đúng runner; không silently fallback910 | Exact runner argv |
| MAP11 | P1 | Sau chọn thay/xóa file trước Start và giữa validate/start | Revalidate/snapshot hash hoặc abort có lý do; không chạy map khác | TOCTOU fixture |
| MAP12 | P1 | Đổi dropdown khi phiên đang chạy; kết thúc rồi đổi map | Không hot-swap âm thầm; phiên mới dùng đúng map và cần Pose phù hợp | Per-session map hashes |
| MAP13 | P0 | Save đang chạy đồng thời Start/Q; completion đến muộn | Serialize/cancel đúng; không tắt SLAM trước save cần thiết, không auto Start sau Q | Interleaving assertions |

### 3.6. Report, ảnh và freshness — G1/G2

| ID | Mức | Bước | Kết quả bắt buộc | Bằng chứng |
| --- | --- | --- | --- | --- |
| OBS01 | P1 | Feed toàn task stage/outcome thật, fault/hold/rebind kết hợp | UI bám report và ưu tiên reason đúng, không tự chuyển mission | Golden payload→widget assertions |
| OBS02 | P1 | JSON/type sai, payload quá lớn, NaN/Inf, missing field, schema lạ | Invalid/Unknown, không crash/không coi thành false/OFF | Parser negative results |
| OBS03 | P1 | Ảnh JPEG hỏng, future stamp, duplicate/lùi stamp, frame sai | Không hiển thị như frame mới; tuổi tăng và quality đúng | Image/frame fixtures |
| OBS04 | P1 | Ngừng ảnh, vẫn nhận report; rồi ảnh hồi | Giữ ảnh có nhãn stale/đen theo policy; không tự dừng/khởi động camera node để sửa UI | Fake clock screenshot |
| OBS05 | P1 | YOLO clear hợp lệ so với camera_stale/decode_error và report mất | “Không phát hiện lửa” chỉ khi có clear hợp lệ; stale không là clear | State table assertions |
| OBS06 | P1 | IR raw mới nhưng preview cũ và ngược lại; heartbeat lặp | Tuổi hai luồng riêng, không đếm heartbeat thành sample mới | Clock/sequence trace |
| OBS07 | P1 | ADC null/invalid so với fresh NO_FLAME_SIGNATURE; số giống nhau qua nhiều callback | Unknown khác no-flame; lấy count từ backend, không đếm ADC bằng so đổi giá trị | IR golden cases |
| OBS08 | P1 | ON request trước ACK, ACK đến muộn, ACK lặp rồi OFF/home | Timer từ báo cáo PWM ACK, không click; request/PWM/home phân biệt; không nói nước chảy | Actor/ACT trace |
| OBS09 | P1 | Sequence3lượt, căn/tiến hoàn tất rồi cấp quota, cycle mới | Widget bám consecutive/max/remaining; không reset do redraw/chuyển tab | Sequence fixtures |
| OBS10 | P1 | Monotonic/wall/ROS time lùi hoặc Pi/laptop khác origin | Không tính latency giả/tuổi âm thành fresh; giữ backend clock fault | Clock discontinuity cases |
| OBS11 | P1 | Ping/SSH tốt nhưng thiếu Pose/identity/actor/readiness | Không hiện AUTO ready; một blocker dễ hiểu + chi tiết gốc | Health/report cases |
| OBS12 | P1 | Map tĩnh không republish, TF static lâu, odom TF ngừng | Map không bị gọi disconnected vì im lặng; TF động cũ cảnh báo, không refresh stamp | Static/dynamic fixtures |

### 3.7. X/Q, frontend và an toàn — G1/G2

| ID | Mức | Bước | Kết quả bắt buộc | Bằng chứng |
| --- | --- | --- | --- | --- |
| CTRL01 | P0 | Nhấn X khi navigation có goal, khi fire cycle và khi idle | Chỉ gọi đúng handler X; cancel đúng UUID/context backend, không gửi motor/pump | Handler/event audit |
| CTRL02 | P0 | Nhấn Q/Esc/nút stop hoặc đóng app ở từng phase, kể cả đang ON | request_shutdown thật, giữ backend/telemetry tới drain, không chờ xác nhận modal | Phase×stop matrix |
| CTRL03 | P0 | Send ACK request nhưng robot proof chưa đạt/thiếu ACT | UI hiện đang dừng/UNCONFIRMED, không báo Đã dừng | ACK vs proof trace |
| CTRL04 | P0 | Mất raw-release/home/identity/remote-exit một lần lượt khi Q | Blocker chính xác; local và remote cleanup không giả confirmed | Missing-proof fixtures |
| CTRL05 | P0 | IPC request sai session/nonce/generation/uid/sequence/expiry, A/F/T/I/P hoặc malformed | Từ chối; không tạo quyền/intent ngoài X/Q | Protocol negative results |
| CTRL06 | P0 | Double-click X/Q, reconnect mang event cũ sau epoch đổi | Q idempotent/ưu tiên, không replay hoặc reset deadline | Request/response trace |
| CTRL07 | P0 | UI event loop freeze trong khi backend và process còn sống | Frontend lease hết hạn→backend STOP; heartbeat backend không che UI treo | Fake lease clock |
| CTRL08 | P0 | Frontend process crash/socket mất; modal/input active lúc yêu cầu STOP | Backend shutdown; Esc/nút dừng còn đường ưu tiên, Q/X gõ tên không thành lệnh | Crash/focus tests |
| CTRL09 | P0 | Bật observer app trong fake exact graph, quan sát pump admission | Reader `/pump_cmd` vẫn đúng2, owner/GID/type/QoS/safety fingerprint không bị đổi | Graph before/after |
| CTRL10 | P0 | Mở preview/replay và thử Start/X/Q/tool Pose/Goal trên data giả | Không spawn live/SSH/serial/publisher/action; chỉ fake effects có nhãn | Side-effect deny assertions |

### 3.8. RViz native — G2

| ID | Mức | Bước | Kết quả bắt buộc | Bằng chứng |
| --- | --- | --- | --- | --- |
| RV01 | P1 | Load native host/panel bằng thư viện RViz cài local, không static screenshot | Map render thật, plugin/dock load, cùng Qt5 ABI | Build/runtime library log |
| RV02 | P1 | Resize/split/close/reopen50 lần với camera fake | Không crash/leak context, camera và map cùng một cửa sổ | Video/log + resource trace |
| RV03 | P1 | Fake runner đi qua startup→STOP→startup/recovery | Không spawn RViz/OpenCV thứ2; external view ownership được xác minh | Process/window inventory |
| RV04 | P2 | Hiện Displays/Views/map/scan/TF, chọn camera property | Nhận ra RViz, không phá toolbar/property tree do theme; camera không che map | Screenshot + config |
| RV05 | P1 | Test Pose/Goal trên fixture domain cô lập | Frame/topic/QoS đúng; pose/selected goal không giả action accepted | Topic capture cô lập |
| RV06 | P1 | Ngắt map/TF/scan fake độc lập; thêm TF profile100ms hiện hành | Hiển thị lỗi đúng; không phát lại odom cũ hay đổi TF policy | TF/scan fixture trace |
| RV07 | P1 | Model/path topic không có; goal status có UUID nhưng không pose | Không vẽ model/path/toạ độ bịa; marker điểm chọn phân biệt goal accepted | Negative visualization cases |
| RV08 | P1 | GPU/OpenGL/display plugin lỗi hoặc backend display không hỗ trợ | Báo lỗi và giữ STOP/cleanup; không fallback sang live một phần hoặc ảnh giả gọi là RViz | Render fault trace |

### 3.9. Phục hồi — G1/G2

| ID | Mức | Bước | Kết quả bắt buộc | Bằng chứng |
| --- | --- | --- | --- | --- |
| REC01 | P0 | Serial same-epoch stale rồi mới; sau đó đổi epoch thật | Hiển thị hold/rebind đúng; không tự reset Nav2/Pose/camera, không resend goal/P | Recovery fixtures |
| REC02 | P0 | STOP đang pending, mất WiFi rồi dữ liệu cũ quay lại | Không coi packet muộn là proof fresh; không tự ARM/Start lại | Link/age interleaving |
| REC03 | P1 | App restart có journal cũ và runner vẫn sống/đã chết | Đối chiếu ownership, không auto take-over/kill/duplicate; báo cách xử lý | Restart audit |
| REC04 | P0 | Mutation START/SEAL ACK mất hoặc delayed reply trong startup | Tái kiểm state theo runner, không replay mutation mù | Fake mailbox timeline |
| REC05 | P1 | Child crash/timeout cleanup, sau đó source dữ liệu hồi | Phân biệt fault/hold/degraded; không dùng recovery che lỗi lifecycle | Fault/recovery report |

### 3.10. Tải và đáp ứng — G1/G2

| ID | Mức | Bước | Kết quả bắt buộc | Bằng chứng |
| --- | --- | --- | --- | --- |
| PERF01 | P1 | Feed JPEG/report nhanh hơn render, render chậm có chủ đích | Latest-only queue≤1 frame chờ; tuổi ảnh không tăng vô hạn do backlog; STOP không xếp sau decode | Queue/latency trace |
| PERF02 | P1 | Replay30phút sau warmup, mở/đóng Details, log flood | Bộ nhớ/log bounded, không leak tăng theo thời gian; resource budget chốt ở M0 | RSS/CPU/frame-gap series |
| PERF03 | P1 | So profile cùng dataset trước/sau UI; kích click Stop khi tải cao | Mục tiêu p95 dispatch UI≤100ms trên máy test; không chạm guard350ms do UI; số đo chưa đạt ghi FAIL, không nới safety | Event timestamps + baseline delta |

Mốc100ms là mục tiêu kiểm UI đề xuất, không cam kết thời gian dừng vật lý.
CPU/RAM budget cụ thể phải chốt từ PoC trên máy thật và ghi vào run metadata,
không tự đặt con số PASS sau khi đã thấy kết quả. Không yêu cầu gap sensor=0.

### 3.11. Robot thật — G3, tùy chọn và có phạm vi riêng

| ID | Mức | Bước | Kết quả bắt buộc | Bằng chứng |
| --- | --- | --- | --- | --- |
| LIVE01 | P0 | Trong phiên được cho phép, snapshot endpoints/profile trước/sau observer | Không thêm owner/reader pump, giữ safety/QoS; packet/receipt/model đúng | Read-only graph + manifests |
| LIVE02 | P1 | Chọn910 hoặcTN1→Start→InitialPose→ready→goal ngắn | Một UI map/camera; không paste pairing, không A/F/T/I/P; readiness đúng | App+runner+Pi logs/video |
| LIVE03 | P0 | Scan→terminal teleop→Save→chọn map mới→chuyển fire | File đúng, stop/SLAM release trước Nav2, đúng map/Pose mới | Maps hashes + transition logs |
| LIVE04 | P0 | Trong lượt motion được phép, nhấn X rồi khi ready đặt goal mới | Đúng goal/cycle bị hủy, backend STOP/release, không replay goal cũ | UUID/status/odom logs |
| LIVE05 | P0 | Trong scope được phép, Q/nút STOP/window-close rồi restart app | STOP confirmed hoặc báo UNCONFIRMED thật; local/remote/child cleanup tách rõ | STOP receipt + process audit |
| LIVE06 | P1 | Nếu user cho thử fire/actuator, quan sát một chuỗi đang có policy | UI số mẫu/lượt/ACK6s/home đúng report; không sửa robot để khớp ảnh; lỗi tự xảy ra lưu raw | Actor/ACT/identity/video |

Không cố tình gây mất USB/link/nguồn trong ON để chạy LIVE06. Nếu chưa cho phép
motion/pump thì các case tương ứng **NOT_RUN**, không chuyển thành PASS nhờ replay.

## 4. Thủ tục một lượt QA

1. Ghi trước scope/gate/case IDs và quyền runtime; snapshot git/profile/dataset.
2. Chạy test phù hợp source thay đổi, dừng mở rộng khi có P0/P1; lưu FAIL raw.
3. Đối chiếu output với expected, không chỉ exit code. Phân biệt test fixture
   sai với bug production bằng chạy cùng fixture trên baseline immutable khi cần.
4. Sửa trong scope được yêu cầu, rerun case lỗi và hồi quy liên quan. Rerun
   cùng ID không cộng thêm ca unique; giữ lịch sử FAIL và final outcome.
5. Lập bảng PASS/FAIL/BLOCKED/NOT_RUN và open issues, ghi artifact/source hash.
6. Chỉ qua gate khi bằng chứng đủ; G1/G2 không chứng nhận robot motion/pump.

## 5. Lệnh verify dự kiến sau khi có code

Các lệnh ở mục này **chưa chạy và chưa thể coi là sẵn có hôm nay**. Đường dẫn
suite/build phải được implementation tạo đúng hoặc cập nhật tài liệu trước dùng.

Ví dụ suite fake/parser/session Python nếu được dùng trong app:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q \
  '/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/apps/fire_robot_control/tests' \
  --junitxml=/tmp/fire_robot_control_offline.xml
```

Native Qt/RViz: M0 cung cấp build command và QtTest/CTest suite cụ thể cho cấu
trúc package đã chọn. Offscreen không tự chứng minh OpenGL RViz chạy trên display
thật; RV01–RV08 vẫn cần G2. Không chạy `colcon build` toàn repo để test UI.

Read-only audit trong phiên G3 đã được cho phép, từ shell đã source đúng profile:

```bash
ros2 node list
ros2 topic info /cmd_vel_raw --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic info /pump_cmd --verbose
ros2 topic info /fire_mission/operator_state --verbose
ros2 topic info /yolo/image_annotated/compressed --verbose
ros2 topic info /map --verbose
ros2 topic info /scan --verbose
```

Không dùng `ros2 topic echo/hz/pub /pump_cmd`. Không lấy lệnh publish cmd_vel
trong checklist QA tổng quát để chạy tự động cho task UI. Tốc độ/FPS đo từ
observer đang có khi được phép, tránh thêm reader ảnh/raw qua WiFi vô ích.

## 6. Bộ evidence và mẫu báo cáo

Trong implementation, artifact QA lưu ở output riêng được ignore, không chứa
secret. Tối thiểu:

- `run_metadata`: source/packet/model/map/profile/dependency/display/gate/permission.
- `case_results`: unique ID, nodeid, expected/actual, PASS/FAIL/BLOCKED/NOT_RUN.
- `events`: local-monotonic action→enqueue→ACK→backend report, source session.
- `process_audit`: owned PID/start_ticks/group/child exit, signal order.
- `stop_audit`: operator STOP, local groups, remote receipt và degraded riêng.
- `visuals`: screenshots tại viewport/mode chọn, không dùng ảnh sinh làm evidence.
- `performance`: warmup/window và CPU/RSS/frame age/GUI-dispatch.
- `issues`: severity/file:line, reproduction, impact, fix scope và verification.

Mẫu kết quả: `Case=CTRL03; Gate=G1; Status=FAIL; Expected=chưa báo STOP_CONFIRMED
khi thiếu ACT; Actual=...; Evidence=...; Source=...; Issue=...`.

## 7. Tiêu chí release và điều chưa được gọi PASS

- Không còn P0/P1 thuộc UI integration; có kết quả tất cả case áp dụng hoặc
  lý do NOT_RUN/BLOCKED rõ ràng, không gán PASS cho case không chạy.
- R03 đạt bằng RViz native thật; R09 không cần terminal ngoại trừ mapping teleop;
  R12 giữ STOP proof và crash/lease behavior.
- Runtime baseline USB/TF/Nav2 cleanup còn OPEN phải được báo; không cần sửa
  ngoài scope để đẹp report, không coi app đã nghiệm thu toàn bộ R5.
- Kết quả offline kế thừa chỉ khi source/fixture/contract tương thích và được
  ghi rõ, không dùng số lượng test cũ làm độ phủ cho app mới.
- User có hướng dẫn mở/dừng/log/rollback; map và source của user được bảo tồn.
