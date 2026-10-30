> **Cập nhật 30/10/2026:** Bảng bên dưới mô tả dự kiến ban đầu. Kết quả thực
> tế theo từng ID tại [registry phiên 1](SESSION1_QA_REGISTRY_2026-10-30.md);
> không coi tất cả ca là PASS.

# QA entrypoint app và phục hồi, không thay R5

Ngày: 29/10/2026. Đi kèm
[kế hoạch triển khai](APP_ENTRY_AND_RECOVERY_PLAN_2026-10-29.md).

**Đây là96 ca dự kiến, tất cả NOT_RUN trong lượt lập kế hoạch.** Mỗi ca có
ID duy nhất; fixture tham số hóa có thể phục vụ nhiều ID nhưng không cộng
rerun thành ca mới. Không gọi kiểm cấu trúc Markdown là PASS chức năng app.

## 1. Cách tổ chức bằng chứng

- G0: source/packet/argv/install/lock chỉ đọc, không ROS/hardware.
- G1: fake process/SSH/PTY, fake clock, unit/replay và fault injection.
- G2: Qt/RViz/IPC thật, dữ liệu ROS giả, domain211/localhost1 trong shell con;
  TF diagnostic213/localhost1, preview212. Không đổi domain sản xuất.
- G3: Pi static riêng; sau đó WiFi/robot có giám sát. Không tự chạy phần cứng
  vì thấy ca nằm trong tài liệu.
- Registry mỗi ID: status, mode, setup/action, expected/actual, nodeid/command,
  source/app/parent packet, duration, evidence path/hash và issue nếu FAIL.
- Bằng chứng tối thiểu: ordered event/spawn count, state/proof assertion,
  raw receipt đã loại secret; Qt thêm screenshot/log/graph khi liên quan.
- Raw log lịch sử chỉ là input replay; timestamp cũ không cấp quyền runtime.
  Các assert readiness/STOP phải dùng mẫu/session/boot hiện tại của fixture.
- Không import/init ROS trong ca pure; stub network/device/process mặc định
  fail nếu bị gọi ngoài kịch bản. Mô phỏng pump/teleop chỉ trong domain cô lập.
- Không gây lại USB dropout/rút cáp khi robot đang chạy hoặc cố ý phun để thử
  lỗi đã biết. Lỗi nguy hiểm kiểm bằng fixture; phần cứng cần scope riêng.

P0: sai quyền chạy/dừng, tác động R5 hoặc phần cứng ngoài scope.
P1: khởi động/phục hồi sai hoặc khóa oan. P2: hiển thị/tính dễ dùng.

## 2. G0/G1 — bảo toàn và đóng gói (12 ca)

| ID | Mức | Setup → thao tác | Kết quả bắt buộc |
| --- | --- | --- | --- |
| AER-P01 | P0 | Hash release R5 trước/sau build app | 364 file + manifest giữ nguyên packet0c54; không chỉ so Git tracked |
| AER-P02 | P0 | Hash candidate/install và hai wrapper R5 trước/sau | Source/script/install không bị thay; argv mặc định/selector910,TN1 nguyên |
| AER-P03 | P0 | Fake/readonly Pi stage trước/sau sync app | R5 stage0c54/baseline nguyên; mọi write chỉ vào stage app mới |
| AER-P04 | P0 | Build artifact mới → so364 file parent trong output | R5 scripts/node byte nguyên; adapter phát sinh có tên app/diff/parent pin |
| AER-P05 | P1 | Đổi parent hash hoặc anchor sinh adapter0/2lần | Build fail cụ thể; không sinh partial artifact/current hoặc fallback source khác |
| AER-P06 | P0 | Chạy CLI R5 mặc định PLAN_ONLY trong sandbox không network/device | Lệnh cũ dùng được, không init ROS/SSH; order/argv như baseline |
| AER-P07 | P0 | Fake common lock đang do R5 giữ → mở app/nhấn Start | App IDLE không ảnh hưởng R5; Start từ chối, không signal/STOP/kill R5 |
| AER-P08 | P0 | Phiên app giữ lock → thử mở CLI R5 bằng backend giả | CLI từ chối như lock hiện có; không có hai I/O/control owner |
| AER-P09 | P1 | Export/verify archive app; sửa file, duplicate, symlink, traversal | Strict reject; thành phần ngoài manifest không lọt; r5_packet.py gốc không sửa |
| AER-P10 | P1 | Candidate/binary/archive/receipt Pi sai hoặc thiếu | Từ chối trước START; UI nêu mismatch; không lấy receipt packet cũ thay |
| AER-P11 | P1 | Build hoặc Pi sync lỗi giữa chừng | Không đổi current, không ghi receipt PASS, không đè artifact/stage cũ |
| AER-P12 | P1 | Rollback khi idle và khi còn phiên/cleanup | Idle chọn được packet khớp; phiên chưa sạch bị từ chối; journal/log giữ nguyên |

## 3. G1/G2 — startup và thứ tự effect (14 ca)

| ID | Mức | Setup → thao tác | Kết quả bắt buộc |
| --- | --- | --- | --- |
| AER-S01 | P0 | Mở app bằng launcher mới | Chỉ Qt shell/IPC/catalog; không Pi START/ROS control/RViz reader sản xuất |
| AER-S02 | P1 | Fire map hợp lệ, dữ liệu lần lượt đạt | Effect order giống R5: clock→I/O→camera→seal→Nav/RViz→Pose→YOLO→consumers |
| AER-S03 | P1 | Quét map dữ liệu lần lượt đạt | Seal trước SLAM/view/teleop; không Nav2/mission/automatic cùng mapping |
| AER-S04 | P0 | Chưa sealed nhưng đã có map/TF/camera hoặc process ready line | Không mở Nav/goal/arm; snapshot khác session không cấp quyền |
| AER-S05 | P1 | TF/camera chậm rồi hồi trong deadline | Hiện chờ/phục hồi; qua gate bằng proof mới; không BLOCKED trước hạn |
| AER-S06 | P1 | Readiness true/false lặp sát deadline | Reset stable timer đúng, không gia hạn deadline tổng hoặc giữ cached admission |
| AER-S07 | P0 | Seal=true nhưng health.io=false ở STATE mới | Vẫn chờ; không chỉ dùng cờ sealed đã từng đúng để qua gate |
| AER-S08 | P1 | Thiếu Initial Pose hoặc pose chưa ổn định | Hướng dẫn đặt Pose, giữ gate5s/identity; không khởi động YOLO/mission sớm |
| AER-S09 | P1 | YOLO/consumers không sẵn sàng hoặc child chết | Đúng stage/deadline/lỗi gốc; thu dọn phần đã start theo ownership |
| AER-S10 | P0 | Camera timeout → RETRY_PREARM hợp lệ | Tối đa1retry; old health/GID được retire; không retry sau seal/arm |
| AER-S11 | P0 | Mất ACK của START/RETRY/SEAL | Đối chiếu STATE cùng binding; không replay mutation chưa chứng minh kết quả |
| AER-S12 | P1 | Pi DONE/SSH thoát sau startup error | UI giữ lỗi seal/readiness đầu tiên và lỗi transport phụ, không thay bằng lỗi chung |
| AER-S13 | P1 | Đo mapping STATE chậm/nhanh bằng fake clock/SSH | Một RPC tại một thời điểm, nhịp quy định, không storm hoặc timer reset theo response |
| AER-S14 | P0 | Serial epoch/bridge/boot đổi lúc startup | Cờ ổn định/provenance reset hoặc từ chối đúng; không ghép dữ liệu hai identity |

## 4. G1/G2 — TF, observer và tải (10 ca)

| ID | Mức | Setup → thao tác | Kết quả bắt buộc |
| --- | --- | --- | --- |
| AER-T01 | P0 | So checker/class và profile app với R5 | Byte class nguyên; TF300ms, lead100ms, source/progress/graph checks giữ |
| AER-T02 | P1 | Fixture TF/odom bình thường, mode R5 và mode app sampled | Cả hai đủ mẫu/không drop trace; source/callback/lookup đo riêng |
| AER-T03 | P1 | Inject25ms sample cost, nguồn/listener đối chứng vẫn mới | Tái hiện control lag; mode app không tạo backlog4–5s; không fake timestamp |
| AER-T04 | P0 | Cả nguồn TF thực sự stale nhưng callback nhận mới | Admission vẫn false; receive time mới không hợp lệ hóa source stamp cũ |
| AER-T05 | P0 | Mất odom, TF đứng stamp, sai frame/profile hoặc tương lai ngoài tolerance | Từ chối đúng từng lỗi, không republish dữ liệu cũ thành nguồn mới |
| AER-T06 | P0 | Snapshot quá750ms, NaN/time đảo hoặc khác session | Reject/fail closed; không fallback cached proof |
| AER-T07 | P1 | Nhịp sample5Hz, callbacks xen kẽ và stable1s/5s | Timer theo monotonic/cùng identity; không đạt sớm vì giảm tần suất |
| AER-T08 | P1 | Qt IDLE/SEAL→gắn view→STOP→tháo view; inspect graph thật | Không UI reader TF/map/scan/camera trước gate/sau teardown; backend health/STOP readers ghi riêng |
| AER-T09 | P0 | Inspect graph toàn pipeline, handoff verifier/operator | Không thêm pump reader, publisher raw/arm hoặc hai operator; old generation reap trước mới |
| AER-T10 | P1 | Callback/ghi JSON chậm/lỗi + STOP khi callback đang xử lý | UI vẫn nhận STOP, tiến trình own được thu dọn; output lỗi không sinh READY giả |

## 5. G1/G2 — STOP, cleanup và phục hồi (16 ca)

| ID | Mức | Setup → thao tác | Kết quả bắt buộc |
| --- | --- | --- | --- |
| AER-R01 | P1 | Replay fire21:59:26: rc2 nhưng STOP/local/remote proof sạch | Giữ startup FAIL, cleanup CLEAN; audit đạt cho IDLE, không khóa chỉ vì rc2 |
| AER-R02 | P0 | Replay mapping22:01:56 thiếu STOP | STOP_UNCONFIRMED; không mở phiên mới, không archive bằng process exit |
| AER-R03 | P1 | Fake Pi seal timeout và laptop drain phối hợp đúng | Pi app giữ telemetry trong cửa sổ, nhận STOP sau local drain; proof mới đạt |
| AER-R04 | P0 | Pi seal timeout + mất laptop/hết cửa sổ drain | Admission rút, watchdog giữ, Pi cleanup hữu hạn; không mở lại hoặc ghi STOP PASS giả |
| AER-R05 | P0 | STOP khi startup ở từng phase clock/pairing/I/O/camera/seal/Pose | Một chuỗi dừng đúng nguồn đã start; không hiệu ứng START muộn sau STOP |
| AER-R06 | P0 | STOP khi mapping hoặc fire đang hoạt động bằng fixture | Rút intent→zero/OFF/home/owner release→child drain→Pi STOP, không replay pulse/goal |
| AER-R07 | P0 | Chỉ có process exit0/ready line/cleanup sạch, không proof telemetry | Không suy ra robot đã dừng; giữ STOP_UNCONFIRMED |
| AER-R08 | P0 | STOP đạt nhưng child crash/force/live group hoặc thiếu child log | CLEANUP_REQUIRED, không coi là retry-ready |
| AER-R09 | P1 | Startup error còn trong Pi DONE nhưng group/child audit sạch | Tách error khỏi cleanup ở adapter; preserve DONE nguyên; không nới audit lỗi child |
| AER-R10 | P0 | Handoff STOP-only monitor sang operator chính; STOP tới muộn | Reap/nhả quyền trước operator mới, không nhận old receipt/cancel nhầm phiên mới |
| AER-R11 | P1 | RECHECK double-click, kết thúc nhanh/chậm/fail rồi thử lần nữa | Một audit, attempt/result giữ trên UI; không chạy xe; nút dùng lại sau hoàn tất |
| AER-R12 | P0 | Audit xong nhưng marker/binding/PID start_ticks/boot đổi trước archive | Reject stale result; giữ pending; không kill/reuse PID cũ |
| AER-R13 | P1 | New session sau STOP/recovery cũ | Không hiện “đã xác nhận dừng” của phiên cũ; log lịch sử còn giữ |
| AER-R14 | P0 | User chọn VERIFY_STOP khi phiên cũ thiếu proof, fake I/O đúng identity | Chỉ STOP-only, proof mới + cleanup mới; archive liên kết hai phiên, không sửa kết quả cũ |
| AER-R15 | P0 | VERIFY_STOP sai identity/boot, lock bận, dữ liệu thiếu hoặc user hủy | Từ chối/STOP đúng scope; pending cũ không mất, không Nav/teleop/arm/ON |
| AER-R16 | P1 | Thiếu quyền/ENOSPC khi ghi/xóa/archive receipt/pending | Không kẹt dead worker; STORAGE_ERROR rõ; cleanup vẫn được thực hiện, không unlock giả |

## 6. G1/G2 — thao tác và hiển thị (12 ca)

| ID | Mức | Setup → thao tác | Kết quả bắt buộc |
| --- | --- | --- | --- |
| AER-U01 | P2 | Click Scan/Fire/Check/Save/STOP nhanh và giữ chuột qua STATUS | Có pressed/busy/ACK rõ; click không mất do refresh enable/disable |
| AER-U02 | P1 | Giữ popup map qua nhiều STATUS rồi chọn bằng chuột/bàn phím | Popup không reset; selection hợp lệ giữ đến Start |
| AER-U03 | P1 | Double-click/triple-click Scan/Fire hoặc nhấn Enter liên tục | Chỉ1session/worker; reject rõ, không im lặng hoặc spawn chồng |
| AER-U04 | P0 | Nút/thao tác ở tất cả state, kể cả STARTING/STOPPING/CHECKING | Save/Start/Scan/Goal/Pose chỉ hợp lệ đúng phase; STOP ưu tiên |
| AER-U05 | P1 | Chờ TF/camera/clock/SSH gần deadline | Hiện nguyên nhân, elapsed/remaining và bước tiếp theo, không “treo không biết đã ăn” |
| AER-U06 | P1 | Khởi động lỗi, STOP đạt/không đạt, cleanup đạt/không đạt | Ba kết quả phân biệt; nguyên nhân gốc còn thấy; có log/details |
| AER-U07 | P0 | Mất IPC/UI heartbeat, rồi status/NEW_SESSION cũ đến muộn | Command tools vẫn khóa; rút intent/STOP; dữ liệu muộn không mở lại quyền |
| AER-U08 | P0 | Gõ tên map có X/Q; Esc/STOP khi có dialog | Không phát X/Q vì đang nhập text; có đường STOP rõ, modal không khóa dừng |
| AER-U09 | P1 | Close khi đang auth/check/save/start/drain; close nhiều lần | Hủy/chờ đúng ownership, báo đang dọn; không đóng PTY trước STOP hoặc tạo worker mới |
| AER-U10 | P2 |1366×768,1920×1080; scale100/125/150%; resize splitter | Camera lớn đúng aspect, map/status/nút dừng đọc được, màu trắng/đỏ/xám giữ |
| AER-U11 | P1 | Camera ngừng, khung cũ và text lỗi dài | Hiện stale/chờ mới đúng, không giữ ảnh cũ như live hoặc che nút STOP |
| AER-U12 | P0 | Chuỗi200event giả theo seed7/19/29 + event đảo/thừa | Assert mọi bước: ≤1owner, STOP không đảo, no spawn khi blocked, không replay quyền |

## 7. G1/G2 — map và chuyển chế độ (10 ca)

| ID | Mức | Setup → thao tác | Kết quả bắt buộc |
| --- | --- | --- | --- |
| AER-M01 | P1 | IDLE chưa quét, có nhiều map cũ → chọn Start | Chọn được map hợp lệ; Nav nhận snapshot đúng file/hash đã chọn |
| AER-M02 | P1 | Quét→lưu map có tên riêng→catalog cập nhật | Cặp YAML/PGM đủ, tự chọn map vừa lưu; vẫn chọn lại map cũ được |
| AER-M03 | P1 | Tên map rỗng/quá64/chứa slash/../ký tự ngoài allowlist | Reject trước write/spawn, chỉ dẫn tên hợp lệ |
| AER-M04 | P1 | Tên trùng hoặc symlink/partial cặp YAML/PGM | Không ghi đè hoặc đi ra root; báo lỗi, map cũ nguyên |
| AER-M05 | P1 | Map đã chọn bị sửa/xóa trước Start hoặc khi snapshot | Reject thay đổi, không chạy nhầm default map910 |
| AER-M06 | P1 | Bấm Save liên tục hoặc save đang chạy rồi STOP | Một transaction; hoàn tất/cancel có kết quả, không để catalog nhận cặp nửa vời |
| AER-M07 | P1 | Map saver timeout/fail/ENOSPC, mất map publisher | Báo lỗi save, cleanup child đúng; không nhận map giả hoặc mất map cũ |
| AER-M08 | P0 | Mapping→Fire và Fire→Mapping khi phiên cũ chưa sạch | Không chạy chồng; dừng/kiểm chứng trước, giữ lựa chọn user |
| AER-M09 | P0 | Teleop mất focus/keyboard channel đóng khi quét | Không kẹt vận tốc, watchdog/STOP hiện có hoạt động; không tự gửi lại phím khi nối lại |
| AER-M10 | P1 | Map mới/cũ path có dấu cách; run app từ cwd khác | Resolve/snapshot đúng, không shell interpolation hay phụ thuộc cwd |

## 8. G1/G2 — mạng, process và độ bền (12 ca)

| ID | Mức | Setup → thao tác | Kết quả bắt buộc |
| --- | --- | --- | --- |
| AER-N01 | P1 | Sai WiFi/route, còn LAN khi live profile yêu cầu rút | Preflight giải thích cụ thể, không tự đổi mạng và chưa mở I/O |
| AER-N02 | P1 | Pi không trả lời, SSH auth sai/hostkey khác, user hủy nhập | Timeout/cancel rõ, không bỏ host-key check hoặc lộ mật khẩu/raw SSH |
| AER-N03 | P0 | Pairing split chunks/duplicate/stale/path sai/session/packet sai | Chỉ nhận một pairing hợp lệ đúng Pi đang own; không tái sử dụng file cũ |
| AER-N04 | P1 | SSH STATE chậm/timeout trong lúc user bấm STOP | Không khóa Qt; rút intent ngay, retry read có giới hạn, không replay mutation |
| AER-N05 | P0 | Mất WiFi lúc chưa START, lúc startup và lúc RUNNING (fake) | Phân loại chưa chạy vs cần STOP đúng; không tự restart/arm/goal khi mạng trở lại |
| AER-N06 | P0 | GUI crash/freeze, supervisor vẫn sống | Lease hết → STOP; supervisor giữ Pi PTY và sở hữu cleanup |
| AER-N07 | P0 | Supervisor/worker chết giữa phiên, mở lại app | Journal phục hồi, không tự start; thiếu proof giữ khóa và chỉ dẫn xác minh |
| AER-N08 | P0 | Child sinh grandchild, leader thoát nhưng group còn sống | Không nhận leader exit làm group exit; giữ ownership, không kill unrelated process |
| AER-N09 | P0 | Snapshot process lỗi/không đủ quyền hoặc PID tái sử dụng | Fail closed với lý do cụ thể; không chạm process từ session khác |
| AER-N10 | P1 | UI/network command queue đầy, output flood/JSON sai/NaN | Bound size/rate, ưu tiên STOP, không RAM tăng vô hạn hoặc lộ secret |
| AER-N11 | P1 | Mất Internet nhưng còn WiFi Pi; mở launcher/assets/help offline | Hoạt động không CDN/pip/model download; hướng dẫn offline truy cập được |
| AER-N12 | P1 |10vòng fake start→stop→retry xen hủy/close | Không tăng FD/thread/process tồn, không stale socket/reader/label hoặc deadlock |

## 9. G3 — tích hợp Pi và nghiệm thu (10 ca)

Không auto chạy nhóm này. Điều kiện: user giám sát, robot/actuator ở trạng
thái phù hợp, lock rảnh, packet và installed source đúng. Motion/phun không
mặc định được phép. Các ca lỗi nguy hiểm đã được fault-inject ở G1/G2.

| ID | Mức | Setup → thao tác | Kết quả bắt buộc |
| --- | --- | --- | --- |
| AER-H01 | P0 | LAN, chỉ sync/build/static check stage app mới | Receipt app đúng source/binary/profile; hash R5 Pi/baseline nguyên; không runtime |
| AER-H02 | P1 | WiFi Pi, Internet tắt; mở app tại IDLE | Một cửa sổ đúng layout, không tự I/O/chạy xe; không UI DDS readers trước gate |
| AER-H03 | P0 | Nếu pending cũ thiếu STOP: user chọn xác minh dừng riêng | Proof mới thật và cleanup sạch; giữ lịch sử cũ, không mission/goal/ON |
| AER-H04 | P1 | User Scan→lái theo workflow được phép→Save tên mới | SLAM/map/camera đúng; lưu file đủ và chọn được; teleop đi luồng watchdog cũ |
| AER-H05 | P1 | Chọn map cũ rồi map mới, dừng/chuyển sang Fire | Không chạy chồng; Nav nhận đúng map; Initial Pose/tool hoạt động đúng phase |
| AER-H06 | P1 | Fire→Pose→quan sát sẵn sàng, không cần goal/phun | Đúng R5 order/gates, camera/YOLO/status; log readiness đầy đủ |
| AER-H07 | P0 | User STOP lúc startup, sau đó audit rồi Scan/Fire mới | Zero/OFF/home và cleanup có proof; retry mới thật, không cached proof |
| AER-H08 | P0 | User X/Q/đóng cửa sổ ở trạng thái phù hợp có giám sát | X đúng phạm vi task, Q/close dừng phiên; không tắt I/O trước verifier |
| AER-H09 | P1 | Capture Pi có tải trong startup; so mode observer theo runbook | Nguồn/callback/lookup/sample age/CPU/reader được đo; chưa đủ mẫu thì NOT_RUN/INCONCLUSIVE, không PASS giả |
| AER-H10 | P0 | Chạy R5 CLI cũ và app tuần tự, cùng điều kiện hợp lý | CLI wrapper/argv/hash vẫn nguyên và dùng được; mọi khác biệt startup/cleanup được ghi, không gán lỗi R5 thành lỗi mới đã sửa |

## 10. Lệnh verify dùng trong phiên code sau

Các lệnh dưới là **kế hoạch chạy sau implement**, không đã chạy ở lượt viết
tài liệu. Review diff/path allowlist trước. Không chạy full colcon workspace
hoặc flash chỉ để kiểm một thay đổi app.

### Chỉ đọc hash R5

Từ root repo, lệnh hiện đã tồn tại và không khởi động ROS/SSH:

~~~bash
python3 -B releases/r5_20261029/ros/scripts/r5_packet.py
python3 -B .codex_qa_backups/r5_operational_candidate_20261026/ros/scripts/r5_packet.py
git status --short
~~~

Snapshot trước/sau còn phải bao gồm wrapper/install/module và Pi khi có kết
nối; hai dòng trên không thay cho toàn bộ AER-P01/P02/P03.

### Build và QA app hiện có, mở rộng bằng suite mới

Chạy trong subshell để không đổi môi trường terminal sản xuất. Các test ROS/
Qt cần display/Xvfb phù hợp, quyền socket và underlay hiện hành; lỗi môi
trường phải ghi BLOCKED_ENV, không sửa domain/QoS sản xuất để ép PASS.

~~~bash
(
  cd '/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS'
  source /opt/ros/humble/setup.bash
  source /home/huudinh/.codex/worktrees/laptop-baseline/NCHK_robotFirefighting_ROS/ros2_ws/install/local_setup.bash
  source .codex_qa_backups/r5_operational_candidate_20261026/ros/ros2_ws/install/local_setup.bash
  export PYTHONDONTWRITEBYTECODE=1
  export ROS_DOMAIN_ID=211
  export ROS_LOCALHOST_ONLY=1
  export PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/releases/r5_20261029/ros/scripts:$PWD/releases/r5_20261029/ros/ros2_ws/src/fire_robot_bringup"
  python3 -B -m unittest discover -s apps/fire_robot_control/tests -p 'test_*.py'
  bash apps/fire_robot_control/launchers/build_preview.sh
  cmake -S apps/fire_robot_control/tests/pre_sync_ui_review -B /tmp/fire_robot_pre_sync_ui -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
  cmake --build /tmp/fire_robot_pre_sync_ui --parallel 2
  ctest --test-dir "/tmp/fire_robot_control_preview_$(id -u)" --output-on-failure
  python3 -B -m pytest -q -p no:cacheprovider apps/fire_robot_control/tests/qa_pre_sync_ui.py apps/fire_robot_control/tests/qa_map_selection.py apps/fire_robot_control/tests/qa_button_feedback.py apps/fire_robot_control/tests/qa_supervisor_qt.py apps/fire_robot_control/tests/qa_supervisor_gui.py apps/fire_robot_control/tests/qa_candidate_runner.py apps/fire_robot_control/tests/qa_candidate_smoke.py apps/fire_robot_control/tests/qa_full_operator.py
)
~~~

Đây là tên suite đang có khi lập kế hoạch. Bổ sung test mới cho lifecycle/
outcome/health/VERIFY_STOP, cập nhật probe nếu host RViz đã refactor. Không
giữ một test cũ vô nghĩa chỉ vì nó PASS trước đó; cũng không sửa expected
nhằm che mất hành vi an toàn. Thêm artifact build/verify dùng package API
không cập nhật current tự động; prepare_candidate.sh hiện đổi current nên
phải tách việc này trong phiên1 trước khi sử dụng làm release.

TF probe có lệnh/mode/domain guard riêng trong
[README diagnostic](../diagnostics/tf_startup/README.md). Không chạy đồng thời
với R5/robot; không lấy fixture nhẹ hoặc25ms giả làm nghiệm thu tải WiFi thật.

Sync Pi, mở candidate live, xác minh dừng và CLI R5 live không nằm trong các
lệnh tự chạy trên. Chuẩn bị runbook theo packet mới sau G1/G2, không dùng
placeholder hiện tại để chạy phần cứng.

## 11. Điều kiện ghi PASS và dừng phát hành

- Lỗi đã xác nhận: có replay FAIL trước sửa → PASS sau sửa, kèm negative
  case vẫn bị chặn đúng. Kịch bản mới chưa có source thì không giả FAIL cũ.
- Mọi trạng thái/action có coverage; mọi nhánh không được phép có assertion
  không-spawn/không-publish, không chỉ assertion có dòng text trên UI.
- Receipt rc2 do startup vẫn FAIL startup; STOP/cleanup là verdict riêng.
  Unconfirmed/unknown/force thiếu audit không được tự chuyển thành CLEAN.
- Phải kiểm artifact thực và process/graph thật ở G2, không chỉ FSM mô phỏng.
- Test G3 chưa chạy giữ NOT_RUN, kết quả observer mơ hồ giữ INCONCLUSIVE;
  không gộp vào PASS. Kế thừa parent ghi INHERITED với lý do và source pin.
- Không còn P0/P1 thuộc phần app trong scope trước chọn current. Known issue
  R5/USB/firmware ngoài scope giữ OPEN, không tự sửa để đạt deadline2phiên.
- Báo cáo cuối: số unique thực tế, fail/open/not-run, giới hạn hardware,
  manifest/evidence/SHA256SUMS, lệnh app và R5 cũ, thao tác rollback.
