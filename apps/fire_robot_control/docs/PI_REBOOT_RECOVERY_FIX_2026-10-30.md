> **Mới nhất — 857b9eb3 đã sửa relay xác thực, sync và mở:** 211 unittest /
> 160 integration / CTest đạt; hộp Qt accept/cancel đi qua worker-supervisor thật.
> Lần xác minh user trước lỗi NTP refresh 100 s, chưa I/O. R5/pending nguyên;
> bản mới chờ user bấm xác minh và nhập mật khẩu Pi khi hỏi.
> [Báo cáo/giới hạn](AUTH_RELAY_FIX_2026-10-30.md).

# Sửa phục hồi sau Pi reboot — 30/10/2026

**Đã sửa, QA laptop, đồng bộ Pi và mở app mới.** Kiểm tra chỉ đọc với Pi
thật đã qua nhánh boot mới; chưa chạy STOP-only/I/O hoặc nghiệm thu dừng vật
lý. App hiện mở BLOCKED đúng hồ sơ cũ, chờ user chọn Xác minh dừng trên robot.
Không tự xóa pending hoặc coi reboot là xác nhận STOP.

Packet: `a387d27e0453acc1a46ac2f19bd1481a437e472e3631a5ed771a99342f507a55`.
Parent R5 vẫn 0c54, model best_29_10_2026.pt. Current đổi từ 59cddc99 sang a387;
R5 CLI, các stage R5/app cũ và node/firmware không đổi.
[Chẩn đoán trước sửa](RECOVERY_AFTER_PI_REBOOT_DIAG_2026-10-30.md).

## Hành vi mới

1. **Kiểm tra lại vẫn chỉ đọc:** hồ sơ thiếu STOP vẫn báo thiếu STOP, không
   tự mở phiên robot hoặc tự archive. Muốn lấy proof mới phải bấm riêng
   **Xác minh dừng trên robot**.
2. Chuẩn bị STOP-only kiểm laptop groups đã thoát, PID không tái sử dụng,
   common lock rảnh và hồ sơ không bị đổi. SSH chỉ tới route Pi đã allowlist,
   giữ StrictHostKeyChecking và deadline 18s.
3. Ở Pi, xác minh binding/pairing cũ đúng session/packet/root/PID/startticks,
   đường dẫn không symlink và file thuộc user Pi. Nếu cùng boot thì vẫn yêu
   cầu DONE/cleanup và PID cũ vắng theo checker hiện có.
4. Nếu boot khác: không đòi boot cũ quay lại. Kiểm common lock Pi rảnh,
   không runner/node I/O hoặc điều khiển còn sống, boot ổn định trong lần
   kiểm. Quét thêm cả lidar/robot_state_publisher/rosbridge/micro-ROS/usb_cam/
   bringup để không bỏ sót child I/O. Kết quả chỉ là
   `NEW_BOOT_QUIESCENT_ONLY`, chưa có bất kỳ STOP success nào.
5. Mỗi roundtrip gắn nonce mới và kiểm binding/current boot/flags/hash; reply
   sai nonce, sai boot dự kiến hoặc sai dạng bị từ chối.
6. Worker pin boot vừa kiểm vào recovery_boot_id. Nếu pairing mới đổi boot
   trước bước START, từ chối trước khi mở runner laptop. Phiên STOP-only mới
   vẫn dùng operator/telemetry và cleanup gốc; không thêm mission/goal/ON.
7. Chỉ sau STOP và cleanup mới đạt, audit yêu cầu profile
   DESKTOP_STOP_RECOVERY, cùng session/packet mới, marker cũ nguyên, boot mới
   bằng boot đã pin và session Pi mới khác session Pi cũ. Kiểm DONE/cleanup
   mới bằng remote_proof nghiêm ngặt; kiểm lại trạng thái Pi ở cùng boot đó.
   Sau cùng mới archive pending kèm liên kết proof mới. Lịch sử FAIL cũ nguyên.
8. Nếu Pi reboot thêm, mạng lỗi, lock bận, PID còn sống hoặc STOP chưa đủ,
   giữ BLOCKED. Lỗi RECOVERY có dạng mã được giữ và diễn giải trên UI;
   ưu tiên nguyên nhân bên trong thay vì che bằng ValueError:stage. Raw SSH,
   password/pairing secret không được đưa vào thông báo.

Không sửa `recovery.audit()` hay `probe_pi()` để cho read-only tự mở khóa.
Không sửa watchdog, TF/QoS/serial, ngưỡng seal/freshness hoặc logic motor/bơm.
Các mismatch lịch sử trong AGENTS giữ nguyên ngoài scope.

## File vận hành thay đổi

| File | Mục đích |
| --- | --- |
| services/recovery_probe.py | Thêm probe idle cho STOP-only; cùng boot dùng probe cũ, khác boot kiểm quiescence dưới lock; mở rộng pattern chỉ cho nhánh này. |
| services/recovery.py | SSH probe idle riêng, challenge/binding/boot validation và diễn giải nested error. |
| services/stop_recovery_runner.py | Prepare trả marker+Pi proof; audit STOP mới chặt binding/boot/session, không dùng proof boot cũ để thay STOP mới. |
| services/live_session.py | Giữ boot pin trong worker/receipt; từ chối đổi boot trước START; giữ nguyên mã RECOVERY trong UI/error. |

Tests: thêm test_reboot_recovery.py, cập nhật fixture test_stop_recovery.py
để có boot và binding đầy đủ. Không sửa test/node/script R5. Diff trước/sau
ở evidence, không bỏ qua thay đổi dirty của user.

## QA và giới hạn

| Bộ | Kết quả | Phạm vi |
| --- | --- | --- |
| Hai ca tái hiện ban đầu | 2 FAIL→PASS | Prepare bị boot-change chặn dù chỉ cần proof idle mới; nested error che boot cause. |
| Unittest app | 205 PASS, 6,277 s | Bao gồm 15 method mới về reboot/probe/transport/audit/worker; 190 ca cũ hồi quy. Subcase không cộng thành testcase riêng. |
| Tích hợp | 158 PASS, 54,29 s | Qt/IPC thật, RViz localhost211 với dữ liệu giả, map/nút/worker/runner/faults, launcher và binary artifact mới. |
| Native CTest | 1/1 PASS | CTest frontend riêng và trong lúc đóng gói cùng đạt. Không cộng lần chạy lặp thành case mới. |
| Pi prepare thật chỉ đọc |PASS có phạm vi | Old boot df14ad0d…→current boot a4b08c52…; Pi rảnh, lock trống, marker giữ nguyên. Không chạy I/O/STOP. |
| Static sync thật |PASS | 426 source + 103 installed Pi và 95 installed laptop; source/binary/archive/receipt đúng packet. |
| STOP telemetry và dừng vật lý sau reboot |NOT_RUN | Cần user giám sát và bấm thao tác riêng; chưa mở I/O, OFF/home hoặc chạy xe/bơm trong lượt này. |

15 test method mới được liệt kê trong delta_reboot.xml, cũng nằm trong 205
unittest; không cộng 15 vào 205 lần nữa. Các nhánh gồm: cùng boot giữ cleanup
và PID reuse check; boot mới không đọc DONE cũ/không tạo STOP success; sai
bảy trường pairing; robot process/lock bận; boot đổi giữa probe, sau proof
STOP hoặc trước START; reply sai challenge/binding/mode/hash/flags; thiếu
STOP/cleanup/boot pin/marker; dùng lại session Pi cũ; archive giữ FAIL cũ;
worker thật bảo toàn mã lỗi và không mở phần điều khiển khi boot đổi.

Trước lượt cuối, một lệnh unittest chưa source ROS và chưa có socket sandbox
cho phép nên lỗi môi trường; lượt tiếp theo có ROS vẫn thiếu socket. Đã chạy
lại đúng ROS+quyền socket cục bộ và 205/205 đạt. Log lỗi lệnh đầu giữ riêng,
không đổi source để ép môi trường PASS. Cảnh báo xacro/Python 3.11 trong bài
RViz không làm test fail và chưa được coi là vấn đề runtime đã xử lý.

## Đồng bộ, bảo toàn và app mới

User đã cho phép cập nhật app và đồng bộ trong chuỗi công việc này. Lượt mới
sync tĩnh qua WiFi Pi 10.0.0.1 đúng route wlp8s0/src 10.0.0.253; không đổi mạng.
Stage: `/home/pi/fire_robot_desktop/a387d27e0453/ros`. Build 2 package trong 8,98 s;
receipt status STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED/errors=[]/live=false.

Đọc lại độc lập 426 source và 103 file cài Pi khớp. Receipt Pi gốc, bản tải và
bản app lưu cùng SHA256
`a600c780577f2ed26170b6b64a02ff9f65a089f2ba5be55951c4403ce44e0547`.
Log sync gốc: `/home/huudinh/.local/state/fire_robot_control/sync_logs/sync_uMoGiM`.
Receipt app:
`/home/huudinh/.local/state/fire_robot_control/pi_receipts/a387d27e0453acc1a46ac2f19bd1481a437e472e3631a5ed771a99342f507a55.json`.
Receipt trong artifact vẫn ghi Pi PENDING của thời điểm build; file niêm phong
đó giữ nguyên, receipt cài thực tế nằm riêng.

- Pi: 21.684 file của 46 bộ cũ/baseline giữ SHA và resolved target. Baseline Git
  sạch; các stage 0c54/2484/dfac nguyên. Không chạy R5 CLI hôm nay để chứng
  minh runtime, không tự flash hoặc mở robot.
- Laptop: 1955 file bảo vệ nguyên trước activation. Sau activation chỉ
  current.txt thay có chủ đích; 1954 file khác và pending giữ nguyên. Parent
  R5 có 364 file trong gói app mới giữ nguyên byte, manifest app 426 pins.
- Binary SHA: `10b4bf45dbdc4efbad9a437ef5f82ef5eb200171afb63f657f4c6a2cccdcd3f6`.
- Archive SHA: `61580e3d9d51c9988b0668bfef4362ae50fe3271d3095e812c4b883941fe21c7`.
- Chỉ đóng đúng supervisor 273082 có log cuối BLOCKED, giữ common lock laptop
  trong lúc đổi. Chọn current mới và mở supervisor 281326. Log từ đúng PID/
  wrapper mới duy nhất: `/tmp/fire-ui-fuq3c3m4/gui.log`; BLOCKED/attempt0,
  chưa gắn RViz reader hoặc gửi RECHECK/VERIFY_STOP. Không chọn log cũ theo
  vòng duyệt mtime để tránh nhầm đường log như lần trước.
- Pending SHA vẫn `a13a087426d353a234743217da9bbd7fff1857c8b78ce353db5c83453f0ce676`.
  Không xóa/archive thủ công. App mở BLOCKED không phải chứng cứ fix thất bại;
  phải có xác minh mới hợp lệ để mở phiên.

## Người dùng thử tiếp

Giữ WiFi Pi và không chạy CLI R5 đồng thời. Khi ở cạnh robot đã dừng và sẵn
sàng cho STOP/OFF/home, bấm **Xác minh dừng trên robot** một lần; servo có thể
về home. Nhập SSH/sudo nếu app hỏi, giữ cửa sổ/kết nối tới khi có kết quả.
Bấm Kiểm tra lại đơn thuần vẫn không tạo STOP proof mới. Chỉ khi app về IDLE
và đã đối chiếu robot dừng mới thử Quét map.

Chưa nghiệm thu hành vi phần cứng, startup TF/USB/Nav2 thực tế hoặc STOP
physical của nhánh mới. Không tự coi kết quả read-only prepare là STOP PASS.
[Hướng dẫn offline](../HUONG_DAN_QUET_MAP_OFFLINE.txt) đã cập nhật để dùng khi
WiFi Pi không có Internet.

[Evidence](qa_evidence/pi_reboot_fix_20261030/) có baseline 2 FAIL, delta XML,
205 unit log, 158 pytest XML/log, CTest, diff/pins, Pi readonly prepare, hash
trước/sau nén, receipt và activation. SHA256SUMS pin các file; chạy kiểm
checksum từ thư mục evidence. Lệnh hồi quy trong run_release_qa.sh cần ROS
overlay, probe Qt/RViz đã build và socket local; không chạy đồng thời với
phiên robot dùng domain test. Không rerun script activation như bước QA.
