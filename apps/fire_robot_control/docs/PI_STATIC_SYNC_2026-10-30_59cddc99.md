> **Sau thao tác user:** RECHECK đã nhận 6 lần nhưng thiếu STOP cũ; user thử
> Xác minh dừng cũng bị chặn bởi Pi đã reboot. Đã tái kiểm chỉ đọc ra lỗi
> RECOVERY_PI_BOOT_CHANGED; chưa I/O, chưa sửa code/xóa pending.
> [Chẩn đoán và đính chính log](RECOVERY_AFTER_PI_REBOOT_DIAG_2026-10-30.md).

# Đồng bộ tĩnh Pi và chọn ứng viên app 59cddc99 — 30/10/2026

**Đồng bộ qua LAN đạt.** App mới được chọn cho lần mở tiếp theo và đã mở ở
BLOCKED với hồ sơ phiên cũ giữ nguyên. Đây là ứng viên để nghiệm thu có giám
sát, **chưa nghiệm thu WiFi/robot, chưa chạy quét hoặc chữa cháy**.

Packet: `59cddc9969669e937e0fd1c98a3982156b6e7e7096204787522895dabc857a38`.
Thay đổi app/QA laptop được ghi trong
[báo cáo sửa ba lỗi và xe/đường](FIX_AND_ROBOT_PATH_QA_2026-10-30.md).
Lượt này không sửa code vận hành, R5 hoặc firmware.

## Kết quả thực tế

| Kiểm tra | Kết quả |
| --- | --- |
| LAN | Laptop enp7s0 UP, 10.42.0.1/24 → Pi 10.42.0.185, SSH host key đã biết, aarch64/ROS Humble |
| Artifact trước sync | Manifest 426 file, binary và archive khớp receipt; archive kiểm cả nội dung |
| Trước sync Pi | Baseline commit 6d8cf0f1b15776e695ae4e7868ca4eb1107837fd, Git sạch, không tiến trình robot trong danh sách kiểm |
| Chép và build | Stage riêng `/home/pi/fire_robot_desktop/59cddc996966/ros`; build fire_robot_bringup và fire_robot_mission thành công trong 10,1 s |
| Source Pi sau build | 426/426 hash đúng packet đầy đủ |
| Installed Pi | 103/103 file đúng hash, errors=[], live=false, STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED |
| Kiểm độc lập | SSH đọc lại source/receipt/installed; receipt gốc Pi, bản tải và bản app lưu cùng SHA256 |
| Laptop | Static checker cùng packet đạt, 95/95 file cài đúng; model/asset/metadata kiểm tĩnh đạt |
| Entry Pi | Generated app entry mặc định in PLAN_ONLY_LIVE_FALSE; không dùng --run-observe, không mở I/O |
| Bộ cũ trên Pi | 21.158 file trước sync giữ nguyên hash và resolved target; 39 stage R5 + 6 stage app cũ được bảo toàn, baseline Git vẫn sạch |
| Bộ cũ trên laptop | 1.954/1.955 file bảo vệ giữ nguyên; thay đổi có chủ đích duy nhất là artifacts/current.txt từ 6734 sang 59cddc99 |
| Pending thật | Giữ nguyên SHA a13a087426d353a234743217da9bbd7fff1857c8b78ce353db5c83453f0ce676 |
| App mới | Supervisor dùng đúng artifact 59cddc99; GUI phase=BLOCKED, recovery attempt=0, chưa gắn RViz view hoặc bấm thao tác vận hành |

Sau khi sync và lần đối chiếu hash độc lập đã hoàn tất, một kiểm tra SSH
chỉ đọc bổ sung cuối lượt bị timeout. Đọc lại route thấy `enp7s0 DOWN`,
không còn địa chỉ LAN; WiFi laptop là 10.251.11.86, chưa phải WiFi Pi. Vì vậy
không tiếp tục SSH qua route khác. Đây không phải lỗi copy/build/receipt;
`pi_final_network_change.json` ghi kiểm cuối chưa hoàn tất do mạng đã đổi,
không gọi nó PASS. Không cần chép lại gói chỉ vì rút LAN sau sync.

Các stage R5 mới từ công việc CLI khác, gồm 2484 và dfac, nằm trong hash bảo
vệ trên Pi; không sửa hoặc nhập chúng vào app. App này vẫn dựa parent 0c54 và
model best_29_10_2026.pt, không đổi sang model minireplay trong lượt đồng bộ.
Command/watchdog, ngưỡng TF/seal và node robot giữ nguyên. Static sync chưa
chứng minh lỗi TF/USB trên robot trước đây đã hết.

## Receipt và selector

Receipt app đọc trên laptop:
`/home/huudinh/.local/state/fire_robot_control/pi_receipts/59cddc9969669e937e0fd1c98a3982156b6e7e7096204787522895dabc857a38.json`.

Receipt gốc Pi:
`/home/pi/fire_robot_desktop/59cddc996966/pi_install_receipt.json`.

SHA256 của ba bản receipt:
`eb973ef6bbbc1756cfe5fcd34a1802eb422a716b6e5381ba5e9d7de3a544159c`.
Log sync gốc: `/home/huudinh/.local/state/fire_robot_control/sync_logs/sync_ymjoLV`.

`build_receipt.json` trong artifact niêm phong vẫn ghi PENDING của thời điểm
build; không sửa file đó sau sync. Receipt Pi thực tế được lưu riêng đúng
packet và đã được validate_pi_receipt chấp nhận.

Đã chọn current mới sau khi hash/receipt/checker hai host đạt, dưới lock của
supervisor và R5. Chỉ thay selector lần mở app; không thay shortcut, script
R5, source artifact cũ hoặc pending. Bản selector cũ được lưu trong evidence
`current.before.txt`; chưa thực hiện rollback và chưa chứng nhận vận hành.

App mở lần đầu thành công. Một lần kiểm tiến trình từ sandbox không thấy
process ngoài namespace nên người thực hiện mở trùng; lần hai bị common
supervisor lock từ chối, hiện hộp lỗi. Đã kiểm lại ngoài sandbox: đúng một
supervisor 273082 và GUI 273107 của artifact mới. Chỉ đóng zenity của lần mở
trùng có đúng log mtqbkm, giữ app chính nguyên. Log của cả hai được lưu;
đây không phải lỗi startup robot hoặc app tạo hai phiên.

Probe PLAN_ONLY đầu ghi đè PYTHONPATH nên thiếu ament_index_python; đã chạy
lại với đường dẫn ROS được giữ nguyên và đạt. Lệnh runtime app vốn đã nối
PYTHONPATH, không sửa source hay cài thêm thư viện để xử lý lỗi lệnh chẩn đoán.

## Phiên cũ cần xác minh

Audit **chỉ đọc** hồ sơ hiện tại trả `RECOVERY_STOP_NOT_CONFIRMED`: desktop
receipt cũ chưa có xác nhận robot dừng. Audit dừng ở bằng chứng laptop,
chưa tới bước kiểm Pi. Không xóa/archive pending hoặc tạo STOP proof giả.
App mới do đó mở BLOCKED là đúng hồ sơ hiện tại, không phải thiếu receipt
đồng bộ 59cddc99 hoặc lỗi nút vừa sửa.

Nút **Kiểm tra lại** chỉ đọc và có thể tiếp tục báo chưa đủ STOP. Nút
**Xác minh dừng trên robot** là thao tác khác: mở I/O để lấy proof mới qua
STOP/OFF/home, không chạy nhiệm vụ; servo có thể về home. Thao tác đó chưa
được thực thi trong lượt này và cần người giám sát ở điều kiện phù hợp.
Cần rút LAN/chuyển WiFi Pi đúng profile trước mọi phiên live, kể cả xác minh.
Chỉ khi xác nhận dừng và cleanup đầy đủ mới lưu hồ sơ cũ và về IDLE.

## Cách thử tiếp khi user sẵn sàng

1. Giữ nguyên các bộ R5 cũ và không chạy chúng cùng app. Mở sẵn
   [hướng dẫn offline](../HUONG_DAN_QUET_MAP_OFFLINE.txt) trước khi mất Internet.
2. Rút LAN, nối laptop vào WiFi Pi theo profile hiện hành. Không cần Internet
   để dùng app; chưa bấm Quét map/Chữa cháy khi còn LAN.
3. Quan sát robot đã dừng. Có thể bấm Kiểm tra lại; với hồ sơ này sẽ cần đọc
   lý do thiếu STOP. Khi sẵn sàng cho STOP/OFF/home có giám sát, dùng Xác minh
   dừng trên robot. Giữ WiFi/app tới khi có kết quả; không tự cho là đã đạt.
4. Khi về IDLE, thử quét→lái theo hướng dẫn→lưu tên mới→dừng, đối chiếu map/
   camera và cleanup. Sau đó mới nghiệm thu chọn map→chữa cháy→Initial Pose→
   quan sát xe/đường Nav2, X/Q và phục hồi. Không cần phun để kiểm launcher.
5. Nếu lỗi, giữ log/ảnh và đọc stage/lỗi gốc; không xóa pending hoặc khởi động
   chồng. Các lỗi TF/USB thực tế trước đây vẫn OPEN cho tới khi đủ bằng chứng.

Không SSH/sudo chạy thiết bị, NTP, đổi mạng, flash, motion/bơm hoặc ROS launch
trong lượt sync này. Các package được build nhưng không khởi động. Nghiệm
thu phần cứng và WiFi vẫn nằm ngoài kết quả static PASS.

## Bằng chứng và kiểm lại

[Thư mục evidence](qa_evidence/pi_sync_20261030_59cddc99/) gồm sync.log, receipt,
verification_summary, pi_before/after nén JSON, snapshot helper, manifest hash,
PLAN_ONLY, pending audit chỉ đọc, selector trước/sau và app GUI log. SHA256SUMS
pin từng file; lệnh kiểm `sha256sum -c SHA256SUMS` từ thư mục evidence.

[Registry mới](PI_SYNC_REGISTRY_2026-10-30.md): 96 tiêu chí → 68 PASS/19 PARTIAL/
9 NOT_RUN. Chỉ nâng P03 và H01 nhờ sync/hash Pi thật; không cộng các snapshot
hash thành testcase chức năng hoặc đổi các phần WiFi/robot chưa chạy thành PASS.

Đã kế thừa 190 unittest + 158 pytest + CTest của bản source nguyên; không chạy
lặp bộ đó trong lượt sync. Đồng bộ có kiểm tra tĩnh thật và PLAN_ONLY như
bảng trên, không phải runtime SLAM/Nav2/YOLO hoặc chứng nhận R5 CLI hôm nay.
