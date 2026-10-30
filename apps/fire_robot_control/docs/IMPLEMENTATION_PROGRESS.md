> **Mới nhất — 857b9eb3 đã sửa relay xác thực, sync và mở:** 211 unittest /
> 160 integration / CTest đạt; hộp Qt accept/cancel đi qua worker-supervisor thật.
> Lần xác minh user trước lỗi NTP refresh 100 s, chưa I/O. R5/pending nguyên;
> bản mới chờ user bấm xác minh và nhập mật khẩu Pi khi hỏi.
> [Báo cáo/giới hạn](AUTH_RELAY_FIX_2026-10-30.md).

> **Mới nhất — a387d27e đã sửa và sync:** Nhánh phục hồi sau Pi reboot đã
> qua 205 unittest / 158 ca tích hợp / CTest; chuẩn bị chỉ đọc trên Pi thật đạt.
> R5/pending giữ nguyên. App mới đã mở BLOCKED, chờ user xác minh STOP mới;
> chưa I/O hoặc nghiệm thu robot. [Báo cáo và bằng chứng](PI_REBOOT_RECOVERY_FIX_2026-10-30.md).

> **Sau thao tác user:** RECHECK đã nhận 6 lần nhưng thiếu STOP cũ; user thử
> Xác minh dừng cũng bị chặn bởi Pi đã reboot. Đã tái kiểm chỉ đọc ra lỗi
> RECOVERY_PI_BOOT_CHANGED; chưa I/O, chưa sửa code/xóa pending.
> [Chẩn đoán và đính chính log](RECOVERY_AFTER_PI_REBOOT_DIAG_2026-10-30.md).

> **Mới nhất 30/10/2026 — app 59cddc99 đã sync Pi và được chọn:** 426 source /
> 103 installed Pi, 95 installed laptop đúng hash; R5/bộ cũ giữ nguyên. App mới
> đang mở BLOCKED do hồ sơ cũ thiếu STOP, pending giữ nguyên. Chưa runtime
> quét/chữa cháy, motion/bơm hoặc nghiệm thu WiFi. [Báo cáo và bước tiếp](PI_STATIC_SYNC_2026-10-30_59cddc99.md).

> **Trước sync 30/10/2026 — ứng viên 59cddc99:** Đã sửa F1/F2/F3 và thêm
> xe nhìn từ trên xuống + đường kế hoạch Nav2. QA laptop 190 unittest,
> 158 pytest và CTest đạt; R5 giữ nguyên. Chưa sync Pi/chọn current hoặc
> nghiệm thu robot. [Báo cáo và bằng chứng](FIX_AND_ROBOT_PATH_QA_2026-10-30.md).

> **QA bổ sung 30/10/2026:** Bản b3b81081 chưa đủ điều kiện đồng bộ/chọn vận
> hành: đã xác nhận 1 lỗi P1 phục hồi STOP sau seal timeout và 2 lỗi P2 bố
> cục. Code vận hành/R5/current/pending giữ nguyên trong lượt QA.
> [Báo cáo QA](QA_RECHECK_SESSION1_2026-10-30.md) ·
> [Registry mới nhất](QA_RECHECK_REGISTRY_2026-10-30.md).

> **Cập nhật 30/10/2026 — ứng viên b3b81081:** Phiên 1 đã triển khai entrypoint
> riêng cho app, STOP/cleanup/recovery và QA offline; R5 local giữ nguyên.
> Chưa chọn current, chưa sync Pi/robot. 10 test R5 cũ lỗi fixture, registry
> còn PARTIAL và 10 ca phần cứng NOT_RUN.
> [Báo cáo](SESSION1_APP_ENTRY_QA_2026-10-30.md) ·
> [Registry](SESSION1_QA_REGISTRY_2026-10-30.md).

> **Cập nhật 29/10/2026 — 6734c284:** Kiểm tra lại hỗ trợ phiên R5_AUTOMATIC,
> số lần/kết quả/lý do hiển thị bền qua STATUS. QA146 Python+42 Qt/IPC+1 smoke
> và 1 CTest PASS; đồng bộ Pi source416/installed103 PASS. Audit thật chỉ đọc
> phiên chữa cháy c3213541 PASS, pending giữ nguyên, mở app mới BLOCKED chờ user
> bấm kiểm tra; chưa runtime robot mới. [Báo cáo](FIRE_RETRY_AND_RESULTS_2026-10-29.md).

# Tiến độ Fire Robot Control — 29/10/2026

**Mới nhất: cc4a3574 đã sync Pi**, sửa lỗi toggle nút giữa lúc nhấn làm mất
RECHECK. Regression FAIL→PASS; 40 Qt/IPC/smoke và CTest PASS. Người dùng bấm
Kiểm tra lại trên app mới đạt: CHECKING→IDLE, archive pending cũ đúng proof.
Chưa quét lại hoặc chạy robot. [Chi tiết](HELD_RETRY_CLICK_FIX_2026-10-29.md).

**Bản c93a42c9 đã sync Pi**, sửa cảm giác nhấn nút và phản hồi gửi/chờ
xử lý. 39 ca Qt/IPC/smoke và 1 target CTest PASS; manifest416/Pi installed103
đạt. Đã mở app mới, giữ pending thật ở BLOCKED; chưa chạy robot.
[Chi tiết](BUTTON_FEEDBACK_2026-10-29.md).

**Bản 1c55c4db đã sync Pi**, thêm Kiểm tra lại ở BLOCKED cho mapping có
STOP/cleanup proof. 141 unittest, 43 ca Qt/tích hợp, 1 target CTest PASS;
audit Pi thật chỉ đọc và controller state tạm PASS, không tạo phiên robot.
Lỗi TF giữ nguyên theo yêu cầu user. [Chi tiết](MAPPING_RETRY_FIX_2026-10-29.md).
Đã mở app mới; pending thật giữ lại để người dùng bấm kiểm tra.

**Runtime quét 19:50:12: chưa đạt, BLOCKED.** Pi SEAL timeout do kiểm tra
freshness TF không đạt; SLAM/teleop chưa khởi động. STOP telemetry confirmed,
group/child cleanup đọc lại sạch, nhưng runner error làm app giữ khóa.
Ở bản 69dfe chưa có nút phục hồi BLOCKED; bản mới bổ sung như trên, không xóa
pending để vượt khóa.
[Phân tích log và giới hạn](MAPPING_RUNTIME_195012_BLOCKED_2026-10-29.md).

**Hiện tại:** packet `69dfe74e…` sửa popup chọn map bị reset mỗi STATUS; ba
ca popup/mouse PASS, 19 ca F1–F3 PASS, 17 tích hợp cũ PASS, 124 unittest và
1 target CTest PASS. Đã sync Pi/source413/installed103 và static laptop95;
receipt đúng packet đã lưu, app đã mở lại IDLE. Chưa runtime robot.
[Báo cáo mới nhất](MAP_SELECTION_FIX_2026-10-29.md). Các đoạn 750008 bên dưới
ghi lại bước trước.

**Đã sửa cả 3 finding của QA lần hai.** 19 ca hồi quy sửa lỗi PASS; bộ cũ
124 Python + 17 tích hợp + 1 target CTest PASS. Gói mới `750008b09dfe…` đã được
chọn trong current.txt; e05 và các bộ ROS cũ giữ nguyên.
[Thay đổi, bằng chứng và giới hạn](QA_FIXES_BEFORE_PI_2026-10-29.md).
**Đã sync Pi packet 750008 thành công**: hai package build, 413 source và
103 file cài Pi khớp hash; receipt đã lưu để app đọc. Bộ cũ giữ nguyên.
[Bằng chứng sync](PI_STATIC_SYNC_2026-10-29.md). Chưa chạy robot/nghiệm thu WiFi.

Đã có hai chế độ riêng:

- **Preview:** RViz native và camera giả; các nút vận hành bị khóa. Script
  `launchers/start_preview.sh` cô lập trong ROS domain 212/localhost.
- **Ứng viên tích hợp:** RViz native, camera, supervisor độc lập và các nút đã
  nối với runner R5, operator và phiên SLAM. Gói mới có manifest riêng; chưa
  được thử trên robot thật. Đã đồng bộ và kiểm cài đặt tĩnh trên Pi.

| Hạng mục | Kết quả |
| --- | --- |
| Map và camera | Một cửa sổ RViz; camera raw khi quét, ảnh YOLO khi chữa cháy. Dữ liệu mất nguồn bị báo chờ, không mặc định là OFF hoặc không có lửa. |
| Quét/lưu/chọn | Phiên quét sở hữu Pi/SLAM và khung bàn phím teleop; lưu cặp YAML/PGM không ghi đè; chọn được map cũ hoặc map mới lưu. Đã thử luồng giao diện bằng supervisor giả. |
| Chữa cháy | Lấy pairing mới, kiểm packet/receipt, tự gọi runner laptop, dùng map đã chụp snapshot; Initial Pose và Goal chỉ hiện theo giai đoạn. Đã kiểm offline, chưa chạy Pi/Nav2/YOLO thật. |
| X/Q/đóng | X tới operator hiện có; Q/đóng hoặc mất heartbeat yêu cầu STOP; chờ local trước Pi, tách STOP proof khỏi cleanup. Trạng thái chưa chứng minh được sẽ khóa phiên tiếp theo. Chưa chứng minh robot dừng vật lý. |
| Gói | Source, manifest, binary, archive và receipt build; launcher đã đăng ký vào menu ứng dụng của huudinh. Đã sync qua LAN thật, build Pi hai package và lưu receipt 103 file đúng packet; chưa runtime. |

[Chi tiết QA, lệnh và các giới hạn](QA_INTEGRATION_2026-10-29.md).
