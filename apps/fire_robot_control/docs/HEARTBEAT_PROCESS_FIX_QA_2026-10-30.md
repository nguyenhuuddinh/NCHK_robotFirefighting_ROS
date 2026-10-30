# Sửa heartbeat của app và QA hồi quy — 30/10/2026

Gói cuối: `d5209520c8f894a0996f3e3841ba2709b945192f1cf38087c3995907d1a2e222`.
R5 gốc vẫn là `0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0`.

## Phạm vi và các lỗi đã kiểm tra thêm

Lượt 461ce77 dừng ở CONSUMERS do `FRONTEND_LEASE_EXPIRED`, rồi thiếu dữ liệu xác nhận
STOP. Chẩn đoán trước đã tái hiện native ROS giữ GIL khiến thread đọc heartbeat cũng
bị đứng, dù Qt vẫn gửi. QA bổ sung trước khi sửa xác định thêm:

1. Q đi qua kiểm tra graph bình thường trước khi bắt đầu STOP.
2. Tuổi dữ liệu và STOP được đánh giá theo thời điểm lấy trước các query native,
   có thể đã cũ khi query trả về.
3. Thông báo `OWNED_CHILD_EXIT:operator` che mất lý do liveness.
4. Cần dành thời gian cho callback sau một lượt kiểm tra chậm, nhưng không được tự
   chèn khoảng chờ khiến chu kỳ 200 ms trở thành hơn 350 ms. Vấn đề lịch xử lý này
   được phát hiện và sửa trong quá trình review bản mới, trước khi chọn phát hành.

## Thay đổi cuối

- `services/operator_process_channel.py`: helper riêng nhận heartbeat thật từ Qt.
  Bootstrap đi qua FD riêng, kiểm UID/session/generation/nonce/sequence giữ nguyên.
  IPC chỉ trả trạng thái hiện tại theo yêu cầu, không tích lũy snapshot cũ. Q được
  chốt và ưu tiên X; helper chết, dữ liệu IPC sai/cũ hoặc UI mất liveness đều không
  cho phiên tiếp tục cấp quyền. Helper không import/publish ROS, không tự tạo heartbeat.
- Helper thuộc cùng process group của operator; parent chết thì EOF làm helper thoát.
  Không có daemon tách rời hoặc helper giữ quyền motor/bơm.
- `operator_runner.py`, `operator_entry.py`, `operator_launch.py`: nối helper và ghi
  diagnostic vào đúng thư mục phiên; vẫn dùng STOP verifier của R5.
- `operator_adapter.py`: yêu cầu STOP ngay khi nhận Q, trước normal tick.
- `app_operator_source.py`, `candidate_build.py`: sinh **bản sao dành riêng cho app**
  từ source R5 đã pin. Đưa query quyền sở hữu lên trước xét policy; lấy thời gian mới
  sau query và trước publish/STOP. Query vượt 350 ms làm giữ quyền ở trạng thái an toàn,
  không biến dữ liệu cũ thành dữ liệu mới. Dữ liệu tốt trở lại đi qua điều kiện phục hồi
  hiện có, không tự hồi sinh phiên đã Q hoặc replay phiên cũ.
- `operator_loop.py`: dành khoảng callback có giới hạn; sau query khoảng 200 ms vẫn
  giữ chu kỳ dưới 350 ms trong ca kiểm. STOP drain được pace để callback có thể nhận
  dữ liệu trong thời hạn xác minh gốc.
- `operator_timing.py`, `liveness_diagnostic.py`, `session_outcome.py`: ghi mã lý do,
  thời điểm gửi/nhận heartbeat, khoảng reader không chạy và query chậm; UI ưu tiên mã
  liveness đã allowlist và khớp session/packet. Diagnostic không phải receipt STOP,
  không chứa nonce/mật khẩu và không thay bằng chứng actuator.
- `recovery_probe.py`: nhận diện thêm operator/helper còn sống khi kiểm tra tiến trình.

Không thay watchdog Pi/firmware, lease UI 350 ms, connect grace 3 s, STOP timeout 4 s,
topic/type/QoS, GPIO, baud hoặc cấu hình mạng vận hành. Thử ROS dùng localhost/domain
riêng, không đổi cấu hình domain của launcher.

## Bảo toàn quét map và R5

Đối chiếu với gói 0b53eb57 trước sửa: các file sau nguyên byte:

- `mapping_runner.py`, `mapping_monitor.py`.
- `map_catalog.py`, `map_save.py`, `map_snapshot.py`.
- `app_health_entry.py`.

Map `TN1_30_10_2026.yaml/.pgm` của người dùng được kiểm hash trước/sau. Lớp Bootstrap
vẫn được mapping import như trước; helper và bản sao operator mới chỉ được tạo khi
chạy operator chữa cháy. Sáu file nguồn quan sát trên và 364 file R5 gốc được kiểm lại
trước chọn bản. Không ghi đè lệnh/candidate R5 terminal, không sửa `ai/` hoặc firmware.

Model app vẫn là `best_minireplay.pt`, SHA256
`e792f896faf3fb584842898e93913cdfd0fc33b5d48d620163fadd15f02212c4`.

## Kết quả QA

| Nhóm | Kết quả |
| --- | --- |
| Toàn bộ unittest của app | 254 PASS |
| QA tích hợp `qa_*.py` | 186 ca đã PASS; một ca health cần chạy lại riêng như ghi dưới |
| Native ROS giữ GIL 500 ms + Qt thật | Thread cũ tái hiện Q; helper mới giữ `FRONTEND_ACTIVE` |
| App-generated operator với bus R5 thật/thiết bị giả | Q ở standby/nav/turn/IR/WAIT_P/pulse/review, X hủy goal, dữ liệu hồi phục, tuổi STOP: PASS |
| Helper chết, UI đứng/mất kết nối, sequence replay, sai identity, IPC cũ, overflow, parent chết | PASS, không tự phục hồi phiên đã chốt dừng |
| Map catalog/lưu/snapshot, mapping runner, UI/bootstrap, STOP/recovery, auth, layout/xe/path | Bộ hồi quy cũ PASS |
| Build native + Qt CTest của gói cuối | PASS |
| Preflight read-only fire/mapping/verify_stop | PASS trên cấu hình vận hành |
| Operator và helper thật của gói cuối, ROS localhost, không telemetry robot | Q nhận được; STOP giữ unconfirmed/rc2; group không còn process và socket được xóa: PASS |

Lần chạy song song cuối có **185 PASS/1 bị guard chặn** ở ca health R5. Ca đó không tiếp
tục thử khi snapshot tiến trình khớp mẫu R5. Không lưu PID gây chặn, nên không khẳng định
tiến trình cụ thể; unit suite cùng lúc có subprocess R5 PLAN_ONLY và operator giả lập.
Snapshot sau đó không thấy tiến trình robot và pending thật không đổi. Chạy riêng đúng
ca health với thư mục log mới: **1 PASS/7,61 s**, không nới guard hoặc sửa fixture.
Không cộng các lần chạy lại thành ca QA mới. Bộ 186 ca trước đó cũng đã PASS toàn bộ;
log bản cuối và lần chạy riêng được giữ riêng.

Một lỗi điểm ghép source được test build bắt trong bản trung gian; đã sửa anchor thành
đoạn riêng của handback, kiểm tra sinh source/cú pháp và chạy lại đầy đủ unittest,
các ca timing, runner, đóng gói. Log bị từ chối vẫn giữ để review, không dùng làm bằng
chứng PASS của bản phát hành.

## Bằng chứng

Thư mục: `qa_evidence/process_heartbeat_fix_20261030/`.

- `unittest_release.log`: 254 ca của source cuối.
- `pytest_release.log`, `health_release_serial.log`: 185 ca và ca health chạy riêng.
- `timing_final.log`: các ca timing/native/runner sau sửa điểm ghép.
- `package_release.log`: gói cuối và CTest.
- `release/packaged_operator_result.json`, `packaged_operator.log`, `packaged_operator_ui.json`:
  operator thật trên localhost, không telemetry.
- `protected_before.json`, `release/protected_after.json`: nguồn mapping/map/pending/R5.
- `release/pi_sync.log`, `release/pi_install_receipt.json`: đồng bộ tĩnh Pi đúng packet.
- `release/app_operator_delta.diff`: diff của bản sao app so với R5 nguyên gốc.

## Triển khai và giới hạn

Gói mới được đặt trong thư mục riêng, chỉ chọn sau khi receipt Pi khớp packet và các
kiểm tra cuối đạt. `pending.json` của phiên 461ce77 được giữ nguyên, không xóa khóa hoặc
sửa receipt cũ. Cửa sổ app đang mở trước đó cần đóng/mở lại để nạp phiên bản mới.

Chưa chạy lại quét/chữa cháy với robot thật trong lượt sửa này. Lỗi Nav2 child cleanup
SIGTERM/-15 đã biết vẫn được nhận diện là cleanup chưa sạch; không sửa Nav2/R5 hoặc gọi
đó là PASS để bỏ guard. Với phiên cũ chưa có STOP, vẫn cần **Xác minh dừng trên robot**
thành công trước khi mở phiên mới. Kết quả localhost và đồng bộ tĩnh không phải nghiệm
thu motor/bơm, Wi-Fi thực hoặc xác nhận dừng vật lý.
