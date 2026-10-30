# QA bổ sung: Q trong lúc Nav2 tự phục hồi

Kết quả dưới là **trước sửa**, được giữ làm đối chứng. Đã có
[bản sửa app 49c68b77 và QA sau sửa](NAV2_RECOVERY_TEARDOWN_FIX_2026-10-30.md).

**Verdict: chưa đủ điều kiện coi mọi ca Q/cleanup đã đạt.** Có lỗi native được
tái hiện trên laptop; chưa sửa production code trong lượt QA này. Không SSH,
sync, chạy robot, motor/bơm, đổi timeout/QoS hoặc thay R5.

## Finding P1: Q trong reset có thể crash dù helper nhận shutdown thành công

Phạm vi source kích hoạt teardown: `services/nav_session.py:44–52` gửi SHUTDOWN
cho hai manager; `services/nav_session.py:95–103` chuyển sang SIGINT sau RPC.
Đây là vị trí cần kiểm tra điều phối, **chưa phải kết luận dòng C++ gây crash**.

Ở packet đang chọn `5054a43e…`, TF/odom/scan giả trên localhost domain 216:
Nav2 đã ACTIVE và goal SUCCEEDED. Tạm dừng đúng container QA 6 giây rồi tiếp
tục; hai lifecycle manager mất bond và tự reset. Khi nhận Q ngay đầu reset:

- Nav log ghi 20 warning `No transition matching`, gồm các node đang unconfigured.
- Cả hai SHUTDOWN RPC trả success; helper rc0, sau đó wrapper gửi SIGINT.
- Container thoát **-6**, kèm `Magick: abort due to signal 11 (SIGSEGV)`.
- `ros2 launch`/wrapper vẫn rc0, nhưng **child audit đánh FAIL**; không được dùng
  riêng rc0 hoặc phản hồi RPC để báo cleanup sạch.
- Bấm Q ba lần chỉ tạo **một** helper; không thấy relaunch container. Các nhóm
  sở hữu đã không còn tiến trình sống sau bài kiểm tra, nhưng crash vẫn là lỗi.
- Ca đối chiếu **Q đúng một lần** tại cùng mốc reset cũng crash -6, một helper,
  child audit FAIL. Vì vậy bấm lặp không phải điều kiện cần để gây lỗi này.

Bằng chứng chính: `q_at_reset/nav.log:349–355` và `result.json` trong
[thư mục bằng chứng](qa_evidence/nav_q_recovery_20261030/).

| Ca Nav2 thật, đầu vào giả | Kết quả teardown |
| --- | --- |
| Dừng process 2 s, chưa bond reset, Q ba lần | PASS, 1,778 s, child 1/1 sạch |
| Dừng 6 s, Q một lần ngay đầu reset | FAIL, crash -6, audit chặn |
| Dừng 6 s, Q ba lần ngay đầu reset | FAIL, crash -6, audit chặn |
| Dừng 6 s, đợi cả hai manager ACTIVE rồi Q | PASS, 2,369 s, child 1/1 sạch |
| GDB giữ inferior 6 s, Q ngay đầu reset | Không crash; chỉ dùng chẩn đoán timing |

Đối chứng không debugger: trễ 2 giây chưa gây bond reset rồi Q lặp thoát sạch;
đợi cả hai manager phục hồi ACTIVE rồi Q cũng thoát sạch. Bản e2 cũ đã có một
ca trễ 6 giây/Q trong recovery dọn không sạch ở lượt QA trước; giữ nguyên
[raw đối chứng cũ](qa_evidence/stop_result_fix_20261030/e2_controlled_pause/result.json).

**Mức chắc chắn:** lỗi cleanup lúc reset có thật. Chưa đủ bằng chứng khẳng định
invalid transition chính là nguyên nhân SIGSEGV: lượt có GDB cũng có 20 warning
nhưng thoát bình thường. Chưa có backtrace crash; không suy ra lỗi model, WiFi,
ESP32, tăng timeout hoặc thay logic R5 là giải pháp đúng.

## Các ca STOP/operator đã đạt

**72 ca PASS**, gồm **48 ca mới** và **24 ca hồi quy liên quan**, 50,42 s.
Không cộng lại thành “đã chạy toàn bộ 466 ca cũ”.

48 ca mới dùng app-generated operator thật và bus R5 giả, không tạo ROS context:

- Hai fault: `NAV2_BACKEND_LOST` và `BOOT_OR_SERIAL_EPOCH_CHANGED`.
- Bốn chuỗi nút: Q, QQQ, X→Q ở hai tick khác nhau, Q→X→Q cùng batch.
- Ba chế độ telemetry, cả thứ tự callback xuôi/ngược.
- 16 ca dữ liệu liên tục: STOP confirmed sau 0,90–0,95 s; owner đã release.
- 16 ca dữ liệu mất 1 giây rồi hồi: confirmed sau 1,85–1,90 s; owner release.
- 16 ca dữ liệu mất hẳn: **0 confirmed**, hết hạn 4 s; gateway giữ raw owner
  zero-only vì chưa có odom/cmd chứng minh đứng yên. Không coi đó là release.
- Mọi ca giữ nguyên fault vận hành, không kéo dài thời hạn STOP khi Q lặp;
  các raw command sau Q đều zero, pump command đều OFF, không replay pulse.

24 ca hồi quy gồm 11 ca bus/operator hiện có, 6 ca process/Nav2 wrapper và 7 ca
terminal result. Bao gồm thiếu proof log, PID/session/packet không khớp, teardown
lỗi, ghi metadata lỗi, process treo và context bị mất.

Log: [targeted_final.log](qa_evidence/nav_q_recovery_20261030/targeted_final.log).
48 kết quả riêng lưu trong `input_cases_checked/`.

## Thử GDB và các lượt không được tính PASS

Đã thử debugger riêng cho inferior QA, không chỉnh binary hoặc guard production.
Prefix chỉ cho đúng container localhost được debug; wrapper dùng API injection
test sẵn có. Lượt giữ inferior 6 giây trước continue có tái hiện bond reset, nhưng
Q không crash. Vì vậy **chưa lấy được backtrace lỗi**, không tuyên bố tìm xong
nguyên nhân native. Kết quả debugger không thay thế verdict của CLI app thật.

Giữ tất cả log lỗi fixture/môi trường:

- Một lần xét duyệt quyền quá hạn trước khi command chạy; retry một lần đạt.
- Lượt bus đầu thiếu ROS overlay: collection fail, chưa test nào chạy.
- Lượt tiếp theo 32 pass/16 fail tại assertion đòi release khi mất toàn bộ
  telemetry. Đọc R5 gateway `live_nav_stop_gateway.py:502–518` xác nhận nó chỉ
  release sau stationary/cmd-zero proof. Đã sửa **tiêu chí QA**, bổ sung kiểm mọi
  lệnh zero/OFF và phải còn WAIT_RAW_RELEASE; không sửa gateway. Kết quả cuối 48/48.
- Hai lượt prefix đặt sai vị trí ROS CLI: Nav2 chưa start, không phải bug app.
- Một lượt SIGSTOP khi debugger attach bị debugger tự continue, không đạt mốc
  reset yêu cầu; bỏ khỏi kết luận hành vi. Lượt đúng dùng bounded GDB hold.

## Bảo toàn và hướng tiếp

Code vận hành, artifact/current, pending, map TN1 và model `best_minireplay.pt`
được đối chiếu hash trước/sau; 364 file R5 phải nguyên byte. Chỉ bổ sung fixture
Python và tài liệu QA, không đóng gói/chọn phiên bản hoặc đồng bộ Pi mới.

Hướng sửa cần thử **ở lớp app**: giữ STOP/false-arm thực hiện ngay, rồi điều phối
teardown Nav2 theo trạng thái recovery, với deadline và fallback/audit nguyên.
Phải kiểm bằng ca tái hiện trước/sau, gồm Q đơn/Q lặp và Q sau hồi phục. Nếu cần
sửa native, cần backtrace hoặc repro khoanh hàm trước; chưa đủ cơ sở sửa R5 hay
chỉ tăng timeout. Có thể tiếp tục khoanh lỗi offline, chưa cần lặp robot để tái
chứng minh ca này và chưa nên nghiệm thu toàn luồng như đã hết lỗi.
