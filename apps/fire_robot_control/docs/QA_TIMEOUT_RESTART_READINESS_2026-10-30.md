# QA timeout → xác minh → khởi động lại

Ngày 30/10/2026, giờ Việt Nam. App được chọn: `59bf2064…`. Lượt này chỉ thêm
QA/tài liệu; không sửa runtime, build, sync, SSH, khởi động robot hoặc xóa pending.

## Kết luận cho buổi bảo vệ

**Bản hiện tại chưa bảo đảm phục hồi được mọi timeout bằng nút Xác minh dừng.**
Có đường khởi động lại khi đủ STOP/cleanup. Nhưng cleanup Pi lỗi cùng boot,
thiếu receipt sau khi đã tạo phiên, hoặc cleanup cưỡng bức chưa được hỗ trợ
vẫn có thể giữ BLOCKED dù người dùng bấm lại. Chưa thể nghiệm thu khả năng
phục hồi toàn luồng để dùng làm bản bảo vệ chỉ dựa trên kết quả QA này.

## Ma trận hành vi đã kiểm

Các kết quả dưới đây có ở cả profile `DESKTOP_MAPPING` và `R5_AUTOMATIC`, trừ
các thời hạn chỉ thuộc từng runner.

| Tình huống | Hành vi hiện tại | Bấm xác minh có mở lại không? |
|---|---|---|
| Lỗi trước khi tạo phiên Pi, cleanup local sạch | Controller về IDLE, bỏ marker của lượt chưa tạo robot | Không cần xác minh |
| Startup timeout, đã có STOP và cleanup hợp lệ | Controller có thể về IDLE; nếu worker còn báo BLOCKED do mã lỗi runner, RECHECK có đường audit/archive | Có đường mở lại; không tự chạy xe |
| STOP cũ chưa xác nhận, group cũ đã hết, cleanup Pi sạch | RECHECK từ chối; prepare cho phép phiên STOP mới | Có, nếu proof STOP mới và cleanup mới đều đạt |
| Pi child TF thoát lỗi, cùng boot hiện tại | RECHECK từ chối; prepare bị chặn bởi cleanup cũ | Vẫn BLOCKED; bấm nhiều lần không đổi receipt |
| Có proof mới sạch nhưng historical cleanup Pi cùng boot vẫn lỗi | Gate audit cuối tiếp tục từ chối | Vẫn BLOCKED; chỉ sửa gate đầu chưa đủ |
| Thiếu DONE Pi hoặc receipt laptop của phiên cần phục hồi | Không đủ dữ liệu để qua gate | Vẫn BLOCKED |
| Local group bị forced ngoài mẫu Nav2 đã hỗ trợ | Từ chối trước probe Pi | Vẫn BLOCKED |
| Pi đã reboot, binding cũ còn kiểm được, Pi hiện không có owner | Nhánh NEW_BOOT_QUIESCENT_ONLY có thể cho phiên STOP mới | Chỉ mở sau proof mới; reboot không phải STOP proof |
| Mất kết nối hoặc telemetry STOP không quay lại | Không tạo xác nhận STOP giả | Tiếp tục BLOCKED; cần xử lý kết nối/dữ liệu trước |

Không khuyến nghị reboot Pi hoặc xóa pending để né guard. Bản đồ đã lưu không
bị xóa khi timeout/phục hồi. Những trường hợp phải giữ khóa vì còn tiến trình
hoặc chưa xác nhận an toàn cần được phân biệt với lỗi phần mềm không có đường
phục hồi dù đã đủ bằng chứng hiện tại.

## Nút Xác minh dừng thực sự làm gì?

1. Chỉ chạy từ BLOCKED khi không còn worker/audit đang hoạt động. Giữ nguyên
   pending của phiên cũ, tạo thư mục kết quả xác minh riêng.
2. Kiểm danh tính session/packet, receipt, group/PID cũ, lock laptop, tình trạng
   Pi qua SSH, boot/binding và lock Pi. **Nó yêu cầu tiến trình cũ đã hết**;
   không phải lệnh quét rồi kill mọi node trên hai máy.
3. Nếu qua gate, mở một phiên Pi I/O riêng bằng entrypoint app; trên laptop
   chỉ mở STOP monitor và startup health. Không mở SLAM, Nav2, YOLO, teleop
   hay chế độ chữa cháy. Pi vẫn chạy bringup I/O cần để nhận telemetry, gồm
   bridge, watchdog, cảm biến/camera và TF.
4. Chờ I/O hợp lệ, ổn định ít nhất 1 s, giới hạn 65 s ở nhánh runner này. Sau
   đó thực hiện dừng bằng cơ chế hiện có: rút quyền phát lệnh, zero/OFF và
   xác minh dữ liệu mới của vận tốc/odom đứng yên, actuator OFF/home, đúng
   boot/nguồn và endpoint lệnh đã nhả. Dữ liệu cũ hoặc chỉ thấy SSH thoát
   không phải proof.
5. Dọn các tiến trình **do phiên xác minh sở hữu**: local trước, Pi sau.
   Audit cả kết quả mới và ràng buộc phiên cũ rồi mới archive pending và về
   IDLE. Người dùng phải bấm Scan/Khởi động chữa cháy lại để tạo phiên mới.

RECHECK (Kiểm tra lại) chỉ audit file/tiến trình/lock, không mở node hay phát
STOP mới. Cả hai nút không dọn bừa các node độc lập, terminal R5 hay process
khác của người dùng. Nếu phát hiện owner còn hoạt động thì sẽ từ chối.

## Findings và file/dòng

**P1 — Thiếu đường phục hồi cleanup Pi lỗi cùng boot:**
`services/recovery_probe.py:203` yêu cầu historical cleanup thành công trước
phiên xác minh; `services/stop_recovery_runner.py:189` lại yêu cầu ở gate cuối.
Đã kiểm cả hai điểm bằng code thật với input giả. Giữ lỗi cũ là đúng; cần
đường xác minh hiện trạng và STOP mới độc lập với việc lịch sử có mã thoát 0.

**P1 — Thiếu receipt có thể làm mất đường xác minh mới:**
`services/stop_recovery_runner.py:108–109` đọc bắt buộc desktop/laptop receipt
trước mọi bước phục hồi. Đã tái hiện FileNotFoundError trong hai profile.
Cần xử lý riêng trạng thái khởi động dở có đủ ownership/binding hiện tại,
không coi mọi file thiếu là an toàn hoặc tự dọn process không xác định.

**P2 — Tên nút có thể khiến hiểu nhầm là dọn tất cả node:**
`services/stop_recovery_runner.py:144–146` kiểm không còn process trước;
`services/mapping_runner.py:175–178` mới thực hiện nhánh STOP-only sau I/O.
Cần cho biết đang kiểm lịch sử, đang xác minh STOP mới hay chưa qua gate,
kèm lý do cụ thể; không chỉ yêu cầu người dùng bấm lại.

Controller phân loại kết quả tại `services/supervisor.py:330–365` theo
STOP/cleanup/phiên có được tạo, không chỉ theo chuỗi timeout. Phiên xác minh
chưa thể về IDLE ngay sau FINISHED; còn audit/archive ở dòng 345–359.

## QA đã chạy: 88 ca khác nhau

Evidence: [timeout_recovery_20261030](qa_evidence/timeout_recovery_20261030/).

- **32 ca mới**, `matrix_32_final.log`: 4 lượt chạy mapping runner thật với
  ROS/process/clock giả (IO, seal, SLAM timeout và IO timeout của xác minh);
  6 deadline chữa cháy qua StartupSequence thật; 14 tổ hợp profile/receipt
  đi qua gate prepare/audit/archive thật với OS/network giả; 8 ca nút
  SCAN/FIRE qua controller với FINISHED được tiêm có chủ đích.
- Timeout camera có đúng một lần RETRY_PI/RESTART_HEALTH trước seal; lần
  tiếp theo timeout thì STOP. Dữ liệu về sau một startup đã kết thúc không
  tự restart hoặc gia hạn deadline.
- Các ca mapping xác nhận local drain trước Pi STOP; timeout không mở
  teleop. STOP-only không mở SLAM/Nav2/YOLO; việc này còn được kiểm trong
  bài runner success bên dưới.
- **48 ca**, `stop_inputs.log`: Q/QQQ/XQ/QXQ, hai fault, thứ tự dữ liệu hai
  chiều, telemetry mới/trễ 1 s/không có. Có hồi phục trong giới hạn thì nhận
  proof; thiếu dữ liệu thì giữ STOP chưa xác nhận. Sau Q chỉ có zero/OFF,
  không phát lại pulse; Q lặp không kéo dài deadline. Đây là fake bus.
- **8 ca**, `entry_transport.log`: scan success, STOP-only success, hai
  trạng thái Pi chờ drain, mất transport ở bốn giai đoạn. Dừng đúng thứ tự,
  không restart ngầm hoặc dựng STOP proof khi mất mạng.
- Các ca PASS kiểm trạng thái dự kiến, **bao gồm các ca chứng minh app vẫn
  BLOCKED**. 88 PASS không phải thông báo đã sửa các finding trên.
- Giữ log các lượt đầu chưa hoàn chỉnh: thiếu tick của process giả; kỳ vọng
  camera timeout chưa tính retry có sẵn; và so sánh error nguyên chuỗi trong
  khi controller thêm nhãn cleanup. Đã sửa fixture/kỳ vọng cho đúng code
  hiện tại; không sửa production để làm QA đạt.
- `protected_before.json`/`qa_summary.json`: source runtime, current, pending
  nguyên hash; kiểm lại 364 file R5. Không chạy một nút nào trên robot thật
  trong lượt này, nên chưa nghiệm thu STOP/khởi động lại phần cứng.

Lệnh tái chạy bộ mới, sau khi nạp đúng ROS/overlay như launcher:

```bash
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=211 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/apps/fire_robot_control/tests:$PWD/apps/fire_robot_control/artifacts/59bf2064b462e66b6ead762752ead07550d7a67e5dec9824541251c54efd4a64/ros/scripts:$PYTHONPATH"
python3 -B -m pytest -q apps/fire_robot_control/tests/qa_timeout_recovery_matrix.py -p no:cacheprovider
```

## Điều kiện hoàn tất bản phục hồi tiếp theo

Cần sửa ở app, giữ R5/watchdog nguyên: phân biệt lỗi khởi động với trạng thái
dừng hiện tại; phục hồi có chứng cứ cả khi lịch sử cleanup lỗi hoặc file thiếu;
chỉ dọn đúng process thuộc phiên; có giới hạn chờ và hiển thị lý do; không mở
phiên mới khi chưa đủ bằng chứng. Sau sửa phải kiểm cả gate đầu/cuối và chuỗi
timeout → STOP/cleanup → phục hồi → bấm khởi động lại cho scan và chữa cháy.
Cuối cùng cần một lượt robot có giám sát để nghiệm thu, không dùng kết quả
fixture thay cho phần cứng.
