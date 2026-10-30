# QA: Kiểm tra lại và Xác minh dừng cùng bị chặn

Ngày 30/10/2026, giờ Việt Nam. Phạm vi: đọc receipt/log thật trên laptop và
Pi qua LAN, đối chiếu source và chạy QA phục hồi bằng fixture. Không sửa code
vận hành, không khởi động node/STOP mới hoặc xóa hồ sơ phiên cũ.

## Kết luận đã xác minh

Hai nút đã nhận thao tác. Bản app mới `59bf2064…` đang xử lý hồ sơ phiên cũ
`49c68b77…`, không phải lỗi phím Esc quay trở lại. Phiên cũ trên laptop là
`session_a7b30b81d0ed4f3382c7a105351b48fe`; Pi tương ứng:
`/home/pi/qa_logs/r5_automatic_20261030_153737`.

Laptop ghi `robot_stop_confirmed=true`, local `stop_confirmed=true`,
`clean=true`, `group_cleanup_clean=true`, `errors=[]`. Pi có DONE và hai nhóm
tiến trình đã thoát, không cưỡng bức, nhưng `children_clean=false` vì
`CHILD_DIED:odom_to_tf_broadcaster-8:1`. Đây là bằng chứng cleanup **thất bại**,
không phải thiếu file DONE. Guard giữ BLOCKED là đúng với dữ liệu hiện có.

## Findings

### P1 — Luồng xác minh mới phụ thuộc vào cleanup thành công của phiên cũ

- `services/recovery.py:48`: Kiểm tra lại đọc receipt cũ rồi gọi
  `remote_cleanup`; receipt Pi thất bại làm dừng trước khi kiểm tra SSH.
- `services/live_session.py:300` gọi bước chuẩn bị xác minh trước preflight
  và trước khi tạo runner mới.
- `services/stop_recovery_runner.py:146` gọi `remote_idle_proof`; cùng boot Pi
  sẽ đi qua `services/recovery_probe.py:203` vào `probe_pi`, và dòng 158 lại
  yêu cầu cleanup cũ thành công bằng cùng validator ở dòng 92–95.
- `services/stop_recovery_runner.py:189` cũng kiểm lại historical cleanup ở
  cuối audit mới; nếu chỉ sửa gate đầu thì vẫn có thể bị chặn ở gate cuối.

Vì receipt cũ không tự đổi, bấm lại hai nút trong cùng trạng thái sẽ tiếp tục
thất bại. Các lần xác minh mới nhất dừng ở `CHECKING_OLD_SESSION`,
`session_was_started=false`, chưa mở runner/telemetry để xác minh STOP mới.

Đây là khoảng trống chức năng phục hồi của app cho trường hợp Pi đã dừng hết
tiến trình nhưng có child thoát lỗi. Không giải quyết bằng đổi `clean=false`
thành true, xóa pending hay bỏ kiểm an toàn. Hướng sửa cần một nhánh chứng minh
Pi hiện không còn chủ sở hữu/tiến trình, đúng binding/boot, rồi mới cho phép
một phiên STOP mới; chỉ archive pending khi có proof mới đầy đủ. Kết quả FAIL
của phiên cũ phải giữ nguyên. Ca cùng boot cần qua được cả gate đầu và cuối.

### P2 — Node TF trên Pi thoát mã 1 trong lúc đóng bringup

`generation_1/pi.log` có chuỗi rõ ràng:

1. Dòng 982: launch nhận SIGINT; dòng 983: gửi SIGINT cho node TF.
2. Dòng 1019–1038: traceback ở `rclpy.spin` → executor →
   `_take_subscription` → `sub.handle.take_message`:
   `RuntimeError: Unable to convert call argument to Python object (compile in debug mode for details)`.
3. Dòng 1044: node TF PID 37795 thoát mã 1. Bảy node còn lại thoát sạch;
   child audit có `started=8`, `terminal=8`, `missing=[]`, không fallback.
4. DONE Pi ghi `error=null`, nhưng child audit thất bại. Laptop nhận
   `REMOTE_EXIT_UNCONFIRMED`, SSH rc2 và lỗi tổng
   `PI_RUNNER_EXITED_DURING_SESSION`.

Source `odom_to_tf_broadcaster.py:250–251` lưu ngoại lệ spin vào `primary_exc`,
rồi dòng 296–297 ném lại sau cleanup. Điều này giải thích exit 1 đã quan sát.
Hash source TF tại Pi khớp bản R5 nguyên gốc được app dùng lại.

Đã xác định vị trí và kết quả ngoại lệ. Chưa tái hiện được nguyên nhân nội bộ
ROS tạo lỗi chuyển đổi; thứ tự log sau SIGINT chưa đủ chứng minh chính SIGINT
gây lỗi. Không được nuốt mọi RuntimeError hoặc quy lỗi ESP32/model/Nav2 chỉ từ
traceback này. Trước khi sửa xử lý node cần tái hiện có kiểm soát và phân biệt
lỗi khi đang chạy với lỗi thật sự trong shutdown. Nếu cần sửa cho app, dùng
lớp/wrapper riêng; không ghi đè source release R5.

### P2 — Thông báo che mất lý do cleanup Pi thất bại

`services/recovery_probe.py:92–95` gộp file không có, group chưa thoát, child
thoát lỗi và forced cleanup vào mã `RECOVERY_PI_CLEANUP_MISSING`.
`services/recovery.py:196` chưa có câu giải thích riêng cho mã này nên hiện
thông báo chung. User không thấy node TF đã thoát mã 1 hay xác minh chưa bắt
đầu. Nên hiển thị nguyên nhân child audit có giới hạn và trạng thái gate,
giữ kết quả lịch sử thay vì chỉ yêu cầu bấm lại.

## Bằng chứng và QA

[Thư mục evidence](qa_evidence/pi_cleanup_recovery_20261030/):

- `actual_local_repro.json`: chạy validator hiện tại trên receipt thật, chỉ
  đọc local; cả `local_proof` và `remote_cleanup` trả đúng
  `RECOVERY_PI_CLEANUP_MISSING`.
- `recent_verification_attempts.json`: các lượt app cũ/mới ghi lỗi ở
  `CHECKING_OLD_SESSION`; một lượt riêng báo chưa nối đúng mạng, các lượt đã
  nối lại vẫn gặp lỗi cleanup. Không gộp lỗi mạng với lỗi hiện tại.
- `pi_read_only_evidence.json`: DONE, audit, 63 dòng shutdown, hash raw Pi và
  source TF. Pi cùng boot cũ, snapshot không có các tiến trình robot được
  kiểm, common lock rảnh. Đây không phải xác nhận vật lý mới của xe/bơm.
- `existing_recovery_tests_final.log`: **48 test PASS**, gồm binding, PID
  reuse/live process, forced cleanup, reboot, fresh STOP proof và archive.
  Các ca cùng boot hiện kiểm hành vi từ chối cleanup cũ không đạt; bộ này chưa
  chứng minh có thể phục hồi ca Pi TF thoát lỗi nêu trên.
- Lượt chạy test đầu có 2 lỗi import vì thiếu đường scripts trong PYTHONPATH;
  thêm đúng scripts của artifact rồi 48/48 đạt, không sửa code/test để qua.
- Bộ duyệt tự động từ chối lệnh tái hiện gọi hàm `prepare` trên state thật
  vì đánh giá có thể chạm luồng STOP trên Pi. Lệnh đó không chạy. Không lách
  bằng thực thi gián tiếp; thay bằng log thật, validator local thuần đọc và
  các fixture đã chạy. Không có phiên STOP mới trong lượt QA.
- `protected_before.json`/`qa_summary.json`: source app, current và pending
  nguyên hash; kiểm lại toàn bộ **364 file R5** đạt. Không build/sync/flash.

## Verify cho bản sửa tiếp theo

Chỉ thêm ca cho behavior phục hồi cần sửa, rồi chạy hồi quy:

```bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/apps/fire_robot_control/tests:$PWD/apps/fire_robot_control/artifacts/59bf2064b462e66b6ead762752ead07550d7a67e5dec9824541251c54efd4a64/ros/scripts"
python3 -B -m unittest test_recovery test_stop_recovery test_reboot_recovery
```

Ca delta bắt buộc: receipt TF lỗi nhưng đủ group/child terminal; group/child
còn sống; PID reuse; thiếu child log; binding/boot thay đổi; sai/chưa có STOP
mới; lỗi cleanup mới; hủy xác minh; bấm lặp; qua gate đầu nhưng thất bại gate
cuối; archive chỉ khi proof đầy đủ, log cũ và map/model/R5 không đổi. Nếu sửa
node, cần thêm bài ROS cô lập cho lỗi shutdown và lỗi trong lúc hoạt động.

Hiện chưa sửa code và chưa yêu cầu user thử chạy robot thêm. Rút/cắm ESP32
không thay receipt lịch sử, nên không tự giải quyết điều kiện chặn này.
