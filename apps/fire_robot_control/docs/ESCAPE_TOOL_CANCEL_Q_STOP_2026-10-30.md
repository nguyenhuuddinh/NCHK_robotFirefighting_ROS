# Esc bỏ công cụ RViz, Q dừng phiên

User chọn Goal rồi muốn bỏ công cụ bằng Esc, nhưng app coi Esc là STOP. Đây là
xung đột phím ở lớp giao diện. Nhãn `RECOVERY_PI_CLEANUP_MISSING` trong ảnh là
kết quả kiểm tra phiên đã dừng sau đó; riêng ảnh không đủ kết luận vì sao receipt
Pi thiếu. Bản sửa phím không bỏ guard hoặc sửa receipt/pending để vượt BLOCKED.

## Hành vi mới

- **Esc trong RViz:** trả về công cụ mặc định (Move Camera), không gửi STOP/CLOSE
  và không làm phiên chuyển STOPPING. Có kiểm cả Goal và Initial Pose chưa đặt.
- **Q** hoặc nút **DỪNG PHIÊN (Q):** giữ nguyên luồng dừng có xác minh.
- **X:** giữ nguyên hủy tác vụ. Đóng cửa sổ vẫn đi qua luồng dừng hiện có.
- Trong khung bàn phím quét, Esc không gửi STOP và không đóng khung. Q và đóng
  khung vẫn yêu cầu dừng phiên quét. K/Space dừng xe giữ nguyên.
- Popup/hộp nhập liệu dùng Esc theo Qt bình thường. Esc hủy hộp lưu map không
  dừng phiên. Luồng hủy xác thực SSH/sudo và dừng khi mất UI vẫn giữ nguyên.

## Thay đổi

`native_rviz/session_ui.cpp`: bỏ shortcut Esc của cửa sổ chính và nhánh Esc gọi
STOP trong KeyboardDialog; chỉ đăng ký Q, cập nhật nhãn dừng. Không chặn Esc ở
main frame nên RViz tự xử lý thoát công cụ.

`native_rviz/main.cpp`: bỏ Esc ở bộ lọc phím cửa sổ RViz con dùng khi mapping,
cập nhật nhãn và dòng hướng dẫn. Không thay key handler RViz, quyền Goal/pose,
điều khiển xe hoặc topic.

Đã sửa hướng dẫn offline/README cho Q. Các tài liệu lịch sử và artifact cũ giữ
nguyên để phản ánh đúng hành vi ở phiên bản đó.

## QA

Probe Qt/RViz thật, fake supervisor qua Unix socket, ROS localhost domain 211:
**21 kiểm tra đạt**, gồm Esc trên Goal, Initial Pose, công cụ mặc định, khung
bàn phím quét, hộp lưu map và bộ lọc cửa sổ mapping con. Kiểm Q từ RViz, bàn
phím mapping và nút STOP vẫn gửi yêu cầu; nhãn không còn Esc. Bốn ca bổ sung
bấm nút Goal trên thanh công cụ thật, xác nhận công cụ được kích hoạt rồi
Esc trả về công cụ mặc định và không gửi STOP/CLOSE.

Probe gọi QTest key event thật trên RenderPanel, kiểm currentTool trở lại default
và đọc IPC để chắc không có STOP/CLOSE. Không giả lập kết quả bằng tìm chuỗi
trong source. Log/result: [thư mục bằng chứng](qa_evidence/escape_q_only_20261030/).

**291 unittest PASS**, 15,725 s. Probe/layout/Qt fixture cũ đã rebuild từ source
mới trước khi chạy hồi quy, tránh dùng binary cũ để báo đạt.

**253 ca pytest PASS**, 139,49 s, một warning Python đã có ở kiểm hiển thị.
Lượt này gồm 236 ca hồi quy và 17 ca phím ban đầu. Lượt bổ sung cuối
`key_toolbar_final.log`: **21 PASS**, 2,24 s, gồm 17 ca trùng và 4 ca thanh
Goal mới; tổng **257 ca tích hợp khác nhau** qua các lượt, không phải một
lệnh chạy 257 ca. Source vận hành không đổi sau lượt 253 ca.

Lượt probe đầu trước sửa trỏ nhầm đường config nên RViz nạp mặc định, đồng thời
tiêu chí child key bị ảnh hưởng focus. Giữ `native_before.log` làm chẩn đoán,
không coi là ma trận trước/sau đầy đủ. Probe cuối dùng đúng `operational.rviz`,
focus cửa sổ child riêng. Lượt thêm thanh Goal ban đầu bị fixture thu hồi
quyền Goal do không có observer thật; kết quả `toolbar/` được giữ. Fixture
cuối dừng timer refresh của frame, cấp quyền giả rõ ràng, kiểm bấm Goal
kích hoạt thật trước khi bấm Esc; cả 21 kiểm tra đều đạt. Không sửa quyền
Goal của code vận hành để làm test đạt.

Gói mới `59bf2064b462e66b6ead762752ead07550d7a67e5dec9824541251c54efd4a64`:

- Build native và CTest **PASS**, gói có **441 file source được ghim hash**.
- Smoke mở/đóng binary đóng gói ở IDLE: **1 PASS**, 2,72 s; không tạo phiên.
- Operator thật localhost nhận Q: **PASS**, không có telemetry nên trả rc2 và
  STOP chưa xác nhận đúng thiết kế; không còn tiến trình con hoặc socket.
  Lượt đầu nạp nhầm overlay workspace nên không mở được helper socket;
  chạy lại đúng hai overlay của launcher đã đạt, giữ cả hai log để đối chiếu.
- Inventory laptop: **95 file cài đúng hash**. Inventory R5 giữ kiểm model R5
  gốc; kiểm model app `best_minireplay.pt` riêng trong `app_model.json` đạt.
- Sync LAN `10.42.0.185`: build hai package tại thư mục riêng
  `/home/pi/fire_robot_desktop/59bf2064b462/ros`, **103 file cài Pi đúng hash**;
  receipt hợp lệ và đã lưu trên laptop. Không chạy launch/node trên Pi.
- Đã chọn `artifacts/current.txt` sang gói mới sau kiểm receipt, lock và không
  có tiến trình robot. Không tự mở/đóng cửa sổ app đang dùng.

`protected_before_sync.json` đối chiếu source gói cũ/mới: chỉ hai file native
nêu trên khác nhau. **364 file R5 nguyên hash**, toàn bộ service Python không
đổi; map TN1/TN2/temp, model, pending và mọi file bảo vệ còn lại giữ nguyên
(marker current chỉ đổi sau khi đủ điều kiện). `selection.json` ghi bản trước
`49c68b77…`, bản mới và trạng thái chưa nghiệm thu phần cứng.

## Bảo toàn và sử dụng

Không sửa service Python, lệnh/bộ release R5, watchdog/STOP policy, launch Nav2,
quét/lưu/chọn map, model `best_minireplay.pt`, map TN1/TN2/temp của user hoặc
pending. Không SSH chạy node, motor/bơm, đổi giờ/NTP hoặc flash trong lượt này.

Sau cập nhật, cần đóng app cũ và mở lại mới nhận keymap mới. Nếu phiên cũ còn
BLOCKED, dùng Kiểm tra lại hoặc Xác minh dừng theo bằng chứng hiện có; không xóa
pending thủ công. Esc chỉ bỏ công cụ đang chọn, không hủy một goal đã gửi; X vẫn
là thao tác hủy tác vụ hiện tại.
