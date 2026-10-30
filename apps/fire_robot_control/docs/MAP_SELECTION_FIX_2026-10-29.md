# Sửa lỗi bấm chọn map ở IDLE — 29/10/2026

**Đã sửa, đóng gói và đồng bộ Pi bản
`69dfe74e2fda9229f89ec103e04fd41ef92da53595b0ded2348f2fbfb6e370ee`.**
User xác nhận danh sách mở nhưng không chọn được map. Không cần quét map mới
để chọn map có sẵn ở IDLE. Chưa chạy phiên robot trong lượt sửa này.

## Nguyên nhân và thay đổi

Supervisor gửi STATUS khoảng mỗi 100 ms. `SessionUi::update()` trước đây
gọi `QComboBox::clear()` rồi thêm lại toàn bộ map ở mỗi STATUS. Việc này xóa
highlight trong popup khi người dùng đang trỏ/bấm một map, dù danh sách không
thay đổi. Bộ QA cũ chọn index bằng code nên chưa bắt hành vi chuột này.

Đã tái hiện bằng popup Qt thật, trỏ dòng thứ hai và nhận bốn STATUS trong
450 ms: highlight từ dòng 2 thành -1. Ca kiểm tra thất bại trước sửa; hai ca
chọn nhanh/cập nhật catalog vẫn qua, không được gọi cả ba là FAIL.

Sửa trong `native_rviz/session_ui.cpp/.hpp`:

- Cache catalog và chỉ tạo lại các item khi nội dung catalog thay đổi.
- Giữ điều hướng chuột/bàn phím trong popup đang mở khi catalog không đổi.
- Sau khi popup đóng, áp dụng lựa chọn được supervisor xác nhận; cập nhật
  catalog/map mới lưu vẫn hoạt động. Update từ server không phát SELECT lặp.

Chỉ hai file UI trên khác gói 750008. Source node/config ROS, watchdog, topic,
map gốc, firmware và code runner không đổi. Hai parent R5 vẫn khớp 364/364
hash; các artifact/receipt cũ được giữ nguyên.

Catalog thật có tám map hợp lệ từ hai thư mục đã cấu hình. Một YAML cũ
`fire_robot_map_2026_09_13.yaml` thiếu ảnh nên bị loại đúng quy tắc; không sửa
file đó hoặc dùng nó làm nguyên nhân của lỗi popup đã tái hiện.

## QA và gói

| Kiểm tra | Kết quả | Giới hạn |
| --- | --- | --- |
| Popup/mouse mới | 3 PASS sau sửa; trước sửa 1 FAIL / 2 PASS | Qt thật, socket giả, localhost domain 211, không bấm Scan/Fire |
| Nhóm pytest kết hợp | 38 PASS / 15,94 s | Ba ca mới + 19 hồi quy F1–F3 + 16 tích hợp cũ; không cộng trùng ba ca mới |
| Binary đóng gói | 1 PASS / 2,25 s | Mở/đóng IDLE, localhost domain 211 |
| Unittest | 124 PASS / 5,055 s | Fake/offline và sync giả |
| CTest | 1 target PASS / 0,41 s | Native hiện tại và build gói cùng pass; không tính rerun thành ca mới |
| Laptop static | 95 file, PASS, errors=[] | Runtime chưa được xác nhận |
| Artifact | 413 pins, source tái dựng cùng packet; binary/archive khớp receipt | Không sửa các gói đã niêm phong |

`tests/qa_map_selection.py` dùng `map_selection_probe` từ
`tests/pre_sync_ui_review/`. Probe mở popup, dùng mouse click và kiểm một
SELECT đúng đường dẫn; kiểm không có SCAN/FIRE, lựa chọn sau ack và map mới.
Nó không click bản đồ, không phát Pose/Goal và không kết nối Pi.

## Đồng bộ và mở lại app

User đã cắm lại LAN theo yêu cầu cập nhật phiên bản. Đã dùng
`sync_pi_candidate.sh` qua `10.42.0.185`:

- Gói riêng trên Pi: `/home/pi/fire_robot_desktop/69dfe74e2fda/ros`.
- Build hai package PASS, tổng 22,3 s.
- Source 413/413 và installed 103/103 khớp hash; script kết thúc
  `PI_STATIC_HASH_PASS=1`, `PI_STATIC_RECEIPT_VALID=1`, exit 0.
- Kiểm tra SSH độc lập sau sync xác nhận lại hash, baseline sạch và không có
  robot process trong danh sách kiểm. Không launch ROS/NTP hoặc flash.
- Receipt ở laptop:
  `/home/huudinh/.local/state/fire_robot_control/pi_receipts/69dfe74e2fda9229f89ec103e04fd41ef92da53595b0ded2348f2fbfb6e370ee.json`.
- SHA receipt Pi và bản laptop đều
  `a9c818ebd5cbc6b8b97b7bc606410a7980bd12c07309dde001162c8012f0910b`.
- `LOG_SYNC=/home/huudinh/.local/state/fire_robot_control/sync_logs/sync_pcdgD3`.

Đã xác minh app cũ giữ khóa đúng PID/start time và không có marker phiên
đang chạy, gửi SIGTERM theo đường đóng chuẩn của supervisor, chờ nhả khóa rồi
mở app gói mới ở IDLE. Không xóa pending marker để ép restart. GUI log mới:
`/tmp/fire-ui-1y50_qh6/gui.log`. Launcher menu dùng `artifacts/current.txt` đã
chọn 69dfe; không cần người dùng nhập đường dẫn artifact.

`build_receipt.json` của artifact giữ PENDING tại thời điểm build; receipt Pi
cài đặt thực tế lưu riêng như trên và đã được validator của app chấp nhận.
Bằng chứng: `qa_evidence/map_selection_fix_20261029/`, kèm SHA256SUMS.

## Cách thử cho người dùng

Ở cửa sổ app mới, chưa cần khởi động robot: mở ô chọn map, di chuyển chuột và
chọn map cũ; tên được chọn phải giữ lại. Việc chọn tên map chỉ chọn dữ liệu
cho phiên chạy; vùng RViz sử dụng `/map` từ ROS khi phiên phù hợp hoạt động.
Sau thử chọn map, rút LAN/kết nối WiFi robot rồi mới tiếp tục các bước vận hành
có giám sát. Camera/TF/Nav2/SLAM và STOP vật lý vẫn chưa được nghiệm thu trong
lượt sửa này.
