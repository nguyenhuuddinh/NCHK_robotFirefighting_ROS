# Đồng bộ app sang Pi thành công — 29/10/2026

> **Có bản mới sau sửa popup map:** packet `69dfe74e…` đã được sync và kiểm
> receipt thành công. [Kết quả hiện hành](MAP_SELECTION_FIX_2026-10-29.md).
> Bên dưới là bằng chứng sync 750008 trước đó; không dùng receipt khác packet.

**Static sync PASS cho packet
`750008b09dfeed1835cfcd0b52849e0d26f0185910c4f9646b1bda7b9d6717cf`.**
User yêu cầu đồng bộ sau khi sửa F1–F3. Đã chuyển source qua LAN
`pi@10.42.0.185`, build và xác minh receipt. Chưa chạy phiên ROS/robot, NTP,
motor/bơm hoặc nạp firmware; `runtime_verified=false`.

## Kết quả thực tế

| Kiểm tra | Kết quả |
| --- | --- |
| Gói laptop trước sync | Source/binary/archive khớp receipt build; 413 pins |
| Trước sync trên Pi | aarch64; baseline đúng commit, Git sạch; không có tiến trình robot trong danh sách kiểm của runner |
| Chép và build | Thư mục riêng `/home/pi/fire_robot_desktop/750008b09dfe/ros`; `fire_robot_bringup` + `fire_robot_mission` build thành công, tổng 23,6 s |
| Source Pi sau build | 413/413 hash khớp; packet đúng đầy đủ 64 ký tự |
| Bản cài Pi | 103/103 file khớp; `STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED`, `errors=[]`, `live=false` |
| Kiểm tra độc lập sau sync | Đọc lại cả 413 source và 103 file cài bằng SSH; receipt trên Pi, bản tải và bản app lưu có cùng SHA-256 |
| Laptop cùng packet | 95 source/config, static PASS, errors=[] |
| Bộ cũ sau sync | Baseline vẫn commit `6d8cf0f1b15776e695ae4e7868ca4eb1107837fd`, Git sạch; không có robot process trong danh sách kiểm |

Console kết thúc với `PI_STATIC_HASH_PASS=1` và
`PI_STATIC_RECEIPT_VALID=1`, exit code 0. Đây là chứng cứ cài đặt đúng file,
không phải xác nhận robot dừng hay nghiệm thu vận hành.

## Receipt và log

- App đọc receipt tại
  `/home/huudinh/.local/state/fire_robot_control/pi_receipts/750008b09dfeed1835cfcd0b52849e0d26f0185910c4f9646b1bda7b9d6717cf.json`.
- Receipt gốc trên Pi:
  `/home/pi/fire_robot_desktop/750008b09dfe/pi_install_receipt.json`.
- SHA-256 của cả hai receipt:
  `e1484c185384cb3e1f3e60b7c8f630146fc6f811f60d8fb77dc883c6c3b2f89e`.
- `LOG_SYNC=/home/huudinh/.local/state/fire_robot_control/sync_logs/sync_wOTALD`.
- Bản sao bằng chứng trong repo:
  `qa_evidence/pi_sync_20261029_750008b09dfe/`, kèm `SHA256SUMS`.

`build_receipt.json` trong artifact giữ nguyên trạng thái PENDING tại thời
điểm build để không sửa gói niêm phong. Receipt cài Pi thực tế được lưu riêng
ở đường dẫn app đọc bên trên; nó đã được validator của app chấp nhận.

## Bộ lệnh cũ

Script chỉ build app ở thư mục mới, đọc baseline cũ làm underlay. Không sửa
`.bashrc`, service, cấu hình mạng hay firmware. Lệnh/bộ R5 cũ được giữ lại;
muốn dùng chúng thì mở terminal mới, source đúng bộ cũ và ghép laptop/Pi cùng
packet với pairing mới của phiên đó. Không chạy app và bộ cũ đồng thời.
Lượt này chưa chạy lại các lệnh cũ trên robot để nghiệm thu runtime.

## Bước tiếp theo

Không cần sync lại packet này nếu source/binary không đổi và receipt còn hợp
lệ. Giữ nguyên firmware đã được xác nhận cho R5; không tự nạp firmware thử
riêng cho actuator.

Khi user sẵn sàng nghiệm thu có giám sát:

1. Rút LAN, kết nối WiFi robot theo profile hiện hành (Pi `10.0.0.1`, laptop
   `10.0.0.253`). Không tự đổi mạng bằng script.
2. Mở `launchers/fire-robot-control.desktop`; app mở IDLE.
3. Thử quét, lái bàn phím, lưu tên map mới, dừng phiên và đối chiếu receipt
   cùng tình trạng thực tế. Xác nhận sạch rồi mới chuyển chế độ.
4. Chọn map mới/cũ, khởi động chữa cháy, đặt Initial Pose và kiểm tra startup,
   camera/map, X, Q/đóng cửa sổ cùng STOP/cleanup thực tế.

SSH/sudo/pairing khi runtime qua WiFi, SLAM/Nav2/YOLO, TF/QoS, camera latency,
recovery và STOP vật lý vẫn cần nghiệm thu riêng. Static sync không đóng các
mục này. QA offline của bản sửa giữ nguyên phạm vi đã ghi trong
[báo cáo sửa lỗi](QA_FIXES_BEFORE_PI_2026-10-29.md), không cộng sync thành ca
runtime PASS.
