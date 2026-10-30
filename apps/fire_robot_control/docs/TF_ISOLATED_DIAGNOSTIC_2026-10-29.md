# Bộ đo TF riêng và kết quả QA — 29/10/2026

User cho phép tiếp tục chẩn đoán nhưng yêu cầu bảo toàn R5. Đã thêm tool tại
`diagnostics/tf_startup/`, ngoài services/native_rviz và ngoài packet app hiện
tại. Không sửa source R5/app/firmware, không đổi gate/STOP/watchdog, không
chạy runner robot hoặc phần cứng. Chỉ dùng dữ liệu giả trong localhost213.

## Kết quả quan trọng

**Đã tái hiện cơ chế backlog của bộ quan sát bằng positive control.**
Thêm25ms xử lý giả vào mỗi vòng observer: TF lúc phát và listener độc lập
vẫn có tuổi nguồn p95 khoảng86ms, nhưng callback/buffer của observer theo
vòng R5 có tuổi gần4,8s. Khi chỉ giảm tần suất sample/ghi file trong process
chẩn đoán xuống khoảng5Hz, callback/buffer trở lại khoảng84ms.

Điều này chứng minh vòng spin một callback rồi sample/ghi file có thể tạo
đúng hình thái “odom mới, TF trong observer cũ” khi không xử lý kịp callback.
Không cần giả lập ESP lỗi để tạo hình thái đó. **Chưa chứng minh Pi trong
lượt robot trước đã có đúng nguyên nhân/tải này.** Không gọi positive
control là tái hiện phần cứng hoặc nghiệm thu R5.

Fixture tự nhiên trên laptop và Pi chưa gây độ trễ4–5s; mô hình reader phụ
nhẹ cũng chưa tạo khác biệt đủ lớn. Chưa có cơ sở kết luận RViz mở sớm là
nguyên nhân duy nhất. Mô hình chưa có camera thật, USB/serial thật, WiFi
laptop–Pi, rosbridge hoặc toàn bộ RViz/Qt.

## Phạm vi và ba điểm đo

- Import class OdomToTfBroadcaster và StartupHealthNode từ source R5 đã pin
  packet0c54, kiểm đúng origin/module và hash trước/sau. Không đổi file hoặc
  monkeypatch tiến trình R5 đang vận hành.
- Trong process fixture mới, bọc sendTransform để ghi lúc phát và gọi method
  thật. Trong process observer mới, bọc Buffer.set_transform để ghi callback
  rồi gọi method thật. Sample/lookup dùng code nguyên byte của R5.
- Listener độc lập spin liên tục cùng TF làm đối chứng. Record cả monotonic,
  wall clock, source stamp, sample/write cost. Trace có giới hạn20.000 event/
  worker; thiếu dữ liệu/tràn trace báo fail. Không ghi file cho mỗi event;
  chỉ giữ RAM rồi xuất khi worker dừng. Health JSON vẫn ghi theo chế độ thử.
- Chế độ callback dùng đúng rclpy.spin_once như R5, gồm add/remove global
  executor mỗi vòng. Không thay bằng executor luôn attach rồi gọi đó là R5.
- Chế độ timer và delay giả là biến thể chẩn đoán, không triển khai cho R5.
- TF20Hz/odom10Hz lặp stamp: không dùng stamp như ID duy nhất để khẳng định
  latency từng message; so source age tại mỗi điểm, giữ raw event để review.

## Kết quả đo

Các số dưới là p95 tuổi nguồn TF, đơn vịms, bỏ2s đầu discovery.

| Host/case | Tại nguồn phát | Listener nhanh | Callback health | Lookup health |
| --- | --- | --- | --- | --- |
| Laptop, callback, không reader phụ | 86,44 | 87,06 | 87,03 | 102,84 |
| Laptop, callback, reader phụ sớm | 86,80 | 87,38 | 87,49 | 103,22 |
| Laptop, sample5Hz, reader phụ sớm | 86,54 | 87,20 | 87,19 | 100,47 |
| Laptop, callback, delay giả25ms | 86,00 | 86,62 | 4782,30 | 4804,55 |
| Laptop, callback, delay25ms + reader phụ | 85,21 | 85,84 | 4813,21 | 4787,22 |
| Laptop, sample5Hz, delay25ms + reader phụ | 82,97 | 83,55 | 83,57 | 84,06 |
| Pi, callback, không reader phụ | 73,60 | 76,65 | 78,02 | 145,14 |
| Pi, callback, reader phụ sớm | 71,09 | 74,29 | 75,52 | 147,06 |
| Pi, sample5Hz, reader phụ sớm | 64,69 | 68,01 | 68,39 | 103,58 |

Laptop thường7s/case, positive control8s/case, Pi10s/case. Chỉ là capture
ngắn, không coi phân vị này là acceptance loaded-WiFi. Không so trực tiếp
phân vị giữa các host để suy ra độ trễ mạng. Worker cùng case trên cùng host.

Pi callback/không reader: sample p95=7,42ms, write p95=3,04ms. Đây là hai
phân vị riêng, không cộng để tuyên bố một vòng cụ thể có tổng như vậy.
Trên laptop, sample p95 khoảng0,58ms và write0,285ms (xem raw chính xác).
Khả năng vượt ngân sách callback khi Pi có tải thêm cần đo ở phiên lỗi thật.

## QA bảo vệ R5 và công cụ

- 12 pure unit/guard PASS: domain robot/nonlocal bị từ chối; packet đổi,
  nội dung source đổi, manifest traversal/duplicate; hiệu chỉnh lead100ms;
  percentile/NaN; trace cap; không ghi đè log; bỏ discovery; không giả định
  stamp duy nhất; output trong R5 bị chặn; duration/delay bị giới hạn;
  snapshot robot đang chạy/không rõ bị chặn; worker fail thu dọn đúng.
- Các case ROS synthetic phía trên đều kết thúc RC0, không mất event.
- Hủy khi3 worker đã sẵn sàng: master kết thúc, cả3 worker được reap,
  source hash nguyên. Không tín hiệu tới tiến trình robot.
- Khởi động từ chối nếu R5/robot đang chạy. Kiểm lại0,5s/lần, nếu phát hiện
  R5 thì chỉ kết thúc worker của chính tool. Không giữ common lock của R5.
- Worker tự ép domain213/localhost1 và FastDDS cấu hình tường minh, loại
  XML transport tùy ý của môi trường cha; không đổi shell/cấu hình sản xuất.
- Không serial/camera/Lidar thật, không pump/fire/goal publisher. Fixture
  tạo odom/scan/identity/OFF/cmd-zero giả **chỉ trong domain213** để thử đúng
  contract. Fake BOOT/ELF không phải chứng cứ firmware hoặc STOP phần cứng.
- README ghi rõ overhead CPU và cấm chạy đồng thời với R5. Guard giảm rủi ro
  chạy chồng, không có nghĩa tool tiêu thụ0 tài nguyên.

## Xác minh bảo toàn

| Bộ được bảo vệ | Packet | Kết quả |
| --- | --- | --- |
| Release R5 local | 0c54d9b79765… /364pins | Trước/sau khớp byte |
| Candidate R5 local | 0c54d9b79765… /364pins | Trước/sau khớp byte |
| Artifact app hiện tại | 6734c2846c46… /416pins | Trước/sau khớp byte |
| Stage R5 Pi | 0c54d9b79765… /364pins | Trước/sau khớp byte, bad=[] |

artifacts/current.txt vẫn chọn6734. Không build/sync app hoặc R5; chỉ copy4
file tool vào `/home/pi/fire_robot_diagnostics/tools_c93d1e5a45b0/`.
Pi fixture output ở `/home/pi/fire_robot_diagnostics/run_c93d1e5a45b0_normal`.
Sau test pgrep chẩn đoán RC1, không còn worker. Chưa nạp firmware/actuator,
chưa sửa hoặc xóa pending thật. Giữ nguyên lệnh khởi động R5 cũ.

## Cần gì để chốt lỗi gốc trên robot

Bộ đo hiện xác minh được các nhánh latency bằng nguồn giả. Để quy nguyên
nhân cho lượt robot thật, cần cùng các mốc send/callback/lookup và chi phí
sample/write ở một phiên chẩn đoán đầy đủ, có log whitelist môi trường/tải
và reader count. Phải dùng bản chẩn đoán/candidate riêng, user giám sát khi
chạy phần cứng; không tự gắn vào R5 đang hoạt động và không coi subscriber
bổ sung là hoàn toàn không có overhead.

Nếu nguồn và listener độc lập vẫn mới nhưng health cũ như positive control,
ưu tiên tách nhịp sample/ghi file khỏi nhịp spin trong **bản thử riêng**, giữ
mọi ngưỡng admission/STOP. Nếu TF đã cũ ngay tại send thì đi ngược về
odom/broadcaster. Nếu cả listener độc lập cũng trễ thì cần rà DDS/scheduling.
Hiện chưa promote biến thể5Hz thành bản sửa R5 hoặc app vận hành.

[Hướng dẫn và lệnh tool](../diagnostics/tf_startup/README.md).
[Raw/hash bằng chứng](qa_evidence/tf_isolated_diagnostic_20261029/SHA256SUMS).
