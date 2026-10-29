# R5 automatic — source hiện hành ngày 29/10/2026

Snapshot này đưa bộ source đang dùng trong QA lên Git để có thể đọc/review/khôi phục đầy đủ. Source gốc vẫn giữ nguyên tại workspace cục bộ; commit không triển khai hoặc chạy robot.

## Thành phần và định danh

- `ros/`: đủ364 file được pin trong `ros/operational_packet.sha256`, gồm bringup, navigation, perception, mission, driver và tests.
- `deployment/`: công cụ đồng bộ/chạy hiện hành, hash tools và archive chỉ chứa source. Những công cụ này giữ profile/đường dẫn máy đã review.
- `release.json`: model/firmware đi kèm, hash và phạm vi kiểm thử.
- Packet: `0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0`.
- Firmware: ELF `ce19ecd3f4fa2bbabea4c7857723a6e0ba385b6088a271136db79d74f826eee5`, env `freenove_esp32_s3_wroom_n8r2_motor_trace_r5_link_home` trong repo ESP32 đi kèm.
- Model ngoài repo: `best_29_10_2026.pt`, SHA256 `28d3b25f70ccd0ea958a2abfa11f35804f5a82c2f711e4786d5bfa6cce46baca`; model có lớp0=`fire`. Cấu hình mặc định vẫn trỏ tới thư mục model trên laptop.

Các log/bag/ảnh chạy thử, build/install/cache, AI context và trọng số model được giữ ngoài snapshot.

## Hành vi đã chốt trong source

Sau startup/Initial Pose, người dùng đặt goal. Robot tự xử lý lửa cả khi chưa có goal: dừng/hủy goal hiện tại, căn tâm, tiến theo IR, xác nhận đứng yên rồi phun. Mất YOLO quá3s trước phun sẽ kết thúc attempt/trả Nav2; lửa xuất hiện lại có thể mở attempt mới. Lệch tâm khi tiến sẽ căn lại.

YOLO confidence0,15, xác nhận3/5 frame. IR trước phun cần3 sequence mới và dữ liệu≤500ms. IR automatic không còn cap0,30m/15s. Mỗi lượt phun6s từ phản hồi PWM ON của ESP32; YOLO/IR thay đổi trong ON không tự cắt lượt. Tối đa3 lượt liên tiếp; căn/tiến lại thực sự hoàn tất mới cấp lại quota. OFF đánh giá lại mục tiêu ngay, lượt ON mới vẫn chờ home/ready. X hủy, Q/ESC STOP. Phục hồi serial giữ Nav2/RViz/camera/định vị, không tự yêu cầu Pose mới.

## Kiểm tra đã có và giới hạn

- 364/364 source hashes khớp packet;95/95 installed source hashes laptop khớp.
- Kế thừa stationary reentry QA50 delta +71 hồi quy liên quan; source điều khiển không đổi khi thay model.
- Model mới load/forward CPU trên một ảnh giả PASS;15 kiểm launch/tools và7 kiểm chọn map offline PASS.84 kiểm session tools nền PASS trước khi commit.
- Firmware/source69 file khớp bộ đã upload; snapshot ESP32 lưu hash riêng.
- **Pi receipt cho packet0c54 còn PENDING; runtime robot/model mới chưa kiểm.** Độ chính xác, FPS/CUDA camera cần quan sát thực tế. USB/TF và Nav2 cleanup còn mục OPEN trong lịch sử; không coi snapshot này là nghiệm thu toàn bộ robot.

## Đọc và build source

Kiểm source, không chạy robot:

```bash
cd "$(git rev-parse --show-toplevel)/releases/r5_20261029/ros"
python3 scripts/r5_packet.py
```

Build cần ROS2 Humble và các dependency/underlay sẵn có trên đúng host:

```bash
source /opt/ros/humble/setup.bash
cd "$(git rev-parse --show-toplevel)/releases/r5_20261029/ros/ros2_ws"
# Source underlay phù hợp của Pi/laptop nếu dependency nằm ở workspace khác.
colcon build --symlink-install --packages-select fire_robot_bringup fire_robot_mission
```

Laptop còn cần các package perception/navigation, dependency Nav2 và venv YOLO theo source. Không build/chạy YOLO trên Pi. Build mới có thể tạo artifact khác; ELF đã nạp được nhận dạng bằng hash ghi ở trên, không giả là mọi rebuild đều tái tạo byte giống nhau.

## Công cụ chạy trên máy dự án

Deployment scripts hiện trỏ tới `/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/.codex_qa_backups/r5_operational_candidate_20261026/ros` và evidence `evidence_r5_5_model_29_10_2026`. Chúng là bản lưu của profile máy đang dùng; clone sang vị trí khác cần cấu hình lại đường dẫn/underlay trước khi dùng. Không chép đè một phiên đang chạy.

Với workspace dự án đã có đúng candidate, đồng bộ file một lần bằng `deployment/sync_pi_files.sh`; chỉ source/build/hash. Cần `PI_STATIC_HASH_PASS=1` và `R5_MODEL_29102026_PI_TOOLS_READY=1`. Firmware ce19 không cần nạp lại chỉ vì model/map.

Từ thư mục `releases/r5_20261029`, Terminal1 dùng `bash deployment/run_pi.sh 10.0.0.1`. Lấy pairing vừa in. Terminal2 chọn một trong hai map:

```bash
bash deployment/run_laptop.sh /home/pi/qa_logs/r5_automatic_THAY_BANG_LUOT_MOI/pairing.json 10.0.0.1 fire_room_9_10_2026
bash deployment/run_laptop.sh /home/pi/qa_logs/r5_automatic_THAY_BANG_LUOT_MOI/pairing.json 10.0.0.1 fire_room_TN1
```

Chỉ chạy một lệnh Terminal2 cho mỗi phiên. Đổi map không cần sync Pi: Nav2 đọc YAML/PGM trên laptop. Kết thúc phiên cũ rồi đặt Initial Pose theo map mới. Profile runtime hiện tại dùng WiFi AP Pi10.0.0.1/laptop10.0.0.253, LAN tắt; LAN dùng cho đồng bộ file. Các lệnh trên là tùy chọn do người vận hành chạy.
