# Thu nhỏ riêng global inflation xuống 0,25 m — 31/10/2026

Người vận hành yêu cầu thử đường đi gần vật cản hơn sau mốc source 0,30 m
đã push. Thay đổi thuộc app, không chỉnh lệnh hoặc source R5 gốc.

## Phạm vi

- Global `inflation_radius`: 0,30 → 0,25 m.
- Local `inflation_radius`: giữ 0,30 m. RPP dùng costmap local để tính khoảng
  cách giảm tốc; `cost_scaling_dist=0,30 m` và scaling local 3,0 giữ nguyên.
- Scaling global 5,0, footprint 34 × 20 cm, padding 1,5 cm, collision detection,
  tốc độ, TF/QoS/watchdog, model và toàn bộ logic phiên/X/Q giữ nguyên.
- BT replanning và lookahead chưa chỉnh. Đường đổi bên/vòng cua rộng còn OPEN;
  giảm inflation không được coi là đã sửa hai hiện tượng đó.

Trong `services/app_nav_params.py`, kiểm tra khoảng giảm tốc RPP trước đây
được áp cho cả hai costmap. Chỉnh để điều kiện này chỉ áp cho local; cả hai
vẫn kiểm tra bán kính bao footprint có padding. Đây là sửa phạm vi kiểm tra
cấu hình, không tắt ràng buộc giảm tốc local hoặc thu nhỏ hình xe.

Nguồn tham chiếu: [RPP Humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/README.md),
tham số `cost_scaling_dist`/`inflation_cost_scaling_factor` gắn với local costmap.

## QA

- Ca cấu hình local 0,30/global 0,25 đã tái hiện FAIL trước sửa do ràng buộc
  RPP áp nhầm global; sau sửa PASS. Giữ lại log/XML trước sửa, không đổi
  lịch sử lỗi thành đạt.
- Ca âm xác nhận local 0,25 vẫn bị từ chối vì không phủ khoảng giảm tốc
  0,30 m; global 0,23 bị từ chối vì nhỏ hơn bán kính bao xe có padding.
- 23 ca source/launch/runner/model đạt, CTest Qt đạt. So sánh toàn cây YAML
  với bản 0,30 m: chỉ một giá trị global radius thay đổi.
- YAML RewrittenYaml thật của Humble giữ đúng hai costmap, map lựa chọn,
  thời gian thực, đường dẫn có khoảng trắng và các kiểm tra va chạm.
- Chỉ service sinh cấu hình `app_nav_params.py` thay đổi; service quản lý phiên,
  STOP, mapping và toàn bộ 364 file parent R5 giữ byte/hash.
- Bản đóng gói có kiểm tra IDLE riêng ở localhost domain 211; kết quả cuối,
  static và trạng thái sync/chọn bản ghi trong `qa_evidence/nav_inflation_global025_20261031/`.
- Tổng kiểm tra sau sửa: 24 ca đạt, 0 bỏ qua/lỗi; 23 source/launch/runner/model
  và 1 mở/đóng IDLE. Native CTest đạt, laptop static 95 file đạt.
- Đồng bộ LAN: 2 package build đạt (9,01 s), 449 source pins và 103 file cài
  Pi xác minh hash đạt; receipt đúng packet. Đã chọn bản mới sau kiểm tra
  không có pending và khóa runner rảnh. Lần SSH đầu khi mạng chưa sẵn sàng
  được giữ trong `pi_sync.log`, lượt thành công trong `pi_sync_retry1.log`.
- Chưa chạy xe hoặc đo khoảng hở thực tế. Điều kiện bao footprint của builder
  là kiểm tra nhất quán cấu hình, không thay thế thử nghiệm phần cứng.

## Phiên bản và khôi phục

- Bản trước: `6133ff387973f5c7ccc6b245b6acaa1c6d74126ea3f2f5b0eb4f75a376f60426`.
- Bản mới: `33f9f7db817fb044da8278bdcd7cf8df18b421290bddf424ab4539f5feed1750`.
- Bản trước đã lưu trên GitHub tại tag `desktop-inflation-030-source-pass-v1`;
  `checkpoint.json` vẫn mô tả đúng mốc này, không sửa nội dung tag đã push.
- Không chọn bản mới khi thiếu receipt Pi đúng packet hoặc còn phiên pending.
  Xem `selection.json` khi việc đồng bộ/chọn bản hoàn tất.
- Cửa sổ cũ vẫn dùng packet cũ. Sau khi đồng bộ và chọn bản mới, đóng app cũ
  và mở lại để nhận cấu hình. Không thay parameter trực tiếp giữa phiên.

0,25 m là phạm vi lớp chi phí quanh ô vật cản, không bảo đảm khoảng hở
xe–tường 25 cm. Nếu đường vẫn né xa hoặc đổi bên, cần quan sát costmap/path
thực trước khi giảm tiếp. Không tự giảm footprint/padding hoặc bỏ collision
detection để đạt khoảng cách nhỏ hơn.

## Lệnh kiểm tra

```bash
source /opt/ros/humble/setup.bash
source /home/huudinh/.codex/worktrees/laptop-baseline/NCHK_robotFirefighting_ROS/ros2_ws/install/local_setup.bash
source .codex_qa_backups/r5_operational_candidate_20261026/ros/ros2_ws/install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/apps/fire_robot_control/tests:$PWD/releases/r5_20261029/ros/scripts:$PYTHONPATH"
export ROS_LOG_DIR=/tmp/fire_robot_global025_qa
python3 -B -m pytest -q apps/fire_robot_control/tests/test_app_nav_params.py apps/fire_robot_control/tests/test_app_entry.py apps/fire_robot_control/tests/test_app_model.py apps/fire_robot_control/tests/qa_candidate_runner.py
```
