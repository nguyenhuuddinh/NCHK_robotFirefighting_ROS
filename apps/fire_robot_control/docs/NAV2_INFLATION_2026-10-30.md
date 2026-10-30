# Giảm vùng chi phí né vật cản cho app — 2026-10-30

Yêu cầu: giữ nguyên chiều dài/rộng của xe, giảm vùng chi phí khiến đường đi
né xa. Đây là tinh chỉnh của app; không chỉnh cấu hình hoặc lệnh R5 gốc.

## Thay đổi

| Thông số | Trước | Sau |
| --- | ---: | ---: |
| Local inflation radius | 0,55 m | 0,40 m |
| Global inflation radius | 0,55 m | 0,40 m |
| Local cost scaling factor | 3,0 | 3,0 |
| Global cost scaling factor | 5,0 | 5,0 |
| Padding | 0,015 m | 0,015 m |

Footprint giữ đúng `[[0.19, 0.10], [-0.15, 0.10], [-0.15, -0.10], [0.19, -0.10]]`:
dài 0,34 m, rộng 0,20 m. RPP, kiểm tra va chạm, giảm tốc gần vật cản,
tốc độ, topic/QoS, watchdog và mapping giữ nguyên.

`config/nav2_tuning.yaml` là cấu hình riêng của app. Khi đóng gói,
`services/app_nav_params.py` tạo một file Nav2 đầy đủ từ parent R5, chỉ thay
hai bán kính. Builder từ chối các khóa ngoài phạm vi và bán kính không hợp lệ,
nhỏ hơn bao xe có padding hoặc khoảng giảm tốc RPP hiện tại. Đây là kiểm tra
nhất quán cấu hình, không phải chứng nhận khoảng hở khi xe chạy thật.

`DesktopHooks.commands()` truyền `params_file:=.../desktop/generated/app_nav2_params.yaml`
cho Nav2 khi mở phiên chữa cháy. File đầy đủ cần thiết vì costmap con trong
Humble phải nhận cùng YAML với container. Mapping dùng runner riêng, không
nhận file này. Không chỉnh file trong artifact đã niêm phong hoặc YAML R5 gốc.

## QA và bằng chứng

- 184 ca Python đạt, 2 ca không áp dụng, 0 lỗi còn lại; đếm theo testcase
  duy nhất sau lượt kiểm tra lại, không cộng trùng.
- Qt CTest đạt; bản đóng gói mở/đóng IDLE trong localhost domain 211 đạt.
- So sánh toàn cây YAML: chỉ hai giá trị inflation radius thay đổi.
- Kiểm tra `RewrittenYaml` thật của Humble: hai costmap con nhận 0,40 m,
  map được chọn và `use_sim_time:=false` vẫn đúng; đường dẫn có khoảng trắng đạt.
- Hệ số local 3,0 vẫn khớp `FollowPath.inflation_cost_scaling_factor`; khoảng
  giảm tốc `cost_scaling_dist=0,30 m` không vượt radius mới.
- Kiểm tra hồi quy generated runner, mapping, tốc độ teleop, lưu map,
  Nav2 teardown/Q, X theo các pha, cancel/Goal và bảo vệ bằng chứng dừng.
- 364 file parent R5 giữ byte/hash; laptop static 95 file đạt.
- Mapping runner SHA giữ `7d769dce6f0170a7ee8c0ad798009593755202023befefef64f5659a990a2503`.
- Lượt QA đầu có hai lỗi môi trường: sandbox chặn Unix socket của fixture
  và thiếu biến domain 211 bắt buộc của bài teleop. Chạy lại ba ca mapping với
  socket cục bộ và ROS localhost 211 đều đạt; không sửa logic mapping.
- Lượt thử cấu hình ban đầu cần chuyển thư mục log ROS sang `/tmp`, vì
  thư mục `~/.ros` nằm ngoài vùng được ghi. Không phải lỗi runtime của app.

Bằng chứng: `qa_evidence/nav_inflation_20261030/summary.json`,
`regression.xml`, `mapping_isolated.xml`, `packaged_smoke.xml`,
`laptop_static.json`, `release_review.json`, `runtime.diff` và `package.log`.
Trạng thái đồng bộ/chọn bản được ghi riêng trong `selection.json` khi hoàn tất.

## Gói và cách dùng

- Gói trước: `ad0a51a757b29651c331361c10eac147cbc093de2d5a5f6f265c4d19a7f33ca3`.
- Gói mới: `eaa652f47fb18127ee48fcb4d27ef657db0ebc25d6fa914fdfbde8828e54a6b2`.
- Model app giữ `best_minireplay.pt`.
- Đã đồng bộ tĩnh qua LAN `10.42.0.185`: 2 package build đạt, 449 source
  pins và 103 file cài trên Pi xác minh đạt, receipt đúng gói mới.
- Đã chọn gói mới trong `artifacts/current.txt` sau khi xác minh không có
  `pending.json` và khóa runner đang rảnh. Cửa sổ app cũ vẫn giữ bản trước;
  không tự đóng/mở hoặc chạy robot trong lượt này.
- Chỉ chọn gói mới sau khi có receipt Pi đúng packet và không có phiên đang chạy.
- Cửa sổ app cũ không được thay đổi cấu hình giữa phiên. Đóng cửa sổ cũ và
  mở lại app sau khi chọn bản mới; bản đồ đã lưu vẫn dùng được.

Chưa chạy xe/bơm để xác nhận khoảng hở thực tế. Khi thử, kiểm tra costmap,
đường đi cùng map/Goal, góc tường và lối hẹp. Giảm radius thu nhỏ phạm vi chi phí;
không bảo đảm khoảng cách xe–tường cố định 0,40 m hoặc mọi đường sẽ sát hơn.
Nếu đường vẫn vòng xa, cần đối chiếu costmap và nguồn vật cản trước khi giảm tiếp.

## Lệnh verify offline

```bash
source /opt/ros/humble/setup.bash
source /home/huudinh/.codex/worktrees/laptop-baseline/NCHK_robotFirefighting_ROS/ros2_ws/install/local_setup.bash
source .codex_qa_backups/r5_operational_candidate_20261026/ros/ros2_ws/install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/apps/fire_robot_control/tests:$PWD/releases/r5_20261029/ros/scripts:$PYTHONPATH"
export ROS_LOG_DIR=/tmp/fire_robot_nav_inflation_verify
export ROS_DOMAIN_ID=211
export ROS_LOCALHOST_ONLY=1
python3 -B -m pytest -q apps/fire_robot_control/tests/test_app_nav_params.py apps/fire_robot_control/tests/test_app_entry.py apps/fire_robot_control/tests/test_app_model.py
```

Các bài QA dùng nguồn giả và localhost; không dùng chúng để thay thế bài kiểm
tra khoảng hở trên robot thực tế.
