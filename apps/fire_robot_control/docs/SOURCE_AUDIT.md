# Khảo sát source trước khi lập kế hoạch

Ngày29/10/2026. Chỉ đọc filesystem/Git và kiểm hash local; không truy cập Pi.
Repo gốc: `/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS`.
Các đường dẫn tương đối trong bảng dưới tính từ repo gốc, trừ ký hiệu riêng.

## 1. Snapshot hiện tại

| Mục | Kết quả đọc local |
| --- | --- |
| Branch | `codex/nav2-phase3-3` |
| HEAD | `cc6bcf11b03952b5d3c5719750a19c2762e87ff5` |
| Worktree trước khi viết tài liệu | `?? output/`; không sửa/xóa nội dung đó |
| Candidate `C` | `.codex_qa_backups/r5_operational_candidate_20261026/ros` |
| Snapshot Git `S` | `releases/r5_20261029/ros` |
| Packet của C và S | `0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0` |
| Hash file | Đọc từng manifest, C364/364 và S364/364 khớp, không có mismatch |
| Firmware | `ce19ecd3f4fa2bbabea4c7857723a6e0ba385b6088a271136db79d74f826eee5` |
| Model | `best_29_10_2026.pt`; SHA `28d3b25f70ccd0ea958a2abfa11f35804f5a82c2f711e4786d5bfa6cce46baca` theo release metadata; không load model lượt này |
| Pi receipt0c54 / runtime | PENDING / NOT_TESTED theo PROJECT_CONTEXT và release.json; không đọc filesystem Pi |

Không lặp các QA50/71/15/7/84 đã ghi ở lịch sử. Hash local chỉ xác nhận bản
source đang lập kế hoạch, không nghiệm thu behavior/UI hoặc độ chính xác YOLO.

## 2. Tài liệu đã dùng

- `AGENTS.md`, `docs/codex/PROJECT_CONTEXT.md`, core/ROS2/QA role và chuẩn workspace.
- `docs/codex/reference/QA_CHECKLIST.md`, `reference/TASK_BRIEF_TEMPLATE.md`.
- Roadmap22/10, R4_COMPLETED_ACCEPTANCE26/10 và phần kiến trúc/behavior liên quan
  của R5_INTEGRATION_DETAILED_PLAN/R5_BEHAVIOR_AND_QA_CONTRACT26/10.
- Receipt stationary121336 và QA50 dùng để hiểu parent8944, không coi là0c54.
- `R5_5_MODEL_29102026_MAP910_SYNC_AND_RUN_2026-10-29.md`, model ready và
  `R5_5_GITHUB_SOURCE_PUBLICATION_2026-10-29.md` cho trạng thái mới nhất.

Các tài liệu lịch sử dài có nhiều marker cũ; ưu tiên marker hiện hành và code.
Không tuyên bố đã đọc lại tuần tự mọi dòng history/ledger.

## 3. Entry point và nguồn thật cần tái sử dụng

| Đường dẫn | Nội dung đã xác minh | Hệ quả cho app |
| --- | --- | --- |
| `scripts/start_mapping.sh` | Source Humble+root install, gọi `fire_robot_session.py mapping` | Cần launcher có môi trường/TTY rõ; không chạy bằng Python mặc định của venv YOLO |
| `scripts/fire_robot_session.py` hàm `mapping/save_map` | Mở SLAM+RViz, foreground teleop; save kiểm SLAM có/Nav2 không; map_saver timeout25s; không ghi đè; ghi selected_map.txt | Có phần reuse; root mapping còn dùng Pi startup cũ, không ngang R5 pairing |
| `C/scripts/r5_laptop_session.py` | Clock→PiIO→camera→Pose/Nav2/RViz→YOLO→consumers/operator; `--map` nhận Path; cleanup local trước Pi | Không rút gọn bằng cách bật hàng loạt launch song song |
| `C/scripts/r5_pi_session.py` | Lock, pairing/mailbox, Pi health, camera15FPS, async/wifi profile | App quản lý đúng một Pi runner và binding thực |
| `C/scripts/operational_peer.py` | Binding gồm session/secret/packet/boot/PID-start/root; local SSH exit không là remote STOP proof | Không log secret; timeout mutation không replay mù |
| `C/scripts/operational_entry.py`, `operational_owned_process.py`, `operational_stack.py` | Owned process group, kiểm PID/start_ticks, streaming logs, STOP/child/remote receipts riêng | Dùng lại cleanup; QProcess chỉ quản lý child không đủ chứng minh toàn stack |
| `releases/r5_20261029/deployment/run_laptop.sh` | Nhận selector910/TN1, source baseline+C install; remote stage0c54; bên trong vẫn trỏ C | Chạy wrapper không đồng nghĩa chạy install từ snapshot Git |
| `releases/r5_20261029/deployment/run_pi.sh` | Bản công cụ profile máy đã pin; không phải deploy portable | Cần app profile đường dẫn; không sửa immutable release để tiện UI |
| `C/ros2_ws/src/fire_robot_mission/.../operational_session.py` | Operator tên cố định, report String; vòng50ms; GUI đọc X/Q/window-close; không có API nút GUI ngoài process | Phải có adapter sự kiện cho UI mới, không publish thay motor/bơm |
| `C/.../operational_session_core.py:report/task_report` | `task.stage/outcome/reason`, IR, handback, spray_sequence, holds | UI đọc report; không dựng mission state machine khác |
| `C/.../operational_view.py` | Ảnh stale0.5s hiện đen; đóng OpenCV window→WINDOW_CLOSED; X/Q khác nhau | Tắt cửa sổ cũ mà không thay hợp đồng close/keys là thiếu chức năng |
| `C/.../operational_contract.py` | Actor thật là `fire_pulse_controller`; report giữ tên `/fire_mission/r4d_state` | Không đổi tên thành topic đoán từ node mới |
| `C/.../pulse_controller.py:pump_subscribers` | `/pump_cmd` chỉ chấp nhận đúng2 reader: serial_bridge_node và fire_robot_operator | **UI không được subscribe trực tiếp `/pump_cmd`**; đọc actor report thay thế |
| `C/.../startup_health.py` | `health.json`, session/sample_mono, freshness/proof/TF profile | Ping/SSH hay topic count không thay cho readiness |
| `C/.../config/operational_navigation.rviz` | Map/scan/TF, Fixed Frame map; chưa có RobotModel/Path display | Robot icon/path xanh trong mockup chưa là feature đã có; kiểm source topic trước thêm |
| `ros2_ws/src/fire_robot_bringup/launch/dashboard.launch.py` | Chỉ rosbridge WebSocket server | Không có web dashboard hoàn chỉnh để bọc ngay |

Trong bảng, `C/.../module.py` ở các hàng mission nghĩa
`C/ros2_ws/src/fire_robot_mission/fire_robot_mission/module.py`; hàng config
thuộc `C/ros2_ws/src/fire_robot_mission/config/`.

## 4. Những mismatch tác động trực tiếp đến kế hoạch

1. Root workflow, candidate R5 và snapshot Git không phải ba install có thể
   source lẫn nhau. Profile phải ghi tường minh underlay, candidate, script,
   packet và remote stage; kiểm trước khi chạy.
2. Root mapping mặc định SSH địa chỉ LAN `pi@10.42.0.185`, có thể detach/reuse
   Pi stack và không cleanup Pi như R5. App chạy AP phải cấu hình profile đã có
   và quản lý transition; không thay mạng hoặc bê nguyên workflow cũ vào R5.
3. Map selector mới đã có910/TN1; nhận định “chỉ có map910” trước đây đã lỗi thời.
   **Map mới bất kỳ chưa có trong wrapper whitelist** dù runner lõi có `--map`.
4. R5 dùng raw serial V2/S3 actuator trực tiếp theo source hiện hành. Tài liệu
   cũ còn microROS/WROOM, camera bị comment, control Reliable/watchdog500ms.
   Runtime safety profile kiểm1000ms/10Hz, cmd Best Effort; pump Reliable.
   Task UI không sửa các mismatch này.
5. Camera Pi15FPS nhưng annotated YOLO config tối đa10FPS; FPS actual phải đo
   từ ảnh mới. Không mặc định camera dock đạt15FPS như ảnh thiết kế.
6. IR yaml nền còn5 mẫu/legacy cap; `operational_gate2.yaml` override3 mẫu và
   unbounded. Đọc đúng config đã resolve, không chỉ default trong class.
7. `operator_present` trong health là proof backend/report, không chứng minh
   cửa sổ mới còn đáp ứng. Headless backend cần hợp đồng frontend-liveness.
8. `/r4d_state` có `published_pump_request`, `actuator`, `pulse_remaining_s`;
   không cần thêm reader `/pump_cmd`, vốn làm actor chặn admission.
9. Marker source mới nhất:0c54/Pi pending, khác tài liệu handoff ban đầu8944.
   Không yêu cầu flash/sync trong lượt tài liệu; lúc chạy dùng receipt đúng bản.
10. Snapshot release có `COLCON_IGNORE` để tránh package trùng. App native nếu
    thêm build package sau này phải kiểm discovery, không làm colcon quét trùng.

## 5. Khả năng ghép RViz và điểm chưa được chứng minh

Đã thấy Qt5/PyQt5 trên filesystem máy, header RViz Humble có
`rviz_common::VisualizationFrame`/`VisualizationManager`, package rviz_common
phụ thuộc Qt5. Chưa compile, load plugin, test display backend X11/Wayland hoặc
đo OpenGL lượt này; sự tồn tại của header không phải PoC PASS.

Hướng ưu tiên là Qt5 native RViz host/panel trong process frontend riêng với
operator backend. Không giả định Python ROS1 `librviz` hoạt động trong Humble,
không dùng reparent window ID làm nền triển khai. Quyết định native host/panel
phải có PoC và bằng chứng vòng đời ở mốc M0.

Nguồn API chính thức đã đối chiếu ngày29/10:
[Qt5 QProcess](https://doc.qt.io/archives/qt-5.15/qprocess.html),
[RViz Humble VisualizationFrame](https://github.com/ros2/rviz/blob/humble/rviz_common/include/rviz_common/visualization_frame.hpp),
[RViz Humble Qt dependencies](https://github.com/ros2/rviz/blob/humble/rviz_common/package.xml).
