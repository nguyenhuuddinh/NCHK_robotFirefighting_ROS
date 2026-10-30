> **Cập nhật sau QA laptop:** 59cddc99 đã sync Pi và được chọn cho nghiệm thu.
> R5/pending giữ nguyên; chưa chạy robot. [Báo cáo sync](PI_STATIC_SYNC_2026-10-30_59cddc99.md) ·
> [Registry mới](PI_SYNC_REGISTRY_2026-10-30.md). Nội dung dưới là snapshot trước sync.

# Sửa phục hồi, bố cục và hiển thị xe/đường — 30/10/2026

Đã sửa ba finding F1/F2/F3 của QA bổ sung ở lớp app và thêm hình xe nhìn từ
trên xuống cùng đường kế hoạch Nav2 trong RViz native. Bộ kiểm laptop đạt;
**chưa đồng bộ Pi, chưa chọn current, chưa nghiệm thu robot**.

Ứng viên mới: `59cddc9969669e937e0fd1c98a3982156b6e7e7096204787522895dabc857a38`.
Bản b3b81081 có ba lỗi vẫn giữ nguyên để đối chiếu; current vẫn là 6734c284.

## Thay đổi và kết quả

| Finding | Thay đổi ở app | Kết quả tái hiện sau sửa |
| --- | --- | --- |
| F1/P1: STATE có lỗi seal timeout chặn cả chuỗi STOP | `services/app_peer.py`: chỉ trong cleanup, chấp nhận STATE/STOP của đúng binding/session, chưa sealed, có app_prearm_drain và đúng lỗi PI_SEAL_HEALTH_TIMEOUT ở WAIT_LAPTOP_DRAIN/STOPPING. Startup vẫn dùng transport R5 nguyên. | Trước: 25 STATE/25 giây giả, 0 STOP. Sau: 1 STOP, DONE trong 1 giây giả, cleanup_confirmed=true. runner_error vẫn PI_SEAL_HEALTH_TIMEOUT và confirmed gốc vẫn false. |
| F2/P2: chuỗi lỗi liên tục kéo cửa sổ rộng 7066 px | `native_rviz/wrapping_label.hpp` đo/vẽ bằng cùng quy tắc ngắt dòng, không để chuỗi lỗi đặt chiều rộng tối thiểu. Giữ nguyên nội dung text/log; vùng trạng thái cuộn. | Sáu cấu hình lỗi dài 360 W đều vừa vùng màn hình; nút STOP thấy được và dùng được. |
| F3/P2: 1366×768/150% thành 911×554 logical | `native_rviz/main.cpp`: vùng trạng thái cuộn, camera co theo vùng còn lại, cảnh báo ở đầu, hai nút dừng ngoài vùng cuộn. Kích thước mở đầu giới hạn theo availableGeometry của màn hình. |1366×768/150% đo 911×512 logical trong probe resize; camera 564×202 logical, tiến độ và STOP thấy được. Cả 12 cấu hình normal/long ×2 độ phân giải ×3 scale đạt. |

F1 không coi startup timeout là dừng thành công, không bỏ xác minh danh tính
hoặc cleanup. Sai bảy trường binding, sai session, request_error, sealed,
phase/cờ/lỗi khác đều bị từ chối. Trong cleanup không gửi START/SEAL/RETRY.
Mất kết nối hoặc thiếu proof vẫn có thể BLOCKED đúng lý do; không tự xóa
pending để ép mở phiên mới. Chưa chứng minh mạng Pi hoặc dừng vật lý bằng
kết quả fake transport này.

## Hình xe và đường kế hoạch

- `native_rviz/operational.rviz` dùng plugin RobotModel sẵn có, nhận
  `/robot_description` Reliable/Transient Local và TF của robot hiện hành.
  Xe là URDF thật chiếu từ trên xuống; app không tự tạo vị trí hoặc phát TF.
- Plugin Path đọc `/plan` Reliable/Volatile, màu xanh, chỉ giữ kế hoạch gần
  nhất. Đó là đường Nav2 dự kiến, chưa phải vệt đường xe đã chạy.
- `native_rviz/session_ui.cpp` bật xe khi vùng quan sát của phiên được mở;
  chỉ bật Path trong chữa cháy. Quét map lái thủ công chưa có đường Nav2.
  STOP/mất kết nối/đóng gate reset rồi tắt hai display, ngừng đọc hai topic.
- Ẩn panel chỉnh sửa RViz lồng trong app để dành chỗ cho map; tắt lớp vẽ trục
  TF cho gọn, không đổi dữ liệu/tree TF. Camera lớn và màu trắng–đỏ–xám giữ.
- Khi chưa có TF hợp lệ tới `map`, chưa định vị/đặt Initial Pose, chưa có kế
  hoạch mới, chưa thể thấy đủ xe/đường. Không vẽ dữ liệu giả ở phiên thật.

Nguồn topic `/plan`: [Nav2 Humble planner_server.cpp](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_planner/src/planner_server.cpp)
phát nav_msgs/Path trên `plan`. R5 đã có robot_state_publisher và URDF;
không thêm node nặng lên Pi, không đổi topic/control/QoS gốc hoặc watchdog.

[Ảnh vùng map render thật trong QA](qa_evidence/fix_and_visual_20261030/robot_path_native.png.map.png)
cho thấy URDF và đường xanh từ **fixture localhost**. Ảnh này không phải robot
đang chạy. [Bố cục 1366×768/150%](qa_evidence/fix_and_visual_20261030/layout_1366_768_1.5_normal.png)
và [lỗi dài 1920×1080/100%](qa_evidence/fix_and_visual_20261030/layout_1920_1080_1.0_long.png)
được chụp bằng Qt offscreen. [Ảnh toàn app trong probe native](qa_evidence/fix_and_visual_20261030/robot_path_native.png)
đã được xem, map/xe/đường và camera nằm cùng cửa sổ. Capture đầu qua QScreen
có ảnh đen; lượt chụp bổ sung cuối đã lấy được toàn app và render Ogre trước/
sau STOP. [Sau STOP](qa_evidence/fix_and_visual_20261030/robot_path_native.png.stopped.map.png)
không còn xe/đường. Lượt chụp thêm chạy lại 10 ca native đạt, không cộng vào
158 ca chính. Chưa kiểm mọi window manager/trang trí màn hình thật.

## QA đã thực thi

| Bộ kiểm | Kết quả | Phạm vi |
| --- | --- | --- |
| Unittest app |190 PASS | Có 5 test method mới về transport drain, với subcase danh tính, phạm vi command và lỗi vận chuyển; không cộng từng subcase thành ca mới. |
| Hồi quy UI/supervisor/runner |67 PASS | Nút/map/lease/STOP/luồng operator/mapping/entry Pi giả/provenance; graph IDLE/detach có thêm hai topic mới. |
| Bổ sung offline |45 PASS | Hai class AppPeer+AppPiController thật phối hợp fake SSH/clock; STOP tại 12 stage, disk full, health lỗi, worker/recovery. |
| Child failure |8 PASS | StartupSequence và generated fire entry thật với child giả lỗi spawn/exit. |
| ACK/network giả |8 PASS | Mất ACK/chậm/cleanup bound theo bộ test phiên 1. Không đo mạng Pi thật. |
| RViz native |10 PASS | Plugin thật, URDF/TF/map/plan fixture; xe có geometry, theo TF x 0→0,55 m, Path statusOK, mapping không đọc plan, STOP ngừng đọc, không publisher motor/bơm. |
| Qt bổ sung |19 PASS |12 layout, 6 dialog/focus và 1 launcher artifact mới từ cwd khác. |
| Packaged smoke |1 PASS | Binary mới mở IDLE, graph không đọc topic robot/plan/camera; state tạm. |
| Native CTest |1/1 PASS | Frontend IPC/lease. CTest trong quá trình đóng gói cũng đạt. |

**158 pytest PASS trong 54,05 giây**, không FAIL/ERROR/SKIP; 190 unittest và
CTest báo riêng. Các lượt chạy lặp để debug/capture không cộng vào 158.
Một cảnh báo tương thích Python 3.11 của xacro/enum trong môi trường hiện tại
không làm test fail. Lệnh chính và từng nodeid ở `run_release_qa.sh`,
`release_qa.xml`, `test_counts.json`; log unittest ở `unittest.log`.

Probe ban đầu dùng sai cách lấy property Status của RViz nên hai assert
không đọc được trạng thái; đã sửa harness để lấy StatusList thật, không sửa
plugin/runtime để ép test đạt. Lượt đóng gói đầu trong sandbox thiếu quyền
socket cục bộ làm CTest fail; chạy lại cùng source với socket được phép đạt.
Log thử đầu giữ trong evidence, không tính chúng là lỗi sản phẩm mới.

ROS kiểm thử dùng domain 211/localhost; không SSH Pi, đọc thiết bị, chạy
SLAM/Nav2/YOLO production, motor/bơm hoặc flash. RViz render thật nhưng các
publisher dữ liệu đều là fixture cục bộ. Mô hình đã học của app không đổi:
parent 0c54 vẫn dùng `best_29_10_2026.pt`; gói R5 CLI minireplay của công việc
khác được bảo toàn riêng, không nhập vào app trong lượt này.

## Bảo toàn và đóng gói

- 1516 file được bảo vệ trước/sau giữ nguyên SHA256, gồm R5 release/candidate,
  app artifact cũ, manifest minireplay, wrapper, current và pending thật.
  95 file cài R5 trên laptop cũng giữ nguyên nội dung và symlink target.
- 364 file parent R5 giữ nguyên byte trong cả release lẫn gói app mới.
  Parent packet: `0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0`.
- App mới có 426 source pins; source services/native trong gói khớp source đã
  kiểm. Archive đã verify toàn bộ nội dung, không chỉ hash file nén. B3 cũ
  cũng verify lại đạt và vẫn mang trạng thái QA lịch sử của nó.
- Binary SHA256: `1f647273050fff334c576677546a3fed8a6f55fb7970b6321c2dba0b4aaea875`.
- Archive SHA256: `82a12318b97fa9ae6f9cc53e76eda5ef32c9099346779bb7de4768a02fd4331a`.
- Receipt mới: `pi_static_receipt=PENDING`, `runtime_verified=false`.
  Không đổi current 6734, không sửa pending, không đè gói/launcher R5.

Thay đổi source vận hành chỉ ở 5 file app (adapter, main, RViz config,
SessionUi, header wrapping mới); còn lại test và docs. `ai/`, node ROS,
firmware, các script R5 không được sửa. `RUNTIME_VALIDATION_HISTORY.md`,
`output/` và `tmp/` đang có thay đổi từ công việc khác được giữ nguyên.

## Còn lại trước vận hành

Ba finding đã đóng ở mức laptop. [Registry cập nhật](FIX_AND_VISUAL_REGISTRY_2026-10-30.md)
vẫn giữ các PARTIAL/NOT_RUN của kế hoạch 96 tiêu chí; không biến kết quả
component thành chứng nhận mọi tình huống. Lỗi TF/USB thực tế trước đây chưa
được kết luận đã hết. Bước tiếp theo là stage gói app riêng trên Pi qua LAN,
build/verify receipt/hash, rồi nghiệm thu WiFi và robot có giám sát trong
phiên 2. Chỉ sau đồng bộ đúng packet mới chọn gói mới cho sử dụng.

Bằng chứng: [qa_evidence/fix_and_visual_20261030](qa_evidence/fix_and_visual_20261030/),
bao gồm diff trước/sau, XML/log, JSON kích thước, ảnh render, receipt,
protection_final, source pins và SHA256SUMS. Chạy lại từ root repo sau khi
source ROS Humble và các overlay như `run_release_qa.sh`; Qt/RViz cần màn
hình OpenGL cùng quyền socket local. Các probe được build từ
`tests/pre_sync_ui_review/CMakeLists.txt` trong `/tmp/fire_robot_pre_sync_ui`,
frontend CTest từ `native_rviz/CMakeLists.txt` trong `/tmp/fire_robot_control_preview_1000`.
Không dùng ROS_DOMAIN_ID=211 nếu một phiên production đang chạy trên domain đó.
