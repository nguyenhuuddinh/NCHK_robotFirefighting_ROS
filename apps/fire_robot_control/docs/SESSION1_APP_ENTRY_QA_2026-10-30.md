> **QA bổ sung 30/10/2026:** Bản b3b81081 chưa đủ điều kiện đồng bộ/chọn vận
> hành: đã xác nhận 1 lỗi P1 phục hồi STOP sau seal timeout và 2 lỗi P2 bố
> cục. Code vận hành/R5/current/pending giữ nguyên trong lượt QA.
> [Báo cáo QA](QA_RECHECK_SESSION1_2026-10-30.md) ·
> [Registry mới nhất](QA_RECHECK_REGISTRY_2026-10-30.md).

# Phiên 1 — entrypoint riêng cho app và QA offline

Ngày: 30/10/2026. **Đã triển khai và đóng gói ứng viên trên laptop; chưa
chọn bản này để vận hành.** Mục tiêu của phiên là giảm khóa oan sau lỗi khởi
động, giữ nguyên đường chạy R5 cũ và chuẩn bị kiểm thử Pi ở phiên 2. Trạng
thái chi tiết từng ca nằm trong [registry 96 ca](SESSION1_QA_REGISTRY_2026-10-30.md).

## Bản đã tạo và phạm vi

- Artifact app: `b3b81081d95e7c1a0ec974a29f61654dc517263594132977df1426069573c828`.
- Parent R5: `0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0`.
- SHA256 binary: `04a2cf778f7efd39ca5945f16b9817d63dba68ff4151e43c434f1c4a37a553fa`.
- SHA256 source archive: `6df4dacb21cf4af71afc7c2704db87e4f2d2bbe55dec875957695f183919b619`.
- Manifest ứng viên có 425 file, bao gồm đủ 364 file parent R5 nguyên byte.
  Binary, archive và manifest đã được verify; receipt build ghi
  `pi_static_receipt=PENDING`, `runtime_verified=false`.
- `artifacts/current.txt` vẫn chọn bản cũ `6734c284…`; launcher mặc định do đó
  chưa mở bản mới. Gói trung gian `bdc40b80…` được build trước khi phát hiện
  lỗi import QoS và không dùng làm ứng viên cuối.
- File pending của phiên thật còn nguyên. Không SSH, đồng bộ Pi, flash hoặc
  chạy robot trong phiên này. Không xóa pending để vượt khóa.

## Thay đổi chính

1. Tạo điểm vào riêng cho app từ parent R5 đã pin. Script, node, watchdog,
   ngưỡng TF và lệnh terminal R5 gốc không bị chỉnh sửa. App dùng common lock
   để từ chối khi runner R5 đã chiếm quyền.
2. Khi khởi động chưa thành công, phân biệt lỗi khởi động, xác nhận STOP và
   cleanup. Nếu STOP và cleanup đều có bằng chứng hợp lệ, lỗi khởi động không
   tự làm khóa oan. Nếu thiếu STOP hoặc còn tiến trình, app vẫn khóa và báo lý
   do. Replay hai phiên lỗi cũ kiểm tra hai nhánh này.
3. Pi entry riêng cho app giữ I/O trong cửa sổ tối đa 30 giây sau seal timeout,
   để laptop có thời gian dừng và nhận telemetry mới; hết hạn thì thu dọn. Đây
   là cửa sổ phục hồi hữu hạn, không thay đổi watchdog hay tự cấp quyền chạy.
4. Nút **Kiểm tra lại** vẫn là audit chỉ đọc. Nếu phiên cũ thiếu bằng chứng
   dừng, có thao tác **Xác minh dừng** riêng: kiểm tra identity/lock/tiến trình
   cũ, tạo phiên STOP-only và chỉ mở lại sau proof mới và cleanup hợp lệ. Kết
   quả lỗi cũ vẫn được giữ cùng liên kết tới proof mới. Chức năng này chưa được
   kiểm chứng với Pi/robot thật.
5. UI mở ở IDLE không tự tạo RViz/ROS readers; RViz thật và camera gắn sau
   gate. Lúc gắn view có handshake giới hạn thời gian; runner chờ UI sẵn sàng
   trước teleop hoặc các bước chữa cháy tiếp theo. Khi một phiên kết thúc, UI
   có thể đóng rồi mở lại nhanh để khởi tạo RViz sạch; supervisor giữ trạng
   thái và lựa chọn map. Không tự chạy phiên robot mới.
6. Giữ hiển thị đang chờ gì, thời gian và lỗi gốc, phân biệt kết quả dừng/thu
   dọn; kiểm tra chọn map, lưu map và các thao tác bấm lặp. Sửa import QoS
   tường minh trong runner quét: trên ROS Humble import `rclpy` đơn lẻ không
   bảo đảm thuộc tính `rclpy.qos` tồn tại. Ca main mapping trực tiếp với I/O
   giả đã FAIL trước sửa và PASS sau sửa.
7. Có mode thử nghiệm `sampled` để khảo sát lịch callback health; mặc định vẫn
   là checker R5. Mode thử không đổi nội dung checker hoặc giới hạn freshness,
   và chưa được chọn mặc định hay nghiệm thu trên Pi có tải.

## Kết quả đã chạy

| Kiểm tra | Kết quả | Phạm vi |
| --- | --- | --- |
| Unit app | **185 PASS** | Fake process, clock, journal, map, lock, recovery, UI lease; 6,233 giây. |
| Tích hợp app | **67 PASS** | Qt/IPC thật, RViz/ROS localhost, runner app với I/O giả; 39,37 giây. Node ID/thời gian trong `integration_final.xml`. |
| Transport và common lock | **8 PASS** | AppPeer: mất ACK START/RETRY/SEAL, proof STATE và không gửi lại mutation; STATE retry có giới hạn. R5 main giả từ chối khi app giữ lock. Không SSH. |
| CTest đóng gói | **1/1 PASS** | Native frontend của artifact cuối. |
| Artifact mở/đóng | **1 PASS** | Binary gói cuối mở IDLE, không GUI ROS reader trong domain cô lập, đóng sạch. |
| Tải callback health | **2 PASS** | Hai mode trên localhost213 với delay nhân tạo 25 ms; không phải đo Pi thật. |
| Giao diện | **PASS có giới hạn** | Qt/RViz gắn/tháo 6 lần trong 3 workflow giả; đã xem ảnh scale 125% và 150%, còn thiếu toàn bộ ma trận độ phân giải/splitter. |

Trong phép thử tải giả, p95 tuổi source mà fast listener thấy khoảng 88 ms;
health lookup mặc định R5 khoảng 4192 ms, mode sampled khoảng 82 ms. Đây là
positive control về một kiểu backlog có thể xảy ra, **không kết luận đó là
nguyên nhân đã được chứng minh trên robot**. TF/USB thực tế vẫn cần đo thêm.

Bộ kiểm bổ sung chạy trực tiếp test R5 cũ có **136 PASS, 10 FAIL, 1
DESELECTED**. Hai FAIL tìm file historical `before/` không có trong release;
tám FAIL do fixture giả `Local` thiếu trường `history` mà runner R5 hiện tại
sử dụng. Các lỗi nằm ở bài kiểm thử cũ/fixture, không phải bằng chứng runtime
R5 hỏng, nhưng cũng không thể ghi bộ đó PASS. Đã lưu log/XML đầy đủ; không
chỉnh source/test R5 để ép đạt. Các ca app riêng vẫn phải xét theo registry và
chưa thay thế nghiệm thu R5 thực tế.

Registry tách **PASS / PARTIAL / NOT_RUN** theo từng tiêu chí; PARTIAL còn
nhánh offline hoặc môi trường chưa được kiểm. Tất cả 10 ca Pi/hardware là
NOT_RUN. Trước khi chọn current phải xử lý các mục PARTIAL phù hợp, nhất là
mất ACK/mạng, pairing, lock và STOP ở các phase; không tuyên bố đã QA đủ mọi
trường hợp.

## Bảo toàn R5 và bằng chứng

`final_protection_verification.json` đối chiếu trước/sau: 364/364 file trong
mỗi bộ release và candidate R5, hai wrapper R5, 95 đường dẫn/file cài trên
laptop, app current cũ và pending thật đều nguyên hash. 425 source pins của
artifact mới và archive đã verify; source app trong gói khớp source làm việc
khi đóng gói. Kết quả này chưa kiểm tra R5 trên Pi trong phiên này và không
chứng minh hai lệnh cũ chạy thành công trên phần cứng hôm nay.

Bằng chứng, log, ảnh, XML, diff và danh sách hash nằm trong
[thư mục QA](qa_evidence/app_entry_session1_20261030/). `SHA256SUMS` pin từng
file bằng chứng; `QA_SOURCE_SHA256SUMS` pin source/test lúc bàn giao. Trong repo,
`apps/`, `output/`, `tmp/` đang là untracked; không xóa thay đổi sẵn có.

## Verify lại offline

Các lệnh dưới không khởi động robot/SSH; ROS dùng domain riêng và localhost.
Cần display/Xvfb cho các bài Qt/RViz. Không chạy khi một bài ROS sản xuất đang
sử dụng domain kiểm thử. Dừng và ghi lỗi nếu thiếu underlay hoặc môi trường.

```bash
cd '/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS'
(
  source /opt/ros/humble/setup.bash
  source /home/huudinh/.codex/worktrees/laptop-baseline/NCHK_robotFirefighting_ROS/ros2_ws/install/local_setup.bash
  source .codex_qa_backups/r5_operational_candidate_20261026/ros/ros2_ws/install/local_setup.bash
  export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=211 ROS_LOCALHOST_ONLY=1
  export PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/releases/r5_20261029/ros/scripts:$PWD/releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission:${PYTHONPATH:-}"
  python3 -B -m unittest discover -s apps/fire_robot_control/tests -p 'test_*.py'
  python3 -B -m pytest -q -p no:cacheprovider apps/fire_robot_control/tests/qa_app_peer_ack.py
  python3 -B -m services.packet_archive \
    apps/fire_robot_control/artifacts/b3b81081d95e7c1a0ec974a29f61654dc517263594132977df1426069573c828/ros
)
```

Các bộ tích hợp Qt/RViz đã dùng fixture/build và điều kiện nêu trong
[registry](SESSION1_QA_REGISTRY_2026-10-30.md), không cần mặc định chạy toàn
workspace `colcon` để kiểm app. Xác minh lại checksum bằng chứng với
`sha256sum -c SHA256SUMS` từ thư mục QA.

## Phiên 2 và điều kiện chọn bản mới

1. Review các PARTIAL còn ảnh hưởng an toàn và xử lý các lỗi fixture kiểm thử
   liên quan bằng test riêng của app, không sửa R5. Chốt lại ma trận QA.
2. Khi có kết nối Pi và phạm vi phù hợp: sync/build **stage app riêng**, kiểm
   receipt, installed sources và hash R5 Pi trước/sau. Không ghi đè stage R5.
3. Kiểm pending cũ, thao tác STOP-only nếu cần, sau đó quét/lưu/chọn map và
   chữa cháy qua các gate có người giám sát. Motion/bơm chỉ khi được cho phép
   trong điều kiện phù hợp. Đối chiếu R5 CLI tuần tự; không tự chạy chồng.
4. Đo TF/health Pi có tải; giữ mode R5 mặc định cho đến khi có đủ bằng chứng
   quyết định mode sampled. Nếu STOP thiếu proof, giữ BLOCKED với lý do rõ.
5. Chỉ chọn `current` khi receipt Pi đúng packet, QA cần thiết đạt và nghiệm
   thu được ghi nhận. Giữ lại artifact cũ để rollback theo quy trình.
