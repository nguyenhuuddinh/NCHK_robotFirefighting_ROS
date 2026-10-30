> **QA bổ sung 30/10/2026:** Bản b3b81081 chưa đủ điều kiện đồng bộ/chọn vận
> hành: đã xác nhận 1 lỗi P1 phục hồi STOP sau seal timeout và 2 lỗi P2 bố
> cục. Code vận hành/R5/current/pending giữ nguyên trong lượt QA.
> [Báo cáo QA](QA_RECHECK_SESSION1_2026-10-30.md) ·
> [Registry mới nhất](QA_RECHECK_REGISTRY_2026-10-30.md).

# Registry96 ca dự kiến — kết quả phiên1, 30/10/2026

Đọc cùng [báo cáo phiên1](SESSION1_APP_ENTRY_QA_2026-10-30.md) và
[expected từng ID](QA_APP_ENTRY_AND_RECOVERY_2026-10-29.md).

**Không quy đổi số unittest/pytest thành96 PASS.** PASS bên dưới chỉ là kết
quả setup offline của dòng kế hoạch; PARTIAL còn nhánh/môi trường chưa đủ;
NOT_RUN dành cho Pi/hardware. Không dòng offline nào chứng minh dừng vật lý.
Các tên `test_*.py` thuộc `../tests/`, kết quả chung `unittest_final185.log`;
`qa_*.py` thuộc `../tests/`, nodeid và thời gian trong `integration_final.xml`;
riêng `qa_app_peer_ack.py` nằm trong `peer_ack.xml`.
File raw/log/ảnh không kèm đường dẫn nằm trong
`qa_evidence/app_entry_session1_20261030/`, được pin bởi SHA256SUMS.
Các health negative của R5 có136 PASS và10 lỗi fixture riêng (xem báo cáo),
chỉ các assertion đạt được dùng làm coverage; không ghi cả suite PASS.

App packet `b3b81081d95e7c1a0ec974a29f61654dc517263594132977df1426069573c828`;
parent `0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0`.
Thời gian chạy theo suite trong báo cáo/XML, không suy ra thời gian từng ID
vì một test có thể phục vụ nhiều ID. Không cộng rerun hoặc positivecontrol
thành test sản phẩm mới. Mọi PARTIAL phải được xem trước promotion phiên2;
kiểm hardware khi cần không thay cho fault injection nguy hiểm.

Tổng registry: **61 PASS**, **25 PARTIAL**, **10 NOT_RUN**.

| ID | Trạng thái | Bằng chứng/nodeid nhóm | Actual / phần chưa chạy |
| --- | --- | --- | --- |
| AER-P01 | PASS | final_protection_verification.json | 364 release file/manifest trước và sau nguyên byte. |
| AER-P02 | PASS | final_protection_verification.json; r5_installed_sources.json | 364 candidate,95 installed paths/hash, hai wrapper giữ nguyên. |
| AER-P03 | PARTIAL | test_sync_script_fake.py | Fake sync dùng stage app riêng; chưa hash stage Pi thật trong phiên1. |
| AER-P04 | PASS | test_app_entry.py; final_protection_verification.json | 425 pins gồm đủ364 parent nguyên byte, generated app entry riêng. |
| AER-P05 | PASS | test_app_entry.py | Parent/anchor sai bị reject; không fallback. |
| AER-P06 | PASS | test_app_entry.py | R5 CLI PLAN_ONLY chạy được; không ROS init/SSH. |
| AER-P07 | PASS | test_app_actions.py; artifact_smoke.log | App IDLE không ROS reader; lock R5 bận từ chối trước SSH, không signal. |
| AER-P08 | PASS | qa_app_peer_ack.py; peer_ack.xml | R5 main gốc với backend giả từ chối khi app giữ common lock, chưa gọi Pi. |
| AER-P09 | PASS | test_app_entry.py; final_protection_verification.json | Archive thật verify425pins; tamper/duplicate/symlink/traversal bị chặn. |
| AER-P10 | PASS | test_supervisor_integration.py; test_live_session_fake.py | Packet/binary/Pi receipt thiếu hoặc sai không START. |
| AER-P11 | PASS | test_sync_script_fake.py; test_app_entry.py | Lỗi fake build/sync không chọn current/receipt PASS; current6734 nguyên. |
| AER-P12 | PARTIAL | final_protection_verification.json | Rollback chưa thao tác; selector cũ vẫn nguyên, không promote bản mới. |
| AER-S01 | PASS | qa_app_view_lifecycle.py[idle]; artifact_smoke.log | Qt thực mở IDLE, không rclcpp/RViz/Observer hoặc I/O START. |
| AER-S02 | PASS | qa_candidate_runner.py; test_app_pi_lifecycle.py | Generated fire main/effect order và các gate được chạy với I/O giả. |
| AER-S03 | PASS | qa_mapping_entry.py[False] | Main mapping thật qua seal trước SLAM/view/teleop; không mission. |
| AER-S04 | PASS | test_app_pi_lifecycle.py; test_view_bootstrap.py | Chưa sealed/current proof không mở effect tiếp hoặc tool. |
| AER-S05 | PASS | test_app_pi_lifecycle.py | Fake health hồi trong hạn đi tiếp; timeout mới vào drain. |
| AER-S06 | PARTIAL | test_app_pi_lifecycle.py; r5_health_negative.log | Sequence/current health có coverage; chưa một ca app đầy đủ flap sát mọi deadline. |
| AER-S07 | PASS | test_app_pi_lifecycle.py | Sealed nhưng health.io=false vẫn chờ, deadline không được kéo dài. |
| AER-S08 | PASS | test_app_pi_lifecycle.py; r5_health_negative.log | Pose/5s/identity/consumers giữ checker; thiếu pose chưa mở YOLO sớm. |
| AER-S09 | PARTIAL | qa_candidate_runner.py; test_session_outcome.py | Có child lỗi/outcome; chưa inject từng child YOLO/consumers ở mọi phase. |
| AER-S10 | PASS | test_app_pi_lifecycle.py | RETRY_PREARM tối đa1, không retry sau seal; lifecycle gốc không đổi. |
| AER-S11 | PASS | qa_app_peer_ack.py; peer_ack.xml | START/RETRY/SEAL mất ACK được kiểm bằng STATE; không replay mutation, thiếu proof bị từ chối. |
| AER-S12 | PASS | test_session_outcome.py | Lỗi gốc structured giữ trước lỗi thoát Pi chung. |
| AER-S13 | PARTIAL | qa_mapping_entry.py; source_diff.patch | Poll đồng bộ +2s đã áp; chưa metric đủ cả hai nhánh SSH chậm/nhanh. |
| AER-S14 | PASS | r5_health_negative.log; test_app_pi_lifecycle.py | HealthCore/identity epoch/profile negative đã chạy, checker source nguyên. |
| AER-T01 | PASS | final_protection_verification.json | StartupHealthNode/HealthCore và broadcaster thuộc364parent, nguyên byte. |
| AER-T02 | PARTIAL | actual_health_load_r5/; actual_health_load_sampled/ | Hai mode chạy entry thật với25ms load, không drop; chưa normal mode riêng không inject. |
| AER-T03 | PASS | app_health_load.log | Nguồn/fast listener mới; r5 backlog>1s, sampled lookup<300ms; localhost giả. |
| AER-T04 | PASS | r5_health_negative.log | Negative source age .301s/future/profile không được admitted bằng receive time. |
| AER-T05 | PASS | r5_health_negative.log | TF/profile/odom/source/pose thiếu hoặc stale bị từ chối. |
| AER-T06 | PARTIAL | test_recovery.py; r5_health_negative.log | JSON nonfinite/duplicate/session/stale có assert; chưa đủ mọi time đảo cho health entry. |
| AER-T07 | PARTIAL | test_app_health_entry.py; test_app_pi_lifecycle.py | Timer.2s và stable gate giữ; chưa đo normal tải thật cả cửa1s/5s sampled. |
| AER-T08 | PASS | qa_app_view_lifecycle.py; cycles.graph.json | 6 attach/detach qua3workflow thực, IDLE không TF/map/scan/camera GUI reader. |
| AER-T09 | PARTIAL | qa_monitor_provenance.py; qa_full_operator.py | Monitor thật không raw/pump publisher hoặc operator pin; toàn pipeline phần cứng chưa inspect. |
| AER-T10 | PARTIAL | app_health_load.log; test_app_health_entry.py | 25ms sample load có test; chưa inject ghi JSON ENOSPC đúng lúc STOP callback đang chạy. |
| AER-R01 | PASS | test_session_outcome.py | Replay fire rc2: hàm6734 FAIL → outcome mới STOP/cleanup true; startup vẫn FAIL. |
| AER-R02 | PASS | test_session_outcome.py | Replay mapping thiếuSTOP vẫn false/blocked. |
| AER-R03 | PASS | qa_app_pi_entry.py; test_app_pi_lifecycle.py | Seal timeout giữ I/O30s cho drain; STOP mới có proof fixture. |
| AER-R04 | PASS | test_app_pi_lifecycle.py | Laptop mất/hết30s tự cleanup hữu hạn; chỉ STATE/STOP khi đang drain. |
| AER-R05 | PARTIAL | test_live_session_fake.py; test_app_actions.py | STOP trước START/pairing và trong startup có coverage; chưa inject riêng mọi phase. |
| AER-R06 | PASS | qa_full_operator.py; qa_mapping_entry.py | Operator thật qua bus giả ở7state; runner mapping/verify drain theo ownership. |
| AER-R07 | PASS | test_session_outcome.py; test_stop_recovery.py | rc0/rc2 hoặc process gone không đủ để suy ra STOP. |
| AER-R08 | PASS | test_recovery.py; test_session_outcome.py | Child crash/forced/live/missing audit tiếp tục khóa. |
| AER-R09 | PASS | test_session_outcome.py | Remote startup error độc lập current group/child cleanup; không sửa DONE gốc. |
| AER-R10 | PASS | qa_candidate_runner.py; test_view_bootstrap.py | Handoff monitor reap/retirement trước mission/operator; foreign/late view không cấp quyền. |
| AER-R11 | PASS | qa_pre_sync_ui.py; test_recovery.py | Held/double/fast/slow/fail check chỉ một worker; kết quả UI giữ rõ. |
| AER-R12 | PASS | test_recovery.py; test_stop_recovery.py | Marker/PID/boot/binding thay đổi không archive stale proof. |
| AER-R13 | PASS | qa_pre_sync_ui.py; qa_pre_sync_faults.py | Phiên mới xóa nhãn STOP phiên trước. |
| AER-R14 | PASS | test_stop_recovery.py; qa_mapping_entry.py[True] | New STOP-only không seal/SLAM/Nav/YOLO/teleop; link proof mới, giữ failure cũ. |
| AER-R15 | PASS | test_stop_recovery.py; test_recovery.py | Wrong boot/livegroups/missing proof/lock/cancel giữ pending, không mission/ON. |
| AER-R16 | PARTIAL | qa_pre_sync_faults.py; test_recovery.py | Ghi/xóa/archive OSError có kiểm không kẹt worker; chưa mọi ENOSPC ở mọi receipt. |
| AER-U01 | PASS | qa_button_feedback.py; qa_pre_sync_ui.py | Held/click/keyboard/pressed pixels và busy label được Qt kiểm thật. |
| AER-U02 | PASS | qa_map_selection.py | Popup không reset quaSTATUS, mouse selection chỉ một SELECT. |
| AER-U03 | PASS | test_app_actions.py; test_supervisor_integration.py | Spam khởi động chỉ một worker, duplicate reject. |
| AER-U04 | PASS | qa_pre_sync_ui.py; test_view_bootstrap.py | Tool/phase/STOP/lost/late status và CHECKING bị khóa đúng. |
| AER-U05 | PARTIAL | test_app_pi_lifecycle.py; source_diff.patch | Progress/remaining/lỗi gốc đã nối; chưa chụp đủ từng clock/SSH/TF/camera gần hạn. |
| AER-U06 | PASS | test_session_outcome.py; qa_pre_sync_ui.py | Startup/STOP/cleanup riêng; UI giữ lỗi/check result và log. |
| AER-U07 | PASS | test_frontend_lease.py; qa_pre_sync_ui.py | Lease350ms bình thường giữ; late status không mở lại tools. |
| AER-U08 | PARTIAL | qa_supervisor_gui.py; session_ui.cpp | Có khóa keyboard khi nhập text; chưa Qt testcase riêng tên chứa X/Q+Esc modal. |
| AER-U09 | PARTIAL | test_recovery.py; qa_pre_sync_faults.py | Close lúc save/start/check và lỗi worker có test; chưa mọi auth/drain/close sequence. |
| AER-U10 | PARTIAL | layout125.png; layout150.png; idle.png | Đã nhìn ảnh100/125/150%, logical1280×720; chưa đủ1366×768/1920×1080+splitter matrix. |
| AER-U11 | PARTIAL | qa_pre_sync_ui.py; session_ui.cpp | Camera stale/status giữ; chưa screenshot riêng mọi text dài/camera frozen. |
| AER-U12 | PASS | test_app_actions.py | 600fake actions, seed7/19/29, assert không chồng/reopen STOP. |
| AER-M01 | PASS | qa_map_selection.py; test_supervisor_integration.py | Map cũ chọn được IDLE, snapshot map truyền runner. |
| AER-M02 | PASS | test_supervisor_integration.py; test_map_save.py | Quét/lưu tên riêng/chọn mới rồi map cũ qua fixture. |
| AER-M03 | PASS | test_map_save.py; test_supervisor_integration.py | Tên sai/rỗng/path/outsideallowlist không spawn/save. |
| AER-M04 | PASS | test_map_save.py; test_map_catalog.py | Trùng/symlink/partial/race không ghi đè file user. |
| AER-M05 | PASS | test_supervisor_integration.py | Map thay đổi sau chọn hoặc trong snapshot bị reject. |
| AER-M06 | PASS | test_map_save.py; test_supervisor_integration.py | Duplicate save và STOP commit có rollback transaction. |
| AER-M07 | PARTIAL | test_map_save.py | Fake generator fail/lostSLAM đạt; chưa đủ timeout và ENOSPC subprocess map_saver thật. |
| AER-M08 | PASS | test_supervisor_integration.py; test_app_actions.py | Không chuyển mode khi phiên cũ chưa sạch hoặc còn pending. |
| AER-M09 | PARTIAL | test_frontend_lease.py; qa_mapping_entry.py | Q/lease/cleanup có test; chưa mô phỏng riêng focus mất trong teleop thật. |
| AER-M10 | PARTIAL | test_app_launch_environment.py; test_supervisor_integration.py | Repo path dấu cách chạy được; chưa launcher với cwd khác và map dấu cách riêng. |
| AER-N01 | PASS | qa_supervisor_qt.py; test_recovery.py | Domain/route reject trướcSSH; không tự đổi network. |
| AER-N02 | PARTIAL | test_recovery.py; test_live_session_fake.py | Route/transport/auth cancel có coverage; chưa đủ hostkey/prompt lỗi bằng SSH thật. |
| AER-N03 | PASS | test_pairing_and_status.py; test_pairing_transfer.py; test_live_session_fake.py | Split, duplicate, stale/path, session/packet và ownership được kiểm bằng fixture. |
| AER-N04 | PARTIAL | test_app_actions.py; test_live_session_fake.py | STOP ưu tiên và worker riêng; chưa ca SSH STATE timeout đủ mô hình mutationACK. |
| AER-N05 | PARTIAL | test_live_session_fake.py; test_frontend_lease.py | Fake transport mất/lease mất failclosed; chưa inject trước/start/running đủ cả3thời điểm. |
| AER-N06 | PASS | test_frontend_lease.py; qa_full_operator.py | Freeze frontend đi Q/STOP qua operator thật+bus giả, backend giữ cleanup. |
| AER-N07 | PASS | test_supervisor_integration.py; test_recovery.py | Pending tồn qua mởcontroller mới, chưa có proof không tựstart. |
| AER-N08 | PASS | test_operator_owned_backend.py; test_recovery.py | Group/descendant/live PID không coi leader exit là sạch. |
| AER-N09 | PASS | test_recovery.py | PID reuse/snapshot quyền/lỗi reject, không signal unrelated. |
| AER-N10 | PASS | test_operator_channel.py; test_supervisor_integration.py | Overflow/JSON/NaN/duplicate/thừa field bị chặn, Q ưu tiên. |
| AER-N11 | PARTIAL | artifact_smoke.log; qa_supervisor_gui.py | Local app/assets mở không tải Internet; chưa thao tác dưới WiFi Pi mất Internet thật. |
| AER-N12 | PARTIAL | qa_app_view_lifecycle.py; test_app_actions.py | 3workflow/6view và600events đạt; chưa10vòng hoàn chỉnh kèm sốFD/thread trước/sau. |
| AER-H01 | NOT_RUN | SESSION2 | Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H02 | NOT_RUN | SESSION2 | Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H03 | NOT_RUN | SESSION2 | Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H04 | NOT_RUN | SESSION2 | Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H05 | NOT_RUN | SESSION2 | Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H06 | NOT_RUN | SESSION2 | Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H07 | NOT_RUN | SESSION2 | Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H08 | NOT_RUN | SESSION2 | Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H09 | NOT_RUN | SESSION2 | Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H10 | NOT_RUN | SESSION2 | Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
