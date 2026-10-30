# Registry sau static sync Pi — 30/10/2026

Ứng viên 59cddc99 đã sync và chọn cho nghiệm thu. [Báo cáo](PI_STATIC_SYNC_2026-10-30_59cddc99.md).
Tổng 96 tiêu chí: **68 PASS, 19 PARTIAL, 0 FAIL, 9 NOT_RUN**. Chỉ nâng P03 và H01
bằng bằng chứng Pi thật; các dòng khác kế thừa [registry sửa lỗi](FIX_AND_VISUAL_REGISTRY_2026-10-30.md).
Các cột trước/sau của dòng kế thừa giữ lịch sử QA1/QA2; P03/H01/P12 là cập
nhật lượt sync. PASS có phạm vi theo dòng, không chứng nhận mọi tình huống.

Bằng chứng sync trong `qa_evidence/pi_sync_20261030_59cddc99/`; reference
QA laptop vẫn thuộc `qa_evidence/fix_and_visual_20261030/` hoặc QA2/QA1 ghi
ở báo cáo tương ứng. H02–H10 chưa chạy; app mở BLOCKED với pending thiếu
STOP đúng hồ sơ, không xóa pending hay tự chạy xác minh để lấy kết quả PASS.

| ID | Trước | Sau QA2 | Cơ sở | Bằng chứng và actual |
| --- | --- | --- | --- | --- |
| AER-P01 | PASS | PASS | Kế thừa QA1 | final_protection_verification.json — 364 release file/manifest trước và sau nguyên byte. |
| AER-P02 | PASS | PASS | Kế thừa QA1 | final_protection_verification.json; r5_installed_sources.json — 364 candidate,95 installed paths/hash, hai wrapper giữ nguyên. |
| AER-P03 | PARTIAL | PASS | Pi thật, chỉ đọc trước/sau | pi_before/after.json.gz: 21.158 file/45 bộ cũ giữ hash; stage app mới 59cddc99 riêng, baseline Git sạch. Không chạm R5. |
| AER-P04 | PASS | PASS | Kế thừa QA1 | test_app_entry.py; final_protection_verification.json — 425 pins gồm đủ364 parent nguyên byte, generated app entry riêng. |
| AER-P05 | PASS | PASS | Kế thừa QA1 | test_app_entry.py — Parent/anchor sai bị reject; không fallback. |
| AER-P06 | PASS | PASS | Kế thừa QA1 | test_app_entry.py — R5 CLI PLAN_ONLY chạy được; không ROS init/SSH. |
| AER-P07 | PASS | PASS | Kế thừa QA1 | test_app_actions.py; artifact_smoke.log — App IDLE không ROS reader; lock R5 bận từ chối trước SSH, không signal. |
| AER-P08 | PASS | PASS | Kế thừa QA1 | qa_app_peer_ack.py; peer_ack.xml — R5 main gốc với backend giả từ chối khi app giữ common lock, chưa gọi Pi. |
| AER-P09 | PASS | PASS | Kế thừa QA1 | test_app_entry.py; final_protection_verification.json — Archive thật verify425pins; tamper/duplicate/symlink/traversal bị chặn. |
| AER-P10 | PASS | PASS | Kế thừa QA1 | test_supervisor_integration.py; test_live_session_fake.py — Packet/binary/Pi receipt thiếu hoặc sai không START. |
| AER-P11 | PASS | PASS | Kế thừa QA1 | test_sync_script_fake.py; test_app_entry.py — Lỗi fake build/sync không chọn current/receipt PASS; current 6734 nguyên. |
| AER-P12 | PARTIAL | PARTIAL | Lịch sử + bước sync | Bản selector 6734 lưu lại, app 59cddc99 được chọn để nghiệm thu dưới lock; pending cũ nguyên. Chưa thực hiện rollback trong cả hai trạng thái. |
| AER-S01 | PASS | PASS | Kế thừa QA1 | qa_app_view_lifecycle.py[idle]; artifact_smoke.log — Qt thực mở IDLE, không rclcpp/RViz/Observer hoặc I/O START. |
| AER-S02 | PASS | PASS | Kế thừa QA1 | qa_candidate_runner.py; test_app_pi_lifecycle.py — Generated fire main/effect order và các gate được chạy với I/O giả. |
| AER-S03 | PASS | PASS | Kế thừa QA1 | qa_mapping_entry.py[False] — Main mapping thật qua seal trước SLAM/view/teleop; không mission. |
| AER-S04 | PASS | PASS | Kế thừa QA1 | test_app_pi_lifecycle.py; test_view_bootstrap.py — Chưa sealed/current proof không mở effect tiếp hoặc tool. |
| AER-S05 | PASS | PASS | Kế thừa QA1 | test_app_pi_lifecycle.py — Fake health hồi trong hạn đi tiếp; timeout mới vào drain. |
| AER-S06 | PARTIAL | PARTIAL | Kế thừa QA1 | test_app_pi_lifecycle.py; r5_health_negative.log — Sequence/current health có coverage; chưa một ca app đầy đủ flap sát mọi deadline. |
| AER-S07 | PASS | PASS | Kế thừa QA1 | test_app_pi_lifecycle.py — Sealed nhưng health.io=false vẫn chờ, deadline không được kéo dài. |
| AER-S08 | PASS | PASS | Kế thừa QA1 | test_app_pi_lifecycle.py; r5_health_negative.log — Pose/5s/identity/consumers giữ checker; thiếu pose chưa mở YOLO sớm. |
| AER-S09 | PARTIAL | PASS | Mới/Đối chiếu | qa_recheck_runner.py; child_faults.xml — 8ca generated fire main với StartupSequence thật: spawn/exit lỗi nav/yolo/mission/operator giữ lỗi và drain local trước Pi. I/O đều giả. |
| AER-S10 | PASS | PASS | Kế thừa QA1 | test_app_pi_lifecycle.py — RETRY_PREARM tối đa1, không retry sau seal; lifecycle gốc không đổi. |
| AER-S11 | PASS | PASS | Kế thừa QA1 | qa_app_peer_ack.py; peer_ack.xml — START/RETRY/SEAL mất ACK được kiểm bằng STATE; không replay mutation, thiếu proof bị từ chối. |
| AER-S12 | PASS | PASS | Kế thừa QA1 | test_session_outcome.py — Lỗi gốc structured giữ trước lỗi thoát Pi chung. |
| AER-S13 | PARTIAL | PARTIAL | Kế thừa QA1 | qa_mapping_entry.py; source_diff.patch — Poll đồng bộ +2s đã áp; chưa metric đủ cả hai nhánh SSH chậm/nhanh. |
| AER-S14 | PASS | PASS | Kế thừa QA1 | r5_health_negative.log; test_app_pi_lifecycle.py — HealthCore/identity epoch/profile negative đã chạy, checker source nguyên. |
| AER-T01 | PASS | PASS | Kế thừa QA1 | final_protection_verification.json — StartupHealthNode/HealthCore và broadcaster thuộc364parent, nguyên byte. |
| AER-T02 | PARTIAL | PASS | Mới/Đối chiếu | qa_recheck_health.py; normal_health_r5/; normal_health_sampled/ — Entry thật cả2mode không delay giả; lookup p95 103,383/80,139ms; không drop trace. Chỉ chứng minh TF quan sát localhost, io/pose vẫnfalse. |
| AER-T03 | PASS | PASS | Kế thừa QA1 | app_health_load.log — Nguồn/fast listener mới; r5 backlog>1s, sampled lookup<300ms; localhost giả. |
| AER-T04 | PASS | PASS | Kế thừa QA1 | r5_health_negative.log — Negative source age .301s/future/profile không được admitted bằng receive time. |
| AER-T05 | PASS | PASS | Kế thừa QA1 | r5_health_negative.log — TF/profile/odom/source/pose thiếu hoặc stale bị từ chối. |
| AER-T06 | PARTIAL | PASS | Mới/Đối chiếu | qa_recheck_offline.py; targeted_regression.xml — Stale750ms/future/NaN/Inf/timeđảo reject; callableclock sau đọc file; session/JSON negative kế thừa và hồi quy đạt. |
| AER-T07 | PARTIAL | PARTIAL | Kế thừa QA1 | test_app_health_entry.py; test_app_pi_lifecycle.py — Timer.2s và stable gate giữ; chưa đo normal tải thật cả cửa1s/5s sampled. |
| AER-T08 | PASS | PASS | Kế thừa QA1 | qa_app_view_lifecycle.py; cycles.graph.json — 6 attach/detach qua3workflow thực, IDLE không TF/map/scan/camera GUI reader. |
| AER-T09 | PARTIAL | PARTIAL | Kế thừa QA1 | qa_monitor_provenance.py; qa_full_operator.py — Monitor thật không raw/pump publisher hoặc operator pin; toàn pipeline phần cứng chưa inspect. |
| AER-T10 | PARTIAL | PARTIAL | Mới/Đối chiếu | qa_recheck_offline.py; qa_recheck_health.py — Sampled entry writeENOSPC và STOP trongsample cócleanup/shutdown/signals đúng. Chưa fullprocess mặc địnhR5 ghi JSON lỗi/STOP đồng thời. |
| AER-R01 | PASS | PASS | Kế thừa QA1 | test_session_outcome.py — Replay fire rc2: hàm6734 FAIL → outcome mới STOP/cleanup true; startup vẫn FAIL. |
| AER-R02 | PASS | PASS | Kế thừa QA1 | test_session_outcome.py — Replay mapping thiếuSTOP vẫn false/blocked. |
| AER-R03 | FAIL | PASS | Sửa + kiểm lại | AppPeer/controller thật qua fake transport: timeout→1 STOP→DONE/cleanup true; giữ runner_error. qa_recheck_offline và test_app_drain_transport; không chứng minh Pi vật lý. |
| AER-R04 | PASS | PASS | Kế thừa QA1 | test_app_pi_lifecycle.py — Laptop mất/hết30s tự cleanup hữu hạn; chỉ STATE/STOP khi đang drain. |
| AER-R05 | PARTIAL | PARTIAL | Mới/Đối chiếu | qa_recheck_offline.py; qa_recheck_runner.py — STOP tại 12 stage controller và lateSTATUS đạt; generated main fault8ca đạt. Chưa inject tín hiệu STOP vào từng phase main/process thật. |
| AER-R06 | PASS | PASS | Kế thừa QA1 | qa_full_operator.py; qa_mapping_entry.py — Operator thật qua bus giả ở7state; runner mapping/verify drain theo ownership. |
| AER-R07 | PASS | PASS | Kế thừa QA1 | test_session_outcome.py; test_stop_recovery.py — rc0/rc2 hoặc process gone không đủ để suy ra STOP. |
| AER-R08 | PASS | PASS | Kế thừa QA1 | test_recovery.py; test_session_outcome.py — Child crash/forced/live/missing audit tiếp tục khóa. |
| AER-R09 | PASS | PASS | Kế thừa QA1 | test_session_outcome.py — Remote startup error độc lập current group/child cleanup; không sửa DONE gốc. |
| AER-R10 | PASS | PASS | Kế thừa QA1 | qa_candidate_runner.py; test_view_bootstrap.py — Handoff monitor reap/retirement trước mission/operator; foreign/late view không cấp quyền. |
| AER-R11 | PASS | PASS | Kế thừa QA1 | qa_pre_sync_ui.py; test_recovery.py — Held/double/fast/slow/fail check chỉ một worker; kết quả UI giữ rõ. |
| AER-R12 | PASS | PASS | Kế thừa QA1 | test_recovery.py; test_stop_recovery.py — Marker/PID/boot/binding thay đổi không archive stale proof. |
| AER-R13 | PASS | PASS | Kế thừa QA1 | qa_pre_sync_ui.py; qa_pre_sync_faults.py — Phiên mới xóa nhãn STOP phiên trước. |
| AER-R14 | PASS | PASS | Kế thừa QA1 | test_stop_recovery.py; qa_mapping_entry.py[True] — New STOP-only không seal/SLAM/Nav/YOLO/teleop; link proof mới, giữ failure cũ. |
| AER-R15 | PASS | PASS | Kế thừa QA1 | test_stop_recovery.py; test_recovery.py — Wrong boot/livegroups/missing proof/lock/cancel giữ pending, không mission/ON. |
| AER-R16 | PARTIAL | PARTIAL | Mới/Đối chiếu | qa_recheck_offline.py; targeted_regression.xml — ENOSPC pending write/unlink và desktopreceipt đạt, worker kết thúc/khóa đúng. Chưa fault mọi remote/status/child receipt ở từng điểm. |
| AER-U01 | PASS | PASS | Kế thừa QA1 | qa_button_feedback.py; qa_pre_sync_ui.py — Held/click/keyboard/pressed pixels và busy label được Qt kiểm thật. |
| AER-U02 | PASS | PASS | Kế thừa QA1 | qa_map_selection.py — Popup không reset quaSTATUS, mouse selection chỉ một SELECT. |
| AER-U03 | PASS | PASS | Kế thừa QA1 | test_app_actions.py; test_supervisor_integration.py — Spam khởi động chỉ một worker, duplicate reject. |
| AER-U04 | PASS | PASS | Kế thừa QA1 | qa_pre_sync_ui.py; test_view_bootstrap.py — Tool/phase/STOP/lost/late status và CHECKING bị khóa đúng. |
| AER-U05 | PARTIAL | PARTIAL | Mới/Đối chiếu | qa_recheck_ui.py; layout_*.json — SEAL_PI12,5s vàblockers hiện đúng trongQt; chưađủ từng clock/SSH/TF/camera sátdeadline. |
| AER-U06 | PASS | PASS | Kế thừa QA1 | test_session_outcome.py; qa_pre_sync_ui.py — Startup/STOP/cleanup riêng; UI giữ lỗi/check result và log. |
| AER-U07 | PASS | PASS | Kế thừa QA1 | test_frontend_lease.py; qa_pre_sync_ui.py — Lease350ms bình thường giữ; late status không mở lại tools. |
| AER-U08 | PARTIAL | PARTIAL | Mới/Đối chiếu | qa_recheck_ui.py::test_native_dialog_and_focus_behaviour — Qt thật gõ tênmap_X_Q không STOP, SAVE đúngtên1lần; hủyAUTH gửiSTOP, FocusOut gửiK. Chưa đầyđủ Esc/STOP cùngmodal. |
| AER-U09 | PARTIAL | PARTIAL | Mới/Đối chiếu | qa_recheck_ui.py; targeted_regression.xml — HủyAUTH vàcloseworker/journal hồi quy đạt; chưa mọi close/auth/drain nhiều lần ởprocess thật. |
| AER-U10 | FAIL | PARTIAL | Sửa + kiểm lại | F3 đã qua 12 layout trong ma trận2 độ phân giải/3 scale, STOP/tiến độ thấy được; main clamp availableGeometry. Chưa phủ mọi thao tác kéo chia vùng/window manager/aspect stream robot. |
| AER-U11 | FAIL | PARTIAL | Sửa + kiểm lại | F2 đã qua6 ca360 W, text không kéo cửa sổ rộng; giữ nội dunglog. Phần camera stale toàn pipeline kế thừa vẫn chưa phủ đủ. |
| AER-U12 | PASS | PASS | Kế thừa QA1 | test_app_actions.py — 600fake actions, seed7/19/29, assert không chồng/reopen STOP. |
| AER-M01 | PASS | PASS | Kế thừa QA1 | qa_map_selection.py; test_supervisor_integration.py — Map cũ chọn được IDLE, snapshot map truyền runner. |
| AER-M02 | PASS | PASS | Kế thừa QA1 | test_supervisor_integration.py; test_map_save.py — Quét/lưu tên riêng/chọn mới rồi map cũ qua fixture. |
| AER-M03 | PASS | PASS | Kế thừa QA1 | test_map_save.py; test_supervisor_integration.py — Tên sai/rỗng/path/outsideallowlist không spawn/save. |
| AER-M04 | PASS | PASS | Kế thừa QA1 | test_map_save.py; test_map_catalog.py — Trùng/symlink/partial/race không ghi đè file user. |
| AER-M05 | PASS | PASS | Kế thừa QA1 | test_supervisor_integration.py — Map thay đổi sau chọn hoặc trong snapshot bị reject. |
| AER-M06 | PASS | PASS | Kế thừa QA1 | test_map_save.py; test_supervisor_integration.py — Duplicate save và STOP commit có rollback transaction. |
| AER-M07 | PARTIAL | PARTIAL | Mới/Đối chiếu | qa_recheck_offline.py; targeted_regression.xml — Atomicpair rollback ENOSPC ở generate/link1/link2/STOP cuối đạt. ros_map_saver helper test timeout/diskfull/nonzero đạt; mapping main dùng nested OwnedProcess generator, chưa fault đủ đường đó. |
| AER-M08 | PASS | PASS | Kế thừa QA1 | test_supervisor_integration.py; test_app_actions.py — Không chuyển mode khi phiên cũ chưa sạch hoặc còn pending. |
| AER-M09 | PARTIAL | PARTIAL | Kế thừa QA1 | test_frontend_lease.py; qa_mapping_entry.py — Q/lease/cleanup có test; chưa mô phỏng riêng focus mất trong teleop thật. |
| AER-M10 | PARTIAL | PASS | Mới/Đối chiếu | qa_recheck_ui.py::test_packaged_launcher_opens_idle_from_different_cwd; qa_recheck_runner.py — Launcher thật artifact b3 mở/đóng IDLE từ cwd códấucách, state tạm; generated runner nhận map path códấucách trongI/Ogiả. Save/catalog root códấucách kế thừa. |
| AER-N01 | PASS | PASS | Kế thừa QA1 | qa_supervisor_qt.py; test_recovery.py — Domain/route reject trướcSSH; không tự đổi network. |
| AER-N02 | PARTIAL | PARTIAL | Kế thừa QA1 | test_recovery.py; test_live_session_fake.py — Route/transport/auth cancel có coverage; chưa đủ hostkey/prompt lỗi bằng SSH thật. |
| AER-N03 | PASS | PASS | Kế thừa QA1 | test_pairing_and_status.py; test_pairing_transfer.py; test_live_session_fake.py — Split, duplicate, stale/path, session/packet và ownership được kiểm bằng fixture. |
| AER-N04 | PARTIAL | PARTIAL | Kế thừa QA1 | test_app_actions.py; test_live_session_fake.py — STOP ưu tiên và worker riêng; chưa ca SSH STATE timeout đủ mô hình mutationACK. |
| AER-N05 | PARTIAL | PARTIAL | Mới/Đối chiếu | qa_recheck_offline.py::test_transport_loss_does_not_restart_or_claim_stop_without_proof — Fake transport mất trướcPi/authwait/pairingread/running giữ proof thiếu vàkhông restart; chưa loss đúngI/O/SEAL trongmain+STATE chậm cùngSTOP. |
| AER-N06 | PASS | PASS | Kế thừa QA1 | test_frontend_lease.py; qa_full_operator.py — Freeze frontend đi Q/STOP qua operator thật+bus giả, backend giữ cleanup. |
| AER-N07 | PASS | PASS | Kế thừa QA1 | test_supervisor_integration.py; test_recovery.py — Pending tồn qua mởcontroller mới, chưa có proof không tựstart. |
| AER-N08 | PASS | PASS | Kế thừa QA1 | test_operator_owned_backend.py; test_recovery.py — Group/descendant/live PID không coi leader exit là sạch. |
| AER-N09 | PASS | PASS | Kế thừa QA1 | test_recovery.py — PID reuse/snapshot quyền/lỗi reject, không signal unrelated. |
| AER-N10 | PASS | PASS | Kế thừa QA1 | test_operator_channel.py; test_supervisor_integration.py — Overflow/JSON/NaN/duplicate/thừa field bị chặn, Q ưu tiên. |
| AER-N11 | PARTIAL | PARTIAL | Kế thừa QA1 | artifact_smoke.log; qa_supervisor_gui.py — Local app/assets mở không tải Internet; chưa thao tác dưới WiFi Pi mất Internet thật. |
| AER-N12 | PARTIAL | PASS | Mới/Đối chiếu | qa_recheck_offline.py::test_ten_start_stop_cycles_leave_no_fd_or_worker_growth — 10vòng cho fire và 10vòng mapping trên worker thật/endpoint giả; không tăng FD/thread, không pending quyền. Không phải10vòng ROS/GUI/phầncứng. |
| AER-H01 | NOT_RUN | PASS | Sync/build/static thật | 426 source + 103 installed Pi, 95 installed laptop; receipt cùng packet, lỗi rỗng, PLAN_ONLY, không runtime. verification_summary.json. |
| AER-H02 | NOT_RUN | NOT_RUN | Kế thừa QA1 | SESSION2 — Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H03 | NOT_RUN | NOT_RUN | Kế thừa QA1 | SESSION2 — Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H04 | NOT_RUN | NOT_RUN | Kế thừa QA1 | SESSION2 — Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H05 | NOT_RUN | NOT_RUN | Kế thừa QA1 | SESSION2 — Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H06 | NOT_RUN | NOT_RUN | Kế thừa QA1 | SESSION2 — Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H07 | NOT_RUN | NOT_RUN | Kế thừa QA1 | SESSION2 — Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H08 | NOT_RUN | NOT_RUN | Kế thừa QA1 | SESSION2 — Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H09 | NOT_RUN | NOT_RUN | Kế thừa QA1 | SESSION2 — Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
| AER-H10 | NOT_RUN | NOT_RUN | Kế thừa QA1 | SESSION2 — Pi/robot thật ngoài scope phiên1; không SSH/sync/launch/flash. |
