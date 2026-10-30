> **Lịch sử QA2 của b3b81081:** Ba finding bên dưới đã được sửa/kiểm lại
> trong ứng viên59cddc99; xem [báo cáo mới](FIX_AND_ROBOT_PATH_QA_2026-10-30.md)
> và [registry cập nhật](FIX_AND_VISUAL_REGISTRY_2026-10-30.md).
> Giữ nguyên kết quả và evidence cũ để đối chiếu.

# QA bổ sung phiên 1 — laptop, 30/10/2026

**Kết luận: ứng viên b3b81081 chưa đủ điều kiện đồng bộ hoặc chọn để vận hành.**
Đã xác nhận 1 lỗi P1 ở phục hồi sau seal timeout và 2 lỗi P2 về bố cục. Đợt
này chỉ thêm test/fixture và tài liệu; chưa sửa code vận hành. Cần sửa ở app,
kiểm lại các ca FAIL và hồi quy liên quan rồi mới sang bước Pi.

## Findings

### F1 — P1: adapter không gửi STOP khi Pi chờ laptop drain

File app: `services/app_peer.py:8`, gọi nguyên `SshPi.stop()` của parent.
Trạng thái app mới được đặt tại `services/app_pi_lifecycle.py:15–17`.
Đối chiếu parent **chỉ đọc**: `scripts/operational_peer.py:99` và `:125–128`.

Sau seal timeout, `AppPiController` chuyển sang `WAIT_LAPTOP_DRAIN` với
`error=PI_SEAL_HEALTH_TIMEOUT` và giữ I/O tối đa 30 giây. Nhưng transport kế
thừa coi mọi STATE có error trước DONE là exception. `stop()` hỏi STATE trước
khi gửi STOP; exception khiến nó bỏ qua gửi STOP và hỏi lại. Vì deadline
transport là 25 giây, nó có thể hết hạn trước cửa sổ 30 giây của Pi.

Tái hiện bằng hai class app thật, clock và SSH transport giả:

| Đối chứng | Thời gian giả | Số STOP gửi | Kết quả |
| --- | --- | --- | --- |
| Dừng bình thường | 1 giây | 1 | DONE, cleanup confirmed |
| Seal timeout → chờ drain | 25 giây | **0** | 25 STATE, vẫn WAIT_LAPTOP_DRAIN, cleanup unconfirmed |

Tác động: đường phục hồi vừa thêm có thể tiếp tục trả cleanup chưa xác nhận
và BLOCKED. Đây là lỗi tích hợp giữa state mới của app và quy tắc transport
cũ. Test trước kiểm controller/mailbox và adapter riêng nên chưa lộ nhánh này.
Không dùng kết quả giả lập để khẳng định đã gây sự cố vật lý; watchdog và các
node robot không được thay đổi trong QA.

Test FAIL:
`qa_recheck_offline.py::test_peer_sends_stop_while_app_pi_is_waiting_for_drain[True]`.
Trace đầy đủ: `peer_drain_trace.json`; nhánh bình thường cùng test vẫn PASS.

Đề xuất sửa: chỉ điều chỉnh adapter app để đọc/nhận diện trạng thái drain đã
ràng buộc đúng session, giữ lỗi khởi động nhưng cho phép chuỗi STOP và thu
proof. Tiếp tục từ chối START/RETRY/SEAL, wrong binding và request_error; không
sửa file transport R5 gốc, không kéo dài ngưỡng TF hoặc coi timeout là STOP.
Cần thêm ca timing/network chậm và lỗi identity sau sửa.

### F2 — P2: thông báo lỗi dài có thể kéo cửa sổ quá rộng

File: `native_rviz/session_ui.cpp:283–295` đưa toàn bộ lỗi vào QLabel;
`native_rviz/main.cpp:229–233` chỉ đặt WordWrap mà chưa có giới hạn nội dung
hoặc cơ chế ngắt chuỗi dài không có dấu cách.

Fault injection dùng thông báo `PI_SEAL_HEALTH_TIMEOUT` kèm 360 ký tự W liên
động, trong giới hạn lỗi khoảng 400 ký tự mà lớp app cho phép. Cả sáu cấu
hình màn hình/scale đều nở tới **7066 logical pixel**. Nội dung camera và nút
STOP vẫn tồn tại trong cửa sổ nhưng có thể nằm ngoài vùng màn hình người dùng.
Các thông báo bình thường không có hiện tượng rộng này trong bài kiểm.

Test FAIL: sáu tham số `long-*` của
`qa_recheck_ui.py::test_native_layout_fits_screen_and_keeps_stop_visible`.
Ảnh ví dụ: `layout_1920_1080_1.0_long.png`; số đo trong JSON cùng tên.

Đề xuất: giới hạn vùng lỗi, ngắt chuỗi dài hoặc đưa phần chi tiết vào vùng
cuộn; luôn giữ nút dừng trong vùng thao tác. Không cắt mất nguyên nhân trong
log. Kiểm thêm chuỗi đường dẫn dài thực tế sau sửa. Đây là phép thử đầu vào
xấu có kiểm soát, không khẳng định mọi log đang dùng đều gây lỗi.

### F3 — P2: bố cục cao quá màn hình 1366×768 ở scale 150%

File liên quan: `native_rviz/main.cpp:239–242` và bố cục camera/status/nút tại
`:324–359`. Camera có chiều cao tối thiểu 180 logical pixel, cộng các dòng
trạng thái và thanh điều khiển làm cửa sổ không co đủ nhỏ.

Ở scale 150%, vùng 1366×768 tương ứng khoảng 911×512 logical pixel. Với
thông báo bình thường ở SEAL_PI, cửa sổ thực thành **911×554** logical pixel.
Nội dung dưới vượt vùng màn hình dự kiến, chưa kể thanh tiêu đề hệ điều hành.
Năm cấu hình thông báo bình thường còn lại đạt bài kiểm. Không kết luận mọi
màn hình hoặc mọi scale 150% đều hỏng.

Test FAIL: tham số `normal-1.5-size0` của test layout nói trên.
Ảnh: `layout_1366_768_1.5_normal.png`, JSON ghi kích thước và trạng thái nút.
Đề xuất: giữ phần dừng cố định và cho vùng trạng thái cuộn/co theo chiều cao
màn hình; kiểm lại với cả RViz gắn thật và trang trí cửa sổ sau sửa.

## Những gì đã chạy và đạt

| Bộ | Kết quả cuối | Phạm vi |
| --- | --- | --- |
| Bổ sung offline | 44 PASS / 1 FAIL | STOP tại 12 stage controller; đầy ổ đĩa; nguồn health stale/future/NaN; mất transport; queue đầy; dừng và thử lại. FAIL là F1. |
| Qt và launcher | 12 PASS / 7 FAIL | 6 kiểm tra dialog/focus, 12 cấu hình layout, 1 launcher artifact thật từ cwd khác. FAIL là F2/F3. |
| Health bình thường | 2 PASS | Entry app thật ở hai mode, không thêm delay25ms, ROS localhost213. |
| Child failure | 8 PASS | Generated fire main + StartupSequence thật; fake nav/yolo/mission/operator lỗi spawn hoặc thoát; giữ lỗi gốc và drain laptop trước Pi. |
| Hồi quy có chọn lọc | 66 PASS | STOP/recovery/map/live worker/lease/mất ACK/mapping entry; không cần chạy lại toàn bộ suite phiên1. |
| Native CTest | 1/1 PASS | Frontend lease/protocol. |

**74 ca bổ sung duy nhất: 66 PASS, 8 FAIL**, quy về 3 findings. 66 ca hồi quy
được báo riêng. Không cộng rerun, snapshot hash hoặc số dòng registry thành
ca chức năng mới. Nguồn kết quả: `offline_final.xml`, phần Qt+health trong
`qa2_final.xml`, `child_faults.xml`; hồi quy tại `targeted_regression.xml`.

Các điểm đáng chú ý trong PASS:

- 10 vòng cho fire và 10 vòng mapping trên `LiveSession` thật với endpoint
  giả; sau warmup số FD và thread không tăng, laptop drain trước Pi. Đây là
  kiểm vòng đời worker, không phải 20 phiên robot/ROS đầy đủ.
- ENOSPC khi tạo map/link ảnh/link YAML, STOP trước hoàn tất; không còn cặp map
  dở hoặc ghi đè file cũ. Helper `ros_map_saver` được giả lập timeout/disk full/
  exit lỗi; generator `OwnedProcess` lồng trong mapping main còn thiếu fault
  injection đầy đủ và giữ PARTIAL.
- ENOSPC ở pending write/unlink hoặc desktop receipt không giữ dead worker
  hoặc tự mở khóa bằng kết quả thiếu. Chưa fault mọi file receipt/status.
- Qt thật: nhập `map_X_Q_2026` không phát STOP; SAVE gửi đúng một lần; hủy AUTH
  gửi STOP; callback FocusOut gửi phím K. Sự kiện FocusOut được inject trực
  tiếp, chưa chứng minh mọi hành vi window manager/Esc/modal/teleop phần cứng.
- Launcher `run_app.sh` mở/đóng artifact b3 ở IDLE từ cwd có dấu cách. Dùng
  XDG state tạm, localhost, không tác động pending hoặc cửa sổ robot đang dùng.
- TF lookup p95 ở normal load: R5 mode **103,383 ms**, sampled **80,139 ms**;
  không drop trace. Health io/pose vẫn false vì fixture không cung cấp toàn
  bộ robot. Chỉ đo observer TF, không gọi đây là readiness robot PASS và không
  đổi mode mặc định.

Bài child fault ban đầu thiếu các trường stable camera/localization/operator
trong fixture nên bị kẹt CAMERA, chưa đi tới lỗi child dự kiến. Đã bổ sung
fixture đúng contract, giữ assert cần thiết và chạy lại 8/8 PASS; không sửa
checker R5. Log ban đầu giữ riêng `initial_fixture_child_faults.*`, không
cộng những lỗi fixture này vào 8 FAIL sản phẩm.

## Registry và giới hạn

[Registry sau QA](QA_RECHECK_REGISTRY_2026-10-30.md) hiện có **65 PASS kế
thừa/đã kiểm, 18 PARTIAL, 3 FAIL, 10 NOT_RUN**. Đã hạ AER-R03 từ component
PASS thành integration FAIL sau phát hiện F1. QA phiên1 là lịch sử, không
thay log cũ để tạo cảm giác luôn đạt.

Các nhánh còn thiếu gồm toàn bộ Pi/hardware, hash stage Pi, rollback thật,
STATE chậm đồng thời STOP, một số phase main/saver lỗi ghi file, modal/đóng
cửa sổ và graph full pipeline. Lỗi TF/USB của phiên robot trước vẫn chưa được
chứng minh đã hết. Không SSH, LAN/WiFi Pi, đọc thiết bị, flash, chạy actuator,
commit hoặc đổi current trong QA này.

Chỉ bổ sung trong `apps/fire_robot_control/tests/` và docs. Build probe ở
`/tmp/fire_robot_pre_sync_ui`; mọi runtime test dùng thư mục tạm. Qt chạy
`offscreen`, domain211/localhost; diagnostic TF dùng213/localhost. Phép thử
layout dùng `configureUi` hiện tại qua entrypoint kiểm thử, chưa khởi tạo
Ogre/RViz hoặc window manager thật; ảnh đã được xem để đối chiếu số đo.

## Bảo toàn và kiểm lại

Snapshot trước/sau **1215 file giữ nguyên hash**, gồm source vận hành app,
launcher, R5 release/candidate/current cũ, hai wrapper và pending thật. Kiểm
thêm 95 file cài R5 trên laptop và manifest/archive/binary artifact b3 đạt.
425 pins app gồm 364 parent R5 nguyên byte. `current.txt` vẫn là6734c284…;
ứng viên b3 chưa được sửa hoặc build lại để che các FAIL này.

Bằng chứng trong [thư mục QA](qa_evidence/qa_recheck_20261030/), có
`protection_final.json`, raw/XML/ảnh và `SHA256SUMS`. `test_sources.sha256` pin
source kiểm thử; `production_sources.sha256` pin mã app đang được kiểm. Dữ liệu
SSH của bài peer là binding giả, không có secret phiên thật.

Để chạy lại các ca thuần sau sửa, từ root repo, trong subshell:

```bash
(
  source /opt/ros/humble/setup.bash
  source /home/huudinh/.codex/worktrees/laptop-baseline/NCHK_robotFirefighting_ROS/ros2_ws/install/local_setup.bash
  source .codex_qa_backups/r5_operational_candidate_20261026/ros/ros2_ws/install/local_setup.bash
  export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=211 ROS_LOCALHOST_ONLY=1
  export PYTHONPATH="$PWD/apps/fire_robot_control:$PWD/apps/fire_robot_control/tests:$PWD/releases/r5_20261029/ros/scripts:$PWD/releases/r5_20261029/ros/ros2_ws/src/fire_robot_mission:${PYTHONPATH:-}"
  python3 -B -m pytest -q -p no:cacheprovider \
    apps/fire_robot_control/tests/qa_recheck_offline.py \
    apps/fire_robot_control/tests/qa_recheck_runner.py
)
```

Qt cần build target `offline_supplement_probe` và `layout_supplement_probe`
trong `tests/pre_sync_ui_review`; sau đó chạy `qa_recheck_ui.py` với socket
cục bộ được phép. Các test dùng output `/tmp/fire_app_qa2_20261030`; lưu bộ
bằng chứng cũ trước khi rerun. `qa_recheck_health.py` cố ý từ chối output đã
có, phải chọn thư mục mới để bảo toàn trace. Không đồng thời chạy diagnostic
với một phiên R5/robot; guard sẽ từ chối. Không thay cấu hình sản xuất để ép
môi trường test đạt.

Bước tiếp theo: sửa F1 trong adapter riêng và F2/F3 trong bố cục app; chạy
lại 8 ca FAIL cùng hồi quy liên quan, chốt các PARTIAL ảnh hưởng phát hành,
đóng gói mới. Khi đó mới đồng bộ stage app riêng và nghiệm thu Pi có giám sát.
