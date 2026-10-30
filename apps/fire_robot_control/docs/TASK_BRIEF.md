> **Mới nhất — 857b9eb3 đã sửa relay xác thực, sync và mở:** 211 unittest /
> 160 integration / CTest đạt; hộp Qt accept/cancel đi qua worker-supervisor thật.
> Lần xác minh user trước lỗi NTP refresh 100 s, chưa I/O. R5/pending nguyên;
> bản mới chờ user bấm xác minh và nhập mật khẩu Pi khi hỏi.
> [Báo cáo/giới hạn](AUTH_RELAY_FIX_2026-10-30.md).

> **Mới nhất — a387d27e đã sửa và sync:** Nhánh phục hồi sau Pi reboot đã
> qua 205 unittest / 158 ca tích hợp / CTest; chuẩn bị chỉ đọc trên Pi thật đạt.
> R5/pending giữ nguyên. App mới đã mở BLOCKED, chờ user xác minh STOP mới;
> chưa I/O hoặc nghiệm thu robot. [Báo cáo và bằng chứng](PI_REBOOT_RECOVERY_FIX_2026-10-30.md).

> **Mới nhất 30/10/2026 — app 59cddc99 đã sync Pi và được chọn:** 426 source /
> 103 installed Pi, 95 installed laptop đúng hash; R5/bộ cũ giữ nguyên. App mới
> đang mở BLOCKED do hồ sơ cũ thiếu STOP, pending giữ nguyên. Chưa runtime
> quét/chữa cháy, motion/bơm hoặc nghiệm thu WiFi. [Báo cáo và bước tiếp](PI_STATIC_SYNC_2026-10-30_59cddc99.md).

> **Trước sync 30/10/2026 — ứng viên 59cddc99:** Đã sửa F1/F2/F3 và thêm
> xe nhìn từ trên xuống + đường kế hoạch Nav2. QA laptop 190 unittest,
> 158 pytest và CTest đạt; R5 giữ nguyên. Chưa sync Pi/chọn current hoặc
> nghiệm thu robot. [Báo cáo và bằng chứng](FIX_AND_ROBOT_PATH_QA_2026-10-30.md).

> **QA bổ sung 30/10/2026:** Bản b3b81081 chưa đủ điều kiện đồng bộ/chọn vận
> hành: đã xác nhận 1 lỗi P1 phục hồi STOP sau seal timeout và 2 lỗi P2 bố
> cục. Code vận hành/R5/current/pending giữ nguyên trong lượt QA.
> [Báo cáo QA](QA_RECHECK_SESSION1_2026-10-30.md) ·
> [Registry mới nhất](QA_RECHECK_REGISTRY_2026-10-30.md).

> **Cập nhật 30/10/2026:** Brief bên dưới là bối cảnh lịch sử lúc bắt đầu app.
> Tiếp quản trạng thái mới theo [báo cáo phiên 1](SESSION1_APP_ENTRY_QA_2026-10-30.md)
> và [registry QA](SESSION1_QA_REGISTRY_2026-10-30.md): artifact b3b81081 chưa
> chọn/sync, pending thật giữ nguyên; phiên 2 phải xử lý coverage còn thiếu và
> nghiệm thu Pi có giám sát, giữ nguyên R5.

# Task brief tiếp quản — Fire Robot Control

Theo format `docs/codex/reference/TASK_BRIEF_TEMPLATE.md`; đặt trong `apps/`
đúng nơi user yêu cầu. **Lượt lập tài liệu không cấp lệnh implement/hardware.**

## Task

Xây dựng theo từng mốc ứng dụng desktop trắng–đỏ–xám trên laptop để quét/lưu/
chọn map, khởi động chữa cháy, quan sát RViz thật+camera cùng cửa sổ và dùng
X/Q qua operator hiện hành; trước hết phải kiểm khả năng tích hợp native.

## File Cần Đọc Trước

Repo: `/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS`.

1. `AGENTS.md` và `docs/codex/PROJECT_CONTEXT.md` mới nhất.
2. Toàn bộ bộ tài liệu `apps/fire_robot_control/docs/`, bắt đầu REQUIREMENTS,
   SOURCE_AUDIT, DATA_CONTRACT, IMPLEMENTATION_PLAN rồi QA_PLAN.
3. `ai/00_core/system_instructions.md`, roles ROS2/QA, workspace/topic standards;
   chỉ đọc, không sửa ai/.
4. Runner/operator/mapping modules đã liệt kê trong SOURCE_AUDIT; inspect thật
   và `git status --short`, không dựa vào ảnh UI hoặc đường dẫn cũ trong README.
5. Source/release/receipt hiện hành; baseline snapshot0c54 ở tài liệu này chỉ
   đúng ngày khảo sát, có thể đã được thay ở chat khác.

## Scope

Sau khi user yêu cầu implementation theo lịch gộp, **phiên1 bao gồm M0–M5**:

- Được tạo PoC/native host hoặc panel, fixture UI giả và tài liệu ADR trong
  `apps/fire_robot_control/`; ở checkpoint M0 chưa mở backend live/SSH hoặc
  thay pinned source.
- Chứng minh Qt5/RViz thật, camera fake cùng cửa sổ, resize/close/font/tools.
- Sau PoC đạt tự nối M1–M5 theo plan, test đúng phần trước mở rộng; không cần
  hỏi user tiếp tục sau từng checkpoint. M6 là phiên nghiệm thu hardware riêng.

Các mốc tiếp có thể cần mapping wrapper, generic map selection và adapter
operator/runner/external-view. Đụng pinned file phải tạo/review bản integration
và receipt tương thích, không sửa sealed release hoặc hash baseline để qua gate.

Ngoài scope: mission core/firmware, safety/watchdog/QoS/network/domain/pin,
tự khám phá map, web app/joystick, publish motor/pump trực tiếp, promotion source
root hoặc push/commit khi chưa được yêu cầu. Không cài shortcut/autostart vào
home hay chạy robot chỉ vì đọc task brief.

## Hành Vi Hiện Tại

- Root mapping mở SLAM+RViz và terminal teleop; save qua map_saver, có kiểm
  tên/no-overwrite nhưng Pi lifecycle còn khác R5.
- R5 hiện có runner Pi/laptop/pairing/STOP/cleanup, operator OpenCV riêng;
  wrapper hỗ trợ910/TN1; core runner có `--map`.
- Actor chỉ cho hai reader `/pump_cmd`; UI muốn quan sát request đọc actor
  `/fire_mission/r4d_state`, không đăng ký thêm pump reader.
- Source current0c54 và published snapshot byte khớp local; Pi receipt model
  mới/runtime còn PENDING/NOT_TESTED theo context lúc khảo sát.

## Hành Vi Mục Tiêu

- Mở app bằng chuột, dùng offline khi laptop đã nối WiFi Pi.
- Quét→terminal lái xe→Save→chọn map mới/cũ→Start fire; app tự quản lý runner,
  pairing, chuyển mode và log; vẫn đặt Pose/Goal trong RViz.
- Một cửa sổ RViz native+camera/status; màu trắng–đỏ–xám, không đổi màu sensor.
- Observer không suy luận mission, không thêm owner hoặc pump reader.
- X/Q/window-close/crash đúng operator/lease/STOP/receipt; không fake readiness.

## Plan Implement

1. Refresh source audit và phạm vi từ user; ghi các delta so với snapshot này.
2. M0: PoC native RViz/Qt5, dữ liệu giả; quyết định kiến trúc bằng evidence.
3. M1–M2: UI/parser/fake lifecycle/ownership; không ROS/hardware live.
4. M3: map/TTY/save/catalog/generic path và transition đúng profile.
5. M4: X/Q/lease/external-view qua operator và runner hiện có; version đúng.
6. M5: offline integration, performance, launcher, docs/rollback.
7. M6: runtime hardware riêng nếu user cho phép, không lặp QA baseline vô cớ.

## Rủi Ro

RViz Qt ABI/binding, process ownership của external view, UI freeze nhưng
backend còn sống, map selection race, partial save, mapping Pi cũ, observer
làm thay exact endpoint counts, STOP confirmed bị nhầm với child cleanup rc2.
Chi tiết biện pháp và case nằm trong IMPLEMENTATION_PLAN/QA_PLAN.

## Verify

Trước code: diff/status/source pin/readability và xác minh scope. Sau M0 dùng
native build/test command đã ghi trong ADR; không mặc định có Python librviz.
Sau các mốc sau chạy fake/Qt/targeted regression và evidence theo QA_PLAN.
Các ví dụ command trong QA_PLAN là dự kiến, chưa tồn tại suite app hôm nay.

Manual/hardware check chỉ thuộc G3, user xác nhận phạm vi riêng. Không SSH/NTP,
launch robot, phát motor/bơm, flash hoặc commit/push ở lượt khảo sát/preview.

## Done Criteria

- M0 xong khi có RViz native thật/camera fake chung cửa sổ và QA tương ứng;
  không cần hardware để hoàn tất mốc này, không gọi toàn app DONE.
- App xong khi R01–R18 và các gate áp dụng đạt có evidence; giữ ngoại lệ user
  đã chấp nhận: terminal để lái khi mapping.
- Mọi known issue robot/runtime ngoài scope vẫn ghi đúng trạng thái.
- Handoff có source/profile/packet, log locations, case status, open issues và
  bước tiếp; không chỉ có ảnh giao diện và câu “đã chạy được”.

## Prompt tiếp tục đề xuất

> Đọc AGENTS.md, PROJECT_CONTEXT và bộ docs trong apps/fire_robot_control.
> Tôi yêu cầu bắt đầu phiên code gộp M0–M5. Kiểm RViz native trước rồi tự tiếp
> tục giao diện, map/TTY/save, runner/pairing và X/Q theo plan; QA xen kẽ từng
> checkpoint, không dừng hỏi sau mỗi mốc. Không chạy robot/SSH/NTP/actuator thật,
> không sửa logic điều khiển hoặc sealed release để làm UI. Giữ thay đổi có sẵn,
> ghi evidence QA, app mở thử được và những phần chờ nghiệm thu hardware M6.

Đoạn trên là prompt mẫu cho user sử dụng sau; không phải lệnh đang có hiệu lực.
