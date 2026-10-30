<!-- R5_SOURCE_PUBLICATION_20261029 -->
> **User yêu cầu commit/push ROS2 và ESP32 hiện hành.** Snapshot source có manifest được đưa vào `releases/r5_20261029` ở hai repo: ROS364 file packet0c54d9b79765 và firmware69 file/ELFce19. Root workflow nền được giữ riêng; README chỉ đúng bản R5 để tránh lấy nhầm source cũ. Hash/cú pháp source/shell và84 kiểm tools nền PASS; kế thừa QA R5 đúng phạm vi. Log/build/AI context/modelweights ngoài release. Publication không chạy robot hoặc flash; Pi packet0c54/runtime mới vẫn PENDING.

<!-- R5_MODEL_29102026_LAPTOP_READY_PI_PENDING_20261029 -->
> **User đổi model mới29/10:** candidate R5 dùng `best_29_10_2026.pt`/SHA28d3b25f, nhãn0:fire; confidence0,15 giữ. Packet3640c54d9b79765 khác parent8944 ở launch/model pin/README; firmwarece19 và logic robot không đổi. Load/CPU ảnh giả PASS, build perception/static95 laptop và15 launch/tools PASS. **Chưa runtime mới, Pi mới PENDING**, không coi smoke là kiểm accuracy/FPS/CUDA hoặc nghiệm thu robot. [Bằng chứng](docs/codex/current/R5_5_MODEL_29102026_READY_2026-10-29.md) · [Sync/map910](docs/codex/current/R5_5_MODEL_29102026_MAP910_SYNC_AND_RUN_2026-10-29.md). NEXT_STEP=USER_PI_STATIC_SYNC_0C54_MODEL29102026_THEN_AUTOMATIC_MAP910_RUNTIME. Không SSH/NTP/hardware/flash/commit; kế thừa QA50/71, không lặp.

<!-- R5_STATIONARY_REENTRY_PI_SYNC121336_PASS_20261029 -->
> **User sync121336 đã xác minh Gate0 hai host PASS:** packet364 `8944d2612c0c`; Pi build2package/9,02s, aarch64/errors[]/livefalse/runtimefalse;103/103 hash Pi và95/95 hash laptop khớp source sealed, map910/archive/tools đúng. Firmwarece19 không đổi. [Receipt/phạm vi](docs/codex/current/R5_5_STATIONARY_REENTRY_PI_SYNC121336_PASS_2026-10-29.md) · [Lệnh map910 mục2](docs/codex/current/R5_5_STATIONARY_REENTRY_MAP910_SYNC_AND_RUN_2026-10-29.md).
> NEXT_STEP=USER_ONE_AUTOMATIC_MAP910_RUNTIME_DELTA_8944_NO_REFLASH_NO_RESYNC. Kế thừaQA50+71+9, không cộng/lặpQA. Chưa runtime8944; USB/TF/Nav2cleanup vẫnOPEN. Lượt này chỉ đọc receipt/hash local, không SSH/NTP/hardware/flash/commit. Các marker Pi PENDING dưới là lịch sử trước receipt mới.

<!-- R5_STATIONARY_REENTRY_110629_QA50_SEALED_20261029 -->
> **Hai lỗi stationary110629 đã sửa/đóng gói:** ABORTED unused→rút request/OFF/STOP/nhả endpoint→cửa3mẫu mới cùng cycle; IR từ vùng phun→APPROACH→nhả stationary→đường IR initial/reentry thật, giữ UUID/step/task. Không fake COMPLETE hoặc dùng proof cũ. QA **50delta +71hồi quy ảnh hưởng +9tool PASS**,4baselineFAIL→PASS; fixture cũ thiếuactorreceipt cũngFAILtrước sửa, đã cập nhật đúngcontract. [Nguyên nhân, cáchQA, nodeid/giới hạn](docs/codex/current/R5_5_STATIONARY_REENTRY_110629_QA50_READY_2026-10-29.md).
> Packet364 `8944d2612c0c`/FWce19 không đổi, build2package/static95 laptopPASS; **Pi mới PENDING, chưa runtime8944**. NEXT_STEP=USER_PI_STATIC_SYNC_8944_NO_REFLASH_THEN_ONE_RUNTIME_DELTA. [Sync/lệnhmap910](docs/codex/current/R5_5_STATIONARY_REENTRY_MAP910_SYNC_AND_RUN_2026-10-29.md); không dùngsync/runbook3fba với laptop8944. Giữ6sACK/3lượt/YOLO0,15/IR3-500ms/khôngcapIR/khôngresetNav2 và watchdog/STOP. USB/TF banđầu/Nav2cleanup cònOPEN; không tựSSH/NTP/hardware/flash/commit hoặc lặpfullR4/Gate1. Marker raw110629 dưới là bằng chứng trước sửa, không ghi đè trạng thái sealed này.

<!-- R5_RUNTIME110629_PULSE_PASS_STATIONARY_GAPS_RAW_20261029 -->
> **Raw110629/Pi110556 đã SHA/đối chiếu:**53 laptop+7 Pi; packet364/3fba, ELFce19, map910, YOLO0,15 đúng.5/5 ACK ON đủ6,008–6,011s rồi OFF/home; chuỗi3 lượt trảNav2, chuỗi2 lượt mất mục tiêu hậu kiểm trảNav2; goal/X/Q xác nhận hoạt động. **Còn2P1 OPEN:** F1 stationary ABORTED giữ tới clear (11:10:26–11:11:04), probe hàm thuần xác nhận không reset qua false/true; F2 stationary range→APPROACH vẫn IR_CONFIRM~40s, thiếu chuyển tiến. Không nâng toànR5 thànhPASS. [Timeline/findings/giới hạn](docs/codex/current/R5_5_110629_PULSE_PASS_STATIONARY_GAPS_RAW_2026-10-29.md).
> User có resetESP sau mấtTF; raw phân biệt USB recovery11:15 không reboot với BOOT uptime reset11:16, ROS phục hồi/giữNav2 và không bắtPose mới. Nguyên nhân TF/USB ban đầu chưa chốt. Nav2cleanup5sSIGTERM/-15/RC2 vẫnOPEN sauQSTOP confirmed0,442s. NEXT_STEP=OFFLINE_DELTA_REVIEW_STATIONARY_ABORT_REENTRY_AND_RANGE_TO_FAR; source3fba/FWce19 không đổi. Không build/sync/flash/commit hoặc tựrobot; chỉ đọc oldPi logs/kernel quaLAN user cho phép. Kế thừaQA56+88+9, không cộng runtime/probe thành caQA PASS hay lặpR4/Gate1; softgap đangON chưa xảy ra trong phiên này.

<!-- R5_IDENTITY_RETRY_PI_SYNC102121_PASS_20261029 -->
> **User sync102121 đã xác minh Gate0 hai host PASS:** packet364 `3fbad10d5949`; Pi build2package/9,03s, receipt aarch64/errors[]/livefalse/runtimefalse;103/103 hash Pi và95/95 hash laptop khớp nguồn sealed. Map910 và tools/hash archive đúng; firmwarece19 không đổi. [Receipt/phạm vi](docs/codex/current/R5_5_IDENTITY_RETRY_PI_SYNC102121_PASS_2026-10-29.md) · [Lệnh chạy map910](docs/codex/current/R5_5_IDENTITY_RETRY_MAP910_SYNC_AND_RUN_2026-10-29.md).
> NEXT_STEP=USER_ONE_AUTOMATIC_MAP910_RUNTIME_DELTA_3FBA_NO_REFLASH_NO_RESYNC. Kế thừa QA56+88+9, không thêm/lặp QA. Chưa runtime3fba; USB/heading/JPEG/Nav2cleanup còn OPEN, Nav2 source không đổi. Lượt này chỉ đọc receipt/hash local, không SSH/NTP/hardware/flash/commit. Các marker Pi PENDING bên dưới là lịch sử trước receipt mới.

<!-- R5_IDENTITY_RETRY_085252_QA56_SEALED_20261029 -->
> **Đã sửa/đóng gói delta085252:** identity dùng chung500ms, gap quyền đã ACK→OFF/home/STOP→xét lượt mới cùng lửa/quota, không đợi clear; mấtYOLO3s trảNav2 và tái nhận khi lửa lại. QA còn tìm/sửa cachedFIRING chặn IR tiến lại sau ngắt; canonical review giữ ABORTED thật. GUI mẫu IR lượt mới đúng. **56ca mới +88hồi quy ảnh hưởng +9tool PASS**, không tính trùng rerun. Packet364 `3fbad10d5949`/FWce19 không đổi; build/static95 laptopPASS, **Pi mới PENDING**, chưa hardware/flash/commit/SSH/NTP. Nav2 offline domain212:2caACTIVE/RC0~1,3s, có transition warnings nhưng chưa tái hiện5sSIGTERM, **không sửa Nav2**. [Báo cáo/cáchQA](docs/codex/current/R5_5_IDENTITY_RETRY_085252_QA56_READY_2026-10-29.md) · [Sync/map910](docs/codex/current/R5_5_IDENTITY_RETRY_MAP910_SYNC_AND_RUN_2026-10-29.md).
> NEXT_STEP=USER_PI_STATIC_SYNC_3FBAD10D5949_NO_REFLASH_THEN_ONE_RUNTIME_DELTA. Không dùng sync/runbook5324 cũ với laptop3fba. Giữ6sACK/3lượt/YOLO0,15/IR3-500ms/khôngcapIR/khôngresetNav2; USB/Nav2cleanup/physical heading cònOPEN.

<!-- R5_RUNTIME085252_CENTER_IR_PUMP_JITTER_RAW_20261029 -->
> **Raw085252/Pi085212 đãphântích/SHA:**50laptop+7Pi snapshotkhớp, source5324/ELFce19/map910đúng. Hai cycleno-goal,6ACKON:1lượtngắt bởi sameepochidentityage356,496ms>350 (IRright926/71ms, epoch7khôngđổi, khôngUSBmất tại08:56:41),5lượt đủ6,018–6,028s. Sauinterrupt, cờretryclear đòiYOLOclear+3IR>20000 nênfirecũkhôngphunlại dùcenter/range; UIthiếuđiềukiệnIRclear. Cycle2căn lại→quotareset→3lượt vàtrảNav2/X/newgoalđạt; QSTOPconfirm, RC2cònNav2cleanupSIGTERMchild-15. [Timeline/findings/sourceprobe](docs/codex/current/R5_5_085252_CENTER_IR_PUMP_IDENTITY_JITTER_RAW_2026-10-29.md).
> NEXT_STEP=REVIEW_SAME_EPOCH_JITTER_COMMITTED_ON_AND_INTERRUPTED_PULSE_RETRY_POLICY. Lượt nàychỉphântích/probehàmthuần, khôngsửasource/build/sync/flash/commit/hardware hoặc lặpQA70/native51/affected56/R4/Gate1. Giữpacket5324/ce19; Gate0PASS, runtimePARTIAL/P1OPEN. ĐãđọcPilog/kernelcũquaLAN; khônglaunchROS/NTP hoặcđiềukhiểnrobot.

<!-- R5_USER_UPLOAD083537_PI_SYNC083606_PASS_20261029 -->
> **Upload và sync mới đã xác minh PASS:** upload083537 raw9SHA/RC0/BINARY_AFTER_RC0, esptool4vùng; ELF `ce19ecd3f4fa`. Sync083606 build2package8,98s, packet364 `5324a9c0f0d8`, Pi103/103 hashes và laptop95/95 khớp. **Gate0 hai host PASS; chưa runtime/BOOT mới.** [Receipt/phạmvi](docs/codex/current/R5_5_UPLOAD083537_PI_SYNC083606_PASS_2026-10-29.md). Không cần nạp/sync lại; dùng [mục3 runbook map910](docs/codex/current/R5_5_LINK_HOME_RETRY_MAP910_SYNC_AND_RUN_2026-10-29.md). Runtime hiện **WiFi Pi10.0.0.1, rút LAN** theo preflight; LAN chỉ sync, lời khuyênruntimeLAN trước đã đính chính.
> NEXT_STEP=USER_AUTOMATIC_MAP910_RUNTIME_DELTA_NO_REFLASH_NO_RESYNC. Không lặpQA70/native51/affected56/R4/Gate1; không tựSSH/NTP/COM/hardware/flash/commit. G3/USB/JPEG/cleanup runtimeOPEN. Các marker pending bên dưới là trạng thái trước receipt này.

<!-- R5_LINK_HOME_RETRY_QA70_SEALED_20261029 -->
> **Đã sửa/đóng gói:** IR/xoay STOP→retry3s cùng cycle; ESP tự localhome sau linkloss với drive interlock, OFF mới trước newON; tách navigation readiness khỏi servo/HOME lỗi; IR monitor sống qua recovery, UI đúng blocker/rawage; TARGET_LOST sau X không giữ goal mới. **50QA hành vi+20tool PASS,51native PASS,56hồi quy ảnh hưởng PASS.** Packet364 `5324a9c0f0d8` /ELF `ce19ecd3f4fa`; laptopbuild/static95 PASS, **chưaPi sync/flash/runtime bộmới**. [Sửa/cáchQA/registry](docs/codex/current/R5_5_LINK_HOME_RETRY_QA70_READY_2026-10-29.md). [Lệnh nạp/sync/map910](docs/codex/current/R5_5_LINK_HOME_RETRY_MAP910_SYNC_AND_RUN_2026-10-29.md).
> NEXT_STEP=USER_UPLOAD_CE19_THEN_SYNC_5324_THEN_AUTOMATIC_MAP910_DELTA. Không dùng ea538/runbook cũ với source này. Giữ6s từACK,3lượt,YOLO0,15,IR3/500ms,khôngcapIR/khôngresetNav2. Không tựhardware/SSH/NTP/flash/commit. G3/USB/JPEG/cleanup runtime OPEN; các marker bên dưới là lịch sử và không ghi đè trạng thái mới này.

<!-- R5_SCREENSHOT070619_RELATED_WARNINGS_AUDIT_DISCUSSION_20261029 -->
> **Ảnh block07:06:19 đã đối chiếu toàn raw/receipt, chưa code/QA mới:** YOLO0/5/fresh, IRraw127ms nhưng preview181s vì node bịretire. GUI giữlýdoUSB banđầu và SESSION_NOT_ARMED_OR_FAULT, thiếublocker thực `WAIT_NEW_POST_PULSE_HOME`/servoLOCKED; “đang xửlýlửa” còn từcyclecũ. Nav2nhận3goal/NoProgress, QSTOP/epoch/home unconfirmed, Navspin188errorlặp/remoteSTOPfail; camera1,41sgap/TFstartup đãhồi. Lidar/USBsauhậuphiên khôngquyngược rootcausephun. XtrongHOLD cógapstatic, rawkhôngcóX nên chưa gọitestFAIL. [Ảnh/từngnhómlỗi/phạmvi](docs/codex/current/R5_5_070619_BLOCK_SCREENSHOT_RELATED_LOG_AUDIT_2026-10-29.md).
> NEXT_STEP=DISCUSSION_ONLY_WAIT_USER_IMPLEMENTATION_REQUEST. Giữea538/b1dc, khônghardware/SSH/NTP/build/sync/flash/commit; khôngđổiQAđãđạtthànhđãphủmọibug.

<!-- R5_DISCUSSION_LOCAL_HOME_ON_LINK_LOSS_IR_PREVIEW_GAP_20261029 -->
> **Tiếp tục chỉ bàn luận, chưa code:** user muốn ESP tựhome khi mấtlink (pumpOFF/driveSTOP, MCU còn hoạt động), lưu counter rồi báo ACT khi nối lại; không đợiROS mới bắt đầuhome. Raw065737 xác nhận IRraw đã hồi:07:03:28 tuổi6,871ms,07:06:19 tuổi127ms; preview phânloại cũ9,78s→180,7s vì nodeIR bị retire cùngmission và recovery chờhome. Đề xuất giữpreview sống/táchGUIraw-vs-preview. [Chi tiết và yêu cầu mới](docs/codex/current/R5_DISCUSSION_STALL_RETRY_USB_HOME_NAV_RECOVERY_2026-10-29.md).
> NEXT_STEP=DISCUSSION_ONLY_WAIT_USER_IMPLEMENTATION_REQUEST. Không sửasource ea538/ELFb1dc, QA/build/sync/SSH/NTP/hardware/flash/commit.

<!-- R5_DISCUSSION_RETRY3S_USB_HOME_NAV_NO_CODE_20261029 -->
> **User yêu cầu chỉ bàn luận, chưa code.** Cycle1 IR065737 không chạy vì user chưa cắm nguồn motor; ghi nhận nguyên nhân này, không quy PWM/odom bug. Đang bàn STOP→retryIR/xoay khoảng3s không cần clear-fire, tách phục hồi navigation khỏi actuatorLOCKED/home sau mấtUSB. Đây là đề xuất, chưa implement/QA; source ea538/ELFb1dc giữ nguyên. [Ghi chú và các điểm cần chốt](docs/codex/current/R5_DISCUSSION_STALL_RETRY_USB_HOME_NAV_RECOVERY_2026-10-29.md).
> NEXT_STEP=CONTINUE_DISCUSSION_WAIT_USER_IMPLEMENTATION_REQUEST. Không build/sync/flash/runtime/SSH/NTP/commit hoặc lặpQA trong lượt trao đổi này.

<!-- CURRENT_R5_RUNTIME065737_USB_PULSE_HOME_LOCK_RAW_20261029 -->
> **Runtime065737/Pi065702: PARTIAL PASS, recovery/cleanup FAIL (RC2).** Raw laptop49 snapshot SHA khớp; Pi5 snapshot từng file tính SHA trên cùng byte remote/copy (nguồn Pi vẫn chạy, không giả before/after ổn định). Hai cycle no-goal: lần1 IR `ODOM_PROGRESS_LOST` (1cm/1s) rồi READY; lần2 tiến0,2981m, IR3 mẫu/right908, phun1 COMPLETE6,086s từACK→OFF/home counter1→tựphun2. Phun2 bị USB CMD write timeout lúc07:03:13.585; user không thao tác USB/nguồn. Kernel cùngboot xác nhận disconnect18.461/reconnect; ELF b1dc đúng, uptime/seq không reset. Sau hồi ACTLOCKED/home1, recovery `WAIT_NEW_POST_PULSE_HOME` giữ mission/gateway nên Nav2 nhận3goal mà xe không chạy. QSTOP/home không confirmed, remoteSTOP mấttransport; runnerPi cũ vẫn sống. [Timeline, nguyên nhân và giới hạn](docs/codex/current/R5_5_065737_TWO_FIRE_CYCLES_USB_PULSE_RECOVERY_RAW_2026-10-29.md).
> NEXT_STEP=REVIEW_USB_MID_PULSE_LOCKED_RECOVERY_AND_SEPARATE_NAV_GATEWAY_FROM_ACTUATOR_READINESS. Chưa sửa source/QA/sync/flash mới, không lặp QA đã đạt; giữ364pins ea538, Gate0/upload PASS. Agent chỉ đọc/copy log/journal/process quaLAN user đã cho phép, không tựactuation/NTP/flash/commit/kill. G3 heading/USB/JPEG/cleanup OPEN. User dừng Terminal1 cũ trước phiên mới; chưa cần lặp robot để tìm lại lỗi đã rõ trong raw.

<!-- CURRENT_R5_POST_ACK_USER_UPLOAD_PI_SYNC_PASS_20261029 -->
> **User nạp ESP32 và sync Pi đã PASS:** upload064527 raw9 SHA khớp, RC0/BINARY_AFTER_RC0, esptool xác minh cả4 vùng flash; ELF `b1dc2925e2bb`. Sync064550 build2package/10,4s, 364pins và Pi103/103 installed hashes khớp candidate, errors[]. Gate0 hai host packet `ea538dfbe3d1` PASS (laptop95 kế thừa). [Receipt, giới hạn và bước tiếp](docs/codex/current/R5_5_UPLOAD064527_PI_SYNC064550_PASS_2026-10-29.md).
> NEXT_STEP=USER_ONE_AUTOMATIC_DELTA_EA538_NO_REFLASH_NO_RESYNC. Chưa runtime/BOOT identity với bản mới; G3 heading và các mục runtime OPEN giữ nguyên. Chỉ xác minh receipt, không tính thêm ca QA hay lặp QA23/47/159/native22/R4/Gate1; agent không SSH/NTP/hardware/flash/commit. Các mục pending bên dưới là lịch sử trước receipt mới.

<!-- CURRENT_R5_UPLOAD_CACHE_FIX_QA23_20261029 -->
> **Uploader063717RC2/mấtbinary đãsửa:**8rawSHA PASS, PlatformIOchecksumđổi doLIBDEPSenv→xóabuildcachetrướcnobuild; chưaghiflash. Rebuildoffline4SHAkhớp nguyênbảnb1dc2925, sao5fileflash_bundlebênngoài.pio, directlocalesptoolcùngCOM/offsets/flags.23QAtoolPASS; khôngđổiROS ea538 hoặcfirmwaresource/ELF, khônglặpQA47/159/native22/R4/Gate1. [Raw/cause/fix/lệnhchạylại](docs/codex/current/R5_5_UPLOAD_CACHE_CLEAN_063717_QA23_FIX_2026-10-29.md).
> NEXT_STEP=USER_RETRY_SAME_UPLOAD_COMMAND_THEN_SEND_LOG_UPLOAD; chưaautoCOM/flash/runtime/SSH/NTP/commit. NếuPichưaea538thì syncfiletheorunbook; khôngresyncROS chỉvìuploaderfix.

<!-- CURRENT_R5_POST_ACK_HOME_QA47_SEALED_20261029 -->
> **QA bổ sung và sửa xong:47ca mới+159hồi quy Python PASS; native22earlyOFFhome PASS (trước18/22khônghome).** PWM_OFF/frozenACT sauACK ngắt đúng, interruptedR5 đủOFF/home/STOP tựtrảNav2 khôngX và khôngreplayP, Xmuộn cùngtoken/context vẫnnhả; oldfixtures8caseđãcậpnhật vàPASS. Firmware mới b1dc2925e2bb chỉ optin earlyOFFhome khi intentionalOFF+livehost+freshzero; hardabort không tựunlock. Packet364/ea538dfbe3d1, buildROS/FW/staticlaptop95PASS; **chưaflash/Pisync/runtime**. [Finding, cáchQA, file/line và giới hạn](docs/codex/current/R5_5_POST_ACK_RECOVERY_HOME_QA47_READY_2026-10-29.md).
> NEXT_STEP=USER_UPLOAD_NEW_HOME_FIRMWARE_THEN_SYNC_EA538DFBE3D1_PI_RECEIPT_THEN_ONE_AUTOMATIC_DELTA. Không dùng packetedaf/nháp904 hoặc firmware52c307 vớisource hiện tại. Giữ6s từACK, IR3/500ms, YOLO0,15,3lượt/căn-tiếnthật resetquota, khôngcapIR/resetNav2. Không tựhardware/SSH/NTP/flash/commit hoặc lặpR4/Gate1. Các mục bên dưới là lịch sử trước bản này.

<!-- CURRENT_R5_IR3_PULSE6_ACK_QA64_SEALED_20261029 -->
> **Đã implement theo user cho phép: packet364/edaf21db481a sealed, QA64delta PASS +273hồi quy PASS;4fixture cũ vẫnFAIL y hệt trước sửa, ghi riêng.** IR3sequence/median5, R5bỏcap0,30m/15s/20s,6s từfirstfreshESP_PWM_ON_ACK;3lượt liên tiếp, căn/IR thực sự hoàn tất cấp lạiquota. OFFđánhgiá ngay/ON-motor mới chờHOME. F05clearflap và F06mấtpublisherperception trongcommittedON đã sửa/QA; source lạ/STOP/serial/ACT vẫn chặn. BuildROS/FW/staticlaptop95PASS; firmware52c3074991f5 candidate riêng, **chưaflash/Pisync/runtime**. [Báo cáo/registry/cáchQA](docs/codex/current/R5_5_IR3_PULSE6_ACK_QA64_READY_2026-10-29.md).
> NEXT_STEP=USER_UPLOAD_PINNED_ACK6_FIRMWARE_THEN_SYNC_PI_STATIC_RECEIPT_THEN_ONE_AUTOMATIC_DELTA. Không tự hardware/SSH/NTP/flash/commit hoặc lặpR4/Gate1/QAđãđạt. Nguồn trước8adc5b và các mục “chỉ bàn luận/chưa code” dưới là lịch sử đã được thay bằng yêu cầu implement mới; không dùng runbook/stage8adc5b với source hiện tại.

<!-- R5_DISCUSSION_POST_OFF_HOME_ACT_ACK_CONFIRMED_20261029 -->
> **Tiếp tục bàn luận; chưa implement.** User đồng ý kiểm YOLO/IR ngay sau OFF, lượt ON kế tiếp chờ servo sẵn sàng; xác nhận bảng sau phun: còn centered/trong vùng thì tiếp lượt, lệch thì căn, xa thì tiến, căn/IR lại hoàn tất thì cấp lại ba lượt. Hết ba lượt còn lửa thì trả Nav2 và chờ mất lửa/thấy lại; không cần goal mới. Quy tắc reset do hành vi thực sự hoàn tất và motor trong lúc servo về còn là chi tiết đang bàn. ESP32 đã có ACT khoảng 5 Hz báo PWM ON/OFF/servo/home, **không phải xác nhận nước chảy**; ROS đã dùng on_ack, còn timer yêu cầu mới vẫn từ first host ON. Source hiện vẫn 5 mẫu/5 s/cap IR cũ; F05/F06 OPEN, không QA/sync/hardware/flash/commit. [Thảo luận và bằng chứng nguồn](docs/codex/current/R5_BEHAVIOR_DISCUSSION_3_IR_SAMPLES_6S_PULSE_2026-10-29.md).
> NEXT_STEP=CONTINUE_DISCUSSION_WAIT_EXPLICIT_IMPLEMENTATION; 6 s cần đổi đồng bộ firmware cap 5.000 ms và ROS, chưa được phát hành.

<!-- R5_DISCUSSION_UPDATE_IR3_PULSE6_NO_IMPLEMENT_20261029 -->
> **Vẫn chỉ bàn luận, user chưa chấp nhận cho code.** User mới yêu cầu3mẫu IR xácnhận, mỗi lượt bơm6s từfirsthostONcommand (khôngMCUACK), đánhgiáYOLO/IRngay sauOFF khôngchờhome; chu kỳlửamới khôngcầngoal. Giữ3lượt liên tiếp, căn/IRlại cấpquota, hết3lượt cònlửa trảNav2/chờclear→fire. Source chưađổi:5mẫu/5s/capIRcũ, F05/F06 cònOPEN. Đãđọc lại proof/IRstop/resume: ROS hardcap5s vàFW92ffd8 PumpSafety5000ms nên6s cầnđổicảhai saukhi usercho phép; chưaflash/QA/hardware/sync/commit. [Trao đổi chi tiết](docs/codex/current/R5_BEHAVIOR_DISCUSSION_3_IR_SAMPLES_6S_PULSE_2026-10-29.md).
> NEXT_STEP=DISCUSSION_ONLY_WAIT_USER_ACCEPTANCE_BEFORE_IMPLEMENTATION. KiểmYOLO/IRsớm khôngđồngnghĩachoON/motortrướcHOME; chi tiết này cònđangdiễngiải.

<!-- R5_DISCUSSION_ONLY_WAIT_USER_ACCEPTANCE_20261029 -->
> **User yêu cầu chỉ bàn luận/phân tích, chờ chấp nhận mới được code.** Đã giải thích/capture đủ30phase phiên và các lớp trạng thái camera, chưa sửa production/QA/sync/hardware/flash/commit. [Bảng trạng thái và quy tắc đang bàn](docs/codex/current/R5_CAMERA_STATES_AND_BEHAVIOR_DISCUSSION_2026-10-29.md). User muốn bỏ IR0,30m/15s/I20s, giữ mấtYOLO3s→trảNav2 và thấy lại lửa→căn/IR khôngkẹt. Mỗi lượt bơm5s→OFF/home→kiểmYOLO/IR→lượt tiếp, tốiđa3lượt **liên tiếp**; căn/IR lại thì cấp quota mới; hết3lượt cònlửa trảNav2/chờ mất rồi thấy lại. TổngON một sự kiện có thể>15s khi quota được reset, khác cap3lượt toàncycle hiện tại. Chưa chốt chi tiết resetquota theo chuyểnphase hay hànhvi thực sự hoànthành; hiện5s đếm từfirstONrequest, chưa từPWM-ACK. F05/F06 vẫnOPEN; source364/8adc5b giữ nguyên, static haihost kếthừaPASS.
> NEXT_STEP=DISCUSS_ONLY_WAIT_USER_EXPLICIT_IMPLEMENTATION_APPROVAL. Các trảlời chốt hànhvi không phải lệnh cho code. Model nhẹ đọc bảng; mạnh khi user yêu cầu implement/QA.

<!-- CURRENT_R5_5_BEHAVIOR_AUDIT32_TWO_OPEN_20261028 -->
> **Audit hành vi/5raw mới nhất, source8adc5b khôngđổi:32ca unique=20PASS chức năng+5FAIL/2nhóm+7diagnosticPASS.** F05P1 trướcphun cảhai clear→PREPARE require_clearTrue, lửatrởlại→WAIT_CLEAR/X khôngthoát dùSTOP; clearthật/Q cònthoát. F06P1 mấtpublisherYOLO/preview/IRraw trongON→graphmissing/armfalse, cắtON0,1–0,2s; sauON1s cũngcắt1,1–1,2s. Chưa sửa source/sync/hardware/flash/commit. QA69 vẫnPASS đúngphạmvi cũ, khônggọiallđãhếtbug. [Điềukiện từngbước, raw/hash vàQA32](docs/codex/current/R5_5_BEHAVIOR_CONDITIONS_LATEST5_QA32_2026-10-28.md).
> Raw293laptop+35PiSHA PASS; nămcuối194144/194336/205542/210916/213722 đều85720/146194, **chưa runtime8adc5b**.194144Lidar/dev/ttyUSB0ENOENT;205542IRcap0,323m+cancelwait40s;210916REALIGN6lần/IRUUIDwait+deadline, ADCphải tấtcảmẫughi>1000 nênchưaP;213722ARMED_IDLE/khônggoalnhậntừraw. Confidence0,15 khớpsource/installed/log. Chưađóng USB/JPEG/Navcleanup/G3/FWearly-OFFLOCKED/falsefire.
> NEXT_STEP=FIX_F05_PRESPRAY_CLEAR_FLAP_AND_F06_COMMITTED_PULSE_GRAPH_THEN_5_FAILED_DELTA; modelmạnh sửa/integration. **Chưa cần user chạyrobot/sync đểtáihiện lỗiđãrõoffline**; khônglặpQA69/R4/Gate1. Static8adc5b haihost kếthừaPASS, source364pinsgiữSHA.

Các mục dưới là lịch sử trước audit32; mô tả giữ5s trước đó đúng với mất/trễ payload nhưng chưa phủ endpointperception biếnmất. Không đổi5FAIL mới thànhPASS hoặc xóaQA69 cũ.

<!-- CURRENT_R5_5_FOUR_BUG_FIX_QA69_PI_PASS_20261028 -->
> **Bốn nhóm kẹt audit48 đã sửa trong candidate; QA69 unique PASS, build/hash laptop95 và Pi103 PASS.** Packet364/8adc5b8400f8, sync223158_ZGe60a `R5_FOUR_BUG_FIX_PI_TOOLS_READY=1`; runtime mới NOT_TESTED. F04 terminal4/5/6 thật trước handoff→retire unused cycle→lửa còn hiện xử lý no-goal; F02 giữ cancel receipt thật, dữ liệu/STOP hồi thì bỏ latch guard để nhả; F03 UNKNOWN cùngGID chờzero/tên đúng tựhồi, quá0,75s retire; F01 căn reentry theo current preflight. Không thêm trạng thái/phím, không resetNav2, giảcancel/home, gia hạnIbudget, hardware/flash/commit. **9FAIL cũ→9PASS;34 boundary mới+26 hồi quy liênquan, không cộng rerun.** [QA/cách kiểm](docs/codex/current/R5_5_FOUR_BUG_FIX_QA69_READY_2026-10-28.md) · [Runbook mới](docs/codex/current/R5_5_FOUR_BUG_FIX_SYNC_AND_RUN_2026-10-28.md). Đã đồng bộ, không cần sync lại; dirty checkout/firmware giữ nguyên, các lỗi USB/JPEG/G3/early-OFFLOCKED/Navcleanup/falsefire vẫnOPEN riêng.
> NEXT_STEP=USER_ONE_8ADC5B_AUTOMATIC_DELTA_THEN_RAW_HASH; không lặp fullQA/R4/Gate1. Model nhẹ chạy lệnh, model mạnh phân tích raw/protocol.

Các mục bên dưới là lịch sử của source trước sửa; audit48/9FAIL được giữ đúng raw, không còn là verdict source8adc5b hiện tại. Không dùng stage146194/runbook cũ với laptop source mới.


- 28/10/2026: [Camera-only cho xem/thu dữ liệu](docs/codex/current/CAMERA_ONLY_FOR_TRAINING_2026-10-28.md): native camera15FPS + viewerrawremap; không sourcechange/sync/flash/hardware doagent, không inference/robotcontrol. Lệnh bashsyntax/ASTviewer kiểm; chưa chạy camera. Không thay auditR5/4bugsOPEN; bước tiếp userview, model nhẹ.
<!-- CURRENT_R5_5_COMPREHENSIVE_AUDIT_48_20261028 -->
> **QA bổ sung rộng packet146194:48ca unique=31PASS chức năng+9FAIL chức năng/4nhóm+8diagnostic xác nhận lỗi.** F04P1 action4/5/6 terminal đúng lúc firecycle bind→WAIT_REAL_GOAL_CANCELED không thể đủ; F02P1 cmd/odom gap0.8s lúcIR→guardIR_APPROACH_CONTRACT_INVALID không được retirement whitelist, X+clear vẫnkẹt; F03P1 UNKNOWN0.15s lúc tạo raw→FAULT chưa có pin/khôngrunnerrecovery; F01P2 stale mission/knownrawmetadata→COMPLETE reason không reentry/ALIGNINGzero mãi. Hai gap lúcALIGN WAIT_STOP đã kiểmactualFSM/rebindPASS, không gọi lock. Startup10, jitter3,3pulse→clear→goalmới→newcycle2 PASS. **Không sửaproduction/sync/hardware/flash/commit**, SHA364pins trước/sau giữ146194, static haihost cũ vẫn đúng. [Từng finding/code/cáchQA/registry](docs/codex/current/R5_5_COMPREHENSIVE_AUDIT_48_FOUR_OPEN_BUGS_2026-10-28.md).
> NEXT_STEP=FIX_F04_TERMINAL_RACE_F02_IR_RETIREMENT_F03_DDS_ACQUISITION_F01_TURN_REENTRY_OFFLINE_THEN_DELTA_QA; chưa cần user chạy robot để tìm lại lỗi đã rõ. Model mạnh sửa/integration; model nhẹ chạy lệnh sau khi sửa+seal.

Các mục dưới là lịch sử trước audit mở rộng; QA42 PASS đúng phạmvi đã kiểm, không có nghĩa toànpacket đã hếtlỗi. Runbook146194 hiện chờ sửa4nhóm trên trước runtime tiếp, không cần sync lại chỉ vì audit.

<!-- CURRENT_R5_5_SIMPLE_FLOW_QA42_194336_20261028 -->
> **Raw194336/Pi194311 SHA51+7PASS: không cóI/P/ON; alignmentCOMPLETE lặp chờ no-goalhandoff rồi DDSUNKNOWN cùngGID đãpin gâyFAULT.** Serial1→8 đã phục hồi giữNav2/GUI; QSTOPconfirmed0.4035s nhưngcleanupRC2/NavfallbackOPEN. Đã sửa giữrawGIDknown quaUNKNOWN, centeredchờSTOP thayvì bậtquaylại, cùngcyclecăn lại sau ngắt và guard tách released khỏicentered; bỏ timeout30s mặcđịnh. QA42unique=33mới+9liênquan PASS; packet364/146194cf4648 laptopbuild/static95PASS, Pi build/static103/sync202807 PASS; **runtime mới NOT_TESTED**. Bơm3×5s/15s, IR0.30m/15s+5s, YOLO0.15/IR500ms/TN1/FW92ffd8 giữ. Không resetNav2, hardware,flash,commit; USB/JPEG/G3/early-OFFLOCKED/Nav2cleanupOPEN. [Raw/QA](docs/codex/current/R5_5_194336_SIMPLE_FLOW_RAW_QA42_2026-10-28.md) · [Runbook mới](docs/codex/current/R5_5_SIMPLE_FLOW_SYNC_AND_RUN_2026-10-28.md).
> NEXT_STEP=USER_ONE_146194_AUTOMATIC_DELTA_THEN_RAW_HASH_NO_RESYNC; không lặpQA42/65/107/R4/Gate1. Model mạnh raw/protocol, model nhẹ chạy lệnh.

Các mục dưới là lịch sử. Packet85720 có runtime lỗi194336, không còn NOT_TESTED; dùng bản146194 sau khi static hai host PASS, không dùng stage/runbook cũ với source mới.

<!-- CURRENT_R5_5_KEEP_NAV_EPOCH_QA65_PI_PASS_20261028 -->
> **Theo yêu cầu user, runtime Serial đổi epoch không còn reset Nav2/bắt Pose mới.** Packet364/85720b23318e: runner chỉ retire/rebuild mission; Nav2/AMCL/RViz/YOLO/operator GUI/health/Pi giữ. Operator ghép lease trong tiến trình cũ bằng HOLD→RESUME đúng session/generation/nonce, current zero/OFF/home/IO/localization và ARM3s. Goal SUCCEEDED/CANCELED/ABORTED không kích restart; action còn ACTIVE có thể tiếp tục, terminal thì nhận goal mới, không gửi lại goal/P. Trước phun còn lửa được xử lý attempt mới; sau phun không replay chưa rõ kết quả. QA65 unique=40mới+25liênquan PASS; build/static95 laptop+103Pi/sync190712 PASS, **runtime mới NOT_TESTED**. Firmware early-OFF LOCKED, USB/JPEG/G3/Nav2cleanup OPEN; không hardware/flash/commit, dirty checkout giữ. [Chi tiết/QA](current/R5_5_KEEP_NAV_EPOCH_QA65_READY_2026-10-28.md) · [Lệnh automatic đúng packet](current/R5_5_KEEP_NAV_EPOCH_SYNC_AND_RUN_2026-10-28.md).
> NEXT_STEP=USER_ONE_85720B_AUTOMATIC_RECOVERY_AND_FULL_PULSE_DELTA_THEN_RAW_HASH; không cố ý rútUSB/ngắtphun để lặp lỗi biết trước, không fullR4/Gate1/QA107/91/77. Model nhẹ chạy lệnh; model mạnh đọc raw/protocol. Giới hạn3 mission rebuild/600s còn giữ (cooldown hữu hạn), không phải Nav2 restart.

Các mục dưới là lịch sử trước packet85720b. Mô tả reset Nav2/đặt Pose mới trong các báo cáo cũ chỉ đúng source cũ; không dùng runbook5159 với laptop nguồn mới.

<!-- CURRENT_R5_5_181042_LOCKED_181526_USB_RECOVERY_RAW_20261028 -->
> **Raw mới packet5159:**181042/Pi180942 chưa mởRViz vì ACTservoLOCKED/home0, serialHEALTHYepoch1/khôngquarantine; không phải mấtUSB làmstartupfail.181526/Pi181427 đãARM/goal,18:19:09 S3USBdisconnect→epochfault→retireoperatorGUI/Nav2restart→WAIT_INITIAL_POSE; YOLOvẫn~15FPS. Sau đó18:20:22 LidarUSBdisconnect, /scan stale68.61s; chưaposemới/scan nên chưareopenGUI. **User xácnhận córút/cắmUSB/hub**, không gọi tấtcả là USBtựrụng; rawkhôngchứngminhPose gâyđóngGUI. SHA39+7/47+7PASS +kernelđúngboot; RC2/auditFAILgiữ, P0→phun5sNOT_EXERCISED. Source/firmwarekhôngđổi, khôngQA/sync/hardware/flash/commit. [Bằng chứng và giải thích](<docs/codex/current/R5_5_181042_LOCKED_181526_USB_RECOVERY_RAW_RESULT_2026-10-28.md>).
> NEXT_STEP=REVIEW_RECOVERY_GUI_LIFETIME_AND_USB_DEVICE_RECOVERY_DISTINGUISH_USER_HOTPLUG; model mạnh phân tích, model nhẹ chạy lệnh. Không trộn trạngtháiLOCKEDcũ với serialdisconnect mới; không lặpfullR4/Gate1/QA107/91/77.

Các mục dưới là lịch sử. Packet5159 đã có runtime tớiNav/recovery nhưng nhánh phun5s vẫnchưađượcđi tới; QA offline đúngphạmvi vẫngiữ.

<!-- CURRENT_R5_5_FULL_PULSE_5S_QA107_PI_READY_20261028 -->
> **Đã sửa theo yêu cầu: R5 xác nhận IR/YOLO trước ON; đã phun thì giữ đủ5s rồi OFF/home mới hậu kiểm IR/YOLO.** Perception None/stale/đổi vùng/lệch tâm không ngắt lượt; STOP/Q/X, heartbeat/serial/epoch, quyền/cancel, command/odom và ACT vẫn kiểm liên tục. Giữ3lượt/tổng15s; trước lượt tiếp đòi proof mới. QA107 delta=69mới+38liênquan PASS, thêm3 native trace dùng lõi firmware92ffd8 đều3ON/3home/0LOCKED. Packet363/5159a47d0a2b build/static94/102 haihost, sync180117 PASS; **không cần sync hoặc flash lại, runtime mới NOT_TESTED**. Kế thừa startupQA91. Firmware early-lock khi ngắt thật giữa sweep vẫnOPEN; không giảhome. USB/JPEG/G3/Nav2shutdown cònOPEN. [QA/chính sách](<docs/codex/current/R5_5_FULL_PULSE_5S_QA107_PI_READY_2026-10-28.md>) · [Lệnh automatic](<docs/codex/current/R5_5_FULL_PULSE_5S_SYNC_AND_RUN_2026-10-28.md>).
> NEXT_STEP=USER_ONE_5159_FULL_PULSE_OFF_HOME_REVIEW_HANDBACK_DELTA_THEN_RAW_HASH. Model nhẹ chạy lệnh, model mạnh phân tích raw/firmware. Không fullR4/Gate1/QA77/91 lặp, hardware/flash/commit.

Các mục dưới là lịch sử. Kế hoạch sửa firmware early-home chưa triển khai; user đã chọn trước mắt hoàn thành5s khi perception bị che. Không đổi FAIL raw165751 thành PASS.

<!-- CURRENT_R5_5_STARTUP_QA91_SERVO_EARLY_ABORT_OPEN_20261028 -->
> **Runtime165751/Pi165715 packet99df: startup qua, P được nhận/ACT ON; lệnh ON0.901s rồi OFF vì IR_NOT_CONFIRMED_SPRAY_RANGE; firmware92ffd8 chuyển LOCKED/home0 khi ngắt trước góc cuối.** Đây là mismatch firmware early-stop chỉ home sau fullsweep với R5 ngắt/retry; C++ đúngconfig tái hiện. Nav2 vẫn nhận goal nhưng quyền xe chưa trả; QhomeproofFAIL/RC2. Pi epoch3 ổn quanh phun;2STATE gaps xảy ra trướcARM, USB/JPEG/G3/Nav2cleanup OPEN. **Startup164430 đã sửa: tách timer localization khỏiIO, vẫn strict currentpose+IO1s, epochreset; CONSUMERS180s hữu hạn/logblocker. QA91=37mới+54startupreg PASS; packet363/7c17cedfaf0e build/static94/102 haihost/sync172738 PASS. Firmware chưa sửa/flash, runtime7c17 NOT_TESTED; chưa cần user runtestphun/sync lại.** [Raw/cause](<docs/codex/current/R5_5_165751_PULSE_INTERRUPTED_SERVO_LOCKED_RAW_RESULT_2026-10-28.md>) · [StartupQA91/Gate0](<docs/codex/current/R5_5_STARTUP_LOCALIZATION_QA91_PI_READY_2026-10-28.md>).
> NEXT_STEP=FIRMWARE_EARLY_OFF_BOUNDED_HOME_OFFLINE_QA_THEN_SHARED_R5_ACTUATOR_INTEGRATION; model mạnh phù hợp. Không lặpQA77/91/R4/Gate1, hardware/flash/commit; không xóa LOCKED/giả home trênROS.

Các mục dưới là lịch sử. RuntimeNOT_TESTED99df được bổsung164430FAILstartup/165751FAILphun, không phủnhận QA offline đúng phạmvi.

<!-- CURRENT_R5_5_STARTUP_164430_RAW_CHECKPOINT_20261028 -->
> **Runtime164430/Pi164354 packet99df: FAIL STARTUP_TIMEOUT:CONSUMERS trước AUTO.** Raw45 laptop/7Pi SHA PASS; CONSUMERS30s hết hạn trong khi pose vừa hồi phục2.27/5s, io/camera/yolo/operator_present đều đang true. Pose phụ thuộcIO nên identity blip4.28ms reset chuỗi; Pi có4STATE timeout504–591ms,4open failures by-id missing,epoch1→9. Không kết luận chỉ WiFi hay USB đãfix. Runner recovery QA77 chỉ khởi tạo sau READY nên chưa được kiểm. STOPconfirmed/Pi8child sạch; Nav2fallbackRC-15 vàtransportSTATE giữOPEN. Source chưa đổi, không QA/hardware/flash/commit. [Raw/checkpoint](<docs/codex/current/R5_5_STARTUP_164430_RAW_QUOTA_CHECKPOINT_2026-10-28.md>).
> NEXT_STEP=FIX_BOUNDED_PREARM_STARTUP_RECOVERY_AND_POSE_IO_COUPLING_THEN_DELTA_QA. Quota còn8%, dừng checkpoint; model mạnh sửa protocol, model nhẹ chạy lệnh. Chưa cần user runtest/sync lại packet99df.

Các mục dưới là lịch sử tại thời điểm phát hành; runtime NOT_TESTED của packet99df đã được bổ sung FAIL startup ở báo cáo trên, không phải QA77 mất giá trị.

<!-- CURRENT_R5_5_RUNTIME_RECOVERY_QA77_PI_READY_20261028 -->
> **Đã sửa phục hồi phiên sau serial/Nav2, QA77=49 mới+28 hồi quy liên quan PASS.** Packet363/99df43f647d1 sealed; build laptop/Pi và hash94/102 hai host PASS, sync161153. Automatic abort attempt cũ→owned STOP drain→fresh observer/zero/OFF/home→Nav2 mới→**user đặt Initial Pose mới**→six adapter/operator mới. Giữ Pi/camera/YOLO/RViz; không replay goal/P. Chỉ cùng bridge pairing;3 restart/600s, Q ưu tiên. Trước phun YOLOclear3s; đã phun thì bothclear3s trước attempt mới, navigation không bị khóa bởi clear gate. Sửa early-goal fire latch chưa forward và feedback A khi hold; thêm SERIAL_QUARANTINE reason. Không đổi YOLO0,15/IR500ms/firmware. USB/JPEG/G3/Nav2shutdown/false fire OPEN, runtime mới NOT_TESTED. [QA/hành vi](<docs/codex/current/R5_5_RUNTIME_RECOVERY_QA77_PI_READY_2026-10-28.md>) · [Lệnh](<docs/codex/current/R5_5_RUNTIME_RECOVERY_SYNC_AND_RUN_2026-10-28.md>).
> NEXT_STEP=USER_ONE_AUTOMATIC_DELTA_99DF43_THEN_RAW_HASH_NO_RESYNC_NO_FLASH. Model nhẹ chạy lệnh, model mạnh phân tích raw. Không hardware/flash/commit hoặc lặp fullR4/Gate1/QA107/77. Các báo cáo trước giữ đúng raw/giới hạn lịch sử.

<!-- CURRENT_R5_5_RUNTIME_152148_FALSE_FIRE_SERIAL_NAV2_20261028 -->
> **Runtime mới152148/Pi152118 packet4ca3: FAIL trước IR/P.** Raw46 laptop/7Pi SHA khớp remote/copy;2 goal SUCCEEDED. User xác nhận không có lửa, nhưng15:26:11 core tạo fire cycle→báo nhầm fire; không có ảnh/box để xác định người hay nền. Lớp person không kích fire theo code, confidence0,15 giữ. WAIT_STOP chưa xong thì serial reopen4→5→6→7,15:26:27 hardfault BOOT_OR_SERIAL_EPOCH_CHANGED; Nav2 mất bond4000ms, lifecycle reset/statusGID đổi nên không goal mới. BOOT uptime/seq liên tục, chưa chứng minh ESP reset hay nguyên nhân reopen. P accepted=0/pulseindex0, ba lượt NOT_EXERCISED; không gọi runtime PASS. Qdrain STOP_UNCONFIRMED epoch/RC2, Pi8clean; JPEG3448/Nav2cleanup/USB vẫn OPEN. [Raw/hash/kết quả](<docs/codex/current/R5_5_152148_FALSE_FIRE_SERIAL_EPOCH_NAV2_RAW_RESULT_2026-10-28.md>).
> NEXT_STEP=OFFLINE_EXACT_GOAL_CANCEL_NAV2_BOND_AND_SERIAL_REOPEN_RECOVERY_REVIEW; model mạnh phân tích/protocol, model nhẹ tra lệnh. Không QA/sync lặp, hardware, flash hoặc commit. Các mục runtime NOT_TESTED bên dưới là lịch sử trước log mới, QA107 offline vẫn giữ đúng phạm vi.

<!-- R5_THREE_PULSES_COMPLETION_REAUDIT_20261028 -->
> **Kiểm lại sau gián đoạn:** packet361/19 file sửa/11 artifact và laptop94 SHA khớp; xác minh107 PASS trong XML gốc, không chạy lại QA/source/build. Code ba lượt đã hoàn tất. Receipt Pi102 sync145929 PASS giữ nguyên; đọc lại remote hiện tại timeout LAN, chưa xác nhận lại file Pi trong lượt này. Runtime4ca3 NOT_TESTED. [Chi tiết](<docs/codex/current/R5_5_THREE_PULSES_QA107_READY_2026-10-28.md#kiểm-lại-sau-các-lần-gián-đoạn--2026-10-28>). NEXT=user một lượt automatic delta/raw; model nhẹ chạy lệnh, model mạnh phân tích. Không hardware/flash/commit.

<!-- CURRENT_R5_5_THREE_PULSES_QA107_PI_PASS_20261028 -->
> **Đã implement user chọn3 lượt×5s/tổng15s ON: QA107=64mới+43hồi quy liênquan PASS; packet361/4ca3b53732d7 seal, Gate0 hai host PASS(laptop94/Pi102,sync145929,errors[]). Runtime mới NOT_TESTED.** OFF/home thật+settle2s+5IRmẫu mới trước retry; căn lại cùng cycle/UUID, không thêm Ibudget/quãng tiến. Counter không reset do gap/recenter, hết3 lượt/30s retry thì chờ clear hoặc X. X sau phun→STOP/OFF/home/exact cancel/6ACK→Nav2 dù còn lửa; thu hồi edge chưa ON không đòi phantom home. Goal chen ACTIVE được cancel sau STOP. ACTOR_TRANSITION bổ sung trace. [Hành vi/giới hạn/QA107](docs/codex/current/R5_5_THREE_PULSES_QA107_READY_2026-10-28.md) · [Lệnh automatic mới](docs/codex/current/R5_5_THREE_PULSES_SYNC_AND_RUN_2026-10-28.md).
> NEXT_STEP=USER_ONE_AUTOMATIC_THREE_PULSE_DELTA_AND_RAW_HASH_NO_RESYNC. Không flash/commit/hardware/fullR4/Gate1/QA87. Giữ firmware/dirty checkout, YOLO0,15,IR500ms,TN1,15FPS. USB/JPEG/G3/Nav2cleanup OPEN; model nhẹ chạy lệnh, model mạnh phân tích raw/ownership.

Các mục dưới là lịch sử trước packet4ca3; các đề xuất “chưa code”, Pi pending và one-pulse bên dưới đã được cập nhật bằng mục mới. Không dùng stage67b77f với laptop nguồn mới.

<!-- CURRENT_R5_5_AUTOMATIC_RUNTIME_123410_20261028 -->
> **2026-10-28: tìm lại runtime123410/Pi123336 packet67b77f.** Laptop44 files/Pi7 remote SHA khớp; Pi sync122937 static101/errors[] bổ sung trạng thái pending cũ. Hai cycle tới P accepted (goal thật/no-goal); user thấy bơm chạy; first handback READY/new goal SUCCEEDED. Cycle2 realign/IR→P, Q trước READY cuối, không suy thành FAIL hoặc PASS handback2. Hai identityhold cùng epoch tự hồi0,55s. Startup USB recovery counter0→1/openfail5 vàepoch1→7→8→9→10 đã tái hiện; không đủ bằng chứng MCU reboot/nguồn/cáp. JPEG2300 warning; Nav2 shutdown5s SIGTERM fallback làmRC2; STOP confirmed0,403s/Pi8clean/allgroups exited. Không có raw ACT liên tục để đo ON/sweep từng lượt. [Báo cáo đầy đủ](docs/codex/current/R5_5_123410_AUTOMATIC_RUNTIME_RAW_RESULT_2026-10-28.md).
> NEXT_STEP=REVIEW_NAV2_SIGINT_SHUTDOWN_AND_USB_STARTUP_FROM_RETAINED_EVIDENCE; DISCUSS_BOUNDED_POSTSPRAY_RETRY_BEFORE_IMPLEMENTATION. One-pulse policy hiện chờ clear sau phun; retry chỉ là đề xuất, chưa implement. Không lặp QA87/R4/Gate1 hoặc tự chạy hardware/flash/commit. Model mạnh phân tích; nhẹ tra lệnh.

Các mục dưới là lịch sử trước receipt122937/runtime123410, không phải trạng thái Pi pending hiện tại.

<!-- CURRENT_R5_5_TRANSIENT_QA87_20261028 -->
> **R5.5 đã sửa phục hồi report/graph, callbackstall, coast sauzero, goalchen lúcphun, IRrange trướcstep0 vàlatecancel. QA87=65mới+22hồi quy bịảnhhưởng PASS, build/laptopstatic93 PASS; packet360/67b77f088b59 sealed. Pi bản cuối PENDING:** LANUPnhưng10.42.0.185NoRoute/neighborFAILED, chưacopy; khôngdùngrunbookd966/nháp8fcbvớilaptopmới. Runtime NOT_TESTED. Lỗi tạmthời giữzero/OFF, freshsameboot+STOP0,5s tựtiếp; deadlineALIGN30s/IR0,30m15s+5s giữ. Hardepoch/source/unsafeactuator/cancelwrongUUID giữ. [QA/giới hạn](docs/codex/current/R5_5_TRANSIENT_RECOVERY_QA87_READY_2026-10-28.md) · [Sync/lệnh mới](docs/codex/current/R5_5_TRANSIENT_RECOVERY_SYNC_AND_RUN_2026-10-28.md). MD/ledger/history đãcậpnhật, khônghardware/flash/commit/fullR4/Gate1.
> NEXT_STEP=USER_SYNC_67B77F_INPUT_RECOVERY_THEN_ONE_AUTOMATIC_DELTA_RAW_HASH; model mạnh phân tíchraw, model nhẹchạylệnh. G3/USB/serial/JPEG/Nav2cleanup OPEN.

Các mục dưới là lịch sử packetd966 trởvềtrước; Gate0Pi cũ không xác nhậnpacketmới đãcài.

<!-- CURRENT_R5_5_RECOVERY3_QA62_PI_READY_20261027 -->
> **Sửa F01/F02/F03 đã QA delta62 PASS +1 fixtureFAIL cũ, Gate0 hai host PASS packet360/d966d85af7db.** Pulse ABORTED→retire kết quả INTERRUPTED với OFF/home/STOP/cảhai-clear; IRreentry vùngphun/close xác nhận đúng UUID/cycle/Ibudget cho cả goal thật/no-goal; status thật từ bt_navigator refresh5Hz để no-goal fire đầu phiên, không giả status/goal. Laptopstatic93/Pistatic101 sync214952 PASS; **runtime mới NOT_TESTED**, R5.5 chưa nghiệm thu toànchuỗi. YOLO0,15/IR500ms giữ. Nhápde019c90 đã thay; không dùng runbook85. [Fix/QA62](docs/codex/current/R5_5_RECOVERY3_QA62_PI_READY_2026-10-27.md) · [Runbook mới](docs/codex/current/R5_5_RECOVERY3_SYNC_PASS_AND_RUN_2026-10-27.md). Không chạyhardware/flash/commit.
> NEXT_STEP=USER_ONE_D966D85_AUTOMATIC_RECOVERY_DELTA_THEN_RAW_HASH; không lặp QA62/R4/Gate1/sync đã đạt. Model mạnh đọc raw/protocol; model nhẹ đủ chạy lệnh. G3/USB/serial/JPEG/Nav2cleanup OPEN.

Các mục dưới là lịch sử; FAIL audit85 giữ nguyên bằng chứng trước sửa, không phải trạng thái source hiện tại.

<!-- CURRENT_R5_SCENARIO_AUDIT_38_20261027 -->
> **2026-10-27 QA offline toàn các nhánh user nêu, packet85c2a0 giữ nguyên:** 38ca khác nhau, 28PASS chức năng +6FAIL +4PASS chẩn đoán. F01 P1: midpulsefireloss hoặc postpulseIRonly→actorABORTED, đãOFF/home/clear vẫn kẹt handback; X không thoát, QSTOP confirmed có/khônggoal. F02 P2: repeatedYOLOblink150/300/600ms rồi IR950 lúcreentry→khôngphun, sau budget20s+retire→READY/goalmới; không kẹt vĩnhviễn nhưng functionalFAIL. Source/default/installedYOLO0,15; nguồn packet355/hash không đổi. Không thêm runtime, không chạyhardware/flash/commit hoặc sync/build. [38ca, cách QA, giới hạn và quy tắc không lặp](docs/codex/current/R5_SCENARIO_QA_REGISTRY_2026-10-27.md).
> **F03 P2 bổ sung:** phiên mới chưa nhận status Nav2 (khác status[]), dù có lửa vẫn WAIT_FRESH_NAV_STATUS; M11 FAIL. Đính chính phạm vi no-goal QA trước: đã có status snapshot. Chưa sửa source trong audit.
> NEXT_STEP=FIX_F01_POSTPULSE_ABORT_RETIREMENT_F02_IR_RANGE_REENTRY_F03_FIRST_NAV_STATUS_BOOTSTRAP_THEN_FAILED_DELTA_QA; dùng model mạnh sửa/proof, model nhẹ tra registry. Runtime85c2a0 chưa nghiệmthu; cleanupNav2/USB/G3/JPEG OPEN.

<!-- CURRENT_R5_5_201558_REACQUIRE_YOLO15_20261027 -->
> **2026-10-27 R5.5/201558:** packeted3311, 44 laptop/7 Pi raw files hashed; Pi key files remote SHA match. Goal-fire T mất lửa, timeout3s→READY; new no-goal fire stuck WAIT_NO_GOAL_HANDSHAKE51,6s, no I/P/ON. Pi serial epoch6 stable during stuck. New packet85c2a0 retains Nav2 status across core reset; no-goal proof wait8s/target loss3s bounded, YOLO confidence0,15, 3/5 and pump/STOP gates unchanged. QA delta6+11+8, build laptop, static laptop93/Pi101 PASS sync204739; new runtime PENDING. Nav2 SIGTERM cleanup RC2 remains OPEN. [Details](docs/codex/current/R5_5_201558_REACQUIRE_YOLO15_PI_SYNC_PASS_2026-10-27.md). No robot run/flash/commit.
> NEXT_STEP=USER_ONE_AUTOMATIC_REACQUIRE_DELTA; model mạnh đọc raw mới, model nhẹ chạy runbook.

<!-- CURRENT_R5_5_192535_IR500_SERIAL_20261027 -->
> **2026-10-27 R5.5/192535:** packet9f6734 có fire→handoff→T; mất YOLO sau0,401s, no-goal cycle mới rồi Pi serial epoch3→4→5/`BOOT_OR_SERIAL_EPOCH_CHANGED`. READY goal từng hiện ở1793104304.120 nhưng bị re-latch trước fault; chưa IR_CONFIRM/P/ON. Lượt mới **không PASS** và không chứng minh PWM yếu. IR500 packeted3311/354 pins đã QA delta (4 mới, 231 impacted PASS, 4 baseline FAIL lặp archive), build+static Pi/laptop PASS sync200518; runtime PENDING. USB reconnect nguyên nhân vật lý, G3 heading, JPEG warning, Nav2 cleanup fallback OPEN. [Chi tiết](docs/codex/current/R5_5_192535_IR500_SERIAL_EPOCH_RESULT_2026-10-27.md).
> NEXT_STEP=SAFE_SERIAL_RECONNECT_RECOVERY; model mạnh cho sửa protocol, model nhẹ cho runbook. Không commit/flash/chạy robot.

<!-- CURRENT_R5_5_164835_IR_CONFIRM_20261027 -->
> **Runtime packet9daade 164835/Pi164714: IR_CONFIRM20s, chưa có P/ON; READY trở lại.**
> RightADC862–946 trong vùng nhưng actor proof chưa COMPLETE/đăng ký bơm;
> exact blocker chưa được log liên tục. IR None ở tuổi371/328ms>300ms,
> Pi serial khỏe, 5/5remoteSHA; không kết luận nhiễu vật lý hoặc đổi ngưỡng.
> Lượt sau mất YOLO trong WAIT_STOP và timeout3s trước ALIGN: cần sửa timer.
> QSTOP confirmed, laptop Nav2cleanupRC−15 OPEN. Packet9daade giữ nguyên,
> chưa fix/seal mới hoặc runtime PASS. [Raw/checkpoint](docs/codex/current/R5_5_164835_IR_CONFIRM_RAW_CHECKPOINT_2026-10-27.md).
> NEXT_STEP=FIX_TARGET_TIMEOUT_AFTER_HANDOFF_AND_ADD_STATIONARY_PROOF_REASON_THEN_DELTA_QA.
> Model mạnh phân tích proof; model nhẹ chạy sync khi packet mới seal.

Các mục dưới là lịch sử trước raw164835; QA41+6 vẫn PASS nhưng runtime mới chưa đạt phun.

> **Quota checkpoint: còn7% primary, dừng sau seal/build laptop; Pi mới PENDING.** [R5_5_SIMPLE_TASKS_QA41_SEALED_QUOTA_CHECKPOINT_2026-10-27.md](docs/codex/current/R5_5_SIMPLE_TASKS_QA41_SEALED_QUOTA_CHECKPOINT_2026-10-27.md). Không lặp QA41+6 đã đạt.

## R5.5-153207 — stationaryIR, simple task outcomes QA41

<!-- CURRENT_R5_5_153207_SIMPLE_TASKS_QA41_20261027 -->
> **Runtime153207/Pi153135 packetdc724: kẹt stationaryIR; chưa phun, Q STOP0.502s.**
> Raw laptop44byte-match/Pi5remoteSHA; graphFAULT cũ không tái hiện, serialepoch
> đổi trướcARM. CleanupNav2 fallback/RC−15 còn OPEN. Sửa stationaryrecenter,
> cửa sổproof mới, boundedALIGN/IR và ngắt trướcphun→STOP/OFF→Nav2 dù cònIRfire;
> chờ YOLO clear/newfire mới thử chu kỳ kế. GUI một tác vụ/kết quả. Giữ Q/watchdog,
> identity/owner/OFF/home/pulse5s. **QA41 mới+6 regression, build/static93 PASS;
> packet352/9daadeb94c78 sealed. Pi sync mới PENDING, runtime NOT_TESTED.**
> Không dùng stagedc724 với laptop mới; không flash/commit/fullQA/R4/Gate1.
> [Raw/fix/QA](docs/codex/current/R5_5_153207_SIMPLE_TASKS_QA41_READY_2026-10-27.md) · [Sync/lệnh](docs/codex/current/R5_5_SIMPLE_TASKS_SYNC_AND_RUN_2026-10-27.md).
> NEXT_STEP=USER_SYNC_9DAADE_SIMPLE_TASKS_SOURCE_ONLY_THEN_ONE_AUTOMATIC_DELTA.
> Model mạnh đọc raw/review; model nhẹ đủ chạy lệnh đã chốt.

Các mục dưới là lịch sử trước runtime153207; NOT_TESTED/TN1 và Gate0 dc724 chỉ áp dụng thời điểm cũ.

## R5.5-MAP-TN1 — đổi map phiên tiếp theo, chỉ verify offline

Theo user chọn fire_room_TN1.pgm, thêm `--map` trỏ YAML TN1 tuyệt đối trong
runbook hiện hành/Terminal2 và block3. Marker chính đã TN1; R5 runner dùng default
9_10 nếu thiếu tham số nên chỉ marker không đủ. Không sửa source sealed, giữ
packet350/dc724 và Gate0 hai host đã PASS. YAML→PGM/hash/ảnh113×127/0,05m và
PLAN_ONLY_LIVE_FALSE đúng `map:=...fire_room_TN1.yaml`, bash syntax/block khớp PASS.
Evidence `.codex_qa_backups/r5_operational_candidate_20261026/evidence_r5_5_graph_fault_134120_134848/map_tn1_selection.json`.
Chưa runtime TN1; không hardware/SSH/NTP/flash/commit, không QA/build/sync lại.
Tiếp: phiên mới đặt Initial Pose theo TN1; [lệnh](docs/codex/current/R5_5_NAV_NAME_ACQUISITION_SYNC_AND_RUN_2026-10-27.md).
Model nhẹ đủ thao tác lệnh; model mạnh đọc raw runtime.

## R5.5-NAV-NAME-PI-SYNC-150501 — Gate0 static PASS, runtime chưa kiểm

<!-- CURRENT_R5_5_NAV_NAME_PI_SYNC_PASS_150501_20261027 -->
> **Gate0 hai host PASS packet350/dc724be6c39a.** Sync150501: Pi build10,4s,
> aarch64/errors[],101/101 installed source hashes khớp candidate; laptopstatic93
> cùng packet kế thừa. QA20+4 giữ nguyên; không sync/build/QA/R4/Gate1 lại.
> Runtime mới NOT_TESTED. Tiếp một capture automatic goal1/X→READY→goal2 kết
> thúc→READY→goal3; không đẩy xe lại, không A/F/T/I/P. USB/G3/JPEG/cleanup OPEN.
> [Receipt và bước tiếp](docs/codex/current/R5_5_NAV_NAME_PI_SYNC_PASS_150501_2026-10-27.md) ·
> [Lệnh hai terminal](docs/codex/current/R5_5_NAV_NAME_ACQUISITION_SYNC_AND_RUN_2026-10-27.md).
> NEXT_STEP=USER_ONE_DC724_AUTOMATIC_RECREATED_NAV_OWNER_CAPTURE_NO_RESYNC.

Các mục dưới là lịch sử trước receipt150501; trạng thái Pi pending đã được giải quyết.

## R5.5-134120-134848 — graphFAULT/USB after fault, candidate QA20

<!-- CURRENT_R5_5_134120_134848_NAV_NAME_QA20_20261027 -->
> **Runtime7d9921: X/goal2 đạt rồi graphFAULT ở raw publisher mới; chưa Gate2.**
> Lượt2 FAULT ngay goal1,2X bị chặn; USB/CDC/epoch còn lỗi sau graphFAULT,
> Q cả hai unconfirmed. Raw laptop46/lượt, Pi6/6 remoteSHA. Root endpoint cũ
> không được log; tái hiện UNKNOWN Nav2 tạo cùng lỗi2/2 trên source7d9921.
> Candidate350/dc724be6c39a sửa zero/name/owner-GID handshake và raw reason;
> QA20 mới+4 regression đúng delta/build0,97s/static93 PASS. **Pi sync mới PENDING,
> runtime NOT_TESTED.** Không ghép stage7d9921 với laptop mới, không flash.
> USB/G3/JPEG/Nav2 cleanup OPEN; giữ QA51/R4/Gate1, không chạy lại full QA.
> [Raw/fix/QA20](docs/codex/current/R5_5_134120_134848_NAV_NAME_QA20_READY_2026-10-27.md) ·
> [Sync/lệnh automatic](docs/codex/current/R5_5_NAV_NAME_ACQUISITION_SYNC_AND_RUN_2026-10-27.md).
> NEXT_STEP=USER_SYNC_DC724_NAV_NAME_SOURCE_ONLY_THEN_VERIFY_PI_RECEIPT.

Các mục dưới là lịch sử trước runtime134120/134848; Gate0 cũ PASS không đồng nghĩa Gate2 runtime PASS.

## R5.5-NAVIGATION-CANCEL-PI-SYNC-133434 — Gate0 static PASS

<!-- CURRENT_R5_5_NAVIGATION_CANCEL_PI_SYNC_PASS_133434_20261027 -->
> **Gate0 hai host PASS packet349/7d9921b81447.** User sync133434: Pi build8,98s,
> aarch64/errors[],101/101 installed source hashes khớp candidate; laptopstatic93
> cùng packet kế thừa. QA51 giữ nguyên, không build/sync/QA lại.
> Runtime mới NOT_TESTED. Tiếp một capture automatic pose/goal→X→READY→goal mới;
> không A/F/T/I/P. USB/G3/JPEG/Nav2 cleanup OPEN; chưa R5.6.
> [Receipt/bước tiếp](docs/codex/current/R5_5_NAVIGATION_CANCEL_PI_SYNC_PASS_133434_2026-10-27.md) ·
> [Lệnh hai terminal](docs/codex/current/R5_5_NAVIGATION_CANCEL_SYNC_AND_RUN_2026-10-27.md).
> NEXT_STEP=USER_ONE_7D9921_AUTOMATIC_NAV_X_NEWGOAL_CAPTURE_NO_RESYNC.

Các mục dưới là lịch sử trước receipt133434; trạng thái Pi pending đã được giải quyết.

## R5.5-NAVIGATION-CANCEL-20261027 — review QA51/build/seal, runtime pending

<!-- CURRENT_R5_5_NAVIGATION_CANCEL_QA51_SEALED_20261027 -->
> **Candidate source đã seal packet349/7d9921b81447; QA51 unique PASS.**
> Gồm11 checkpoint kế thừa+40 review mới; chỉ recheck đúng delta bị ảnh hưởng.
> Sửa X ngoài fire cycle/trong identity hold, exact cancel/replay/response,
> goal đến sớm và Nav2 terminal→no-goal fire. Status dùng pinned event-stream,
> mọi identity/STOP/OFF/owner gate vẫn giữ; epoch đổi vẫn FAULT.
> Laptop build/static93 PASS, archive350/hash/helper PASS. **Pi static mới PENDING,
> runtime NOT_TESTED.** Không dùng runbook/sync8144 hoặc draft7b64.
> USB/G3/JPEG/Nav2 cleanup OPEN; không hardware/SSH/NTP/flash/commit.
> [Review và QA51](docs/codex/current/R5_5_NAVIGATION_CANCEL_QA51_PACKET_READY_2026-10-27.md) ·
> [Sync/lệnh automatic](docs/codex/current/R5_5_NAVIGATION_CANCEL_SYNC_AND_RUN_2026-10-27.md).
> NEXT_STEP=USER_SYNC_7D9921_NAVIGATION_CANCEL_SOURCE_ONLY_THEN_REVIEW_PI_RECEIPT.

Các mục dưới là lịch sử; UNSEALED cũ đã được giải quyết bằng packet trên.

## R5.5-124637-125512 — USB epoch fault / Nav2 no-fire X recovery candidate

<!-- CURRENT_R5_5_NAVIGATION_124637_125512_UNSEALED_20261027 -->
> **Runtime8144 hai lượt chưa đạt; candidate Nav2/X đang UNSEALED, KHÔNG CHẠY.**
> 124637: kernel USB disconnect12:49:47/epoch2→3; hard lock giữ nguyên,
> Q unconfirmed.125512: goal1 SUCCEEDED; goal2 hold cùng epoch latchSTOP,
> 9X rejected ngoài fire cycle. Đã sửa offline3 module,11 QA delta unique PASS
> theo các XML; còn review status freshness/UUID, build/static/seal/helper mới.
> Pi raw6/6 remoteSHA; USB/JPEG/G3/Nav2 cleanup OPEN. Không sync8144/runbook
> cũ với source laptop đã đổi; không hardware/flash/commit hoặc lặp QA cũ.
> [Checkpoint](docs/codex/current/R5_5_NAVIGATION_124637_125512_QA11_QUOTA_CHECKPOINT_2026-10-27.md).
> NEXT_STEP=REVIEW_NAV_CANCEL_STATUS_BOUNDARIES_THEN_BUILD_SEAL_NEW_PACKET.

Các mục dưới là lịch sử trước runtime124637/125512; trạng thái sync/runtime pending8144 cũ đã được thay thế trên.

## R5.5-115816-IR-HOLD-RELEASE-20261027-13 — runtime FAIL, delta45 PASS

UserLOG112501 bịdánlại; ảnh12:01–12:03 khớp laptop115816/Pi115752/sessione61419e2/
packetdea3. Rawlaptop46byte-match/hash, Pi5remoteSHA; sync115440 static100/build9,15s
PASS/100archivehash đúng. IR tiến0,124m,6REALIGNrecovery, original15s hết rồi range
reached; chưa P. Identitytotalage350,780ms hold làm guardOPERATOR_ARM_LOST; OFFpub
khôngnhả vì phụthuộc cancelproof, retire chờpump0; usergoal sớm thêmactiveblock.
User xác nhận dập hẳn lửa. Q4,016875s STOP_UNCONFIRMED/WAIT_PUMP_RELEASE; RC2 còn
Nav2SIGTERM5s/child−15. Mission10/10,YOLO2/2,Pi8/8sạch; review−11 cũ chưa gọi fix.
SửaIR cậpnhật/xácnhậnfreshseq, boundglobalhold, OFFrelease mớiACT/home táchACK,
exact cancelgoal mớiđếntrongretire, UIproof/tuổiIR.45newuniqueQA PASS; build30,8s/
staticlaptop93/errors[]. Packet347/814458157455, archive15de129f95c0; Pi sync/new
runtime PENDING. Không QA43/45cũ/R4/Gate1 lại, không hardware/flash/commit.
USB/JPEG/G3/Nav2cleanupOPEN; baseline6d8/userdirty/firmware giữnguyên.
Chi tiết: docs/codex/current/R5_5_115816_IR_FRESHNESS_IDENTITY_RELEASE_QA45_2026-10-27.md

## R5.5-112501-IR-STOP-RECOVERY-20261027-12 — runtime chưa phun, candidate QA43

Automatic ARM/FIRE_WATCH/ALIGN/I đã nhận, IR MOVING rồi STOP có chủ đích bị hiểu
thành arm fault/ABORTED sauI1,56s, không có P/pumpON. Ảnh rightADC938/vùng phun
không thể thay thế IR STOP proof. Clear→NAV2_READY11:29:50,615, goal mới nhận/
SUCCEEDED11:30:13,238; Q STOP0,402s confirmed. Raw laptop44byte-match/intakehash,
Pi5/5remoteSHA đọc/copy qua LAN; source344/b6e8 khớp trước sửa. Pi8/8/YOLO2/2
cleanup sạch, RC2 do Nav2 fallback5s/child−15 và review−11 sauSIGINT, giữ OPEN.
Pi epoch2 trướcARM,28odom-stale startup, không epoch đổi lúc IR abort; USB/G3/JPEG
vẫnOPEN. Sửa6module candidate: STOP tag có binding, điều phối/callback order,
recoveryflag sau range proof và exact-cancel retire khi clear lúcALIGN.43unique
QA mới PASS (không lặp QA45/38+3/44/R4/Gate1), build0,78s/static92 PASS.
Packet345/dea3fa6a47d5, archive4f391cab3cd3, Pi sync/runtime PENDING.
Không tự chạyhardware/flash/commit. Baseline6d8/userdirty giữ nguyên.
Chi tiết: docs/codex/current/R5_5_112501_IR_STOP_RECOVERY_QA43_2026-10-27.md

## R5.5-AUTOMATIC-PI-SYNC-20261027-11 — Gate0 PASS, runtime pending

Sync105934 packet344/b6e8; Pi build10,6s/static100/errors[]/aarch64. Validator
PASS,100/100 installed hashes khớp candidate. Raw2file copy/hash; laptop92 và
QA45 kế thừa, không lặp. Tiếp automatic, không A/F/T/I/P; chỉ Initial Pose/READY/
goal, robot tự chữa cháy/hậu kiểm. USB/G3/JPEG/Nav2 cleanup OPEN. Không SSH/
NTP/hardware/flash/commit. Báo cáo: docs/codex/current/R5_5_AUTOMATIC_PI_SYNC_PASS_105934_2026-10-27.md

## R5.5-065731-AUTOMATIC-20261027-10 — runtime HOLD/cleanup FAIL, candidate QA45

Raw laptop43 file/hash, Pi6/6remotehash +kernel lịch sử. A/F/T/I accepted;
IR progress0,2179m rồi REALIGNING,6P rejected. Q STOP0,436s confirmed/operatorRC0;
actor drain0,840s/Pi8/8/mission10/10/YOLO2/2 sạch; Nav2 child−6 sauQ ->RC2 đúng.
Pi2reconnect trướcA, epoch3 trong điều khiển, STATEmaxage119ms; USB vẫnOPEN.
Source d450/340pins đúng trước sửa. Offline tái hiện spray-stop recenter gap,
sửa thêm async target-loss STOP. User chốt chỉgoal/noAFTIP, đã ghép automatic
policy trên core/gate cũ, opt-in CLI và private READY receipt, một pulse/cycle.
45 unique QA mới PASS, không lặp QA38+3/44/R4/Gate1. Build30,7s/static92 PASS.
Packet344/b6e8c108a9d5, Pi sync/runtimepending; chưa flash/hardware/commit.
Chi tiết: docs/codex/current/R5_5_065731_RECENTER_AND_AUTOMATIC_QA45_2026-10-27.md

## R5.5-PARITY-PI-SYNC-20261027-09 — Gate0 PASS, runtime pending

Sync065011 source340/d450; Pi build8,91s/static99/errors[]/aarch64. Validator
PASS và99/99 installed hashes khớp candidate. Laptop91/QA38+3 kế thừa, không lặp.
Raw2file copy/hash; evidence_r5_5_parity_pi_sync_065011. Runtime mới NOT_TESTED;
USB/G3/JPEG/Nav2 cleanup OPEN. Quota4% checkpoint, không SSH/hardware/flash/commit.
Chi tiết: docs/codex/current/R5_5_PARITY_PI_SYNC_PASS_065011_QUOTA_CHECKPOINT_2026-10-27.md

## R5.5-GATE2-042557-PARITY-20261027-08 — runtime FAIL, candidate QA38

042557/042521: A accepted125ms sau pending, Nav2/fire/cancel/WAIT_T; USB S3
kernel disconnect04:28:02, enumerate04:28:04 nhưng chưa STATE. Q STOP4,024s
unconfirmed; camera/YOLO còn mới. Không có T receipt hợp lệ trong raw cũ.
Thu45 laptop raw +6 Pi file/hash +kernel lịch sử qua LAN đã được user cho.
Review19 source port R4 khớp/29 overlay/77 firmwarepins; sửa9 module candidate:
freshness numeric boundary, stationary close policy, P admission/receipt,
exact cancel history khi loss IR, fault key feedback. QA38 mới+3 fixture sửa
PASS, build0,82s/static laptop91 PASS. Packet340/d450c50f986e; Pi/runtimepending.
Giữ USB/G3/JPEG/Nav2 cleanup OPEN, không flash/hardware/commit.
Báo cáo: docs/codex/current/R5_5_042557_FULL_PARITY_QA38_2026-10-27.md

## R5.5-IR-PREARM-SYNC-20261027-07 — PASS static, runtime pending

User sync042154 source339/packet65fd92, Pi build9,04s/static99/errors[]/aarch64.
Validator replay PASS và99/99 source hashes khớp candidate; laptop91 cùng packet.
Raw2 file copy/hash; không manifest runner gốc. Không lặp QA44/build/sync.
Runtime mới NOT_TESTED; tiếp một combined Gate2 capture. Quota7% checkpoint:
docs/codex/current/R5_5_IR_PREARM_PI_SYNC_PASS_042154_QUOTA_CHECKPOINT_2026-10-27.md

## R5.5-GATE2-IR-PREARM-20261027-06 — candidate only, no commit

035821 packetbbba6f: READY_GATE2/retry0 nhưng A bị chặn;35 proof, max sampled
stable0,552s,9ir_fresh blockers. Q STOP0,403s; Nav2 SIGTERM5s/−15 cleanup OPEN.
Raw43 files/hash; Pi chỉ paired receipt8/8, direct raw chưa thu.
Port R4 raw continuity600ms/grant350ms và bounded explicit A; QA27+17=44 PASS,
build retry0,77s/static laptop91 PASS. Packet339/65fd92aa5fa1…; Pi sync/runtime
PENDING. Không flash/robot/SSH/NTP/commit; giữ dirty user và baseline sạch.
Tiếp một sync rồi cùng capture ARM/nav/fire/P/clear/newgoal; kế thừa R4/Gate1.
Chi tiết: docs/codex/current/R5_5_GATE2_035821_IR_PREARM_PARITY_QA44_2026-10-27.md

# Nhật ký thay đổi và kiểm thử runtime

Mỗi bản code phải có mô tả thay đổi, bằng chứng kiểm thử và giới hạn sử dụng.
**Commit đã tạo hoặc push không đồng nghĩa robot đã chạy ổn.**

## Cách cập nhật sau mỗi lượt test

1. Ghi đúng commit CODE được chạy, máy chạy, cấu hình/điều kiện và ngày nhận kết quả.
   Nếu cây làm việc có sửa code, ghi thêm hash hoặc patch; không gán PASS cho HEAD sạch.
2. Ghi lệnh/tool, kết quả, nguồn bằng chứng (output người vận hành hay raw log đã kiểm),
   đường dẫn log và kiểm hash nếu có. Không cộng số test của các suite chạy chồng nhau.
3. Cập nhật riêng OFFLINE, BUILD, STATIC RUNTIME và MOTION/ACTUATOR.
   Dùng PASS, FAIL, BLOCKED, INCOMPLETE hoặc NOT_TESTED; luôn ghi phạm vi của PASS.
4. Giữ lịch sử lỗi cũ. Thêm kết quả mới thay vì xóa lần FAIL/BLOCKED hoặc đổi nhãn
   một lượt chưa đủ bằng chứng thành PASS. Liên kết commit sửa với lỗi đã quan sát.
5. Ghi vấn đề còn mở, bước tiếp theo và bản code đã nghiệm thu gần nhất cho đúng phạm vi.
   Bản chỉ đổi tài liệu phải trỏ về commit code được test, không tự nhận PASS runtime.
6. Trước commit runtime, body phải nêu thay đổi, kiểm thử, hạn chế và mã bản ghi này.
   Sau khi có SHA, cập nhật mục lục cục bộ `docs/codex/COMMIT_TEST_LEDGER.md` và checkpoint.
   File này được theo dõi trong Git; mục lục cục bộ nằm trong docs/ đang gitignore.

## R5.5-GATE2-ARM-SYNC-20261027-05 — Gate0 hai host PASS

- User sync034640: Pi336source/build8,90s/static98/errors[], packetbbba6f khớp
  laptopstatic90. So98/98 source trong receipt với candidate PASS; raw2filecopy/hash.
- QA26 kế thừa, không lặp QA/build/Gate1; runtime mới NOT_TESTED. Không hardware,
  SSH/NTP/flash/commit. Nav2 crash−6 sauQ và fallback/G3/JPEG/USB OPEN.
- Report: `docs/codex/current/R5_5_GATE2_ARM_PI_SYNC_PASS_034640_2026-10-27.md`.
- NEXT_STEP=R5_5_USER_ONE_GATE2_BBBA6F_ARM_ACCEPTED_NAV_THEN_SAME_CAPTURE_FIRE_CHAIN.

## R5.5-GATE2-ARM-20261027-04 — runtime032826 blocked ARM, callback fix QA26

- Packet92072d: READY_GATE2/retry0; ảnh A accepted=false/WAIT3s, không có armed
  phase. Nav2 nhận4goal/progressfail3; user Q, STOP0,408752s/actor0,890200s.
- Cleanup: mission10/YOLO2 sạch, paired Pi8/8 sạch; Nav2 SIGSEGV/Magick→exit−6
  sauQ làmRC2, lỗi riêng OPEN. Chưa raw Pi trực tiếp; laptop43file đãcopy/hash.
- Tái hiện offline lỗi mộtcallback rồiGUI/graph làm input mấtfreshness/hold3s;
  bản sửa xử lýcallback giữa các nhịp20Hz, thêm ARM blocker/countdown và key/proofraw.
  Giữ nguyên safety/core/config/firmware. Chưa chứng minh sole blocker runtime.
- 26QA delta/buildmission0,86s/laptopstatic90 PASS; packet336/bbba6f55d99e…
  Pi sync/runtime PENDING. Không hardware/SSH/NTP/flash/commit.
- Report: `docs/codex/current/R5_5_GATE2_032826_ARM_CALLBACK_QA26_CANDIDATE_2026-10-27.md`.
- NEXT_STEP=USER_SYNC_PACKET_BBBA6F_PI_STATIC_THEN_ONE_GATE2_ARM_NAV_CAPTURE.

## R5.5-GATE2-PREARM-RECEIPT-20261027-03 — Gate0 hai host PASS

- User collector032339 trả READY=1; không lỗi mới. Hash2/2, role Pi/aarch64,
  packet334/92072d, errors[] và source98/98 khớp candidate; laptop static90 kế thừa.
- Build Pi8,88s/static98 kế thừa031840. Helper copy nhầm receipt đã khôi phục;
  không lặp build/QA, không hardware/flash/commit. Runtime Gate2 NOT_TESTED.
- Report: `docs/codex/current/R5_5_GATE2_PREARM_PI_RECEIPT_PASS_032339_2026-10-27.md`.
- Tiếp một supervised Gate2 stage92072d/pairing mới; giữ Nav2 fallback/G3/JPEG/USB OPEN.

## R5.5-GATE2-PREARM-SYNC-20261027-02 — Pi build/static PASS, receipt helper fixed

- User-run log: `gate2_prearm_sync_runs/sync_20261027_031840_dI5mIg`.
- Pi source334, build bringup+mission8,88s, packet92072d và remote rehash98
  `PI_STATIC_HASH_PASS=1`.
- Final laptop assertion FAIL vì helper lấy receipt stage4ac53c66. Raw receipt
  xác nhận errors rỗng/98 file nhưng packet/path cũ; không làm mất kết quả
  build/static mới và không phản ánh robot runtime.
- Helper path đã sửa; collector receipt-only + validator qua syntax/compile,
  reject-old/accept-new-shape. Receipt đúng chưa copy; runtime NOT_TESTED.
- Không lặp build/static. Bản ghi chi tiết:
  `docs/codex/current/R5_5_GATE2_PREARM_SYNC_031840_RECEIPT_PATH_FIX_2026-10-27.md`.

## R5.5-GATE2-PREARM-20261027-01 — runtime030229 FAIL, candidate QA67

- Runtime nguồn: packet334 `4ac53c66cc07…`; laptop log
  `/home/huudinh/qa_logs/r5_gate2_laptop_20261027_030229`, Pi path
  `/home/pi/qa_logs/r5_gate2_20261027_030206`.
- Kết quả: `STARTUP_TIMEOUT:CONSUMERS`, chưa `R5_GATE2_READY`; không nhận A,
  không motion/pump. I/O/camera/pose/YOLO true; operator WAIT_ARM nhưng health
  chặn duy nhất `no_live` vì nhầm false heartbeat của enabled executor thành
  active live intent. Pi remote cleanup8/8 sạch; Nav2 laptop fallback SIGTERM5s
  vẫn làm cleanup audit degraded.
- Candidate mới tách published/active, giữ fail-closed và mọi safety/STOP gate.
  67 targeted tests PASS0,71s; mission build và laptop static90 PASS.
- Packet334 mới `92072d8801dd…`; Pi build/static về sau đạt ở bản ghi `-02`,
  runtime vẫn NOT_TESTED; không commit/flash.
  Sweep ngoài scope có cùng 56PASS/22FAIL trên packet trước và sau sửa, xác nhận
  fixture debt có sẵn; không dùng làm runtime verdict.
- Evidence/report:
  `docs/codex/current/R5_5_GATE2_030229_PREARM_FIX_QA67_2026-10-27.md`.


## Bảng trạng thái hiện tại — ghi nhận ngày 2026-10-20

| Bản code | Offline | Build Pi | Runtime Pi của M3 | Kết luận sử dụng |
| --- | --- | --- | --- | --- |
| `c1022014bd327105d06b10b6edc392145a0dbeb3` | PASS 1078 Python + 1 CTest trong Gate A | PASS Gate B | BLOCKED Gate C | Chưa nghiệm thu M3; baseline đang có trên Pi |
| `M3-CLOCK-20261020-01` (commit chứa bản ghi này) | PASS 530 test | PASS Gate B, raw log verified | NOT_TESTED | Chờ runtime V3 |

Bảng chỉ tổng hợp các mốc M3 liên quan, không đánh giá lại mọi commit lịch sử.
Chưa có bản M3 nào được xác nhận ổn toàn bộ từ các phép thử được ghi ở đây.

## M3-CLOCK-20261020-01 — lấy mẫu giờ cho stop evidence

- Commit cha: `c1022014bd327105d06b10b6edc392145a0dbeb3`.
- Commit code: commit thêm mục này, nhận diện bằng trailer
  `Validation-Record: M3-CLOCK-20261020-01` trong body Git.
- Ngày nhận kết quả QA: 2026-10-20.
- Trạng thái: **OFFLINE_PASS / PI_BUILD_NOT_TESTED / PI_RUNTIME_NOT_TESTED**.

### Đã thay đổi gì và vì sao

Supervisor cũ đọc giờ ROS rồi monotonic riêng rẽ. Khi bị tạm ngừng giữa hai lần
đọc, cặp giờ có thể lệch quá 20 ms dù hai đồng hồ thực tế vẫn nhất quán. Có test
fake-clock tái hiện đường lỗi phần mềm này; chưa chứng minh đây chính là nguyên
nhân của lần lỗi Pi đã ghi.

Thêm cách đọc monotonic → ROS → monotonic. Nhận mẫu khi khoảng đọc không quá
1 ms; thử lại toàn bộ cặp tối đa 3 lần. Giá trị invalid hoặc đi lùi chặn ngay,
kể cả so với lần đọc trước và giữa các lần retry. Nếu hết lượt vẫn không có mẫu
đủ tin cậy, giữ lỗi `GLOBAL_CLOCK_SAMPLE_UNCERTAIN` và xóa zero hold.

Dùng cặp giờ ở callback stop, admission, kiểm graph, close barrier và kiểm tra
sau publish. Header trạng thái vẫn lấy trước công việc graph/evaluate. Bổ sung
counters, mẫu cuối và cặp clock đầu tiên gây fault vào shadow state hiện có.
Không có node/timer/topic mới; không nội suy ROS hoặc thay timestamp nguồn.

Giữ freshness 350 ms, skew/clock guard 20 ms hoặc cấu hình chặt hơn, watchdog,
STOP, source identity, quota/ledger và yêu cầu operator recovery. Chưa đo CPU.

### Bằng chứng offline

Người vận hành chạy trên laptop:

```bash
bash '/media/huudinh/New Volume/esp32/NCHK_robotFirefighting_ROS/.codex_qa_backups/qa_m3_stop_clock_candidate_20261020/verify_offline.sh'
```

Kết quả: **530 passed in 1.74s**, marker `M3_STOP_CLOCK_OFFLINE_QA_OK=1`.
11 file test bao gồm tái hiện lỗi cũ, retry, clock jump thật, rollback, invalid,
hết retry, dữ liệu DDS 400 ms cũ, callback, close proof và replay nhiều phiên.
15/15 hash source/test/script khớp manifest đã được chấp nhận.
Đây là output user cung cấp; assistant kiểm hash/diff, không tự chạy test/build.
Không thay code/test sau lượt PASS này.

Hồ sơ cục bộ: `.codex_qa_backups/qa_m3_stop_clock_candidate_20261020/`:
`user_gate_result.json`, `accepted_offline_manifest/`, `FINAL_REVIEW.md`,
`before/`, `before.sha256`, `candidate.patch`, `commit_sources.sha256`.
Các hồ sơ/log này đang gitignore, không có trong bản clone GitHub.

Hash bốn file code/test được QA (file Markdown này không đổi hành vi runtime):

```text
1352c210d2ddc229f7dbf29060fac0ceb25e55666f520b81953a4ad99352791b  ros2_ws/src/fire_robot_mission/fire_robot_mission/stop_clock_sampling.py
400b4bb3261135776c5d6832a6423e15ebfd733448c01c630d45e37becf418a0  ros2_ws/src/fire_robot_mission/fire_robot_mission/mission_supervisor.py
7468fb3a5a9bf06a54e12ce7d235abe26abf39e0d40395a8ad12c53cb7a51e68  ros2_ws/src/fire_robot_mission/fire_robot_mission/global_stop_evidence.py
fe6e1fbe550ba70734dff74077b026430e5a387514035dea3a13d401a9372513  ros2_ws/src/fire_robot_mission/test/test_stop_clock_sampling.py
```

### Trạng thái tại thời điểm tạo commit (xem kết quả mới phía dưới)

- Chưa build/import bản này trên Pi, chưa chạy Gate C bằng bản này.
- Chưa xác định dứt điểm clock fault của Pi có phải do cặp đọc bị lệch hay không.
- RViz trên laptop vẫn có sự kiện SIGSEGV (`exit -11`); chưa có bản sửa.
- Ảnh đầu vào YOLO từng có khoảng trống; chưa kết luận tải Pi hay WiFi/camera.
- Readiness V2 giữ 64 thay đổi đầu, chưa lưu trạng thái cuối rõ khi bão hòa;
  cần sửa chẩn đoán trước lần nghiệm thu tiếp theo.
- Chưa nghiệm thu motor/bơm hoặc đổi quyết định Nav2 chạy ở máy nào.

### Bước tiếp theo

Chốt commit, xác minh blob code khớp bản QA. Sau user duyệt push mới chuẩn bị
backup/checkout/build Pi. Chuẩn bị đủ final-readiness/clock diagnostics và rà RViz
trước phép đo runtime có giới hạn. Giữ ledger hiện có; không tự chạy lại Gate C.

## c1022014bd327105d06b10b6edc392145a0dbeb3 — baseline M3

Thêm bounded multi-session shadow runtime. Gate A user PASS 1078 Python tests
và 1 native CTest; Gate B Pi build/import/install đạt với 96 source hash khớp.
Log Gate B: `/home/pi/qa_logs/qa_m3_gate_b_20261020_141720`.

Gate C gần nhất: `/home/pi/qa_logs/qa_m3_runtime_v2_20261020_153210`.
13/13 file log khớp hash; ledger không đổi và cleanup sạch. Cả 8 lifecycle Nav2
cuối lượt active nhưng mission giữ `GLOBAL_CLOCK_DISCONTINUITY`; readiness
BLOCKED, chưa bắt đầu measurement. Vì thế **M3_RUNTIME_BLOCKED**, không gọi bản
baseline này là robot đã chạy ổn. Camera/RViz cần theo dõi riêng.

Bản sửa M3-CLOCK-20261020-01 xử lý nguy cơ lấy mẫu giờ; kết quả runtime còn chờ.


## Kết quả mới — Gate B Pi, 2026-10-20

- Commit CODE được script chọn: `690673a456cc596703222dccc990a2cadf27804b` (M3-CLOCK-20261020-01).
- Người vận hành chạy deploy_pi.sh trên Pi, gửi marker
  `M3_STOP_CLOCK_GATE_B_OK=1`, kết thúc 16:25:43+07:00.
- **BUILD/IMPORT/INSTALL: PASS theo output user; chờ đọc raw log và xác minh hash.**
  Script chỉ ghi marker sau backup, 97 source hash, build mission, import/install,
  kiểm native navigator không đổi, ledger không đổi và cleanup. Chưa kiểm độc lập
  các giá trị receipt này vì bản sao raw log chưa có trên laptop.
- Log Pi: `/home/pi/qa_logs/qa_m3_stop_clock_gate_b_20261020_162520`.
- Backup: `/home/pi/qa_backups/pi_wireless_before_m3_stop_clock_20261020_162520`.
- Offline vẫn PASS 530 test; **runtime bản code này: NOT_TESTED**.
- Bước tiếp: copy log để review; chuẩn bị chẩn đoán clock/final readiness và
  xử lý phụ thuộc RViz trước phép đo có giới hạn. Chưa suy clock lỗi đã hết.

Yêu cầu vận hành bổ sung: ưu tiên tiếp tục test Nav2 trên Pi. Giữ khả năng
chuyển nhanh về laptop bằng profile launch dùng chung code/config khi Pi không
đạt tải hoặc độ trễ. Đây là chuyển giữa các phiên đã dừng, chưa có selector
được triển khai/nghiệm thu trong code hiện tại; không chuyển nóng khi có goal.

Ghi chú số đo: M1 Pi Nav2 active đã đạt cửa sổ tĩnh20s với odom gap tối đa154.40ms,
header age tối đa107.09ms. Phép đo WiFi tĩnh riêng có gap odom Pi264.56ms và
laptop669.15ms; lúc đó Nav2 chưa full active. Hai kết quả không phải đối chứng
A/B Nav2Pi/laptop cùng tải mission/YOLO, chưa chứng minh placement tốt hơn tổng thể.


## Gate B đã kiểm raw log — 2026-10-20, code690673a

Commit code: `690673a456cc596703222dccc990a2cadf27804b`.
Đã đọc bản sao qa_m3_stop_clock_gate_b_20261020_162520 trên laptop: 8/8 file
khớp hashes.sha256, 97/97 runtime source khớp đúng bản source; import path
bao gồm stop_clock_sampling ở đúng Piworkspace, packageprefix đúng. Build
fire_robot_mission thành công6.92s toàn bướccolcon; native navigator trước/sau
cùnghash. Receipt ghi backup/ledgerkhôngđổi; processes_after rỗng.
**PI_BUILD_IMPORT_INSTALL=PASS_RAW_LOG_VERIFIED; PI_RUNTIME=NOT_TESTED.**

Bước kế: QA offline toolsV3 giữ finalblockers/clockfault và RVizposeconfig nhẹ.
Cấu hìnhRViz là workaround chẩn đoán chưa testGUI, không tuyên bố chữaSIGSEGV.
Không thay source690673a sau530PASS, không thêm runtime-node/timer nền. M3 OPEN.


## QA tools V3 — 2026-10-20

Code sẽ đo: `690673a456cc596703222dccc990a2cadf27804b`.
User chạy verify_tools.sh của qa_m3_gate_c_v3_tools_20261020:25passed/0.20s,
markerM3_GATE_C_V3_OFFLINE_QA_OK=1;8/8toolhashkhớp,acceptedcopyđãlưu.
PhạmviPASSlàreadiness/observerdiagnostics/mock/YAMLposeconfig,khôngGUIhayhardware.
Toolgiữfirst+latest/dropcount/finalblockers/clockfirstfault;acceptancekhôngnới.
Khôngcộng25testtoolvào530testcode. CodePiGateBđạt;runtimeM3vẫnNOT_TESTED.
BướctiếpuserchạyV3quaWiFivớiInitialPosethật/ledgercũ,haiSHOW/HIDE,
readiness<=180s/measure300s. RViztốigiảnchưađược xácminhhếtcrash.


## V3 runtime16:44 — bằng chứng laptop mới, chờ log Pi

Tool kỳ vọngcode690673a. User cung cấpLOG_PI qa_m3_runtime_v3_20261020_164455,
rawPi chưa được chép nên chưa xác minh readiness/clock/session hoặcHEADlượt này.
LAPTOP: RVizconfigkhớpQA,exitRC0 vàSIGINT/SIGTERM,không thấycrash-11 tronglượt.
Viewer4/4hashkhớp,1546JPEG,0decode/overlayerror. YOLOinferencep5016.55ms;
reportedsourceagep9549.73ms/max724.67ms,2camera_stale. Không làA/B hoặcbằngchứng
chữa tận gốc. Không có fireoutput; chưa biếtmeasurementđãbắt đầu hayvẫngiữEMPTY.
**M3_RUNTIME_RESULT_PENDING_PI_LOGS**. Không gọi codeổn hoặcFAILtoànbộ từcảmgiácUI.
CầncopyPi đểxácđịnhfinal_blockers/clockfirstfault/proof/ledger, khônglặp bài ngay.


## V3 Pi16:44–16:48 đã kiểm — code690673a

14/14loghashvà97/97sourcehashkhớp;ledgerkhôngđổi,cleanup sạch. **M3_RUNTIME_BLOCKED**:
readinesskhôngđạt,chưađo2phiên. Clock10922calls/82retries/0uncertain/first_faultnull,
khôngthấyclockfaultcũ tronglượt; targetfixcótínhiệutốt,chưanóiổntoànrobot.
BugQA:3get_stateRPCresponsemất,toolpendingkhôngdeadlinevàkhôngretry;
Nav2manageractive16:45:58.575nhưng3statesmissingcuối. CầnfixQA,noresponsecasechưa
được25mocktestcover. KhônggọibaNav2nodechếtchỉtừkếtquảprobe.
Odomvẫn16stale/5reject,maxwarningage564ms. 16:47:52.418–55.596TFpause3.178s,
serialnativeportmất/reopenepoch1->7,firmwareUSBrecovery0->1. Missiongiữ
GLOBAL_ACTUATOR_CONTEXT_CHANGEDsauđổiepoch;freshproof/recoverychưanghiệmthu.
RVizRC0,YOLOảnhđỡtrễlượt này;chưafulllivemission. BướctiếpfixQAdeadline,
đốichiếuUSBkernelđúngmốc vàworkflowrecovery,khônglặpbàimù/khôngnớiguard.
Evidence:qa_m3_v3_review_20261020_164455/REPORT.md,pi_review_result.json.

## USB-102 — loaded Wi-Fi232001, scan guard chuẩn bị

Nav2 runtime shutdown PASS exit0/no forced cleanup; firmware USB rare stall OPEN.
Lượt đo chính vẫn trễ: laptop odom944ms/JPEG2141ms/12,47FPS; YOLOmean16,44ms.
User xác nhận mở Settings→Wi-Fi; chu kỳ ping≈15s phù hợp timerGNOME, chưa có
historicalLastScan để chứng minh nhân quả. Đã code laptop preflight/recordLastScan,
16 tests+real read-only snapshot PASS; robot/Pi/fw không đổi, runtime cải thiện NOT_RUN.
Số liệu đầy đủ trong PERFORMANCE_COMPARISON_LEDGER.json/USB-102 và [review](docs/codex/current/USB_WIFI_LOADED_REVIEW_SCAN_GUARD_2026-10-22_232001.md).
NEXT_STEP=ONE_STATIC_WIFI_CAPTURE_SETTINGS_CLOSED_WITH_PASSIVE_SCAN_EVIDENCE

## USB-103 — run234730 reviewed, mission shutdown patch applied

Main120s: laptop JPEG14,95FPS/maxgap172,629ms, odom205,184ms, scan265,150ms;
ping mean6,186ms/max56,6ms; LastScan changes0. Nhận JPEG480674B/s so384940B/s
ở232001. User ít đi qua cảm biến hơn; điều kiện cảnh không giống hoàn toàn.
Nav2/YOLO/RViz exit0; mission exit1 do context-invalid WaitSet khi SIGINT.
Patch chỉ nhánh thoát đó đã áp baseline a7cbda4:58 unit/regression PASS (gồm9mới),
6 ca ROS thật localhost PASS. Không đổi hành vi/QoS/safety/Pi/firmware; chưa commit.
Source symlink có hiệu lực, chưa full hardware saupatch; runner cũ sẽ chặn hash.
JPEG warnings còn mở, cần ảnh gốc ghép cặp; USB rare stall chưa tái hiện, vẫnOPEN.
Xem [review](docs/codex/current/USB_WIFI_SCAN_RESULT_MISSION_FIX_2026-10-22_234730.md) và PERFORMANCE_COMPARISON_LEDGER.json/USB-103.
NEXT_STEP=REVIEW_MISSION_SHUTDOWN_PATCH_THEN_BOUNDED_MATCHED_JPEG_SCENE_CAPTURE
<!-- CURRENT_R5_5_HANDOFF_IR_DIAGNOSTIC_20261027 -->
> **R5.5 packet353/9f67346eccb7 Gate0 hai host PASS; runtime PENDING.**
> Từ raw164835: IR_CONFIRM20s không P/ON, lần2 mất target trước ALIGN vì
> timer tính khi Nav2 còn hủy goal. Sửa timer bắt đầu sau exact handoff; thêm
> log/GUI stationary reason/seq/age/blocker≤1Hz. QA40 unique selected PASS,
> laptopstatic93/Pistatic101 hash PASS sync171901. Không đổi IR threshold/safety,
> không hardware/flash/commit. [Báo cáo](docs/codex/current/R5_5_HANDOFF_IR_DIAGNOSTIC_PI_PASS_2026-10-27.md).
> NEXT_STEP=USER_ONE_9F6734_AUTOMATIC_DELTA_CAPTURE_STATIONARY_PROOF_AND_PUMP_ADMISSION.

Các mục dưới là lịch sử trước packet9f6734; runtime phun R5 chưa PASS.


## App source checkpoint trước chỉnh đường đi — 2026-10-30

Validation-Record: APP_SOURCE_CHECKPOINT_20261030_6133

Lưu source giao diện RViz/camera, quản lý quét/lưu/chọn map và phiên chữa cháy,
phục hồi STOP/cleanup, X/Goal và quy tắc Q dừng/Esc bỏ công cụ. Packet app
6133ff387973f5c7ccc6b245b6acaa1c6d74126ea3f2f5b0eb4f75a376f60426 dùng inflation
local/global 0,30 m, model best_minireplay.pt; giữ nguyên 364 file parent R5,
footprint 34 × 20 cm, padding 1,5 cm, watchdog và mapping runner.

Bằng chứng có trước publication: sửa X/Goal 798 PASS + 2 SKIP; bản inflation
0,40 m 184 PASS + 2 SKIP; delta 0,30 m 21 PASS và native CTest PASS. Không cộng
các suite này thành một tổng runtime. Static laptop/Pi 95/103 file đúng hash;
449 source pins khớp. Thông tin packet/hash, giới hạn và cách khôi phục nằm ở
apps/fire_robot_control/checkpoint.json và docs/SOURCE_CHECKPOINT_2026-10-30.md
trong thư mục app. Raw log và artifact giữ cục bộ, ngoài Git.

Đường Nav2 đổi bên/vòng cua rộng vẫn OPEN, chưa đổi BT hoặc lookahead. Đây là
SOURCE/BUILD/STATIC checkpoint, không nghiệm thu toàn bộ robot. Lượt publication
chỉ kiểm source, tái dựng packet và kiểm tra Git; không chạy ROS/xe/bơm, SSH Pi,
NTP hoặc flash. Không stage mục observer R5 đã có từ công việc khác.

Kiểm tra publication: xuất source từ Git index, dựng lại đúng packet6133 với
449/449 pins byte-exact; 20 ca source/launch/runner/model chạy lại trên checkout
xuất từ Git PASS, 154 Python AST và 7 bash syntax PASS, scan credential không
có finding. Các kiểm tra này không phát sinh nghiệm thu phần cứng mới.


## Map TN1/TN2 dùng cùng app source checkpoint — 2026-10-30

Validation-Record: MAP_TN1_TN2_CHECKPOINT_20261030

Lưu hai cặp YAML/PGM TN1_30_10_2026 và TN2_30_10_2026 đã có trên laptop,
để mốc khôi phục app có dữ liệu bản đồ đi kèm. Không đổi pixel, origin hoặc
tham số map; không sửa launch/Nav2/SLAM/R5. Map tạm temp.* giữ ngoài commit.
Kiểm tra offline: hai YAML trỏ đúng PGM cùng thư mục, đọc được toàn ảnh xám;
TN1 118 × 132 ô, TN2 115 × 115 ô, resolution 0,05 m, ngưỡng free/occupied
hợp lệ và SHA256 ảnh/YAML được đối chiếu trước stage. Chưa nghiệm thu độ chính
xác bản đồ hoặc chạy robot trong lượt publication.

## App global inflation 0.25 m — 2026-10-31

Validation-Record: APP_GLOBAL_INFLATION_025_20261031

Giảm global inflation từ 0,30 m xuống 0,25 m cho app; local vẫn 0,30 m.
Sửa kiểm tra cost_scaling_dist chỉ áp dụng cho local costmap. Giữ footprint,
padding, collision detection và R5. QA 24 ca source/launch/model và smoke IDLE
PASS; Qt CTest, static laptop 95 file và Pi 103 file PASS theo báo cáo app.
Chưa nghiệm thu khoảng hở robot thực; đổi bên và vòng cua rộng còn OPEN.
