# Sửa hộp xác thực bị mất khi mở Pi — 30/10/2026

**Đã sửa đường worker → supervisor → Qt, QA và đồng bộ bản cuối.**
App mới `857b9eb3cea65145a60e49f4eda38eeac5909d93f2bb64cb8d234b3d762409cf` đã được chọn và mở ở BLOCKED,
chưa tự chạy xác minh STOP/I/O hoặc nhiệm vụ. User cần bấm Xác minh dừng
một lần nữa và nhập mật khẩu Pi trong hộp thoại khi được hỏi.

## Lần user vừa chạy và nguyên nhân

Ảnh user cho thấy STARTING_PI. Log đúng app a387: đã qua CHECKING_OLD_SESSION
và STARTING_PI rồi BLOCKED. Desktop receipt phiên 044348f8… trong
session_e8d09bfc0b7f4ef6a23045aa29ca10d4 ghi PI_RUNNER_EXITED,
stop=false/cleanup=false; không có laptop receipt vì chưa chạy laptop runner.
Pi log `/home/pi/qa_logs/r5_automatic_20261030_045135/error.json` ghi
`RuntimeError:PI_NTP_REFRESH_FAILED`. Từ pairing tới error đúng 100,027 giây,
khớp deadline 100 s của helper. Không có generation, state hoặc DONE; chưa I/O.

Kiểm chỉ đọc: hostname ubuntu, eth0 down, peer WiFi 10.0.0.253, chrony active,
nguồn WiFi cấu hình đúng, guard không có robot process. sudo -n -v không có
cache ở phiên chẩn đoán; trạng thái này riêng lẻ không chứng minh mọi TTY
không có cache. User xác nhận không thấy hộp mật khẩu. Pi/laptop vẫn có nguồn
chrony nhưng sample đã cũ; chưa bỏ freshness gate hoặc kết luận NTP đã đạt.

Source xác nhận worker emit AUTH_REQUIRED khi đọc prompt, nhưng tick của
SessionController chỉ xử lý SESSION/SAVED/FINISHED và bỏ AUTH_REQUIRED/DONE.
Qt có handler modal nhưng không nhận sự kiện. Test tái hiện **FAIL trước sửa**
với row prompt hợp lệ đúng session rồi tick trả []; sau sửa PASS. Phối hợp
lỗi relay này, không thấy hộp và timeout 100 s giải thích việc chờ sudo hết hạn.
Raw PTY không được ghi lại nên không khẳng định đã có trace prompt thực tế
của riêng lần chạy cũ. Không quy lỗi cho ESP32 hoặc runner R5.

## Thay đổi

- `services/live_session.py`: gắn session ID vào AUTH_REQUIRED và AUTH_DONE.
- `services/supervisor.py`: relay hai sự kiện này chỉ khi đúng session đang
  STARTING/MAPPING/RUNNING, chưa CLOSE/STOP. Nhãn cố định, không chuyển raw
  SSH/extra fields. Child đã kết thúc thì bỏ prompt còn chờ trong cùng batch.
- Dữ liệu trả lời vẫn đi IPC đã ràng buộc UI lease/session, rồi worker ghi vào
  đúng PTY đang chờ. Không lưu password, không hardcode hoặc bỏ qua sudo.
- R5/clock helper, ngưỡng 50 ms/deadline 100 s, watchdog, startup/STOP và firmware
  không đổi. Không NTP mutation hoặc ROS launch do agent trong lượt sửa.

## QA cuối

| Bộ | Kết quả và phạm vi |
| --- | --- |
| Unittest | 211 PASS, gồm 6 method auth mới và 205 ca cũ; 6,292 s |
| Tích hợp cuối | 160 PASS = 158 ca hồi quy + 2 ca modal Qt accept/cancel |
| Native CTest | 1/1 PASS, cả build và frontend regression |
| Tái hiện relay | 1 FAIL trước sửa → PASS sau sửa |
| Qt thực | Worker prompt/pump thật, supervisor, AppClient, SessionUi và QInputDialog thật; child PTY thay bằng fixture, không SSH/sudo |
| Static hai host | 426 source/103 installed Pi và 95 installed laptop khớp |
| Xác minh robot sau fix |NOT_RUN; chưa nhập mật khẩu thật hoặc mở I/O bằng bản mới |

6 method gồm prompt→UI→answer→worker, prompt từ session sai/mất/đã kết thúc,
DONE/label sanitize, sudo prompt tách chunk qua worker/controller, CLOSE ưu
tiên và child kết thúc cùng batch retract prompt. Hộp Qt che ký tự; accept
trả fixture đúng, cancel gửi STOP không gửi password; log không có fixture.
Các test helper chứa chuỗi fixture là source thử, không phải password user.

Có một gói trung gian 7a482f01 đã build/sync trước khi thêm ca child kết thúc
cùng batch. Gói đó **không chọn current hoặc mở cho user**. Gói cuối 857b9eb3
thêm guard tương ứng và chạy lại 211 unittest/160 integration/CTest. Lượt 210 unittest/
160 integration trước đó là trung gian, không cộng dồn thành testcase mới.
Cảnh báo xacro/Python 3.11 hiện có không làm test fail. Không gọi mọi hành vi
app hoặc mọi trường hợp phần cứng PASS từ các fixture này.

## Đồng bộ và bảo toàn

Stage cuối: `/home/pi/fire_robot_desktop/857b9eb3cea6/ros`. Build 2 package thành công.
Receipt Pi: `9b574b5ea1dc133191d261247133e04f570acd28621d097aaa395400e9f3e42f`; bản gốc Pi/tải/lưu app cùng hash.
22736 file của 48 bộ cũ/baseline trên Pi
nguyên hash/resolved target, baseline Git sạch, không robot process trong lần
kiểm. R5 CLI 0c54/2484/dfac và các app cũ không bị sửa. Parent R5 có 364 file
nguyên trong manifest 426 của gói app cuối.

2385 file laptop nguyên trước activation. Sau activation chỉ
current.txt đổi từ a387→857b9eb3; pending thật giữ SHA a13a0874….
Binary SHA: `b740431d3de92c6697fba7c32ea73c1f27937dee9cc7c53b6aa045987a081994`.
Archive SHA: `2825434badbeadceb93132fc682c561b8512f93a550a94dca4425febe9b1a410`.
Build receipt artifact giữ Pi PENDING/runtime false tại thời điểm build;
receipt cài Pi thực tế lưu riêng đúng packet, không sửa gói niêm phong.

Khi activation, không còn supervisor cũ; lock supervisor và common lock đều
rảnh. Đổi selector dưới lock, không cắt hoặc khởi động phiên robot.
App mới PID 301134, log đúng từ launcher mới:
`/tmp/fire-ui-w162xwy_/gui.log`. Đã kiểm PID/candidate, phase BLOCKED và chưa action.
Không lựa log theo mtime để tránh nhầm. Giữ pending cũ chưa chứng minh STOP.

## Thử tiếp và giới hạn

Giữ WiFi Pi; không chạy terminal R5 cùng app. Khi robot đã dừng và user ở cạnh,
bấm **Xác minh dừng trên robot** một lần. Nếu hiện **Xác thực Pi**, nhập
mật khẩu tài khoản pi vào app ngay khi được hỏi, không gửi mật khẩu qua chat.
Thao tác xác minh có thể mở I/O và STOP/OFF/home; servo có thể về home.
Chờ kết quả, chỉ khi IDLE và quan sát robot dừng mới thử quét.

Nếu lại thất bại, giữ log/stage. Fix này đảm bảo đường chuyển xác thực đã
được QA; không bảo đảm clock fresh/TF/USB hoặc STOP telemetry sẽ đạt trên
phần cứng. Không xóa pending, không nới đồng bộ giờ để vượt lỗi.

**Finding ngoài phạm vi còn mở:** tick hiện cũng chưa relay sự kiện OPERATOR
của live worker tới Qt. Cần kiểm/sửa riêng trước nghiệm thu nút X qua operator
trong chữa cháy; test core/operator trước chủ yếu nối trực tiếp, chưa chứng
minh nhánh live supervisor này. Không dùng ca AUTH thành chứng cứ X/mission
đã đạt. Lượt này giữ thay đổi hẹp ở relay AUTH, chưa chạy nhiệm vụ chữa cháy.

[Evidence](qa_evidence/auth_relay_fix_20261030/) lưu runtime receipt/mã lỗi đã
lọc, baseline FAIL, delta, unit/pytest/Qt logs/XML, diff/sourcepins, manifest,
Pi hash nén trước/sau và activation. SHA256SUMS pin từng file. Lệnh QA ở
run_release_qa_final.sh cần ROS overlays, socket/OpenGL localhost211 và probe
Qt build từ tests/pre_sync_ui_review. Không chạy script activation như QA.
