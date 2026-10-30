# Phục hồi Goal khi dữ liệu trở lại — bản sửa 31/10/2026

Gói app: `95e9facbc7d283373a185c0788185bee0f8e12d33c0feb285b7fe620faf80dd0`.
Baseline lỗi: `62c78ddf`, phiên `session_a60110eb8c894cccbe042bf4a326ac2b`.
Bằng chứng QA ban đầu được giữ trong [báo cáo 02:31](QA_DATA_RETURN_0231_2026-10-31.md).
Kết quả kiểm cuối và triển khai nằm trong `qa_evidence/data_return_fix_20261031/`.
Đây là kiểm thử source, bus mô phỏng và ROS/Qt localhost; chưa nghiệm thu xe/bơm thật.

## Các lỗi đã sửa

1. **Dữ liệu odom/mission/ARM trở lại nhưng cùng Goal vẫn giữ dừng.**
   `app_nav_recovery.py` nay phân biệt gap dữ liệu với lỗi hợp đồng, nguồn,
   quyền điều khiển và UUID. Chỉ giữ ý định của cùng operator/cancel epoch/
   identity/runtime generation/pins/Goal đã từng chạy. ARM false do operator
   đang giữ input không cấp quyền chuyển động; phải có operator hiện hành,
   đầy đủ preflight R5, lệnh Nav2 mới sau STOP, raw owner đã nhả, odom đứng
   yên, cmd zero và actuator OFF mới, liên tục một giây. Sau đó vẫn phải qua
   khám phá publisher/GID và xác nhận owner như lần đầu. Không gửi lại Goal,
   không dùng lệnh vận tốc cache, không reset Nav2.
2. **Recovery mission làm mất các subclass sửa lỗi của app.**
   `candidate_build.patch_runner` bọc cả command factory phục hồi qua
   `desktop.commands`, như startup ban đầu. Launch mới vẫn có tên child đúng
   để audit cleanup và vẫn dùng wrapper gateway/cancel guard. Giữ các binding
   automatic/recovery/after-pulse; chỉ mission được dựng lại, Nav2/operator
   được giữ. QA runner nay thực sự gọi factory trước/sau pulse thay vì chỉ
   dựng fake recovery không sử dụng factory.
3. **Query ROS chậm làm thu hồi vĩnh viễn ý định phục hồi.**
   Test bổ sung đã FAIL trước sửa (`query_baseline.xml`). Tách freshness của
   mẫu kiểm khỏi identity của ý định: query trễ bắt buộc STOP và xóa khoảng
   proof liên tục; khi query/dữ liệu mới trở lại thì xây proof lại. X/Q,
   đổi identity/pins/Goal hay nguồn mâu thuẫn vẫn thu hồi ý định. Hai ca
   query cuối kiểm cả không phát vận tốc lúc trễ lẫn phục hồi sau đó.

## Pose AMCL: cần bằng chứng mới, không bỏ guard

Observer app ghi frame, tuổi stamp thực dùng bởi callback gốc, số hữu hạn,
chuẩn quaternion và kết quả policy để lần sau xác định đúng điều kiện con.
Khi chỉ pose chưa hợp lệ, đã có Initial Pose, cùng nguồn AMCL, map TF/action/
lifecycle tốt và I/O đứng yên/OFF ổn định một giây, observer yêu cầu
`/request_nomotion_update` của AMCL. Tối đa một request đang chờ, timeout hai
giây và retry cách năm giây. Không yêu cầu pose mới khi xe còn chạy, I/O xấu
hoặc chưa có Initial Pose. Pi không dùng helper này.

Trong [source AMCL Humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/amcl_node.cpp),
dịch vụ chỉ đặt `force_update_` để cập nhật với laser tiếp theo. App không
đổi Initial Pose, reset bộ lọc/Nav2, sửa stamp hay tự gán pose valid.
Phản hồi dịch vụ không phải pose proof: callback AMCL mới vẫn phải vượt
chính kiểm tra frame/tuổi/số hữu hạn/quaternion của R5, rồi đủ thời gian ổn
định gốc. Test dùng HealthCore và callback R5 thật xác nhận ACK dịch vụ đơn
thuần không mở readiness; chỉ message pose mới hợp lệ mới mở được.

Raw phiên 02:31 thiếu message AMCL nên chưa kết luận trường con nào sai
trên robot. Nếu AMCL không cung cấp pose mới hợp lệ, xe vẫn phải giữ dừng;
helper này không biến lỗi định vị thật thành điều kiện được phép chạy.

## Hành vi bơm như R5

Config `operational_gate2.yaml` nguyên byte: tối đa ba lượt **liên tiếp**,
mỗi lượt sáu giây từ ACK ON; OFF/HOME giữa các lượt, quota ba lượt/18 giây
cho chuỗi đó. Hết mục tiêu hoặc mất điều kiện hiện hành có thể ngừng sớm.
Sau di chuyển/căn lại, R5 có thể reset quota liên tiếp; không hiểu thành
chỉ được phun ba lần trong toàn bộ phiên hoặc toàn bộ sự kiện lửa.

Hai ca bus mô phỏng dùng operator app và node mission thật, profile R5 thực,
có/không Goal: đủ 3 × 6 giây, OFF/HOME từng lượt, không tự phun lượt thứ tư
liên tiếp, Q confirmed, max raw owner một. Đây không phải thử bơm vật lý.

## QA và giới hạn

**Kết quả cuối: 982 PASS, 2 SKIP, 0 FAIL trên 984 test ID duy nhất.**
Không cộng rerun: full 980 PASS/2 SKIP (492,42 giây), lượt ảnh hưởng sau sửa
query 452 PASS/2 SKIP (378,11 giây), release 30 PASS (4,60 giây). Chẩn đoán
pose có thêm một dòng reset tuổi mẫu cũ khi frame sai; 22 ca pose/diagnostic
được kiểm lại và nằm trong 30 ca release. Hai SKIP là tổ hợp không có Goal
nhưng yêu cầu trạng thái navigation; các nhánh không Goal được kiểm riêng.
Hai file runtime thay đổi sau khi full bắt đầu đã được kiểm lại trong lượt
ảnh hưởng/release. Không gọi full cũ là một lượt chạy lại toàn bộ trên source
cuối. `summary.json` lưu ID và lượt kiểm mới nhất cho từng ca.

Gói 452 source pins, native CTest và smoke IDLE PASS; laptop **95/95** và Pi
**103/103** file cài đúng hash. Đã sync LAN/build hai package tại
`/home/pi/fire_robot_desktop/95e9facbc7d2/ros`, chọn `artifacts/current.txt`.
Bộ trước 62c78ddf và các gói thử trung gian giữ nguyên để đối chiếu.

Probe chuẩn bị STOP chỉ đọc phiên lỗi thật đã PASS (`SAME_BOOT_CLEANUP`),
không mở I/O. Pending SHA `48bf5239b1167c44c1215627346b7e9da2f8638a0c0fe0fa27ec5ad47d58559c`
nguyên vẹn trước/sau chọn gói. Chưa tạo phiên STOP mới, chưa chạy xe/bơm,
không flash firmware, đổi giờ, commit hoặc push.

- Ma trận 32 ca baseline: 30 ca chức năng (24 gap đơn, ba gap chồng, hai
  X rồi Goal mới, một command factory) và hai diagnostic pose/STOP epoch.
  12 lỗi chức năng đã FAIL trước sửa được kiểm lại PASS; diagnostic xác nhận
  guard đúng, không đếm thành khả năng phục hồi identity mới.
- Thêm ca dữ liệu lại trễ trong cửa CREATE/first-forward, operator giữ input,
  proof liên tục, NaN/ngoài giới hạn, nguồn sai, X/Q/cancel epoch/terminal.
- 17 ca pose refresh kiểm guard âm, rate-limit, một future, timeout/loại request
  cũ, dịch vụ lỗi và callback R5 thực; không lấy service response làm proof.
- Test nâng gói khi còn pending cũ xác nhận SCAN/FIRE vẫn bị từ chối và chỉ
  VERIFY_STOP được tạo phiên mới đúng packet; pending cũ không bị ghi đè.
- Full regression và lượt ảnh hưởng cuối sau sửa query được ghi riêng trong
  XML; `summary.json` hợp nhất test ID duy nhất, không cộng lần chạy lại.
- R5 gốc 364 file và manifest parent `0c54d9b7` nguyên hash; mapping runner,
  teleop, map saver, model `best_minireplay.pt`, inflation global/local
  0,25/0,30 m, footprint/padding nguyên byte. Không đổi watchdog, firmware,
  topic/QoS, ARM/freshness, deadline và emergency STOP.

Q khi operator còn context epoch cũ vẫn không được giả STOP confirmed cho
identity mới. Phải dùng đường **Xác minh dừng trên robot** để lấy proof hiện
hành sau khi các tiến trình cũ được xác nhận đã thoát. Phiên lỗi 02:31 và
pending của nó được giữ nguyên. Không tự chạy xe/bơm, xóa pending hoặc sửa
receipt lịch sử trong lượt sửa code này.

## Sử dụng sau triển khai

Đóng app cũ, mở lại từ launcher thường dùng. Nếu còn pending phiên 02:31,
kết nối Pi và dùng **Xác minh dừng trên robot**; chỉ sau STOP/cleanup mới đạt
và app về IDLE mới chọn map và khởi động. Với cùng Goal/cùng identity, dữ liệu
trễ sẽ giữ dừng và dữ liệu hợp lệ ổn định trở lại sẽ cho tiếp tục. X vẫn hủy
nhiệm vụ để đặt Goal mới; Q dừng phiên; Esc chỉ bỏ công cụ RViz.
