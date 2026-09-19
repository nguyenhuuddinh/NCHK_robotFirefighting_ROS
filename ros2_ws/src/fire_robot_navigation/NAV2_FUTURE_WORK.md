# Nav2: baseline đã chốt và lộ trình tối ưu sau giai đoạn YOLO

Ngày chốt: 2026-09-19. Tài liệu này **chỉ ghi kế hoạch**, không thay đổi cấu
hình đang chạy và không thay thế quy trình an toàn khi thử trên xe thật. Từ đây
ưu tiên kiểm thử YOLO; chỉ mở lại Nav2 khi có thời gian làm từng bài A/B có
rosbag, UART và phép đo định lượng.

## 1. Trạng thái được giữ nguyên

- Nav2 chạy trên laptop; Pi giữ LiDAR, bridge, TF và safety watchdog.
- Controller `RegulatedPurePursuitController` (RPP), planner NavFn, replan BT
  `2 Hz`, controller `10 Hz`, odom/IMU thật khoảng `10 Hz`.
- Tốc độ mong muốn `0,10 m/s`, quay tối đa cấu hình `1,0 rad/s`; RPP dùng
  `rotate_to_heading_min_angle=0,174533 rad` (~10°). Đây **không** phải cam kết
  rằng xe sẽ xoay chính xác dưới 10° theo đường thẳng nối tới goal trước khi
  bắt đầu tiến: RPP căn với tiếp tuyến của global path/carrot đang thay đổi.
- Goal tolerance `0,08 m` và `0,15 rad` (~8,59°). Global và local inflation
  cùng `radius=0,55 m`, `cost_scaling_factor=3,0`; footprint là polygon đo từ
  xe, padding `0,015 m`. Giữ collision detection, watchdog và cấm chạy lùi.
- Người vận hành báo xe đã chạy được và tránh vật cản. Chưa có bài đo định
  lượng đủ chuẩn để kết luận né **vật cản động** đã mượt hay an toàn ở mọi góc.

Baseline sau hiệu chuẩn geometry (rosbag/UART tại `docs/codex/artifacts/`,
thư mục này bị `.gitignore` bỏ qua nên cần giữ bản local riêng):

| Bài | Kết quả | Số đo đáng chú ý |
|---|---|---|
| Goal thẳng ~0,4 m | `SUCCEEDED`, 4,85 s, 0 recovery | Path đầu 0,4423 m / đường thẳng 0,3897 m (+13,5%); lệch ngang 5,82 cm; 4 lần đổi dấu `wz` đáng kể. |
| Goal ~0,5 m phía sau | `SUCCEEDED`, 21,270 s, 0 recovery | Sai số cuối 7,86 cm / 3,39°; quay tại chỗ 17,0 s, tiến 4,2 s; khoảng 8 đoạn tiến xen quay, 9 lần đổi dấu `wz`. |

Ở bài phía sau, lệnh tiến đầu xuất hiện khi hướng xe còn lệch 25,32° so với
đường thẳng tới goal. Có 41 path ở 1,888 Hz; tiếp tuyến đầu nhảy >10° ở
12/40 lần replan, lớn nhất 77,63°. Path lệch ngang tối đa 17,27 cm, trong
khi đường nối thẳng không chứa cell lethal/unknown ở những mẫu đã kiểm tra.
Đây là bằng chứng cho dao động path/cost, **không** chứng minh toàn bộ swept
footprint đi thẳng đều an toàn. Pha quay đầu tích phân lệnh ~300° nhưng
odom/IMU chỉ đổi ~156°; phản hồi bánh còn lệch nhau. Planner và phần cứng quay
đều có phần đóng góp, không nên quy mọi sự giật cho một tham số RPP.

Nguồn chi tiết:

- `docs/codex/artifacts/fire_robot_nav2_forward_geometry_20260917_lR8Kqs/README.md`
- `docs/codex/artifacts/fire_robot_nav2_behind_geometry_20260917_xvUEMv/README.md`
- `NAV2_TUNING_GUIDE.md` trong package này.

## 2. Mục tiêu hành vi, không phải thông số đã triển khai

Trong vùng trống đủ rộng, ưu tiên một path ngắn, ít đổi tiếp tuyến. Khi bắt
đầu với goal phía sau, robot nên quay gần hướng **đoạn path đầu an toàn** rồi
tiến ổn định. Mốc 8–10° ban đầu, 10–25° thì giảm tốc/sửa hướng, >25° thì
dừng tiến để xoay lại là ý tưởng thử nghiệm của người vận hành. Phải phân biệt
góc tới goal với góc tiếp tuyến path: nếu vật cản buộc đi vòng, xe **không
được** luôn ép đầu về vector thẳng tới goal. Cần hysteresis và thời gian giữ
trạng thái để tránh chuyển tiến/quay liên tục tại ranh ngưỡng.

RPP hiện tại không cung cấp nguyên bộ điều khiển 3 dải góc độc lập như trên.
Không tự viết wrapper/BT mới hay đổi controller chỉ để khớp các con số trước
khi xử lý global path và đo khả năng quay thực tế.

## 3. Thứ tự A/B khi mở lại Nav2

Mỗi lượt chỉ đổi **một** biến, ghi commit/config/bag, cùng initial pose, goal,
vị trí vật cản và tình trạng sàn/pin. Luôn có phép thử goal thẳng, phía sau,
và vật cản; so với baseline chứ không dựa vào cảm giác nhìn RViz.

1. **Chẩn đoán path/costmap không có motor.** Lưu `/plan`, global/local
   costmap, `/scan`, TF, AMCL, `/odom`, `/cmd_vel_nav`, `/cmd_vel_raw`, action
   status và UART. Chụp start/goal theo tọa độ `map`. Đo chiều dài path / dây
   cung, lệch ngang, tiếp tuyến đầu và biến thiên qua từng replan; quét **toàn
   bộ footprint theo đường thẳng**, không chỉ tâm robot. Kiểm tra nhiễu scan,
   dấu footprint, map/AMCL và chướng ngại chưa xóa. Không làm mỏng footprint
   hoặc bỏ collision check để tạo đường thẳng giả.
2. **Inflation global-only.** Thử `global_costmap.inflation_layer.cost_scaling_factor`
   `3,0 -> 4,0` như một candidate, giữ `inflation_radius=0,55 m`, local
   inflation và polygon/padding. Hệ số cao làm cost giảm nhanh hơn theo khoảng
   cách; NavFn có thể bớt vòng trong vùng hồng nhưng clearance sẽ giảm. Chỉ
   chấp nhận nếu swept footprint + biên an toàn không chạm obstacle, path gọn
   hơn ở nhiều pose và bài vật cản vẫn né thành công. Nếu không, rollback.
3. **Smoothing riêng.** `SimpleSmoother` đã có trong YAML nhưng chưa được gọi
   bởi Behavior Tree. A/B thêm `SmoothPath` đúng phiên bản Nav2 Humble đang cài,
   giữ NavFn, inflation, RPP và tần số. Đo path tangent, số pha quay/tiến và
   clearance sau smooth. Smoother làm bớt góc gãy; nó **không** sửa nguyên
   nhân NavFn chọn hành lang vòng. Không smooth xuyên vật cản.
4. **Planner riêng.** Nếu đường vẫn vòng trong vùng đủ clearance, so NavFn
   với Theta* (hoặc Smac2D nếu Humble trên máy có plugin tương thích) ở cùng
   costmap. Đường thẳng hơn không mặc nhiên tốt hơn: kiểm tra footprint/cua
   tại chỗ, nhiễu path khi replan và vật cản xuất hiện đột ngột. Không đồng
   thời tăng planner Hz; `2 Hz` hiện đã tạo nhiều đổi tiếp tuyến.
5. **Quay CW/CCW và tracking từng bánh.** Thử riêng ở dải `wz` Nav2 dùng,
   ghi command, odom, IMU, UART target/actual/PWM, độ trôi tâm, điện áp pin và
   loại sàn. Định lượng gain thực, độ trễ khi đảo chiều và sai lệch bốn bánh.
   Chỉ sau đó mới cân nhắc FF/PID hoặc effective track width bằng thay đổi
   firmware riêng, có test/build/flash gate và kiểm tra tiến thẳng lại.
6. **RPP và goal approach.** Sau khi path/rotation ổn định, thử lookahead,
   tốc độ trong cua, ngưỡng rotate-to-heading và cuối goal, mỗi lần một biến.
   So số lần `vx=0,wz!=0`, đổi dấu `wz`, độ lệch path, thời gian/overshoot và
   tỷ lệ `SUCCEEDED`. Chỉ cân nhắc state machine 3 dải góc có hysteresis nếu
   RPP thuần vẫn không đạt. Giữ tolerance `8 cm / 8,59°` trừ khi sai số thực
   cho thấy cần thay; không nới tolerance để che lỗi định vị hay motor.
7. **Vật cản động.** Đặt vật đủ cao để LiDAR nhìn thấy, ở nhiều khoảng cách
   và phía trái/phải, có người đứng cạnh để dừng khẩn. Đo từ phát hiện ->
   costmap -> path mới -> lệnh chậm/dừng/né, khoảng cách gần nhất, số recovery
   và tỷ lệ đạt goal. Nếu hành lang không đủ rộng, hành vi đúng là dừng/chờ,
   không ép lách. Kiểm tra vật cản rời đi và path phục hồi.
8. **Chỉ sau các bước trên:** cân nhắc MPPI khi RPP vẫn có giới hạn điều khiển
   đã được chứng minh, benchmark CPU/tần số và safety trên cùng bag. MPPI
   không tự chữa path đầu vào dao động hay gain quay sai. Tăng planner Hz hay
   odom/IMU Hz chỉ khi log chứng minh nghẽn thời gian/dữ liệu; publish nhanh
   hơn không tạo telemetry thật nhanh hơn từ firmware.

Mốc đánh giá đề xuất cho **thí nghiệm**, không phải lời hứa đã đạt: zero va
chạm và không thu hẹp clearance đo theo footprint; goal thành công không
recovery trong ít nhất 3 lượt/ca; góc path đầu ít nhảy >10° hơn baseline
12/40; goal thẳng có path/direct và lệch ngang nhỏ hơn baseline +13,5%/5,82 cm;
goal phía sau có ít hơn 8 burst tiến và 17 s quay nhưng vẫn dừng trong tolerance.
Không đánh đổi các mốc mượt này lấy việc đi vào vùng không an toàn.

## 4. Quy trình safety và rollback

Trước **mỗi** lệnh có thể làm xe chạy: operator xác nhận đang cạnh xe, vùng
chạy trống/được chặn, pump OFF, stop sẵn sàng; ROS time/TF, LiDAR, odom, IMU,
costmap, footprint và safety watchdog khỏe; không có teleop tranh lệnh. Dry-run
path + footprint rồi mới gửi một goal có timeout/cancel và bảo đảm phát zero
cuối bài. Nếu mất scan/TF, path bất thường, va chạm gần, pump không xác nhận
OFF hoặc UART mất dữ liệu khi cần đo motor: dừng, không tiếp tục goal.

Lưu nguyên baseline config và bag trước A/B; ghi diff một tham số, lệnh test,
ảnh/costmap và số đo. Thất bại thì rollback đúng diff của lượt đó, không reset
cả worktree của người dùng. Không flash firmware cùng lượt đổi planner/RPP.

## 5. Bàn giao sang YOLO

Nav2 hiện đủ làm baseline di chuyển thử nghiệm, chưa tuyên bố tối ưu. Khi
chuyển sang YOLO: kiểm tra luồng camera, FPS/độ trễ và timestamp/TF camera;
đánh giá phát hiện lửa/người/vật cản trên dữ liệu thật; ghi log false positive
và mất frame. Giữ chức năng phát hiện tách khỏi lệnh motor/pump cho tới khi có
interlock và safety test riêng. Nếu YOLO dùng goal hay can thiệp Nav2, thử
trước ở chế độ chỉ quan sát, không làm suy yếu watchdog hoặc tránh va chạm.
