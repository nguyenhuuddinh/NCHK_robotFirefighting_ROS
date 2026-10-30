# App thoát khi khởi tạo RViz trong lượt quét — 30/10/2026

**Lượt quét chưa thành công; STOP telemetry đã xác nhận và Pi dọn sạch.**
Laptop session `4d6778e7-3c19-4b8e-9e0a-c62f05ce05dc`, log
`~/.local/state/fire_robot_control/session_dced8fec91914a96b1f2f0cae7db4e5a`.
Pi `/home/pi/qa_logs/r5_automatic_20261030_065524`. App packet `abf4aae3`.

Sau khi user bấm Quét và xác thực, app qua WAIT_PI_IO và WAIT_SLAM. Pi I/O
đạt; laptop khởi chạy slam_toolbox. GUI bắt đầu dựng RViz, sau đó có
APP_VIEW_ATTACHED và ngay sau đó STOPPING/APP_VIEW_DETACHED; cuối cùng GUI bị
supervisor đóng, exit code -15. Laptop ghi `FRONTEND_STOP_REQUEST` tại
WAIT_SLAM và `InterruptedError:MAPPING_STOP_DURING_SLAM`. Receipt không tách
được đây là CLOSE do lease hết hạn, mất socket hay một yêu cầu frontend khác;
không gán chắc stop request do user tự bấm. Trạng thái STOPPING ghi còn
57,021s trong hạn WAIT_SLAM60s; đây là khoảng thời gian kể từ deadline được
thiết lập, không phải phép đo chính xác thời gian khởi tạo RViz.

STOP monitor báo `FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED` sau 0,938s; local
clean, Pi DONE và 8/8 child sạch, không ép dừng. Pi không còn process robot;
pending không còn sau cleanup. Không có bằng chứng xe di chuyển hoặc map đã
quét thành công; STOP telemetry không thay thế quan sát vật lý xe/bơm.

Điều tra code: GUI tạo RViz đồng bộ trên Qt event loop; frontend phải gửi
heartbeat. Có cửa sổ khởi tạo tối đa 3s trước admission. Test GUI thực qua
supervisor giả ROS localhost211, chèn trễ 3,3s trước initialize: thao tác SCAN
vào WAIT_SLAM, GUI gắn RViz rồi bị đóng (GUI_EXIT_CODE=-9), workflow không
hoàn thành. Chứng minh đường tải chậm có thể gây lỗi tương tự; log thực chưa
ghi thời điểm VIEW_INIT/lease nên chưa chứng minh tuyệt đối trigger trong lượt
thật. Test cũ `slow_workflow` chỉ chèn 0,9s và không phủ trường hợp này.

Chưa sửa logic bảo vệ hoặc nâng timeout. Cần sửa kiến trúc khởi tạo view để
event loop vẫn xử lý STOP/heartbeat trong lúc tải, và QA deadline, mất GUI,
STOP trong khởi tạo, admission trước teleop cùng RViz thật. Không tự kéo dài
lease hoặc giả heartbeat trong khi GUI bị treo. Trước khi sync cần xác nhận
R5/ROS/STOP/watchdog giữ nguyên và chạy QA đủ; sau đó thử quét không lái.

Lỗi JPEG corrupt được log đồng thời và còn mở riêng; không khẳng định nó gây
app thoát. [Evidence](qa_evidence/mapping_gui_disconnect_20261030/) gồm receipt,
log Pi đã lọc và fixture tái hiện, không chứa mật khẩu/pairing bí mật.
