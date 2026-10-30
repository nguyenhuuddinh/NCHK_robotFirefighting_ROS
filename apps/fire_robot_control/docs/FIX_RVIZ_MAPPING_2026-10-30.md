# Sửa khởi tạo RViz khi quét map — 30/10/2026

Bản app `ea4a2eec84e0074ac2cefc5d1558477b88c3858f4962524e43a75c9fa30a322f` đã được chọn cho launcher; Pi đã nhận cùng packet qua Wi-Fi và có receipt kiểm tra tĩnh. Chưa chạy thử phần cứng với bản này. Báo cáo điều tra ban đầu: [MAPPING_GUI_DISCONNECT_2026-10-30.md](MAPPING_GUI_DISCONNECT_2026-10-30.md).

## Thay đổi

- Lượt quét dựng RViz trong tiến trình con, gắn cửa sổ RViz vào app; luồng Qt chính tiếp tục xử lý heartbeat và nút dừng. Chờ view dùng lease 350 ms và giám sát heartbeat worker 750 ms thông thường, không tăng deadline và không giả heartbeat.
- Chỉ cấp quyền hiển thị cho đúng phiên mapping ở WAIT_SLAM. Runner vẫn chờ view sẵn sàng trước khi tiếp tục; view con chỉ cho phép MoveCamera và Select. Khi view lỗi, app yêu cầu STOP; khi dừng, app yêu cầu view đóng, sau 1,2 s thì cưỡng chế nếu cần. Nếu GUI cha chết, kernel kết thúc tiến trình con.
- Nút Q/Esc tại khung RViz chuyển yêu cầu STOP về app. Khung camera chiếm tối đa 55% chiều rộng khi map đang mở, nhằm giữ cả hai vùng nhìn được.
- Luồng chữa cháy vẫn dùng cơ chế khởi tạo RViz cũ; lỗi tải chậm của luồng đó chưa được xử lý bởi thay đổi mapping này. R5, watchdog và firmware không thay đổi.

## Kết quả QA

- 224 unittest đạt trên laptop với Unix socket được phép.
- QA tích hợp: 164 đạt; 4 phép thử health trong lượt đầu dừng vì thư mục bằng chứng cũ đã tồn tại. Chạy lại bốn phép thử trong thư mục mới: 4 đạt. Thêm lượt GUI cuối 7 đạt, gồm RViz chậm 3,3 s, dừng lúc tải, tiến trình view lỗi, phím Q và đủ chuỗi quét/lưu/dừng/chọn/khởi động/dừng giả lập. Thử vòng đời lặp và graph dọn sạch: 2 đạt. Toàn bộ ROS thử nghiệm dùng localhost/domain cô lập và fake session.
- Kiểm tra hình với dữ liệu map giả: RViz và camera cùng cửa sổ. Gói release build và CTest đạt; mở/đóng nhàn rỗi gói release đạt.
- Pi kiểm tra tĩnh 427 source pin, 103 installed source pin; laptop kiểm tra 95 installed source. Đối chiếu 24.843 tệp cũ trên Pi trước/sau: không đổi; baseline Pi sạch và không có tiến trình robot sau sync. R5 parent SHA-256 `0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0`, kiểm tra manifest gốc đạt.
- Log đối chiếu tại `/tmp/fire_app_async_sync_20261030`; receipt Pi ở `~/.local/state/fire_robot_control/pi_receipts/` theo tên packet. Các kiểm thử không xác nhận trạng thái cơ khí robot hay hoàn thành quét thật.

## Giới hạn và bước thử thật

Nguyên nhân của lần thoát trước chưa được chứng minh duy nhất bởi log thực: test chỉ cho thấy RViz tải chậm có thể gây hiện tượng tương tự. Lỗi JPEG ghi nhận trước đó chưa xử lý. Cần thử quét thật khi người dùng có mặt, đảm bảo khu vực an toàn: mở app mới, ấn Quét map, xác thực nếu được hỏi, quan sát khung map/trạng thái, thử DỪNG PHIÊN trước khi điều khiển. Nếu xuất hiện lỗi hoặc STOP chưa xác nhận, giữ robot an toàn và thu log; không khẳng định robot đã dừng chỉ từ kết quả giả lập.
