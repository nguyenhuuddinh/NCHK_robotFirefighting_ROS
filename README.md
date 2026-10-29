# Fire Robot ROS 2

Robot chữa cháy dùng ROS 2 Humble: Raspberry Pi thu thập camera/Lidar và giao tiếp ESP32; laptop chạy Nav2, YOLO, RViz và điều phối hành vi.

## Bản R5 hiện hành

Source R5 automatic ngày 29/10/2026 nằm trong **[releases/r5_20261029](releases/r5_20261029/README.md)**. Đây là snapshot đầy đủ của packet `0c54d9b79765`, dùng model `best_29_10_2026.pt` với confidence0,15 và firmware ESP32 `ce19ecd3f4fa`.

Thư mục `ros2_ws/` và `scripts/` ở gốc vẫn giữ workflow nền trước khi tích hợp R5. Khi cần đúng code R5 đã debug, dùng snapshot và manifest trong thư mục release; xem README của release để phân biệt kết quả offline và runtime còn chờ.

Firmware đi kèm: [NCHK_robotFirefighting_mainBoard_esp32_S3](https://github.com/nguyenhuuddinh/NCHK_robotFirefighting_mainBoard_esp32_S3), snapshot `releases/r5_20261029/firmware`.
