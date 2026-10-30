# Mốc khôi phục source app trước khi chỉnh đường đi — 30/10/2026

Validation-Record: `APP_SOURCE_CHECKPOINT_20261030_6133`.
Nhánh: `codex/nav2-phase3-3`.
Tag dự kiến khi hoàn tất push: `desktop-inflation-030-source-pass-v1`.

## Trạng thái được lưu

- App source packet: `6133ff387973f5c7ccc6b245b6acaa1c6d74126ea3f2f5b0eb4f75a376f60426`.
- Parent R5: `0c54d9b797655ebca350b9ad21062791b1dd2a3fbab6949516983d3af9ffcdd0`;
  364 file trong `releases/r5_20261029/ros` giữ nguyên.
- Local/global inflation 0,30 m; scaling 3,0/5,0; footprint 34 × 20 cm,
  padding 1,5 cm. RPP lookahead_time 2,0 s giữ nguyên.
- App quản lý quét/lưu/chọn map, khởi động chữa cháy, STOP/cleanup, xác minh
  dừng, X/Goal, Q dừng phiên, Esc bỏ công cụ RViz và hiển thị xe/đường Nav2.
- Model ngoài repo `best_minireplay.pt`, SHA256
  `e792f896faf3fb584842898e93913cdfd0fc33b5d48d620163fadd15f02212c4`.
- Static preflight của parent còn kiểm tra model cũ `best_29_10_2026.pt`, SHA
  `28d3b25f70ccd0ea958a2abfa11f35804f5a82c2f711e4786d5bfa6cce46baca`.
  Hai file weights và venv YOLO là dependency ngoài Git; không tự tải từ tên.

Code nằm trong Git; artifact native/build/install, dữ liệu xác thực, model,
log chạy thử và raw QA giữ cục bộ. `checkpoint.json` ghi packet, hash artifact
đã QA và hash bằng chứng. Commit source không sao lưu toàn bộ môi trường máy.

## Kết quả QA đã có

- Bản sửa X/Goal trước thay đổi inflation: 798 ca đạt, 2 không áp dụng;
  không coi đây là toàn bộ runtime robot đã đạt.
- Bản inflation 0,40 m: 184 ca đạt, 2 không áp dụng; CTest đạt.
- Bản inflation 0,30 m: 20 ca source/launch/runner/model và 1 ca mở/đóng gói
  IDLE đạt, CTest đạt. YAML chỉ hai bán kính thay đổi so với bản 0,40 m;
  tất cả service app giữ byte/hash.
- Laptop static 95 file, Pi static 103 file đạt; 449 source pins khớp.
- Các báo cáo lịch sử trong thư mục này giữ nguyên kết quả FAIL/BLOCKED và
  giới hạn tại thời điểm ghi. Raw `docs/qa_evidence/` giữ trên laptop, không
  đưa log xác thực/phiên lên GitHub.

**Còn OPEN:** đường xanh đổi trái/phải quanh vật cản, vòng cua rộng; nguyên
nhân chưa chốt. Chưa áp dụng BT giữ đường hoặc giảm lookahead. Khoảng hở xe
và nghiệm thu đầy đủ robot không được suy ra từ source/build/static PASS.
Lượt commit/push không chạy xe, bơm, ROS, SSH Pi, NTP hoặc flash.

## Khôi phục

Ưu tiên worktree riêng để không ghi đè code đang sửa:

```bash
git fetch origin --tags
git worktree add --detach /tmp/fire-robot-restore-030 desktop-inflation-030-source-pass-v1
```

Nếu artifact `6133ff38…` và receipt Pi tương ứng vẫn còn trên máy, có thể
chọn lại gói này sau khi dừng phiên, xác nhận STOP/cleanup và đóng app cũ.
Chỉ khôi phục file code bằng Git không thay cấu hình của app đang chạy.

Khi cần dựng lại từ checkout, dùng `services.candidate_build` với parent R5
và app trong cùng checkout; packet source tạo lại phải bằng SHA đầy đủ nêu
trên. Đóng gói bằng `launchers/prepare_candidate.sh`, kiểm tra static hai
host, rồi đồng bộ Pi và chọn gói sau khi không có phiên đang chạy. Các wrapper
desktop/dependency có đường dẫn theo laptop hiện tại; cần điều chỉnh đường
dẫn cho checkout mới, không chạy launcher mặc định nếu nó vẫn trỏ về repo cũ.

Binary dựng lại có thể có SHA khác do đường dẫn build/toolchain. Phải so
packet source và chạy lại build/static cần thiết; không giả receipt Pi hoặc
xóa `pending.json` để vượt khóa. Không reset/force-push nhánh đang phát triển.
