# TripGuard ThingsBoard Dashboard

File cần import: `TripGuard_Full_System_Dashboard.json`

## Cách import

1. Mở ThingsBoard và chọn **Dashboards**.
2. Chọn **+ Add dashboard** → **Import dashboard**.
3. Chọn file `TripGuard_Full_System_Dashboard.json` rồi nhấn **Import**.
4. Mở dashboard **TripGuard - Full System Monitor**.

Dashboard gồm ba khu vực:

- Tổng quan toàn bộ trạng thái STM32, ESP32, Raspberry Pi, radar, SOS,
  modem, âm thanh và servo.
- Bản đồ GPS dùng telemetry `latitude` và `longitude`.
- Camera trong xe hiển thị ảnh evidence mới nhất từ telemetry
  `evidenceImage`, kèm trạng thái phát hiện người và độ tin cậy.

Dashboard mặc định tìm thiết bị có tên `Raspberry Pi Demo Device` và device type
`default`. Nếu dashboard không có dữ liệu, vào chế độ chỉnh sửa, mở
**Entity aliases**, sửa alias **TripGuard Device** và chọn trực tiếp đúng thiết bị.

Dashboard chỉ đọc telemetry, không gửi RPC và không thay đổi thiết bị.

Lưu ý: bản đồ chỉ có marker khi GPS có fix hợp lệ và đã gửi
`latitude`/`longitude`. Khung camera là ảnh sự kiện gần nhất, không phải
video stream liên tục.
