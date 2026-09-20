# Cài dashboard điều khiển và bản đồ GPS TripGuard

Gói này dùng cho ThingsBoard CE 4.3.x và thiết bị tên `tripguard-01`.

## Trước khi import

1. Docker Desktop phải hiện **Engine running**.
2. ThingsBoard mở được tại `http://localhost:8081` (hoặc cổng web bạn đã đặt).
3. Mở **Entities > Devices > tripguard-01 > Latest telemetry** và kiểm tra có dữ liệu.
4. Để bản đồ có vị trí, phải có hai khóa số `latitude` và `longitude`. Tốt nhất `gps_state = FIXED` và `position_current = true`.

## Import đúng thứ tự

### 1. Cài hai widget tùy chỉnh

Mở một dashboard bất kỳ, bấm **Edit > Add widget > Import widget**:

1. Import `01_TripGuard_Control_Widget.json`.
2. Import `02_TripGuard_GPS_Live_Map_Widget.json`.

Hai widget này sẽ được lưu trong widget library của tenant. Widget điều khiển dùng **one-way RPC**, đúng với firmware MQTT hiện tại.

### 2. Import dashboard hoàn chỉnh

Vào **Dashboards > dấu + > Import dashboard**, chọn:

`03_TripGuard_Dashboard_Control_GPS_Map.json`

Mở dashboard vừa tạo. Alias mặc định trỏ tới device có tên chính xác `tripguard-01`.

Nếu thiết bị của bạn có tên khác: mở dashboard ở chế độ Edit > Entity aliases > sửa alias thành đúng device.

## Các nút điều khiển

- **Phát cảnh báo SOS**: phát track cảnh báo SOS trên loa.
- **Dừng loa**: dừng âm thanh.
- **Bật/Tắt cảnh báo radar**: cho phép hoặc khóa cảnh báo radar.
- **Xác nhận / dừng cảnh báo**: acknowledge trạng thái SOS.
- **Cập nhật telemetry ngay**: yêu cầu STM32 gửi dữ liệu mới.
- **Gửi thử SMS radar**: gửi SMS tới số khẩn cấp, có hộp xác nhận.
- **KÍCH HOẠT SOS TỪ XA**: phát loa, gửi SMS và gọi số khẩn cấp; chỉ bấm khi cần thử thật.
- **Camera về radar / Camera về home**: quay servo 360 theo thời gian rồi tự dừng.
- **Dừng servo**: đưa PWM về mức dừng ngay.
- **2/5/10 giây**: đổi chu kỳ gửi telemetry.

Sau khi bấm, xem `last_rpc_method`, `last_rpc_result` và `remote_command_count` trên dashboard để xác nhận STM32 đã nhận lệnh.

## Khi bản đồ chưa hiện

- Đưa anten GPS ra ngoài trời, mặt anten hướng lên trời; lần fix đầu có thể mất vài phút.
- Kiểm tra `gps_link_alive = true`, `position_available = true`, `latitude` và `longitude` khác 0.
- Trình duyệt phải có Internet để tải lớp bản đồ OpenStreetMap. Telemetry MQTT vẫn chạy ngay cả khi nền bản đồ chưa tải.

## Lưu ý vận hành

- Luôn giữ cửa sổ Pinggy MQTT đang chạy. Tunnel miễn phí đổi host/port sau khi tạo lại, khi đó phải cập nhật `TRIPGUARD_TB_HOST` và `TRIPGUARD_TB_PORT`, build rồi nạp lại STM32.
- Khi gửi SMS hoặc gọi SOS, firmware tạm dừng MQTT để modem A7670 thực hiện cuộc gọi/SMS rồi tự kết nối lại.
