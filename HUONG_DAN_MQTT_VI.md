# Bắt đầu nhanh: STM32 + A7670 gửi ThingsBoard bằng MQTT

## Bản này đã đổi gì

- Bỏ luồng `AT+HTTP...` trong firmware.
- Dùng MQTT 3.1.1 của A7670 với `AT+CMQTTSTART`, `AT+CMQTTCONNECT`,
  `AT+CMQTTSUB...` và `AT+CMQTTPUB...`.
- Giữ một phiên MQTT, gửi telemetry mặc định mỗi 2 giây.
- Subscribe `v1/devices/me/rpc/request/+` để nhận lệnh từ dashboard/app.
- Khi cần dùng SMS/cuộc gọi, firmware đóng MQTT an toàn rồi tự kết nối lại.

## Chỉ gắn SIM/A7670 thì nối như sau

| A7670 | STM32F411 | Ghi chú |
|---|---|---|
| TXD | PA3 / USART2_RX | TX modem sang RX STM32 |
| RXD | PA2 / USART2_TX | RX modem nhận từ TX STM32 |
| GND | GND | Bắt buộc chung mass |
| Nguồn | Nguồn riêng đúng chuẩn module | Không lấy nguồn modem từ chân 3.3 V STM32 |

GPS, radar, loa và nút có thể chưa gắn. Firmware vẫn chạy MQTT; các trường
telemetry tương ứng sẽ ở trạng thái mặc định/offline.

## Việc phải làm trước khi nạp

1. Trên đúng máy đang chạy ThingsBoard Docker, kiểm tra
   `http://localhost:8081` vào được.
2. Cũng trên máy đó, mở PowerShell và giữ cửa sổ này luôn chạy:

```powershell
ssh -p 443 -R0:localhost:1883 tcp@a.pinggy.io
```

3. Pinggy sẽ in một dòng dạng:

```text
tcp://ten-moi.run.pinggy-free.link:so-port-moi
```

4. Mở `Core/Inc/tripguard_config.h`, thay đúng hostname và port vừa nhận:

```c
#define TRIPGUARD_TB_HOST          "ten-moi.run.pinggy-free.link"
#define TRIPGUARD_TB_PORT          12345U
#define TRIPGUARD_TB_ACCESS_TOKEN  "Trankhoinguyen0110"
```

Không ghi `tcp://` vào `TRIPGUARD_TB_HOST`. Port bên ngoài của Pinggy thường
là số ngẫu nhiên, không phải 1883. Bản ZIP đang điền endpoint gần nhất
`vsxvv-58-187-48-135.run.pinggy-free.link:45483`; nếu cửa sổ Pinggy cũ đã
đóng hoặc quá 60 phút thì endpoint này đã hết hạn và phải thay lại.

5. Trong STM32CubeIDE chọn `Project > Clean`, sau đó `Build Project` và nạp
   firmware mới. Không dùng file `.elf` cũ trong bản ZIP ban đầu.

## Cấu hình MQTT ThingsBoard đang dùng

| Mục | Giá trị |
|---|---|
| Giao thức | MQTT 3.1.1 qua TCP |
| Broker nội bộ | ThingsBoard Docker, cổng 1883 |
| Username MQTT | Access token của device |
| Password | Không truyền tham số password |
| Client ID | `tripguard-stm32f411` |
| Telemetry topic | `v1/devices/me/telemetry` |
| RPC subscribe | `v1/devices/me/rpc/request/+` |
| Keepalive | 60 giây |
| Publish QoS | 1 |

## Kiểm tra sau khi nạp

Thêm vào Live Expressions:

```text
tripguard_modem_state
tripguard_modem_ready
tripguard_network_registered
tripguard_pdp_active
tripguard_mqtt_state
tripguard_mqtt_phase
tripguard_mqtt_connected
tripguard_publish_count
tripguard_mqtt_error_count
(char*)tripguard_mqtt_last_error
```

Kết nối thành công khi:

- `tripguard_modem_state = MODEM_READY`;
- `tripguard_network_registered = 1`;
- `tripguard_pdp_active = 1`;
- `tripguard_mqtt_state = MQTT_CONNECTED`;
- `tripguard_mqtt_connected = 1`;
- `tripguard_publish_count` tăng khoảng mỗi 2 giây.

Nếu modem đã READY nhưng MQTT không kết nối, kiểm tra trước tiên cửa sổ
Pinggy còn mở không, endpoint trong `tripguard_config.h` có đúng không và
token có đúng với device ThingsBoard không.

## RPC đã hỗ trợ

Dashboard/app có thể gửi one-way RPC với các method:

| Method | Params | Tác dụng |
|---|---|---|
| `playAlarm` | `1` hoặc `2` | Phát âm thanh khi đã gắn loa |
| `stopAudio` | bỏ qua | Tắt âm thanh |
| `setRadarAlerts` | `true` / `false` | Bật/tắt cảnh báo radar |
| `acknowledgeSOS` | bỏ qua | Xác nhận SOS |
| `sendRadarSms` | bỏ qua | Gửi SMS radar |
| `triggerSOS` | bỏ qua | Phát loa, gửi SMS và gọi |
| `setTelemetryPeriod` | `2000` đến `60000` | Đổi chu kỳ gửi, đơn vị ms |
| `refreshTelemetry` | bỏ qua | Yêu cầu gửi telemetry sớm |

Khi hiện tại chỉ gắn A7670, nên thử `setTelemetryPeriod` hoặc
`refreshTelemetry` trước. Các lệnh loa/radar cần phần cứng tương ứng.
