# TripGuard: trạng thái logic và đấu dây

## Luồng chuẩn hiện tại

```text
SELF_TEST cục bộ
       |
       v
STANDBY / logical sleep
       |-- Rear Confirm hợp lệ ----------------------+
       |                                              |
       |-- GPS ở geofence + tốc độ thấp đủ 15 s --> WAIT_REAR_CONFIRM
                                                      |-- Rear Confirm
                                                      |-- hết 300 s
                                                      v
                                            bật nguồn Pi + SCAN_START 900 s
                                                       |
                                                  ARM_DELAY 60 s
                                                      |
                                                      v
                                                   ACTIVE
```

- STM32 giữ quyền quyết định trạng thái và đếm timeout 300 giây.
- GPS chỉ xác nhận xe đã về bến; GPS không thể tự đánh thức MCU nếu cả GPS và
  MCU đều bị cắt nguồn.
- Rear Confirm hợp lệ có thể đánh thức từ `STANDBY` hoặc
  `WAIT_REAR_CONFIRM`. Nhánh timeout dùng nguồn `AUTO_REAR_TIMEOUT`.
- Camera/radar/servo gây cảnh báo chỉ được bật sau khi vào `ACTIVE`.
- Rear Confirm làm STM32 bật chân EN của load-switch Pi và giữ yêu cầu
  `SCAN_START` cho tới khi Pi boot, gửi READY và ACK lệnh. Nhánh GPS timeout cũng
  tạo một phiên quét tương tự.
- Hết 900 giây, `CLEAR` cho phép shutdown; `OCCUPIED` hoặc `FAULT` giữ Pi bật.
  `OCCUPIED` chỉ được xóa khi tài xế nhấn nút ACK PB10; STM32 gửi
  `SCAN_STOP(reason=OCCUPANCY_RESOLVED)` và Pi mới xóa latch bền. Detector bỏ
  sót vài frame, camera service thoát hoặc Pi reboot đều không tự xóa trạng thái.
- Nếu xe chạy lại, `WAIT_REAR_CONFIRM`, `ARM_DELAY` hoặc `ACTIVE` bị hủy về
  `STANDBY`.
- RPC và Raspberry Pi không có quyền giả lập Rear Confirm.

Khi `TRIPGUARD_CONNECTIVITY_OWNER_PI=1`, STM32 không khởi tạo USART2/A7670 hay
MQTT. STM32 chia telemetry JSON thành các chunk CRC + ACK/retry trên USART6;
Pi ghép dữ liệu, trộn kết quả camera, lưu SQLite và đẩy ThingsBoard qua 4G.

Geofence mặc định vẫn tắt vì chưa có tọa độ bến thật. Phải điền tọa độ rồi đặt
`TRIPGUARD_GEOFENCE_ENABLED=1` thì nhánh tự động chờ 5 phút mới hoạt động.

## Đấu dây cuối

| Thiết bị/tín hiệu | STM32F411 | Cấu hình |
|---|---|---|
| A7670/SIM7680 TX/RX | Pi GPIO5 RXD3 / GPIO4 TXD3 | `/dev/ttyAMA3`, 115200 8N1 |
| GPS NEO-M8L TX/RX | PA9 / PA10 | USART1, 9600 baud |
| Raspberry Pi RXD/TXD | PA11 TX / PA12 RX | USART6, 115200 8N1, DMA RX-to-IDLE |
| PI_SHUTDOWN_REQ | PA1 → Pi BCM23 | active HIGH, giữ ít nhất 1 s |
| PI_SHUTDOWN_ACK | Pi BCM26 → PB7 | active HIGH khi Linux đã poweroff |
| PI_POWER_EN | PB5 → EN load-switch | chỉ là logic 3,3 V, không cấp dòng cho Pi |
| Servo 360° signal | PA6 | TIM3 CH1 PWM |
| JQ8900 IO1 | PB0 | output |
| JQ8900 BUSY | PB12 | input |
| SOS | PB1 | active LOW, pull-up, EXTI |
| ACK tài xế | PB10 | active LOW, pull-up, EXTI |
| POWER_SENSE | PA8 | digital, EXTI hai cạnh |

### ESP32 Gateway → STM32 SPI2

| ESP32 Gateway | STM32F411 | Chiều |
|---|---|---|
| GPIO17 CS | PB9 NSS | ESP → STM |
| GPIO18 SCLK | PB13 SCK | ESP → STM |
| GPIO23 MOSI | PB15 MOSI | ESP → STM |
| GND | GND | chung mass |

SPI2 là slave RX-only, Mode 0, MSB-first, 8 bit. Không nối GPIO19/PB14 MISO,
không dùng GPIO16/PB8 READY và không dùng GPIO4/WAKE trong luồng mới.

### Nguồn Raspberry Pi

Không lấy nguồn Pi từ GPIO, chân 5 V hoặc bộ ổn áp của STM32. Dùng nguồn 5,1 V
tối thiểu 3 A đi qua **high-side load-switch** chịu 4–5 A; PB5 chỉ điều khiển
chân `EN`. Không ngắt GND. STM32, Pi và nguồn phải chung GND.

Để tránh cấp nguồn ngược qua PA11/PA12 khi Pi đã mất 5 V, phần cứng phải có
buffer/bus-switch 3,3 V hỗ trợ partial-power-down hoặc chân OE nối theo
`PI_POWER_EN`. Điện trở nối tiếp chỉ giới hạn dòng, không thay thế được cách ly.

Chuỗi tắt bắt buộc: `SCAN_STOP` → PA1 HIGH → Pi chạy `poweroff` → BCM26 HIGH →
STM32 chờ thêm 5 giây → PB5 LOW. Nếu thiếu ACK, timeout hoặc còn `OCCUPIED`,
STM32 hủy ngắt nguồn.

## Bảo vệ PA8

PA8 chỉ nhận 0–3,3 V từ optocoupler/comparator. Không nối nguồn xe 12–14,4 V
trực tiếp. Mức mặc định:

```text
HIGH = xe đang cấp nguồn/sạc
LOW  = nguồn xe đã ngắt
```

Nếu mạch cách ly đảo mức, đổi `TRIPGUARD_POWER_ACTIVE_LEVEL` trong
`Core/Inc/tripguard_config.h`; không đảo logic rải rác trong code.

## Frame SPI

Frame mới dài 12 byte:

| Byte | Nội dung |
|---:|---|
| 0 | Magic `0xA5` |
| 1 | Version `0x01` |
| 2 | Command `REAR_CONFIRM` (`0x10`) |
| 3 | Node ID |
| 4–5 | Sequence ID, little-endian |
| 6–7 | Value, little-endian |
| 8 | Flags |
| 9–10 | CRC-16/CCITT-FALSE của byte 0–8, little-endian |
| 11 | End `0x5A` |

Decoder STM32 vẫn nhận frame legacy `AA 55 ... CRC` 12 byte để không làm gãy
ESP32 đã nạp trước đó. Tuy nhiên giao thức vật lý mới là một chiều: Gateway gửi
cùng sequence ba lần; STM32 deduplicate, không trả ACK qua MISO.

## ThingsBoard

Telemetry bổ sung:

```text
systemState, systemArmed, activationSource, activationCountdown,
rearConfirmed, armRemainingSec, powerPresent, stationDetected, gpsFixValid,
espGatewayOnline, radarEnabled, cameraEnabled, cameraPerson,
piOnline, piReady, piRxFrames, piTxFrames, piCrcErrors, piUartErrors,
piLastPacketAgeMs, lastActivationReason, lastError
```

`activationCountdown` và `armRemainingSec` dùng đơn vị giây. RPC được hỗ trợ:

```text
setSystemArmed(false)
cancelArm
getSystemState
```

Kết quả chỉ được publish tới `v1/devices/me/rpc/response/<requestId>` sau khi
SupervisorTask thực sự áp dụng hoặc từ chối yêu cầu.

## Cảnh báo và modem

- Radar Remote trong ACTIVE: loa + servo + SMS, không gọi.
- Camera Pi chỉ hỗ trợ phát hiện/xác nhận và chẩn đoán, không tự tạo cảnh báo.
- SOS: loa + SMS + gọi.
- CellularTask là chủ duy nhất của A7670. Alert xin exclusive session, MQTT
  cleanup an toàn, và luôn resume/reconnect sau thành công, lỗi hoặc timeout.
- Cooldown/rearm phát hiện người của firmware cũ được giữ nguyên.

## Cấu hình trước khi triển khai

Geofence đang tắt:

```c
#define TRIPGUARD_GEOFENCE_ENABLED 0U
#define TRIPGUARD_STATION_LAT      0.0f
#define TRIPGUARD_STATION_LON      0.0f
```

Chỉ điền tọa độ bến thật và bật macro sau khi đo bán kính thực tế. Endpoint
MQTT/TCP của ThingsBoard cũng phải là endpoint còn hiệu lực; URL HTTPS
Cloudflare thông thường không thay thế được cổng MQTT TCP.

## Nạp firmware

Mở project này trong STM32CubeIDE 1.17, chọn `TripGuard_STM32F411_RTOS`, chạy
Clean/Build rồi Debug/Run qua ST-LINK. File build hiện tại:

```text
Debug\TripGuard_STM32F411_RTOS.elf
Debug\TripGuard_STM32F411_RTOS.hex
Debug\TripGuard_STM32F411_RTOS.bin
```
