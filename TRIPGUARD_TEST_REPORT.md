# Báo cáo kiểm thử TripGuard STM32

Ngày cập nhật: 2026-09-14

Phạm vi gồm STM32F411, UART điều khiển phiên camera và handshake shutdown với
Raspberry Pi. Pi giữ SIM 4G và kết nối ThingsBoard.

## Kết quả tự động

| Hạng mục | Kết quả |
|---|---|
| Build STM32 Debug ELF | ĐẠT — 0 lỗi, 0 cảnh báo từ mã nguồn thay đổi |
| Dung lượng `text/data/bss` | 100704 / 488 / 62544 byte |
| Kiểm tra bất biến firmware | ĐẠT — 95/95 |
| USART6 115200 8N1, DMA RX-to-IDLE | ĐẠT tĩnh |
| CRC16, sequence, ACK và retry giới hạn | ĐẠT tĩnh |
| Telemetry JSON phân mảnh tối đa 128 byte/frame | ĐẠT tĩnh |
| STM32 không khởi tạo A7670/MQTT ở chế độ Pi gateway | ĐẠT tĩnh |

Chạy lại kiểm tra:

```powershell
python tools\verify_tripguard_invariants.py
```

## Luồng trạng thái hiện tại

```text
BOOT -> SELF_TEST -> STANDBY

STANDBY --Rear Confirm hợp lệ--> ARM_DELAY 60 s --> ACTIVE

STANDBY --GPS ở geofence, tốc độ thấp đủ 15 s--> WAIT_REAR_CONFIRM
WAIT_REAR_CONFIRM --Rear Confirm--> ARM_DELAY 60 s --> ACTIVE
WAIT_REAR_CONFIRM --hết 300 s--> ARM_DELAY 60 s --> ACTIVE

ARM_DELAY/ACTIVE --SCAN_COMPLETE CLEAR sau 900 s--> STANDBY -> shutdown Pi
ACTIVE --SCAN_COMPLETE OCCUPIED--> giữ ACTIVE và giữ nguồn Pi
OCCUPIED --tài xế nhấn ACK PB10--> SCAN_STOP RESOLVED -> STANDBY -> shutdown Pi
ARM_DELAY/ACTIVE --SCAN_COMPLETE FAULT--> giữ hệ thống thức, không cắt nguồn
```

`POWER_SENSE` chỉ dùng chẩn đoán và xác định xe chạy lại; mất nguồn không có
quyền tự ARM. Camera Pi không có quyền ARM/DISARM. Camera chỉ tạo cảnh báo khi
STM32 đã ở `ACTIVE`.

## Mười tám kịch bản nghiệm thu hệ thống

Các mục dưới đây cần chạy lại trên bo thật sau khi nạp ELF mới.

| # | Thao tác | Kết quả bắt buộc |
|---:|---|---|
| 1 | Boot STM32 khi Pi/ESP32 chưa online | `SELF_TEST -> STANDBY`; không chờ SIM/MQTT, không bật phát hiện người. |
| 2 | Gửi một Rear Confirm hợp lệ ở `STANDBY` | Vào `ARM_DELAY`, đếm 60 về 0 rồi `ACTIVE`; `activationSource=MANUAL_REAR_CONFIRM`. |
| 3 | Gửi ba Rear Confirm cùng node/sequence | Chỉ xử lý một lần; duplicate tăng; bộ đếm 60 giây không khởi động lại. |
| 4 | GPS mất fix, dữ liệu cũ hoặc xe chạy nhanh | Không đặt `stationDetected`, không vào `WAIT_REAR_CONFIRM`. |
| 5 | GPS trong geofence, tốc độ thấp liên tục 15 giây | Vào `WAIT_REAR_CONFIRM`, `rear_confirm_wait_remaining_s` đếm từ 300. |
| 6 | Không bấm Rear Confirm trong 300 giây | `rear_confirm_timeout_count` tăng, vào `ARM_DELAY`; nguồn kích hoạt là `AUTO_REAR_TIMEOUT`. |
| 7 | Bấm Rear Confirm khi đang chờ 300 giây | Chuyển ngay sang `ARM_DELAY`; không chờ hết 300 giây. |
| 8 | POWER_SENSE đã HIGH ổn định và GPS báo xe chạy lại | Nếu đang chờ/đếm/ACTIVE thì về `STANDBY`, tắt radar/camera detection. |
| 9 | Pi gửi camera person trước `ACTIVE`, rồi gửi lại khi `ACTIVE` | Trước ACTIVE chỉ cập nhật chẩn đoán. Trong ACTIVE phát loa và gửi một alert UART; frame lặp không cảnh báo lại. |
| 10 | ESP Gateway gửi radar PERSON đủ 2 giây trong `ACTIVE` | Phát loa, quay servo, gửi alert UART cho Pi; không tạo cuộc gọi. |
| 11 | Ngắt Pi hoặc bỏ ACK telemetry rồi nối lại | `pi_online=0` sau 3 giây; retry tối đa 3 lần, không treo RTOS; khi Pi READY lại thì telemetry tiếp tục. |
| 12 | Nhấn SOS PB1 | Phát loa và gửi alert loại SOS qua UART cho Pi; driver ACK dừng âm thanh, chống dội nút. |
| 13 | Rear Confirm khi Pi chưa READY | STM32 giữ trạng thái mong muốn và gửi `SCAN_START(session,900)` ngay khi Pi READY; Pi ACK sau khi camera service khởi động thành công. |
| 14 | Để phiên camera chạy đủ 900 giây và không có người | Pi gửi `SCAN_COMPLETE=CLEAR`; STM32 về `STANDBY` và chỉ xin shutdown nếu phần cứng đã được xác nhận. |
| 15 | Camera xác nhận người trong phiên | Pi lưu JPEG/hàng đợi ThingsBoard, gửi PERSON và `SCAN_COMPLETE=OCCUPIED`; camera service dừng nhưng latch vẫn còn qua reboot, không shutdown. |
| 16 | Khi đang OCCUPIED, nhấn ACK PB10 | STM32 gửi `SCAN_STOP reason=1`; Pi xóa `/var/lib/tripguard/camera-state.json`, gửi CAMERA_CLEAR; hệ thống mới được về STANDBY. |
| 17 | Camera lỗi hoặc trả FAULT | STM32 giữ hệ thống thức, `last_error=PI_CAMERA_SCAN_FAULT`; không ngắt Pi để tránh âm tính giả. |
| 18 | Với load-switch thật, STM32 yêu cầu shutdown | PA1 HIGH đủ 1 s; Linux poweroff; BCM26 HIGH; STM32 chờ thêm 5 s rồi mới cho PB5 LOW. Thiếu ACK hoặc OCCUPIED thì tuyệt đối không cắt nguồn. |

## Live Expressions STM32

```text
tripguard_system_state
tripguard_activation_source
tripguard_state_started_ms
tripguard_supervisor_rear_confirmed
tripguard_station_detected
tripguard_supervisor_gps_fix_valid
tripguard_rear_confirm_wait_remaining_s
tripguard_rear_confirm_timeout_count
tripguard_supervisor_radar_enabled
tripguard_supervisor_camera_enabled

pi_online
pi_ready
pi_camera_person
pi_camera_confidence_permille
pi_last_packet_age_ms
pi_rx_frame_count
pi_tx_frame_count
pi_crc_error_count
pi_uart_error_count
pi_duplicate_count
pi_rx_overflow_count
pi_telemetry_queued_count
pi_telemetry_sent_count
pi_telemetry_drop_count
pi_scan_session_id
pi_scan_command_sent_count
pi_scan_command_ack_count
pi_scan_complete_count
pi_scan_last_result

tripguard_camera_person
tripguard_camera_confidence_permille
tripguard_camera_alert_armed
tripguard_camera_alert_count
tripguard_camera_last_alert_ms
tripguard_person_detected

tripguard_gateway_radar_healthy
tripguard_gateway_radar_person
tripguard_gateway_radar_distance_cm
tripguard_gateway_radar_last_sequence
tripguard_gateway_radar_duplicate_count
tripguard_rear_confirm_count
tripguard_rear_duplicate_count
tripguard_gateway_valid_rx_count
tripguard_gateway_invalid_rx_count
tripguard_spi_transaction_count
tripguard_spi_hal_error_count
tripguard_sos_count
tripguard_driver_ack_count
tripguard_alert_error_count

tripguard_power_state
tripguard_pi_shutdown_ack
tripguard_shutdown_request_count
tripguard_shutdown_success_count
tripguard_shutdown_timeout_count
tripguard_shutdown_cancel_count
tripguard_power_hw_inhibit_count
```

Giá trị `tripguard_system_state`:

- `0`: SELF_TEST
- `1`: STANDBY
- `2`: ARM_DELAY
- `3`: ACTIVE
- `4`: FAULT
- `5`: WAIT_REAR_CONFIRM

## Điều kiện để test nhánh GPS tự động

Firmware vẫn để `TRIPGUARD_GEOFENCE_ENABLED=0` nhằm tránh kích hoạt sai vì
chưa có tọa độ thật. Trước khi thử kịch bản 5–7, điền đúng
`TRIPGUARD_STATION_LAT`, `TRIPGUARD_STATION_LON`, bán kính và đổi macro thành
`1`, sau đó build và nạp lại.

## Dây STM32 cần giữ nguyên

| Kết nối | STM32 |
|---|---|
| Pi RX/TX | PA11 USART6_TX / PA12 USART6_RX, 115200 8N1 |
| GPS | PA9 USART1_TX / PA10 USART1_RX, 9600 |
| ESP32 Gateway SPI | PB9 NSS / PB13 SCK / PB15 MOSI |
| POWER_SENSE | PA8, chỉ nhận 0–3.3 V đã cách ly/bảo vệ |
| Servo / loa / SOS / ACK | PA6 / PB0 / PB1 / PB10 |

STM32 và Pi phải chung GND; không nối 5 V giữa hai bo. PB5 chỉ nối chân EN của
high-side load-switch 5,1 V/ít nhất 3 A. Trước khi hoàn thành kịch bản 18, giữ
`TRIPGUARD_POWER_HARDWARE_READY=0` và `TRIPGUARD_USE_STOP_MODE=0`.
