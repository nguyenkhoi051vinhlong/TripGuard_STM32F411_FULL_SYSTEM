# TripGuard Gateway ESP32-WROOM-32

Gateway luôn bật, làm BLE Mesh Provisioner + Friend và gửi sự kiện một chiều
tới STM32 qua SPI. Gateway tự tìm node có UUID `TGRP` (Rear Pod) hoặc `TGR3` (SOS
Remote), provision, thêm AppKey index 0 và bind Vendor Model tương ứng.

## Chân SPI tới STM32F411

```text
ESP32-WROOM GPIO18 / SCK  ---> STM32 PB13 / SPI2_SCK
ESP32-WROOM GPIO23 / MOSI ---> STM32 PB15 / SPI2_MOSI
ESP32-WROOM GPIO17 / CS   ---> STM32 PB9  / SPI2_NSS (AF5)
ESP32-WROOM GND ---------- STM32 GND
```

ESP32 là SPI master, STM32 là SPI2 slave: Mode 0, MSB first, 1 MHz, mức logic
3,3 V. Không nối MISO vì đường truyền chỉ từ Gateway sang STM32.

Gateway chuyển event Mesh sang frame 12 byte `A5 ... CRC_L CRC_H 5A` đúng với
`tripguard_protocol.c` của STM32. Mỗi lần CS xuống LOW là một frame hoàn chỉnh.
Event quan trọng được phát 3 lần, cách nhau 50 ms; STM32 chống xử lý trùng bằng
bộ `(node_id, event, sequence)`.

Do liên kết một chiều không có ACK từ STM32, Gateway ACK Mesh sau khi driver SPI
đã phát thành công 3 bản sao. Nếu cần xác nhận STM32 đã thật sự nhận và xử lý,
phải bổ sung MISO hoặc một chân ACK riêng.

## Nạp COM15

```powershell
python -m esptool --chip esp32 --port COM15 --baud 460800 write-flash --flash-mode dio --flash-size 4MB --flash-freq 40m 0x10000 tripguard_gateway.bin
python -m serial.tools.miniterm COM15 115200
```

Không `erase-flash` khi chỉ cập nhật ứng dụng vì thao tác đó xóa mạng Mesh đã
provision.
