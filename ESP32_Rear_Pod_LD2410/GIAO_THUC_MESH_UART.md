# Giao thức Rear Pod Mesh và SPI gateway

Tài liệu này chốt giao tiếp cho chuỗi:

```text
Rear Pod ESP32-C3 Mini <-- Bluetooth Mesh --> Gateway ESP32-WROOM-32
                                             |
                                             `-- SPI một chiều --> STM32
```

## 1. Tham số Bluetooth Mesh

| Tham số | Giá trị |
|---|---:|
| Company ID | `0x02E5` |
| Rear Pod Vendor Model ID | `0x1002` |
| SOS/JQ8900 Remote Model ID | `0x1001` |
| Gateway unicast address mặc định | `0x0001` |
| NetKey index | `0` |
| AppKey index | `0` |
| Protocol version | `3` |

Gateway provision Rear Pod, bind AppKey vào vendor model `0x1002` và bật tính
năng Friend. Rear Pod bật Low Power Node. Model ID riêng giúp gateway không
nhầm gói Rear Pod với gói của remote SOS/JQ8900.

## 2. Gateway gửi lệnh cho Rear Pod

Opcode `ESP_BLE_MESH_MODEL_OP_3(0x10, 0x02E5)`, payload đúng 8 byte,
little-endian:

| Byte | Nội dung |
|---:|---|
| 0 | Version = `3` |
| 1 | Command |
| 2–5 | `uint32_t transaction_id` |
| 6 | Parameter |
| 7 | Reserved = `0` |

Command:

| Giá trị | Tên | Parameter |
|---:|---|---|
| 1 | `START_SCAN` | Số phút quét; `0` nghĩa là mặc định 5 phút |
| 2 | `STOP_SCAN` | `0` |
| 3 | `ACK_EVENT` | `0` |
| 4 | `GET_STATUS` | `0` |
| 5 | `CLEAR_FAULT` | `0` |

`START_SCAN` và `STOP_SCAN` phải có ID khác 0 và tăng dần. Rear Pod lưu ID lệnh
cuối cùng trong NVS; gói trùng được ACK/trả status nhưng không chạy lại hành
động. `ACK_EVENT` phải mang đúng ID của event đang chờ ACK.

Ví dụ bắt đầu quét mặc định, ID 1:

```text
03 01 01 00 00 00 00 00
```

## 3. Rear Pod gửi sự kiện

Opcode `ESP_BLE_MESH_MODEL_OP_3(0x11, 0x02E5)`, payload đúng 8 byte:

| Byte | Nội dung |
|---:|---|
| 0 | Version = `3` |
| 1 | Event |
| 2–5 | `uint32_t transaction_id` |
| 6 | Flags |
| 7 | Detail |

Event:

| Giá trị | Tên | Ý nghĩa |
|---:|---|---|
| 1 | `STATUS` | Trạng thái hiện tại |
| 2 | `REAR_CONFIRM` | Nút đã được bấm hợp lệ |
| 3 | `RADAR_UPDATE` | Trạng thái phát hiện đã đổi hoặc bản tin định kỳ |
| 4 | `SCAN_DONE` | Cửa sổ quét kết thúc |
| 5 | `FAULT` | Lỗi cảm biến, nút, nguồn hoặc Mesh |

Flags byte 6:

- bit0: nguồn LD2410C đang bật;
- bit1: cửa sổ quét đang chạy;
- bit2: LD2410C báo có mục tiêu;
- bit3: có mục tiêu chuyển động;
- bit4: có mục tiêu đứng yên;
- bit5: đã thiết lập Mesh Friendship;
- bit6: lỗi LD2410C;
- bit7: dự phòng, luôn bằng `0`. Trạng thái thô của nút không đưa vào gói radar
  để tránh tạo lưu lượng Mesh khi tiếp điểm rung.

Detail byte 7:

- `RADAR_UPDATE`: khoảng cách mục tiêu gần nhất theo đơn vị 0,1 m, bão hòa ở
  255; `0xFF` nếu khoảng cách không hợp lệ;
- `SCAN_DONE`: `0` hết 5 phút, `1` do lệnh `STOP_SCAN`, `2` lỗi radar;
- `FAULT`: `1` radar không khởi động, `2` UART radar timeout, `4` mất
  Friendship;
- event khác: `0`.

## 4. Luồng REAR CONFIRM

1. C3 bắt xung LOW đầu tiên, khóa các xung dội và chỉ mở lại sau
   khi nút đã HIGH ổn định 80 ms.
2. C3 tạo ID mới, lưu ID/event vào NVS, bật radar và bắt đầu timer 5 phút.
3. C3 gửi `REAR_CONFIRM` tới gateway. Nếu chưa có ACK, gửi lại cùng ID sau mỗi
   2 giây; không tạo ID mới cho cùng lần bấm.
4. Gateway khử trùng theo `(mesh_source_address, event, transaction_id)`.
5. Gateway chuyển sự kiện quan trọng tới STM32 ba lần bằng SPI một chiều.
6. Khi ba giao dịch SPI hoàn tất, Gateway gửi `ACK_EVENT` về C3.
7. Gateway đồng thời có thể gửi command `REAR_CONFIRM` sang remote
   SOS/JQ8900 Model `0x1001` để remote đó phát track cảnh báo 3 phút.
8. Dù ACK tới trễ, timer quét 5 phút vẫn tính từ lúc nút được xác nhận; ACK chỉ
   xác nhận giao tiếp, không dừng radar.

Do SPI không có MISO hoặc chân ACK, Gateway chỉ xác nhận driver đã phát dữ liệu;
không thể biết STM32 đã xử lý thành công hay chưa. STM32 phải khử bản sao trùng.

## 5. Nhịp gửi trạng thái

- Khi chờ: `STATUS` mỗi 60 giây.
- Khi quét: `RADAR_UPDATE` gửi ngay khi trạng thái có mục tiêu hoặc trạng thái
  lỗi thay đổi; khoảng cách và chuyển động được cập nhật tối đa mỗi 1 giây.
- `REAR_CONFIRM`, `SCAN_DONE` và `FAULT` là sự kiện quan trọng, gửi lặp tới khi
  nhận đúng `ACK_EVENT` hoặc tới giới hạn retry đã cấu hình.
- Gateway phải chấp nhận gói trùng nhưng chỉ kích hoạt STM32/loa một lần.

## 6. Frame SPI một chiều giữa gateway và STM32

ESP32-WROOM là master; STM32F411 là SPI2 slave. Cấu hình Mode 0, MSB first,
1 MHz. Chân: GPIO18→PB13/SCK, GPIO23→PB15/MOSI, GPIO17→PB9/SPI2_NSS (AF5)
và GND chung.
Mỗi lần NSS xuống LOW chứa đúng một frame có cấu trúc:

| Byte | Trường |
|---:|---|
| 0 | Magic `0xA5` |
| 1 | Protocol version `0x01` |
| 2 | STM32 event |
| 3 | Node ID `0x02` |
| 4–5 | Sequence, little-endian (16 bit thấp của Mesh transaction ID) |
| 6–7 | Value, little-endian |
| 8 | Flags, hiện đặt bằng `0` |
| 9–10 | CRC16 low/high |
| 11 | End `0x5A` |

CRC dùng CRC-16/CCITT-FALSE: polynomial `0x1021`, initial value `0xFFFF`, không
reflect, xor-out `0x0000`; tính trên byte 0–8.

Ánh xạ event STM32: `0x10 REAR_CONFIRM`, `0x11 PERSON`, `0x12 RADAR_CLEAR`,
`0x13 RADAR_HEALTH`. `REAR_CONFIRM` có Value=`1`; khoảng cách PERSON dùng cm.
Sự kiện quan trọng được phát ba lần, cách nhau 50 ms. STM32 chỉ kích hoạt hành
động một lần cho cùng bộ `(node_id, event, sequence)`.

## 7. Quy tắc gateway

- WROOM chạy liên tục; không light sleep/deep sleep vì nó là Mesh Friend và cầu
  SPI của hệ thống.
- Duy trì bảng node gồm địa chỉ, model ID, transaction cuối, Friendship và thời
  điểm status cuối.
- Không dựa vào một gói duy nhất: kiểm tra version, đúng chiều opcode, đúng độ
  dài, model ID và transaction ID.
- Sau reset, khôi phục transaction counter và transaction đã ACK Mesh để tránh
  kích hoạt trùng.
- STM32 kiểm tra `A5`, byte cuối `5A` và CRC trước khi xử lý mỗi frame 12 byte
  nhận dưới một lần NSS xuống LOW.
- Watchdog task gateway không được bị chặn bởi việc gửi Mesh hoặc SPI.

## 8. Light sleep của Rear Pod

Rear Pod không tự gọi deep sleep. Khi rảnh, firmware để ESP-IDF power management
tự đưa CPU vào light sleep trong lúc BLE controller duy trì lịch nhận của LPN.
Friend Queue trên WROOM giữ message cho tới lần LPN poll tiếp theo. Vì vậy lệnh
từ gateway có thể có độ trễ bằng poll timeout; thông số Friendship phải được
chọn sao cho cân bằng giữa pin và thời gian phản hồi.

Radar luôn bị ngắt nguồn trong trạng thái chờ. Khi radar đang quét, C3 vẫn dùng
modem sleep lúc có thể nhưng ưu tiên đọc UART đúng hạn; hết 5 phút mới trở lại
mức tiết kiệm điện cao nhất.
