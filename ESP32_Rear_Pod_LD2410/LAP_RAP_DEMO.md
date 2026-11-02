# Lắp ráp demo Rear Pod Bluetooth Mesh

## Linh kiện

- board ESP32-C3 Mini/SuperMini 4 MB;
- HLK-LD2410C;
- nút LA38 có cặp tiếp điểm thường hở 23–24;
- load switch 5 V active-high, ví dụ TPS22919 hoặc module tương đương;
- nguồn 5 V/1 A ổn định.

Không cần MAX3485, THVD1450, SM712, termination 120 Ω hoặc dây A/B RS485.
Rear Pod cũng không lắp đèn, còi hoặc JQ8900; các phần đó thuộc remote SOS.

## Cấp nguồn

| Nguồn | Kết nối |
|---|---|
| 5 V ổn định | Chân `5V` của ESP32-C3 Mini và VIN load switch radar |
| GND nguồn | GND chung ESP32-C3, radar và nút |
| 3V3 trên board | Mạng điện trở nút và các tín hiệu logic dòng nhỏ |

Không cấp 5 V vào chân `3V3`. Khi nạp bằng USB, bản demo nên tháo nguồn 5 V
ngoài để tránh cấp ngược giữa hai nguồn.

## Đấu LD2410C

| LD2410C/load switch | ESP32-C3 Mini / nguồn |
|---|---|
| Load switch VIN | 5 V |
| Load switch VOUT | LD2410C VCC |
| Load switch EN | GPIO10; thêm 100 kΩ kéo xuống GND |
| LD2410C GND | GND chung |
| LD2410C TX | GPIO4 qua 220 Ω |
| LD2410C RX | GPIO5 qua 220 Ω |
| LD2410C OUT | GPIO0 qua 1 kΩ |

Đặt 470 µF song song 10 µF và 100 nF tại VOUT gần radar. UART LD2410C dùng
256000 baud, 8N1. Trước khi tắt load switch, firmware phải dừng UART và đưa
GPIO4/GPIO5 về input floating để tránh cấp ngược radar qua chân tín hiệu.

## Đấu nút REAR CONFIRM LA38 chân 23–24

```text
LA38 chân 23 ------------- GPIO3
LA38 chân 24 ------------- GND
GPIO3 ------ 10 kΩ ------- 3V3
GPIO3 ------ 100 nF ------ GND
```

Chân 23 và 24 không phân cực nên có thể đảo cho nhau. Firmware cấu hình GPIO3
`INPUT_PULLUP`: nhả là HIGH, nhấn là LOW. Xung LOW đầu tiên tạo
REAR CONFIRM; firmware bỏ qua các xung dội và chỉ nhận lần nhấn kế tiếp
sau khi HIGH ổn định 80 ms. Không đưa 5 V vào GPIO3.

## Gateway ESP32-WROOM-32 nối STM32

```text
ESP32-WROOM GPIO17 / TX2 ----> STM32 PA10 / USART1_RX
ESP32-WROOM GPIO16 / RX2 <---- STM32 PA9  / USART1_TX
ESP32-WROOM GND -------------- STM32 GND
```

Hai bên dùng UART TTL 3,3 V, ví dụ 115200 baud 8N1. Không nối TX với TX.
Gateway phải được cấp điện liên tục và bật BLE Mesh Friend Node.

## Trình tự thử

1. Chỉ cấp ESP32-C3, kiểm tra 3,3 V và nạp firmware qua USB.
2. Provision Rear Pod vào gateway; xác nhận Friendship đã thiết lập.
3. Kiểm tra GPIO3: nhả nút là HIGH, giữ nút là LOW.
4. Cắm radar, kiểm tra frame UART và chân OUT.
5. Bấm REAR CONFIRM một lần: gateway phải nhận event, radar bật và quét.
6. Xác nhận radar tự tắt sau 5 phút và C3 trở lại Mesh LPN + light sleep.
7. Rút radar hoặc làm BUSY/UART lỗi: gateway/STM32 phải nhận FAULT, không kết
   luận xe trống.
8. Chạy thử liên tục tối thiểu 2 giờ và kiểm tra mất Mesh, reset, ACK trễ.
