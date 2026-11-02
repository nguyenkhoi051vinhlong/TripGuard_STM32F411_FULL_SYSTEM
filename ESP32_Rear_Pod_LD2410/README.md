# ESP32-C3 Mini Rear Pod: REAR CONFIRM + LD2410C + Bluetooth Mesh

Rear Pod đặt ở cuối xe, dùng board ESP32-C3 Mini/SuperMini 4 MB để đọc nút
`REAR CONFIRM` và radar HLK-LD2410C. Rear Pod không có đèn, còi hoặc JQ8900.
Dữ liệu được truyền không dây bằng
Bluetooth Mesh tới gateway ESP32-WROOM-32; gateway nối trực tiếp STM32 bằng
UART TTL 3,3 V. Thiết kế demo này không dùng RS485.

## Luồng hoạt động

- Chưa provision: C3 thức để gateway tìm và provision.
- Đã provision, chờ: LD2410C tắt nguồn; C3 chạy BLE Mesh Low Power Node,
  Bluetooth modem sleep và automatic light sleep.
- Bấm REAR CONFIRM một lần: C3 gửi sự kiện về gateway, bật LD2410C và
  quét liên tục 5 phút.
- Trong 5 phút: gửi trạng thái radar và lỗi về gateway; nút không trực tiếp
  kết luận xe trống.
- Hết 5 phút: dừng radar, tắt nguồn radar và quay lại light sleep.
- Gateway ESP32-WROOM-32 luôn bật, chạy Friend Node và chuyển dữ liệu sang
  STM32 qua UART.

Không dùng deep sleep vì radio Bluetooth sẽ tắt và gateway không thể gửi lệnh
hay ACK tới Rear Pod. “Tight sleep” trong trao đổi được hiểu là light sleep.

## Tài liệu

- [Thiết kế phần cứng](./THIET_KE_PHAN_CUNG.md)
- [Đấu dây bản demo](./LAP_RAP_DEMO.md)
- [Đặc tả Bluetooth Mesh và UART gateway](./GIAO_THUC_MESH_UART.md)
- [Hướng dẫn nạp firmware](./firmware/HUONG_DAN_NAP.md)

## Firmware Rear Pod

Firmware ESP-IDF cho ESP32-C3 đã có trong thư mục `main/` và đã build thành
`firmware/tripguard_rear_pod.bin`. Firmware bao gồm:

- GPIO3 đánh thức bằng nút LA38 chân 23–24; bắt ngay xung LOW đầu tiên,
  khóa dội tiếp điểm và chỉ mở lại sau khi HIGH ổn định 80 ms;
- điều khiển nguồn LD2410C bằng GPIO10;
- parser UART LD2410C 256000 baud trên GPIO4/GPIO5 và tín hiệu OUT GPIO0;
- quét đúng 5 phút, báo mục tiêu/lỗi qua Mesh rồi tắt nguồn radar;
- Vendor Model `0x1002`, transaction ID, ACK, retry và lưu NVS;
- Mesh LPN, Bluetooth modem sleep và automatic light sleep khi chờ.

Firmware gateway ESP32-WROOM-32 là dự án riêng và chưa nằm trong thư mục này.
