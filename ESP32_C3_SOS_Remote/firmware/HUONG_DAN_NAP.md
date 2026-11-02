# Bộ firmware ESP32-C3: REAR CONFIRM + SOS + JQ8900-16P + đèn

Build ngày 13/09/2026 bằng ESP-IDF v6.1. Target `esp32c3`, flash DIO 80MHz,
layout 4MB. Build thành công; app 945328 byte, còn trống 38% partition.
Toàn bộ file nhị phân trong thư mục này là bản JQ8900, không còn
logic cảm biến cũ.

## Cấu hình bản nhị phân

- GPIO7: nút SOS active-low.
- GPIO4: ESP RX <- JQ8900 TX.
- GPIO5: ESP TX -> JQ8900 RX.
- GPIO3: JQ8900 BUSY, HIGH khi đang phát.
- GPIO6: EN nguồn JQ8900 active-high.
- GPIO10: driver đèn active-high.
- JQ8900 UART 9600-8-N-1, volume 20/30.
- REAR CONFIRM phát/lặp track 1 trong 180 giây; SOS phát track 2 một lần.
- Gateway Mesh address `0x0001`, AppKey index 0, NetKey index 0.
- Giao thức Mesh version 3; xem `../PROTOCOL.md`.
- Khi chưa provision, C3 giữ light sleep tắt để Bluetooth quảng bá ổn định;
  sau khi provision mới tự chuyển sang LPN + automatic light sleep.

JQ8900 cấp 5V và loa thụ động 8 ohm tối đa 3W nối SPK+/SPK-.
GPIO6 chỉ điều khiển EN của load switch, không cấp nguồn module trực tiếp.

## Nạp qua USB native

Đưa C3 vào download mode: giữ BOOT, bấm/thả RST, chờ một giây rồi
thả BOOT. Thay `COMx` bằng cổng hiện tại của C3:

```powershell
python -m esptool --chip esp32c3 --port COMx --baud 115200 --before no-reset --after no-reset write-flash --flash-mode dio --flash-size 4MB --flash-freq 80m 0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0x10000 tripguard_sos_remote.bin
```

Nạp xong bấm RST. Khi cập nhật firmware, có thể giữ NVS để giữ lại khóa
Mesh và trạng thái SOS đang chờ ACK.

Sau khi nạp, JQ8900 phải có file REAR là track 1 và file SOS là track 2. Không
cắm USB của JQ8900 vào máy trong lúc thử phát vì module ở chế độ ổ đĩa
sẽ không nhận lệnh UART.

Chưa nạp/thử bản này trên JQ8900 và loa thật. Cần kiểm tra nguồn 5V
khi loa phát, tín hiệu BUSY và thứ tự file trên module.
