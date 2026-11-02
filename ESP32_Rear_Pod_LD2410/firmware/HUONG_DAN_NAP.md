# Nạp firmware ESP32-C3 Rear Pod

## 1. Chân đã chốt

| Chức năng | ESP32-C3 Mini |
|---|---:|
| LA38 chân 23 | GPIO3 |
| LA38 chân 24 | GND |
| LD2410C OUT | GPIO0 qua 1 kΩ |
| LD2410C TX | GPIO4 qua 220 Ω |
| LD2410C RX | GPIO5 qua 220 Ω |
| EN load switch nguồn radar | GPIO10, kéo xuống GND bằng 100 kΩ |

LD2410C dùng nguồn 5 V sau load switch. ESP32-C3, radar và nguồn phải chung
GND. Không đưa 5 V vào bất kỳ GPIO nào của C3.

## 2. Vào bootloader

Thông thường esptool tự reset được board. Nếu không kết nối được:

1. Nối GPIO9 với GND hoặc giữ nút BOOT.
2. Nhấn rồi nhả RESET, hoặc rút/cắm USB trong khi vẫn giữ BOOT.
3. Nhả BOOT sau khi máy tính nhận cổng COM.
4. Đóng Serial Monitor trước khi nạp.

## 3. Lệnh nạp

Mở PowerShell tại thư mục `firmware`, thay `COM21` bằng cổng của board:

```powershell
python -m esptool --chip esp32c3 --port COM21 --baud 115200 write-flash --flash-mode dio --flash-size 4MB --flash-freq 80m 0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0x10000 tripguard_rear_pod.bin
```

Sau khi nạp xong, nhấn RESET hoặc rút/cắm lại USB. Nếu board vẫn ở bootloader,
tháo dây GPIO9–GND trước khi reset.

Khi chuyển từ một firmware Mesh khác sang Rear Pod, nên xóa flash một lần để
không giữ credential/model cũ:

```powershell
python -m esptool --chip esp32c3 --port COM21 erase-flash
```

Sau đó nạp đủ ba file bằng lệnh trên.

## 4. Xem log

```powershell
python -m serial.tools.miniterm COM21 115200
```

Thoát miniterm bằng `Ctrl+]`. Log khởi động đúng có dòng gần giống:

```text
READY: button GPIO3, radar OFF, waiting for Mesh
```

Bấm LA38 một lần sẽ có:

```text
REAR CONFIRM id=...; 5-minute scan started
Radar ON; scan duration=300000 ms
```

Hết 5 phút:

```text
Radar OFF; scan done reason=0; light sleep allowed
```

## 5. Chế độ ngủ

- Chưa provision Mesh: C3 cố ý không light sleep để dễ ghép mạng.
- Đã provision: C3 chạy Mesh Low Power Node, Bluetooth modem sleep và automatic
  light sleep.
- Khi chờ: nguồn LD2410C tắt, UART radar tắt và GPIO4/GPIO5 ở high-impedance.
- GPIO3 mức LOW đánh thức C3; firmware bắt xung LOW đầu tiên, bỏ qua
  dội tiếp điểm và chỉ mở lại sau khi HIGH ổn định 80 ms.
- Trong 5 phút quét: firmware khóa light sleep để không mất frame UART 256000.
- Hết 5 phút: tắt radar, nhả khóa và tự quay lại automatic light sleep.

Đây không phải deep sleep. Deep sleep sẽ tắt Bluetooth và Rear Pod không nhận
được lệnh/ACK từ gateway.

## 6. Bluetooth Mesh

Rear Pod dùng:

- Company ID `0x02E5`;
- Vendor Model ID `0x1002`;
- AppKey index `0`;
- gateway mặc định `0x0001`.

Gateway phải provision node, bind AppKey vào Model `0x1002` và bật Friend Node.
Nếu chưa provision, bấm nút vẫn bật radar nhưng event Mesh sẽ được giữ và thử
gửi lại sau.
