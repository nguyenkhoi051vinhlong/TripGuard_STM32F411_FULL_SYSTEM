# Firmware ESP32-C3 Mini: REAR CONFIRM + SOS + JQ8900-16P + đèn

Remote nhận REAR CONFIRM từ gateway, phát âm thanh cảnh báo ngay tại
JQ8900-16P và gửi SOS về gateway qua Bluetooth Mesh. Dữ liệu MP3 không
được truyền qua Mesh.

Remote phải có thể nhận lệnh từ gateway nên dùng BLE Mesh Low Power Node,
Bluetooth modem sleep và automatic light sleep; không dùng deep sleep. Gateway
phải luôn cấp điện và bật tính năng Friend.

Trước khi được provision, firmware giữ light sleep tắt để quá trình quảng bá
và provision ổn định. Sau khi vào mạng Mesh, khóa này tự nhả và C3 mới dùng
automatic light sleep. Log nút hiển thị trực tiếp mức `GPIO7 LOW/HIGH`.

## Hành vi

- Khởi động: JQ8900 tắt, đèn tắt nếu không có SOS đang chờ ACK.
- Gateway gửi `REAR_CONFIRM`: bật JQ8900 và phát track 1. Nếu file ngắn,
  remote tự phát lại tới đủ 180 giây; sau đó dừng, tắt JQ8900 và trở lại
  light sleep.
- Bấm SOS: nếu REAR đang phát thì ngắt ngay; bật đèn, phát track 2 một lần
  và gửi `EVENT_SOS` lặp mỗi 2 giây tới khi gateway ACK đúng ID.
- SOS có ưu tiên cao nhất. REAR mới bị từ chối khi SOS còn chờ ACK và lệnh
  `REAR_STOP` không được phép cắt âm thanh SOS.
- JQ8900 hâm nóng 1 giây, chọn bộ nhớ FLASH và đặt âm lượng 20/30. Khi một
  cảnh báo kết thúc, firmware gửi `EVENT_ALERT_DONE` và ngắt nguồn module.
- ACK SOS chỉ dừng gửi lặp; không tắt đèn và không cắt ngang âm thanh.
- Gateway có thể gửi `REAR_CONFIRM`, `REAR_STOP`, `SOS_ACK`, `LAMP` và
  `GET_STATUS`.
- SOS chưa ACK được lưu NVS và khôi phục sau reset.

## Chân mặc định

Sửa tại `main/remote_config.h` nếu board khác:

| Chức năng | ESP32-C3 Mini |
|---|---:|
| LA38 chân 23, active-low | GPIO7 |
| JQ8900 TX -> ESP RX | GPIO4 |
| ESP TX -> JQ8900 RX | GPIO5 |
| JQ8900 BUSY | GPIO3 |
| EN mạch cấp nguồn JQ8900 | GPIO6 |
| Driver đèn cảnh báo | GPIO10 |

LA38 chân 24 nối GND. JQ8900 dùng nguồn 5V, GND chung và loa thụ động
8 ohm tối đa 3W nối giữa SPK+ và SPK-. Không nối một đầu loa xuống GND.
GPIO6 chỉ nối với chân EN của load switch 5V; không cấp JQ8900 trực tiếp.
Nếu chưa có load switch, có thể cấp JQ8900 từ 5V cố định nhưng sẽ tốn
dòng chờ.

JQ8900-16P dùng UART 9600-8-N-1, IO 3,3V. Khi cắm USB của JQ8900 vào máy
để chép MP3, module ở chế độ ổ đĩa và không nhận lệnh UART. Với firmware
này, nên xóa file thử rồi chép theo đúng thứ tự: file REAR là track 1,
file SOS là track 2.

## Build và nạp

Chạy `build.ps1` bằng ESP-IDF v6.1. Bộ file nạp được sao chép vào `firmware/`.
Giữ NVS khi cập nhật cùng layout để không mất khóa Mesh và SOS chờ ACK.

Provision remote, thêm AppKey index 0 và bind vendor model `0x1001`, CID `0x02E5`.
Giao thức byte/opcode nằm trong `PROTOCOL.md`.

Chưa kiểm thử trên JQ8900 và loa thật. Cần xác minh BUSY, thứ tự track,
dòng nguồn khi phát và độ trễ Mesh trên phần cứng thực tế.
