# Remote REAR/SOS/JQ8900 - giao thức gateway

ESP32-C3 là Vendor Server CID `0x02E5`, Model `0x1001`, một element. Gateway
mặc định địa chỉ `0x0001`, NetKey index 0, AppKey index 0. Gateway phải
provision node, bind AppKey và bật Friend; remote chạy Low Power Node.

## Gateway gửi command

Opcode `ESP_BLE_MESH_MODEL_OP_3(0x10, 0x02E5)`, payload 8 byte little-endian:

| Byte | Nội dung |
|---|---|
| 0 | Version = 3 |
| 1 | Command: 1 REAR_CONFIRM, 2 SOS_ACK, 3 LAMP, 4 GET_STATUS, 5 REAR_STOP |
| 2-5 | uint32 transaction ID |
| 6 | LAMP: 0/1; command khác: 0 |
| 7 | Reserved = 0 |

`REAR_CONFIRM` phải dùng ID khác 0, tăng dần và lưu bền qua reboot. Remote
không chạy lại ID nhỏ hơn hoặc bằng ID REAR cuối đã chấp nhận. Khi nhận lệnh,
remote phát/lặp track 1 đến đúng 180 giây. REAR mới bị từ chối trong lúc SOS
còn chờ ACK; gateway phải thử lại sau.

`SOS_ACK` phải dùng đúng ID của `EVENT_SOS`. ACK chỉ dừng gửi lặp; không tắt
đèn và không cắt track SOS. Sau khi đã nhận ACK, gateway có thể gửi `LAMP=0`.
Remote từ chối tắt đèn khi SOS vẫn chưa ACK. `REAR_STOP` chỉ dừng REAR và
không bao giờ dừng track SOS.

Ví dụ REAR CONFIRM ID 1: `03 01 01 00 00 00 00 00`.

Ví dụ ACK SOS ID 2: `03 02 02 00 00 00 00 00`.

## Remote gửi event

Opcode `ESP_BLE_MESH_MODEL_OP_3(0x11, 0x02E5)`, payload 8 byte:

| Byte | Nội dung |
|---|---|
| 0 | Version = 3 |
| 1 | Event: 1 STATUS, 2 SOS, 3 ALERT_DONE |
| 2-5 | ID của command REAR hoặc SOS |
| 6 | Flags trạng thái |
| 7 | Detail |

Flags byte 6:

- bit0: JQ8900 đang được cấp nguồn
- bit1: SOS chưa ACK
- bit2: đèn đang bật
- bit3: BUSY đang HIGH
- bit4: đã thiết lập Friendship
- bit5: lỗi audio/BUSY timeout
- bit6: REAR đang hoạt động
- bit7: âm thanh SOS đang hoạt động

Với `STATUS` và `SOS`, byte 7 có nibble cao là loại cảnh báo (`0` idle,
`1` REAR, `2` SOS); nibble thấp là trạng thái audio (`0` off, `1` power-up,
`2` chờ BUSY, `3` playing).

Với `ALERT_DONE`, nibble cao là loại cảnh báo vừa kết thúc (`1` REAR,
`2` SOS); nibble thấp là kết quả (`0` hoàn tất, `1` bị REAR_STOP hoặc REAR
mới thay thế, `2` REAR bị SOS ưu tiên, `15` lỗi JQ8900/BUSY timeout).

SOS gửi lặp mỗi 2 giây tới khi nhận ACK đúng ID. Gateway chống xử lý trùng
bằng cặp `(node address, SOS ID)` và chỉ ACK sau khi đã tiếp nhận sự kiện.
STATUS gửi mỗi giây khi audio hoạt động và mỗi phút khi chờ.

Gói Mesh không chứa file hay luồng âm thanh. JQ8900 giải mã file MP3/WAV đã
lưu cục bộ trên module remote: track 1 cho REAR và track 2 cho SOS.
