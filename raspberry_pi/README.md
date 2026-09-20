# TripGuard UART – Raspberry Pi 4 và STM32F411

`tripguard_uart.py` là tiến trình duy nhất mở `/dev/serial0`. Ứng dụng camera
và lệnh kiểm thử gửi sự kiện qua socket cục bộ
`/run/tripguard-uart/control.sock`; chúng không được tự mở UART.

Raspberry Pi không gửi ARM, DISARM, TOGGLE hoặc giả lập Rear Confirm. Việc kích
hoạt hệ thống vẫn chỉ đến từ nút Rear Confirm của ESP32 Remote.

## 1. Đấu dây

| STM32F411 | Raspberry Pi 4 | Chân vật lý Pi |
|---|---|---:|
| PA11 / USART6_TX | GPIO15 / RXD | 10 |
| PA12 / USART6_RX | GPIO14 / TXD | 8 |
| GND | GND | 6 |

- UART dùng logic **3.3 V**, 115200 baud, 8N1.
- Không nối 5 V giữa hai bo.
- GPIO14 và GPIO15 không được dùng làm serial login console.
- Dịch vụ shutdown cũ dùng BCM23/BCM26 nên có thể chạy song song.

## 2. Bật UART trên Raspberry Pi

```bash
sudo raspi-config
```

Chọn `Interface Options` → `Serial Port`:

- `Would you like a login shell ... over serial?` → **No**.
- `Would you like the serial port hardware to be enabled?` → **Yes**.

Khởi động lại và kiểm tra:

```bash
sudo reboot
readlink -f /dev/serial0
ls -l /dev/serial0
systemctl status serial-getty@serial0.service --no-pager
```

`serial-getty@serial0.service` phải ở trạng thái disabled/inactive. Nếu chưa:

```bash
sudo systemctl disable --now serial-getty@serial0.service
```

## 3. Cài đặt

```bash
sudo apt update
sudo apt install -y python3-serial
sudo usermod -aG dialout "$USER"

cd /duong/dan/toi/project/raspberry_pi
python3 tripguard_uart.py self-test

sudo install -d -m 0755 /opt/tripguard
sudo install -m 0755 tripguard_uart.py /opt/tripguard/tripguard_uart.py
sudo install -m 0644 tripguard-uart.service /etc/systemd/system/tripguard-uart.service
sudo install -d -m 0750 /etc/tripguard
sudo install -m 0640 tripguard-gateway.env.example /etc/tripguard/gateway.env
sudo systemctl daemon-reload
sudo systemctl enable --now tripguard-uart.service
```

Mở `/etc/tripguard/gateway.env`, thay `PASTE_DEVICE_ACCESS_TOKEN_HERE` bằng
access token của **device TripGuard** trên ThingsBoard. Không dùng tài khoản
admin và không ghi token thật vào source code.

Xem trạng thái và log:

```bash
sudo systemctl status tripguard-uart.service --no-pager
sudo journalctl -u tripguard-uart.service -f
```

Không cần dừng `tripguard-shutdown.service`; hai dịch vụ dùng tài nguyên khác
nhau.

## 4. Kiểm thử camera từ terminal

Giữ `tripguard-uart.service` đang chạy rồi mở terminal khác:

```bash
sudo python3 /opt/tripguard/tripguard_uart.py ready
sudo python3 /opt/tripguard/tripguard_uart.py status
sudo python3 /opt/tripguard/tripguard_uart.py person --confidence 0.95
sudo python3 /opt/tripguard/tripguard_uart.py clear
```

Kết quả `OK queued` chỉ có nghĩa là lệnh đã vào hàng đợi UART, chưa có nghĩa là
STM32 đã ACK. Xem ACK, retry và trạng thái STM32 bằng `journalctl`. `person`
trước trạng thái `ACTIVE` chỉ được cập
nhật chẩn đoán, không được kích loa, servo, SMS hoặc cuộc gọi.

Để kiểm thử kèm một ảnh JPEG đã nén nhỏ hơn 80 KiB:

```bash
sudo python3 /opt/tripguard/tripguard_uart.py person \
  --confidence 0.95 --image /duong/dan/anh-thu.jpg
```

Gateway đọc ảnh ngay khi nhận lệnh, kiểm tra JPEG, mã hóa Base64 và lưu vào
SQLite. Vì vậy ứng dụng camera có thể xóa file tạm sau khi nhận `OK queued`.

## 5. Camera được STM32 điều khiển theo phiên

### Cài camera nhưng không tự quét khi Pi khởi động

Với chương trình nhận diện tại `/home/kt/TripGuardAI`, cài service:

```bash
sudo install -m 0755 tripguard_camera_endtrip.py \
  /home/kt/TripGuardAI/tripguard_camera_endtrip.py
sudo install -m 0644 tripguard-camera.service \
  /etc/systemd/system/tripguard-camera.service
sudo systemctl daemon-reload
sudo systemctl disable --now tripguard-camera.service
```

`tripguard-camera.service` chỉ được `tripguard-uart.service` khởi động khi STM32
gửi `STM_MSG_SCAN_START`. Payload gồm `sessionId` và thời lượng; cấu hình chuẩn
là 900 giây tính cả thời gian camera warm-up. `STM_MSG_SCAN_STOP` dừng phiên hiện
tại. Khi AI xác nhận người, script gửi `PI_MSG_CAMERA_PERSON` cho STM32 qua
`tripguard-uart.service`. Trước khi có người được xác nhận, trạng thái live có
thể trở về `PI_MSG_CAMERA_CLEAR` khi vắng người ổn định. Sau khi đã xác nhận,
`OCCUPIED` được khóa an toàn cho toàn bộ phiên quét và không bị xóa bởi vài
khung hình detector bỏ sót. Camera đồng thời tạo đúng một JPEG 640×360 có khung
nhận diện, confidence và thời gian cho phiên đó. Ảnh được nén dưới 75 KiB rồi
chuyển cho gateway; không chụp hoặc gửi ảnh trên từng frame.
Nếu Camera Module 3 không báo khóa autofocus nhưng ảnh đã vượt qua kiểm tra độ
sáng, độ nét và ổn định phơi sáng, chương trình khóa vị trí lens cuối và tiếp
tục quét. Ảnh hoặc phơi sáng không đạt vẫn tạo `FAULT`; fallback này không bỏ
qua bộ kiểm tra chất lượng ảnh.
Service giới hạn camera ở 8 FPS để Pi 4 không lặp khởi động do vượt ngưỡng bảo
vệ 75 °C. Thử nghiệm thực tế vẫn xác nhận người và tạo evidence, trong khi
nhiệt độ ổn định dưới ngưỡng và không xuất hiện cờ throttling.

Hết 900 giây, camera gửi `PI_MSG_SCAN_COMPLETE`. Kết quả `CLEAR` cho phép STM32
bắt đầu shutdown Pi nếu mạch nguồn đã được xác nhận an toàn. `OCCUPIED` và
`FAULT` giữ Pi hoạt động; `OCCUPIED` không tự bị xóa và phải có xác nhận xử lý
riêng trước khi hệ thống được phép ngắt nguồn. Xác nhận chuẩn là nút tài xế PB10:
STM32 gửi `SCAN_STOP` với reason `OCCUPANCY_RESOLVED`; Pi chỉ khi đó mới ghi
`person=false` vào `/var/lib/tripguard/camera-state.json` và gửi
`PI_MSG_CAMERA_CLEAR` bền về STM32.

Kiểm tra:

```bash
sudo systemctl status tripguard-camera.service --no-pager
sudo journalctl -u tripguard-camera.service -f
```

Ứng dụng camera chỉ cần gửi lệnh tới socket cục bộ. Có thể import hàm sẵn có:

```python
from tripguard_uart import send_control_command

SOCKET = "/run/tripguard-uart/control.sock"

# Gọi sau khi camera và model đã sẵn sàng.
send_control_command(SOCKET, "ready")

# confidence dùng thang 0..1000.
send_control_command(SOCKET, "person 927")
send_control_command(SOCKET, "clear")
```

Lệnh nội bộ có ảnh dùng dạng `person 927 /tmp/evidence.jpg`. CLI công khai nên
dùng tùy chọn `--image` để tự quote đường dẫn an toàn.

Đặt `/opt/tripguard` vào `PYTHONPATH`, hoặc chép phần camera vào cùng package.
Không gọi lệnh `person` liên tục cho từng frame; chỉ gửi khi trạng thái phát
hiện đổi từ clear → person và gửi `clear` khi person → clear.

## 6. Khung UART nhị phân

| Offset | Nội dung |
|---:|---|
| 0..1 | Magic `54 50` |
| 2 | Version `01` |
| 3 | Message type |
| 4 | Flags: bit 0 ACK-required, bit 1 retry |
| 5..6 | Sequence uint16 little-endian |
| 7..8 | Payload length uint16 little-endian, tối đa 128 |
| 9.. | Payload |
| cuối | CRC16-CCITT-FALSE little-endian |

CRC dùng init `0xFFFF`, polynomial `0x1021`, xor-out `0x0000`, tính từ byte
version đến hết payload.

Message type:

| Giá trị | Hướng | Tên |
|---:|---|---|
| `0x01` | Pi → STM32 | `PI_MSG_HEARTBEAT` |
| `0x02` | Pi → STM32 | `PI_MSG_READY` |
| `0x03` | Pi → STM32 | `PI_MSG_STATUS_REQUEST` |
| `0x10` | Pi → STM32 | `PI_MSG_CAMERA_PERSON` |
| `0x11` | Pi → STM32 | `PI_MSG_CAMERA_CLEAR` |
| `0x12` | Pi → STM32 | `PI_MSG_SCAN_COMPLETE` |
| `0x7F` | Pi → STM32 | `PI_MSG_ACK` |
| `0x81` | STM32 → Pi | `STM_MSG_HEARTBEAT` |
| `0x82` | STM32 → Pi | `STM_MSG_STATUS` |
| `0x83` | STM32 → Pi | `STM_MSG_EVENT` |
| `0x84` | STM32 → Pi | `STM_MSG_TELEMETRY_CHUNK` |
| `0x85` | STM32 → Pi | `STM_MSG_ALERT_REQUEST` |
| `0x86` | STM32 → Pi | `STM_MSG_SCAN_START` |
| `0x87` | STM32 → Pi | `STM_MSG_SCAN_STOP` |
| `0xFE` | STM32 → Pi | `STM_MSG_ACK` |
| `0xFF` | STM32 → Pi | `STM_MSG_NACK` |

Payload hiện dùng:

- Heartbeat: uptime milliseconds, `uint32` little-endian.
- READY: một byte `gateway_ready=1`; camera có thể đang tắt giữa hai phiên.
- CAMERA_PERSON: confidence `uint16` theo thang 0..1000.
- CAMERA_CLEAR và STATUS_REQUEST: rỗng.
- STATUS: `state:uint8`, `rearConfirmed:uint8`, `armRemainingSec:uint16`.
- EVENT: `eventCode:uint8`, `state:uint8`, `value:uint16` little-endian.
- TELEMETRY_CHUNK: `messageId:uint32`, `totalLength:uint16`,
  `offset:uint16`, sau đó là tối đa 120 byte JSON. Từng chunk đều được ACK;
  Pi chỉ lưu khi ghép đủ toàn bộ JSON.
- ALERT_REQUEST: `alertKind:uint8`, `state:uint8`, `value:uint16`.
- SCAN_START: `sessionId:uint32`, `durationSec:uint16`; mặc định 900 giây.
- SCAN_STOP: `sessionId:uint32`, `reason:uint8`.
  Reason: `0=STANDBY`, `1=OCCUPANCY_RESOLVED`, `2=SHUTDOWN`.
- SCAN_COMPLETE: `sessionId:uint32`, `result:uint8`; `0=CLEAR`,
  `1=OCCUPIED`, `2=FAULT`, `3=STOPPED`.
- ACK/NACK: payload dài đúng 3 byte gồm `ackedType:uint8` và
  `ackedSequence:uint16` little-endian. Sequence trên header là sequence riêng
  của chính frame ACK/NACK.

READY, STATUS_REQUEST, CAMERA_PERSON, CAMERA_CLEAR và STM_MSG_EVENT yêu cầu
ACK. Khi hết timeout, daemon gửi lại cùng sequence tối đa ba lần. Frame trùng
được ACK lại nhưng không xử lý lần hai. Sau ba lần thất bại, trạng thái bền mới
nhất của `READY` và `CAMERA_PERSON/CAMERA_CLEAR` được giữ trong RAM và tự xếp
hàng lại sau 10 giây. Vì vậy STM32 vừa được cắm hoặc reset sẽ tự nhận lại trạng
thái camera; `STATUS_REQUEST` thủ công thì không lặp vô hạn.

## 7. Cài handshake shutdown

```bash
cd /duong/dan/toi/project/raspberry_pi
sudo sh ./install_shutdown_handshake.sh
```

Installer chỉ cài và enable service. Trước khi reboot, xác nhận BCM23 đang LOW
và đã có điện trở kéo xuống. Sau reboot, kiểm tra:

```bash
systemctl is-active tripguard-shutdown.service
pinctrl get 23
pinctrl get 26
```

BCM23 phải LOW khi bình thường. Listener chỉ shutdown sau khi BCM23 HIGH liên
tục một giây. BCM26 do kernel `gpio-poweroff` kéo HIGH khi đã vào đường tắt máy.

## 8. Kiểm tra nhanh khi không kết nối được

```bash
python3 /opt/tripguard/tripguard_uart.py self-test
sudo fuser -v /dev/serial0
sudo journalctl -u tripguard-uart.service -n 100 --no-pager
```

Chỉ `tripguard_uart.py` được sở hữu `/dev/serial0`. Nếu `/dev/serial0` trỏ sang
Bluetooth UART thay vì UART chân GPIO, kiểm tra lại cấu hình `enable_uart=1`
và serial console trong `raspi-config`.

## 9. Kết nối Internet bằng SIM 4G UART3

Luồng mới dùng hai UART độc lập:

- `/dev/serial0`, 115200: STM32 ↔ Pi.
- `/dev/ttyAMA3`, 115200: Pi ↔ SIMCom A7670/SIM7680.

Module phải trả `OK` ổn định ở 115200 trước khi bật PPP. Tốc độ 9600 chỉ phù
hợp kiểm tra AT, không đủ cho telemetry liên tục hoặc ảnh. Sau đó cài PPP:

```bash
sudo apt update
sudo apt install -y ppp
sudo install -m 0755 tripguard_modem_prepare.py \
  /opt/tripguard/tripguard_modem_prepare.py
sudo install -m 0600 tripguard-4g.chat /etc/chatscripts/tripguard-4g
sudo install -m 0600 tripguard-4g.peer /etc/ppp/peers/tripguard-4g
sudo install -m 0755 tripguard-ppp-dns-route /etc/ppp/ip-up.d/0100tripguard-dns-route
sudo install -m 0644 tripguard-4g.service /etc/systemd/system/tripguard-4g.service
sudo systemctl daemon-reload
sudo systemctl enable --now tripguard-4g.service
```

Không bật `persist` trong peer PPP. Khi modem rớt link hoặc trở về baud mặc
định, `pppd` phải thoát để systemd chạy lại `tripguard_modem_prepare.py`, khóa
modem về 115200 rồi mới quay số lại.

Kiểm tra đúng là Pi đã có đường mạng 4G:

```bash
ip address show ppp0
ip route
ping -I ppp0 -c 3 1.1.1.1
sudo journalctl -u tripguard-4g.service -n 100 --no-pager
```

`tripguard_uart.py` không mở `/dev/ttyAMA3`; `pppd` là tiến trình duy nhất sở
hữu UART modem. Bộ gửi ThingsBoard dùng kết nối mạng của Linux. Khi mất sóng,
JSON đã ghép đủ được giữ trong `/var/lib/tripguard/telemetry.db` và gửi lại sau
khi `ppp0` phục hồi. Evidence camera có priority 100, cảnh báo STM32 priority
50 và telemetry định kỳ priority 0, nên bằng chứng khẩn cấp được gửi trước phần
telemetry tồn đọng.

## 9. Telemetry bằng chứng camera

Một sự kiện người được gửi tới endpoint ThingsBoard `/api/v1/<token>/telemetry`
với timestamp phía Pi và các key:

- `eventType=PERSON_DETECTED`
- `eventId`, `eventSeq`
- `personDetected=true`
- `confidence`, `piCameraConfidencePermille`
- `evidenceContentType=image/jpeg`, `evidenceBytes`
- `evidenceImage=data:image/jpeg;base64,...`

Không in `evidenceImage` ra terminal vì chuỗi rất dài. Kiểm tra hàng đợi chỉ
bằng loại sự kiện, priority và độ dài payload. Trên dashboard, tạo Custom Latest
Values widget với datasource là device TripGuard và data key `evidenceImage`,
sau đó dùng giá trị mới nhất làm thuộc tính `src` của thẻ `img`. Dashboard hiện
tại dùng `HTML widgets` → `Markdown/HTML card` với mẫu:

```html
<div style="height:100%;display:flex;flex-direction:column;align-items:center;justify-content:center;padding:12px;background:#111827;color:#fff;overflow:hidden;">
  <div style="font-size:20px;font-weight:700;margin-bottom:10px;">Bằng chứng phát hiện người</div>
  <img src="${evidenceImage}" alt="TripGuard person evidence" style="display:block;max-width:100%;max-height:calc(100% - 45px);object-fit:contain;border-radius:6px;background:#000;" />
</div>
```
