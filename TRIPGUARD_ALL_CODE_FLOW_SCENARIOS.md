# TOÀN BỘ KỊCH BẢN VÀ LUỒNG CHẠY TRIPGUARD

Cập nhật theo firmware STM32 hiện tại ngày 2026-09-11. Tài liệu mô tả phần
đang có trong project; các kiểm thử phần cứng vẫn phải chạy trên thiết bị thật.

## 1. Cấu hình và quyền sở hữu tài nguyên

| Thành phần | Tài nguyên | Chủ sở hữu |
|---|---|---|
| GPS NEO-M8L | USART1 PA9/PA10, 9600 | GpsTask / gps.c |
| SIM A7670 | USART2 PA2/PA3, 9600 | CellularTask / a7670.c |
| Raspberry Pi 4 | USART6 PA11/PA12, 115200 8N1 | PiLinkTask / pi_link.c |
| Pi RX DMA | DMA2 Stream1 Channel5, circular RX-to-IDLE | PiLink |
| ESP32 Gateway | SPI2 PB9/PB13/PB15, Mode 0, 1 MHz | GatewayLinkTask |
| Radar LD2410 | Trong ESP32 Remote, không nối UART STM32 | ESP32 Remote |
| Servo | PA6 / TIM3 CH1, PWM 50 Hz | SafetyTask |
| JQ8900 | PB0; BUSY PB12 | AudioTask |
| SOS / ACK tài xế | PB1 / PB10, active-low | SafetyTask |
| Power sense | PA8, tín hiệu đã hạ về 0–3,3 V | Supervisor/PowerManager |
| Debug | PA13 SWDIO, PA14 SWCLK | ST-LINK |

PA11/PA12 không dùng USB OTG khi PiLink hoạt động. Raspberry Pi và STM32 phải
chung GND; không nối 5 V giữa hai bo.

## 2. Luồng tổng quát

```text
Boot STM32
  -> SELF_TEST không blocking
  -> đủ SIM/network + MQTT publish + Radar Health + Pi READY/heartbeat
  -> STANDBY
  -> Rear Confirm từ LA38 qua BLE Mesh và SPI
  -> ARM_DELAY 60 giây
  -> ACTIVE

ESP32 Remote + LD2410
  -> BLE Mesh
  -> ESP32 Gateway
  -> SPI2 một chiều
  -> STM32 kiểm frame/CRC/sequence
  -> SafetyTask
  -> loa + servo + SMS khi Radar PERSON hợp lệ trong ACTIVE
  -> TelemetryTask/CellularTask
  -> A7670 MQTT
  -> ThingsBoard

Camera CSI
  -> daemon Python trên Pi
  -> PiLink UART nhị phân
  -> STM32 cập nhật cameraPerson/chẩn đoán
```

Camera Pi không có quyền ARM/DISARM/TOGGLE và không tự kích hoạt loa, servo,
SMS hoặc cuộc gọi.

## 3. Các task FreeRTOS

| Task | Ưu tiên | Chức năng |
|---|---:|---|
| CellularTask | Realtime | A7670, SMS/call, MQTT, RPC, publish |
| SafetyTask | High | Radar Remote, SOS, ACK, servo, cảnh báo |
| GatewayLinkTask | Above normal | Nhận và kiểm tra SPI một chiều |
| PowerManagerTask | Above normal | Power sense, rail, shutdown/wake |
| TripGuardSupervisorTask | Above normal | SELF_TEST và state machine logic |
| PiLinkTask | Above normal | DMA RX, parser, TX queue và retry UART Pi |
| AudioTask | Above normal | JQ8900 |
| GpsTask | Normal | NMEA/fix/tốc độ |
| TelemetryTask | Normal | Kích hoạt telemetry định kỳ |
| HealthTask | Low | Đồng bộ biến chẩn đoán |
| defaultTask | Low | Heartbeat RTOS |

Callback UART được tập trung trong `main.c`; ISR chỉ chuyển byte/sự kiện và
đánh thức task, không chạy MQTT/servo/loa hoặc xử lý parser dài.

## 4. Nhóm A — Boot và SELF_TEST

### A01 — Boot bình thường

GPIO, DMA, SPI2, TIM3, USART1/2/6 được khởi tạo; GPS, A7670, JQ8900, servo,
MQTT và 11 task bắt đầu. Supervisor vào `SELF_TEST`, detection bị khóa.

### A02 — Đủ bốn điều kiện

Các bit lần lượt là SIM/network, MQTT connected và có publish mới, Radar
Health từ Gateway, Pi online và READY. Khi `selfTestMask=15`, state chuyển
`SELF_TEST -> STANDBY`. Self-test không phát SMS, không gọi và không quay
servo.

### A03 — Thiếu một thành phần

State giữ `SELF_TEST`, `lastError=SELF_TEST_WAITING`; firmware queue
telemetry và thử lại mỗi 5 giây. Không có vòng chờ vô hạn, các task khác vẫn
chạy.

### A04 — Pi hoặc radar đến muộn

Khi heartbeat/READY Pi hoặc HEALTH radar xuất hiện sau boot, bit tương ứng được
cập nhật ở vòng self-test kế tiếp; không cần reset STM32.

### A05 — Lỗi khởi tạo nghiêm trọng

HAL init lỗi gọi `Error_Handler()`. Thiếu queue/task bắt buộc đưa supervisor
vào `FAULT`. Stack overflow đặt `tripguard_stack_overflow_fault=1` rồi dừng
an toàn.

## 5. Nhóm B — State machine và Rear Confirm

### B01 — Rear Confirm hợp lệ

Chỉ khi đang STANDBY, Rear Confirm hợp lệ từ node Remote mới đặt
`rearConfirmed=true` và chuyển sang `ARM_DELAY`. Countdown mặc định 60 giây.

### B02 — Hết ARM_DELAY

Sau 60 giây, state chuyển thẳng `ACTIVE`; radar/camera detection được enable.
Không còn trạng thái WARMUP.

### B03 — Rear Confirm trước STANDBY

Rear Confirm đến trong SELF_TEST/FAULT không ARM hệ thống. Sự kiện vẫn được
ghi nhận ở tầng transport/chẩn đoán nhưng state không vượt qua self-test.

### B04 — Rear Confirm trùng

Ba bản sao cùng node/sequence chỉ commit một lần.
`tripguard_rear_duplicate_count` tăng nhưng countdown không khởi động lại.

### B05 — RPC yêu cầu ARM

`setSystemArmed(true)` luôn bị từ chối với `rear_confirm_required`. GPS,
geofence, PA8 và Raspberry Pi không thể thay thế Rear Confirm.

### B06 — Hủy/Disarm

`cancelArm` chỉ hủy khi đang ARM_DELAY. `setSystemArmed(false)` có thể đưa
STANDBY/ARM_DELAY/ACTIVE về STANDBY; SELF_TEST và FAULT từ chối thay đổi.

### B07 — Xe chạy lại

Nếu PA8 đã HIGH đủ chuẩn, GPS còn mới và tốc độ vượt ngưỡng, ARM_DELAY hoặc
ACTIVE trở về STANDBY. Đây là điều kiện hủy, không phải nguồn ARM.

## 6. Nhóm C — ESP32 Remote, BLE Mesh và SPI một chiều

### C01 — Một lần nhấn LA38

Remote debounce GPIO32 active-low, tạo sequence 16 bit và gửi Rear Confirm qua
Mesh. Gateway chuyển cùng sự kiện qua SPI; STM32 kiểm CRC/node/type/value,
commit một lần, queue telemetry và gửi sự kiện sang PiLink.

### C02 — Frame SPI mới

Frame dài 12 byte: magic `A5`, version, command, node, sequence LE, value LE,
flags, CRC16/CCITT-FALSE LE và end `5A`. Lệnh hỗ trợ:
`REAR_CONFIRM=0x10`, `PERSON=0x11`, `CLEAR=0x12`,
`RADAR_HEALTH=0x13`.

### C03 — Tương thích frame cũ

Decoder STM32 vẫn nhận frame 12 byte bắt đầu `AA 55` để không làm gãy ESP32
đã nạp trước đó. Mọi frame vẫn phải đúng version, trường dữ liệu và CRC.

### C04 — Cơ chế một chiều

Chỉ dùng CS/SCLK/MOSI; không dùng MISO, READY hoặc ACK end-to-end. Gateway gửi
ba bản sao cùng sequence, STM32 deduplicate. Remote/Gateway không được chờ ACK
từ STM32 trong cấu hình này.

### C05 — Frame sai hoặc queue đầy

Sai magic/version/end/CRC/type/value làm tăng invalid count và không vào
SafetyTask. Queue/ring đầy tăng overflow/drop counter; không ghi đè ngoài
buffer.

### C06 — Sequence quay vòng

Sequence là uint16; `65535 -> 0` được chấp nhận. Dedup dùng lịch sử sequence
gần nhất, không dùng phép so sánh số học làm hỏng tại điểm wrap.

### C07 — Gateway/radar mất liên lạc

Quá 3 giây không có Radar Health hợp lệ làm radar unhealthy/offline và xóa
trạng thái person cũ. Self-test không qua hoặc ACTIVE không tạo cảnh báo mới.

### C08 — Firmware ESP32 chưa hỗ trợ sự kiện mới

Nếu Gateway/Remote chỉ gửi REAR_CONFIRM mà chưa gửi HEALTH/PERSON/CLEAR,
STM32 sẽ ở SELF_TEST vì thiếu Radar Health. Phải nạp đúng firmware ESP32 tương
ứng trước nghiệm thu toàn hệ thống.

## 7. Nhóm D — PiLink UART

### D01 — Frame hợp lệ

Frame là `54 50 01 type flags seq_le len_le payload crc_le`; payload tối đa
128 byte, CRC16-CCITT tính từ version đến hết payload.

### D02 — DMA phân mảnh hoặc gộp frame

RX-to-IDLE DMA đưa byte vào ring tĩnh. Parser task xử lý được một frame chia
nhiều callback, nhiều frame trong một callback và byte rác trước magic.

### D03 — Length/CRC sai

Length lớn hơn 128 hoặc CRC sai bị bỏ; parser tìm lại magic để nhận frame kế
tiếp. `pi_crc_error_count`/counter lỗi tăng, không ảnh hưởng CellularTask.

### D04 — Heartbeat và reconnect

Pi gửi heartbeat mỗi 1 giây. Quá 3 giây không nhận, STM32 đặt
`pi_online=0`, `pi_ready=0` và xóa camera person cũ. Frame hợp lệ mới đưa
link online lại.

### D05 — READY

Daemon gửi `PI_MSG_READY` payload 1 byte sau khi camera/app sẵn sàng. READY
và heartbeat là điều kiện Pi của SELF_TEST.

### D06 — Status request

`PI_MSG_STATUS_REQUEST` không payload; STM32 trả `STM_MSG_STATUS` gồm
state, rearConfirmed và countdown uint16 LE.

### D07 — Camera person/clear

`PI_MSG_CAMERA_PERSON` mang confidence permille uint16 LE, tối đa 1000.
`PI_MSG_CAMERA_CLEAR` không payload. Trước ACTIVE và cả trong ACTIVE, các
message này chỉ cập nhật chẩn đoán/xác nhận; không sở hữu state hoặc cảnh báo.

### D08 — ACK, retry và duplicate

READY, STATUS_REQUEST, CAMERA_PERSON và CAMERA_CLEAR là message quan trọng.
Receiver ACK theo type+sequence; duplicate được ACK lại nhưng không thực thi
lần hai. STM event cần ACK retry tối đa 3 lần.

### D09 — UART/DMA lỗi

Callback lỗi chỉ tăng `pi_uart_error_count`, yêu cầu restart và đánh thức
PiLinkTask. Task abort/clear/restart RX-to-IDLE; SIM/MQTT không bị chặn.

### D10 — Daemon Pi mất serial

Python đóng/mở lại `/dev/serial0`, không busy-loop, dùng TX queue thread-safe.
Lệnh terminal hỗ trợ `ready`, `person`, `clear`, `status`.

## 8. Nhóm E — Radar, servo, loa và nút

### E01 — PERSON trước ACTIVE

Gateway state/counter vẫn cập nhật nhưng SafetyTask khóa mọi side effect:
không loa, servo, SMS hoặc gọi.

### E02 — PERSON hợp lệ trong ACTIVE

Radar Remote healthy và PERSON liên tục 2 giây đặt person detected, quay servo
về phía radar 700 ms rồi dừng, phát track cảnh báo, queue telemetry và yêu cầu
SMS. Không gọi điện.

### E03 — CLEAR

Mất hiện diện dưới 8 giây chưa xóa detection. CLEAR liên tục 8 giây xóa person
và quay servo về home 700 ms rồi dừng.

### E04 — Cooldown/rearm

Người đứng liên tục không tạo SMS lặp vô hạn. Cảnh báo mới chỉ được rearm sau
khoảng bảo vệ cấu hình và điều kiện clear.

### E05 — SOS

Nhấn PB1 hợp lệ tăng SOS count, phát loa, gửi SMS rồi gọi điện. Debounce/latch
ngăn giữ nút từ boot hoặc dội phím tạo nhiều sự kiện.

### E06 — ACK tài xế

PB10 active-low qua debounce sẽ tăng ACK count, xóa SOS pending và đưa lệnh
dừng audio. ACK không thể hủy AT command SMS/cuộc gọi đã bắt đầu.

### E07 — Audio/servo lỗi

Queue đầy hoặc rail bị tắt được ghi counter; task không block vô hạn. Khi rail
bật và epoch đổi, driver liên quan được khởi tạo lại.

## 9. Nhóm F — SIM, MQTT, telemetry và RPC

### F01 — A7670 lên mạng

FSM đi qua AT, SIM, network và PDP đến READY. CellularTask là chủ duy nhất của
USART2 và phiên MQTT.

### F02 — Mất mạng/PDP/MQTT

FSM cleanup, backoff và retry. Sự kiện cục bộ vẫn xử lý; telemetry nằm trong
RAM và không có flash spool dài hạn.

### F03 — Radar alert và SOS dùng modem

Radar yêu cầu SMS; SOS yêu cầu SMS và call. Alert xin phiên exclusive, tạm
dừng MQTT an toàn rồi resume/reconnect ở mọi nhánh thành công/lỗi/timeout.

### F04 — Telemetry

JSON giữ các key cũ và thêm:
`piOnline`, `piReady`, `piRxFrames`, `piTxFrames`, `piCrcErrors`,
`piUartErrors`, `piLastPacketAgeMs`, `cameraPerson`, `systemState`,
`rearConfirmed`, `armRemainingSec`.

### F05 — Telemetry dồn

Queue có giới hạn; sự kiện được gộp khi cần. Nếu payload vượt buffer, firmware
gửi payload lỗi rút gọn thay vì ghi tràn.

### F06 — RPC

RPC audio/radar/SOS/telemetry/servo cũ vẫn hoạt động theo điều kiện an toàn.
Servo RPC chỉ được chạy trong ACTIVE. RPC ARM không thể thay Rear Confirm.

### F07 — Endpoint ThingsBoard sai

Web HTTPS mở được không chứng minh MQTT TCP hoạt động. Host/port/tunnel sai làm
MQTT retry; phải cấu hình endpoint raw MQTT ổn định cho A7670.

## 10. Nhóm G — GPS và power

### G01 — GPS

GpsTask parse NMEA không blocking. Fix mới dùng cho telemetry/chẩn đoán và
điều kiện xe chạy lại; fix cũ/timeout không được dùng để ARM.

### G02 — PowerManager

PowerManager quản lý BUS sense, rail, graceful shutdown Pi và wake. Đây là FSM
nguồn riêng, không thay quyền sở hữu state logic của Supervisor.

### G03 — Khóa phần cứng

`TRIPGUARD_POWER_HARDWARE_READY=0` và `TRIPGUARD_USE_STOP_MODE=0` mặc định
ngăn cắt rail/STOP thật trên bench. Logical standby vẫn hoạt động.

### G04 — Graceful shutdown Pi

PA1 yêu cầu Pi shutdown; Pi ACK qua PB7; STM32 chờ settle rồi mới được phép
cắt rail khi khóa phần cứng đã bật và mạch đã nghiệm thu. Đây là đường power
riêng, không thay PiLink PA11/PA12.

### G05 — Race wake/sleep

Trước STOP, PowerManager kiểm tra lại BUS/wake/SPI; nếu có sự kiện mới thì hủy
sleep và rearm ngoại vi. Sau wake, epoch buộc các task phục hồi driver.

## 11. Nhóm H — Tình huống kết hợp

| Tình huống | Kết quả bắt buộc |
|---|---|
| Mất Internet, nhấn Rear Confirm | State/commit cục bộ vẫn chạy; telemetry chờ MQTT |
| Mất Internet, radar PERSON trong ACTIVE | Loa/servo/SMS tùy còn mạng GSM; MQTT retry |
| SOS khi đang publish | SOS được ưu tiên qua phiên modem exclusive |
| Pi offline trong ACTIVE | Camera chẩn đoán mất; radar Remote/SOS vẫn hoạt động |
| Radar offline nhưng Pi thấy người | Không tự tạo cảnh báo; ghi cameraPerson |
| Pi gửi ARM giả | Loại message/NACK; state không đổi |
| SPI và Pi UART cùng lúc | Hai task/ngoại vi riêng, không tranh UART SIM |
| Sequence duplicate | Counter duplicate tăng, business count không tăng |
| CRC lỗi rồi frame tốt | Parser resync; frame tốt sau đó được xử lý |
| Reset/mất điện | Queue RAM mất; sequence nguồn phát giảm lặp nhờ NVS phía ESP |

## 12. Live Expressions

```text
tripguard_system_state
tripguard_supervisor_rear_confirmed
tripguard_rear_confirm_node
tripguard_rear_confirm_sequence
tripguard_rear_confirm_count
tripguard_rear_duplicate_count
tripguard_gateway_valid_rx_count
tripguard_gateway_invalid_rx_count
tripguard_spi_transaction_count
tripguard_spi_hal_error_count
tripguard_gateway_radar_healthy
tripguard_gateway_radar_person
tripguard_gateway_radar_distance_cm
tripguard_gateway_radar_last_sequence
pi_online
pi_ready
pi_camera_person
pi_last_packet_age_ms
pi_rx_frame_count
pi_tx_frame_count
pi_crc_error_count
pi_uart_error_count
pi_duplicate_count
pi_rx_overflow_count
tripguard_mqtt_connected
tripguard_publish_count
tripguard_sms_sent_count
tripguard_call_started_count
```

`tripguard_system_state`: 0=SELF_TEST, 1=STANDBY, 2=ARM_DELAY, 3=ACTIVE,
4=FAULT.

## 13. Tiêu chí nghiệm thu

1. Build Debug 0 error, 0 warning liên quan thay đổi.
2. Pi daemon self-test parser/CRC/fragment/noise/wrap/ACK đạt.
3. SELF_TEST chỉ qua khi đủ bốn bit.
4. Chỉ Rear Confirm chuyển STANDBY sang ARM_DELAY.
5. Ba bản sao cùng sequence chỉ tăng business count một lần.
6. Radar PERSON chỉ cảnh báo trong ACTIVE.
7. Camera Pi không tự ARM hoặc cảnh báo.
8. Pi mất heartbeat hơn 3 giây được phát hiện mà MQTT vẫn chạy.
9. CRC/UART lỗi Pi tự phục hồi.
10. ThingsBoard nhận đủ telemetry mới.

## 14. Giới hạn hiện tại

- Cần firmware ESP32 Remote/Gateway thực sự phát HEALTH/PERSON/CLEAR đúng mã;
  nếu chưa có, self-test sẽ chờ Radar Health.
- Ảnh camera không đi qua UART STM32; PiLink chỉ mang event/metadata.
- Telemetry chưa có flash spool khi mất Internet lâu.
- Cắt rail/STOP thật đang khóa cho đến khi nghiệm thu mạch nguồn.
- Endpoint MQTT tạm thời phải đổi sang endpoint ổn định trước triển khai.
