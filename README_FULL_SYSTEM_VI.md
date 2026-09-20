# TRIPGUARD STM32F411 - FULL SYSTEM

Project STM32CubeIDE cho STM32F411CEU6 BlackPill, gom day du:

- SIM A7670: dang ky mang, PDP, SMS va cuoc goi SOS.
- MQTT ThingsBoard: gui telemetry va nhan RPC dieu khien tu dashboard.
- Radar HLK-LD2410 nam trong ESP32 Remote; du lieu den STM32 qua BLE Mesh,
  ESP32 Gateway va SPI2.
- Raspberry Pi 4: giao tiep hai chieu voi STM32 bang PiLink tren USART6.
- Servo 360 do: quay camera ve huong radar theo thoi gian, sau do tu dung.
- GPS NEO-M8L: doc NMEA, gui latitude/longitude len ThingsBoard.
- Nut SOS va nut ACK: EXTI, pull-up, chong doi phim.
- Loa JQ8900: kich file am thanh bang IO1, doc trang thai BUSY.
- FreeRTOS: tach task Safety, Cellular, GPS, Telemetry, Audio va Health.

## 1. So do chan day du

| Thiet bi | Chan module | STM32F411 | Cau hinh |
|---|---|---|---|
| A7670 | TXD | PA3 / USART2_RX | 9600, 8N1, DMA RX-to-IDLE |
| A7670 | RXD | PA2 / USART2_TX | 9600, 8N1 |
| A7670 | GND | GND chung | Bat buoc |
| Raspberry Pi 4 | GPIO14/TXD, pin 8 | PA12 / USART6_RX | 115200, 8N1, DMA RX-to-IDLE |
| Raspberry Pi 4 | GPIO15/RXD, pin 10 | PA11 / USART6_TX | 115200, 8N1 |
| Raspberry Pi 4 | GND, pin 6 | GND chung | Khong noi 5V giua hai bo |
| NEO-M8L | TX | PA10 / USART1_RX | 9600, 8N1, interrupt tung byte |
| NEO-M8L | RX | PA9 / USART1_TX | Khong bat buoc khi chi doc GPS |
| Servo 360 | Signal | PA6 / TIM3_CH1 | PWM 50 Hz |
| Servo 360 | VCC | Nguon 5V ngoai | Khong cap tu chan 5V/3V3 STM32 |
| Servo 360 | GND | GND chung | Chung GND voi STM32 |
| JQ8900 | IO1 | PB0 | Active LOW, kich 200 ms |
| JQ8900 | BUSY | PB12 | Digital input |
| Nut SOS | Mot dau | PB1 | EXTI falling, pull-up noi |
| Nut SOS | Dau con lai | GND | Nhan = muc LOW |
| Nut ACK | Mot dau | PB10 | EXTI falling, pull-up noi |
| Nut ACK | Dau con lai | GND | Nhan = muc LOW |
| ST-LINK | SWDIO | PA13 | Nap/debug |
| ST-LINK | SWCLK | PA14 | Nap/debug |
| Tat ca module | GND | GND chung | Bat buoc |

PA11/PA12 trung voi USB D-/D+ cua BlackPill, vi vay khi dung PiLink khong dung
USB data cua STM32 cung luc. Cap USB chi nen dung de cap nguon neu he thong
nguon da duoc thiet ke an toan.

## 2. Nguon dien

- A7670 can nguon rieng dung dien ap cua board A7670 va dong dinh cao (thuong
  can toi khoang 2 A). Khong nuoi modem tu chan 5V/3V3 cua STM32.
- Servo dung nguon 5V rieng du dong. Dat tu 470 uF den 1000 uF gan servo.
- Noi chung GND cua nguon modem, nguon servo, ESP32, Pi, GPS, loa va STM32.
- Neu servo lam STM32 reset/MQTT mat ket noi, nguyen nhan dau tien can kiem tra
  la sut ap va day GND.

## 3. Hoat dong cua he thong

| Su kien | Servo | Loa | SMS | Cuoc goi | MQTT |
|---|---|---|---|---|---|
| Radar Remote co nguoi lien tuc 2 giay trong ACTIVE | Quay ve radar 700 ms roi dung | Phat canh bao | Co | Khong | Gui telemetry |
| Radar Remote het nguoi lien tuc 8 giay | Quay nguoc ve home 700 ms roi dung | Khong | Khong | Khong | Gui dinh ky |
| Camera Pi bao nguoi | Khong tu kich hoat | Khong | Khong | Khong | Cap nhat chan doan |
| Nhan nut SOS PB1 | Khong doi | Phat SOS | Co | Co, 20 giay | Gui trang thai |
| Nhan nut ACK PB10 | Khong doi | Tra IO1 ve muc nghi | Khong | Khong | Xoa SOS pending |
| Radar bi thao/mat du lieu | Khong phat hien nguoi; PWM luon ve 1500 us | Khong tao canh bao moi | Khong | Khong | `radar_online=false` |

Self-test phai co SIM/network, MQTT publish, Radar Health tu Gateway va Pi
READY/heartbeat truoc khi vao STANDBY. Mot canh bao radar moi chi duoc mo lai
sau khi vung quet yen 8 giay va qua thoi gian bao ve 30 giay.

## 4. Can chinh servo 360 do

Mo `Core/Inc/tripguard_config.h`:

```c
#define TRIPGUARD_SERVO_STOP_PULSE_US          1500U
#define TRIPGUARD_SERVO_TO_RADAR_PULSE_US      1700U
#define TRIPGUARD_SERVO_TO_HOME_PULSE_US       1300U
#define TRIPGUARD_SERVO_MOVE_TIME_MS            700U
```

- Servo van quay nhe khi phai dung: thu STOP 1490, 1480, 1510 hoac 1520 us.
- Quay sai chieu: doi hai gia tri 1700 va 1300.
- Quay chua du goc: tang MOVE_TIME, vi du 900 ms.
- Quay qua goc: giam MOVE_TIME, vi du 500 ms.
- Servo 360 do khong co feedback goc. `servo_angle_deg` tren dashboard chi la
  vi tri logic; `servo_pulse_us=1500` va `servo_moving=false` moi xac nhan da
  gui lenh dung.

## 5. Cau hinh SIM, so dien thoai va MQTT

Tat ca nam trong `Core/Inc/tripguard_config.h`:

```c
#define TRIPGUARD_CELLULAR_APN       "v-internet"
#define TRIPGUARD_TB_HOST            "vsxvv-58-187-48-135.run.pinggy-free.link"
#define TRIPGUARD_TB_PORT            45483U
#define TRIPGUARD_TB_ACCESS_TOKEN    "Trankhoinguyen0110"
#define TRIPGUARD_EMERGENCY_NUMBER   "+84838740019"
```

Endpoint Pinggy free se doi khi tao tunnel moi. Moi lan PowerShell in host/port
moi, thay dung `TRIPGUARD_TB_HOST` va `TRIPGUARD_TB_PORT`, sau do Clean, Build
va nap lai. HOST chi ghi hostname, khong ghi `tcp://`.

Lenh tao tunnel mau:

```powershell
ssh -p 443 -R0:localhost:1883 a.pinggy.io
```

Mat khau Pinggy khong duoc luu trong firmware. Firmware chi can host va port
TCP ma Pinggy cap.

## 6. RPC ThingsBoard da ho tro

| Method | Params | Tac dung |
|---|---|---|
| `playAlarm` | `1` hoac `2` | Phat canh bao loa |
| `stopAudio` | `{}` | Dua IO1 ve muc nghi |
| `setRadarAlerts` | `true`/`false` | Bat/tat canh bao radar |
| `acknowledgeSOS` | `{}` | Xac nhan SOS va dung loa |
| `sendRadarSms` | `{}` | Gui thu SMS radar |
| `triggerSOS` | `{}` | Phat loa + SMS + goi dien |
| `setTelemetryPeriod` | `2000..60000` | Doi chu ky telemetry ms |
| `refreshTelemetry` | `{}` | Gui telemetry ngay |
| `cameraToRadar` | `{}` | Quay camera ve huong radar roi tu dung |
| `cameraHome` | `{}` | Quay camera ve home roi tu dung |
| `stopServo` | `{}` | Dua servo ve xung dung ngay |

Thu muc `ThingsBoard_Dashboard` co hai widget va dashboard JSON. Import theo
`ThingsBoard_Dashboard/HUONG_DAN_IMPORT_VI.md`. Widget dieu khien da co callback
complete va watchdog, khong bi ket mai o trang thai `Dang gui`.

## 7. Mo, build va nap

1. STM32CubeIDE: `File > Import > Existing Projects into Workspace`.
2. Chon thu muc project nay, tick project `TripGuard_STM32F411_RTOS`.
3. `Project > Clean`, sau do `Project > Build Project`.
4. Cam ST-LINK va bam Run hoac Debug de nap.
5. Sau khi nap, bam Resume neu dang o Debug. Neu CPU dung o `main()` thi cac
   task va Live Expressions se khong cap nhat.

Clock dung HSE 25 MHz cua BlackPill, SYSCLK/APB2 96 MHz. USART6 chay 115200,
8N1, TX/RX voi DMA2 Stream1 Channel5 cho PiLink; khong con gan truc tiep LD2410.

## 8. Live Expressions de kiem tra

Them cac bien sau:

```text
tripguard_uptime_ms
tripguard_modem_state
tripguard_network_registered
tripguard_pdp_active
tripguard_ip_address
tripguard_mqtt_state
tripguard_mqtt_connected
tripguard_publish_count
tripguard_mqtt_error_count
tripguard_system_state
tripguard_supervisor_rear_confirmed
tripguard_gateway_radar_healthy
tripguard_gateway_radar_person
tripguard_gateway_radar_distance_cm
tripguard_gateway_radar_last_sequence
tripguard_gateway_radar_duplicate_count
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
tripguard_servo_pulse_us
tripguard_servo_moving
tripguard_servo_tracking_radar
tripguard_sos_count
tripguard_sos_button_raw
tripguard_driver_ack_count
tripguard_ack_button_raw
tripguard_audio_busy_raw
tripguard_sms_sent_count
tripguard_call_started_count
tripguard_alert_error_count
tripguard_gps_state
tripguard_gps_comm_state
```

Gia tri mong doi:

- Radar Remote: `tripguard_gateway_radar_healthy=1`; sequence/valid RX tang khi
  Gateway gui HEALTH/PERSON/CLEAR.
- PiLink: `pi_online=1`, `pi_ready=1`, frame RX/TX tang; CRC/UART error on dinh.
- MQTT: `mqtt_connected=1`, `publish_count` tang theo chu ky.
- Servo dung: `servo_pulse_us` gan 1500, `servo_moving=0`.
- GPS ngoai troi: `gps_state=GPS_FIXED`; telemetry co latitude/longitude.

## 9. Luu y ve loa JQ8900

Project dang dung che do mot nut IO1: keo PB0 xuong LOW 200 ms de phat file
am thanh dau tien. Hay dat file am thanh dung thu tu ma module yeu cau trong
bo nho cua JQ8900. Che do IO1 khong co lenh stop giua file; `stopAudio` chi tra
IO1 ve HIGH. Neu can dung ngay giua bai, can noi UART cua JQ8900 va dung bo
lenh serial cua dung phien ban module.
