# Huong dan import TripGuard vao STM32CubeIDE

File `.ioc` tuong thich STM32CubeMX 6.13.0 (ban tich hop trong CubeIDE cua ban).

## Import

1. Dong cac phien Debug dang chay bang nut Stop mau do.
2. Vao `File > Import...`.
3. Chon `Existing Projects into Workspace` (CubeIDE 1.17 co the hien truc tiep,
   khong co nhom `General`).
4. Chon `Select root directory` neu da giai nen, hoac `Select archive file` neu
   dang dung file ZIP.
5. Danh dau project `TripGuard_STM32F411_RTOS` va bam `Finish`.
6. Bam chuot phai project, chon `Build Configurations > Set Active > Debug`.
7. Chon `Project > Clean...`, danh dau `TripGuard_STM32F411_RTOS` va Clean.
8. Bam `Build Project`.

Neu CubeIDE bao project da ton tai, hay xoa project cu khoi Project Explorer
bang `Delete` nhung KHONG chon xoa file tren o dia, sau do import lai.

## Nap chuong trinh

1. Cam ST-LINK va cap nguon cho cac module.
2. Bam chuot phai project, chon `Debug As > STM32 C/C++ Application`.
3. Neu CPU dung tai `main`, bam `Resume (F8)`.

## So do chan hien tai

| Thiet bi | Chan STM32F411 |
|---|---|
| GPS NEO-M8L | PA9 TX, PA10 RX, USART1 9600 |
| A7670/SIM7680 | PA2 TX, PA3 RX, USART2 9600 |
| Raspberry Pi 4 | PA11 TX -> GPIO15/RXD pin 10; PA12 RX <- GPIO14/TXD pin 8; USART6 115200 |
| ESP32 Gateway | PB9 NSS, PB13 SCK, PB15 MOSI; nhan radar Remote qua SPI2 |
| JQ8900 IO1 | PB0, active LOW |
| JQ8900 BUSY | PB12 |
| Nut SOS | PB1 noi GND, pull-up noi |
| Nut ACK | PB10 noi GND, pull-up noi |
| Servo | PA6, TIM3 CH1, 50 Hz |

## Clock da cau hinh san

- SYSCLK/HCLK: 96 MHz tu HSE 25 MHz qua PLL (M=25, N=192, P=2, Q=4).
- APB1: 48 MHz; timer APB1: 96 MHz.
- APB2: 96 MHz.
- TIM3: prescaler 95, period 19999, pulse 1500 -> PWM 50 Hz; servo 360 do
  quay theo thoi gian va luon tro ve xung dung.
- USART6 115200 baud, 8N1, DMA RX-to-IDLE cho PiLink.

Tat ca nguon phai noi chung GND. A7670 can nguon rieng du dong, khong cap tu
chan 5 V yeu cua ST-LINK.

## Cau hinh da dat

- So dien thoai canh bao: `+84838740019`.
- Radar: phat loa + gui SMS, khong goi.
- Nut SOS: phat loa + gui SMS `CUU TUI` + goi dien.
- APN, ThingsBoard host/port va token nam trong `Core/Inc/tripguard_config.h`.

Pinggy mien phi co the doi hostname/port sau moi lan tao tunnel. Khi website
khong nhan du lieu, cap nhat lai `TRIPGUARD_TB_HOST` va
`TRIPGUARD_TB_PORT`, sau do build va nap lai.

## Bien nen them vao Live Expressions

```text
tripguard_modem_state
tripguard_modem_ready
tripguard_cereg_status
tripguard_last_at_command
(char*)tripguard_last_at_response
tripguard_modem_uart_error_count
tripguard_system_state
tripguard_gateway_radar_healthy
tripguard_gateway_radar_person
tripguard_gateway_radar_distance_cm
pi_online
pi_ready
pi_camera_person
pi_last_packet_age_ms
pi_rx_frame_count
pi_tx_frame_count
pi_crc_error_count
pi_uart_error_count
tripguard_sos_count
tripguard_audio_track
tripguard_audio_busy_raw
tripguard_alert_phase
tripguard_alert_last_result
tripguard_sms_sent_count
tripguard_call_started_count
```
