# TripGuard GPS + 4G

Project STM32CubeIDE: `TripGuard_STM32F411_RTOS`.

## Cau hinh truoc khi nap

Sua `Core/Inc/tripguard_config.h`:

- `TRIPGUARD_CELLULAR_APN`: APN cua SIM dang dung.
- `TRIPGUARD_TB_HOST`: may chu ThingsBoard.
- `TRIPGUARD_TB_PORT`: cong MQTT TCP cong khai. Neu dung Pinggy, day la
  port ngau nhien Pinggy hien ra, khong phai luon la `1883`.
- `TRIPGUARD_TB_ACCESS_TOKEN`: access token cua device ThingsBoard.
- `TRIPGUARD_TB_CLIENT_ID`: client ID duy nhat cua thiet bi.
- `TRIPGUARD_EMERGENCY_NUMBER`: so nhan SMS va cuoc goi SOS.

Khong commit access token that len kho ma nguon cong khai. Neu token bi lo, tao
token moi tren ThingsBoard va cap nhat lai file cau hinh.

## Phan cong UART

- USART1, 9600 baud, RX interrupt tung byte: NEO-M8L (`gps.c`).
- USART2, 9600 baud, RX DMA circular + Receive-to-IDLE: SIM7680C/A7670C (`a7670.c`).
- USART6, 115200 baud, 8N1, TX/RX DMA RX-to-IDLE: Raspberry Pi 4 (`pi_link.c`).
- DMA1 Stream5 Channel4: USART2 RX.
- DMA2 Stream1 Channel5: USART6 RX.

## So do chan RTOS da chot

- GPS NEO-M8L: PA9/USART1_TX -> RX GPS, PA10/USART1_RX <- TX GPS.
- A7670C: PA2/USART2_TX -> RX modem, PA3/USART2_RX <- TX modem.
- Raspberry Pi: PA11/USART6_TX -> GPIO15/RXD (pin 10),
  PA12/USART6_RX <- GPIO14/TXD (pin 8), GND chung (pin 6); khong noi 5V.
- LD2410 nam trong ESP32 Remote; radar den STM32 qua BLE Mesh -> ESP32 Gateway
  -> SPI2, khong noi truc tiep vao PA11/PA12.
- JQ8900 IO1 -> PB0, kich muc thap; radar va SOS dung chung mot am thanh.
- JQ8900 BUSY -> PB12.
- Nut SOS -> PB1 va GND, pull-up noi, ngat canh xuong.
- Nut ACK -> PB10 va GND, pull-up noi, ngat canh xuong.
- Servo PWM -> PA6/TIM3_CH1, 50 Hz.

Clock he thong da dat 96 MHz bang HSE 25 MHz + PLL. APB1 = 48 MHz,
APB2 = 96 MHz; TIM3 dung prescaler 95 va period 19999 de tao PWM 50 Hz.
- Tat ca module va STM32 phai noi chung GND.

Hanh vi canh bao:

- Radar xac nhan co nguoi: phat loa va gui SMS `PHAT HIEN CO NGUOI`, khong goi.
- Nhan nut SOS PB1: phat loa, gui SMS `CUU TUI`, sau do goi dien.
- Nhan nut ACK PB10: xac nhan/tat canh bao dang cho.

Thu tu khoi tao bat buoc trong `main.c`:

1. `MX_DMA_Init()`
2. `MX_TIM3_Init()`
3. `MX_USART1_UART_Init()`
4. `MX_USART2_UART_Init()`
5. `MX_USART6_UART_Init()`
6. `GPS_Init()`
7. `A7670_Init()`
8. `JQ8900_Init()`
9. `TB_MQTT_Init()`

Sau `osKernelStart()`, khong co driver nao chay trong vong `while` cua
`main`. GPS, cellular/MQTT, safety, telemetry, audio va health chay trong cac
task rieng o `Core/Src/freertos.c`. `PiLinkTask` la chu duy nhat cua parser va
hang doi TX USART6. CellularTask la task duy nhat so huu modem va MQTT; task
khac gui yeu cau telemetry qua queue de tranh tranh chap UART.

## State machine

Modem:

`BOOTING -> AT_WAIT -> SIM_WAIT -> NETWORK_WAIT -> PDP_WAIT -> READY`

MQTT:

`IDLE -> START_SERVICE -> ACQUIRE_CLIENT -> CONNECTING -> CONNECTED`

Khi loi mang/MQTT, firmware huy phien MQTT cu, release client, stop service va
thu lai voi backoff. Firmware khong reset MCU va khong reset modem tu dong.

## Bien/API chan doan

Them cac bien sau vao Live Expressions (khong them dau ngoac):

- `tripguard_modem_state`, `tripguard_modem_ready`
- `tripguard_cereg_status`, `tripguard_network_registered`
- `tripguard_pdp_active`, `tripguard_ip_address`
- `tripguard_last_at_command`, `tripguard_last_at_response`
- `tripguard_mqtt_state`, `tripguard_mqtt_phase`
- `tripguard_mqtt_connected`, `tripguard_mqtt_last_error`
- `tripguard_publish_count`, `tripguard_mqtt_error_count`
- `tripguard_modem_uart_error_count`, `tripguard_modem_rx_overflow_count`
- `tripguard_gps_state`, `tripguard_gps_comm_state`, `tripguard_speed_kmh`
- `tripguard_driving_state` (`0=UNKNOWN`, `1=STOPPED`, `2=MOVING`)
- `tripguard_radar_online`, `tripguard_radar_state`
- `tripguard_person_detected`, `tripguard_sos_pending`, `tripguard_sos_count`
- `pi_online`, `pi_ready`, `pi_camera_person`, `pi_last_packet_age_ms`
- `pi_rx_frame_count`, `pi_tx_frame_count`, `pi_crc_error_count`
- `pi_uart_error_count`, `pi_duplicate_count`, `pi_rx_overflow_count`
- `tripguard_gateway_radar_healthy`, `tripguard_gateway_radar_person`
- `tripguard_gateway_radar_distance_cm`, `tripguard_gateway_radar_last_sequence`
- `tripguard_driver_ack_count`, `tripguard_audio_track`
- `tripguard_audio_busy_raw`, `tripguard_audio_error_count`
- `tripguard_stack_overflow_fault` (binh thuong bang `0`)

Gia tri thanh cong mong doi:

- `tripguard_modem_state = MODEM_READY`
- `tripguard_cereg_status = 1` hoac `5`
- `tripguard_pdp_active = 1` va `tripguard_ip_address` khong rong
- `tripguard_mqtt_state = MQTT_CONNECTED`
- `tripguard_publish_count` tang sau moi 2 giay

Firmware build sach thanh file:

`Debug/TripGuard_STM32F411_RTOS.elf`

## Trinh tu bring-up khuyen nghi

1. Xac nhan modem di den `MODEM_AT_WAIT`, sau do `MODEM_SIM_WAIT`.
2. Xac nhan `+CPIN: READY` va state `MODEM_NETWORK_WAIT`.
3. Xac nhan `+CEREG` la `1` hoac `5`.
4. Xac nhan context 1 active va state `MODEM_READY`.
5. Xac nhan MQTT di den `MQTT_CONNECTED`.
6. Theo doi `TB_MQTT_GetPublishCount()` tang moi lan telemetry thanh cong.

Neu modem dung o mot state, xem response AT gan nhat va cac bo dem UART/ring
buffer truoc khi them reset phan cung.

## Luu y nap va chay

Launch configuration da tat `Stop at main`, vi vay sau khi bam Debug chuong
trinh tu chay. Neu tao launch configuration moi va thay CPU dung tai `main`,
bam Resume (F8) hoac bo chon `Stop on startup at: main`.

File `.ioc` hien tai dung CubeMX 6.13.0 va STM32Cube FW_F4 V1.28.x, phu hop
STM32CubeIDE 1.17. Sau khi Generate Code, build lai va xac nhan `pi_link.c`
van nam trong cau hinh build.

Broker ThingsBoard phai cho phep ket noi MQTT TCP truc tiep toi cong 1883.
Neu ten mien duoc proxy qua Cloudflare, tao mot DNS record rieng (DNS only)
cho MQTT hoac dung IP/hostname truc tiep cua broker; Cloudflare proxy web thong
thuong khong chuyen tiep MQTT TCP 1883.

Neu ThingsBoard chay trong Docker tren PC, mo Pinggy TCP tunnel tren CHINH PC
dang chay Docker bang `ssh -p 443 -R0:localhost:1883 tcp@a.pinggy.io`. Sau do
copy hostname va port moi Pinggy in ra vao `tripguard_config.h`. Tunnel mien
phi het han sau 60 phut va endpoint cu se khong ket noi lai duoc.

Ma `+CMQTTCONNECT: 0,3` cua A76xx co nghia la `socket connect fail`: kiem tra
port 1883, firewall/NAT va DNS record cua broker. Ma nay khong phai loi UART,
SIM hay APN.
