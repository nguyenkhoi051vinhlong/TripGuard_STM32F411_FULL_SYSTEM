# TripGuard GPS + 4G

## Cau hinh truoc khi nap

Sua `Core/Inc/tripguard_config.h`:

- `TRIPGUARD_CELLULAR_APN`: APN cua SIM dang dung.
- `TRIPGUARD_TB_HOST`: may chu ThingsBoard.
- `TRIPGUARD_TB_PORT`: cong MQTT TCP, mac dinh `1883`.
- `TRIPGUARD_TB_ACCESS_TOKEN`: access token cua device ThingsBoard.
- `TRIPGUARD_TB_CLIENT_ID`: client ID duy nhat cua thiet bi.

Gia tri token trong source hien la placeholder; can thay truoc khi test MQTT.

## Phan cong UART

- USART1, 9600 baud, RX interrupt tung byte: NEO-M8L (`gps.c`).
- USART2, 115200 baud, RX DMA circular + Receive-to-IDLE: A7670C (`a7670.c`).
- DMA1 Stream5 Channel4: USART2 RX.

Thu tu khoi tao bat buoc trong `main.c`:

1. `MX_DMA_Init()`
2. `MX_USART1_UART_Init()`
3. `MX_USART2_UART_Init()`
4. `GPS_Init()`
5. `A7670_Init()`
6. `TB_MQTT_Init()`

## State machine

Modem:

`BOOTING -> AT_WAIT -> SIM_WAIT -> NETWORK_WAIT -> PDP_WAIT -> READY`

MQTT:

`IDLE -> START_SERVICE -> ACQUIRE_CLIENT -> CONNECTING -> CONNECTED`

Khi loi mang/MQTT, firmware huy phien MQTT cu, release client, stop service va
thu lai voi backoff. Firmware khong reset MCU va khong reset modem tu dong.

## Bien/API chan doan

- `A7670_GetState()`
- `A7670_GetCommandResponse()`
- `A7670_GetRxOverflowCount()`
- `A7670_GetUartErrorCount()`
- `TB_MQTT_GetState()`
- `TB_MQTT_GetPublishCount()`
- `TB_MQTT_GetErrorCount()`
- `GPS_GetData()`

## Trinh tu bring-up khuyen nghi

1. Xac nhan modem di den `MODEM_AT_WAIT`, sau do `MODEM_SIM_WAIT`.
2. Xac nhan `+CPIN: READY` va state `MODEM_NETWORK_WAIT`.
3. Xac nhan `+CEREG` la `1` hoac `5`.
4. Xac nhan context 1 active va state `MODEM_READY`.
5. Xac nhan MQTT di den `MQTT_CONNECTED`.
6. Theo doi `TB_MQTT_GetPublishCount()` tang moi lan telemetry thanh cong.

Neu modem dung o mot state, xem response AT gan nhat va cac bo dem UART/ring
buffer truoc khi them reset phan cung.
