# SERVO 360 DO QUAY CAMERA THEO RADAR CO DINH

Radar LD2410 nam trong ESP32 Remote. Du lieu PERSON/CLEAR/HEALTH di qua BLE
Mesh den ESP32 Gateway, sau do qua SPI2 den STM32. Trong ACTIVE, khi co nguoi
lien tuc 2 giay, servo quay camera ve huong radar trong mot khoang thoi gian
roi firmware dua PWM ve muc dung. Khi het nguoi lien tuc 8 giay, servo quay
nguoc ve home va cung tu dung.

## Noi day

| Thiet bi | STM32F411 |
|---|---|
| ESP32 Gateway CS/SCLK/MOSI | PB9 / PB13 / PB15, SPI2 Mode 0, 1 MHz |
| Raspberry Pi TX/RX | PA12 USART6_RX / PA11 USART6_TX, 115200 baud |
| Servo signal | PA6 / TIM3_CH1, 50 Hz |
| Servo VCC | Nguon 5V ngoai du dong |
| ESP32 Remote + LD2410 | Nguon theo mach Remote, khong noi UART vao STM32 |
| GND | Tat ca noi chung GND |

## Can chinh trong `Core/Inc/tripguard_config.h`

```c
#define TRIPGUARD_SERVO_STOP_PULSE_US          1500U
#define TRIPGUARD_SERVO_TO_RADAR_PULSE_US      1700U
#define TRIPGUARD_SERVO_TO_HOME_PULSE_US       1300U
#define TRIPGUARD_SERVO_MOVE_TIME_MS            700U
```

- Van quay khi dung: chinh STOP quanh 1500 us.
- Sai chieu: doi 1700 va 1300.
- Chua du goc: tang MOVE_TIME.
- Qua goc: giam MOVE_TIME.

## Live Expressions

```text
tripguard_gateway_radar_healthy
tripguard_gateway_radar_person
tripguard_gateway_radar_distance_cm
tripguard_gateway_radar_last_sequence
tripguard_gateway_radar_duplicate_count
tripguard_servo_pulse_us
tripguard_servo_moving
tripguard_servo_tracking_radar
tripguard_servo_move_count
```

Khi servo da dung, `tripguard_servo_pulse_us` phai gan 1500 va
`tripguard_servo_moving=0`. Khi Gateway khong con gui Radar Health hop le,
khong co canh bao moi va servo khong duoc quay lien tuc.
