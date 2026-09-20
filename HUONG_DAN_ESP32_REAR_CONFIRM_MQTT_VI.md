# ESP32 REAR_CONFIRM sang STM32 va ThingsBoard

Firmware STM32 nay da duoc ghep voi firmware ESP32 Gateway/Remote da test truoc do. Khong can nap lai ESP32 neu hai ESP32 dang chay che do SPI one-way, moi lan bam gui 3 ban sao cung mot sequence.

## Noi day ESP32 Gateway sang STM32F411

| ESP32 Gateway | STM32F411 Black Pill | Chuc nang |
|---|---|---|
| GPIO18 | PB13 | SPI2 SCK |
| GPIO23 | PB15 | SPI2 MOSI |
| GPIO17 | PB9 | SPI2 NSS/CS |
| GND | GND | Mass chung bat buoc |

PB14/MISO khong can noi vi duong truyen nay la mot chieu. Ca hai board dung muc logic 3.3 V.

## Luong xu ly

1. Remote bam LA38 va gui `REAR_CONFIRM` qua BLE Mesh.
2. Gateway gui frame SPI 12 byte ba lan.
3. STM32 kiem header, protocol version va CRC16/CCITT-FALSE.
4. STM32 chi chap nhan node 2, event `0x10`, flags 0 va value 1.
5. Ba ban sao duoc loai trung bang cap `node + sequence`.
6. Mot event MQTT duoc xep hang. Neu MQTT dang mat mang, STM32 giu mot snapshot cho den khi ket noi lai.

## Du lieu ThingsBoard

Khi nhan mot lan bam hop le, telemetry co cac truong chinh:

```json
{
  "event": "REAR_CONFIRM",
  "event_code": 3,
  "rear_confirmed": true,
  "rear_confirm_node": 2,
  "rear_confirm_sequence": 86,
  "rear_confirm_count": 1,
  "rear_duplicate_count": 2,
  "rear_invalid_count": 0,
  "gateway_spi_ok": true
}
```

`rear_duplicate_count` tang la binh thuong vi Gateway co y gui 3 ban sao de tang do tin cay. `rear_confirm_count` chi tang mot lan cho moi sequence.

## Live Expressions de test

Them cac bien sau trong STM32CubeIDE:

- `tripguard_gateway_init_ok`
- `tripguard_spi_transaction_count`
- `tripguard_spi_hal_error_count`
- `tripguard_spi_rearm_error_count`
- `tripguard_gateway_valid_rx_count`
- `tripguard_gateway_invalid_rx_count`
- `tripguard_rear_confirmed`
- `tripguard_rear_confirm_node`
- `tripguard_rear_confirm_sequence`
- `tripguard_rear_confirm_count`
- `tripguard_rear_duplicate_count`
- `tripguard_rear_invalid_count`
- `tripguard_telemetry_pending`
- `tripguard_publish_count`
- `tripguard_mqtt_connected`

Ket qua mong doi sau mot lan bam: `transaction_count` tang 3, `valid_rx_count` tang 3, `rear_confirm_count` tang 1, `rear_duplicate_count` tang khoang 2. Khi MQTT ket noi, `telemetry_pending` tro ve 0 va `publish_count` tang.

## File firmware

Build Debug da kiem tra thanh cong voi STM32CubeIDE 1.17.0, 0 error va 0 warning. File ELF nam tai `Debug/TripGuard_STM32F411_RTOS.elf`.
