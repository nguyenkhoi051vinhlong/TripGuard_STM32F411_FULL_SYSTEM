#!/usr/bin/env python3
"""Static acceptance checks for the TripGuard STM32 logical supervisor."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
passed = 0


def check(condition: bool, name: str) -> None:
    global passed
    if not condition:
        raise AssertionError(name)
    passed += 1
    print(f"PASS {passed:02d}: {name}")


def read(relative: str) -> str:
    path = ROOT / relative
    check(path.is_file(), f"file exists: {relative}")
    return path.read_text(encoding="utf-8", errors="strict")


def macro(text: str, name: str) -> str:
    match = re.search(rf"^#define\s+{re.escape(name)}\s+([^\s/]+)",
                      text, re.MULTILINE)
    if match is None:
        raise AssertionError(f"missing macro {name}")
    return match.group(1).rstrip("UuLl")


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) if crc & 0x8000
                   else (crc << 1)) & 0xFFFF
    return crc


def main() -> int:
    config = read("Core/Inc/tripguard_config.h")
    supervisor_h = read("Core/Inc/tripguard_supervisor.h")
    supervisor_c = read("Core/Src/tripguard_supervisor.c")
    pi_h = read("Core/Inc/pi_link.h")
    pi_c = read("Core/Src/pi_link.c")
    protocol_h = read("Core/Inc/tripguard_protocol.h")
    protocol_c = read("Core/Src/tripguard_protocol.c")
    spi_c = read("Core/Src/spi.c")
    link_c = read("Core/Src/tripguard_spi.c")
    bus_c = read("Core/Src/tripguard_bus_power.c")
    freertos = read("Core/Src/freertos.c")
    main_c = read("Core/Src/main.c")
    main_h = read("Core/Inc/main.h")
    usart_c = read("Core/Src/usart.c")
    dma_c = read("Core/Src/dma.c")
    irq_c = read("Core/Src/stm32f4xx_it.c")
    gateway_h = read("Core/Inc/tripguard_gateway.h")
    gateway_c = read("Core/Src/tripguard_gateway.c")
    gpio_c = read("Core/Src/gpio.c")
    mqtt_c = read("Core/Src/tb_mqtt.c")
    pi_script = read("raspberry_pi/tripguard_uart.py")
    ioc = read("TripGuard_STM32F411_RTOS.ioc")

    for state in ("SYSTEM_SELF_TEST", "SYSTEM_STANDBY",
                  "SYSTEM_WAIT_REAR_CONFIRM", "SYSTEM_ARM_DELAY",
                  "SYSTEM_ACTIVE", "SYSTEM_FAULT"):
        check(state in supervisor_h and state in supervisor_c,
              f"state implemented: {state}")

    required_macros = {
        "TRIPGUARD_ARM_DELAY_MS": "60000",
        "TRIPGUARD_POWER_QUALIFY_HIGH_MS": "30000",
        "TRIPGUARD_POWER_LOSS_DEBOUNCE_MS": "3000",
        "TRIPGUARD_GEOFENCE_DWELL_MS": "15000",
        "TRIPGUARD_GPS_MAX_AGE_MS": "3000",
        "TRIPGUARD_GEOFENCE_ENABLED": "0",
        "TRIPGUARD_SPI_ONE_WAY": "1",
        "TRIPGUARD_USE_STOP_MODE": "0",
        "TRIPGUARD_CONNECTIVITY_OWNER_PI": "1",
        "TRIPGUARD_REAR_CONFIRM_TIMEOUT_MS": "300000",
    }
    for name, value in required_macros.items():
        check(macro(config, name) == value, f"config {name}={value}")

    for name in ("TRIPGUARD_STATION_LAT", "TRIPGUARD_STATION_LON",
                 "TRIPGUARD_STATION_RADIUS_M",
                 "TRIPGUARD_MAX_ARRIVAL_SPEED_KMH",
                 "TRIPGUARD_POWER_ACTIVE_LEVEL"):
        check(re.search(rf"^#define\s+{name}\s+", config, re.MULTILINE)
              is not None, f"deployment macro exists: {name}")

    check("TripGuardSupervisorTask" in freertos and
          "TripGuard_Supervisor_Task(argument)" in freertos,
          "50 ms supervisor has a dedicated FreeRTOS task")
    check("osDelay(TG_SUPERVISOR_PERIOD_MS)" in supervisor_c and
          "TG_SUPERVISOR_PERIOD_MS          50U" in supervisor_c,
          "supervisor period is 50 ms")
    check("osDelay(60000" not in freertos + supervisor_c and
          "HAL_Delay(60000" not in freertos + supervisor_c,
          "60 second arming delay is non-blocking")
    check("(uint32_t)(now_ms - started_ms)" in supervisor_c,
          "state timers use wrap-safe unsigned subtraction")

    check("TripGuard_IsRearDuplicate" in freertos and
          "TripGuard_Supervisor_PostRearConfirm" in freertos and
          "tripguard_rear_duplicate_count++" in freertos,
          "rear sequence is deduplicated before supervisor")
    first_branch = freertos.index("TripGuard_Supervisor_PostRearConfirm")
    duplicate_branch = freertos.index("tripguard_rear_duplicate_count++",
                                       first_branch)
    check(first_branch < duplicate_branch,
          "duplicate branch cannot restart arming countdown")

    check("TripGuard_Supervisor_StartArm(now_ms," in supervisor_c and
          "ACTIVATION_MANUAL_REAR_CONFIRM" in supervisor_c and
          "ACTIVATION_AUTO_REAR_TIMEOUT" in supervisor_c and
          "ACTIVATION_GPS_GEOFENCE" not in supervisor_c and
          "ACTIVATION_POWER_LOSS" not in supervisor_c and
          "ACTIVATION_RPC" not in supervisor_c,
          "only rear-confirm or its five-minute timeout can arm")
    check("#if (TRIPGUARD_GEOFENCE_ENABLED != 0U)" in supervisor_c,
          "geofence is compile-time disabled without real coordinates")
    check("TripGuard_Supervisor_StartRearConfirmWait" in supervisor_c and
          "TRIPGUARD_GEOFENCE_DWELL_MS" in supervisor_c and
          "TRIPGUARD_REAR_CONFIRM_TIMEOUT_MS" in supervisor_c,
          "GPS arrival starts a non-blocking five-minute rear-confirm wait")

    check("tripguard_bus_power_present = 0U" in bus_c and
          "tripguard_bus_power_high_qualified = 0U" in bus_c,
          "boot LOW cannot create a power-loss event")
    check("TRIPGUARD_POWER_QUALIFY_HIGH_MS" in bus_c and
          "TRIPGUARD_POWER_LOSS_DEBOUNCE_MS" in bus_c,
          "power HIGH 30 s and LOW 3 s qualification is implemented")

    check("TripGuard_Supervisor_IsDetectionActive" in freertos and
          "radar_raw_state = 0U" in freertos,
          "radar/camera detection is gated by ACTIVE state")
    check("TRIPGUARD_ALERT_RADAR_SMS" in freertos and
          "TRIPGUARD_ALERT_SOS_SMS_CALL" in freertos,
          "radar is SMS-only while SOS retains SMS plus call")

    legacy_data = bytes((1, 2, 0x10, 0, 0x34, 0x12, 1, 0))
    legacy_crc = crc16_ccitt_false(legacy_data)
    check(legacy_crc == 0xF4A0,
          "legacy CRC-16/CCITT-FALSE reference vector")
    framed_prefix = bytes((0xA5, 1, 0x10, 2, 0x34, 0x12, 1, 0, 0))
    framed_crc = crc16_ccitt_false(framed_prefix)
    framed = framed_prefix + bytes((framed_crc & 0xFF,
                                    framed_crc >> 8, 0x5A))
    check(len(framed) == 12 and framed[0] == 0xA5 and framed[-1] == 0x5A,
          "new 12-byte frame has Magic, Version, Command, CRC and End")
    check("tripguard_spi_encode_framed" in protocol_h + protocol_c and
          "TG_SPI_FRAME_END" in protocol_c,
          "new framed codec is implemented")
    check("TRIPGUARD_SPI_ACCEPT_LEGACY_FRAME" in protocol_c,
          "existing ESP32 12-byte frame remains accepted")

    check("SPI_DIRECTION_2LINES_RXONLY" in spi_c and
          "HAL_SPI_Receive_DMA" in link_c and
          "HAL_SPI_TransmitReceive_DMA" not in link_c,
          "SPI2 is DMA receive-only")
    check("SPI2_MISO_Pin" not in spi_c + main_h and
          "PB14.Signal=GPIO_Analog" in ioc,
          "PB14 MISO is not configured")
    check(all(token in main_h for token in
              ("BUS_POWER_SENSE_Pin GPIO_PIN_8",
               "SPI2_NSS_Pin GPIO_PIN_9",
               "SPI2_SCK_Pin GPIO_PIN_13",
               "SPI2_MOSI_Pin GPIO_PIN_15")),
          "fixed PA8/PB9/PB13/PB15 pin map")
    check("PA8.GPIO_Label=BUS_POWER_SENSE" in ioc and
          "PA15.Signal=GPIO_Analog" in ioc,
          "CubeMX metadata matches PA8 power sense")
    check("HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0)" in gpio_c,
          "POWER_SENSE EXTI priority is FreeRTOS-safe")

    check("huart6.Init.BaudRate = 115200" in usart_c and
          "huart6.Init.Mode = UART_MODE_TX_RX" in usart_c,
          "USART6 is Raspberry Pi 115200 8N1 full-duplex")
    check("DMA2_Stream1" in usart_c + dma_c + irq_c and
          "DMA_CHANNEL_5" in usart_c and
          "HAL_UARTEx_ReceiveToIdle_DMA" in pi_c,
          "USART6 RX-to-IDLE uses DMA2 Stream1 Channel5")
    check("PI_UART_TX_Pin GPIO_PIN_11" in main_h and
          "PI_UART_RX_Pin GPIO_PIN_12" in main_h and
          "PA11.GPIO_Label=PI_UART_TX" in ioc and
          "PA12.GPIO_Label=PI_UART_RX" in ioc,
          "PA11/PA12 labels and mapping are PiLink-only")
    check("LD2410_Init(&huart6)" not in main_c and
          "LD2410_Task()" not in freertos and
          "LD2410_RxCallback" not in main_c and
          "LD2410_ErrorCallback" not in main_c,
          "old direct LD2410 no longer owns USART6")
    check(main_c.count("void HAL_UARTEx_RxEventCallback") == 1 and
          main_c.count("void HAL_UART_ErrorCallback") == 1 and
          main_c.count("void HAL_UART_TxCpltCallback") == 1,
          "UART HAL callbacks remain centralized")

    for counter in ("pi_rx_frame_count", "pi_tx_frame_count",
                    "pi_crc_error_count", "pi_uart_error_count",
                    "pi_duplicate_count", "pi_rx_overflow_count",
                    "pi_last_packet_age_ms", "pi_online"):
        check(counter in pi_h and counter in pi_c,
              f"PiLink diagnostic exported: {counter}")
    check("PI_LINK_MAX_PAYLOAD                128U" in pi_h and
          "PiLink_Crc16" in pi_c and "PiLink_ParseBuffered" in pi_c and
          "malloc(" not in pi_c,
          "PiLink parser is bounded, CRC protected and allocation-free")
    check(all(token in pi_h for token in
              ("PI_MSG_HEARTBEAT", "PI_MSG_READY",
               "PI_MSG_STATUS_REQUEST", "PI_MSG_CAMERA_PERSON",
                "PI_MSG_CAMERA_CLEAR", "PI_MSG_ACK",
                "STM_MSG_HEARTBEAT", "STM_MSG_STATUS", "STM_MSG_EVENT",
                "STM_MSG_TELEMETRY_CHUNK", "STM_MSG_ALERT_REQUEST",
                "STM_MSG_ACK", "STM_MSG_NACK")),
          "all required Pi/STM message types exist")
    check("PiLink_QueueTelemetryJson" in pi_h + pi_c and
          "PiLink_ServiceTelemetry" in pi_c and
          "PI_LINK_FLAG_ACK_REQUIRED" in pi_c,
          "full STM32 JSON is chunked with UART ACK/retry")
    check("PI_MSG_ARM" not in pi_h + pi_c + freertos and
          "PI_MSG_DISARM" not in pi_h + pi_c + freertos and
          "PI_MSG_TOGGLE" not in pi_h + pi_c + freertos,
          "Pi protocol has no arm/disarm/toggle command")
    check(all(token in protocol_h + gateway_c for token in
              ("TG_EVENT_PERSON", "TG_EVENT_RADAR_CLEAR",
               "TG_EVENT_RADAR_HEALTH")) and
          "TripGuard_Gateway_IsRadarHealthy" in gateway_h + freertos,
          "Remote radar events remain on Gateway SPI path")
    check("TRIPGUARD_SELF_TEST_REQUIRED_MASK    0U" in supervisor_h and
          "TRIPGUARD_CONNECTIVITY_OWNER_PI == 0U" in supervisor_c,
          "sleep boot does not wait for Pi/radar/modem rails")
    check("TRIPGUARD_CONNECTIVITY_OWNER_PI == 0U" in main_c and
          "PiLink_QueueTelemetryJson" in freertos and
          "PiLink_SendAlertRequest" in freertos,
          "STM32 modem/MQTT ownership is disabled in Pi-gateway mode")
    check("PiLink_GetCameraPerson" in freertos and
          "TripGuard_Supervisor_IsDetectionActive() != 0U" in freertos and
          "tripguard_camera_alert_count++" in freertos,
          "Pi camera detection alerts locally only through ACTIVE gating")
    check("PI_MSG_CAMERA_PERSON" not in supervisor_c and
          "TRIPGUARD_SUP_EVT_CAMERA_PERSON" not in supervisor_c and
          "ACTIVATION_MANUAL_REAR_CONFIRM" in supervisor_c and
          "ACTIVATION_AUTO_REAR_TIMEOUT" in supervisor_c,
          "Pi detection cannot arm; only scan completion may end a session")

    required_telemetry = (
        "systemState", "systemArmed", "activationSource",
        "activationCountdown", "rearConfirmWaitRemainingSec",
        "connectivityOwner", "rearConfirmed", "powerPresent",
        "stationDetected", "gpsFixValid", "espGatewayOnline",
        "radarEnabled", "cameraEnabled", "lastActivationReason",
        "lastError",
        "piOnline", "piReady", "piRxFrames", "piTxFrames",
        "piCrcErrors", "piUartErrors", "piLastPacketAgeMs",
        "cameraPerson", "cameraConfidencePermille",
        "piScanSessionId", "piScanCommandSentCount",
        "piScanCommandAckCount", "piScanCompleteCount",
        "piScanLastResult",
        "cameraAlertArmed", "cameraAlertCount", "armRemainingSec",
    )
    check(all(f'\\"{key}\\"' in freertos for key in required_telemetry),
          "all required ThingsBoard telemetry keys are emitted")
    check(all(method in freertos for method in
              ("setSystemArmed", "cancelArm", "getSystemState")) and
          "TB_MQTT_PublishRpcResponse" in freertos + mqtt_c,
          "required RPC methods publish applied results")
    check("v1/devices/me/rpc/response/" in mqtt_c and
          "mqtt_publish_topic" in mqtt_c,
          "ThingsBoard RPC response uses request-specific topic")

    app_sources = "".join(path.read_text(encoding="utf-8", errors="strict")
                          for path in (ROOT / "Core/Src").glob("*.c"))
    check(re.search(r"\bmalloc\s*\(", app_sources) is None,
          "application code does not call malloc")
    check((ROOT / "raspberry_pi/tripguard_uart.py").is_file() and
          (ROOT / "raspberry_pi/tripguard-uart.service").is_file() and
          (ROOT / "raspberry_pi/tripguard-camera.service").is_file() and
          (ROOT / "raspberry_pi/README.md").is_file(),
          "Raspberry Pi UART/camera services and README exist")
    check("TelemetryStore" in pi_script and "sqlite3" in pi_script and
          "ThingsBoardPublisher" in pi_script and
          "STM_MSG_TELEMETRY_CHUNK" in pi_script,
          "Pi merges, stores and forwards telemetry with an offline queue")
    check((ROOT / "raspberry_pi/tripguard-4g.peer").is_file() and
          (ROOT / "raspberry_pi/tripguard-4g.chat").is_file() and
          (ROOT / "raspberry_pi/tripguard-4g.service").is_file() and
          (ROOT / "raspberry_pi/tripguard-ppp-dns-route").is_file(),
          "UART3 PPP configuration exists for the Pi-owned 4G modem")
    check((ROOT / "Debug/TripGuard_STM32F411_RTOS.elf").is_file(),
          "STM32 Debug ELF artifact exists")

    print(f"\nALL {passed} CHECKS PASSED")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, UnicodeError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
