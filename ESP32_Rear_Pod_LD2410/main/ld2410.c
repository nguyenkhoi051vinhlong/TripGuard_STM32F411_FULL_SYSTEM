#include "ld2410.h"
#include "rear_config.h"

#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#define MAX_FRAME_DATA 96

typedef enum {
    PARSER_HEADER,
    PARSER_LENGTH_LOW,
    PARSER_LENGTH_HIGH,
    PARSER_DATA,
    PARSER_FOOTER,
} parser_state_t;

static const char *TAG = "ld2410";
static const uint8_t header[] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t footer[] = {0xF8, 0xF7, 0xF6, 0xF5};
static parser_state_t parser_state;
static uint8_t frame_data[MAX_FRAME_DATA];
static uint16_t frame_length;
static uint16_t frame_index;
static uint8_t match_index;
static bool uart_started;

static void parser_reset(void)
{
    parser_state = PARSER_HEADER;
    frame_length = 0;
    frame_index = 0;
    match_index = 0;
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static bool decode_frame(ld2410_measurement_t *measurement)
{
    // Both normal (0x02) and engineering (0x01) reports begin with the
    // same basic target fields: type, 0xAA, state, distances and energies.
    if (frame_length < 13 ||
        (frame_data[0] != 0x01 && frame_data[0] != 0x02) ||
        frame_data[1] != 0xAA ||
        frame_data[frame_length - 2] != 0x55 ||
        frame_data[frame_length - 1] != 0x00) {
        return false;
    }

    uint8_t target_state = frame_data[2] & 0x03;
    *measurement = (ld2410_measurement_t) {
        .valid = true,
        .target = target_state != 0,
        .moving = (target_state & 0x01) != 0,
        .stationary = (target_state & 0x02) != 0,
        .moving_distance_cm = read_u16_le(&frame_data[3]),
        .moving_energy = frame_data[5],
        .stationary_distance_cm = read_u16_le(&frame_data[6]),
        .stationary_energy = frame_data[8],
        .detection_distance_cm = read_u16_le(&frame_data[9]),
    };
    return true;
}

static bool parser_feed(uint8_t byte, ld2410_measurement_t *measurement)
{
    switch (parser_state) {
    case PARSER_HEADER:
        if (byte == header[match_index]) {
            ++match_index;
            if (match_index == sizeof(header)) {
                parser_state = PARSER_LENGTH_LOW;
                match_index = 0;
            }
        } else {
            match_index = byte == header[0] ? 1 : 0;
        }
        break;

    case PARSER_LENGTH_LOW:
        frame_length = byte;
        parser_state = PARSER_LENGTH_HIGH;
        break;

    case PARSER_LENGTH_HIGH:
        frame_length |= (uint16_t)byte << 8;
        if (frame_length == 0 || frame_length > MAX_FRAME_DATA) {
            parser_reset();
        } else {
            frame_index = 0;
            parser_state = PARSER_DATA;
        }
        break;

    case PARSER_DATA:
        frame_data[frame_index++] = byte;
        if (frame_index == frame_length) {
            frame_index = 0;
            parser_state = PARSER_FOOTER;
        }
        break;

    case PARSER_FOOTER:
        if (byte != footer[frame_index]) {
            parser_reset();
            break;
        }
        ++frame_index;
        if (frame_index == sizeof(footer)) {
            bool decoded = decode_frame(measurement);
            parser_reset();
            return decoded;
        }
        break;
    }
    return false;
}

esp_err_t ld2410_uart_start(void)
{
    if (uart_started) return ESP_OK;

    uart_config_t config = {
        .baud_rate = RADAR_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        // Keep 256000 baud independent of CPU/APB frequency scaling while
        // BLE Mesh low-power operation changes the CPU clock dynamically.
        .source_clk = UART_SCLK_XTAL,
    };
    esp_err_t error = uart_driver_install(RADAR_UART, 2048, 0, 0, NULL, 0);
    if (error != ESP_OK) return error;
    error = uart_param_config(RADAR_UART, &config);
    if (error == ESP_OK) {
        error = uart_set_pin(RADAR_UART, RADAR_TX_GPIO, RADAR_RX_GPIO,
                             UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (error != ESP_OK) {
        uart_driver_delete(RADAR_UART);
        return error;
    }

    uart_flush_input(RADAR_UART);
    parser_reset();
    uart_started = true;
    ESP_LOGI(TAG, "UART started at %d baud", RADAR_BAUD_RATE);
    return ESP_OK;
}

void ld2410_uart_stop(void)
{
    if (uart_started) {
        uart_driver_delete(RADAR_UART);
        uart_started = false;
    }

    // High impedance prevents back-powering a radar whose 5 V rail is off.
    gpio_reset_pin(RADAR_TX_GPIO);
    gpio_reset_pin(RADAR_RX_GPIO);
    gpio_set_direction(RADAR_TX_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(RADAR_RX_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(RADAR_TX_GPIO, GPIO_FLOATING);
    gpio_set_pull_mode(RADAR_RX_GPIO, GPIO_FLOATING);
    parser_reset();
}

bool ld2410_uart_is_started(void)
{
    return uart_started;
}

bool ld2410_poll(ld2410_measurement_t *measurement)
{
    if (!uart_started || measurement == NULL) return false;

    uint8_t buffer[128];
    bool got_measurement = false;
    for (;;) {
        int count = uart_read_bytes(RADAR_UART, buffer, sizeof(buffer), 0);
        if (count <= 0) break;
        for (int index = 0; index < count; ++index) {
            ld2410_measurement_t decoded;
            if (parser_feed(buffer[index], &decoded)) {
                *measurement = decoded;
                got_measurement = true;
            }
        }
    }
    return got_measurement;
}
