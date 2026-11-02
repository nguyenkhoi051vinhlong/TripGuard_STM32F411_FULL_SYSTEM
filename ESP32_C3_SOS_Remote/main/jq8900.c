#include "jq8900.h"
#include "remote_config.h"
#include <stdbool.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool uart_started;

static esp_err_t send_command(uint8_t command, const uint8_t *data, uint8_t data_len)
{
    uint8_t frame[12] = {0xAA, command, data_len};
    if (data_len > sizeof(frame) - 4) return ESP_ERR_INVALID_SIZE;
    uint8_t checksum = (uint8_t)(0xAA + command + data_len);
    for (uint8_t i = 0; i < data_len; ++i) {
        frame[3 + i] = data[i];
        checksum = (uint8_t)(checksum + data[i]);
    }
    frame[3 + data_len] = checksum;
    int length = 4 + data_len;
    int written = uart_write_bytes(JQ_UART, frame, length);
    if (written != length) return ESP_FAIL;
    return uart_wait_tx_done(JQ_UART, pdMS_TO_TICKS(100));
}

esp_err_t jq8900_uart_start(void)
{
    if (uart_started) return ESP_OK;
    uart_config_t config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(JQ_UART, 256, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    err = uart_param_config(JQ_UART, &config);
    if (err == ESP_OK) {
        err = uart_set_pin(JQ_UART, JQ_TX_GPIO, JQ_RX_GPIO,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        uart_driver_delete(JQ_UART);
        return err;
    }
    uart_started = true;
    return ESP_OK;
}

void jq8900_uart_stop(void)
{
    if (!uart_started) return;
    uart_driver_delete(JQ_UART);
    uart_started = false;
}

esp_err_t jq8900_prepare_and_play(uint16_t track, uint8_t volume)
{
    if (!uart_started || track == 0 || volume > 30) return ESP_ERR_INVALID_ARG;

    const uint8_t flash_disk = 0x02;
    esp_err_t err = send_command(0x0B, &flash_disk, 1);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    err = send_command(0x13, &volume, 1);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t track_data[2] = {(uint8_t)(track >> 8), (uint8_t)track};
    return send_command(0x07, track_data, 2);
}

esp_err_t jq8900_stop_playback(void)
{
    if (!uart_started) return ESP_ERR_INVALID_STATE;
    return send_command(0x04, NULL, 0);
}
