#pragma once

#include <stdint.h>
#include "esp_err.h"

esp_err_t jq8900_uart_start(void);
void jq8900_uart_stop(void);
esp_err_t jq8900_prepare_and_play(uint16_t track, uint8_t volume);
esp_err_t jq8900_stop_playback(void);
