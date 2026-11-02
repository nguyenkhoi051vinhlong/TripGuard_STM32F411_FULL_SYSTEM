#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    bool valid;
    bool target;
    bool moving;
    bool stationary;
    uint16_t moving_distance_cm;
    uint16_t stationary_distance_cm;
    uint16_t detection_distance_cm;
    uint8_t moving_energy;
    uint8_t stationary_energy;
} ld2410_measurement_t;

esp_err_t ld2410_uart_start(void);
void ld2410_uart_stop(void);
bool ld2410_uart_is_started(void);
bool ld2410_poll(ld2410_measurement_t *measurement);
