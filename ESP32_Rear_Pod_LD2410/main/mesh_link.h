#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

enum {
    CMD_START_SCAN = 1,
    CMD_STOP_SCAN = 2,
    CMD_ACK_EVENT = 3,
    CMD_GET_STATUS = 4,
    CMD_CLEAR_FAULT = 5,
};

enum {
    EVENT_STATUS = 1,
    EVENT_REAR_CONFIRM = 2,
    EVENT_RADAR_UPDATE = 3,
    EVENT_SCAN_DONE = 4,
    EVENT_FAULT = 5,
};

typedef struct {
    uint8_t type;
    uint32_t id;
    uint8_t arg;
} rear_command_t;

typedef void (*rear_command_cb_t)(const rear_command_t *command);

esp_err_t mesh_link_init(rear_command_cb_t callback);
esp_err_t mesh_link_send(uint8_t event, uint32_t id,
                         uint8_t flags, uint8_t detail);
bool mesh_link_friend_ready(void);
bool mesh_link_is_provisioned(void);
