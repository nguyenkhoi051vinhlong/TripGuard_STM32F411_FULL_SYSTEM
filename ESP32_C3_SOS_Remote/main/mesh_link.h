#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
enum {
    CMD_REAR_CONFIRM = 1,
    CMD_SOS_ACK = 2,
    CMD_LAMP = 3,
    CMD_GET_STATUS = 4,
    CMD_REAR_STOP = 5,
};
enum {EVENT_STATUS=1, EVENT_SOS=2, EVENT_ALERT_DONE=3};
// All messages have an 8-byte payload. See PROTOCOL.md.
typedef struct { uint8_t type; uint32_t id; uint8_t arg; } remote_command_t;
typedef void (*remote_command_cb)(const remote_command_t *cmd);
esp_err_t mesh_link_init(remote_command_cb cb);
esp_err_t mesh_link_send(uint8_t type, uint32_t id, uint8_t flags, uint8_t detail);
bool mesh_link_friend_ready(void);
bool mesh_link_is_provisioned(void);
