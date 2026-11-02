#pragma once

#include "driver/spi_master.h"

#define TG_COMPANY_ID             0x02E5
#define TG_GATEWAY_MODEL_ID       0x1000
#define TG_SOS_MODEL_ID           0x1001
#define TG_REAR_MODEL_ID          0x1002

#define TG_GATEWAY_ADDRESS        0x0001
#define TG_FIRST_NODE_ADDRESS     0x0100
#define TG_NET_INDEX              0x0000
#define TG_APP_INDEX              0x0000
#define TG_PROTOCOL_VERSION       3

#define STM32_SPI_HOST            SPI3_HOST
#define STM32_SPI_SCK_GPIO        18
#define STM32_SPI_MOSI_GPIO       23
#define STM32_SPI_CS_GPIO         17
#define STM32_SPI_CLOCK_HZ        1000000
#define STM32_SPI_MODE            0

#define STM32_FRAME_MAGIC         0xA5
#define STM32_FRAME_END           0x5A
#define STM32_PROTOCOL_VERSION    0x01
#define STM32_FRAME_SIZE          12
#define STM32_NODE_REMOTE         0x02
#define STM32_EVENT_REAR_CONFIRM  0x10
#define STM32_EVENT_PERSON        0x11
#define STM32_EVENT_RADAR_CLEAR   0x12
#define STM32_EVENT_RADAR_HEALTH  0x13
#define STM32_EVENT_REMOTE_SOS    0x14
#define SPI_CRITICAL_REPEATS      3
#define SPI_REPEAT_GAP_MS         50

#define REAR_FLAG_TARGET          (1U << 2)
#define REAR_FLAG_RADAR_FAULT     (1U << 6)

#define MAX_TRACKED_NODES         8

enum {
    TG_EVENT_STATUS = 1,
    TG_EVENT_PRIMARY = 2,
    TG_EVENT_UPDATE_OR_DONE = 3,
    TG_EVENT_SCAN_DONE = 4,
    TG_EVENT_FAULT = 5,
};

enum {
    REAR_CMD_START_SCAN = 1,
    REAR_CMD_STOP_SCAN = 2,
    REAR_CMD_ACK_EVENT = 3,
    REAR_CMD_GET_STATUS = 4,
    REAR_CMD_CLEAR_FAULT = 5,
};

enum {
    SOS_CMD_REAR_CONFIRM = 1,
    SOS_CMD_SOS_ACK = 2,
    SOS_CMD_LAMP = 3,
    SOS_CMD_GET_STATUS = 4,
    SOS_CMD_REAR_STOP = 5,
};
