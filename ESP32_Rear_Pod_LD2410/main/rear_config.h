#pragma once

#include "driver/uart.h"

// GPIO numbers, not the printed header-pin positions on the board.
#define REAR_BUTTON_GPIO       3
#define RADAR_OUT_GPIO         0
#define RADAR_RX_GPIO          4  // ESP RX <- LD2410C TX
#define RADAR_TX_GPIO          5  // ESP TX -> LD2410C RX
#define RADAR_POWER_GPIO      10  // EN of external 5 V load switch
#define RADAR_UART    UART_NUM_1

#define RADAR_POWER_ACTIVE_LEVEL 1
#define RADAR_BAUD_RATE       256000
#define RADAR_WARMUP_MS       2000LL
#define RADAR_FIRST_FRAME_MS  5000LL
#define RADAR_FRAME_STALE_MS  3000LL
#define DEFAULT_SCAN_MS       300000LL
#define MAX_SCAN_MINUTES      30

#define BUTTON_PRESS_MS          30LL
#define BUTTON_RELEASE_MS        80LL
#define ACTIVE_REPORT_MS       1000LL
#define IDLE_STATUS_MS        60000LL
#define EVENT_RETRY_MS         2000LL

// Prototype vendor ID only. Replace it with an assigned Company ID for production.
#define TG_COMPANY_ID          0x02E5
#define TG_MODEL_ID            0x1002
#define TG_GATEWAY_ADDRESS     0x0001
#define TG_APP_INDEX           0
#define TG_NET_INDEX           0
