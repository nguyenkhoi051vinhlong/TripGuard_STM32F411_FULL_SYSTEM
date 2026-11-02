#pragma once
// GPIO numbers, not board header pin numbers. Defaults for an ESP32-C3 Mini.
#define SOS_GPIO 7
#define JQ_RX_GPIO 4       // ESP RX <- JQ8900 TX
#define JQ_TX_GPIO 5       // ESP TX -> JQ8900 RX
#define JQ_BUSY_GPIO 3     // JQ8900 BUSY: high while audio is playing
#define JQ_POWER_GPIO 6    // EN of external 5V load switch; NOT module VCC directly
#define LAMP_GPIO 10       // External lamp driver input; avoid C3 strapping GPIO8
#define LAMP_ACTIVE_LEVEL 1
#define JQ_POWER_ACTIVE_LEVEL 1
#define JQ_BUSY_ACTIVE_LEVEL 1
#define JQ_UART UART_NUM_1
#define JQ_VOLUME 20
#define JQ_REAR_TRACK 1
#define JQ_SOS_TRACK 2
#define JQ_POWERUP_MS 1000LL
#define JQ_BUSY_START_TIMEOUT_MS 3000LL
#define JQ_TRACK_TIMEOUT_MS 240000LL
#define REAR_ALERT_MS 180000LL
#define SOS_DEBOUNCE_MS 40
#define ACTIVE_STATUS_MS 1000LL
#define SOS_RETRY_MS 2000LL
#define IDLE_STATUS_MS 60000LL
// Prototype vendor ID only; replace with your assigned Company ID for production.
#define TG_COMPANY_ID 0x02E5
#define TG_MODEL_ID 0x1001
#define TG_GATEWAY_ADDRESS 0x0001
#define TG_APP_INDEX 0
#define TG_NET_INDEX 0
