#ifndef INC_PI_LINK_H_
#define INC_PI_LINK_H_

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PI_LINK_MAGIC_1                    0x54U
#define PI_LINK_MAGIC_2                    0x50U
#define PI_LINK_VERSION                    0x01U
#define PI_LINK_MAX_PAYLOAD                128U

#define PI_LINK_FLAG_ACK_REQUIRED          0x01U
#define PI_LINK_FLAG_RETRY                 0x02U

#define PI_MSG_HEARTBEAT                   0x01U
#define PI_MSG_READY                       0x02U
#define PI_MSG_STATUS_REQUEST              0x03U
#define PI_MSG_CAMERA_PERSON               0x10U
#define PI_MSG_CAMERA_CLEAR                0x11U
#define PI_MSG_SCAN_COMPLETE               0x12U
#define PI_MSG_ACK                         0x7FU

#define STM_MSG_HEARTBEAT                  0x81U
#define STM_MSG_STATUS                     0x82U
#define STM_MSG_EVENT                      0x83U
#define STM_MSG_TELEMETRY_CHUNK            0x84U
#define STM_MSG_ALERT_REQUEST              0x85U
#define STM_MSG_SCAN_START                 0x86U
#define STM_MSG_SCAN_STOP                  0x87U
#define STM_MSG_ACK                        0xFEU
#define STM_MSG_NACK                       0xFFU

#define PI_SCAN_RESULT_CLEAR               0U
#define PI_SCAN_RESULT_OCCUPIED            1U
#define PI_SCAN_RESULT_FAULT               2U
#define PI_SCAN_RESULT_STOPPED             3U

#define PI_SCAN_STOP_REASON_STANDBY        0U
#define PI_SCAN_STOP_REASON_CANCELLED      1U
#define PI_SCAN_STOP_REASON_SHUTDOWN       2U

#define PI_LINK_MAX_TELEMETRY_JSON          2560U

typedef struct
{
  uint8_t online;
  uint8_t ready;
  uint8_t camera_person;
  uint16_t camera_confidence_permille;
  uint32_t rx_frames;
  uint32_t tx_frames;
  uint32_t crc_errors;
  uint32_t uart_errors;
  uint32_t duplicates;
  uint32_t rx_overflows;
  uint32_t last_packet_age_ms;
} PiLink_Snapshot_t;

uint8_t PiLink_Init(UART_HandleTypeDef *huart, TaskHandle_t owner_task);
void PiLink_Task(void *argument);

/* These hooks are called by the central HAL callbacks in main.c. */
void PiLink_RxEventFromISR(UART_HandleTypeDef *huart, uint16_t size);
void PiLink_TxCompleteFromISR(UART_HandleTypeDef *huart);
void PiLink_ErrorFromISR(UART_HandleTypeDef *huart);

uint8_t PiLink_SendEvent(uint8_t event_code, uint16_t value,
                         uint8_t ack_required);
uint8_t PiLink_QueueTelemetryJson(const char *json, uint16_t length);
uint8_t PiLink_SendAlertRequest(uint8_t alert_kind, uint16_t value);
uint8_t PiLink_RequestScanStart(uint16_t duration_seconds);
uint8_t PiLink_RequestScanStop(uint8_t reason);
void PiLink_ClearCameraOccupancy(void);
void PiLink_GetSnapshot(PiLink_Snapshot_t *snapshot);
uint8_t PiLink_IsOnline(void);
uint8_t PiLink_IsReady(void);
uint8_t PiLink_GetCameraPerson(void);
uint16_t PiLink_GetCameraConfidencePermille(void);

extern volatile uint32_t pi_rx_frame_count;
extern volatile uint32_t pi_tx_frame_count;
extern volatile uint32_t pi_crc_error_count;
extern volatile uint32_t pi_uart_error_count;
extern volatile uint32_t pi_duplicate_count;
extern volatile uint32_t pi_rx_overflow_count;
extern volatile uint32_t pi_last_packet_age_ms;
extern volatile uint8_t pi_online;
extern volatile uint8_t pi_ready;
extern volatile uint8_t pi_camera_person;
extern volatile uint32_t pi_telemetry_queued_count;
extern volatile uint32_t pi_telemetry_sent_count;
extern volatile uint32_t pi_telemetry_drop_count;
extern volatile uint32_t pi_scan_session_id;
extern volatile uint32_t pi_scan_command_sent_count;
extern volatile uint32_t pi_scan_command_ack_count;
extern volatile uint32_t pi_scan_complete_count;
extern volatile uint8_t pi_scan_last_result;

#ifdef __cplusplus
}
#endif

#endif /* INC_PI_LINK_H_ */
