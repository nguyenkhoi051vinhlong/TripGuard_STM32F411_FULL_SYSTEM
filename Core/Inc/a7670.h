#ifndef INC_A7670_H_
#define INC_A7670_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MODEM_OFF = 0,
    MODEM_BOOTING,
    MODEM_AT_WAIT,
    MODEM_SIM_WAIT,
    MODEM_NETWORK_WAIT,
    MODEM_PDP_WAIT,
    MODEM_READY,
    MODEM_ERROR
} A7670_State_t;

typedef enum
{
    A7670_COMMAND_IDLE = 0,
    A7670_COMMAND_PENDING,
    A7670_COMMAND_SUCCESS,
    A7670_COMMAND_ERROR,
    A7670_COMMAND_TIMEOUT
} A7670_CommandResult_t;

enum
{
    A7670_EVENT_NONE = 0U,
    A7670_EVENT_NETWORK_LOST = (1UL << 0),
    A7670_EVENT_PDP_DEACTIVATED = (1UL << 1),
    A7670_EVENT_MQTT_LOST = (1UL << 2)
};

#define A7670_MQTT_TOPIC_SIZE    96U
#define A7670_MQTT_PAYLOAD_SIZE 384U

typedef struct
{
    char topic[A7670_MQTT_TOPIC_SIZE];
    char payload[A7670_MQTT_PAYLOAD_SIZE];
} A7670_MqttMessage_t;

void A7670_Init(UART_HandleTypeDef *huart);
void A7670_Task(void);

/* Callback chi dua du lieu DMA vao ring buffer; parser chay trong Task. */
void A7670_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void A7670_ErrorCallback(UART_HandleTypeDef *huart);

A7670_State_t A7670_GetState(void);
uint8_t A7670_IsNetworkReady(void);
uint8_t A7670_IsRegistered(void);
uint8_t A7670_IsPdpActive(void);
int16_t A7670_GetRegistrationStatus(void);
const char *A7670_GetIpAddress(void);

/* Hai ham thap tang nay chi truyen byte, khong doi phan hoi modem. */
uint8_t A7670_SendCommand(const char *cmd);
uint8_t A7670_SendRaw(const uint8_t *data, uint16_t len);

/*
 * Giao dien giao dich AT dung chung cho tb_mqtt.c.
 * expected_token == NULL hoac chuoi rong: doi dong OK.
 * expected_token == ">": doi prompt nhap du lieu.
 * Gia tri khac: doi URC/token tuong ung.
 */
uint8_t A7670_CommandStart(const char *cmd,
                          const char *expected_token,
                          uint32_t timeout_ms);
uint8_t A7670_WaitResponse(const char *expected_token,
                           uint32_t timeout_ms);
A7670_CommandResult_t A7670_GetCommandResult(void);
const char *A7670_GetCommandResponse(void);
void A7670_ClearCommandResult(void);
void A7670_CancelCommand(void);

/*
 * Tam dung may trang thai noi bo de mot chuoi lenh khan cap (SMS/cuoc goi)
 * doc quyen transaction AT. Van phai goi A7670_Task() de parser RX chay.
 */
uint8_t A7670_BeginExclusiveSession(void);
void A7670_EndExclusiveSession(void);
uint8_t A7670_IsExclusiveSessionActive(void);

/* Ban sao chan doan duoc giu lai sau khi transaction hien tai bi xoa. */
const char *A7670_GetLastCommand(void);
const char *A7670_GetLastResponse(void);
A7670_CommandResult_t A7670_GetLastCommandResult(void);
uint32_t A7670_GetCommandTimeoutCount(void);

uint32_t A7670_GetEventFlags(void);
void A7670_ClearEventFlags(uint32_t flags);

/* Lay mot goi MQTT nhan tu URC +CMQTTRX... cua modem. */
uint8_t A7670_TakeMqttMessage(A7670_MqttMessage_t *message);
uint32_t A7670_GetMqttRxDroppedCount(void);

uint32_t A7670_GetRxOverflowCount(void);
uint32_t A7670_GetUartErrorCount(void);
uint32_t A7670_GetLastUartErrorCode(void);
uint32_t A7670_GetUartParityErrorCount(void);
uint32_t A7670_GetUartNoiseErrorCount(void);
uint32_t A7670_GetUartFrameErrorCount(void);
uint32_t A7670_GetUartOverrunErrorCount(void);
uint32_t A7670_GetDmaErrorCount(void);
uint32_t A7670_GetDmaRestartCount(void);
uint32_t A7670_GetDmaRestartFailureCount(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_A7670_H_ */
