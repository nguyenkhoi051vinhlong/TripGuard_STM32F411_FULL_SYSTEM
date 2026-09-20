#ifndef INC_TB_MQTT_H_
#define INC_TB_MQTT_H_

#include <stdint.h>

/* FullFix V2 API marker: freertos.c uses safe MQTT quiesce before SMS/call. */
#define TB_MQTT_SAFE_QUIESCE_API  2U

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MQTT_IDLE = 0,
    MQTT_START_SERVICE,
    MQTT_ACQUIRE_CLIENT,
    MQTT_CONNECTING,
    MQTT_CONNECTED,
    MQTT_PUBLISHING,
    MQTT_WAIT_RETRY,
    MQTT_SUBSCRIBING,
    MQTT_STOPPING
} MQTT_State_t;

#define TB_RPC_METHOD_SIZE  32U
#define TB_RPC_PARAMS_SIZE  128U

typedef struct
{
    uint32_t id;
    char method[TB_RPC_METHOD_SIZE];
    char params[TB_RPC_PARAMS_SIZE];
} TB_RPC_Command_t;

void TB_MQTT_Init(void);
void TB_MQTT_Task(void);
uint8_t TB_MQTT_IsConnected(void);
uint8_t TB_MQTT_PublishTelemetry(const char *json);
uint8_t TB_MQTT_PublishRpcResponse(uint32_t request_id, const char *json);
uint8_t TB_MQTT_TakeRpcCommand(TB_RPC_Command_t *command);

/*
 * Phien khan cap khong duoc xoa transaction AT dang PENDING.
 * AbortForEmergency chi yeu cau MQTT dung an toan; doi IsQuiesced()==1
 * roi moi bat dau SMS/cuoc goi. Sau do goi ResumeAfterEmergency().
 */
void TB_MQTT_AbortForEmergency(void);
uint8_t TB_MQTT_IsQuiesced(void);
void TB_MQTT_ResumeAfterEmergency(void);

MQTT_State_t TB_MQTT_GetState(void);
uint8_t TB_MQTT_GetPhase(void);
uint32_t TB_MQTT_GetPublishCount(void);
uint32_t TB_MQTT_GetErrorCount(void);
uint32_t TB_MQTT_GetRpcCommandCount(void);
uint32_t TB_MQTT_GetRpcErrorCount(void);
uint8_t TB_MQTT_GetLastCommandResult(void);
MQTT_State_t TB_MQTT_GetLastErrorState(void);
const char *TB_MQTT_GetLastResponse(void);
const char *TB_MQTT_GetLastErrorResponse(void);
const char *TB_MQTT_GetLastRpcMethod(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_TB_MQTT_H_ */
