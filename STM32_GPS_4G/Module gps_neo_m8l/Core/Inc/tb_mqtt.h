#ifndef INC_TB_MQTT_H_
#define INC_TB_MQTT_H_

#include <stdint.h>

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
    MQTT_WAIT_RETRY
} MQTT_State_t;

void TB_MQTT_Init(void);
void TB_MQTT_Task(void);
uint8_t TB_MQTT_IsConnected(void);
uint8_t TB_MQTT_PublishTelemetry(const char *json);

MQTT_State_t TB_MQTT_GetState(void);
uint32_t TB_MQTT_GetPublishCount(void);
uint32_t TB_MQTT_GetErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_TB_MQTT_H_ */
