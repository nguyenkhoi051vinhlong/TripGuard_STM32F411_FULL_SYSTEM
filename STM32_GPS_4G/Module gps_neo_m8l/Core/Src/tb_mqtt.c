#include "tb_mqtt.h"

#include "a7670.h"
#include "tripguard_config.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define TB_MQTT_COMMAND_SIZE          384U
#define TB_MQTT_PAYLOAD_SIZE          512U
#define TB_MQTT_COMMAND_TIMEOUT_MS  10000U
#define TB_MQTT_CONNECT_TIMEOUT_MS  70000U
#define TB_MQTT_PUBLISH_TIMEOUT_MS  65000U
#define TB_MQTT_RETRY_MIN_MS         5000U
#define TB_MQTT_RETRY_MAX_MS        60000U

static MQTT_State_t mqtt_state;
static uint8_t mqtt_phase;
static uint8_t mqtt_connected;
static uint8_t mqtt_cleanup_required;
static uint8_t mqtt_publish_pending;

static char mqtt_command[TB_MQTT_COMMAND_SIZE];
static char mqtt_publish_payload[TB_MQTT_PAYLOAD_SIZE];
static uint32_t mqtt_retry_at_ms;
static uint32_t mqtt_retry_delay_ms;
static uint32_t mqtt_publish_count;
static uint32_t mqtt_error_count;

static uint8_t TB_MQTT_TimeReached(uint32_t now, uint32_t deadline)
{
    return (uint8_t)(((int32_t)(now - deadline)) >= 0);
}

static void TB_MQTT_SetState(MQTT_State_t state)
{
    mqtt_state = state;
    mqtt_phase = 0U;
}

static void TB_MQTT_EnterRetry(uint32_t now)
{
    mqtt_connected = 0U;
    mqtt_publish_pending = 0U;
    mqtt_publish_payload[0] = '\0';
    mqtt_cleanup_required = 1U;
    mqtt_error_count++;

    TB_MQTT_SetState(MQTT_WAIT_RETRY);
    mqtt_retry_at_ms = now + mqtt_retry_delay_ms;

    if (mqtt_retry_delay_ms < TB_MQTT_RETRY_MAX_MS)
    {
        mqtt_retry_delay_ms *= 2U;
        if (mqtt_retry_delay_ms > TB_MQTT_RETRY_MAX_MS)
        {
            mqtt_retry_delay_ms = TB_MQTT_RETRY_MAX_MS;
        }
    }
}

static uint8_t TB_MQTT_ResultAvailable(A7670_CommandResult_t *result)
{
    A7670_CommandResult_t current = A7670_GetCommandResult();

    if ((current == A7670_COMMAND_IDLE) ||
        (current == A7670_COMMAND_PENDING))
    {
        return 0U;
    }

    if (result != NULL)
    {
        *result = current;
    }
    return 1U;
}

static void TB_MQTT_TaskStartService(uint32_t now)
{
    A7670_CommandResult_t result;

    if (mqtt_phase == 0U)
    {
        if (A7670_CommandStart("AT+CMQTTSTART",
                              "+CMQTTSTART: 0",
                              TB_MQTT_COMMAND_TIMEOUT_MS) != 0U)
        {
            mqtt_phase = 1U;
        }
    }
    else if ((mqtt_phase == 1U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        A7670_ClearCommandResult();
        if (result == A7670_COMMAND_SUCCESS)
        {
            TB_MQTT_SetState(MQTT_ACQUIRE_CLIENT);
        }
        else
        {
            TB_MQTT_EnterRetry(now);
        }
    }
}

static void TB_MQTT_TaskAcquireClient(uint32_t now)
{
    A7670_CommandResult_t result;

    if (mqtt_phase == 0U)
    {
        (void)snprintf(mqtt_command,
                       sizeof(mqtt_command),
                       "AT+CMQTTACCQ=0,\"%s\",0",
                       TRIPGUARD_TB_CLIENT_ID);
        if (A7670_CommandStart(mqtt_command,
                              NULL,
                              TB_MQTT_COMMAND_TIMEOUT_MS) != 0U)
        {
            mqtt_phase = 1U;
        }
    }
    else if ((mqtt_phase == 1U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        A7670_ClearCommandResult();
        if (result == A7670_COMMAND_SUCCESS)
        {
            TB_MQTT_SetState(MQTT_CONNECTING);
        }
        else
        {
            TB_MQTT_EnterRetry(now);
        }
    }
}

static void TB_MQTT_TaskConnecting(uint32_t now)
{
    A7670_CommandResult_t result;

    if (mqtt_phase == 0U)
    {
        (void)snprintf(
            mqtt_command,
            sizeof(mqtt_command),
            "AT+CMQTTCONNECT=0,\"tcp://%s:%u\",60,1,\"%s\",\"\"",
            TRIPGUARD_TB_HOST,
            (unsigned int)TRIPGUARD_TB_PORT,
            TRIPGUARD_TB_ACCESS_TOKEN);

        if (A7670_CommandStart(mqtt_command,
                              "+CMQTTCONNECT: 0,0",
                              TB_MQTT_CONNECT_TIMEOUT_MS) != 0U)
        {
            mqtt_phase = 1U;
        }
    }
    else if ((mqtt_phase == 1U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        A7670_ClearCommandResult();
        if (result == A7670_COMMAND_SUCCESS)
        {
            mqtt_connected = 1U;
            mqtt_cleanup_required = 1U;
            mqtt_retry_delay_ms = TB_MQTT_RETRY_MIN_MS;
            TB_MQTT_SetState(MQTT_CONNECTED);
        }
        else
        {
            TB_MQTT_EnterRetry(now);
        }
    }
}

static void TB_MQTT_PublishFailed(uint32_t now)
{
    A7670_ClearCommandResult();
    TB_MQTT_EnterRetry(now);
}

static void TB_MQTT_TaskPublishing(uint32_t now)
{
    A7670_CommandResult_t result;
    size_t length;

    if (mqtt_phase == 0U)
    {
        length = strlen(TRIPGUARD_TB_TELEMETRY_TOPIC);
        (void)snprintf(mqtt_command,
                       sizeof(mqtt_command),
                       "AT+CMQTTTOPIC=0,%u",
                       (unsigned int)length);
        if (A7670_CommandStart(mqtt_command,
                              ">",
                              TB_MQTT_COMMAND_TIMEOUT_MS) != 0U)
        {
            mqtt_phase = 1U;
        }
    }
    else if ((mqtt_phase == 1U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        A7670_ClearCommandResult();
        if (result != A7670_COMMAND_SUCCESS)
        {
            TB_MQTT_EnterRetry(now);
            return;
        }

        length = strlen(TRIPGUARD_TB_TELEMETRY_TOPIC);
        if ((A7670_SendRaw(
                 (const uint8_t *)TRIPGUARD_TB_TELEMETRY_TOPIC,
                 (uint16_t)length) == 0U) ||
            (A7670_WaitResponse(NULL,
                                TB_MQTT_COMMAND_TIMEOUT_MS) == 0U))
        {
            TB_MQTT_EnterRetry(now);
            return;
        }
        mqtt_phase = 2U;
    }
    else if ((mqtt_phase == 2U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        if (result != A7670_COMMAND_SUCCESS)
        {
            TB_MQTT_PublishFailed(now);
            return;
        }

        A7670_ClearCommandResult();
        length = strlen(mqtt_publish_payload);
        (void)snprintf(mqtt_command,
                       sizeof(mqtt_command),
                       "AT+CMQTTPAYLOAD=0,%u",
                       (unsigned int)length);
        if (A7670_CommandStart(mqtt_command,
                              ">",
                              TB_MQTT_COMMAND_TIMEOUT_MS) == 0U)
        {
            TB_MQTT_EnterRetry(now);
            return;
        }
        mqtt_phase = 3U;
    }
    else if ((mqtt_phase == 3U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        A7670_ClearCommandResult();
        if (result != A7670_COMMAND_SUCCESS)
        {
            TB_MQTT_EnterRetry(now);
            return;
        }

        length = strlen(mqtt_publish_payload);
        if ((A7670_SendRaw((const uint8_t *)mqtt_publish_payload,
                           (uint16_t)length) == 0U) ||
            (A7670_WaitResponse(NULL,
                                TB_MQTT_COMMAND_TIMEOUT_MS) == 0U))
        {
            TB_MQTT_EnterRetry(now);
            return;
        }
        mqtt_phase = 4U;
    }
    else if ((mqtt_phase == 4U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        if (result != A7670_COMMAND_SUCCESS)
        {
            TB_MQTT_PublishFailed(now);
            return;
        }

        A7670_ClearCommandResult();
        if (A7670_CommandStart("AT+CMQTTPUB=0,0,60",
                              "+CMQTTPUB: 0,0",
                              TB_MQTT_PUBLISH_TIMEOUT_MS) == 0U)
        {
            TB_MQTT_EnterRetry(now);
            return;
        }
        mqtt_phase = 5U;
    }
    else if ((mqtt_phase == 5U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        A7670_ClearCommandResult();
        if (result == A7670_COMMAND_SUCCESS)
        {
            mqtt_publish_pending = 0U;
            mqtt_publish_payload[0] = '\0';
            mqtt_publish_count++;
            TB_MQTT_SetState(MQTT_CONNECTED);
        }
        else
        {
            TB_MQTT_EnterRetry(now);
        }
    }
}

static void TB_MQTT_TaskCleanup(uint32_t now)
{
    A7670_CommandResult_t result;

    if (mqtt_phase == 0U)
    {
        if (TB_MQTT_TimeReached(now, mqtt_retry_at_ms) == 0U)
        {
            return;
        }

        if (mqtt_cleanup_required == 0U)
        {
            TB_MQTT_SetState(MQTT_START_SERVICE);
            return;
        }

        if (A7670_CommandStart("AT+CMQTTDISC=0,60",
                              "+CMQTTDISC: 0,0",
                              TB_MQTT_PUBLISH_TIMEOUT_MS) != 0U)
        {
            mqtt_phase = 1U;
        }
    }
    else if ((mqtt_phase == 1U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        (void)result;
        A7670_ClearCommandResult();
        if (A7670_CommandStart("AT+CMQTTREL=0",
                              NULL,
                              TB_MQTT_COMMAND_TIMEOUT_MS) != 0U)
        {
            mqtt_phase = 2U;
        }
    }
    else if ((mqtt_phase == 2U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        (void)result;
        A7670_ClearCommandResult();
        if (A7670_CommandStart("AT+CMQTTSTOP",
                              "+CMQTTSTOP: 0",
                              TB_MQTT_COMMAND_TIMEOUT_MS) != 0U)
        {
            mqtt_phase = 3U;
        }
    }
    else if ((mqtt_phase == 3U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        (void)result;
        A7670_ClearCommandResult();
        mqtt_cleanup_required = 0U;
        TB_MQTT_SetState(MQTT_START_SERVICE);
    }
}

void TB_MQTT_Init(void)
{
    mqtt_state = MQTT_IDLE;
    mqtt_phase = 0U;
    mqtt_connected = 0U;
    mqtt_cleanup_required = 0U;
    mqtt_publish_pending = 0U;
    mqtt_command[0] = '\0';
    mqtt_publish_payload[0] = '\0';
    mqtt_retry_at_ms = 0U;
    mqtt_retry_delay_ms = TB_MQTT_RETRY_MIN_MS;
    mqtt_publish_count = 0U;
    mqtt_error_count = 0U;
}

void TB_MQTT_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t events = A7670_GetEventFlags();

    if ((events & A7670_EVENT_MQTT_LOST) != 0U)
    {
        A7670_ClearEventFlags(A7670_EVENT_MQTT_LOST);
        if (A7670_IsNetworkReady() != 0U)
        {
            A7670_CancelCommand();
            TB_MQTT_EnterRetry(now);
        }
    }

    if (A7670_IsNetworkReady() == 0U)
    {
        mqtt_connected = 0U;
        if (mqtt_state != MQTT_IDLE)
        {
            mqtt_cleanup_required = 1U;
            mqtt_publish_pending = 0U;
            mqtt_publish_payload[0] = '\0';
            TB_MQTT_SetState(MQTT_IDLE);
        }
        return;
    }

    if (mqtt_state == MQTT_IDLE)
    {
        if (mqtt_cleanup_required != 0U)
        {
            TB_MQTT_SetState(MQTT_WAIT_RETRY);
            mqtt_retry_at_ms = now + 1000U;
        }
        else
        {
            TB_MQTT_SetState(MQTT_START_SERVICE);
        }
    }

    switch (mqtt_state)
    {
        case MQTT_START_SERVICE:
            TB_MQTT_TaskStartService(now);
            break;

        case MQTT_ACQUIRE_CLIENT:
            TB_MQTT_TaskAcquireClient(now);
            break;

        case MQTT_CONNECTING:
            TB_MQTT_TaskConnecting(now);
            break;

        case MQTT_CONNECTED:
            if (mqtt_publish_pending != 0U)
            {
                TB_MQTT_SetState(MQTT_PUBLISHING);
            }
            break;

        case MQTT_PUBLISHING:
            TB_MQTT_TaskPublishing(now);
            break;

        case MQTT_WAIT_RETRY:
            TB_MQTT_TaskCleanup(now);
            break;

        case MQTT_IDLE:
        default:
            break;
    }
}

uint8_t TB_MQTT_IsConnected(void)
{
    return (uint8_t)((mqtt_connected != 0U) &&
                     (A7670_IsNetworkReady() != 0U));
}

uint8_t TB_MQTT_PublishTelemetry(const char *json)
{
    size_t length;

    if ((json == NULL) ||
        (TB_MQTT_IsConnected() == 0U) ||
        (mqtt_publish_pending != 0U))
    {
        return 0U;
    }

    length = strlen(json);
    if ((length == 0U) || (length >= sizeof(mqtt_publish_payload)))
    {
        return 0U;
    }

    memcpy(mqtt_publish_payload, json, length + 1U);
    mqtt_publish_pending = 1U;
    return 1U;
}

MQTT_State_t TB_MQTT_GetState(void)
{
    return mqtt_state;
}

uint32_t TB_MQTT_GetPublishCount(void)
{
    return mqtt_publish_count;
}

uint32_t TB_MQTT_GetErrorCount(void)
{
    return mqtt_error_count;
}
