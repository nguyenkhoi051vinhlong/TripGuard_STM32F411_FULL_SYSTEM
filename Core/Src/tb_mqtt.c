#include "tb_mqtt.h"

#include "a7670.h"
#include "tripguard_config.h"

#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * ThingsBoard MQTT 3.1.1 qua MQTT engine tich hop trong SIMCom A76XX.
 * Mot thoi diem chi co mot giao dich AT. Phien SMS/cuoc goi chiem modem
 * sau khi TB_MQTT_IsQuiesced() bao da dung MQTT an toan.
 */
#define TB_MQTT_CLIENT_INDEX             0U
#define TB_MQTT_COMMAND_SIZE           384U
#define TB_MQTT_PAYLOAD_SIZE          2560U
#define TB_MQTT_RESPONSE_SIZE          768U
#define TB_MQTT_COMMAND_TIMEOUT_MS   10000U
#define TB_MQTT_CONNECT_TIMEOUT_MS   90000U
#define TB_MQTT_PUBLISH_TIMEOUT_MS   60000U
#define TB_MQTT_CLEANUP_TIMEOUT_MS    8000U
#define TB_MQTT_PROMPT_SETTLE_MS       100U
#define TB_MQTT_RETRY_MIN_MS          3000U
#define TB_MQTT_RETRY_MAX_MS         60000U
#define TB_MQTT_RPC_REQUEST_TOPIC     "v1/devices/me/rpc/request/+"
#define TB_MQTT_RPC_REQUEST_PREFIX    "v1/devices/me/rpc/request/"
#define TB_MQTT_RPC_RESPONSE_PREFIX   "v1/devices/me/rpc/response/"
#define TB_MQTT_TOPIC_SIZE             96U

static MQTT_State_t mqtt_state;
static uint8_t mqtt_phase;
static uint8_t mqtt_connected;
static uint8_t mqtt_service_started;
static uint8_t mqtt_client_acquired;
static uint8_t mqtt_subscribed;
static uint8_t mqtt_cleanup_forced;
static uint8_t mqtt_publish_pending;
static uint8_t mqtt_abort_requested;
static uint8_t mqtt_quiesced;
static uint8_t mqtt_rpc_pending;

static char mqtt_command[TB_MQTT_COMMAND_SIZE];
static char mqtt_publish_topic[TB_MQTT_TOPIC_SIZE];
static char mqtt_publish_payload[TB_MQTT_PAYLOAD_SIZE];
static TB_RPC_Command_t mqtt_rpc_command;
static uint32_t mqtt_retry_at_ms;
static uint32_t mqtt_retry_delay_ms;
static uint32_t mqtt_prompt_ready_at_ms;
static uint32_t mqtt_publish_count;
static uint32_t mqtt_error_count;
static uint32_t mqtt_rpc_command_count;
static uint32_t mqtt_rpc_error_count;
static uint32_t mqtt_modem_rx_drop_snapshot;
static uint8_t mqtt_last_command_result;
static MQTT_State_t mqtt_last_error_state;
static char mqtt_last_response[TB_MQTT_RESPONSE_SIZE];
static char mqtt_last_error_response[TB_MQTT_RESPONSE_SIZE];
static char mqtt_last_rpc_method[TB_RPC_METHOD_SIZE];

static void TB_MQTT_CopyText(char *destination, size_t destination_size,
                             const char *source)
{
    if ((destination == NULL) || (destination_size == 0U))
    {
        return;
    }
    if (source == NULL)
    {
        destination[0] = '\0';
        return;
    }
    strncpy(destination, source, destination_size - 1U);
    destination[destination_size - 1U] = '\0';
}

static uint8_t TB_MQTT_TimeReached(uint32_t now, uint32_t deadline)
{
    return (uint8_t)(((int32_t)(now - deadline)) >= 0);
}

static void TB_MQTT_SetState(MQTT_State_t state)
{
    mqtt_state = state;
    mqtt_phase = 0U;
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

    mqtt_last_command_result = (uint8_t)current;
    TB_MQTT_CopyText(mqtt_last_response, sizeof(mqtt_last_response),
                     A7670_GetCommandResponse());
    if (current != A7670_COMMAND_SUCCESS)
    {
        TB_MQTT_CopyText(mqtt_last_error_response,
                         sizeof(mqtt_last_error_response),
                         A7670_GetCommandResponse());
    }
    return 1U;
}

static void TB_MQTT_EnterRetry(uint32_t now)
{
    mqtt_last_error_state = mqtt_state;
    if (mqtt_last_error_response[0] == '\0')
    {
        TB_MQTT_CopyText(mqtt_last_error_response,
                         sizeof(mqtt_last_error_response),
                         "MQTT state failed without a modem response");
    }

    mqtt_connected = 0U;
    mqtt_subscribed = 0U;
    mqtt_cleanup_forced = 1U;
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

static const char *TB_MQTT_FindJsonValue(const char *json, const char *key)
{
    char pattern[48];
    const char *cursor;

    if ((json == NULL) || (key == NULL))
    {
        return NULL;
    }
    if (snprintf(pattern, sizeof(pattern), "\"%s\"", key) <= 0)
    {
        return NULL;
    }
    cursor = strstr(json, pattern);
    if (cursor == NULL)
    {
        return NULL;
    }
    cursor += strlen(pattern);
    while (isspace((unsigned char)*cursor) != 0)
    {
        cursor++;
    }
    if (*cursor != ':')
    {
        return NULL;
    }
    cursor++;
    while (isspace((unsigned char)*cursor) != 0)
    {
        cursor++;
    }
    return cursor;
}

static uint8_t TB_MQTT_CopyJsonString(const char *value,
                                      char *destination,
                                      size_t destination_size)
{
    size_t used = 0U;
    uint8_t escaped = 0U;

    if ((value == NULL) || (destination == NULL) ||
        (destination_size == 0U) || (*value != '"'))
    {
        return 0U;
    }
    value++;
    while (*value != '\0')
    {
        char current = *value++;

        if ((current == '"') && (escaped == 0U))
        {
            destination[used] = '\0';
            return 1U;
        }
        if ((used + 1U) >= destination_size)
        {
            return 0U;
        }
        destination[used++] = current;
        if ((current == '\\') && (escaped == 0U))
        {
            escaped = 1U;
        }
        else
        {
            escaped = 0U;
        }
    }
    return 0U;
}

static uint8_t TB_MQTT_CopyJsonValue(const char *value,
                                     char *destination,
                                     size_t destination_size)
{
    size_t used = 0U;
    int depth = 0;
    char opener = '\0';
    char closer = '\0';
    uint8_t in_string = 0U;
    uint8_t escaped = 0U;

    if ((value == NULL) || (destination == NULL) ||
        (destination_size == 0U))
    {
        return 0U;
    }
    if ((*value == '{') || (*value == '['))
    {
        opener = *value;
        closer = (*value == '{') ? '}' : ']';
    }
    else if (*value == '"')
    {
        opener = '"';
        closer = '"';
    }

    while (*value != '\0')
    {
        char current = *value++;

        if ((used + 1U) >= destination_size)
        {
            return 0U;
        }
        destination[used++] = current;
        if (opener == '"')
        {
            if ((current == closer) && (used > 1U) && (escaped == 0U))
            {
                break;
            }
            escaped = (uint8_t)((current == '\\') && (escaped == 0U));
        }
        else if ((opener == '{') || (opener == '['))
        {
            if ((current == '"') && (escaped == 0U))
            {
                in_string = (uint8_t)(in_string == 0U);
            }
            escaped = (uint8_t)((current == '\\') &&
                                (in_string != 0U) &&
                                (escaped == 0U));
            if (in_string == 0U)
            {
                if (current == opener)
                {
                    depth++;
                }
                else if (current == closer)
                {
                    depth--;
                    if (depth == 0)
                    {
                        break;
                    }
                }
            }
        }
        else if ((current == ',') || (current == '}') ||
                 (isspace((unsigned char)current) != 0))
        {
            used--;
            break;
        }
    }

    while ((used > 0U) &&
           (isspace((unsigned char)destination[used - 1U]) != 0))
    {
        used--;
    }
    destination[used] = '\0';
    return (uint8_t)(used > 0U);
}

static uint8_t TB_MQTT_ParseRpcMessage(const A7670_MqttMessage_t *message,
                                       TB_RPC_Command_t *command)
{
    const char *id_text;
    const char *value;
    char *end_pointer;
    unsigned long request_id;
    size_t prefix_length = strlen(TB_MQTT_RPC_REQUEST_PREFIX);

    if ((message == NULL) || (command == NULL) ||
        (strncmp(message->topic, TB_MQTT_RPC_REQUEST_PREFIX,
                 prefix_length) != 0))
    {
        return 0U;
    }
    id_text = &message->topic[prefix_length];
    request_id = strtoul(id_text, &end_pointer, 10);
    if ((end_pointer == id_text) || (*end_pointer != '\0'))
    {
        return 0U;
    }

    memset(command, 0, sizeof(*command));
    command->id = (uint32_t)request_id;
    value = TB_MQTT_FindJsonValue(message->payload, "method");
    if (TB_MQTT_CopyJsonString(value, command->method,
                               sizeof(command->method)) == 0U)
    {
        return 0U;
    }
    value = TB_MQTT_FindJsonValue(message->payload, "params");
    if (value == NULL)
    {
        command->params[0] = '\0';
    }
    else if (TB_MQTT_CopyJsonValue(value, command->params,
                                   sizeof(command->params)) == 0U)
    {
        return 0U;
    }
    return 1U;
}

static void TB_MQTT_ProcessIncoming(void)
{
    A7670_MqttMessage_t message;
    uint32_t dropped = A7670_GetMqttRxDroppedCount();

    if (dropped != mqtt_modem_rx_drop_snapshot)
    {
        mqtt_rpc_error_count += dropped - mqtt_modem_rx_drop_snapshot;
        mqtt_modem_rx_drop_snapshot = dropped;
    }

    if (A7670_TakeMqttMessage(&message) == 0U)
    {
        return;
    }
    if (mqtt_rpc_pending != 0U)
    {
        mqtt_rpc_error_count++;
        return;
    }
    if (TB_MQTT_ParseRpcMessage(&message, &mqtt_rpc_command) == 0U)
    {
        mqtt_rpc_error_count++;
        return;
    }

    mqtt_rpc_pending = 1U;
    mqtt_rpc_command_count++;
    TB_MQTT_CopyText(mqtt_last_rpc_method, sizeof(mqtt_last_rpc_method),
                     mqtt_rpc_command.method);
}

static void TB_MQTT_TaskStartService(uint32_t now)
{
    A7670_CommandResult_t result;

    if (mqtt_phase == 0U)
    {
        if (A7670_CommandStart("AT+CMQTTSTART", "+CMQTTSTART: 0",
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
            mqtt_service_started = 1U;
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
        (void)snprintf(mqtt_command, sizeof(mqtt_command),
                       "AT+CMQTTACCQ=%u,\"%s\",0",
                       (unsigned int)TB_MQTT_CLIENT_INDEX,
                       TRIPGUARD_TB_CLIENT_ID);
        if (A7670_CommandStart(mqtt_command, NULL,
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
            mqtt_client_acquired = 1U;
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
            mqtt_command, sizeof(mqtt_command),
            "AT+CMQTTCONNECT=%u,\"tcp://%s:%u\",%u,%u,\"%s\"",
            (unsigned int)TB_MQTT_CLIENT_INDEX,
            TRIPGUARD_TB_HOST,
            (unsigned int)TRIPGUARD_TB_PORT,
            (unsigned int)TRIPGUARD_TB_KEEPALIVE_SECONDS,
            (unsigned int)TRIPGUARD_TB_CLEAN_SESSION,
            TRIPGUARD_TB_ACCESS_TOKEN);
        if (A7670_CommandStart(mqtt_command, "+CMQTTCONNECT: 0,0",
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
            TB_MQTT_SetState(MQTT_SUBSCRIBING);
        }
        else
        {
            TB_MQTT_EnterRetry(now);
        }
    }
}

static void TB_MQTT_TaskSubscribing(uint32_t now)
{
    A7670_CommandResult_t result;
    size_t topic_length = strlen(TB_MQTT_RPC_REQUEST_TOPIC);

    if (mqtt_phase == 0U)
    {
        (void)snprintf(mqtt_command, sizeof(mqtt_command),
                       "AT+CMQTTSUBTOPIC=%u,%u,1",
                       (unsigned int)TB_MQTT_CLIENT_INDEX,
                       (unsigned int)topic_length);
        if (A7670_CommandStart(mqtt_command, ">",
                              TB_MQTT_COMMAND_TIMEOUT_MS) != 0U)
        {
            mqtt_phase = 1U;
        }
    }
    else if ((mqtt_phase == 1U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        if (result != A7670_COMMAND_SUCCESS)
        {
            A7670_ClearCommandResult();
            TB_MQTT_EnterRetry(now);
            return;
        }
        /* Giu transaction o SUCCESS de chan health-poll chen vao prompt. */
        mqtt_prompt_ready_at_ms = now + TB_MQTT_PROMPT_SETTLE_MS;
        mqtt_phase = 2U;
    }
    else if ((mqtt_phase == 2U) &&
             (TB_MQTT_TimeReached(now, mqtt_prompt_ready_at_ms) != 0U))
    {
        A7670_ClearCommandResult();
        if (A7670_WaitResponse(NULL, TB_MQTT_COMMAND_TIMEOUT_MS) == 0U)
        {
            TB_MQTT_EnterRetry(now);
            return;
        }
        mqtt_phase = 3U;
        /*
         * Mo transaction nhan truoc khi TX raw. Neu UART TX that bai giua
         * chung, transaction se timeout co kiem soat thay vi chen lenh cleanup
         * trong khi modem van dang doi du byte topic.
         */
        (void)A7670_SendRaw((const uint8_t *)TB_MQTT_RPC_REQUEST_TOPIC,
                            (uint16_t)topic_length);
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
        if (mqtt_abort_requested != 0U)
        {
            mqtt_phase = 0U;
            return;
        }
        if (A7670_CommandStart("AT+CMQTTSUB=0", "+CMQTTSUB: 0,0",
                              TB_MQTT_PUBLISH_TIMEOUT_MS) != 0U)
        {
            mqtt_phase = 4U;
        }
    }
    else if ((mqtt_phase == 4U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        A7670_ClearCommandResult();
        if (result == A7670_COMMAND_SUCCESS)
        {
            mqtt_subscribed = 1U;
            mqtt_retry_delay_ms = TB_MQTT_RETRY_MIN_MS;
            TB_MQTT_SetState(MQTT_CONNECTED);
        }
        else
        {
            TB_MQTT_EnterRetry(now);
        }
    }
}

static void TB_MQTT_TaskPublishing(uint32_t now)
{
    A7670_CommandResult_t result;
    size_t topic_length = strlen(mqtt_publish_topic);
    size_t payload_length = strlen(mqtt_publish_payload);

    if (mqtt_phase == 0U)
    {
        (void)snprintf(mqtt_command, sizeof(mqtt_command),
                       "AT+CMQTTTOPIC=%u,%u",
                       (unsigned int)TB_MQTT_CLIENT_INDEX,
                       (unsigned int)topic_length);
        if (A7670_CommandStart(mqtt_command, ">",
                              TB_MQTT_COMMAND_TIMEOUT_MS) != 0U)
        {
            mqtt_phase = 1U;
        }
    }
    else if ((mqtt_phase == 1U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        if (result != A7670_COMMAND_SUCCESS)
        {
            A7670_ClearCommandResult();
            TB_MQTT_EnterRetry(now);
            return;
        }
        /* Giu transaction o SUCCESS de chan health-poll chen vao prompt. */
        mqtt_prompt_ready_at_ms = now + TB_MQTT_PROMPT_SETTLE_MS;
        mqtt_phase = 2U;
    }
    else if ((mqtt_phase == 2U) &&
             (TB_MQTT_TimeReached(now, mqtt_prompt_ready_at_ms) != 0U))
    {
        A7670_ClearCommandResult();
        if (A7670_WaitResponse(NULL, TB_MQTT_COMMAND_TIMEOUT_MS) == 0U)
        {
            TB_MQTT_EnterRetry(now);
            return;
        }
        mqtt_phase = 3U;
        (void)A7670_SendRaw((const uint8_t *)mqtt_publish_topic,
                            (uint16_t)topic_length);
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
        mqtt_phase = 4U;
    }
    else if (mqtt_phase == 4U)
    {
        (void)snprintf(mqtt_command, sizeof(mqtt_command),
                       "AT+CMQTTPAYLOAD=%u,%u",
                       (unsigned int)TB_MQTT_CLIENT_INDEX,
                       (unsigned int)payload_length);
        if (A7670_CommandStart(mqtt_command, ">",
                              TB_MQTT_COMMAND_TIMEOUT_MS) != 0U)
        {
            mqtt_phase = 5U;
        }
    }
    else if ((mqtt_phase == 5U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        if (result != A7670_COMMAND_SUCCESS)
        {
            A7670_ClearCommandResult();
            TB_MQTT_EnterRetry(now);
            return;
        }
        /* Giu transaction o SUCCESS de chan health-poll chen vao prompt. */
        mqtt_prompt_ready_at_ms = now + TB_MQTT_PROMPT_SETTLE_MS;
        mqtt_phase = 6U;
    }
    else if ((mqtt_phase == 6U) &&
             (TB_MQTT_TimeReached(now, mqtt_prompt_ready_at_ms) != 0U))
    {
        A7670_ClearCommandResult();
        if (A7670_WaitResponse(NULL, TB_MQTT_COMMAND_TIMEOUT_MS) == 0U)
        {
            TB_MQTT_EnterRetry(now);
            return;
        }
        mqtt_phase = 7U;
        (void)A7670_SendRaw((const uint8_t *)mqtt_publish_payload,
                            (uint16_t)payload_length);
    }
    else if ((mqtt_phase == 7U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        A7670_ClearCommandResult();
        if (result != A7670_COMMAND_SUCCESS)
        {
            TB_MQTT_EnterRetry(now);
            return;
        }
        mqtt_phase = 8U;
    }
    else if (mqtt_phase == 8U)
    {
        (void)snprintf(mqtt_command, sizeof(mqtt_command),
                       "AT+CMQTTPUB=%u,%u,60,0,0",
                       (unsigned int)TB_MQTT_CLIENT_INDEX,
                       (unsigned int)TRIPGUARD_TB_PUBLISH_QOS);
        if (A7670_CommandStart(mqtt_command, "+CMQTTPUB: 0,0",
                              TB_MQTT_PUBLISH_TIMEOUT_MS) != 0U)
        {
            mqtt_phase = 9U;
        }
    }
    else if ((mqtt_phase == 9U) &&
             (TB_MQTT_ResultAvailable(&result) != 0U))
    {
        A7670_ClearCommandResult();
        if (result == A7670_COMMAND_SUCCESS)
        {
            mqtt_publish_pending = 0U;
            mqtt_publish_topic[0] = '\0';
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

static void TB_MQTT_TaskCleanup(uint8_t stop_for_alert)
{
    A7670_CommandResult_t result;
    uint8_t force = mqtt_cleanup_forced;

    if (mqtt_phase == 0U)
    {
        result = A7670_GetCommandResult();
        if (result == A7670_COMMAND_PENDING)
        {
            return;
        }
        if (result != A7670_COMMAND_IDLE)
        {
            A7670_ClearCommandResult();
        }
        if ((force != 0U) || (mqtt_connected != 0U))
        {
            if (A7670_CommandStart("AT+CMQTTDISC=0,120",
                                  "+CMQTTDISC: 0,0",
                                  TB_MQTT_CLEANUP_TIMEOUT_MS) != 0U)
            {
                mqtt_phase = 1U;
            }
        }
        else
        {
            mqtt_phase = 2U;
        }
    }
    else if ((mqtt_phase == 1U) &&
             ((result = A7670_GetCommandResult()) != A7670_COMMAND_IDLE) &&
             (result != A7670_COMMAND_PENDING))
    {
        A7670_ClearCommandResult();
        mqtt_connected = 0U;
        mqtt_subscribed = 0U;
        mqtt_phase = 2U;
    }
    else if (mqtt_phase == 2U)
    {
        if ((force != 0U) || (mqtt_client_acquired != 0U))
        {
            if (A7670_CommandStart("AT+CMQTTREL=0", NULL,
                                  TB_MQTT_CLEANUP_TIMEOUT_MS) != 0U)
            {
                mqtt_phase = 3U;
            }
        }
        else
        {
            mqtt_phase = 4U;
        }
    }
    else if ((mqtt_phase == 3U) &&
             ((result = A7670_GetCommandResult()) != A7670_COMMAND_IDLE) &&
             (result != A7670_COMMAND_PENDING))
    {
        A7670_ClearCommandResult();
        mqtt_client_acquired = 0U;
        mqtt_phase = 4U;
    }
    else if (mqtt_phase == 4U)
    {
        if ((force != 0U) || (mqtt_service_started != 0U))
        {
            if (A7670_CommandStart("AT+CMQTTSTOP", "+CMQTTSTOP: 0",
                                  TB_MQTT_CLEANUP_TIMEOUT_MS) != 0U)
            {
                mqtt_phase = 5U;
            }
        }
        else
        {
            mqtt_phase = 6U;
        }
    }
    else if ((mqtt_phase == 5U) &&
             ((result = A7670_GetCommandResult()) != A7670_COMMAND_IDLE) &&
             (result != A7670_COMMAND_PENDING))
    {
        A7670_ClearCommandResult();
        mqtt_service_started = 0U;
        mqtt_phase = 6U;
    }
    else if (mqtt_phase == 6U)
    {
        mqtt_connected = 0U;
        mqtt_subscribed = 0U;
        mqtt_client_acquired = 0U;
        mqtt_service_started = 0U;
        mqtt_cleanup_forced = 0U;
        if (stop_for_alert != 0U)
        {
            mqtt_quiesced = 1U;
            TB_MQTT_SetState(MQTT_IDLE);
        }
        else
        {
            TB_MQTT_SetState(MQTT_START_SERVICE);
        }
    }
}

static uint8_t TB_MQTT_PublishPromptSequenceActive(void)
{
    return (uint8_t)((mqtt_state == MQTT_PUBLISHING) &&
                     (((mqtt_phase >= 1U) && (mqtt_phase <= 3U)) ||
                      ((mqtt_phase >= 5U) && (mqtt_phase <= 7U))));
}

static uint8_t TB_MQTT_SubscribePromptSequenceActive(void)
{
    return (uint8_t)((mqtt_state == MQTT_SUBSCRIBING) &&
                     ((mqtt_phase >= 1U) && (mqtt_phase <= 3U)));
}

static void TB_MQTT_HandleAbort(uint32_t now)
{
    A7670_CommandResult_t result;

    if (A7670_IsNetworkReady() == 0U)
    {
        result = A7670_GetCommandResult();
        if (result == A7670_COMMAND_PENDING)
        {
            return;
        }
        if (result != A7670_COMMAND_IDLE)
        {
            A7670_ClearCommandResult();
        }
        mqtt_connected = 0U;
        mqtt_subscribed = 0U;
        mqtt_client_acquired = 0U;
        mqtt_service_started = 0U;
        mqtt_quiesced = 1U;
        TB_MQTT_SetState(MQTT_IDLE);
        return;
    }

    if (TB_MQTT_PublishPromptSequenceActive() != 0U)
    {
        TB_MQTT_TaskPublishing(now);
        return;
    }
    if (TB_MQTT_SubscribePromptSequenceActive() != 0U)
    {
        TB_MQTT_TaskSubscribing(now);
        return;
    }
    if (mqtt_state != MQTT_STOPPING)
    {
        mqtt_cleanup_forced = 1U;
        TB_MQTT_SetState(MQTT_STOPPING);
    }
    TB_MQTT_TaskCleanup(1U);
}

void TB_MQTT_Init(void)
{
    mqtt_state = MQTT_IDLE;
    mqtt_phase = 0U;
    mqtt_connected = 0U;
    mqtt_service_started = 0U;
    mqtt_client_acquired = 0U;
    mqtt_subscribed = 0U;
    mqtt_cleanup_forced = 1U;
    mqtt_publish_pending = 0U;
    mqtt_abort_requested = 0U;
    mqtt_quiesced = 0U;
    mqtt_rpc_pending = 0U;
    mqtt_command[0] = '\0';
    mqtt_publish_topic[0] = '\0';
    mqtt_publish_payload[0] = '\0';
    memset(&mqtt_rpc_command, 0, sizeof(mqtt_rpc_command));
    mqtt_retry_at_ms = 0U;
    mqtt_retry_delay_ms = TB_MQTT_RETRY_MIN_MS;
    mqtt_prompt_ready_at_ms = 0U;
    mqtt_publish_count = 0U;
    mqtt_error_count = 0U;
    mqtt_rpc_command_count = 0U;
    mqtt_rpc_error_count = 0U;
    mqtt_modem_rx_drop_snapshot = 0U;
    mqtt_last_command_result = (uint8_t)A7670_COMMAND_IDLE;
    mqtt_last_error_state = MQTT_IDLE;
    mqtt_last_response[0] = '\0';
    mqtt_last_error_response[0] = '\0';
    mqtt_last_rpc_method[0] = '\0';
}

void TB_MQTT_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t events;
    A7670_CommandResult_t result;

    TB_MQTT_ProcessIncoming();

    if (mqtt_abort_requested != 0U)
    {
        TB_MQTT_HandleAbort(now);
        return;
    }

    if (A7670_IsNetworkReady() == 0U)
    {
        mqtt_connected = 0U;
        mqtt_subscribed = 0U;
        result = A7670_GetCommandResult();
        if (result == A7670_COMMAND_PENDING)
        {
            return;
        }
        if (result != A7670_COMMAND_IDLE)
        {
            A7670_ClearCommandResult();
        }
        mqtt_service_started = 0U;
        mqtt_client_acquired = 0U;
        mqtt_cleanup_forced = 1U;
        TB_MQTT_SetState(MQTT_IDLE);
        return;
    }

    events = A7670_GetEventFlags();
    if ((events & A7670_EVENT_MQTT_LOST) != 0U)
    {
        A7670_ClearEventFlags(A7670_EVENT_MQTT_LOST);
        TB_MQTT_CopyText(mqtt_last_error_response,
                         sizeof(mqtt_last_error_response),
                         "+CMQTTCONNLOST received");
        result = A7670_GetCommandResult();
        if (result == A7670_COMMAND_PENDING)
        {
            return;
        }
        if (result != A7670_COMMAND_IDLE)
        {
            A7670_ClearCommandResult();
        }
        TB_MQTT_EnterRetry(now);
    }

    if (mqtt_state == MQTT_IDLE)
    {
        TB_MQTT_SetState(MQTT_WAIT_RETRY);
        mqtt_retry_at_ms = now + 500U;
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
        case MQTT_SUBSCRIBING:
            TB_MQTT_TaskSubscribing(now);
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
            if (TB_MQTT_TimeReached(now, mqtt_retry_at_ms) != 0U)
            {
                TB_MQTT_TaskCleanup(0U);
            }
            break;
        case MQTT_STOPPING:
            TB_MQTT_TaskCleanup(1U);
            break;
        case MQTT_IDLE:
        default:
            break;
    }
}

uint8_t TB_MQTT_IsConnected(void)
{
    return (uint8_t)((mqtt_abort_requested == 0U) &&
                     (mqtt_connected != 0U) &&
                     (mqtt_subscribed != 0U) &&
                     (A7670_IsNetworkReady() != 0U));
}

uint8_t TB_MQTT_IsQuiesced(void)
{
    return (uint8_t)((mqtt_quiesced != 0U) &&
                     (A7670_GetCommandResult() == A7670_COMMAND_IDLE));
}

uint8_t TB_MQTT_PublishTelemetry(const char *json)
{
    size_t length;

    if ((json == NULL) || (TB_MQTT_IsConnected() == 0U) ||
        (mqtt_publish_pending != 0U))
    {
        return 0U;
    }
    length = strlen(json);
    if ((length == 0U) || (length >= sizeof(mqtt_publish_payload)) ||
        (length > UINT16_MAX))
    {
        return 0U;
    }
    memcpy(mqtt_publish_payload, json, length + 1U);
    TB_MQTT_CopyText(mqtt_publish_topic, sizeof(mqtt_publish_topic),
                     TRIPGUARD_TB_TELEMETRY_TOPIC);
    mqtt_publish_pending = 1U;
    return 1U;
}

uint8_t TB_MQTT_PublishRpcResponse(uint32_t request_id, const char *json)
{
    size_t length;
    int written;

    if ((json == NULL) || (TB_MQTT_IsConnected() == 0U) ||
        (mqtt_publish_pending != 0U))
    {
        return 0U;
    }
    length = strlen(json);
    if ((length == 0U) || (length >= sizeof(mqtt_publish_payload)) ||
        (length > UINT16_MAX))
    {
        return 0U;
    }

    written = snprintf(mqtt_publish_topic, sizeof(mqtt_publish_topic),
                       "%s%lu", TB_MQTT_RPC_RESPONSE_PREFIX,
                       (unsigned long)request_id);
    if ((written < 0) || ((size_t)written >= sizeof(mqtt_publish_topic)))
    {
        mqtt_publish_topic[0] = '\0';
        return 0U;
    }
    memcpy(mqtt_publish_payload, json, length + 1U);
    mqtt_publish_pending = 1U;
    return 1U;
}

uint8_t TB_MQTT_TakeRpcCommand(TB_RPC_Command_t *command)
{
    if ((command == NULL) || (mqtt_rpc_pending == 0U))
    {
        return 0U;
    }
    memcpy(command, &mqtt_rpc_command, sizeof(*command));
    mqtt_rpc_pending = 0U;
    memset(&mqtt_rpc_command, 0, sizeof(mqtt_rpc_command));
    return 1U;
}

void TB_MQTT_AbortForEmergency(void)
{
    mqtt_abort_requested = 1U;
    mqtt_quiesced = 0U;
    /* Preserve an already accepted RPC response across reconnect. */
    if (strncmp(mqtt_publish_topic, TB_MQTT_RPC_RESPONSE_PREFIX,
                strlen(TB_MQTT_RPC_RESPONSE_PREFIX)) != 0)
    {
        mqtt_publish_pending = 0U;
        mqtt_publish_topic[0] = '\0';
        mqtt_publish_payload[0] = '\0';
    }
}

void TB_MQTT_ResumeAfterEmergency(void)
{
    mqtt_abort_requested = 0U;
    mqtt_quiesced = 0U;
    mqtt_connected = 0U;
    mqtt_subscribed = 0U;
    mqtt_service_started = 0U;
    mqtt_client_acquired = 0U;
    mqtt_cleanup_forced = 1U;
    mqtt_retry_delay_ms = TB_MQTT_RETRY_MIN_MS;
    mqtt_retry_at_ms = HAL_GetTick() + 500U;
    TB_MQTT_SetState(MQTT_WAIT_RETRY);
}

MQTT_State_t TB_MQTT_GetState(void) { return mqtt_state; }
uint8_t TB_MQTT_GetPhase(void) { return mqtt_phase; }
uint32_t TB_MQTT_GetPublishCount(void) { return mqtt_publish_count; }
uint32_t TB_MQTT_GetErrorCount(void) { return mqtt_error_count; }
uint32_t TB_MQTT_GetRpcCommandCount(void)
{
    return mqtt_rpc_command_count;
}
uint32_t TB_MQTT_GetRpcErrorCount(void) { return mqtt_rpc_error_count; }
uint8_t TB_MQTT_GetLastCommandResult(void)
{
    return mqtt_last_command_result;
}
MQTT_State_t TB_MQTT_GetLastErrorState(void)
{
    return mqtt_last_error_state;
}
const char *TB_MQTT_GetLastResponse(void) { return mqtt_last_response; }
const char *TB_MQTT_GetLastErrorResponse(void)
{
    return mqtt_last_error_response;
}
const char *TB_MQTT_GetLastRpcMethod(void)
{
    return mqtt_last_rpc_method;
}
