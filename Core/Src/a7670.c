#include "a7670.h"
#include "tripguard_config.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MODEM_DMA_RX_SIZE              512U
#define MODEM_RING_BUFFER_SIZE        2048U
#define MODEM_LINE_BUFFER_SIZE         512U
#define MODEM_RESPONSE_BUFFER_SIZE     768U
#define MODEM_EXPECTED_TOKEN_SIZE       96U
#define MODEM_COMMAND_BUFFER_SIZE       192U
#define MODEM_IP_ADDRESS_SIZE            48U

#define MODEM_BOOT_WAIT_MS            8000U
#define MODEM_COMMAND_TIMEOUT_MS       3000U
#define MODEM_NETWORK_QUERY_MS         2000U
#define MODEM_READY_POLL_MS           30000U
#define MODEM_RETRY_DELAY_MS           2000U
#define MODEM_ERROR_RETRY_MS          10000U
#define MODEM_UART_TX_TIMEOUT_MS       1000U
#define MODEM_UART_RAW_TX_TIMEOUT_MS   6000U
#define MODEM_DMA_RESTART_DELAY_MS       10U
#define MODEM_DMA_RETRY_DELAY_MS         50U

typedef struct
{
    A7670_CommandResult_t result;
    uint32_t deadline_ms;
    char expected_token[MODEM_EXPECTED_TOKEN_SIZE];
    char response[MODEM_RESPONSE_BUFFER_SIZE];
    size_t response_length;
} A7670_Transaction_t;

static UART_HandleTypeDef *modem_uart;
static uint8_t modem_dma_rx[MODEM_DMA_RX_SIZE];
static volatile uint16_t modem_dma_position;

static uint8_t modem_ring[MODEM_RING_BUFFER_SIZE];
static volatile uint16_t modem_ring_head;
static volatile uint16_t modem_ring_tail;
static volatile uint32_t modem_ring_overflow_count;
static volatile uint32_t modem_uart_error_count;
static volatile uint8_t modem_dma_restart_requested;
static volatile uint32_t modem_uart_last_error_code;
static volatile uint32_t modem_uart_parity_error_count;
static volatile uint32_t modem_uart_noise_error_count;
static volatile uint32_t modem_uart_frame_error_count;
static volatile uint32_t modem_uart_overrun_error_count;
static volatile uint32_t modem_dma_error_count;
static uint32_t modem_dma_restart_count;
static uint32_t modem_dma_restart_failure_count;
static uint32_t modem_dma_restart_at_ms;

static char modem_line[MODEM_LINE_BUFFER_SIZE];
static size_t modem_line_length;
static A7670_Transaction_t modem_transaction;
static char modem_last_command[MODEM_COMMAND_BUFFER_SIZE];
static char modem_last_response[MODEM_RESPONSE_BUFFER_SIZE];
static char modem_ip_address[MODEM_IP_ADDRESS_SIZE];
static A7670_CommandResult_t modem_last_command_result;
static uint32_t modem_command_timeout_count;

static A7670_State_t modem_state;
static uint8_t modem_phase;
static uint8_t modem_retry_count;
static uint8_t modem_network_registered;
static uint8_t modem_pdp_active;
static int16_t modem_registration_status;
static uint8_t modem_ready_poll_pending;
static uint32_t modem_next_action_ms;
static uint32_t modem_ready_poll_ms;
static uint32_t modem_event_flags;
static uint8_t modem_exclusive_session;
static A7670_MqttMessage_t modem_mqtt_rx_message;
static char modem_mqtt_rx_topic[A7670_MQTT_TOPIC_SIZE];
static char modem_mqtt_rx_payload[A7670_MQTT_PAYLOAD_SIZE];
static uint8_t modem_mqtt_rx_stage;
static uint8_t modem_mqtt_rx_pending;
static uint8_t modem_mqtt_rx_invalid;
static uint32_t modem_mqtt_rx_dropped_count;

static uint8_t A7670_TimeReached(uint32_t now, uint32_t deadline)
{
    return (uint8_t)(((int32_t)(now - deadline)) >= 0);
}

static void A7670_CopyText(char *destination,
                           size_t destination_size,
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

static void A7670_AppendResponse(const char *text)
{
    size_t text_length;
    size_t available;

    if (text == NULL)
    {
        return;
    }

    text_length = strlen(text);
    available = sizeof(modem_transaction.response) -
                modem_transaction.response_length - 1U;

    if (text_length > available)
    {
        text_length = available;
    }

    if (text_length > 0U)
    {
        memcpy(&modem_transaction.response[modem_transaction.response_length],
               text,
               text_length);
        modem_transaction.response_length += text_length;
        modem_transaction.response[modem_transaction.response_length] = '\0';
    }
}

static void A7670_SnapshotCommandResult(void)
{
    if ((modem_transaction.result == A7670_COMMAND_IDLE) ||
        (modem_transaction.result == A7670_COMMAND_PENDING))
    {
        return;
    }

    modem_last_command_result = modem_transaction.result;
    A7670_CopyText(modem_last_response,
                   sizeof(modem_last_response),
                   modem_transaction.response);
}

static void A7670_RingPushFromIsr(uint8_t byte)
{
    uint16_t head = modem_ring_head;
    uint16_t next_head =
        (uint16_t)((head + 1U) % MODEM_RING_BUFFER_SIZE);

    if (next_head == modem_ring_tail)
    {
        modem_ring_overflow_count++;
        return;
    }

    modem_ring[head] = byte;
    __DMB();
    modem_ring_head = next_head;
}

static uint8_t A7670_RingPop(uint8_t *byte)
{
    uint16_t tail;

    if (byte == NULL)
    {
        return 0U;
    }

    tail = modem_ring_tail;
    if (tail == modem_ring_head)
    {
        return 0U;
    }

    __DMB();
    *byte = modem_ring[tail];
    modem_ring_tail =
        (uint16_t)((tail + 1U) % MODEM_RING_BUFFER_SIZE);
    return 1U;
}

static HAL_StatusTypeDef A7670_StartDmaReception(void)
{
    HAL_StatusTypeDef status;

    if ((modem_uart == NULL) || (modem_uart->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    modem_dma_position = 0U;
    status = HAL_UARTEx_ReceiveToIdle_DMA(modem_uart,
                                         modem_dma_rx,
                                         MODEM_DMA_RX_SIZE);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(modem_uart->hdmarx, DMA_IT_HT);
    }

    return status;
}

static void A7670_RequestDmaRestart(uint32_t delay_ms)
{
    uint32_t restart_at = HAL_GetTick() + delay_ms;

    if (modem_dma_restart_requested == 0U)
    {
        modem_dma_restart_at_ms = restart_at;
    }
    modem_dma_restart_requested = 1U;
}

static void A7670_RestartDmaIfNeeded(void)
{
    uint32_t now;

    if ((modem_dma_restart_requested == 0U) || (modem_uart == NULL))
    {
        return;
    }

    now = HAL_GetTick();
    if (A7670_TimeReached(now, modem_dma_restart_at_ms) == 0U)
    {
        return;
    }

    modem_dma_restart_requested = 0U;
    modem_line_length = 0U;

    /*
     * Restart RX only in CellularTask context.  Clearing PE/FE/NE/ORE before
     * arming DMA prevents one electrical glitch from causing a restart loop.
     * The current AT transaction is deliberately kept pending: any complete
     * response received after recovery can still finish it, otherwise its
     * normal deadline performs a clean retry.
     */
    (void)HAL_UART_AbortReceive(modem_uart);
    __HAL_UART_CLEAR_PEFLAG(modem_uart);
    modem_uart->ErrorCode = HAL_UART_ERROR_NONE;

    if (A7670_StartDmaReception() != HAL_OK)
    {
        modem_dma_restart_failure_count++;
        A7670_RequestDmaRestart(MODEM_DMA_RETRY_DELAY_MS);
    }
    else
    {
        modem_dma_restart_count++;
    }
}

static void A7670_ParseCereg(const char *line)
{
    const char *cursor;
    char *end_pointer;
    long first_value;
    long registration_value;
    uint8_t was_registered;

    cursor = strstr(line, "+CEREG:");
    if (cursor == NULL)
    {
        return;
    }

    cursor += strlen("+CEREG:");
    while (*cursor == ' ')
    {
        cursor++;
    }

    first_value = strtol(cursor, &end_pointer, 10);
    if (end_pointer == cursor)
    {
        return;
    }

    registration_value = first_value;
    cursor = end_pointer;
    while (*cursor == ' ')
    {
        cursor++;
    }

    /* Query response: +CEREG: <n>,<stat>. URC co the la <stat>,"TAC",... */
    if (*cursor == ',')
    {
        cursor++;
        while (*cursor == ' ')
        {
            cursor++;
        }

        if ((*cursor >= '0') && (*cursor <= '9'))
        {
            long second_value = strtol(cursor, &end_pointer, 10);
            if (end_pointer != cursor)
            {
                registration_value = second_value;
            }
        }
    }

    was_registered = modem_network_registered;
    modem_network_registered =
        (uint8_t)((registration_value == 1L) ||
                  (registration_value == 5L));
    modem_registration_status = (int16_t)registration_value;

    if ((was_registered != 0U) && (modem_network_registered == 0U))
    {
        modem_event_flags |= A7670_EVENT_NETWORK_LOST;
    }
}

static void A7670_ParseUrc(const char *line)
{
    A7670_ParseCereg(line);

    if ((strstr(line, "+PDP: DEACT") != NULL) ||
        (strstr(line, "+CGEV: NW DEACT") != NULL) ||
        (strstr(line, "+CGEV: ME DEACT") != NULL))
    {
        modem_pdp_active = 0U;
        modem_ip_address[0] = '\0';
        modem_event_flags |= A7670_EVENT_PDP_DEACTIVATED;
    }

    if (strstr(line, "+CMQTTCONNLOST:") != NULL)
    {
        modem_event_flags |= A7670_EVENT_MQTT_LOST;
    }
}

static uint8_t A7670_ParseMqttRx(const char *line)
{
    if (strstr(line, "+CMQTTRXSTART:") != NULL)
    {
        modem_mqtt_rx_topic[0] = '\0';
        modem_mqtt_rx_payload[0] = '\0';
        modem_mqtt_rx_invalid = 0U;
        modem_mqtt_rx_stage = 1U;
        return 1U;
    }

    if (strstr(line, "+CMQTTRXTOPIC:") != NULL)
    {
        if (modem_mqtt_rx_stage == 1U)
        {
            modem_mqtt_rx_stage = 2U;
        }
        else
        {
            modem_mqtt_rx_invalid = 1U;
        }
        return 1U;
    }

    if (strstr(line, "+CMQTTRXPAYLOAD:") != NULL)
    {
        if (modem_mqtt_rx_stage == 3U)
        {
            modem_mqtt_rx_stage = 4U;
        }
        else
        {
            modem_mqtt_rx_invalid = 1U;
        }
        return 1U;
    }

    if (strstr(line, "+CMQTTRXEND:") != NULL)
    {
        if ((modem_mqtt_rx_stage == 5U) &&
            (modem_mqtt_rx_invalid == 0U) &&
            (modem_mqtt_rx_topic[0] != '\0') &&
            (modem_mqtt_rx_payload[0] != '\0'))
        {
            if (modem_mqtt_rx_pending == 0U)
            {
                A7670_CopyText(modem_mqtt_rx_message.topic,
                               sizeof(modem_mqtt_rx_message.topic),
                               modem_mqtt_rx_topic);
                A7670_CopyText(modem_mqtt_rx_message.payload,
                               sizeof(modem_mqtt_rx_message.payload),
                               modem_mqtt_rx_payload);
                modem_mqtt_rx_pending = 1U;
            }
            else
            {
                modem_mqtt_rx_dropped_count++;
            }
        }
        else
        {
            modem_mqtt_rx_dropped_count++;
        }

        modem_mqtt_rx_stage = 0U;
        modem_mqtt_rx_invalid = 0U;
        return 1U;
    }

    if (modem_mqtt_rx_stage == 2U)
    {
        if (strlen(line) < sizeof(modem_mqtt_rx_topic))
        {
            A7670_CopyText(modem_mqtt_rx_topic,
                           sizeof(modem_mqtt_rx_topic), line);
            modem_mqtt_rx_stage = 3U;
        }
        else
        {
            modem_mqtt_rx_invalid = 1U;
        }
        return 1U;
    }

    if (modem_mqtt_rx_stage == 4U)
    {
        if (strlen(line) < sizeof(modem_mqtt_rx_payload))
        {
            A7670_CopyText(modem_mqtt_rx_payload,
                           sizeof(modem_mqtt_rx_payload), line);
            modem_mqtt_rx_stage = 5U;
        }
        else
        {
            modem_mqtt_rx_invalid = 1U;
        }
        return 1U;
    }

    return 0U;
}

static uint8_t A7670_IsErrorLine(const char *line)
{
    return (uint8_t)((strcmp(line, "ERROR") == 0) ||
                     (strcmp(line, "NO CARRIER") == 0) ||
                     (strcmp(line, "NO DIALTONE") == 0) ||
                     (strcmp(line, "BUSY") == 0) ||
                     (strcmp(line, "NO ANSWER") == 0) ||
                     (strstr(line, "+CME ERROR:") != NULL) ||
                     (strstr(line, "+CMS ERROR:") != NULL));
}

static uint8_t A7670_IsExpectedFamilyFailure(const char *line)
{
    const char *separator;
    size_t prefix_length;

    if (modem_transaction.expected_token[0] != '+')
    {
        return 0U;
    }

    separator = strchr(modem_transaction.expected_token, ':');
    if (separator == NULL)
    {
        return 0U;
    }

    prefix_length = (size_t)(separator - modem_transaction.expected_token) + 1U;
    return (uint8_t)((strncmp(line,
                              modem_transaction.expected_token,
                              prefix_length) == 0) &&
                     (strstr(line, modem_transaction.expected_token) == NULL));
}

static void A7670_ProcessLine(const char *line)
{
    if ((line == NULL) || (line[0] == '\0'))
    {
        return;
    }

    A7670_ParseUrc(line);

    /* URC nhan MQTT khong duoc tinh la phan hoi cua lenh AT dang chay. */
    if (A7670_ParseMqttRx(line) != 0U)
    {
        return;
    }

    if (modem_transaction.result != A7670_COMMAND_PENDING)
    {
        return;
    }

    A7670_AppendResponse(line);
    A7670_AppendResponse("\r\n");

    if (A7670_IsErrorLine(line) != 0U)
    {
        modem_transaction.result = A7670_COMMAND_ERROR;
    }
    else if (modem_transaction.expected_token[0] == '\0')
    {
        if (strcmp(line, "OK") == 0)
        {
            modem_transaction.result = A7670_COMMAND_SUCCESS;
        }
    }
    else if (strstr(line, modem_transaction.expected_token) != NULL)
    {
        modem_transaction.result = A7670_COMMAND_SUCCESS;
    }
    else if (A7670_IsExpectedFamilyFailure(line) != 0U)
    {
        modem_transaction.result = A7670_COMMAND_ERROR;
    }
}

static void A7670_ProcessByte(uint8_t byte)
{
    if ((byte == '>') &&
        (modem_transaction.result == A7670_COMMAND_PENDING) &&
        (strcmp(modem_transaction.expected_token, ">") == 0))
    {
        A7670_AppendResponse(">");
        modem_transaction.result = A7670_COMMAND_SUCCESS;
        return;
    }

    if (byte == '\r')
    {
        return;
    }

    if (byte == '\n')
    {
        if (modem_line_length > 0U)
        {
            modem_line[modem_line_length] = '\0';
            A7670_ProcessLine(modem_line);
            modem_line_length = 0U;
        }
        return;
    }

    if (modem_line_length < (sizeof(modem_line) - 1U))
    {
        modem_line[modem_line_length++] = (char)byte;
    }
    else
    {
        modem_line_length = 0U;
    }
}

static void A7670_ProcessRx(void)
{
    uint8_t byte;

    while (A7670_RingPop(&byte) != 0U)
    {
        A7670_ProcessByte(byte);
    }
}

static void A7670_UpdateCommandTimeout(uint32_t now)
{
    if ((modem_transaction.result == A7670_COMMAND_PENDING) &&
        (A7670_TimeReached(now, modem_transaction.deadline_ms) != 0U))
    {
        modem_transaction.result = A7670_COMMAND_TIMEOUT;
        modem_command_timeout_count++;
    }
}

static void A7670_SetState(A7670_State_t state)
{
    modem_state = state;
    modem_phase = 0U;
    modem_retry_count = 0U;
    modem_next_action_ms = HAL_GetTick();

    if (state == MODEM_ERROR)
    {
        modem_network_registered = 0U;
        modem_pdp_active = 0U;
        modem_ip_address[0] = '\0';
        modem_next_action_ms += MODEM_ERROR_RETRY_MS;
    }
}

static void A7670_RetryState(uint32_t now, uint8_t maximum_retries)
{
    A7670_ClearCommandResult();
    modem_retry_count++;
    modem_phase = 0U;
    modem_next_action_ms = now + MODEM_RETRY_DELAY_MS;

    if (modem_retry_count >= maximum_retries)
    {
        A7670_SetState(MODEM_ERROR);
    }
}

static uint8_t A7670_ResponseHasPdpActive(const char *response)
{
    const char *cursor = response;

    while ((cursor = strstr(cursor, "+CGACT:")) != NULL)
    {
        char *end_pointer;
        long cid;
        long active;

        cursor += strlen("+CGACT:");
        while (*cursor == ' ')
        {
            cursor++;
        }

        cid = strtol(cursor, &end_pointer, 10);
        if ((end_pointer == cursor) || (*end_pointer != ','))
        {
            continue;
        }

        cursor = end_pointer + 1;
        active = strtol(cursor, &end_pointer, 10);
        if ((cid == 1L) && (active == 1L))
        {
            return 1U;
        }

        cursor = end_pointer;
    }

    return 0U;
}

static uint8_t A7670_ParseIpAddress(const char *response)
{
    const char *cursor;
    size_t length = 0U;

    modem_ip_address[0] = '\0';
    if (response == NULL)
    {
        return 0U;
    }

    cursor = strstr(response, "+CGPADDR:");
    if (cursor == NULL)
    {
        return 0U;
    }

    cursor = strchr(cursor, ',');
    if (cursor == NULL)
    {
        return 0U;
    }
    cursor++;

    while ((*cursor == ' ') || (*cursor == '\"'))
    {
        cursor++;
    }

    while ((cursor[length] != '\0') &&
           (cursor[length] != '\"') &&
           (cursor[length] != ',') &&
           (cursor[length] != '\r') &&
           (cursor[length] != '\n') &&
           (length < (sizeof(modem_ip_address) - 1U)))
    {
        modem_ip_address[length] = cursor[length];
        length++;
    }
    modem_ip_address[length] = '\0';

    if ((length == 0U) ||
        (strcmp(modem_ip_address, "0.0.0.0") == 0))
    {
        modem_ip_address[0] = '\0';
        return 0U;
    }

    return 1U;
}

static void A7670_TaskAtWait(uint32_t now)
{
    A7670_CommandResult_t result;

    if ((modem_phase == 0U) &&
        (A7670_TimeReached(now, modem_next_action_ms) != 0U))
    {
        if (A7670_CommandStart("AT", NULL, MODEM_COMMAND_TIMEOUT_MS) != 0U)
        {
            modem_phase = 1U;
        }
    }
    else if (modem_phase == 1U)
    {
        result = A7670_GetCommandResult();
        if ((result == A7670_COMMAND_IDLE) ||
            (result == A7670_COMMAND_PENDING))
        {
            return;
        }

        if (result == A7670_COMMAND_SUCCESS)
        {
            A7670_ClearCommandResult();
            modem_phase = 2U;
            modem_retry_count = 0U;
        }
        else
        {
            A7670_RetryState(now, 10U);
        }
    }
    else if (modem_phase == 2U)
    {
        if (A7670_CommandStart("ATE0", NULL, MODEM_COMMAND_TIMEOUT_MS) != 0U)
        {
            modem_phase = 3U;
        }
    }
    else if (modem_phase == 3U)
    {
        result = A7670_GetCommandResult();
        if ((result == A7670_COMMAND_IDLE) ||
            (result == A7670_COMMAND_PENDING))
        {
            return;
        }

        if (result == A7670_COMMAND_SUCCESS)
        {
            A7670_ClearCommandResult();
            modem_phase = 4U;
        }
        else
        {
            A7670_RetryState(now, 10U);
        }
    }
    else if (modem_phase == 4U)
    {
        if (A7670_CommandStart("AT+CMEE=2", NULL,
                              MODEM_COMMAND_TIMEOUT_MS) != 0U)
        {
            modem_phase = 5U;
        }
    }
    else if (modem_phase == 5U)
    {
        result = A7670_GetCommandResult();
        if ((result == A7670_COMMAND_IDLE) ||
            (result == A7670_COMMAND_PENDING))
        {
            return;
        }

        /* CMEE chi tang thong tin loi; modem van co the chay neu lenh nay loi. */
        A7670_ClearCommandResult();
        A7670_SetState(MODEM_SIM_WAIT);
    }
}

static void A7670_TaskSimWait(uint32_t now)
{
    A7670_CommandResult_t result;
    uint8_t sim_ready;

    if ((modem_phase == 0U) &&
        (A7670_TimeReached(now, modem_next_action_ms) != 0U))
    {
        if (A7670_CommandStart("AT+CPIN?", NULL,
                              MODEM_COMMAND_TIMEOUT_MS) != 0U)
        {
            modem_phase = 1U;
        }
    }
    else if (modem_phase == 1U)
    {
        result = A7670_GetCommandResult();
        if ((result == A7670_COMMAND_IDLE) ||
            (result == A7670_COMMAND_PENDING))
        {
            return;
        }

        sim_ready = (uint8_t)((result == A7670_COMMAND_SUCCESS) &&
                              (strstr(A7670_GetCommandResponse(),
                                      "+CPIN: READY") != NULL));
        A7670_ClearCommandResult();

        if (sim_ready != 0U)
        {
            A7670_SetState(MODEM_NETWORK_WAIT);
        }
        else
        {
            modem_retry_count++;
            modem_phase = 0U;
            modem_next_action_ms = now + MODEM_RETRY_DELAY_MS;
            if (modem_retry_count >= 30U)
            {
                A7670_SetState(MODEM_ERROR);
            }
        }
    }
}

static void A7670_TaskNetworkWait(uint32_t now)
{
    A7670_CommandResult_t result;

    if ((modem_phase == 0U) &&
        (A7670_TimeReached(now, modem_next_action_ms) != 0U))
    {
        if (A7670_CommandStart("AT+CEREG=2", NULL,
                              MODEM_COMMAND_TIMEOUT_MS) != 0U)
        {
            modem_phase = 1U;
        }
    }
    else if (modem_phase == 1U)
    {
        result = A7670_GetCommandResult();
        if ((result == A7670_COMMAND_IDLE) ||
            (result == A7670_COMMAND_PENDING))
        {
            return;
        }

        A7670_ClearCommandResult();
        modem_phase = 2U;
        modem_next_action_ms = now;
    }
    else if ((modem_phase == 2U) &&
             (A7670_TimeReached(now, modem_next_action_ms) != 0U))
    {
        if (A7670_CommandStart("AT+CEREG?", NULL,
                              MODEM_COMMAND_TIMEOUT_MS) != 0U)
        {
            modem_phase = 3U;
        }
    }
    else if (modem_phase == 3U)
    {
        result = A7670_GetCommandResult();
        if ((result == A7670_COMMAND_IDLE) ||
            (result == A7670_COMMAND_PENDING))
        {
            return;
        }

        A7670_ClearCommandResult();
        if ((result == A7670_COMMAND_SUCCESS) &&
            (modem_network_registered != 0U))
        {
            A7670_SetState(MODEM_PDP_WAIT);
        }
        else
        {
            modem_retry_count++;
            modem_phase = 2U;
            modem_next_action_ms = now + MODEM_NETWORK_QUERY_MS;
            if (modem_retry_count >= 60U)
            {
                A7670_SetState(MODEM_ERROR);
            }
        }
    }
}

static void A7670_TaskPdpWait(uint32_t now)
{
    A7670_CommandResult_t result;
    char command[128];
    uint8_t condition;

    if ((modem_phase == 0U) &&
        (A7670_TimeReached(now, modem_next_action_ms) != 0U))
    {
        (void)snprintf(command,
                       sizeof(command),
                       "AT+CGDCONT=1,\"IP\",\"%s\"",
                       TRIPGUARD_CELLULAR_APN);
        if (A7670_CommandStart(command, NULL,
                              MODEM_COMMAND_TIMEOUT_MS) != 0U)
        {
            modem_phase = 1U;
        }
    }
    else if (modem_phase == 1U)
    {
        result = A7670_GetCommandResult();
        if ((result == A7670_COMMAND_IDLE) ||
            (result == A7670_COMMAND_PENDING))
        {
            return;
        }
        A7670_ClearCommandResult();

        if (result == A7670_COMMAND_SUCCESS)
        {
            modem_phase = 2U;
        }
        else
        {
            A7670_RetryState(now, 10U);
        }
    }
    else if (modem_phase == 2U)
    {
        if (A7670_CommandStart("AT+CGATT?", NULL,
                              MODEM_COMMAND_TIMEOUT_MS) != 0U)
        {
            modem_phase = 3U;
        }
    }
    else if (modem_phase == 3U)
    {
        result = A7670_GetCommandResult();
        if ((result == A7670_COMMAND_IDLE) ||
            (result == A7670_COMMAND_PENDING))
        {
            return;
        }

        condition = (uint8_t)((result == A7670_COMMAND_SUCCESS) &&
                              (strstr(A7670_GetCommandResponse(),
                                      "+CGATT: 1") != NULL));
        A7670_ClearCommandResult();
        modem_phase = (condition != 0U) ? 6U : 4U;
    }
    else if (modem_phase == 4U)
    {
        if (A7670_CommandStart("AT+CGATT=1", NULL, 10000U) != 0U)
        {
            modem_phase = 5U;
        }
    }
    else if (modem_phase == 5U)
    {
        result = A7670_GetCommandResult();
        if ((result == A7670_COMMAND_IDLE) ||
            (result == A7670_COMMAND_PENDING))
        {
            return;
        }
        A7670_ClearCommandResult();

        if (result == A7670_COMMAND_SUCCESS)
        {
            modem_phase = 6U;
        }
        else
        {
            A7670_RetryState(now, 10U);
        }
    }
    else if (modem_phase == 6U)
    {
        if (A7670_CommandStart("AT+CGACT?", NULL,
                              MODEM_COMMAND_TIMEOUT_MS) != 0U)
        {
            modem_phase = 7U;
        }
    }
    else if (modem_phase == 7U)
    {
        result = A7670_GetCommandResult();
        if ((result == A7670_COMMAND_IDLE) ||
            (result == A7670_COMMAND_PENDING))
        {
            return;
        }

        condition = (uint8_t)((result == A7670_COMMAND_SUCCESS) &&
                              (A7670_ResponseHasPdpActive(
                                  A7670_GetCommandResponse()) != 0U));
        A7670_ClearCommandResult();

        if (condition != 0U)
        {
            modem_phase = 10U;
        }
        else
        {
            modem_phase = 8U;
        }
    }
    else if (modem_phase == 8U)
    {
        if (A7670_CommandStart("AT+CGACT=1,1", NULL, 15000U) != 0U)
        {
            modem_phase = 9U;
        }
    }
    else if (modem_phase == 9U)
    {
        result = A7670_GetCommandResult();
        if ((result == A7670_COMMAND_IDLE) ||
            (result == A7670_COMMAND_PENDING))
        {
            return;
        }
        A7670_ClearCommandResult();

        if (result == A7670_COMMAND_SUCCESS)
        {
            modem_retry_count++;
            modem_phase = 6U;
            if (modem_retry_count >= 10U)
            {
                A7670_SetState(MODEM_ERROR);
            }
        }
        else
        {
            A7670_RetryState(now, 10U);
        }
    }
    else if (modem_phase == 10U)
    {
        if (A7670_CommandStart("AT+CGPADDR=1", NULL,
                              MODEM_COMMAND_TIMEOUT_MS) != 0U)
        {
            modem_phase = 11U;
        }
    }
    else if (modem_phase == 11U)
    {
        result = A7670_GetCommandResult();
        if ((result == A7670_COMMAND_IDLE) ||
            (result == A7670_COMMAND_PENDING))
        {
            return;
        }

        condition = (uint8_t)((result == A7670_COMMAND_SUCCESS) &&
                              (A7670_ParseIpAddress(
                                  A7670_GetCommandResponse()) != 0U));
        A7670_ClearCommandResult();

        if (condition != 0U)
        {
            modem_pdp_active = 1U;
            modem_ready_poll_ms = now + MODEM_READY_POLL_MS;
            A7670_SetState(MODEM_READY);
        }
        else
        {
            modem_pdp_active = 0U;
            modem_retry_count++;
            modem_phase = 6U;
            modem_next_action_ms = now + MODEM_RETRY_DELAY_MS;
            if (modem_retry_count >= 10U)
            {
                A7670_SetState(MODEM_ERROR);
            }
        }
    }
}

static void A7670_TaskReady(uint32_t now)
{
    A7670_CommandResult_t result;

    if (modem_ready_poll_pending != 0U)
    {
        result = A7670_GetCommandResult();
        if ((result != A7670_COMMAND_IDLE) &&
            (result != A7670_COMMAND_PENDING))
        {
            A7670_ClearCommandResult();
            modem_ready_poll_pending = 0U;
            modem_ready_poll_ms = now + MODEM_READY_POLL_MS;
        }
        return;
    }

    if ((A7670_TimeReached(now, modem_ready_poll_ms) != 0U) &&
        (A7670_GetCommandResult() == A7670_COMMAND_IDLE))
    {
        if (A7670_CommandStart("AT+CEREG?", NULL,
                              MODEM_COMMAND_TIMEOUT_MS) != 0U)
        {
            modem_ready_poll_pending = 1U;
        }
    }
}

void A7670_Init(UART_HandleTypeDef *huart)
{
    modem_uart = huart;
    modem_dma_position = 0U;
    modem_ring_head = 0U;
    modem_ring_tail = 0U;
    modem_ring_overflow_count = 0U;
    modem_uart_error_count = 0U;
    modem_dma_restart_requested = 0U;
    modem_uart_last_error_code = HAL_UART_ERROR_NONE;
    modem_uart_parity_error_count = 0U;
    modem_uart_noise_error_count = 0U;
    modem_uart_frame_error_count = 0U;
    modem_uart_overrun_error_count = 0U;
    modem_dma_error_count = 0U;
    modem_dma_restart_count = 0U;
    modem_dma_restart_failure_count = 0U;
    modem_dma_restart_at_ms = 0U;
    modem_line_length = 0U;
    modem_network_registered = 0U;
    modem_pdp_active = 0U;
    modem_registration_status = -1;
    modem_ready_poll_pending = 0U;
    modem_event_flags = A7670_EVENT_NONE;
    modem_exclusive_session = 0U;
    memset(&modem_mqtt_rx_message, 0, sizeof(modem_mqtt_rx_message));
    modem_mqtt_rx_topic[0] = '\0';
    modem_mqtt_rx_payload[0] = '\0';
    modem_mqtt_rx_stage = 0U;
    modem_mqtt_rx_pending = 0U;
    modem_mqtt_rx_invalid = 0U;
    modem_mqtt_rx_dropped_count = 0U;
    memset(&modem_transaction, 0, sizeof(modem_transaction));
    modem_last_command[0] = '\0';
    modem_last_response[0] = '\0';
    modem_ip_address[0] = '\0';
    modem_last_command_result = A7670_COMMAND_IDLE;
    modem_command_timeout_count = 0U;

    if (A7670_StartDmaReception() == HAL_OK)
    {
        modem_state = MODEM_BOOTING;
        modem_phase = 0U;
        modem_retry_count = 0U;
        modem_next_action_ms = HAL_GetTick() + MODEM_BOOT_WAIT_MS;
    }
    else
    {
        modem_dma_restart_requested = 1U;
        A7670_SetState(MODEM_ERROR);
    }
}

void A7670_Task(void)
{
    uint32_t now;

    A7670_RestartDmaIfNeeded();

    /* Recover if HAL stopped RX without delivering another error callback. */
    if ((modem_uart != NULL) &&
        (modem_uart->RxState == HAL_UART_STATE_READY) &&
        (modem_dma_restart_requested == 0U))
    {
        A7670_RequestDmaRestart(MODEM_DMA_RESTART_DELAY_MS);
    }

    A7670_ProcessRx();
    now = HAL_GetTick();
    A7670_UpdateCommandTimeout(now);

    /*
     * Trong phien khan cap, chi van hanh DMA/parser/timeout. Alert FSM o
     * CellularTask la chu duy nhat cua transaction AT, tranh MQTT/PDP chen lenh.
     */
    if (modem_exclusive_session != 0U)
    {
        return;
    }

    if (modem_state == MODEM_READY)
    {
        if ((modem_network_registered == 0U) ||
            (modem_pdp_active == 0U))
        {
            A7670_CommandResult_t current = A7670_GetCommandResult();

            /*
             * Khong xoa transaction MQTT/HTTP dang PENDING khi mat mang/PDP.
             * Parser van phai nhan terminal URC/timeout cua lenh cu truoc khi
             * modem FSM chuyen owner sang recovery NETWORK/PDP.
             */
            if (current == A7670_COMMAND_PENDING)
            {
                return;
            }
            if (current != A7670_COMMAND_IDLE)
            {
                A7670_ClearCommandResult();
            }

            modem_ready_poll_pending = 0U;
            if (modem_network_registered == 0U)
            {
                modem_pdp_active = 0U;
                A7670_SetState(MODEM_NETWORK_WAIT);
            }
            else
            {
                A7670_SetState(MODEM_PDP_WAIT);
            }
        }
    }

    switch (modem_state)
    {
        case MODEM_BOOTING:
            if (A7670_TimeReached(now, modem_next_action_ms) != 0U)
            {
                A7670_SetState(MODEM_AT_WAIT);
            }
            break;

        case MODEM_AT_WAIT:
            A7670_TaskAtWait(now);
            break;

        case MODEM_SIM_WAIT:
            A7670_TaskSimWait(now);
            break;

        case MODEM_NETWORK_WAIT:
            A7670_TaskNetworkWait(now);
            break;

        case MODEM_PDP_WAIT:
            A7670_TaskPdpWait(now);
            break;

        case MODEM_READY:
            A7670_TaskReady(now);
            break;

        case MODEM_ERROR:
            if (A7670_TimeReached(now, modem_next_action_ms) != 0U)
            {
                A7670_CommandResult_t current = A7670_GetCommandResult();
                if (current == A7670_COMMAND_PENDING)
                {
                    break;
                }
                if (current != A7670_COMMAND_IDLE)
                {
                    A7670_ClearCommandResult();
                }
                A7670_SetState(MODEM_AT_WAIT);
            }
            break;

        case MODEM_OFF:
        default:
            break;
    }
}

void A7670_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t old_position;
    uint16_t index;

    if ((modem_uart == NULL) ||
        (huart == NULL) ||
        (huart->Instance != modem_uart->Instance) ||
        (size > MODEM_DMA_RX_SIZE))
    {
        return;
    }

    old_position = modem_dma_position;
    if (size == old_position)
    {
        return;
    }

    if (size > old_position)
    {
        for (index = old_position; index < size; index++)
        {
            A7670_RingPushFromIsr(modem_dma_rx[index]);
        }
    }
    else
    {
        for (index = old_position; index < MODEM_DMA_RX_SIZE; index++)
        {
            A7670_RingPushFromIsr(modem_dma_rx[index]);
        }
        for (index = 0U; index < size; index++)
        {
            A7670_RingPushFromIsr(modem_dma_rx[index]);
        }
    }

    modem_dma_position =
        (size == MODEM_DMA_RX_SIZE) ? 0U : size;
}

void A7670_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t error_code;
    uint16_t dma_position;

    if ((modem_uart == NULL) ||
        (huart == NULL) ||
        (huart->Instance != modem_uart->Instance))
    {
        return;
    }

    error_code = huart->ErrorCode;
    modem_uart_last_error_code = error_code;
    modem_uart_error_count++;

    if ((error_code & HAL_UART_ERROR_PE) != 0U)
    {
        modem_uart_parity_error_count++;
    }
    if ((error_code & HAL_UART_ERROR_NE) != 0U)
    {
        modem_uart_noise_error_count++;
    }
    if ((error_code & HAL_UART_ERROR_FE) != 0U)
    {
        modem_uart_frame_error_count++;
    }
    if ((error_code & HAL_UART_ERROR_ORE) != 0U)
    {
        modem_uart_overrun_error_count++;
    }
    if ((error_code & HAL_UART_ERROR_DMA) != 0U)
    {
        modem_dma_error_count++;
    }

    /* Preserve bytes DMA already wrote before HAL stopped the transfer. */
    if (huart->hdmarx != NULL)
    {
        dma_position = (uint16_t)(MODEM_DMA_RX_SIZE -
                        __HAL_DMA_GET_COUNTER(huart->hdmarx));
        if (dma_position <= MODEM_DMA_RX_SIZE)
        {
            A7670_RxEventCallback(huart, dma_position);
        }
    }

    A7670_RequestDmaRestart(MODEM_DMA_RESTART_DELAY_MS);
}

A7670_State_t A7670_GetState(void)
{
    return modem_state;
}

uint8_t A7670_IsNetworkReady(void)
{
    return (uint8_t)((modem_state == MODEM_READY) &&
                     (modem_network_registered != 0U) &&
                     (modem_pdp_active != 0U));
}

uint8_t A7670_IsRegistered(void)
{
    return modem_network_registered;
}

uint8_t A7670_IsPdpActive(void)
{
    return modem_pdp_active;
}

int16_t A7670_GetRegistrationStatus(void)
{
    return modem_registration_status;
}

const char *A7670_GetIpAddress(void)
{
    return modem_ip_address;
}

uint8_t A7670_SendCommand(const char *cmd)
{
    static const uint8_t line_ending[] = {'\r', '\n'};
    size_t command_length;

    if ((modem_uart == NULL) || (cmd == NULL))
    {
        return 0U;
    }

    command_length = strlen(cmd);
    if ((command_length == 0U) || (command_length > UINT16_MAX))
    {
        return 0U;
    }

    if (HAL_UART_Transmit(modem_uart,
                          (uint8_t *)(uintptr_t)cmd,
                          (uint16_t)command_length,
                          MODEM_UART_TX_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    return (uint8_t)(HAL_UART_Transmit(modem_uart,
                                       (uint8_t *)(uintptr_t)line_ending,
                                       sizeof(line_ending),
                                       MODEM_UART_TX_TIMEOUT_MS) == HAL_OK);
}

uint8_t A7670_SendRaw(const uint8_t *data, uint16_t len)
{
    if ((modem_uart == NULL) || (data == NULL) || (len == 0U))
    {
        return 0U;
    }

    return (uint8_t)(HAL_UART_Transmit(modem_uart,
                                       (uint8_t *)(uintptr_t)data,
                                       len,
                                       MODEM_UART_RAW_TX_TIMEOUT_MS) == HAL_OK);
}

static uint8_t A7670_PrepareTransaction(const char *expected_token,
                                        uint32_t timeout_ms)
{
    if (modem_transaction.result != A7670_COMMAND_IDLE)
    {
        return 0U;
    }

    modem_transaction.response[0] = '\0';
    modem_transaction.response_length = 0U;
    A7670_CopyText(modem_transaction.expected_token,
                   sizeof(modem_transaction.expected_token),
                   expected_token);
    modem_transaction.deadline_ms = HAL_GetTick() + timeout_ms;
    modem_transaction.result = A7670_COMMAND_PENDING;
    return 1U;
}

uint8_t A7670_CommandStart(const char *cmd,
                          const char *expected_token,
                          uint32_t timeout_ms)
{
    if ((cmd == NULL) ||
        (A7670_PrepareTransaction(expected_token, timeout_ms) == 0U))
    {
        return 0U;
    }

    A7670_CopyText(modem_last_command,
                   sizeof(modem_last_command),
                   cmd);

    if (A7670_SendCommand(cmd) == 0U)
    {
        modem_transaction.result = A7670_COMMAND_ERROR;
    }

    return 1U;
}

uint8_t A7670_WaitResponse(const char *expected_token,
                           uint32_t timeout_ms)
{
    return A7670_PrepareTransaction(expected_token, timeout_ms);
}

A7670_CommandResult_t A7670_GetCommandResult(void)
{
    return modem_transaction.result;
}

const char *A7670_GetCommandResponse(void)
{
    return modem_transaction.response;
}

void A7670_ClearCommandResult(void)
{
    if (modem_transaction.result == A7670_COMMAND_PENDING)
    {
        return;
    }

    A7670_SnapshotCommandResult();
    modem_transaction.result = A7670_COMMAND_IDLE;
    modem_transaction.expected_token[0] = '\0';
    modem_transaction.response[0] = '\0';
    modem_transaction.response_length = 0U;
}

void A7670_CancelCommand(void)
{
    A7670_SnapshotCommandResult();
    modem_transaction.result = A7670_COMMAND_IDLE;
    modem_transaction.expected_token[0] = '\0';
    modem_transaction.response[0] = '\0';
    modem_transaction.response_length = 0U;
    modem_ready_poll_pending = 0U;
    modem_ready_poll_ms = HAL_GetTick() + MODEM_READY_POLL_MS;
}

uint8_t A7670_BeginExclusiveSession(void)
{
    if (modem_exclusive_session != 0U)
    {
        return 1U;
    }

    /* SMS va voice call can dang ky mang, khong bat buoc PDP data. */
    if ((modem_uart == NULL) || (modem_network_registered == 0U))
    {
        return 0U;
    }

    /*
     * Tuyet doi khong A7670_CancelCommand() de "cuop" modem. Cancel chi
     * xoa transaction phia STM32, khong dam bao A76XX dung lenh dang xu ly.
     * Owner moi chi duoc vao khi transaction cu da IDLE that su.
     */
    if (A7670_GetCommandResult() != A7670_COMMAND_IDLE)
    {
        return 0U;
    }

    modem_ready_poll_pending = 0U;
    modem_exclusive_session = 1U;
    return 1U;
}

void A7670_EndExclusiveSession(void)
{
    if (modem_exclusive_session == 0U)
    {
        return;
    }

    /* Alert FSM phai consume/clear terminal response truoc khi roi phien. */
    if (A7670_GetCommandResult() != A7670_COMMAND_IDLE)
    {
        return;
    }

    modem_exclusive_session = 0U;

    if (modem_network_registered == 0U)
    {
        modem_pdp_active = 0U;
        A7670_SetState(MODEM_NETWORK_WAIT);
    }
    else if (modem_pdp_active == 0U)
    {
        A7670_SetState(MODEM_PDP_WAIT);
    }
    else
    {
        A7670_SetState(MODEM_READY);
    }
}

uint8_t A7670_IsExclusiveSessionActive(void)
{
    return modem_exclusive_session;
}

const char *A7670_GetLastCommand(void)
{
    return modem_last_command;
}

const char *A7670_GetLastResponse(void)
{
    return modem_last_response;
}

A7670_CommandResult_t A7670_GetLastCommandResult(void)
{
    return modem_last_command_result;
}

uint32_t A7670_GetCommandTimeoutCount(void)
{
    return modem_command_timeout_count;
}

uint32_t A7670_GetEventFlags(void)
{
    return modem_event_flags;
}

void A7670_ClearEventFlags(uint32_t flags)
{
    modem_event_flags &= ~flags;
}

uint8_t A7670_TakeMqttMessage(A7670_MqttMessage_t *message)
{
    if ((message == NULL) || (modem_mqtt_rx_pending == 0U))
    {
        return 0U;
    }

    memcpy(message, &modem_mqtt_rx_message, sizeof(*message));
    modem_mqtt_rx_pending = 0U;
    modem_mqtt_rx_message.topic[0] = '\0';
    modem_mqtt_rx_message.payload[0] = '\0';
    return 1U;
}

uint32_t A7670_GetMqttRxDroppedCount(void)
{
    return modem_mqtt_rx_dropped_count;
}

uint32_t A7670_GetRxOverflowCount(void)
{
    return modem_ring_overflow_count;
}

uint32_t A7670_GetUartErrorCount(void)
{
    return modem_uart_error_count;
}

uint32_t A7670_GetLastUartErrorCode(void)
{
    return modem_uart_last_error_code;
}

uint32_t A7670_GetUartParityErrorCount(void)
{
    return modem_uart_parity_error_count;
}

uint32_t A7670_GetUartNoiseErrorCount(void)
{
    return modem_uart_noise_error_count;
}

uint32_t A7670_GetUartFrameErrorCount(void)
{
    return modem_uart_frame_error_count;
}

uint32_t A7670_GetUartOverrunErrorCount(void)
{
    return modem_uart_overrun_error_count;
}

uint32_t A7670_GetDmaErrorCount(void)
{
    return modem_dma_error_count;
}

uint32_t A7670_GetDmaRestartCount(void)
{
    return modem_dma_restart_count;
}

uint32_t A7670_GetDmaRestartFailureCount(void)
{
    return modem_dma_restart_failure_count;
}
