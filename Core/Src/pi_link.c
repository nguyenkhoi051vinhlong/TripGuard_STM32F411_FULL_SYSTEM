#include "pi_link.h"

#include "tripguard_config.h"
#include "tripguard_power.h"
#include "tripguard_supervisor.h"

#include <string.h>

#define PI_LINK_DMA_RX_SIZE               128U
#define PI_LINK_RX_RING_SIZE              512U
#define PI_LINK_PARSE_SIZE                320U
#define PI_LINK_FRAME_OVERHEAD             11U
#define PI_LINK_MAX_FRAME_SIZE \
  (PI_LINK_FRAME_OVERHEAD + PI_LINK_MAX_PAYLOAD)
#define PI_LINK_TX_QUEUE_DEPTH               8U
#define PI_LINK_DUP_HISTORY_SIZE             8U
#define PI_LINK_PENDING_DEPTH                4U
#define PI_LINK_HEARTBEAT_MS              1000U
#define PI_LINK_OFFLINE_MS                3000U
#define PI_LINK_ACK_TIMEOUT_MS             300U
#define PI_LINK_ACK_RETRY_MAX                 3U
#define PI_LINK_TELEMETRY_HEADER_SIZE          8U
#define PI_LINK_TELEMETRY_DATA_SIZE \
  (PI_LINK_MAX_PAYLOAD - PI_LINK_TELEMETRY_HEADER_SIZE)
#define PI_LINK_SCAN_RETRY_MS             1000U

typedef struct
{
  uint16_t length;
  uint8_t bytes[PI_LINK_MAX_FRAME_SIZE];
} PiLink_TxFrame_t;

typedef struct
{
  uint8_t valid;
  uint8_t type;
  uint16_t sequence;
} PiLink_Duplicate_t;

typedef struct
{
  uint8_t active;
  uint8_t type;
  uint8_t retries;
  uint16_t sequence;
  uint32_t deadline_ms;
  PiLink_TxFrame_t frame;
} PiLink_PendingAck_t;

static UART_HandleTypeDef *pi_uart;
static TaskHandle_t pi_owner_task;
static uint8_t pi_dma_rx[PI_LINK_DMA_RX_SIZE];
static volatile uint16_t pi_dma_old_position;
static uint8_t pi_rx_ring[PI_LINK_RX_RING_SIZE];
static volatile uint16_t pi_rx_head;
static volatile uint16_t pi_rx_tail;
static uint8_t pi_parse[PI_LINK_PARSE_SIZE];
static uint16_t pi_parse_length;

static PiLink_TxFrame_t pi_tx_queue[PI_LINK_TX_QUEUE_DEPTH];
static volatile uint8_t pi_tx_head;
static volatile uint8_t pi_tx_tail;
static volatile uint8_t pi_tx_busy;
static uint8_t pi_tx_active[PI_LINK_MAX_FRAME_SIZE];
static uint16_t pi_tx_active_length;

static PiLink_Duplicate_t pi_duplicates[PI_LINK_DUP_HISTORY_SIZE];
static uint8_t pi_duplicate_next;
static PiLink_PendingAck_t pi_pending[PI_LINK_PENDING_DEPTH];

static volatile uint8_t pi_rx_restart_requested;
static volatile uint8_t pi_tx_kick_requested;
static uint16_t pi_next_sequence;
static uint32_t pi_last_packet_ms;
static uint32_t pi_next_heartbeat_ms;
static uint16_t pi_camera_confidence_permille;
static uint8_t pi_telemetry_json[PI_LINK_MAX_TELEMETRY_JSON];
static volatile uint8_t pi_telemetry_active;
static volatile uint8_t pi_telemetry_in_flight;
static uint16_t pi_telemetry_length;
static uint16_t pi_telemetry_offset;
static uint16_t pi_telemetry_chunk_length;
static uint16_t pi_telemetry_chunk_sequence;
static uint32_t pi_telemetry_message_id;
static volatile uint8_t pi_scan_desired_active;
static volatile uint16_t pi_scan_desired_duration_s;
static volatile uint8_t pi_scan_stop_reason;
static volatile uint8_t pi_scan_command_dirty;
static volatile uint8_t pi_scan_command_in_flight;
static uint8_t pi_scan_in_flight_type;
static uint16_t pi_scan_in_flight_sequence;
static uint32_t pi_scan_next_attempt_ms;

volatile uint32_t pi_rx_frame_count;
volatile uint32_t pi_tx_frame_count;
volatile uint32_t pi_crc_error_count;
volatile uint32_t pi_uart_error_count;
volatile uint32_t pi_duplicate_count;
volatile uint32_t pi_rx_overflow_count;
volatile uint32_t pi_last_packet_age_ms;
volatile uint8_t pi_online;
volatile uint8_t pi_ready;
volatile uint8_t pi_camera_person;
volatile uint32_t pi_telemetry_queued_count;
volatile uint32_t pi_telemetry_sent_count;
volatile uint32_t pi_telemetry_drop_count;
volatile uint32_t pi_scan_session_id;
volatile uint32_t pi_scan_command_sent_count;
volatile uint32_t pi_scan_command_ack_count;
volatile uint32_t pi_scan_complete_count;
volatile uint8_t pi_scan_last_result;

static uint16_t PiLink_GetU16(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint32_t PiLink_GetU32(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static void PiLink_PutU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)(value >> 8U);
}

static void PiLink_PutU32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
  data[2] = (uint8_t)((value >> 16U) & 0xFFU);
  data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint16_t PiLink_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t index;
  uint8_t bit;

  for (index = 0U; index < length; index++)
  {
    crc ^= (uint16_t)data[index] << 8U;
    for (bit = 0U; bit < 8U; bit++)
    {
      crc = ((crc & 0x8000U) != 0U) ?
            (uint16_t)((crc << 1U) ^ 0x1021U) :
            (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

static uint8_t PiLink_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
}

static uint16_t PiLink_AllocateSequence(void)
{
  uint16_t sequence;

  taskENTER_CRITICAL();
  sequence = pi_next_sequence++;
  taskEXIT_CRITICAL();
  return sequence;
}

static uint16_t PiLink_BuildFrame(uint8_t *output, uint8_t type,
                                  uint8_t flags, uint16_t sequence,
                                  const uint8_t *payload,
                                  uint16_t payload_length)
{
  uint16_t crc;

  if ((output == NULL) || (payload_length > PI_LINK_MAX_PAYLOAD) ||
      ((payload_length != 0U) && (payload == NULL)))
  {
    return 0U;
  }

  output[0] = PI_LINK_MAGIC_1;
  output[1] = PI_LINK_MAGIC_2;
  output[2] = PI_LINK_VERSION;
  output[3] = type;
  output[4] = flags;
  PiLink_PutU16(&output[5], sequence);
  PiLink_PutU16(&output[7], payload_length);
  if (payload_length != 0U)
  {
    memcpy(&output[9], payload, payload_length);
  }
  crc = PiLink_Crc16(&output[2], (uint16_t)(7U + payload_length));
  PiLink_PutU16(&output[9U + payload_length], crc);
  return (uint16_t)(PI_LINK_FRAME_OVERHEAD + payload_length);
}

static uint8_t PiLink_QueueRaw(const uint8_t *data, uint16_t length)
{
  uint8_t next;
  uint8_t accepted = 0U;

  if ((data == NULL) || (length == 0U) ||
      (length > PI_LINK_MAX_FRAME_SIZE))
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  next = (uint8_t)((pi_tx_head + 1U) % PI_LINK_TX_QUEUE_DEPTH);
  if (next != pi_tx_tail)
  {
    pi_tx_queue[pi_tx_head].length = length;
    memcpy(pi_tx_queue[pi_tx_head].bytes, data, length);
    pi_tx_head = next;
    pi_tx_kick_requested = 1U;
    accepted = 1U;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

static uint8_t PiLink_QueueFrame(uint8_t type, uint8_t flags,
                                 uint16_t sequence,
                                 const uint8_t *payload,
                                 uint16_t payload_length,
                                 uint8_t remember_ack)
{
  PiLink_TxFrame_t frame;
  uint8_t queued;
  uint8_t slot = PI_LINK_PENDING_DEPTH;

  frame.length = PiLink_BuildFrame(frame.bytes, type, flags, sequence,
                                   payload, payload_length);
  if (frame.length == 0U)
  {
    return 0U;
  }
  if (remember_ack != 0U)
  {
    taskENTER_CRITICAL();
    for (slot = 0U; slot < PI_LINK_PENDING_DEPTH; slot++)
    {
      if (pi_pending[slot].active == 0U)
      {
        pi_pending[slot].active = 1U;
        pi_pending[slot].type = type;
        pi_pending[slot].sequence = sequence;
        pi_pending[slot].retries = 0U;
        pi_pending[slot].deadline_ms = HAL_GetTick() +
                                       PI_LINK_ACK_TIMEOUT_MS;
        memcpy(&pi_pending[slot].frame, &frame, sizeof(frame));
        break;
      }
    }
    taskEXIT_CRITICAL();
    if (slot >= PI_LINK_PENDING_DEPTH)
    {
      return 0U;
    }
  }

  queued = PiLink_QueueRaw(frame.bytes, frame.length);
  if ((queued == 0U) && (remember_ack != 0U))
  {
    taskENTER_CRITICAL();
    if ((pi_pending[slot].active != 0U) &&
        (pi_pending[slot].type == type) &&
        (pi_pending[slot].sequence == sequence))
    {
      pi_pending[slot].active = 0U;
    }
    taskEXIT_CRITICAL();
  }
  return queued;
}

static void PiLink_SendAck(uint8_t acked_type, uint16_t acked_sequence,
                           uint8_t accepted)
{
  uint8_t payload[3];
  uint16_t tx_sequence = PiLink_AllocateSequence();

  payload[0] = acked_type;
  PiLink_PutU16(&payload[1], acked_sequence);
  (void)PiLink_QueueFrame((accepted != 0U) ? STM_MSG_ACK : STM_MSG_NACK,
                          0U, tx_sequence, payload, sizeof(payload), 0U);
}

static uint8_t PiLink_IsImportant(uint8_t type)
{
  return ((type == PI_MSG_READY) ||
          (type == PI_MSG_STATUS_REQUEST) ||
          (type == PI_MSG_CAMERA_PERSON) ||
          (type == PI_MSG_CAMERA_CLEAR) ||
          (type == PI_MSG_SCAN_COMPLETE)) ? 1U : 0U;
}

static uint8_t PiLink_IsDuplicate(uint8_t type, uint16_t sequence)
{
  uint8_t index;

  for (index = 0U; index < PI_LINK_DUP_HISTORY_SIZE; index++)
  {
    if ((pi_duplicates[index].valid != 0U) &&
        (pi_duplicates[index].type == type) &&
        (pi_duplicates[index].sequence == sequence))
    {
      return 1U;
    }
  }
  return 0U;
}

static void PiLink_Remember(uint8_t type, uint16_t sequence)
{
  pi_duplicates[pi_duplicate_next].valid = 1U;
  pi_duplicates[pi_duplicate_next].type = type;
  pi_duplicates[pi_duplicate_next].sequence = sequence;
  pi_duplicate_next = (uint8_t)((pi_duplicate_next + 1U) %
                                PI_LINK_DUP_HISTORY_SIZE);
}

static void PiLink_HandleAck(const uint8_t *payload, uint16_t length)
{
  uint8_t slot;
  uint8_t acked_type;
  uint16_t acked_sequence;
  uint8_t matched = 0U;

  if (length != 3U)
  {
    return;
  }
  acked_type = payload[0];
  acked_sequence = PiLink_GetU16(&payload[1]);
  taskENTER_CRITICAL();
  for (slot = 0U; slot < PI_LINK_PENDING_DEPTH; slot++)
  {
    if ((pi_pending[slot].active != 0U) &&
        (pi_pending[slot].type == acked_type) &&
        (pi_pending[slot].sequence == acked_sequence))
    {
      pi_pending[slot].active = 0U;
      matched = 1U;
      break;
    }
  }
  taskEXIT_CRITICAL();

  if ((matched != 0U) &&
      (acked_type == STM_MSG_TELEMETRY_CHUNK) &&
      (pi_telemetry_in_flight != 0U) &&
      (acked_sequence == pi_telemetry_chunk_sequence))
  {
    taskENTER_CRITICAL();
    pi_telemetry_offset = (uint16_t)(pi_telemetry_offset +
                                     pi_telemetry_chunk_length);
    pi_telemetry_in_flight = 0U;
    if (pi_telemetry_offset >= pi_telemetry_length)
    {
      pi_telemetry_active = 0U;
      pi_telemetry_sent_count++;
    }
    taskEXIT_CRITICAL();
  }

  if ((matched != 0U) &&
      ((acked_type == STM_MSG_SCAN_START) ||
       (acked_type == STM_MSG_SCAN_STOP)) &&
      (pi_scan_command_in_flight != 0U) &&
      (acked_type == pi_scan_in_flight_type) &&
      (acked_sequence == pi_scan_in_flight_sequence))
  {
    taskENTER_CRITICAL();
    pi_scan_command_in_flight = 0U;
    pi_scan_command_ack_count++;
    taskEXIT_CRITICAL();
  }
}

static void PiLink_SendStatus(void)
{
  TripGuard_SupervisorSnapshot_t state;
  uint8_t payload[4];

  TripGuard_Supervisor_GetSnapshot(&state);
  payload[0] = (uint8_t)state.state;
  payload[1] = state.rear_confirmed;
  PiLink_PutU16(&payload[2],
                 (uint16_t)((state.activation_countdown_s > 65535U) ?
                            65535U : state.activation_countdown_s));
  (void)PiLink_QueueFrame(STM_MSG_STATUS, 0U, PiLink_AllocateSequence(),
                          payload, sizeof(payload), 0U);
}

static void PiLink_HandleFrame(const uint8_t *frame, uint16_t total_length)
{
  uint8_t type = frame[3];
  uint8_t flags = frame[4];
  uint16_t sequence = PiLink_GetU16(&frame[5]);
  uint16_t payload_length = PiLink_GetU16(&frame[7]);
  const uint8_t *payload = &frame[9];
  uint8_t important = PiLink_IsImportant(type);
  uint8_t accepted = 1U;

  (void)total_length;
  pi_rx_frame_count++;
  pi_last_packet_ms = HAL_GetTick();
  pi_last_packet_age_ms = 0U;
  pi_online = 1U;

  if ((important != 0U) && (PiLink_IsDuplicate(type, sequence) != 0U))
  {
    pi_duplicate_count++;
    PiLink_SendAck(type, sequence, 1U);
    return;
  }

  switch (type)
  {
    case PI_MSG_HEARTBEAT:
      accepted = (payload_length == 4U) ? 1U : 0U;
      break;

    case PI_MSG_READY:
      if (payload_length == 1U)
      {
        pi_ready = (payload[0] != 0U) ? 1U : 0U;
        if (pi_ready != 0U)
        {
          /* Re-assert the desired scan state after every Pi daemon restart. */
          pi_scan_command_dirty = 1U;
          pi_scan_next_attempt_ms = HAL_GetTick();
        }
      }
      else
      {
        accepted = 0U;
      }
      break;

    case PI_MSG_STATUS_REQUEST:
      if (payload_length == 0U)
      {
        PiLink_SendStatus();
      }
      else
      {
        accepted = 0U;
      }
      break;

    case PI_MSG_CAMERA_PERSON:
      if (payload_length == 2U)
      {
        pi_camera_confidence_permille = PiLink_GetU16(payload);
        if (pi_camera_confidence_permille > 1000U)
        {
          pi_camera_confidence_permille = 1000U;
        }
        pi_camera_person = 1U;
      }
      else
      {
        accepted = 0U;
      }
      break;

    case PI_MSG_CAMERA_CLEAR:
      if (payload_length == 0U)
      {
        pi_camera_person = 0U;
        pi_camera_confidence_permille = 0U;
      }
      else
      {
        accepted = 0U;
      }
      break;

    case PI_MSG_SCAN_COMPLETE:
      if ((payload_length == 5U) &&
          (payload[4] <= PI_SCAN_RESULT_STOPPED))
      {
        uint32_t completed_session = PiLink_GetU32(payload);
        uint8_t completed_result = payload[4];

        if (completed_session == pi_scan_session_id)
        {
          pi_scan_last_result = completed_result;
          pi_scan_complete_count++;
          if (completed_result == PI_SCAN_RESULT_OCCUPIED)
          {
            pi_camera_person = 1U;
          }
           else if (completed_result == PI_SCAN_RESULT_CLEAR)
           {
             pi_camera_person = 0U;
             pi_camera_confidence_permille = 0U;
           }
           (void)TripGuard_Supervisor_PostPiScanComplete(
               completed_session, completed_result);
         }
      }
      else
      {
        accepted = 0U;
      }
      break;

    case PI_MSG_ACK:
      PiLink_HandleAck(payload, payload_length);
      return;

    default:
      accepted = 0U;
      break;
  }

  if ((important != 0U) && (accepted != 0U))
  {
    PiLink_Remember(type, sequence);
  }

  if ((important != 0U) ||
      ((flags & PI_LINK_FLAG_ACK_REQUIRED) != 0U))
  {
    PiLink_SendAck(type, sequence, accepted);
  }
}

static void PiLink_ParseBuffered(void)
{
  uint16_t offset;
  uint16_t payload_length;
  uint16_t frame_length;
  uint16_t expected_crc;
  uint16_t actual_crc;

  for (;;)
  {
    if (pi_parse_length < 2U)
    {
      return;
    }

    offset = 0U;
    while ((offset + 1U < pi_parse_length) &&
           ((pi_parse[offset] != PI_LINK_MAGIC_1) ||
            (pi_parse[offset + 1U] != PI_LINK_MAGIC_2)))
    {
      offset++;
    }
    if (offset != 0U)
    {
      memmove(pi_parse, &pi_parse[offset], pi_parse_length - offset);
      pi_parse_length = (uint16_t)(pi_parse_length - offset);
    }
    if ((pi_parse_length < 2U) ||
        (pi_parse[0] != PI_LINK_MAGIC_1) ||
        (pi_parse[1] != PI_LINK_MAGIC_2))
    {
      if ((pi_parse_length != 0U) &&
          (pi_parse[pi_parse_length - 1U] == PI_LINK_MAGIC_1))
      {
        pi_parse[0] = PI_LINK_MAGIC_1;
        pi_parse_length = 1U;
      }
      else
      {
        pi_parse_length = 0U;
      }
      return;
    }
    if (pi_parse_length < 9U)
    {
      return;
    }

    payload_length = PiLink_GetU16(&pi_parse[7]);
    if ((pi_parse[2] != PI_LINK_VERSION) ||
        (payload_length > PI_LINK_MAX_PAYLOAD))
    {
      memmove(pi_parse, &pi_parse[1], pi_parse_length - 1U);
      pi_parse_length--;
      continue;
    }
    frame_length = (uint16_t)(PI_LINK_FRAME_OVERHEAD + payload_length);
    if (pi_parse_length < frame_length)
    {
      return;
    }

    expected_crc = PiLink_GetU16(&pi_parse[9U + payload_length]);
    actual_crc = PiLink_Crc16(&pi_parse[2],
                              (uint16_t)(7U + payload_length));
    if (expected_crc != actual_crc)
    {
      pi_crc_error_count++;
      memmove(pi_parse, &pi_parse[1], pi_parse_length - 1U);
      pi_parse_length--;
      continue;
    }

    PiLink_HandleFrame(pi_parse, frame_length);
    memmove(pi_parse, &pi_parse[frame_length],
            pi_parse_length - frame_length);
    pi_parse_length = (uint16_t)(pi_parse_length - frame_length);
  }
}

static void PiLink_DrainRx(void)
{
  uint8_t byte;

  while (pi_rx_tail != pi_rx_head)
  {
    byte = pi_rx_ring[pi_rx_tail];
    pi_rx_tail = (uint16_t)((pi_rx_tail + 1U) % PI_LINK_RX_RING_SIZE);
    if (pi_parse_length >= PI_LINK_PARSE_SIZE)
    {
      memmove(pi_parse, &pi_parse[1], PI_LINK_PARSE_SIZE - 1U);
      pi_parse_length = PI_LINK_PARSE_SIZE - 1U;
      pi_rx_overflow_count++;
    }
    pi_parse[pi_parse_length++] = byte;
  }
  PiLink_ParseBuffered();
}

static uint8_t PiLink_StartRx(void)
{
  HAL_StatusTypeDef status;

  if (pi_uart == NULL)
  {
    return 0U;
  }
  pi_dma_old_position = 0U;
  status = HAL_UARTEx_ReceiveToIdle_DMA(pi_uart, pi_dma_rx,
                                        PI_LINK_DMA_RX_SIZE);
  if (status == HAL_OK)
  {
    if (pi_uart->hdmarx != NULL)
    {
      __HAL_DMA_DISABLE_IT(pi_uart->hdmarx, DMA_IT_HT);
    }
    pi_rx_restart_requested = 0U;
    return 1U;
  }
  pi_uart_error_count++;
  return 0U;
}

static void PiLink_RecoverRx(void)
{
  if ((pi_uart == NULL) || (pi_rx_restart_requested == 0U))
  {
    return;
  }
  (void)HAL_UART_AbortReceive(pi_uart);
  __HAL_UART_CLEAR_PEFLAG(pi_uart);
  pi_uart->ErrorCode = HAL_UART_ERROR_NONE;
  (void)PiLink_StartRx();
}

static void PiLink_KickTx(void)
{
  uint8_t tail;

  if ((pi_uart == NULL) || (pi_tx_busy != 0U) ||
      (pi_tx_tail == pi_tx_head))
  {
    return;
  }

  taskENTER_CRITICAL();
  tail = pi_tx_tail;
  pi_tx_active_length = pi_tx_queue[tail].length;
  memcpy(pi_tx_active, pi_tx_queue[tail].bytes, pi_tx_active_length);
  pi_tx_tail = (uint8_t)((tail + 1U) % PI_LINK_TX_QUEUE_DEPTH);
  pi_tx_busy = 1U;
  pi_tx_kick_requested = 0U;
  taskEXIT_CRITICAL();

  if (HAL_UART_Transmit_IT(pi_uart, pi_tx_active,
                           pi_tx_active_length) != HAL_OK)
  {
    pi_tx_busy = 0U;
    pi_uart_error_count++;
  }
}

static void PiLink_ServicePending(uint32_t now_ms)
{
  uint8_t slot;
  uint8_t should_queue;
  PiLink_TxFrame_t retry_frame;

  for (slot = 0U; slot < PI_LINK_PENDING_DEPTH; slot++)
  {
    should_queue = 0U;
    taskENTER_CRITICAL();
    if ((pi_pending[slot].active != 0U) &&
        (PiLink_TimeReached(now_ms, pi_pending[slot].deadline_ms) != 0U))
    {
      if (pi_pending[slot].retries >= PI_LINK_ACK_RETRY_MAX)
      {
        if ((pi_pending[slot].type == STM_MSG_TELEMETRY_CHUNK) &&
            (pi_telemetry_in_flight != 0U) &&
            (pi_pending[slot].sequence == pi_telemetry_chunk_sequence))
        {
          pi_telemetry_active = 0U;
          pi_telemetry_in_flight = 0U;
          pi_telemetry_drop_count++;
        }
        if (((pi_pending[slot].type == STM_MSG_SCAN_START) ||
             (pi_pending[slot].type == STM_MSG_SCAN_STOP)) &&
            (pi_scan_command_in_flight != 0U) &&
            (pi_pending[slot].type == pi_scan_in_flight_type) &&
            (pi_pending[slot].sequence == pi_scan_in_flight_sequence))
        {
          pi_scan_command_in_flight = 0U;
          pi_scan_command_dirty = 1U;
          pi_scan_next_attempt_ms = now_ms + PI_LINK_SCAN_RETRY_MS;
        }
        pi_pending[slot].active = 0U;
      }
      else
      {
        pi_pending[slot].frame.bytes[4] |= PI_LINK_FLAG_RETRY;
        {
          uint16_t payload_length =
              PiLink_GetU16(&pi_pending[slot].frame.bytes[7]);
          uint16_t crc = PiLink_Crc16(
              &pi_pending[slot].frame.bytes[2],
              (uint16_t)(7U + payload_length));
          PiLink_PutU16(
              &pi_pending[slot].frame.bytes[9U + payload_length], crc);
        }
        memcpy(&retry_frame, &pi_pending[slot].frame,
               sizeof(retry_frame));
        pi_pending[slot].retries++;
        pi_pending[slot].deadline_ms = now_ms + PI_LINK_ACK_TIMEOUT_MS;
        should_queue = 1U;
      }
    }
    taskEXIT_CRITICAL();
    if (should_queue != 0U)
    {
      (void)PiLink_QueueRaw(retry_frame.bytes, retry_frame.length);
    }
  }
}

static void PiLink_ServiceScanCommand(uint32_t now_ms)
{
  uint8_t payload[6];
  uint16_t payload_length;
  uint8_t type;
  uint16_t sequence;
  uint8_t desired_active;
  uint32_t session_id;

  if ((pi_online == 0U) || (pi_ready == 0U) ||
      (pi_scan_command_dirty == 0U) ||
      (pi_scan_command_in_flight != 0U) ||
      (PiLink_TimeReached(now_ms, pi_scan_next_attempt_ms) == 0U))
  {
    return;
  }

  taskENTER_CRITICAL();
  desired_active = pi_scan_desired_active;
  session_id = pi_scan_session_id;
  PiLink_PutU32(&payload[0], session_id);
  if (desired_active != 0U)
  {
    PiLink_PutU16(&payload[4], pi_scan_desired_duration_s);
    payload_length = 6U;
    type = STM_MSG_SCAN_START;
  }
  else
  {
    payload[4] = pi_scan_stop_reason;
    payload_length = 5U;
    type = STM_MSG_SCAN_STOP;
  }
  taskEXIT_CRITICAL();

  sequence = PiLink_AllocateSequence();
  if (PiLink_QueueFrame(type, PI_LINK_FLAG_ACK_REQUIRED, sequence,
                        payload, payload_length, 1U) != 0U)
  {
    taskENTER_CRITICAL();
    pi_scan_in_flight_type = type;
    pi_scan_in_flight_sequence = sequence;
    pi_scan_command_in_flight = 1U;
    /* Preserve a newer request that raced with the frame construction. */
    if ((desired_active == pi_scan_desired_active) &&
        (session_id == pi_scan_session_id))
    {
      pi_scan_command_dirty = 0U;
    }
    pi_scan_command_sent_count++;
    taskEXIT_CRITICAL();
  }
  else
  {
    pi_scan_next_attempt_ms = now_ms + PI_LINK_SCAN_RETRY_MS;
  }
}

static void PiLink_ServiceTelemetry(void)
{
  uint8_t payload[PI_LINK_MAX_PAYLOAD];
  uint16_t remaining;
  uint16_t chunk_length;
  uint16_t sequence;
  uint16_t offset;
  uint16_t total_length;
  uint32_t message_id;

  taskENTER_CRITICAL();
  if ((pi_telemetry_active == 0U) ||
      (pi_telemetry_in_flight != 0U))
  {
    taskEXIT_CRITICAL();
    return;
  }
  offset = pi_telemetry_offset;
  total_length = pi_telemetry_length;
  message_id = pi_telemetry_message_id;
  remaining = (uint16_t)(total_length - offset);
  chunk_length = (remaining > PI_LINK_TELEMETRY_DATA_SIZE) ?
                 PI_LINK_TELEMETRY_DATA_SIZE : remaining;
  PiLink_PutU32(&payload[0], message_id);
  PiLink_PutU16(&payload[4], total_length);
  PiLink_PutU16(&payload[6], offset);
  memcpy(&payload[PI_LINK_TELEMETRY_HEADER_SIZE],
         &pi_telemetry_json[offset], chunk_length);
  taskEXIT_CRITICAL();

  sequence = PiLink_AllocateSequence();
  if (PiLink_QueueFrame(STM_MSG_TELEMETRY_CHUNK,
                        PI_LINK_FLAG_ACK_REQUIRED,
                        sequence,
                        payload,
                        (uint16_t)(PI_LINK_TELEMETRY_HEADER_SIZE +
                                   chunk_length),
                        1U) != 0U)
  {
    taskENTER_CRITICAL();
    pi_telemetry_chunk_sequence = sequence;
    pi_telemetry_chunk_length = chunk_length;
    pi_telemetry_in_flight = 1U;
    taskEXIT_CRITICAL();
  }
}

uint8_t PiLink_Init(UART_HandleTypeDef *huart, TaskHandle_t owner_task)
{
  if ((huart == NULL) || (huart->Instance != USART6) ||
      (owner_task == NULL))
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  pi_uart = huart;
  pi_owner_task = owner_task;
  pi_dma_old_position = 0U;
  pi_rx_head = 0U;
  pi_rx_tail = 0U;
  pi_parse_length = 0U;
  pi_tx_head = 0U;
  pi_tx_tail = 0U;
  pi_tx_busy = 0U;
  pi_tx_active_length = 0U;
  pi_duplicate_next = 0U;
  memset(pi_duplicates, 0, sizeof(pi_duplicates));
  memset(pi_pending, 0, sizeof(pi_pending));
  pi_next_sequence = 0U;
  pi_rx_restart_requested = 1U;
  pi_tx_kick_requested = 0U;
  pi_online = 0U;
  pi_ready = 0U;
  pi_camera_person = 0U;
  pi_camera_confidence_permille = 0U;
  pi_telemetry_active = 0U;
  pi_telemetry_in_flight = 0U;
  pi_telemetry_length = 0U;
  pi_telemetry_offset = 0U;
  pi_telemetry_chunk_length = 0U;
  pi_telemetry_chunk_sequence = 0U;
  pi_telemetry_message_id = 0U;
  pi_scan_desired_active = 0U;
  pi_scan_desired_duration_s =
      (uint16_t)TRIPGUARD_CAMERA_SCAN_DURATION_SECONDS;
  pi_scan_stop_reason = PI_SCAN_STOP_REASON_STANDBY;
  pi_scan_command_dirty = 1U;
  pi_scan_command_in_flight = 0U;
  pi_scan_in_flight_type = 0U;
  pi_scan_in_flight_sequence = 0U;
  pi_scan_session_id = 0U;
  pi_scan_last_result = PI_SCAN_RESULT_STOPPED;
  pi_last_packet_age_ms = UINT32_MAX;
  taskEXIT_CRITICAL();

  pi_next_heartbeat_ms = HAL_GetTick();
  pi_scan_next_attempt_ms = pi_next_heartbeat_ms;
  return PiLink_StartRx();
}

void PiLink_Task(void *argument)
{
  uint32_t now_ms;
  uint8_t heartbeat[4];

  (void)argument;
  for (;;)
  {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20U));
    PiLink_RecoverRx();
    PiLink_DrainRx();
    now_ms = HAL_GetTick();

    if (pi_online != 0U)
    {
      pi_last_packet_age_ms = (uint32_t)(now_ms - pi_last_packet_ms);
      if (pi_last_packet_age_ms > PI_LINK_OFFLINE_MS)
      {
        pi_online = 0U;
        pi_ready = 0U;
        pi_camera_person = 0U;
        pi_camera_confidence_permille = 0U;
        memset(pi_duplicates, 0, sizeof(pi_duplicates));
        pi_duplicate_next = 0U;
        if (pi_scan_command_in_flight == 0U)
        {
          pi_scan_command_dirty = 1U;
        }
      }
    }

    if (PiLink_TimeReached(now_ms, pi_next_heartbeat_ms) != 0U)
    {
      PiLink_PutU32(heartbeat, now_ms);
      (void)PiLink_QueueFrame(STM_MSG_HEARTBEAT, 0U,
                              PiLink_AllocateSequence(), heartbeat,
                              sizeof(heartbeat), 0U);
      pi_next_heartbeat_ms = now_ms + PI_LINK_HEARTBEAT_MS;
    }

    PiLink_ServicePending(now_ms);
    PiLink_ServiceScanCommand(now_ms);
    PiLink_ServiceTelemetry();
    if ((pi_tx_kick_requested != 0U) || (pi_tx_tail != pi_tx_head))
    {
      PiLink_KickTx();
    }
  }
}

void PiLink_RxEventFromISR(UART_HandleTypeDef *huart, uint16_t size)
{
  uint16_t position;
  uint16_t index;
  uint16_t count;
  uint16_t next;
  BaseType_t higher_priority_task_woken = pdFALSE;

  if ((huart == NULL) || (huart != pi_uart) ||
      (size > PI_LINK_DMA_RX_SIZE))
  {
    return;
  }

  position = (size == PI_LINK_DMA_RX_SIZE) ? 0U : size;
  index = pi_dma_old_position;
  count = (size >= pi_dma_old_position) ?
          (uint16_t)(size - pi_dma_old_position) :
          (uint16_t)((PI_LINK_DMA_RX_SIZE - pi_dma_old_position) + size);
  while (count != 0U)
  {
    next = (uint16_t)((pi_rx_head + 1U) % PI_LINK_RX_RING_SIZE);
    if (next == pi_rx_tail)
    {
      pi_rx_overflow_count++;
      break;
    }
    pi_rx_ring[pi_rx_head] = pi_dma_rx[index];
    pi_rx_head = next;
    index = (uint16_t)((index + 1U) % PI_LINK_DMA_RX_SIZE);
    count--;
  }
  pi_dma_old_position = position;

  if (pi_owner_task != NULL)
  {
    vTaskNotifyGiveFromISR(pi_owner_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}

void PiLink_TxCompleteFromISR(UART_HandleTypeDef *huart)
{
  BaseType_t higher_priority_task_woken = pdFALSE;

  if ((huart == NULL) || (huart != pi_uart))
  {
    return;
  }
  pi_tx_busy = 0U;
  pi_tx_frame_count++;
  pi_tx_kick_requested = 1U;
  if (pi_owner_task != NULL)
  {
    vTaskNotifyGiveFromISR(pi_owner_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}

void PiLink_ErrorFromISR(UART_HandleTypeDef *huart)
{
  BaseType_t higher_priority_task_woken = pdFALSE;

  if ((huart == NULL) || (huart != pi_uart))
  {
    return;
  }
  pi_uart_error_count++;
  pi_rx_restart_requested = 1U;
  if (pi_owner_task != NULL)
  {
    vTaskNotifyGiveFromISR(pi_owner_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}

uint8_t PiLink_SendEvent(uint8_t event_code, uint16_t value,
                         uint8_t ack_required)
{
  uint8_t payload[4];
  uint8_t flags = (ack_required != 0U) ?
                  PI_LINK_FLAG_ACK_REQUIRED : 0U;
  uint16_t sequence = PiLink_AllocateSequence();

  payload[0] = event_code;
  payload[1] = (uint8_t)tripguard_system_state;
  PiLink_PutU16(&payload[2], value);
  return PiLink_QueueFrame(STM_MSG_EVENT, flags, sequence,
                           payload, sizeof(payload), ack_required);
}

uint8_t PiLink_QueueTelemetryJson(const char *json, uint16_t length)
{
  uint8_t accepted = 0U;

  if ((json == NULL) || (length == 0U) ||
      (length > PI_LINK_MAX_TELEMETRY_JSON))
  {
    pi_telemetry_drop_count++;
    return 0U;
  }

  taskENTER_CRITICAL();
  if (pi_telemetry_active == 0U)
  {
    memcpy(pi_telemetry_json, json, length);
    pi_telemetry_length = length;
    pi_telemetry_offset = 0U;
    pi_telemetry_chunk_length = 0U;
    pi_telemetry_in_flight = 0U;
    pi_telemetry_message_id++;
    pi_telemetry_active = 1U;
    pi_telemetry_queued_count++;
    accepted = 1U;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

uint8_t PiLink_SendAlertRequest(uint8_t alert_kind, uint16_t value)
{
  uint8_t payload[4];

  payload[0] = alert_kind;
  payload[1] = (uint8_t)tripguard_system_state;
  PiLink_PutU16(&payload[2], value);
  return PiLink_QueueFrame(STM_MSG_ALERT_REQUEST,
                           PI_LINK_FLAG_ACK_REQUIRED,
                           PiLink_AllocateSequence(),
                            payload, sizeof(payload), 1U);
}

uint8_t PiLink_RequestScanStart(uint16_t duration_seconds)
{
  if (duration_seconds == 0U)
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  pi_scan_session_id++;
  if (pi_scan_session_id == 0U)
  {
    pi_scan_session_id = 1U;
  }
  pi_scan_desired_duration_s = duration_seconds;
  pi_scan_desired_active = 1U;
  pi_scan_command_dirty = 1U;
  pi_scan_next_attempt_ms = HAL_GetTick();
  taskEXIT_CRITICAL();
  return 1U;
}

uint8_t PiLink_RequestScanStop(uint8_t reason)
{
  taskENTER_CRITICAL();
  pi_scan_stop_reason = reason;
  pi_scan_desired_active = 0U;
  pi_scan_command_dirty = 1U;
  pi_scan_next_attempt_ms = HAL_GetTick();
  taskEXIT_CRITICAL();
  return 1U;
}

void PiLink_ClearCameraOccupancy(void)
{
  taskENTER_CRITICAL();
  pi_camera_person = 0U;
  pi_camera_confidence_permille = 0U;
  taskEXIT_CRITICAL();
}

void PiLink_GetSnapshot(PiLink_Snapshot_t *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }
  taskENTER_CRITICAL();
  snapshot->online = pi_online;
  snapshot->ready = pi_ready;
  snapshot->camera_person = pi_camera_person;
  snapshot->camera_confidence_permille = pi_camera_confidence_permille;
  snapshot->rx_frames = pi_rx_frame_count;
  snapshot->tx_frames = pi_tx_frame_count;
  snapshot->crc_errors = pi_crc_error_count;
  snapshot->uart_errors = pi_uart_error_count;
  snapshot->duplicates = pi_duplicate_count;
  snapshot->rx_overflows = pi_rx_overflow_count;
  snapshot->last_packet_age_ms = pi_last_packet_age_ms;
  taskEXIT_CRITICAL();
}

uint8_t PiLink_IsOnline(void)
{
  return pi_online;
}

uint8_t PiLink_IsReady(void)
{
  return pi_ready;
}

uint8_t PiLink_GetCameraPerson(void)
{
  return pi_camera_person;
}

uint16_t PiLink_GetCameraConfidencePermille(void)
{
  return pi_camera_confidence_permille;
}
