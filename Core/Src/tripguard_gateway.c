#include "tripguard_gateway.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tripguard_event.h"
#include "tripguard_protocol.h"
#include "tripguard_rtos.h"
#include "tripguard_spi.h"
#include "tripguard_supervisor.h"

#include <stdint.h>

static osMessageQueueId_t gateway_event_queue;
static osMessageQueueId_t gateway_ack_queue;
static osEventFlagsId_t gateway_system_events;

volatile uint32_t tripguard_gateway_valid_rx_count;
volatile uint32_t tripguard_gateway_invalid_rx_count;
volatile uint32_t tripguard_gateway_queue_full_count;
volatile uint32_t tripguard_gateway_ack_timeout_count;
volatile uint32_t tripguard_gateway_stale_ack_count;

volatile uint8_t tripguard_gateway_radar_healthy;
volatile uint8_t tripguard_gateway_radar_person;
volatile uint16_t tripguard_gateway_radar_distance_cm;
volatile uint16_t tripguard_gateway_radar_last_sequence;
volatile uint32_t tripguard_gateway_radar_last_packet_ms;
volatile uint32_t tripguard_gateway_radar_person_count;
volatile uint32_t tripguard_gateway_radar_clear_count;
volatile uint32_t tripguard_gateway_radar_health_count;
volatile uint32_t tripguard_gateway_radar_duplicate_count;

static uint8_t gateway_radar_sequence_valid;

static uint8_t TripGuard_Gateway_IsRadarEvent(uint8_t event)
{
  return (uint8_t)((event == TG_EVENT_PERSON) ||
                   (event == TG_EVENT_RADAR_CLEAR) ||
                   (event == TG_EVENT_RADAR_HEALTH));
}

static uint8_t TripGuard_Gateway_IsSupportedPacket(
    const tripguard_spi_packet_t *packet,
    tripguard_event_type_t *event_type)
{
  if ((packet == NULL) || (event_type == NULL) ||
      (packet->node_id != TG_NODE_REMOTE) || (packet->flags != 0U))
  {
    return 0U;
  }

  switch (packet->event)
  {
    case TG_EVENT_REAR_CONFIRM:
      if (packet->value != 1U)
      {
        return 0U;
      }
      *event_type = TG_EVT_REAR_CONFIRM;
      return 1U;

    case TG_EVENT_REMOTE_SOS:
      if (packet->value != 1U)
      {
        return 0U;
      }
      *event_type = TG_EVT_REMOTE_SOS;
      return 1U;

    case TG_EVENT_PERSON:
      /* For PERSON, value carries the distance in centimetres; zero means
       * that the remote did not provide a reliable distance. */
      *event_type = TG_EVT_PERSON;
      return 1U;

    case TG_EVENT_RADAR_CLEAR:
      if (packet->value != 0U)
      {
        return 0U;
      }
      *event_type = TG_EVT_RADAR_CLEAR;
      return 1U;

    case TG_EVENT_RADAR_HEALTH:
      if (packet->value > TG_VALUE_RADAR_HEALTHY)
      {
        return 0U;
      }
      *event_type = TG_EVT_RADAR_HEALTH;
      return 1U;

    default:
      return 0U;
  }
}

static uint8_t TripGuard_Gateway_IsNewRadarSequence(uint16_t sequence,
                                                     uint32_t now_ms)
{
  uint32_t age_ms;

  if (gateway_radar_sequence_valid == 0U)
  {
    return 1U;
  }

  age_ms = now_ms - tripguard_gateway_radar_last_packet_ms;
  if (age_ms > TRIPGUARD_GATEWAY_RADAR_TIMEOUT_MS)
  {
    /* Allow a remote that rebooted and restarted its sequence at zero to
     * establish a new session after the old session has timed out. */
    return 1U;
  }

  /* Signed modulo comparison accepts 65535 -> 0 and rejects copies/stale
   * frames. A forward jump is deliberately limited to half the sequence
   * space so ordering remains unambiguous. */
  return (uint8_t)((int16_t)(sequence -
                             tripguard_gateway_radar_last_sequence) > 0);
}

static uint8_t TripGuard_Gateway_UpdateRadar(
    const tripguard_spi_packet_t *packet,
    uint32_t now_ms)
{
  uint8_t is_new_sequence;

  if ((packet == NULL) ||
      (TripGuard_Gateway_IsRadarEvent(packet->event) == 0U))
  {
    return 1U;
  }

  is_new_sequence = TripGuard_Gateway_IsNewRadarSequence(packet->sequence,
                                                          now_ms);
  if (is_new_sequence == 0U)
  {
    tripguard_gateway_radar_duplicate_count++;
    return 0U;
  }

  gateway_radar_sequence_valid = 1U;
  tripguard_gateway_radar_last_sequence = packet->sequence;
  /* A stale copy must not keep an old sequence alive forever.  Update the
   * timestamp only after accepting a fresh sequence so a rebooted Gateway can
   * establish its restarted counter after the normal timeout. */
  tripguard_gateway_radar_last_packet_ms = now_ms;

  switch (packet->event)
  {
    case TG_EVENT_PERSON:
      /* Gateway only maps a PERSON packet when the Rear Pod fault flag is
       * clear. A fresh PERSON therefore also proves that the radar recovered,
       * even if the one-shot HEALTHY status packet was lost on BLE Mesh. */
      tripguard_gateway_radar_healthy = 1U;
      tripguard_gateway_radar_person = 1U;
      tripguard_gateway_radar_distance_cm = packet->value;
      tripguard_gateway_radar_person_count++;
      break;

    case TG_EVENT_RADAR_CLEAR:
      /* A fresh CLEAR packet is also a valid, fault-free radar sample. */
      tripguard_gateway_radar_healthy = 1U;
      tripguard_gateway_radar_person = 0U;
      tripguard_gateway_radar_distance_cm = 0U;
      tripguard_gateway_radar_clear_count++;
      break;

    case TG_EVENT_RADAR_HEALTH:
      tripguard_gateway_radar_healthy =
          (packet->value == TG_VALUE_RADAR_HEALTHY) ? 1U : 0U;
      if (tripguard_gateway_radar_healthy == 0U)
      {
        tripguard_gateway_radar_person = 0U;
        tripguard_gateway_radar_distance_cm = 0U;
      }
      tripguard_gateway_radar_health_count++;
      break;

    default:
      break;
  }
  return 1U;
}

bool TripGuard_Gateway_Init(osMessageQueueId_t event_queue,
                            osMessageQueueId_t ack_queue,
                            osEventFlagsId_t system_events)
{
  if ((event_queue == NULL) || (ack_queue == NULL) ||
      (system_events == NULL))
  {
    return false;
  }

  gateway_event_queue = event_queue;
  gateway_ack_queue = ack_queue;
  gateway_system_events = system_events;

  tripguard_gateway_radar_healthy = 0U;
  tripguard_gateway_radar_person = 0U;
  tripguard_gateway_radar_distance_cm = 0U;
  tripguard_gateway_radar_last_sequence = 0U;
  tripguard_gateway_radar_last_packet_ms = 0U;
  tripguard_gateway_radar_person_count = 0U;
  tripguard_gateway_radar_clear_count = 0U;
  tripguard_gateway_radar_health_count = 0U;
  tripguard_gateway_radar_duplicate_count = 0U;
  gateway_radar_sequence_valid = 0U;

  /* DMA is armed by GatewayLinkTask after the scheduler starts, so an early
   * SPI interrupt can never call the RTOS before the kernel is running. */
  return true;
}

static bool TripGuard_Gateway_ProcessRx(
    const tripguard_spi_rx_frame_t *raw_frame)
{
  tripguard_spi_packet_t packet;
  tripguard_event_t event;
  tripguard_event_type_t event_type;
  uint32_t now_ms;

  if ((raw_frame == NULL) ||
      !tripguard_spi_decode(raw_frame->bytes, &packet))
  {
    tripguard_gateway_invalid_rx_count++;
    return false;
  }

  if (TripGuard_Gateway_IsSupportedPacket(&packet, &event_type) == 0U)
  {
    tripguard_gateway_invalid_rx_count++;
    return false;
  }

  now_ms = HAL_GetTick();
  TripGuard_Supervisor_NotifyGatewayActivity();

  /* Copies 2/3 and 3/3 rearm SPI immediately; only a new radar sequence is
   * committed by SafetyTask and forwarded to the Pi. */
  if (TripGuard_Gateway_UpdateRadar(&packet, now_ms) == 0U)
  {
    tripguard_gateway_valid_rx_count++;
    return false;
  }

  event.type = event_type;
  event.source_node = packet.node_id;
  event.sequence = packet.sequence;
  event.value = packet.value;
  if (osMessageQueuePut(gateway_event_queue, &event, 0U, 0U) != osOK)
  {
    tripguard_gateway_queue_full_count++;
    return false;
  }

  tripguard_gateway_valid_rx_count++;
  (void)osEventFlagsSet(gateway_system_events,
                        TRIPGUARD_EVENT_GATEWAY_RX);
  return true;
}

void TripGuard_Gateway_Task(void *argument)
{
  tripguard_spi_rx_frame_t raw_frame;
  tripguard_ack_t ack;
  uint8_t request_pending = 0U;
  uint8_t pending_node = 0U;
  uint16_t pending_sequence = 0U;
  uint32_t pending_deadline = 0U;

  (void)argument;
  if (!TripGuard_Spi_Init(xTaskGetCurrentTaskHandle()))
  {
    /* Other vehicle functions remain alive if the optional SPI link fails. */
    for (;;)
    {
      vTaskDelay(pdMS_TO_TICKS(1000U));
    }
  }

  for (;;)
  {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20U));
    TripGuard_Spi_Service();

    if ((request_pending == 0U) && TripGuard_Spi_PopFrame(&raw_frame))
    {
      tripguard_spi_packet_t packet;

      if (tripguard_spi_decode(raw_frame.bytes, &packet) &&
          TripGuard_Gateway_ProcessRx(&raw_frame))
      {
        request_pending = 1U;
        pending_node = packet.node_id;
        pending_sequence = packet.sequence;
        pending_deadline = HAL_GetTick() + 1000U;
      }
      else
      {
        TripGuard_Spi_CompleteRequestWithoutAck();
      }
    }

    while ((request_pending != 0U) &&
           (osMessageQueueGet(gateway_ack_queue, &ack, NULL, 0U) == osOK))
    {
      if ((ack.node_id == pending_node) &&
          (ack.sequence == pending_sequence) &&
          (ack.value == TG_VALUE_ACK_OK) &&
          TripGuard_Spi_SendAck(ack.node_id, ack.sequence))
      {
        request_pending = 0U;
      }
      else
      {
        tripguard_gateway_stale_ack_count++;
      }
    }

    if ((request_pending != 0U) &&
        ((int32_t)(HAL_GetTick() - pending_deadline) >= 0))
    {
      tripguard_gateway_ack_timeout_count++;
      request_pending = 0U;
      TripGuard_Spi_CompleteRequestWithoutAck();
    }
  }
}

uint32_t TripGuard_Gateway_GetRadarLastPacketAgeMs(void)
{
  if (gateway_radar_sequence_valid == 0U)
  {
    return UINT32_MAX;
  }

  return HAL_GetTick() - tripguard_gateway_radar_last_packet_ms;
}

uint8_t TripGuard_Gateway_IsRadarOnline(void)
{
  return (uint8_t)(TripGuard_Gateway_GetRadarLastPacketAgeMs() <=
                   TRIPGUARD_GATEWAY_RADAR_TIMEOUT_MS);
}

uint8_t TripGuard_Gateway_IsRadarHealthy(void)
{
  return (uint8_t)((TripGuard_Gateway_IsRadarOnline() != 0U) &&
                   (tripguard_gateway_radar_healthy != 0U));
}

uint8_t TripGuard_Gateway_IsRadarPersonDetected(void)
{
  return (uint8_t)((TripGuard_Gateway_IsRadarOnline() != 0U) &&
                   (tripguard_gateway_radar_person != 0U));
}

uint16_t TripGuard_Gateway_GetRadarDistanceCm(void)
{
  return (TripGuard_Gateway_IsRadarOnline() != 0U) ?
         tripguard_gateway_radar_distance_cm : 0U;
}
