#include "tripguard_power.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "a7670.h"
#include "tb_mqtt.h"
#include "tripguard_bus_power.h"
#include "tripguard_config.h"
#include "tripguard_event.h"
#include "pi_link.h"
#include "tripguard_protocol.h"
#include "tripguard_rtos.h"
#include "tripguard_shutdown_button.h"
#include "tripguard_sleep.h"
#include "tripguard_spi.h"

#define TG_POWER_TASK_PERIOD_MS          20U
#define TG_PI_SHUTDOWN_TIMEOUT_MS     30000U
#define TG_PI_ACK_SETTLE_MS            5000U
#define TG_RTC_WAKE_PERIOD_SECONDS       60U
#define TG_REPORTING_WINDOW_MS         20000U
#define TG_SLEEP_RETRY_MS                250U
#define TG_WAKE_REMOTE_BIT          (1UL << 0)
#define TG_WAKE_BUS_BIT             (1UL << 1)
#define TG_WAKE_BUTTON_BIT          (1UL << 2)
#define TG_WAKE_TIMER_BIT           (1UL << 3)
#define TG_PI_ACK_BIT               (1UL << 4)
#define TG_WAKE_GPS_BIT             (1UL << 5)
#define TG_AUTO_IDLE_MS              60000U

#ifndef TRIPGUARD_POWER_HARDWARE_READY
#define TRIPGUARD_POWER_HARDWARE_READY  0U
#endif

static osMessageQueueId_t power_safety_queue;
static osMessageQueueId_t power_ack_queue;
static osMessageQueueId_t power_telemetry_queue;
static osMessageQueueId_t power_audio_queue;
static volatile uint32_t power_isr_events;
static volatile uint8_t power_idle_requested;
static uint32_t power_state_entered_ms;
static uint32_t power_shutdown_deadline_ms;
static uint32_t power_pi_ack_seen_ms;
static uint32_t power_station_candidate_ms;
static uint8_t power_station_candidate;

extern volatile uint8_t tripguard_alert_active;

volatile tg_power_state_t tripguard_power_state = TG_POWER_BOOT;
volatile tg_wake_reason_t tripguard_wake_reason = TG_WAKE_NONE;
volatile uint32_t tripguard_sleep_count;
volatile uint32_t tripguard_wake_count;
volatile uint32_t tripguard_wake_gps_count;
volatile uint32_t tripguard_wake_power_loss_count;
volatile uint32_t tripguard_wake_remote_count;
volatile uint32_t tripguard_shutdown_request_count;
volatile uint32_t tripguard_shutdown_cancel_count;
volatile uint32_t tripguard_shutdown_success_count;
volatile uint32_t tripguard_shutdown_timeout_count;
volatile uint32_t tripguard_power_transition_count;
volatile uint32_t tripguard_power_sleep_blocked_count;
volatile uint32_t tripguard_power_hw_inhibit_count;
volatile uint32_t tripguard_pi_ack_stuck_count;
volatile uint32_t tripguard_wake_pin_stuck_count;
volatile uint32_t tripguard_gps_verify_timeout_count;
volatile uint8_t tripguard_switched_rail_on = 1U;
volatile uint8_t tripguard_pi_power_on = 1U;
volatile uint8_t tripguard_modem_power_on = 1U;
volatile uint8_t tripguard_gps_power_on = 1U;
volatile uint8_t tripguard_audio_power_on = 1U;
volatile uint8_t tripguard_pi_shutdown_ack;
volatile uint32_t tripguard_power_rail_epoch;
volatile uint8_t tripguard_sleep_ready;

static uint8_t TripGuard_Power_TimeReached(uint32_t now, uint32_t deadline)
{
  return (uint8_t)(((int32_t)(now - deadline)) >= 0);
}

static void TripGuard_Power_SetState(tg_power_state_t state, uint32_t now)
{
  if (tripguard_power_state != state)
  {
    tripguard_power_state = state;
    power_state_entered_ms = now;
    tripguard_power_transition_count++;
  }
}

static void TripGuard_Power_SetRails(uint8_t pi_on,
                                     uint8_t modem_on,
                                     uint8_t gps_on,
                                     uint8_t audio_on)
{
  uint8_t enabling_new_rail;

  if (((pi_on == 0U) || (modem_on == 0U) ||
       (gps_on == 0U) || (audio_on == 0U)) &&
      (TRIPGUARD_POWER_HARDWARE_READY == 0U))
  {
    tripguard_power_hw_inhibit_count++;
    return;
  }

  enabling_new_rail = (uint8_t)(((pi_on != 0U) &&
                                 (tripguard_pi_power_on == 0U)) ||
                                ((modem_on != 0U) &&
                                 (tripguard_modem_power_on == 0U)) ||
                                ((gps_on != 0U) &&
                                 (tripguard_gps_power_on == 0U)) ||
                                ((audio_on != 0U) &&
                                 (tripguard_audio_power_on == 0U)));
  HAL_GPIO_WritePin(PI_POWER_EN_GPIO_Port, PI_POWER_EN_Pin,
                    (pi_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MODEM_POWER_EN_GPIO_Port, MODEM_POWER_EN_Pin,
                    (modem_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPS_POWER_EN_GPIO_Port, GPS_POWER_EN_Pin,
                    (gps_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AUDIO_POWER_EN_GPIO_Port, AUDIO_POWER_EN_Pin,
                    (audio_on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  tripguard_pi_power_on = (pi_on != 0U) ? 1U : 0U;
  tripguard_modem_power_on = (modem_on != 0U) ? 1U : 0U;
  tripguard_gps_power_on = (gps_on != 0U) ? 1U : 0U;
  tripguard_audio_power_on = (audio_on != 0U) ? 1U : 0U;
  tripguard_switched_rail_on = (uint8_t)(tripguard_pi_power_on ||
                                         tripguard_modem_power_on ||
                                         tripguard_gps_power_on ||
                                         tripguard_audio_power_on);
  if (enabling_new_rail != 0U)
  {
    tripguard_power_rail_epoch++;
  }
}

static uint8_t TripGuard_Power_CanSleep(void)
{
  if ((power_safety_queue == NULL) || (power_ack_queue == NULL) ||
      (power_telemetry_queue == NULL) || (power_audio_queue == NULL))
  {
    return 0U;
  }
  if ((osMessageQueueGetCount(power_safety_queue) != 0U) ||
      (osMessageQueueGetCount(power_ack_queue) != 0U) ||
      !TripGuard_Spi_IsIdle() ||
      (tripguard_alert_active != 0U) ||
      (A7670_IsExclusiveSessionActive() != 0U) ||
      (TB_MQTT_IsQuiesced() == 0U))
  {
    return 0U;
  }
  return 1U;
}

static tg_wake_reason_t TripGuard_Power_ResolveWakeReason(uint32_t events)
{
  if ((events & TG_WAKE_GPS_BIT) != 0U)
  {
    return TG_WAKE_GPS_STATION;
  }
  if ((events & TG_WAKE_REMOTE_BIT) != 0U)
  {
    return TG_WAKE_REMOTE_REAR_CONFIRM;
  }
  if ((events & TG_WAKE_BUTTON_BIT) != 0U)
  {
    return TG_WAKE_DRIVER_BUTTON;
  }
  if ((events & TG_WAKE_BUS_BIT) != 0U)
  {
    return (HAL_GPIO_ReadPin(BUS_POWER_SENSE_GPIO_Port,
                             BUS_POWER_SENSE_Pin) == GPIO_PIN_RESET) ?
        TG_WAKE_BUS_POWER_LOST : TG_WAKE_OTHER;
  }
  if ((events & TG_WAKE_TIMER_BIT) != 0U)
  {
    return TG_WAKE_TIMER;
  }
  return TG_WAKE_OTHER;
}

void TripGuard_Power_Init(osMessageQueueId_t safety_queue,
                          osMessageQueueId_t ack_queue,
                          osMessageQueueId_t telemetry_queue,
                          osMessageQueueId_t audio_queue)
{
  uint32_t now = HAL_GetTick();

  power_safety_queue = safety_queue;
  power_ack_queue = ack_queue;
  power_telemetry_queue = telemetry_queue;
  power_audio_queue = audio_queue;
  power_isr_events = 0U;
  power_idle_requested = 0U;
  power_station_candidate = 0U;
  power_pi_ack_seen_ms = 0U;
  tripguard_pi_shutdown_ack = 0U;
  tripguard_switched_rail_on = 1U;
  tripguard_pi_power_on = 1U;
  tripguard_modem_power_on = 1U;
  tripguard_gps_power_on = 1U;
  tripguard_audio_power_on = 1U;
  TripGuard_ShutdownButton_Init(now);
  TripGuard_BusPower_Init(now);
  tripguard_sleep_ready = TripGuard_Sleep_Init() ? 1U : 0U;
  TripGuard_Power_SetState(TG_POWER_ACTIVE, now);
}

void TripGuard_Power_NotifyGpioFromIsr(uint16_t gpio_pin)
{
  if (gpio_pin == BUS_POWER_SENSE_Pin)
  {
    power_isr_events |= TG_WAKE_BUS_BIT;
    TripGuard_BusPower_NotifyEdgeFromIsr();
  }
  else if (gpio_pin == SHUTDOWN_BUTTON_Pin)
  {
    power_isr_events |= TG_WAKE_BUTTON_BIT;
    TripGuard_ShutdownButton_NotifyEdgeFromIsr();
  }
  else if (gpio_pin == PI_SHUTDOWN_ACK_Pin)
  {
    power_isr_events |= TG_PI_ACK_BIT;
    tripguard_pi_shutdown_ack = 1U;
  }
}

void TripGuard_Power_NotifyRtcFromIsr(void)
{
  power_isr_events |= TG_WAKE_TIMER_BIT;
}

void TripGuard_Power_NotifyRemoteCommitted(void)
{
  taskENTER_CRITICAL();
  power_isr_events |= TG_WAKE_REMOTE_BIT;
  taskEXIT_CRITICAL();
}

void TripGuard_Power_RequestIdle(void)
{
  if (TRIPGUARD_POWER_HARDWARE_READY == 0U)
  {
    tripguard_power_hw_inhibit_count++;
    return;
  }
  if (PiLink_GetCameraPerson() != 0U)
  {
    /* Never remove evidence connectivity while OCCUPIED is latched. */
    tripguard_power_sleep_blocked_count++;
    return;
  }
  power_idle_requested = 1U;
}

uint8_t TripGuard_Power_IsSwitchedRailOn(void)
{
  return tripguard_switched_rail_on;
}

uint8_t TripGuard_Power_IsPiOn(void)
{
  return tripguard_pi_power_on;
}

uint8_t TripGuard_Power_IsModemOn(void)
{
  return tripguard_modem_power_on;
}

uint8_t TripGuard_Power_IsGpsOn(void)
{
  return tripguard_gps_power_on;
}

uint8_t TripGuard_Power_IsAudioOn(void)
{
  return tripguard_audio_power_on;
}

void TripGuard_Power_ReportGps(float latitude,
                               float longitude,
                               float speed_kmh,
                               uint8_t position_valid)
{
  float dx;
  float dy;
  float distance_squared;
  float radius_squared;
  uint32_t now = HAL_GetTick();
  uint8_t in_station;

  if ((TRIPGUARD_STATION_GEOFENCE_ENABLED == 0U) ||
      (position_valid == 0U))
  {
    power_station_candidate = 0U;
    return;
  }

  dy = (latitude - TRIPGUARD_STATION_LATITUDE) * 111320.0f;
  dx = (longitude - TRIPGUARD_STATION_LONGITUDE) *
       TRIPGUARD_STATION_LON_METERS_PER_DEG;
  distance_squared = (dx * dx) + (dy * dy);
  radius_squared = TRIPGUARD_STATION_RADIUS_M *
                   TRIPGUARD_STATION_RADIUS_M;
  in_station = (uint8_t)((distance_squared <= radius_squared) &&
                         (speed_kmh <= TRIPGUARD_STATION_MAX_SPEED_KMH));

  if (in_station == 0U)
  {
    power_station_candidate = 0U;
    return;
  }
  if (power_station_candidate == 0U)
  {
    power_station_candidate = 1U;
    power_station_candidate_ms = now;
  }
  else if ((uint32_t)(now - power_station_candidate_ms) >=
           TRIPGUARD_STATION_CONFIRM_MS)
  {
    taskENTER_CRITICAL();
    power_isr_events |= TG_WAKE_GPS_BIT;
    taskEXIT_CRITICAL();
    power_station_candidate = 0U;
  }
}

static void TripGuard_Power_BeginShutdown(uint32_t now)
{
  uint32_t audio_stop = (uint32_t)TRIPGUARD_AUDIO_STOP;

  if (PiLink_GetCameraPerson() != 0U)
  {
    power_idle_requested = 0U;
    tripguard_shutdown_cancel_count++;
    tripguard_power_sleep_blocked_count++;
    TripGuard_Power_SetState(TG_POWER_ACTIVE, now);
    return;
  }

  tripguard_shutdown_request_count++;
  tripguard_pi_shutdown_ack = 0U;
  power_pi_ack_seen_ms = 0U;
  if (HAL_GPIO_ReadPin(PI_SHUTDOWN_ACK_GPIO_Port,
                       PI_SHUTDOWN_ACK_Pin) == GPIO_PIN_SET)
  {
    /* A pre-asserted ACK cannot prove that this shutdown completed. */
    tripguard_pi_ack_stuck_count++;
  }
  power_shutdown_deadline_ms = now + TG_PI_SHUTDOWN_TIMEOUT_MS;
  TB_MQTT_AbortForEmergency();
  (void)osMessageQueuePut(power_audio_queue, &audio_stop, 0U, 0U);
  (void)PiLink_RequestScanStop(PI_SCAN_STOP_REASON_SHUTDOWN);
  if (tripguard_pi_power_on != 0U)
  {
    HAL_GPIO_WritePin(PI_SHUTDOWN_REQ_GPIO_Port,
                      PI_SHUTDOWN_REQ_Pin,
                      GPIO_PIN_SET);
  }
  else
  {
    /* No powered Pi means there is no Linux filesystem to wait for. */
    tripguard_pi_shutdown_ack = 1U;
    power_pi_ack_seen_ms = now;
  }
  TripGuard_Power_SetState(TG_POWER_SHUTTING_DOWN, now);
}

static void TripGuard_Power_EnterSleep(uint32_t now)
{
  uint32_t wake_events;

  if ((TRIPGUARD_USE_STOP_MODE == 0U) ||
      (TRIPGUARD_POWER_HARDWARE_READY == 0U))
  {
    tripguard_power_hw_inhibit_count++;
    TB_MQTT_ResumeAfterEmergency();
    TripGuard_Power_SetState(TG_POWER_ACTIVE, now);
    return;
  }
  if (tripguard_sleep_ready == 0U)
  {
    tripguard_power_sleep_blocked_count++;
    TB_MQTT_ResumeAfterEmergency();
    TripGuard_Power_SetState(TG_POWER_ACTIVE, now);
    return;
  }
  if (TripGuard_Power_CanSleep() == 0U)
  {
    tripguard_power_sleep_blocked_count++;
    return;
  }

  TripGuard_Power_SetState(TG_POWER_SLEEP, now);
  TripGuard_Spi_PrepareForStop();

  /* A queued remote event blocks the optional legacy STOP entry. */
  __DMB();
  if ((power_isr_events & TG_WAKE_REMOTE_BIT) != 0U)
  {
    (void)TripGuard_Spi_ResumeAfterStop();
    TB_MQTT_ResumeAfterEmergency();
    tripguard_power_sleep_blocked_count++;
    TripGuard_Power_SetState(TG_POWER_ACTIVE, HAL_GetTick());
    return;
  }
  tripguard_sleep_count++;
  if (!TripGuard_Sleep_EnterStop(TG_RTC_WAKE_PERIOD_SECONDS))
  {
    tripguard_power_sleep_blocked_count++;
    (void)TripGuard_Spi_ResumeAfterStop();
    TripGuard_Power_SetState(TG_POWER_ACTIVE, HAL_GetTick());
    return;
  }

  if (TripGuard_Sleep_TakeRtcWake() != 0U)
  {
    power_isr_events |= TG_WAKE_TIMER_BIT;
  }
  wake_events = power_isr_events;
  tripguard_wake_reason = TripGuard_Power_ResolveWakeReason(wake_events);
  tripguard_wake_count++;
  if (tripguard_wake_reason == TG_WAKE_REMOTE_REAR_CONFIRM)
  {
    tripguard_wake_remote_count++;
  }
  else if (tripguard_wake_reason == TG_WAKE_BUS_POWER_LOST)
  {
    tripguard_wake_power_loss_count++;
  }

  if (tripguard_wake_reason == TG_WAKE_TIMER)
  {
    TripGuard_Power_SetRails(0U, 0U, 1U, 0U);
  }
  else if (tripguard_wake_reason == TG_WAKE_REMOTE_REAR_CONFIRM)
  {
    TripGuard_Power_SetRails(1U, 1U, 1U, 1U);
  }
  else
  {
    TripGuard_Power_SetRails(1U, 1U, 1U, 1U);
  }
  (void)TripGuard_Spi_ResumeAfterStop();
  if (tripguard_modem_power_on != 0U)
  {
    TB_MQTT_ResumeAfterEmergency();
  }
  power_idle_requested = 0U;
  TripGuard_Power_SetState(TG_POWER_WAKE, HAL_GetTick());
}

void TripGuard_Power_Task(void *argument)
{
  uint32_t now;
  uint32_t events;
  tg_bus_power_event_t bus_event;

  (void)argument;
  for (;;)
  {
    now = HAL_GetTick();
    taskENTER_CRITICAL();
    events = power_isr_events;
    power_isr_events &= ~events;
    taskEXIT_CRITICAL();
    bus_event = TG_BUS_POWER_NO_CHANGE;

    if (TripGuard_ShutdownButton_Poll(now))
    {
      TripGuard_Power_SetState(TG_POWER_SHUTDOWN_REQUEST, now);
    }

    if ((TRIPGUARD_LOGICAL_STANDBY_ENABLED == 0U) &&
        (bus_event == TG_BUS_POWER_LOST))
    {
      tripguard_wake_reason = TG_WAKE_BUS_POWER_LOST;
      tripguard_wake_power_loss_count++;
      TripGuard_Power_SetState(TG_POWER_REPORTING, now);
    }
    else if ((TRIPGUARD_LOGICAL_STANDBY_ENABLED == 0U) &&
             (bus_event == TG_BUS_POWER_RESTORED))
    {
      power_idle_requested = 0U;
      TripGuard_Power_SetRails(1U, 1U, 1U, 1U);
      TripGuard_Power_SetState(TG_POWER_ACTIVE, now);
    }

    if ((events & TG_WAKE_REMOTE_BIT) != 0U)
    {
      tripguard_wake_reason = TG_WAKE_REMOTE_REAR_CONFIRM;
      if (tripguard_power_state != TG_POWER_ACTIVE)
      {
        tripguard_wake_remote_count++;
      }
      power_idle_requested = 0U;
      TripGuard_Power_SetRails(1U, 1U, 1U, 1U);
      TripGuard_Power_SetState(TG_POWER_ACTIVE, now);
    }
    if ((events & TG_WAKE_GPS_BIT) != 0U)
    {
      tripguard_wake_reason = TG_WAKE_GPS_STATION;
      tripguard_wake_gps_count++;
      power_idle_requested = 0U;
      TripGuard_Power_SetRails(1U, 1U, 1U, 1U);
      TripGuard_Power_SetState(TG_POWER_ACTIVE, now);
    }

    switch (tripguard_power_state)
    {
      case TG_POWER_BOOT:
        TripGuard_Power_SetState(TG_POWER_ACTIVE, now);
        break;

      case TG_POWER_SHUTDOWN_REQUEST:
        TripGuard_Power_BeginShutdown(now);
        break;

      case TG_POWER_SHUTTING_DOWN:
        if (((events & TG_PI_ACK_BIT) != 0U) ||
            (tripguard_pi_shutdown_ack != 0U))
        {
          if (power_pi_ack_seen_ms == 0U)
          {
            power_pi_ack_seen_ms = now;
          }
          if ((tripguard_sleep_ready != 0U) &&
              ((tripguard_pi_power_on == 0U) ||
               (HAL_GPIO_ReadPin(PI_SHUTDOWN_ACK_GPIO_Port,
                                 PI_SHUTDOWN_ACK_Pin) == GPIO_PIN_SET)) &&
              ((uint32_t)(now - power_pi_ack_seen_ms) >=
               TG_PI_ACK_SETTLE_MS) &&
              (TB_MQTT_IsQuiesced() != 0U) &&
              (A7670_IsExclusiveSessionActive() == 0U))
          {
            HAL_GPIO_WritePin(PI_SHUTDOWN_REQ_GPIO_Port,
                              PI_SHUTDOWN_REQ_Pin,
                              GPIO_PIN_RESET);
            if (TRIPGUARD_POWER_HARDWARE_READY != 0U)
            {
              TripGuard_Power_SetRails(0U, 0U, 0U, 0U);
              tripguard_shutdown_success_count++;
              TripGuard_Power_SetState(TG_POWER_OFF, now);
            }
            else
            {
              tripguard_power_hw_inhibit_count++;
              tripguard_shutdown_cancel_count++;
              TB_MQTT_ResumeAfterEmergency();
              TripGuard_Power_SetState(TG_POWER_ACTIVE, now);
            }
          }
        }
        if ((tripguard_power_state == TG_POWER_SHUTTING_DOWN) &&
            (TripGuard_Power_TimeReached(
                 now, power_shutdown_deadline_ms) != 0U))
        {
          HAL_GPIO_WritePin(PI_SHUTDOWN_REQ_GPIO_Port,
                            PI_SHUTDOWN_REQ_Pin,
                            GPIO_PIN_RESET);
          tripguard_shutdown_timeout_count++;
          tripguard_shutdown_cancel_count++;
          TB_MQTT_ResumeAfterEmergency();
          TripGuard_Power_SetState(TG_POWER_ACTIVE, now);
        }
        break;

      case TG_POWER_REPORTING:
        if ((tripguard_wake_reason == TG_WAKE_GPS_STATION) ||
            TripGuard_Power_TimeReached(
                now, power_state_entered_ms + TG_REPORTING_WINDOW_MS))
        {
          if (tripguard_wake_reason != TG_WAKE_GPS_STATION)
          {
            tripguard_gps_verify_timeout_count++;
          }
          power_idle_requested = 1U;
          TripGuard_Power_SetState(TG_POWER_IDLE, now);
        }
        break;

      case TG_POWER_ACTIVE:
        if ((TRIPGUARD_LOGICAL_STANDBY_ENABLED == 0U) &&
            (TRIPGUARD_POWER_HARDWARE_READY != 0U) &&
            (TripGuard_BusPower_IsPresent() == 0U) &&
            ((uint32_t)(now - power_state_entered_ms) >= TG_AUTO_IDLE_MS))
        {
          power_idle_requested = 1U;
        }
        if (power_idle_requested != 0U)
        {
          TripGuard_Power_SetState(TG_POWER_IDLE, now);
        }
        break;

      case TG_POWER_IDLE:
        if (tripguard_switched_rail_on != 0U)
        {
          TripGuard_Power_SetState(TG_POWER_SHUTDOWN_REQUEST, now);
        }
        else
        {
          TB_MQTT_AbortForEmergency();
          TripGuard_Power_SetState(TG_POWER_PREPARE_SLEEP, now);
        }
        break;

      case TG_POWER_PREPARE_SLEEP:
        if ((uint32_t)(now - power_state_entered_ms) >= TG_SLEEP_RETRY_MS)
        {
          TripGuard_Power_EnterSleep(now);
        }
        break;

      case TG_POWER_OFF:
        TripGuard_Power_EnterSleep(now);
        break;

      case TG_POWER_WAKE:
        if (tripguard_wake_reason == TG_WAKE_TIMER)
        {
          TripGuard_Power_SetState(TG_POWER_REPORTING, now);
        }
        else
        {
          TripGuard_Power_SetState(TG_POWER_ACTIVE, now);
        }
        break;

      case TG_POWER_SLEEP:
      default:
        break;
    }

    osDelay(TG_POWER_TASK_PERIOD_MS);
  }
}
