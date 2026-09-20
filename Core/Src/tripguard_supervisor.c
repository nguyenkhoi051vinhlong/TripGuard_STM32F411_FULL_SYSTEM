#include "tripguard_supervisor.h"

#include "camera.h"
#include "main.h"
#include "a7670.h"
#include "pi_link.h"
#include "tb_mqtt.h"
#include "tripguard_bus_power.h"
#include "tripguard_config.h"
#include "tripguard_event.h"
#include "tripguard_gateway.h"
#include "tripguard_power.h"
#include "tripguard_rtos.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#define TG_SUPERVISOR_PERIOD_MS          50U
#define TG_GATEWAY_ONLINE_TIMEOUT_MS 120000U
#define TG_SELF_TEST_RETRY_PERIOD_MS    5000U

static osMessageQueueId_t supervisor_event_queue;
static osMessageQueueId_t supervisor_rpc_result_queue;
static osMessageQueueId_t supervisor_telemetry_queue;

volatile TripGuard_SystemState_t tripguard_system_state;
volatile TripGuard_ActivationSource_t tripguard_activation_source;
volatile uint32_t tripguard_state_started_ms;
volatile uint8_t tripguard_supervisor_rear_confirmed;
volatile uint8_t tripguard_station_detected;
volatile uint8_t tripguard_supervisor_gps_fix_valid;
volatile uint8_t tripguard_supervisor_radar_enabled;
volatile uint8_t tripguard_supervisor_camera_enabled;
volatile uint32_t tripguard_rear_confirm_wait_remaining_s;
volatile uint32_t tripguard_rear_confirm_timeout_count;

static volatile uint32_t gateway_last_activity_ms;
static volatile uint8_t gateway_activity_seen;
static char last_activation_reason[32];
static char last_error[64];

static uint32_t geofence_since_ms;
static uint8_t geofence_timing;
static float latest_speed_kmh;
static uint32_t latest_gps_age_ms;
static uint8_t self_test_pass_mask;
static uint32_t self_test_publish_baseline;
static uint32_t self_test_last_retry_ms;
static uint32_t self_test_retry_count;

volatile uint32_t tripguard_supervisor_event_drop_count;
volatile uint32_t tripguard_supervisor_rpc_result_drop_count;

static uint8_t TripGuard_Supervisor_TimeReached(uint32_t now_ms,
                                                uint32_t started_ms,
                                                uint32_t interval_ms)
{
  return ((uint32_t)(now_ms - started_ms) >= interval_ms) ? 1U : 0U;
}

static void TripGuard_Supervisor_Copy(char *destination, size_t size,
                                      const char *source)
{
  if ((destination == NULL) || (size == 0U))
  {
    return;
  }
  (void)snprintf(destination, size, "%s", (source != NULL) ? source : "");
}

const char *TripGuard_Supervisor_StateName(TripGuard_SystemState_t state)
{
  switch (state)
  {
    case SYSTEM_SELF_TEST: return "SELF_TEST";
    case SYSTEM_STANDBY:   return "STANDBY";
    case SYSTEM_ARM_DELAY: return "ARM_DELAY";
    case SYSTEM_ACTIVE:    return "ACTIVE";
    case SYSTEM_FAULT:     return "FAULT";
    case SYSTEM_WAIT_REAR_CONFIRM: return "WAIT_REAR_CONFIRM";
    default:               return "FAULT";
  }
}

const char *TripGuard_Supervisor_SourceName(TripGuard_ActivationSource_t source)
{
  switch (source)
  {
    case ACTIVATION_MANUAL_REAR_CONFIRM: return "MANUAL_REAR_CONFIRM";
    case ACTIVATION_AUTO_REAR_TIMEOUT:    return "AUTO_REAR_TIMEOUT";
    case ACTIVATION_NONE:
    default:                             return "NONE";
  }
}

static void TripGuard_Supervisor_QueueTelemetry(void)
{
  uint32_t event = (uint32_t)TRIPGUARD_TELEMETRY_SAFETY_EVENT;

  if ((supervisor_telemetry_queue != NULL) &&
      (osMessageQueuePut(supervisor_telemetry_queue,
                         &event, 0U, 0U) != osOK))
  {
    tripguard_supervisor_event_drop_count++;
  }
}

static void TripGuard_Supervisor_SetDetection(uint8_t enabled)
{
  tripguard_supervisor_radar_enabled = (enabled != 0U) ? 1U : 0U;
  tripguard_supervisor_camera_enabled = tripguard_supervisor_radar_enabled;
  Camera_Enable(tripguard_supervisor_camera_enabled);
}

static void TripGuard_Supervisor_EnterStandby(uint32_t now_ms,
                                              uint8_t scan_stop_reason)
{
  tripguard_system_state = SYSTEM_STANDBY;
  tripguard_activation_source = ACTIVATION_NONE;
  tripguard_state_started_ms = now_ms;
  geofence_timing = 0U;
  tripguard_supervisor_rear_confirmed = 0U;
  tripguard_station_detected = 0U;
  tripguard_rear_confirm_wait_remaining_s = 0U;
  last_error[0] = '\0';
  TripGuard_Supervisor_SetDetection(0U);
  (void)PiLink_RequestScanStop(scan_stop_reason);
  if (PiLink_GetCameraPerson() == 0U)
  {
    TripGuard_Power_RequestIdle();
  }
  TripGuard_Supervisor_QueueTelemetry();
}

static void TripGuard_Supervisor_StartArm(uint32_t now_ms,
                                         TripGuard_ActivationSource_t source)
{
  if (((tripguard_system_state != SYSTEM_STANDBY) &&
       (tripguard_system_state != SYSTEM_WAIT_REAR_CONFIRM)) ||
      ((source != ACTIVATION_MANUAL_REAR_CONFIRM) &&
       (source != ACTIVATION_AUTO_REAR_TIMEOUT)))
  {
    return;
  }

  tripguard_activation_source = source;
  tripguard_system_state = SYSTEM_ARM_DELAY;
  tripguard_state_started_ms = now_ms;
  TripGuard_Supervisor_Copy(last_activation_reason,
                            sizeof(last_activation_reason),
                            TripGuard_Supervisor_SourceName(source));
  (void)PiLink_RequestScanStart(
      (uint16_t)TRIPGUARD_CAMERA_SCAN_DURATION_SECONDS);
  TripGuard_Supervisor_QueueTelemetry();
}

static void TripGuard_Supervisor_StartRearConfirmWait(uint32_t now_ms)
{
  if (tripguard_system_state != SYSTEM_STANDBY)
  {
    return;
  }

  tripguard_system_state = SYSTEM_WAIT_REAR_CONFIRM;
  tripguard_activation_source = ACTIVATION_NONE;
  tripguard_state_started_ms = now_ms;
  tripguard_rear_confirm_wait_remaining_s =
      TRIPGUARD_REAR_CONFIRM_TIMEOUT_MS / 1000U;
  TripGuard_Supervisor_Copy(last_activation_reason,
                            sizeof(last_activation_reason),
                            "GPS_ARRIVAL_WAIT_REAR");
  TripGuard_Supervisor_QueueTelemetry();
}

static uint8_t TripGuard_Supervisor_InGeofence(
    const TripGuard_SupervisorEvent_t *event)
{
#if (TRIPGUARD_GEOFENCE_ENABLED != 0U)
  float latitude_delta_m;
  float longitude_delta_m;
  float distance_squared;
  float radius_squared;

  if ((event->data.gps.fix_valid == 0U) ||
      (event->data.gps.age_ms > TRIPGUARD_GPS_MAX_AGE_MS) ||
      (event->data.gps.speed_kmh > TRIPGUARD_MAX_ARRIVAL_SPEED_KMH))
  {
    return 0U;
  }

  latitude_delta_m = (event->data.gps.latitude - TRIPGUARD_STATION_LAT) *
                     111320.0f;
  longitude_delta_m = (event->data.gps.longitude - TRIPGUARD_STATION_LON) *
                      TRIPGUARD_STATION_LON_METERS_PER_DEG;
  distance_squared = (latitude_delta_m * latitude_delta_m) +
                     (longitude_delta_m * longitude_delta_m);
  radius_squared = TRIPGUARD_STATION_RADIUS_M * TRIPGUARD_STATION_RADIUS_M;
  return (distance_squared <= radius_squared) ? 1U : 0U;
#else
  (void)event;
  return 0U;
#endif
}

static void TripGuard_Supervisor_SelfTestStep(uint32_t now_ms)
{
  if (tripguard_system_state != SYSTEM_SELF_TEST)
  {
    return;
  }

#if (TRIPGUARD_CONNECTIVITY_OWNER_PI == 0U)
  if ((A7670_IsRegistered() != 0U) &&
      (A7670_IsNetworkReady() != 0U))
  {
    self_test_pass_mask |= TRIPGUARD_SELF_TEST_SIM_OK;
  }
  else
  {
    self_test_pass_mask &= (uint8_t)~TRIPGUARD_SELF_TEST_SIM_OK;
  }

  if ((TB_MQTT_IsConnected() != 0U) &&
      (TB_MQTT_GetPublishCount() > self_test_publish_baseline))
  {
    self_test_pass_mask |= TRIPGUARD_SELF_TEST_MQTT_PUBLISH_OK;
  }
  else
  {
    self_test_pass_mask &= (uint8_t)~TRIPGUARD_SELF_TEST_MQTT_PUBLISH_OK;
  }
#else
  /* In Pi-gateway mode, modem and cloud health belong to the Pi. */
  self_test_pass_mask &= (uint8_t)~(
      TRIPGUARD_SELF_TEST_SIM_OK | TRIPGUARD_SELF_TEST_MQTT_PUBLISH_OK);
#endif

  if (TripGuard_Gateway_IsRadarHealthy() != 0U)
  {
    self_test_pass_mask |= TRIPGUARD_SELF_TEST_RADAR_HEALTH_OK;
  }
  else
  {
    self_test_pass_mask &= (uint8_t)~TRIPGUARD_SELF_TEST_RADAR_HEALTH_OK;
  }

  if ((PiLink_IsOnline() != 0U) && (PiLink_IsReady() != 0U))
  {
    self_test_pass_mask |= TRIPGUARD_SELF_TEST_PI_READY_OK;
  }
  else
  {
    self_test_pass_mask &= (uint8_t)~TRIPGUARD_SELF_TEST_PI_READY_OK;
  }

  if ((self_test_pass_mask & TRIPGUARD_SELF_TEST_REQUIRED_MASK) ==
      TRIPGUARD_SELF_TEST_REQUIRED_MASK)
  {
    TripGuard_Supervisor_EnterStandby(now_ms,
                                      PI_SCAN_STOP_REASON_STANDBY);
    return;
  }

  if (TripGuard_Supervisor_TimeReached(now_ms, self_test_last_retry_ms,
                                       TG_SELF_TEST_RETRY_PERIOD_MS) != 0U)
  {
    self_test_last_retry_ms = now_ms;
    self_test_retry_count++;
    TripGuard_Supervisor_Copy(last_error, sizeof(last_error),
                              "SELF_TEST_WAITING");
    TripGuard_Supervisor_QueueTelemetry();
  }
}

static void TripGuard_Supervisor_PublishRpcResult(uint32_t request_id,
                                                  uint8_t success,
                                                  const char *message)
{
  TripGuard_SupervisorRpcResult_t result;

  memset(&result, 0, sizeof(result));
  result.request_id = request_id;
  result.success = success;
  (void)snprintf(result.json, sizeof(result.json),
                 "{\"success\":%s,\"state\":\"%s\",\"message\":\"%s\"}",
                 (success != 0U) ? "true" : "false",
                 TripGuard_Supervisor_StateName(tripguard_system_state),
                 (message != NULL) ? message : "");

  if ((supervisor_rpc_result_queue == NULL) ||
      (osMessageQueuePut(supervisor_rpc_result_queue,
                         &result, 0U, 0U) != osOK))
  {
    tripguard_supervisor_rpc_result_drop_count++;
  }
}

static void TripGuard_Supervisor_HandleEvent(
    const TripGuard_SupervisorEvent_t *event, uint32_t now_ms)
{
  uint8_t inside;

  switch (event->type)
  {
    case TRIPGUARD_SUP_EVT_REAR_CONFIRM:
      if ((tripguard_system_state == SYSTEM_STANDBY) ||
          (tripguard_system_state == SYSTEM_WAIT_REAR_CONFIRM))
      {
        tripguard_supervisor_rear_confirmed = 1U;
        TripGuard_Supervisor_StartArm(now_ms,
                                      ACTIVATION_MANUAL_REAR_CONFIRM);
      }
      break;

    case TRIPGUARD_SUP_EVT_GPS_SAMPLE:
      latest_speed_kmh = event->data.gps.speed_kmh;
      latest_gps_age_ms = event->data.gps.age_ms;
      tripguard_supervisor_gps_fix_valid = ((event->data.gps.fix_valid != 0U) &&
                       (event->data.gps.age_ms <=
                        TRIPGUARD_GPS_MAX_AGE_MS)) ? 1U : 0U;
      inside = TripGuard_Supervisor_InGeofence(event);
      if ((inside != 0U) && (geofence_timing == 0U))
      {
        geofence_timing = 1U;
        geofence_since_ms = now_ms;
      }
      else if (inside == 0U)
      {
        geofence_timing = 0U;
        geofence_since_ms = 0U;
        tripguard_station_detected = 0U;
      }
      else if ((tripguard_station_detected == 0U) &&
               (TripGuard_Supervisor_TimeReached(
                    now_ms, geofence_since_ms,
                    TRIPGUARD_GEOFENCE_DWELL_MS) != 0U))
      {
        tripguard_station_detected = 1U;
        TripGuard_Supervisor_StartRearConfirmWait(now_ms);
      }
      break;

    case TRIPGUARD_SUP_EVT_RPC_SET_ARMED:
      if (event->data.armed != 0U)
      {
        TripGuard_Supervisor_PublishRpcResult(event->request_id, 0U,
                                              "rear_confirm_required");
      }
      else if ((tripguard_system_state == SYSTEM_SELF_TEST) ||
               (tripguard_system_state == SYSTEM_FAULT))
      {
        TripGuard_Supervisor_PublishRpcResult(event->request_id, 0U,
                                              "state_change_rejected");
      }
      else
      {
        TripGuard_Supervisor_EnterStandby(now_ms,
                                          PI_SCAN_STOP_REASON_STANDBY);
        TripGuard_Supervisor_PublishRpcResult(event->request_id, 1U,
                                              "disarmed");
      }
      break;

    case TRIPGUARD_SUP_EVT_RPC_CANCEL_ARM:
      if (tripguard_system_state == SYSTEM_ARM_DELAY)
      {
        TripGuard_Supervisor_EnterStandby(now_ms,
                                          PI_SCAN_STOP_REASON_STANDBY);
        TripGuard_Supervisor_PublishRpcResult(event->request_id, 1U,
                                              "arm_cancelled");
      }
      else if (tripguard_system_state == SYSTEM_STANDBY)
      {
        TripGuard_Supervisor_PublishRpcResult(event->request_id, 1U,
                                              "already_standby");
      }
      else
      {
        TripGuard_Supervisor_PublishRpcResult(event->request_id, 0U,
                                              "cannot_cancel_active");
      }
      break;

    case TRIPGUARD_SUP_EVT_RPC_GET_STATE:
      TripGuard_Supervisor_PublishRpcResult(event->request_id, 1U,
                                            "state_read");
      break;

    case TRIPGUARD_SUP_EVT_PI_SCAN_COMPLETE:
      if (event->data.scan.result == PI_SCAN_RESULT_CLEAR)
      {
        if ((tripguard_system_state == SYSTEM_ARM_DELAY) ||
            (tripguard_system_state == SYSTEM_ACTIVE))
        {
          TripGuard_Supervisor_EnterStandby(
              now_ms, PI_SCAN_STOP_REASON_STANDBY);
        }
      }
      else if (event->data.scan.result == PI_SCAN_RESULT_OCCUPIED)
      {
        /* OCCUPIED is deliberately latched until the driver acknowledges. */
        TripGuard_Supervisor_Copy(last_error, sizeof(last_error),
                                  "CAMERA_OCCUPIED");
        TripGuard_Supervisor_QueueTelemetry();
      }
      else if (event->data.scan.result == PI_SCAN_RESULT_FAULT)
      {
        /* Fail safe: keep the system awake when the 15-minute scan failed. */
        TripGuard_Supervisor_Copy(last_error, sizeof(last_error),
                                  "PI_CAMERA_SCAN_FAULT");
        TripGuard_Supervisor_QueueTelemetry();
      }
      break;

    case TRIPGUARD_SUP_EVT_OCCUPANCY_RESOLVED:
      if (PiLink_GetCameraPerson() != 0U)
      {
        PiLink_ClearCameraOccupancy();
        TripGuard_Supervisor_Copy(last_activation_reason,
                                  sizeof(last_activation_reason),
                                  "DRIVER_OCCUPANCY_ACK");
        TripGuard_Supervisor_EnterStandby(
            now_ms, PI_SCAN_STOP_REASON_CANCELLED);
      }
      break;

    default:
      TripGuard_Supervisor_Copy(last_error, sizeof(last_error),
                                "UNKNOWN_SUPERVISOR_EVENT");
      break;
  }
}

uint8_t TripGuard_Supervisor_Init(osMessageQueueId_t event_queue,
                                  osMessageQueueId_t rpc_result_queue,
                                  osMessageQueueId_t telemetry_queue)
{
  supervisor_event_queue = event_queue;
  supervisor_rpc_result_queue = rpc_result_queue;
  supervisor_telemetry_queue = telemetry_queue;
  tripguard_system_state = SYSTEM_SELF_TEST;
  tripguard_activation_source = ACTIVATION_NONE;
  tripguard_state_started_ms = HAL_GetTick();
  tripguard_supervisor_rear_confirmed = 0U;
  tripguard_station_detected = 0U;
  tripguard_supervisor_gps_fix_valid = 0U;
  tripguard_rear_confirm_wait_remaining_s = 0U;
  tripguard_rear_confirm_timeout_count = 0U;
  gateway_activity_seen = 0U;
  latest_speed_kmh = 0.0f;
  latest_gps_age_ms = UINT32_MAX;
  geofence_timing = 0U;
  geofence_since_ms = 0U;
  self_test_pass_mask = 0U;
#if (TRIPGUARD_CONNECTIVITY_OWNER_PI == 0U)
  self_test_publish_baseline = TB_MQTT_GetPublishCount();
#else
  self_test_publish_baseline = 0U;
#endif
  self_test_last_retry_ms = HAL_GetTick();
  self_test_retry_count = 0U;
  last_error[0] = '\0';
  TripGuard_Supervisor_Copy(last_activation_reason,
                            sizeof(last_activation_reason), "BOOT");
  Camera_Init();
  TripGuard_Supervisor_SetDetection(0U);
  TripGuard_BusPower_Init(HAL_GetTick());
  if ((event_queue == NULL) || (rpc_result_queue == NULL))
  {
    tripguard_system_state = SYSTEM_FAULT;
    TripGuard_Supervisor_Copy(last_error, sizeof(last_error),
                              "SUPERVISOR_QUEUE_CREATE_FAILED");
    return 0U;
  }
  /* First normal telemetry publish is also the non-blocking MQTT self-test. */
  TripGuard_Supervisor_QueueTelemetry();
  return 1U;
}

static uint8_t TripGuard_Supervisor_Post(
    const TripGuard_SupervisorEvent_t *event)
{
  if ((supervisor_event_queue == NULL) ||
      (osMessageQueuePut(supervisor_event_queue,
                         event, 0U, 0U) != osOK))
  {
    tripguard_supervisor_event_drop_count++;
    return 0U;
  }
  return 1U;
}

uint8_t TripGuard_Supervisor_PostRearConfirm(uint8_t node_id,
                                             uint16_t sequence)
{
  TripGuard_SupervisorEvent_t event;
  memset(&event, 0, sizeof(event));
  event.type = TRIPGUARD_SUP_EVT_REAR_CONFIRM;
  event.data.rear.node_id = node_id;
  event.data.rear.sequence = sequence;
  return TripGuard_Supervisor_Post(&event);
}

uint8_t TripGuard_Supervisor_PostGps(float latitude, float longitude,
                                     float speed_kmh, uint8_t fix_valid,
                                     uint32_t age_ms)
{
  TripGuard_SupervisorEvent_t event;
  memset(&event, 0, sizeof(event));
  event.type = TRIPGUARD_SUP_EVT_GPS_SAMPLE;
  event.data.gps.latitude = latitude;
  event.data.gps.longitude = longitude;
  event.data.gps.speed_kmh = speed_kmh;
  event.data.gps.fix_valid = fix_valid;
  event.data.gps.age_ms = age_ms;
  return TripGuard_Supervisor_Post(&event);
}

uint8_t TripGuard_Supervisor_PostRpcSetArmed(uint32_t request_id,
                                             uint8_t armed)
{
  TripGuard_SupervisorEvent_t event;
  memset(&event, 0, sizeof(event));
  event.type = TRIPGUARD_SUP_EVT_RPC_SET_ARMED;
  event.request_id = request_id;
  event.data.armed = armed;
  return TripGuard_Supervisor_Post(&event);
}

uint8_t TripGuard_Supervisor_PostRpcCancelArm(uint32_t request_id)
{
  TripGuard_SupervisorEvent_t event;
  memset(&event, 0, sizeof(event));
  event.type = TRIPGUARD_SUP_EVT_RPC_CANCEL_ARM;
  event.request_id = request_id;
  return TripGuard_Supervisor_Post(&event);
}

uint8_t TripGuard_Supervisor_PostRpcGetState(uint32_t request_id)
{
  TripGuard_SupervisorEvent_t event;
  memset(&event, 0, sizeof(event));
  event.type = TRIPGUARD_SUP_EVT_RPC_GET_STATE;
  event.request_id = request_id;
  return TripGuard_Supervisor_Post(&event);
}

uint8_t TripGuard_Supervisor_PostPiScanComplete(uint32_t session_id,
                                                uint8_t result)
{
  TripGuard_SupervisorEvent_t event;
  memset(&event, 0, sizeof(event));
  event.type = TRIPGUARD_SUP_EVT_PI_SCAN_COMPLETE;
  event.data.scan.session_id = session_id;
  event.data.scan.result = result;
  return TripGuard_Supervisor_Post(&event);
}

uint8_t TripGuard_Supervisor_PostOccupancyResolved(void)
{
  TripGuard_SupervisorEvent_t event;
  memset(&event, 0, sizeof(event));
  event.type = TRIPGUARD_SUP_EVT_OCCUPANCY_RESOLVED;
  return TripGuard_Supervisor_Post(&event);
}

uint8_t TripGuard_Supervisor_QueueRpcError(uint32_t request_id,
                                           const char *message)
{
  uint32_t before = tripguard_supervisor_rpc_result_drop_count;
  TripGuard_Supervisor_PublishRpcResult(request_id, 0U, message);
  return (tripguard_supervisor_rpc_result_drop_count == before) ? 1U : 0U;
}

uint8_t TripGuard_Supervisor_TakeRpcResult(
    TripGuard_SupervisorRpcResult_t *result)
{
  if ((result == NULL) || (supervisor_rpc_result_queue == NULL))
  {
    return 0U;
  }
  return (osMessageQueueGet(supervisor_rpc_result_queue,
                            result, NULL, 0U) == osOK) ? 1U : 0U;
}

void TripGuard_Supervisor_NotifyGatewayActivity(void)
{
  gateway_last_activity_ms = HAL_GetTick();
  gateway_activity_seen = 1U;
}

void TripGuard_Supervisor_Task(void *argument)
{
  TripGuard_SupervisorEvent_t event;
  uint32_t now_ms;

  (void)argument;
  for (;;)
  {
    now_ms = HAL_GetTick();
    /* Power remains diagnostic here; it is never an arming authority. */
    (void)TripGuard_BusPower_Poll(now_ms);

    while ((supervisor_event_queue != NULL) &&
           (osMessageQueueGet(supervisor_event_queue,
                              &event, NULL, 0U) == osOK))
    {
      TripGuard_Supervisor_HandleEvent(&event, now_ms);
    }

    TripGuard_Supervisor_SelfTestStep(now_ms);

    if ((TripGuard_BusPower_IsQualifiedHigh() != 0U) &&
        (tripguard_supervisor_gps_fix_valid != 0U) &&
        (latest_gps_age_ms <= TRIPGUARD_GPS_MAX_AGE_MS) &&
        (latest_speed_kmh > TRIPGUARD_MAX_ARRIVAL_SPEED_KMH) &&
         ((tripguard_system_state == SYSTEM_WAIT_REAR_CONFIRM) ||
          (tripguard_system_state == SYSTEM_ARM_DELAY) ||
         (tripguard_system_state == SYSTEM_ACTIVE)))
    {
      TripGuard_Supervisor_EnterStandby(now_ms,
                                        PI_SCAN_STOP_REASON_STANDBY);
    }

    if ((tripguard_system_state == SYSTEM_WAIT_REAR_CONFIRM) &&
        (TripGuard_Supervisor_TimeReached(
             now_ms, tripguard_state_started_ms,
             TRIPGUARD_REAR_CONFIRM_TIMEOUT_MS) != 0U))
    {
      tripguard_rear_confirm_timeout_count++;
      tripguard_rear_confirm_wait_remaining_s = 0U;
      TripGuard_Supervisor_StartArm(now_ms,
                                    ACTIVATION_AUTO_REAR_TIMEOUT);
    }

    if ((tripguard_system_state == SYSTEM_ARM_DELAY) &&
        (TripGuard_Supervisor_TimeReached(now_ms, tripguard_state_started_ms,
                                         TRIPGUARD_ARM_DELAY_MS) != 0U))
    {
      tripguard_system_state = SYSTEM_ACTIVE;
      tripguard_state_started_ms = now_ms;
      TripGuard_Supervisor_SetDetection(1U);
      TripGuard_Supervisor_QueueTelemetry();
    }

    osDelay(TG_SUPERVISOR_PERIOD_MS);
  }
}

void TripGuard_Supervisor_GetSnapshot(TripGuard_SupervisorSnapshot_t *snapshot)
{
  uint32_t now_ms;
  uint32_t elapsed_ms;

  if (snapshot == NULL)
  {
    return;
  }

  now_ms = HAL_GetTick();
  taskENTER_CRITICAL();
  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->state = tripguard_system_state;
  snapshot->activation_source = tripguard_activation_source;
  snapshot->system_armed = ((tripguard_system_state == SYSTEM_ARM_DELAY) ||
                            (tripguard_system_state == SYSTEM_ACTIVE)) ? 1U : 0U;
  if (tripguard_system_state == SYSTEM_ARM_DELAY)
  {
    elapsed_ms = (uint32_t)(now_ms - tripguard_state_started_ms);
    snapshot->activation_countdown_s =
        (elapsed_ms < TRIPGUARD_ARM_DELAY_MS) ?
        ((TRIPGUARD_ARM_DELAY_MS - elapsed_ms + 999U) / 1000U) : 0U;
  }
  else if (tripguard_system_state == SYSTEM_WAIT_REAR_CONFIRM)
  {
    elapsed_ms = (uint32_t)(now_ms - tripguard_state_started_ms);
    snapshot->rear_confirm_wait_remaining_s =
        (elapsed_ms < TRIPGUARD_REAR_CONFIRM_TIMEOUT_MS) ?
        ((TRIPGUARD_REAR_CONFIRM_TIMEOUT_MS - elapsed_ms + 999U) / 1000U) :
        0U;
    tripguard_rear_confirm_wait_remaining_s =
        snapshot->rear_confirm_wait_remaining_s;
  }
  snapshot->rear_confirmed = tripguard_supervisor_rear_confirmed;
  snapshot->power_present = TripGuard_BusPower_IsPresent();
  snapshot->station_detected = tripguard_station_detected;
  snapshot->gps_fix_valid = tripguard_supervisor_gps_fix_valid;
  snapshot->esp_gateway_online =
      ((gateway_activity_seen != 0U) &&
       ((uint32_t)(now_ms - gateway_last_activity_ms) <=
        TG_GATEWAY_ONLINE_TIMEOUT_MS)) ? 1U : 0U;
  snapshot->radar_enabled = tripguard_supervisor_radar_enabled;
  snapshot->camera_enabled = tripguard_supervisor_camera_enabled;
  snapshot->self_test_pass_mask = self_test_pass_mask;
  snapshot->self_test_complete =
      ((self_test_pass_mask & TRIPGUARD_SELF_TEST_REQUIRED_MASK) ==
       TRIPGUARD_SELF_TEST_REQUIRED_MASK) ? 1U : 0U;
  snapshot->self_test_retry_count = self_test_retry_count;
  memcpy(snapshot->last_activation_reason, last_activation_reason,
         sizeof(snapshot->last_activation_reason));
  snapshot->last_activation_reason[
      sizeof(snapshot->last_activation_reason) - 1U] = '\0';
  memcpy(snapshot->last_error, last_error, sizeof(snapshot->last_error));
  snapshot->last_error[sizeof(snapshot->last_error) - 1U] = '\0';
  taskEXIT_CRITICAL();
}

uint8_t TripGuard_Supervisor_IsRadarEnabled(void)
{
  return tripguard_supervisor_radar_enabled;
}

uint8_t TripGuard_Supervisor_IsCameraEnabled(void)
{
  return tripguard_supervisor_camera_enabled;
}

uint8_t TripGuard_Supervisor_IsDetectionActive(void)
{
  return (tripguard_system_state == SYSTEM_ACTIVE) ? 1U : 0U;
}
