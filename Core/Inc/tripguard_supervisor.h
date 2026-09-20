#ifndef INC_TRIPGUARD_SUPERVISOR_H_
#define INC_TRIPGUARD_SUPERVISOR_H_

#include "cmsis_os2.h"
#include "tripguard_config.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  SYSTEM_SELF_TEST = 0U,
  SYSTEM_STANDBY,
  SYSTEM_ARM_DELAY,
  SYSTEM_ACTIVE,
  SYSTEM_FAULT,
  SYSTEM_WAIT_REAR_CONFIRM
} TripGuard_SystemState_t;

typedef enum
{
  ACTIVATION_NONE = 0U,
  ACTIVATION_MANUAL_REAR_CONFIRM,
  ACTIVATION_AUTO_REAR_TIMEOUT
} TripGuard_ActivationSource_t;

#define TRIPGUARD_SELF_TEST_SIM_OK           (1U << 0)
#define TRIPGUARD_SELF_TEST_MQTT_PUBLISH_OK  (1U << 1)
#define TRIPGUARD_SELF_TEST_RADAR_HEALTH_OK  (1U << 2)
#define TRIPGUARD_SELF_TEST_PI_READY_OK      (1U << 3)
#define TRIPGUARD_SELF_TEST_ALL_OK           \
    (TRIPGUARD_SELF_TEST_SIM_OK | \
     TRIPGUARD_SELF_TEST_MQTT_PUBLISH_OK | \
     TRIPGUARD_SELF_TEST_RADAR_HEALTH_OK | \
     TRIPGUARD_SELF_TEST_PI_READY_OK)

#if (TRIPGUARD_CONNECTIVITY_OWNER_PI != 0U)
/* Pi/camera/radar may be intentionally asleep at boot. */
#define TRIPGUARD_SELF_TEST_REQUIRED_MASK    0U
#else
#define TRIPGUARD_SELF_TEST_REQUIRED_MASK TRIPGUARD_SELF_TEST_ALL_OK
#endif

typedef enum
{
  TRIPGUARD_SUP_EVT_REAR_CONFIRM = 0U,
  TRIPGUARD_SUP_EVT_GPS_SAMPLE,
  TRIPGUARD_SUP_EVT_RPC_SET_ARMED,
  TRIPGUARD_SUP_EVT_RPC_CANCEL_ARM,
  TRIPGUARD_SUP_EVT_RPC_GET_STATE,
  TRIPGUARD_SUP_EVT_PI_SCAN_COMPLETE,
  TRIPGUARD_SUP_EVT_OCCUPANCY_RESOLVED
} TripGuard_SupervisorEventType_t;

typedef struct
{
  TripGuard_SupervisorEventType_t type;
  uint32_t request_id;
  union
  {
    struct
    {
      uint8_t node_id;
      uint16_t sequence;
    } rear;
    struct
    {
      float latitude;
      float longitude;
      float speed_kmh;
      uint32_t age_ms;
      uint8_t fix_valid;
    } gps;
    struct
    {
      uint32_t session_id;
      uint8_t result;
    } scan;
    uint8_t armed;
  } data;
} TripGuard_SupervisorEvent_t;

#define TRIPGUARD_RPC_RESPONSE_JSON_SIZE 192U

typedef struct
{
  uint32_t request_id;
  uint8_t success;
  char json[TRIPGUARD_RPC_RESPONSE_JSON_SIZE];
} TripGuard_SupervisorRpcResult_t;

typedef struct
{
  TripGuard_SystemState_t state;
  TripGuard_ActivationSource_t activation_source;
  uint32_t activation_countdown_s;
  uint32_t rear_confirm_wait_remaining_s;
  uint8_t system_armed;
  uint8_t rear_confirmed;
  uint8_t power_present;
  uint8_t station_detected;
  uint8_t gps_fix_valid;
  uint8_t esp_gateway_online;
  uint8_t radar_enabled;
  uint8_t camera_enabled;
  uint8_t self_test_pass_mask;
  uint8_t self_test_complete;
  uint32_t self_test_retry_count;
  char last_activation_reason[32];
  char last_error[64];
} TripGuard_SupervisorSnapshot_t;

uint8_t TripGuard_Supervisor_Init(osMessageQueueId_t event_queue,
                                  osMessageQueueId_t rpc_result_queue,
                                  osMessageQueueId_t telemetry_queue);
void TripGuard_Supervisor_Task(void *argument);

uint8_t TripGuard_Supervisor_PostRearConfirm(uint8_t node_id,
                                             uint16_t sequence);
uint8_t TripGuard_Supervisor_PostGps(float latitude, float longitude,
                                     float speed_kmh, uint8_t fix_valid,
                                     uint32_t age_ms);
uint8_t TripGuard_Supervisor_PostRpcSetArmed(uint32_t request_id,
                                             uint8_t armed);
uint8_t TripGuard_Supervisor_PostRpcCancelArm(uint32_t request_id);
uint8_t TripGuard_Supervisor_PostRpcGetState(uint32_t request_id);
uint8_t TripGuard_Supervisor_PostPiScanComplete(uint32_t session_id,
                                                uint8_t result);
uint8_t TripGuard_Supervisor_PostOccupancyResolved(void);
uint8_t TripGuard_Supervisor_QueueRpcError(uint32_t request_id,
                                           const char *message);
uint8_t TripGuard_Supervisor_TakeRpcResult(
    TripGuard_SupervisorRpcResult_t *result);
void TripGuard_Supervisor_NotifyGatewayActivity(void);
void TripGuard_Supervisor_GetSnapshot(TripGuard_SupervisorSnapshot_t *snapshot);
uint8_t TripGuard_Supervisor_IsRadarEnabled(void);
uint8_t TripGuard_Supervisor_IsCameraEnabled(void);
uint8_t TripGuard_Supervisor_IsDetectionActive(void);

const char *TripGuard_Supervisor_StateName(TripGuard_SystemState_t state);
const char *TripGuard_Supervisor_SourceName(TripGuard_ActivationSource_t source);

extern volatile uint32_t tripguard_supervisor_event_drop_count;
extern volatile uint32_t tripguard_supervisor_rpc_result_drop_count;
extern volatile TripGuard_SystemState_t tripguard_system_state;
extern volatile TripGuard_ActivationSource_t tripguard_activation_source;
extern volatile uint32_t tripguard_state_started_ms;
extern volatile uint8_t tripguard_supervisor_rear_confirmed;
extern volatile uint8_t tripguard_station_detected;
extern volatile uint8_t tripguard_supervisor_gps_fix_valid;
extern volatile uint8_t tripguard_supervisor_radar_enabled;
extern volatile uint8_t tripguard_supervisor_camera_enabled;
extern volatile uint32_t tripguard_rear_confirm_wait_remaining_s;
extern volatile uint32_t tripguard_rear_confirm_timeout_count;

#ifdef __cplusplus
}
#endif

#endif /* INC_TRIPGUARD_SUPERVISOR_H_ */
