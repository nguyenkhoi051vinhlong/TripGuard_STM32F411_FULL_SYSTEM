#ifndef INC_TRIPGUARD_RTOS_H_
#define INC_TRIPGUARD_RTOS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Event flags written by GPIO interrupt callbacks and consumed by SafetyTask. */
#define TRIPGUARD_EVENT_RADAR_CHANGED   (1UL << 0)
#define TRIPGUARD_EVENT_SOS_PRESSED     (1UL << 1)
#define TRIPGUARD_EVENT_ACK_PRESSED     (1UL << 2)
#define TRIPGUARD_EVENT_GATEWAY_RX      (1UL << 3)
#define TRIPGUARD_EVENT_ALL_SAFETY      (TRIPGUARD_EVENT_RADAR_CHANGED | \
                                         TRIPGUARD_EVENT_SOS_PRESSED | \
                                         TRIPGUARD_EVENT_ACK_PRESSED | \
                                         TRIPGUARD_EVENT_GATEWAY_RX)

typedef enum
{
  TRIPGUARD_AUDIO_STOP = 0U,
  TRIPGUARD_AUDIO_RADAR_WARNING = 1U,
  TRIPGUARD_AUDIO_SOS_WARNING = 2U
} TripGuard_AudioCommand_t;

typedef enum
{
  TRIPGUARD_TELEMETRY_PERIODIC = 1U,
  TRIPGUARD_TELEMETRY_SAFETY_EVENT = 2U,
  TRIPGUARD_TELEMETRY_REAR_CONFIRM = 3U
} TripGuard_TelemetryEvent_t;

typedef enum
{
  TRIPGUARD_DRIVING_UNKNOWN = 0U,
  TRIPGUARD_DRIVING_STOPPED,
  TRIPGUARD_DRIVING_MOVING
} TripGuard_DrivingState_t;

void TripGuard_UpdateDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_TRIPGUARD_RTOS_H_ */
