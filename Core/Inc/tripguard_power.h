#ifndef INC_TRIPGUARD_POWER_H_
#define INC_TRIPGUARD_POWER_H_

#include "cmsis_os.h"

#include <stdint.h>

typedef enum
{
  TG_POWER_BOOT = 0,
  TG_POWER_ACTIVE,
  TG_POWER_IDLE,
  TG_POWER_PREPARE_SLEEP,
  TG_POWER_SLEEP,
  TG_POWER_WAKE,
  TG_POWER_REPORTING,
  TG_POWER_SHUTDOWN_REQUEST,
  TG_POWER_SHUTTING_DOWN,
  TG_POWER_OFF
} tg_power_state_t;

typedef enum
{
  TG_WAKE_NONE = 0,
  TG_WAKE_GPS_STATION,
  TG_WAKE_BUS_POWER_LOST,
  TG_WAKE_REMOTE_REAR_CONFIRM,
  TG_WAKE_DRIVER_BUTTON,
  TG_WAKE_TIMER,
  TG_WAKE_OTHER
} tg_wake_reason_t;

void TripGuard_Power_Init(osMessageQueueId_t safety_queue,
                          osMessageQueueId_t ack_queue,
                          osMessageQueueId_t telemetry_queue,
                          osMessageQueueId_t audio_queue);
void TripGuard_Power_Task(void *argument);
void TripGuard_Power_NotifyGpioFromIsr(uint16_t gpio_pin);
void TripGuard_Power_NotifyRtcFromIsr(void);
void TripGuard_Power_NotifyRemoteCommitted(void);
void TripGuard_Power_ReportGps(float latitude,
                               float longitude,
                               float speed_kmh,
                               uint8_t position_valid);
void TripGuard_Power_RequestIdle(void);
uint8_t TripGuard_Power_IsSwitchedRailOn(void);
uint8_t TripGuard_Power_IsPiOn(void);
uint8_t TripGuard_Power_IsModemOn(void);
uint8_t TripGuard_Power_IsGpsOn(void);
uint8_t TripGuard_Power_IsAudioOn(void);

extern volatile tg_power_state_t tripguard_power_state;
extern volatile tg_wake_reason_t tripguard_wake_reason;
extern volatile uint32_t tripguard_sleep_count;
extern volatile uint32_t tripguard_wake_count;
extern volatile uint32_t tripguard_wake_gps_count;
extern volatile uint32_t tripguard_wake_power_loss_count;
extern volatile uint32_t tripguard_wake_remote_count;
extern volatile uint32_t tripguard_shutdown_request_count;
extern volatile uint32_t tripguard_shutdown_cancel_count;
extern volatile uint32_t tripguard_shutdown_success_count;
extern volatile uint32_t tripguard_shutdown_timeout_count;
extern volatile uint32_t tripguard_power_transition_count;
extern volatile uint32_t tripguard_power_sleep_blocked_count;
extern volatile uint32_t tripguard_power_hw_inhibit_count;
extern volatile uint32_t tripguard_pi_ack_stuck_count;
extern volatile uint32_t tripguard_wake_pin_stuck_count;
extern volatile uint32_t tripguard_gps_verify_timeout_count;
extern volatile uint8_t tripguard_switched_rail_on;
extern volatile uint8_t tripguard_pi_power_on;
extern volatile uint8_t tripguard_modem_power_on;
extern volatile uint8_t tripguard_gps_power_on;
extern volatile uint8_t tripguard_audio_power_on;
extern volatile uint8_t tripguard_pi_shutdown_ack;
extern volatile uint32_t tripguard_power_rail_epoch;
extern volatile uint8_t tripguard_sleep_ready;

#endif /* INC_TRIPGUARD_POWER_H_ */
