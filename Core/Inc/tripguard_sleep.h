#ifndef INC_TRIPGUARD_SLEEP_H_
#define INC_TRIPGUARD_SLEEP_H_

#include <stdbool.h>
#include <stdint.h>

bool TripGuard_Sleep_Init(void);
bool TripGuard_Sleep_EnterStop(uint32_t wake_after_seconds);
void TripGuard_Sleep_RtcWakeFromIsr(void);
uint8_t TripGuard_Sleep_TakeRtcWake(void);

extern volatile uint32_t tripguard_sleep_rtc_init_error_count;
extern volatile uint32_t tripguard_sleep_clock_restore_error_count;

#endif /* INC_TRIPGUARD_SLEEP_H_ */
