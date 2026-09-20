#ifndef INC_TRIPGUARD_SHUTDOWN_BUTTON_H_
#define INC_TRIPGUARD_SHUTDOWN_BUTTON_H_

#include <stdbool.h>
#include <stdint.h>

void TripGuard_ShutdownButton_Init(uint32_t now_ms);
void TripGuard_ShutdownButton_NotifyEdgeFromIsr(void);
bool TripGuard_ShutdownButton_Poll(uint32_t now_ms);

extern volatile uint8_t tripguard_shutdown_button_raw;
extern volatile uint8_t tripguard_shutdown_button_armed;
extern volatile uint32_t tripguard_shutdown_button_edge_count;
extern volatile uint32_t tripguard_shutdown_button_trigger_count;

#endif /* INC_TRIPGUARD_SHUTDOWN_BUTTON_H_ */
