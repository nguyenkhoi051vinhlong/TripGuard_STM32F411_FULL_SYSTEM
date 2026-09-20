#ifndef INC_TRIPGUARD_BUS_POWER_H_
#define INC_TRIPGUARD_BUS_POWER_H_

#include <stdint.h>

typedef enum
{
  TG_BUS_POWER_NO_CHANGE = 0,
  TG_BUS_POWER_LOST,
  TG_BUS_POWER_RESTORED
} tg_bus_power_event_t;

void TripGuard_BusPower_Init(uint32_t now_ms);
void TripGuard_BusPower_NotifyEdgeFromIsr(void);
tg_bus_power_event_t TripGuard_BusPower_Poll(uint32_t now_ms);
uint8_t TripGuard_BusPower_IsPresent(void);
uint8_t TripGuard_BusPower_IsQualifiedHigh(void);

extern volatile uint8_t tripguard_bus_power_present;
extern volatile uint32_t tripguard_bus_power_edge_count;
extern volatile uint32_t tripguard_bus_power_lost_count;
extern volatile uint32_t tripguard_bus_power_restore_count;
extern volatile uint8_t tripguard_bus_power_high_qualified;

#endif /* INC_TRIPGUARD_BUS_POWER_H_ */
