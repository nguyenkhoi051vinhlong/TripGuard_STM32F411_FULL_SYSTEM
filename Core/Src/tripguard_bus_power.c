#include "tripguard_bus_power.h"

#include "main.h"
#include "tripguard_config.h"

static uint8_t bus_raw_present;
static uint32_t bus_raw_changed_ms;

volatile uint8_t tripguard_bus_power_present;
volatile uint32_t tripguard_bus_power_edge_count;
volatile uint32_t tripguard_bus_power_lost_count;
volatile uint32_t tripguard_bus_power_restore_count;
volatile uint8_t tripguard_bus_power_high_qualified;

static uint8_t TripGuard_BusPower_Read(void)
{
  uint8_t raw_high =
      (HAL_GPIO_ReadPin(BUS_POWER_SENSE_GPIO_Port,
                       BUS_POWER_SENSE_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  return (raw_high == TRIPGUARD_POWER_ACTIVE_LEVEL) ? 1U : 0U;
}

void TripGuard_BusPower_Init(uint32_t now_ms)
{
  bus_raw_present = TripGuard_BusPower_Read();
  /* Boot LOW is only the initial condition, never a POWER_LOSS event. */
  tripguard_bus_power_present = 0U;
  tripguard_bus_power_high_qualified = 0U;
  bus_raw_changed_ms = now_ms;
}

void TripGuard_BusPower_NotifyEdgeFromIsr(void)
{
  tripguard_bus_power_edge_count++;
}

tg_bus_power_event_t TripGuard_BusPower_Poll(uint32_t now_ms)
{
  uint8_t present = TripGuard_BusPower_Read();

  if (present != bus_raw_present)
  {
    bus_raw_present = present;
    bus_raw_changed_ms = now_ms;
  }

  if (present != 0U)
  {
    if ((tripguard_bus_power_high_qualified == 0U) &&
        ((uint32_t)(now_ms - bus_raw_changed_ms) >=
         TRIPGUARD_POWER_QUALIFY_HIGH_MS))
    {
      tripguard_bus_power_high_qualified = 1U;
      tripguard_bus_power_present = 1U;
      tripguard_bus_power_restore_count++;
      return TG_BUS_POWER_RESTORED;
    }
  }
  else if ((tripguard_bus_power_high_qualified != 0U) &&
           ((uint32_t)(now_ms - bus_raw_changed_ms) >=
            TRIPGUARD_POWER_LOSS_DEBOUNCE_MS))
  {
    tripguard_bus_power_high_qualified = 0U;
    tripguard_bus_power_present = 0U;
    tripguard_bus_power_lost_count++;
    return TG_BUS_POWER_LOST;
  }

  return TG_BUS_POWER_NO_CHANGE;
}

uint8_t TripGuard_BusPower_IsPresent(void)
{
  return tripguard_bus_power_present;
}

uint8_t TripGuard_BusPower_IsQualifiedHigh(void)
{
  return tripguard_bus_power_high_qualified;
}
