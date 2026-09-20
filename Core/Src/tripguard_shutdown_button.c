#include "tripguard_shutdown_button.h"

#include "main.h"

#define TG_SHUTDOWN_DEBOUNCE_MS  40U
#define TG_SHUTDOWN_HOLD_MS    2000U
#define TG_SHUTDOWN_RELEASE_MS  150U

static uint8_t button_raw_pressed;
static uint8_t button_stable_pressed;
static uint32_t button_raw_changed_ms;
static uint32_t button_press_started_ms;
static uint32_t button_release_started_ms;

volatile uint8_t tripguard_shutdown_button_raw;
volatile uint8_t tripguard_shutdown_button_armed;
volatile uint32_t tripguard_shutdown_button_edge_count;
volatile uint32_t tripguard_shutdown_button_trigger_count;

static uint8_t TripGuard_ShutdownButton_Read(void)
{
  return (HAL_GPIO_ReadPin(SHUTDOWN_BUTTON_GPIO_Port,
                          SHUTDOWN_BUTTON_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

void TripGuard_ShutdownButton_Init(uint32_t now_ms)
{
  button_raw_pressed = TripGuard_ShutdownButton_Read();
  button_stable_pressed = button_raw_pressed;
  button_raw_changed_ms = now_ms;
  button_press_started_ms = now_ms;
  button_release_started_ms = now_ms;
  tripguard_shutdown_button_raw = button_raw_pressed;

  /* A button held while the MCU boots must first be released. */
  tripguard_shutdown_button_armed =
      (button_stable_pressed == 0U) ? 1U : 0U;
}

void TripGuard_ShutdownButton_NotifyEdgeFromIsr(void)
{
  tripguard_shutdown_button_edge_count++;
}

bool TripGuard_ShutdownButton_Poll(uint32_t now_ms)
{
  uint8_t pressed = TripGuard_ShutdownButton_Read();

  tripguard_shutdown_button_raw = pressed;
  if (pressed != button_raw_pressed)
  {
    button_raw_pressed = pressed;
    button_raw_changed_ms = now_ms;
  }

  if ((pressed != button_stable_pressed) &&
      ((uint32_t)(now_ms - button_raw_changed_ms) >=
       TG_SHUTDOWN_DEBOUNCE_MS))
  {
    button_stable_pressed = pressed;
    if (pressed != 0U)
    {
      button_press_started_ms = now_ms;
      button_release_started_ms = 0U;
    }
    else
    {
      button_release_started_ms = now_ms;
    }
  }

  if ((button_stable_pressed != 0U) &&
      (tripguard_shutdown_button_armed != 0U) &&
      ((uint32_t)(now_ms - button_press_started_ms) >=
       TG_SHUTDOWN_HOLD_MS))
  {
    tripguard_shutdown_button_armed = 0U;
    tripguard_shutdown_button_trigger_count++;
    return true;
  }

  if ((button_stable_pressed == 0U) &&
      (tripguard_shutdown_button_armed == 0U) &&
      (button_release_started_ms != 0U) &&
      ((uint32_t)(now_ms - button_release_started_ms) >=
       TG_SHUTDOWN_RELEASE_MS))
  {
    tripguard_shutdown_button_armed = 1U;
  }

  return false;
}
