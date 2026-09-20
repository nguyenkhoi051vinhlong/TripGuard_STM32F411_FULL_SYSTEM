#include "tripguard_sleep.h"

#include "main.h"

extern void SystemClock_Config(void);

#define TG_RTC_TIMEOUT_LOOPS       2000000UL
#define TG_LSE_STARTUP_TIMEOUT_MS     2000UL

static volatile uint8_t rtc_wake_pending;

volatile uint32_t tripguard_sleep_rtc_init_error_count;
volatile uint32_t tripguard_sleep_clock_restore_error_count;

static void TripGuard_Rtc_Unlock(void)
{
  RTC->WPR = 0xCAU;
  RTC->WPR = 0x53U;
}

static void TripGuard_Rtc_Lock(void)
{
  RTC->WPR = 0xFFU;
}

bool TripGuard_Sleep_Init(void)
{
  uint32_t timeout = TG_RTC_TIMEOUT_LOOPS;
  uint32_t started_ms;

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  if ((RCC->BDCR & RCC_BDCR_LSERDY) == 0U)
  {
    RCC->BDCR |= RCC_BDCR_LSEON;
    started_ms = HAL_GetTick();
    while (((RCC->BDCR & RCC_BDCR_LSERDY) == 0U) &&
           ((uint32_t)(HAL_GetTick() - started_ms) <
            TG_LSE_STARTUP_TIMEOUT_MS))
    {
    }
    if ((RCC->BDCR & RCC_BDCR_LSERDY) == 0U)
    {
      tripguard_sleep_rtc_init_error_count++;
      return false;
    }
  }

  if ((RCC->BDCR & RCC_BDCR_RTCSEL) != RCC_BDCR_RTCSEL_0)
  {
    RCC->BDCR &= ~RCC_BDCR_RTCEN;
    RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCSEL_0;
  }
  RCC->BDCR |= RCC_BDCR_RTCEN;

  TripGuard_Rtc_Unlock();
  if ((RTC->ISR & RTC_ISR_INITS) == 0U)
  {
    timeout = TG_RTC_TIMEOUT_LOOPS;
    RTC->ISR |= RTC_ISR_INIT;
    while (((RTC->ISR & RTC_ISR_INITF) == 0U) && (timeout > 0U))
    {
      timeout--;
    }
    if (timeout == 0U)
    {
      TripGuard_Rtc_Lock();
      tripguard_sleep_rtc_init_error_count++;
      return false;
    }

    /* 32.768 kHz LSE / (127 + 1) / (255 + 1) = 1 Hz ck_spre. */
    RTC->PRER = (127UL << RTC_PRER_PREDIV_A_Pos) | 255UL;
    RTC->TR = 0U;
    RTC->DR = 0x00002101UL;
    RTC->ISR &= ~RTC_ISR_INIT;
  }
  TripGuard_Rtc_Lock();

  EXTI->IMR |= EXTI_IMR_MR22;
  EXTI->RTSR |= EXTI_RTSR_TR22;
  NVIC_SetPriority(RTC_WKUP_IRQn, 5U);
  NVIC_EnableIRQ(RTC_WKUP_IRQn);
  return true;
}

static bool TripGuard_Rtc_SetWakeTimer(uint32_t seconds)
{
  uint32_t timeout = TG_RTC_TIMEOUT_LOOPS;

  if (seconds == 0U)
  {
    seconds = 1U;
  }
  if (seconds > 65536U)
  {
    seconds = 65536U;
  }

  TripGuard_Rtc_Unlock();
  RTC->CR &= ~(RTC_CR_WUTE | RTC_CR_WUTIE);
  while (((RTC->ISR & RTC_ISR_WUTWF) == 0U) && (timeout > 0U))
  {
    timeout--;
  }
  if (timeout == 0U)
  {
    TripGuard_Rtc_Lock();
    tripguard_sleep_rtc_init_error_count++;
    return false;
  }

  RTC->WUTR = seconds - 1U;
  RTC->ISR &= ~RTC_ISR_WUTF;
  RTC->CR = (RTC->CR & ~RTC_CR_WUCKSEL) | RTC_CR_WUCKSEL_2;
  RTC->CR |= RTC_CR_WUTIE | RTC_CR_WUTE;
  TripGuard_Rtc_Lock();

  EXTI->PR = EXTI_PR_PR22;
  rtc_wake_pending = 0U;
  return true;
}

bool TripGuard_Sleep_EnterStop(uint32_t wake_after_seconds)
{
  uint32_t systick_control;

  if (!TripGuard_Rtc_SetWakeTimer(wake_after_seconds))
  {
    return false;
  }

  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  systick_control = SysTick->CTRL;
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
  HAL_SuspendTick();
  __DSB();
  __ISB();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

  /* STOP exits on HSI. Restore the 96 MHz HSE/PLL tree before any owner
     resumes a baud-rate or SPI dependent peripheral. */
  SystemClock_Config();
  HAL_ResumeTick();
  SysTick->CTRL = systick_control;
  return true;
}

void TripGuard_Sleep_RtcWakeFromIsr(void)
{
  TripGuard_Rtc_Unlock();
  RTC->ISR &= ~RTC_ISR_WUTF;
  TripGuard_Rtc_Lock();
  EXTI->PR = EXTI_PR_PR22;
  rtc_wake_pending = 1U;
}

uint8_t TripGuard_Sleep_TakeRtcWake(void)
{
  uint8_t result = rtc_wake_pending;
  rtc_wake_pending = 0U;
  return result;
}
