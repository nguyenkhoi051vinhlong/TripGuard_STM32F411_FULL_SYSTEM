/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gps.h"
#include "a7670.h"
#include "tb_mqtt.h"
#include "jq8900.h"
#include "pi_link.h"
#include "tripguard_rtos.h"
#include "tripguard_spi.h"
#include "tripguard_power.h"
#include "tripguard_config.h"

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Them truc tiep cac bien tripguard_* nay vao Live Expressions. */
volatile uint32_t tripguard_uptime_ms;
volatile A7670_State_t tripguard_modem_state;
volatile uint8_t tripguard_modem_ready;
volatile uint8_t tripguard_network_registered;
volatile int16_t tripguard_cereg_status;
volatile uint8_t tripguard_pdp_active;
volatile A7670_CommandResult_t tripguard_last_at_result;
volatile uint32_t tripguard_at_timeout_count;
volatile uint32_t tripguard_modem_uart_error_count;
volatile uint32_t tripguard_modem_rx_overflow_count;
volatile uint32_t tripguard_modem_uart_last_error_code;
volatile uint32_t tripguard_modem_uart_parity_error_count;
volatile uint32_t tripguard_modem_uart_noise_error_count;
volatile uint32_t tripguard_modem_uart_frame_error_count;
volatile uint32_t tripguard_modem_uart_overrun_error_count;
volatile uint32_t tripguard_modem_dma_error_count;
volatile uint32_t tripguard_modem_dma_restart_count;
volatile uint32_t tripguard_modem_dma_restart_failure_count;
volatile MQTT_State_t tripguard_mqtt_state;
volatile uint8_t tripguard_mqtt_phase;
volatile uint8_t tripguard_mqtt_connected;
volatile uint8_t tripguard_mqtt_last_result;
volatile MQTT_State_t tripguard_mqtt_last_error_state;
volatile uint8_t tripguard_last_publish_queued;
volatile uint32_t tripguard_publish_count;
volatile uint32_t tripguard_mqtt_error_count;
char tripguard_ip_address[48];
char tripguard_last_at_command[192];
char tripguard_last_at_response[768];
char tripguard_mqtt_last_response[384];
char tripguard_mqtt_last_error[384];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
extern osEventFlagsId_t systemEventsHandle;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void TripGuard_CopyText(char *destination,
                               size_t destination_size,
                               const char *source)
{
  if ((destination == NULL) || (destination_size == 0U))
  {
    return;
  }

  if (source == NULL)
  {
    destination[0] = '\0';
    return;
  }

  strncpy(destination, source, destination_size - 1U);
  destination[destination_size - 1U] = '\0';
}

void TripGuard_UpdateDiagnostics(void)
{
  tripguard_uptime_ms = HAL_GetTick();
  tripguard_modem_state = A7670_GetState();
  tripguard_modem_ready = A7670_IsNetworkReady();
  tripguard_network_registered = A7670_IsRegistered();
  tripguard_cereg_status = A7670_GetRegistrationStatus();
  tripguard_pdp_active = A7670_IsPdpActive();
  tripguard_last_at_result = A7670_GetLastCommandResult();
  tripguard_at_timeout_count = A7670_GetCommandTimeoutCount();
  tripguard_modem_uart_error_count = A7670_GetUartErrorCount();
  tripguard_modem_rx_overflow_count = A7670_GetRxOverflowCount();
  tripguard_modem_uart_last_error_code = A7670_GetLastUartErrorCode();
  tripguard_modem_uart_parity_error_count = A7670_GetUartParityErrorCount();
  tripguard_modem_uart_noise_error_count = A7670_GetUartNoiseErrorCount();
  tripguard_modem_uart_frame_error_count = A7670_GetUartFrameErrorCount();
  tripguard_modem_uart_overrun_error_count = A7670_GetUartOverrunErrorCount();
  tripguard_modem_dma_error_count = A7670_GetDmaErrorCount();
  tripguard_modem_dma_restart_count = A7670_GetDmaRestartCount();
  tripguard_modem_dma_restart_failure_count =
      A7670_GetDmaRestartFailureCount();

  tripguard_mqtt_state = TB_MQTT_GetState();
  tripguard_mqtt_phase = TB_MQTT_GetPhase();
  tripguard_mqtt_connected = TB_MQTT_IsConnected();
  tripguard_mqtt_last_result = TB_MQTT_GetLastCommandResult();
  tripguard_mqtt_last_error_state = TB_MQTT_GetLastErrorState();
  tripguard_publish_count = TB_MQTT_GetPublishCount();
  tripguard_mqtt_error_count = TB_MQTT_GetErrorCount();

  TripGuard_CopyText(tripguard_ip_address,
                     sizeof(tripguard_ip_address),
                     A7670_GetIpAddress());
  TripGuard_CopyText(tripguard_last_at_command,
                     sizeof(tripguard_last_at_command),
                     A7670_GetLastCommand());
  TripGuard_CopyText(tripguard_last_at_response,
                     sizeof(tripguard_last_at_response),
                     A7670_GetLastResponse());
  TripGuard_CopyText(tripguard_mqtt_last_response,
                     sizeof(tripguard_mqtt_last_response),
                     TB_MQTT_GetLastResponse());
  TripGuard_CopyText(tripguard_mqtt_last_error,
                     sizeof(tripguard_mqtt_last_error),
                     TB_MQTT_GetLastErrorResponse());
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
#if (TRIPGUARD_CONNECTIVITY_OWNER_PI == 0U)
  MX_USART2_UART_Init();
#endif
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USART1: NEO-M8L, RX interrupt tung byte. */
  GPS_Init(&huart1);

  /* USART2 is left untouched when the Pi owns the A7670/SIM UART. */
#if (TRIPGUARD_CONNECTIVITY_OWNER_PI == 0U)
  A7670_Init(&huart2);
#endif

  /* USART6 is owned by PiLinkTask: Raspberry Pi full-duplex UART, 115200 8N1. */
  /* JQ8900: mot am thanh, IO1 active-low tren PB0. */
  JQ8900_Init(JQ8900_IO1_GPIO_Port, JQ8900_IO1_Pin);
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

#if (TRIPGUARD_CONNECTIVITY_OWNER_PI == 0U)
  TB_MQTT_Init();
#endif
  TripGuard_UpdateDiagnostics();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Khong den day khi RTOS scheduler dang hoat dong. */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    A7670_RxEventCallback(huart, Size);
  }
  else if ((huart != NULL) && (huart->Instance == USART6))
  {
    PiLink_RxEventFromISR(huart, Size);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    GPS_RxCallback(huart);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART6))
  {
    PiLink_TxCompleteFromISR(huart);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART1)
  {
    GPS_ErrorCallback(huart);
  }
  else if (huart->Instance == USART2)
  {
    A7670_ErrorCallback(huart);
  }
  else if (huart->Instance == USART6)
  {
    PiLink_ErrorFromISR(huart);
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if ((hspi != NULL) && (hspi->Instance == SPI2))
  {
    TripGuard_Spi_TxRxCompleteFromIsr();
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if ((hspi != NULL) && (hspi->Instance == SPI2))
  {
    TripGuard_Spi_TxRxCompleteFromIsr();
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if ((hspi != NULL) && (hspi->Instance == SPI2))
  {
    TripGuard_Spi_ErrorFromIsr();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t event_flags = 0U;

  TripGuard_Power_NotifyGpioFromIsr(GPIO_Pin);

  if (GPIO_Pin == SOS_BTN_Pin)
  {
    event_flags = TRIPGUARD_EVENT_SOS_PRESSED;
  }
  else if (GPIO_Pin == DRIVER_ACK_BTN_Pin)
  {
    event_flags = TRIPGUARD_EVENT_ACK_PRESSED;
  }

  if ((event_flags != 0U) && (systemEventsHandle != NULL))
  {
    (void)osEventFlagsSet(systemEventsHandle, event_flags);
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
