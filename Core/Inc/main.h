/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TX_SIM7680_Pin GPIO_PIN_2
#define TX_SIM7680_GPIO_Port GPIOA
#define RX_SIM7680_Pin GPIO_PIN_3
#define RX_SIM7680_GPIO_Port GPIOA
#define PI_SHUTDOWN_REQ_Pin GPIO_PIN_1
#define PI_SHUTDOWN_REQ_GPIO_Port GPIOA
#define SERVO_PWM_Pin GPIO_PIN_6
#define SERVO_PWM_GPIO_Port GPIOA
#define BUS_POWER_SENSE_Pin GPIO_PIN_8
#define BUS_POWER_SENSE_GPIO_Port GPIOA
#define BUS_POWER_SENSE_EXTI_IRQn EXTI9_5_IRQn
#define JQ8900_IO1_Pin GPIO_PIN_0
#define JQ8900_IO1_GPIO_Port GPIOB
#define SHUTDOWN_BUTTON_Pin GPIO_PIN_2
#define SHUTDOWN_BUTTON_GPIO_Port GPIOB
#define SHUTDOWN_BUTTON_EXTI_IRQn EXTI2_IRQn
#define AUDIO_POWER_EN_Pin GPIO_PIN_3
#define AUDIO_POWER_EN_GPIO_Port GPIOB
#define PI_POWER_EN_Pin GPIO_PIN_5
#define PI_POWER_EN_GPIO_Port GPIOB
#define MODEM_POWER_EN_Pin GPIO_PIN_6
#define MODEM_POWER_EN_GPIO_Port GPIOB
#define PI_SHUTDOWN_ACK_Pin GPIO_PIN_7
#define PI_SHUTDOWN_ACK_GPIO_Port GPIOB
#define PI_SHUTDOWN_ACK_EXTI_IRQn EXTI9_5_IRQn
#define STM32_READY_Pin GPIO_PIN_8
#define STM32_READY_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_9
#define SPI2_NSS_GPIO_Port GPIOB
#define SOS_BTN_Pin GPIO_PIN_1
#define SOS_BTN_GPIO_Port GPIOB
#define SOS_BTN_EXTI_IRQn EXTI1_IRQn
#define DRIVER_ACK_BTN_Pin GPIO_PIN_10
#define DRIVER_ACK_BTN_GPIO_Port GPIOB
#define DRIVER_ACK_BTN_EXTI_IRQn EXTI15_10_IRQn
#define GPS_POWER_EN_Pin GPIO_PIN_11
#define GPS_POWER_EN_GPIO_Port GPIOB
#define AUDIO_BUSY_Pin GPIO_PIN_12
#define AUDIO_BUSY_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define GPS_UART_TX_Pin GPIO_PIN_9
#define GPS_UART_TX_GPIO_Port GPIOA
#define GPS_UART_RX_Pin GPIO_PIN_10
#define GPS_UART_RX_GPIO_Port GPIOA
#define PI_UART_TX_Pin GPIO_PIN_11
#define PI_UART_TX_GPIO_Port GPIOA
#define PI_UART_RX_Pin GPIO_PIN_12
#define PI_UART_RX_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
