/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(JQ8900_IO1_GPIO_Port, JQ8900_IO1_Pin, GPIO_PIN_SET);

  /* Power-domain defaults: keep the existing system ON at boot, keep the
     shutdown request and SPI READY deasserted until their owners start. */
  HAL_GPIO_WritePin(PI_SHUTDOWN_REQ_GPIO_Port, PI_SHUTDOWN_REQ_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,
                    AUDIO_POWER_EN_Pin|PI_POWER_EN_Pin|
                    MODEM_POWER_EN_Pin|GPS_POWER_EN_Pin,
                    GPIO_PIN_SET);
  HAL_GPIO_WritePin(STM32_READY_GPIO_Port, STM32_READY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : JQ8900_IO1_Pin */
  GPIO_InitStruct.Pin = JQ8900_IO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(JQ8900_IO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PI_SHUTDOWN_REQ_Pin */
  GPIO_InitStruct.Pin = PI_SHUTDOWN_REQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PI_SHUTDOWN_REQ_GPIO_Port, &GPIO_InitStruct);

  /* Each high-current load has its own load-switch enable. */
  GPIO_InitStruct.Pin = AUDIO_POWER_EN_Pin|PI_POWER_EN_Pin|
                        MODEM_POWER_EN_Pin|GPS_POWER_EN_Pin|
                        STM32_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SOS_BTN_Pin DRIVER_ACK_BTN_Pin */
  GPIO_InitStruct.Pin = SOS_BTN_Pin|DRIVER_ACK_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SHUTDOWN_BUTTON_Pin */
  GPIO_InitStruct.Pin = SHUTDOWN_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SHUTDOWN_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* POWER_SENSE accepts only a protected 0..3.3 V digital signal. */
  GPIO_InitStruct.Pin = BUS_POWER_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUS_POWER_SENSE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PI_SHUTDOWN_ACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PI_SHUTDOWN_ACK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AUDIO_BUSY_Pin */
  GPIO_InitStruct.Pin = AUDIO_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AUDIO_BUSY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
