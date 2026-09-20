#ifndef INC_LD2410_H_
#define INC_LD2410_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void LD2410_Init(UART_HandleTypeDef *huart);
void LD2410_Task(void);
void LD2410_RxCallback(UART_HandleTypeDef *huart);
void LD2410_ErrorCallback(UART_HandleTypeDef *huart);

uint8_t LD2410_IsOnline(void);
uint8_t LD2410_IsPresenceDetected(void);
uint8_t LD2410_GetTargetState(void);
uint16_t LD2410_GetDistanceCm(void);
uint16_t LD2410_GetMovingDistanceCm(void);
uint16_t LD2410_GetStillDistanceCm(void);
uint8_t LD2410_GetMovingEnergy(void);
uint8_t LD2410_GetStillEnergy(void);
uint32_t LD2410_GetValidFrameCount(void);
uint32_t LD2410_GetInvalidFrameCount(void);
uint32_t LD2410_GetUartErrorCount(void);
uint32_t LD2410_GetRxOverflowCount(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_LD2410_H_ */
