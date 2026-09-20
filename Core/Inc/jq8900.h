#ifndef INC_JQ8900_H_
#define INC_JQ8900_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void JQ8900_Init(GPIO_TypeDef *io_port, uint16_t io_pin);
uint8_t JQ8900_PlayTrack(uint16_t track);
uint8_t JQ8900_Stop(void);
uint8_t JQ8900_IsBusyRaw(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_JQ8900_H_ */
