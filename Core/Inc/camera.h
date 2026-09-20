#ifndef INC_CAMERA_H_
#define INC_CAMERA_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Camera integration boundary.  This project has no camera transport/model
 * yet, therefore the safe stub never reports a person.
 */
void Camera_Init(void);
void Camera_Enable(uint8_t enabled);
uint8_t Camera_GetPersonDetected(void);
uint8_t Camera_IsEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_CAMERA_H_ */
