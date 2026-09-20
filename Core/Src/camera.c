#include "camera.h"

static volatile uint8_t camera_enabled;

void Camera_Init(void)
{
  camera_enabled = 0U;
}

void Camera_Enable(uint8_t enabled)
{
  camera_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t Camera_GetPersonDetected(void)
{
  /* Safe stub: add the real camera model/protocol here later. */
  return 0U;
}

uint8_t Camera_IsEnabled(void)
{
  return camera_enabled;
}
