#include "jq8900.h"

#define JQ8900_IO_TRIGGER_MS 200U

static GPIO_TypeDef *jq8900_io_port;
static uint16_t jq8900_io_pin;

void JQ8900_Init(GPIO_TypeDef *io_port, uint16_t io_pin)
{
  jq8900_io_port = io_port;
  jq8900_io_pin = io_pin;
  if (jq8900_io_port != NULL)
  {
    HAL_GPIO_WritePin(jq8900_io_port, jq8900_io_pin, GPIO_PIN_SET);
  }
}

uint8_t JQ8900_PlayTrack(uint16_t track)
{
  if ((track == 0U) || (jq8900_io_port == NULL))
  {
    return 0U;
  }

  /* IO1 active-low: radar va SOS dung chung mot file am thanh. */
  HAL_GPIO_WritePin(jq8900_io_port, jq8900_io_pin, GPIO_PIN_RESET);
  HAL_Delay(JQ8900_IO_TRIGGER_MS);
  HAL_GPIO_WritePin(jq8900_io_port, jq8900_io_pin, GPIO_PIN_SET);
  return 1U;
}

uint8_t JQ8900_Stop(void)
{
  if (jq8900_io_port == NULL)
  {
    return 0U;
  }

  /* IO1 khong co lenh stop rieng; tra chan ve muc nghi. */
  HAL_GPIO_WritePin(jq8900_io_port, jq8900_io_pin, GPIO_PIN_SET);
  return 1U;
}

uint8_t JQ8900_IsBusyRaw(void)
{
  return (HAL_GPIO_ReadPin(AUDIO_BUSY_GPIO_Port,
                           AUDIO_BUSY_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}
