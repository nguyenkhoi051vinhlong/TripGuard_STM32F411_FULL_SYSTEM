#include "ld2410.h"

#define LD2410_FRAME_SIZE         23U
#define LD2410_ONLINE_TIMEOUT_MS 1500U

static UART_HandleTypeDef *ld2410_uart;
static uint8_t ld2410_rx_byte;
static uint8_t ld2410_frame[LD2410_FRAME_SIZE];
static uint8_t ld2410_frame_index;

static volatile uint8_t ld2410_target_state;
static volatile uint16_t ld2410_distance_cm;
static volatile uint16_t ld2410_moving_distance_cm;
static volatile uint16_t ld2410_still_distance_cm;
static volatile uint8_t ld2410_moving_energy;
static volatile uint8_t ld2410_still_energy;
static volatile uint32_t ld2410_last_frame_ms;
static volatile uint32_t ld2410_valid_frame_count;
static volatile uint32_t ld2410_invalid_frame_count;
static volatile uint32_t ld2410_uart_error_count;
static volatile uint32_t ld2410_rx_restart_error_count;

static void LD2410_StartReceive(void)
{
  HAL_StatusTypeDef status;

  if (ld2410_uart == NULL)
  {
    return;
  }

  status = HAL_UART_Receive_IT(ld2410_uart, &ld2410_rx_byte, 1U);
  if ((status != HAL_OK) && (status != HAL_BUSY))
  {
    ld2410_rx_restart_error_count++;
  }
}

static void LD2410_ResetParser(void)
{
  ld2410_frame_index = 0U;
}

/*
 * Parse the fixed 23-byte basic target frame emitted by the LD2410.
 * This is intentionally the same framing logic as the proven standalone
 * radar test: F4 F3 F2 F1, data length 13, then F8 F7 F6 F5.
 */
static void LD2410_ProcessByte(uint8_t value)
{
  static const uint8_t header[4] = {0xF4U, 0xF3U, 0xF2U, 0xF1U};

  if (ld2410_frame_index < 4U)
  {
    if (value == header[ld2410_frame_index])
    {
      ld2410_frame[ld2410_frame_index++] = value;
    }
    else
    {
      ld2410_frame_index = (value == 0xF4U) ? 1U : 0U;
      if (ld2410_frame_index == 1U)
      {
        ld2410_frame[0] = value;
      }
    }
    return;
  }

  ld2410_frame[ld2410_frame_index++] = value;

  if (ld2410_frame_index == 6U)
  {
    if ((ld2410_frame[4] != 0x0DU) ||
        (ld2410_frame[5] != 0x00U))
    {
      ld2410_invalid_frame_count++;
      LD2410_ResetParser();
    }
    return;
  }

  if (ld2410_frame_index < LD2410_FRAME_SIZE)
  {
    return;
  }

  LD2410_ResetParser();

  if ((ld2410_frame[6]  != 0x02U) ||
      (ld2410_frame[7]  != 0xAAU) ||
      (ld2410_frame[8]  >  0x03U) ||
      (ld2410_frame[11] >  100U)  ||
      (ld2410_frame[14] >  100U)  ||
      (ld2410_frame[17] != 0x55U) ||
      (ld2410_frame[18] != 0x00U) ||
      (ld2410_frame[19] != 0xF8U) ||
      (ld2410_frame[20] != 0xF7U) ||
      (ld2410_frame[21] != 0xF6U) ||
      (ld2410_frame[22] != 0xF5U))
  {
    ld2410_invalid_frame_count++;
    return;
  }

  ld2410_target_state = ld2410_frame[8];
  ld2410_moving_distance_cm =
      (uint16_t)((uint16_t)ld2410_frame[9] |
                 ((uint16_t)ld2410_frame[10] << 8));
  ld2410_moving_energy = ld2410_frame[11];
  ld2410_still_distance_cm =
      (uint16_t)((uint16_t)ld2410_frame[12] |
                 ((uint16_t)ld2410_frame[13] << 8));
  ld2410_still_energy = ld2410_frame[14];

  /*
   * Mot so firmware LD2410 tra detection_distance (byte 15..16) bang 0.
   * Chon khoang cach tu moving/still theo target state de dashboard van
   * hien dung du lieu da duoc chung minh bang bai test TTL tren PA12.
   */
  if (ld2410_target_state == 1U)
  {
    ld2410_distance_cm = ld2410_moving_distance_cm;
  }
  else if (ld2410_target_state == 2U)
  {
    ld2410_distance_cm = ld2410_still_distance_cm;
  }
  else if (ld2410_target_state == 3U)
  {
    if (ld2410_moving_distance_cm == 0U)
    {
      ld2410_distance_cm = ld2410_still_distance_cm;
    }
    else if (ld2410_still_distance_cm == 0U)
    {
      ld2410_distance_cm = ld2410_moving_distance_cm;
    }
    else
    {
      ld2410_distance_cm =
          (ld2410_moving_distance_cm < ld2410_still_distance_cm) ?
          ld2410_moving_distance_cm : ld2410_still_distance_cm;
    }
  }
  else
  {
    ld2410_distance_cm = 0U;
  }
  ld2410_last_frame_ms = HAL_GetTick();
  ld2410_valid_frame_count++;
}

void LD2410_Init(UART_HandleTypeDef *huart)
{
  ld2410_uart = huart;
  ld2410_target_state = 0U;
  ld2410_distance_cm = 0U;
  ld2410_moving_distance_cm = 0U;
  ld2410_still_distance_cm = 0U;
  ld2410_moving_energy = 0U;
  ld2410_still_energy = 0U;
  ld2410_last_frame_ms = 0U;
  ld2410_valid_frame_count = 0U;
  ld2410_invalid_frame_count = 0U;
  ld2410_uart_error_count = 0U;
  ld2410_rx_restart_error_count = 0U;
  LD2410_ResetParser();
  LD2410_StartReceive();
}

void LD2410_Task(void)
{
  if ((ld2410_last_frame_ms != 0U) &&
      ((uint32_t)(HAL_GetTick() - ld2410_last_frame_ms) >
       LD2410_ONLINE_TIMEOUT_MS))
  {
    ld2410_target_state = 0U;
    ld2410_distance_cm = 0U;
    ld2410_moving_distance_cm = 0U;
    ld2410_still_distance_cm = 0U;
    ld2410_moving_energy = 0U;
    ld2410_still_energy = 0U;
  }
}

void LD2410_RxCallback(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart != ld2410_uart))
  {
    return;
  }

  LD2410_ProcessByte(ld2410_rx_byte);
  LD2410_StartReceive();
}

void LD2410_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart != ld2410_uart))
  {
    return;
  }

  ld2410_uart_error_count++;
  LD2410_ResetParser();

  /* Reading SR/DR clears PE, FE, NE and ORE on the STM32F411 UART. */
  __HAL_UART_CLEAR_PEFLAG(huart);
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  huart->RxState = HAL_UART_STATE_READY;
  LD2410_StartReceive();
}

uint8_t LD2410_IsOnline(void)
{
  return (uint8_t)((ld2410_last_frame_ms != 0U) &&
                   ((uint32_t)(HAL_GetTick() - ld2410_last_frame_ms) <=
                    LD2410_ONLINE_TIMEOUT_MS));
}

uint8_t LD2410_IsPresenceDetected(void)
{
  return (uint8_t)((LD2410_IsOnline() != 0U) &&
                   (ld2410_target_state != 0U));
}

uint8_t LD2410_GetTargetState(void)
{
  return ld2410_target_state;
}

uint16_t LD2410_GetDistanceCm(void)
{
  return ld2410_distance_cm;
}

uint16_t LD2410_GetMovingDistanceCm(void)
{
  return ld2410_moving_distance_cm;
}

uint16_t LD2410_GetStillDistanceCm(void)
{
  return ld2410_still_distance_cm;
}

uint8_t LD2410_GetMovingEnergy(void)
{
  return ld2410_moving_energy;
}

uint8_t LD2410_GetStillEnergy(void)
{
  return ld2410_still_energy;
}

uint32_t LD2410_GetValidFrameCount(void)
{
  return ld2410_valid_frame_count;
}

uint32_t LD2410_GetInvalidFrameCount(void)
{
  return ld2410_invalid_frame_count;
}

uint32_t LD2410_GetUartErrorCount(void)
{
  return ld2410_uart_error_count;
}

uint32_t LD2410_GetRxOverflowCount(void)
{
  return ld2410_rx_restart_error_count;
}
