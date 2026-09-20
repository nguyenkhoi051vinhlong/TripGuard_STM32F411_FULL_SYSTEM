#include "tripguard_spi.h"

#include "main.h"
#include "spi.h"

#include <string.h>

#define TG_SPI_RX_RING_CAPACITY  8U

typedef enum
{
  TG_SPI_PHASE_STOPPED = 0U,
  TG_SPI_PHASE_REQUEST,
  TG_SPI_PHASE_REQUEST_DONE,
  TG_SPI_PHASE_ERROR
} TripGuard_SpiPhase_t;

typedef struct
{
  uint8_t bytes[TG_SPI_PACKET_SIZE];
} TripGuard_SpiRingEntry_t;

static uint8_t spi_rx_dma_buffer[TG_SPI_PACKET_SIZE];
static TripGuard_SpiRingEntry_t spi_rx_ring[TG_SPI_RX_RING_CAPACITY];

static volatile uint8_t spi_rx_head;
static volatile uint8_t spi_rx_tail;
static volatile uint8_t spi_dma_armed;
static volatile TripGuard_SpiPhase_t spi_phase;
static TaskHandle_t spi_gateway_task;

volatile uint32_t tripguard_spi_transaction_count;
volatile uint32_t tripguard_spi_rx_overflow_count;
volatile uint32_t tripguard_spi_hal_error_count;
volatile uint32_t tripguard_spi_rearm_error_count;
volatile uint32_t tripguard_spi_ack_count;
volatile uint32_t tripguard_spi_ack_abort_count;

static HAL_StatusTypeDef TripGuard_Spi_ArmDma(TripGuard_SpiPhase_t phase)
{
  HAL_StatusTypeDef status;

  memset(spi_rx_dma_buffer, 0, sizeof(spi_rx_dma_buffer));
  spi_phase = phase;
  status = HAL_SPI_Receive_DMA(&hspi2,
                               spi_rx_dma_buffer,
                               TG_SPI_PACKET_SIZE);
  spi_dma_armed = (status == HAL_OK) ? 1U : 0U;
  if (status != HAL_OK)
  {
    spi_phase = TG_SPI_PHASE_ERROR;
    tripguard_spi_rearm_error_count++;
  }
  /* One-way wiring has no READY line; keep the legacy output deasserted. */
  HAL_GPIO_WritePin(STM32_READY_GPIO_Port, STM32_READY_Pin, GPIO_PIN_RESET);
  return status;
}

bool TripGuard_Spi_Init(TaskHandle_t gateway_task)
{
  spi_gateway_task = gateway_task;
  spi_rx_head = 0U;
  spi_rx_tail = 0U;
  spi_dma_armed = 0U;
  spi_phase = TG_SPI_PHASE_STOPPED;
  return TripGuard_Spi_ArmDma(TG_SPI_PHASE_REQUEST) == HAL_OK;
}

bool TripGuard_Spi_SendAck(uint8_t node_id, uint16_t sequence)
{
  (void)node_id;
  (void)sequence;

  if ((spi_phase != TG_SPI_PHASE_REQUEST_DONE) ||
      (spi_dma_armed != 0U))
  {
    return false;
  }

  /* Safety commit is complete. Count it locally and receive the next copy. */
  tripguard_spi_ack_count++;
  return TripGuard_Spi_ArmDma(TG_SPI_PHASE_REQUEST) == HAL_OK;
}

void TripGuard_Spi_CompleteRequestWithoutAck(void)
{
  if ((spi_phase == TG_SPI_PHASE_REQUEST_DONE) &&
      (spi_dma_armed == 0U))
  {
    tripguard_spi_ack_abort_count++;
    (void)TripGuard_Spi_ArmDma(TG_SPI_PHASE_REQUEST);
  }
}

bool TripGuard_Spi_IsIdle(void)
{
  bool ring_empty;

  taskENTER_CRITICAL();
  ring_empty = (spi_rx_head == spi_rx_tail);
  taskEXIT_CRITICAL();
  return ring_empty &&
         ((spi_phase == TG_SPI_PHASE_REQUEST) ||
          (spi_phase == TG_SPI_PHASE_STOPPED));
}

void TripGuard_Spi_PrepareForStop(void)
{
  HAL_GPIO_WritePin(STM32_READY_GPIO_Port, STM32_READY_Pin, GPIO_PIN_RESET);
  if (spi_dma_armed != 0U)
  {
    (void)HAL_SPI_Abort(&hspi2);
  }
  spi_dma_armed = 0U;
  spi_phase = TG_SPI_PHASE_STOPPED;
}

bool TripGuard_Spi_ResumeAfterStop(void)
{
  (void)HAL_SPI_Abort(&hspi2);
  if (HAL_SPI_DeInit(&hspi2) != HAL_OK)
  {
    tripguard_spi_rearm_error_count++;
    return false;
  }
  MX_SPI2_Init();
  return TripGuard_Spi_ArmDma(TG_SPI_PHASE_REQUEST) == HAL_OK;
}

bool TripGuard_Spi_PopFrame(tripguard_spi_rx_frame_t *frame)
{
  uint8_t tail;

  if (frame == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  tail = spi_rx_tail;
  if (tail == spi_rx_head)
  {
    taskEXIT_CRITICAL();
    return false;
  }

  memcpy(frame->bytes, spi_rx_ring[tail].bytes, TG_SPI_PACKET_SIZE);
  spi_rx_tail = (uint8_t)((tail + 1U) % TG_SPI_RX_RING_CAPACITY);
  taskEXIT_CRITICAL();
  return true;
}

void TripGuard_Spi_Service(void)
{
  if ((spi_phase != TG_SPI_PHASE_ERROR) ||
      (spi_dma_armed != 0U) ||
      (HAL_GPIO_ReadPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin) != GPIO_PIN_SET))
  {
    return;
  }

  (void)HAL_SPI_Abort(&hspi2);
  (void)TripGuard_Spi_ArmDma(TG_SPI_PHASE_REQUEST);
}

void TripGuard_Spi_TxRxCompleteFromIsr(void)
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  uint8_t head;
  uint8_t next_head;

  spi_dma_armed = 0U;
  HAL_GPIO_WritePin(STM32_READY_GPIO_Port, STM32_READY_Pin, GPIO_PIN_RESET);
  tripguard_spi_transaction_count++;

  if (spi_phase == TG_SPI_PHASE_REQUEST)
  {
    head = spi_rx_head;
    next_head = (uint8_t)((head + 1U) % TG_SPI_RX_RING_CAPACITY);
    if (next_head == spi_rx_tail)
    {
      tripguard_spi_rx_overflow_count++;
    }
    else
    {
      memcpy(spi_rx_ring[head].bytes,
             spi_rx_dma_buffer,
             TG_SPI_PACKET_SIZE);
      __DMB();
      spi_rx_head = next_head;
    }
    spi_phase = TG_SPI_PHASE_REQUEST_DONE;
  }
  else
  {
    spi_phase = TG_SPI_PHASE_ERROR;
  }

  if (spi_gateway_task != NULL)
  {
    vTaskNotifyGiveFromISR(spi_gateway_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}

void TripGuard_Spi_ErrorFromIsr(void)
{
  BaseType_t higher_priority_task_woken = pdFALSE;

  spi_dma_armed = 0U;
  spi_phase = TG_SPI_PHASE_ERROR;
  HAL_GPIO_WritePin(STM32_READY_GPIO_Port, STM32_READY_Pin, GPIO_PIN_RESET);
  tripguard_spi_hal_error_count++;
  if (spi_gateway_task != NULL)
  {
    vTaskNotifyGiveFromISR(spi_gateway_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}
