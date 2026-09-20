#ifndef INC_TRIPGUARD_SPI_H_
#define INC_TRIPGUARD_SPI_H_

#include "FreeRTOS.h"
#include "task.h"
#include "tripguard_protocol.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint8_t bytes[TG_SPI_PACKET_SIZE];
} tripguard_spi_rx_frame_t;

bool TripGuard_Spi_Init(TaskHandle_t gateway_task);
bool TripGuard_Spi_PopFrame(tripguard_spi_rx_frame_t *frame);
bool TripGuard_Spi_SendAck(uint8_t node_id, uint16_t sequence);
void TripGuard_Spi_CompleteRequestWithoutAck(void);
bool TripGuard_Spi_IsIdle(void);
void TripGuard_Spi_PrepareForStop(void);
bool TripGuard_Spi_ResumeAfterStop(void);
void TripGuard_Spi_Service(void);
void TripGuard_Spi_TxRxCompleteFromIsr(void);
void TripGuard_Spi_ErrorFromIsr(void);

#endif /* INC_TRIPGUARD_SPI_H_ */
