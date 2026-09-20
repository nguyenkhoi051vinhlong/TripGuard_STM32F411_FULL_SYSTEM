#ifndef INC_TRIPGUARD_GATEWAY_H_
#define INC_TRIPGUARD_GATEWAY_H_

#include "cmsis_os.h"

#include <stdbool.h>
#include <stdint.h>

/* Rear Pod reports once per second, but BLE Mesh LPN/Friend recovery can lose
 * several consecutive reports. The Rear Pod itself still reports a real UART
 * fault after 3 seconds, so this longer transport timeout only prevents false
 * offline transitions caused by Mesh jitter. */
#define TRIPGUARD_GATEWAY_RADAR_TIMEOUT_MS  10000U

extern volatile uint8_t tripguard_gateway_radar_healthy;
extern volatile uint8_t tripguard_gateway_radar_person;
extern volatile uint16_t tripguard_gateway_radar_distance_cm;
extern volatile uint16_t tripguard_gateway_radar_last_sequence;
extern volatile uint32_t tripguard_gateway_radar_last_packet_ms;
extern volatile uint32_t tripguard_gateway_radar_person_count;
extern volatile uint32_t tripguard_gateway_radar_clear_count;
extern volatile uint32_t tripguard_gateway_radar_health_count;
extern volatile uint32_t tripguard_gateway_radar_duplicate_count;

bool TripGuard_Gateway_Init(osMessageQueueId_t event_queue,
                            osMessageQueueId_t ack_queue,
                            osEventFlagsId_t system_events);
void TripGuard_Gateway_Task(void *argument);

uint8_t TripGuard_Gateway_IsRadarOnline(void);
uint8_t TripGuard_Gateway_IsRadarHealthy(void);
uint8_t TripGuard_Gateway_IsRadarPersonDetected(void);
uint16_t TripGuard_Gateway_GetRadarDistanceCm(void);
uint32_t TripGuard_Gateway_GetRadarLastPacketAgeMs(void);

#endif /* INC_TRIPGUARD_GATEWAY_H_ */
