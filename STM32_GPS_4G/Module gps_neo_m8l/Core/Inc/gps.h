/*
 * gps.h
 *
 *  Created on: Aug 20, 2026
 *      Author: Tran Khoi Nguyen
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_UTC_TIME_LENGTH  11U
#define GPS_UTC_DATE_LENGTH   7U

typedef enum
{
    GPS_NO_FIX = 0,
    GPS_FIXED,
    GPS_LAST_VALID,
    GPS_STALE,
    GPS_FAULT
} GPS_State_t;

typedef enum
{
    GPS_COMM_STARTING = 0,
    GPS_COMM_OK,
    GPS_COMM_TIMEOUT,
    GPS_COMM_UART_ERROR
} GPS_CommState_t;

typedef struct
{
    /* Vi tri hop le hien tai, hoac vi tri hop le gan nhat. */
    double latitude;
    double longitude;

    float speed_kmh;
    float course;
    float hdop;
    float altitude;

    uint8_t satellites;
    uint8_t fix_quality;
    uint8_t rmc_valid;
    uint8_t fix_valid;
    uint8_t has_position;

    /* Thoi gian tinh theo HAL_GetTick(), dung de tinh tuoi du lieu. */
    uint32_t last_nmea_ms;
    uint32_t last_gga_ms;
    uint32_t last_rmc_ms;
    uint32_t last_fix_ms;
    uint32_t location_age_ms;

    /* UTC cua vi tri hop le gan nhat, lay tu cau RMC. */
    char utc_time[GPS_UTC_TIME_LENGTH];
    char utc_date[GPS_UTC_DATE_LENGTH];

    GPS_State_t state;
    GPS_CommState_t comm_state;

    /* Bo dem chan doan de xem tren debugger/dashboard. */
    uint32_t checksum_error_count;
    uint32_t overflow_count;
    uint32_t dropped_line_count;
    uint32_t uart_error_count;
} GPS_Data_t;

void GPS_Init(UART_HandleTypeDef *huart);

/* Goi thuong xuyen trong main loop; khong goi parser trong ISR. */
void GPS_Task(void);

/* Hai ham nay duoc goi tu callback HAL trong main.c. */
void GPS_RxCallback(UART_HandleTypeDef *huart);
void GPS_ErrorCallback(UART_HandleTypeDef *huart);

const GPS_Data_t *GPS_GetData(void);
const char *GPS_StateToString(GPS_State_t state);
const char *GPS_CommStateToString(GPS_CommState_t state);

void GPS_BuildThingsBoardJSON(char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* INC_GPS_H_ */
