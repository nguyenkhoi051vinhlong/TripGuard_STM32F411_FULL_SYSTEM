/*
 * gps.c
 *
 *  Created on: Aug 20, 2026
 *      Author: Tran Khoi Nguyen
 */

#include "gps.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GPS_NMEA_BUFFER_SIZE       128U
#define GPS_LINE_QUEUE_DEPTH         4U
#define GPS_TIMEOUT_MS           15000U
#define GPS_SENTENCE_FRESH_MS     3000U
#define GPS_LAST_VALID_MS       120000U
#define GPS_RESTART_RETRY_MS       100U
#define GPS_MAX_FIELDS              24

static UART_HandleTypeDef *gps_uart;
static uint8_t gps_rx_byte;

/* ISR chi ghep cau va dua vao hang doi. Parser chi chay trong main loop. */
static char gps_rx_buffer[GPS_NMEA_BUFFER_SIZE];
static volatile uint16_t gps_rx_index;
static volatile uint8_t gps_receiving_sentence;

/* Hang doi SPSC: ISR ghi head, main loop ghi tail. */
static char gps_line_queue[GPS_LINE_QUEUE_DEPTH][GPS_NMEA_BUFFER_SIZE];
static volatile uint8_t gps_queue_head;
static volatile uint8_t gps_queue_tail;

static volatile uint8_t gps_restart_requested;
static volatile uint32_t gps_isr_overflow_count;
static volatile uint32_t gps_isr_dropped_line_count;
static volatile uint32_t gps_isr_uart_error_count;

static GPS_Data_t gps_data;
static uint32_t gps_start_ms;
static uint32_t gps_last_restart_attempt_ms;

static uint8_t gps_has_nmea;
static uint8_t gps_has_gga;
static uint8_t gps_has_rmc;
static uint8_t gps_gga_valid;
static uint8_t gps_rmc_position_valid;

static double gps_gga_latitude;
static double gps_gga_longitude;
static double gps_rmc_latitude;
static double gps_rmc_longitude;
static char gps_rmc_utc_time[GPS_UTC_TIME_LENGTH];
static char gps_rmc_utc_date[GPS_UTC_DATE_LENGTH];

static uint32_t GPS_AgeMs(uint32_t now, uint32_t timestamp)
{
    return now - timestamp;
}

static uint8_t GPS_IsFresh(uint32_t now,
                           uint32_t timestamp,
                           uint8_t has_value)
{
    return (uint8_t)((has_value != 0U) &&
                     (GPS_AgeMs(now, timestamp) <= GPS_SENTENCE_FRESH_MS));
}

static void GPS_CopyField(char *destination,
                          size_t destination_size,
                          const char *source)
{
    if ((destination == NULL) || (destination_size == 0U))
    {
        return;
    }

    if (source == NULL)
    {
        destination[0] = '\0';
        return;
    }

    strncpy(destination, source, destination_size - 1U);
    destination[destination_size - 1U] = '\0';
}

static int GPS_HexValue(char character)
{
    if ((character >= '0') && (character <= '9'))
    {
        return character - '0';
    }

    if ((character >= 'A') && (character <= 'F'))
    {
        return character - 'A' + 10;
    }

    if ((character >= 'a') && (character <= 'f'))
    {
        return character - 'a' + 10;
    }

    return -1;
}

static uint8_t GPS_ChecksumIsValid(const char *sentence)
{
    const char *checksum_separator;
    uint8_t checksum = 0U;
    int high_nibble;
    int low_nibble;

    if ((sentence == NULL) || (sentence[0] != '$'))
    {
        return 0U;
    }

    checksum_separator = strchr(sentence, '*');
    if ((checksum_separator == NULL) ||
        (checksum_separator[1] == '\0') ||
        (checksum_separator[2] == '\0'))
    {
        return 0U;
    }

    high_nibble = GPS_HexValue(checksum_separator[1]);
    low_nibble = GPS_HexValue(checksum_separator[2]);
    if ((high_nibble < 0) || (low_nibble < 0))
    {
        return 0U;
    }

    for (const char *cursor = sentence + 1;
         cursor < checksum_separator;
         cursor++)
    {
        checksum ^= (uint8_t)(*cursor);
    }

    return (uint8_t)(checksum ==
                     (uint8_t)((high_nibble << 4) | low_nibble));
}

/* Tach field nhung van giu cac field rong cua NMEA. */
static int GPS_SplitFields(char *sentence,
                           char *fields[],
                           int max_fields)
{
    int count = 0;

    if ((sentence == NULL) ||
        (fields == NULL) ||
        (max_fields <= 0))
    {
        return 0;
    }

    fields[count++] = sentence;

    for (char *cursor = sentence;
         (*cursor != '\0') && (count < max_fields);
         cursor++)
    {
        if (*cursor == ',')
        {
            *cursor = '\0';
            fields[count++] = cursor + 1;
        }
    }

    return count;
}

static uint8_t GPS_ParseUInt8(const char *field, uint8_t *result)
{
    char *end_pointer;
    unsigned long value;

    if ((field == NULL) || (result == NULL) || (field[0] == '\0'))
    {
        return 0U;
    }

    value = strtoul(field, &end_pointer, 10);
    if ((end_pointer == field) ||
        (*end_pointer != '\0') ||
        (value > UINT8_MAX))
    {
        return 0U;
    }

    *result = (uint8_t)value;
    return 1U;
}

static uint8_t GPS_ParseFloat(const char *field, float *result)
{
    char *end_pointer;
    float value;

    if ((field == NULL) || (result == NULL) || (field[0] == '\0'))
    {
        return 0U;
    }

    value = strtof(field, &end_pointer);
    if ((end_pointer == field) ||
        (*end_pointer != '\0') ||
        (!isfinite(value)))
    {
        return 0U;
    }

    *result = value;
    return 1U;
}

static uint8_t GPS_ParseCoordinate(const char *value_field,
                                   const char *direction_field,
                                   uint8_t is_latitude,
                                   double *result)
{
    char *end_pointer;
    double raw_value;
    double minutes;
    double decimal;
    uint32_t degrees;
    uint32_t max_degrees;
    char direction;

    if ((value_field == NULL) ||
        (direction_field == NULL) ||
        (result == NULL) ||
        (value_field[0] == '\0') ||
        (direction_field[0] == '\0') ||
        (direction_field[1] != '\0'))
    {
        return 0U;
    }

    raw_value = strtod(value_field, &end_pointer);
    max_degrees = (is_latitude != 0U) ? 90U : 180U;

    if ((end_pointer == value_field) ||
        (*end_pointer != '\0') ||
        (!isfinite(raw_value)) ||
        (raw_value < 0.0) ||
        (raw_value > (((double)max_degrees * 100.0) + 60.0)))
    {
        return 0U;
    }

    degrees = (uint32_t)(raw_value / 100.0);
    minutes = raw_value - ((double)degrees * 100.0);

    if ((degrees > max_degrees) ||
        (minutes < 0.0) ||
        (minutes >= 60.0) ||
        ((degrees == max_degrees) && (minutes > 0.0)))
    {
        return 0U;
    }

    direction = direction_field[0];
    if (is_latitude != 0U)
    {
        if ((direction != 'N') && (direction != 'S'))
        {
            return 0U;
        }
    }
    else if ((direction != 'E') && (direction != 'W'))
    {
        return 0U;
    }

    decimal = (double)degrees + (minutes / 60.0);
    if ((direction == 'S') || (direction == 'W'))
    {
        decimal = -decimal;
    }

    *result = decimal;
    return 1U;
}

static void GPS_RefreshFix(uint32_t now)
{
    uint8_t gga_fresh = GPS_IsFresh(now,
                                    gps_data.last_gga_ms,
                                    gps_has_gga);
    uint8_t rmc_fresh = GPS_IsFresh(now,
                                    gps_data.last_rmc_ms,
                                    gps_has_rmc);

    if ((gga_fresh != 0U) &&
        (rmc_fresh != 0U) &&
        (gps_gga_valid != 0U) &&
        (gps_rmc_position_valid != 0U))
    {
        /* RMC co toa do va UTC; GGA bo sung chat luong fix/HDOP. */
        gps_data.latitude = gps_rmc_latitude;
        gps_data.longitude = gps_rmc_longitude;
        gps_data.fix_valid = 1U;
        gps_data.has_position = 1U;
        gps_data.last_fix_ms = now;
        gps_data.location_age_ms = 0U;

        GPS_CopyField(gps_data.utc_time,
                      sizeof(gps_data.utc_time),
                      gps_rmc_utc_time);
        GPS_CopyField(gps_data.utc_date,
                      sizeof(gps_data.utc_date),
                      gps_rmc_utc_date);
    }
    else
    {
        gps_data.fix_valid = 0U;
    }
}

static void GPS_ParseGGA(char *sentence, uint32_t now)
{
    char *fields[GPS_MAX_FIELDS];
    int field_count;
    uint8_t value_u8;
    float value_float;
    double latitude;
    double longitude;

    field_count = GPS_SplitFields(sentence, fields, GPS_MAX_FIELDS);
    if (field_count < 10)
    {
        return;
    }

    gps_has_gga = 1U;
    gps_data.last_gga_ms = now;
    gps_gga_valid = 0U;

    if (GPS_ParseUInt8(fields[6], &value_u8) != 0U)
    {
        gps_data.fix_quality = value_u8;
    }
    else
    {
        gps_data.fix_quality = 0U;
    }

    if (GPS_ParseUInt8(fields[7], &value_u8) != 0U)
    {
        gps_data.satellites = value_u8;
    }
    else
    {
        gps_data.satellites = 0U;
    }

    if (GPS_ParseFloat(fields[8], &value_float) != 0U)
    {
        gps_data.hdop = value_float;
    }

    if (GPS_ParseFloat(fields[9], &value_float) != 0U)
    {
        gps_data.altitude = value_float;
    }

    if ((gps_data.fix_quality > 0U) &&
        (GPS_ParseCoordinate(fields[2], fields[3], 1U, &latitude) != 0U) &&
        (GPS_ParseCoordinate(fields[4], fields[5], 0U, &longitude) != 0U))
    {
        gps_gga_latitude = latitude;
        gps_gga_longitude = longitude;
        gps_gga_valid = 1U;
    }

    GPS_RefreshFix(now);
}

static void GPS_ParseRMC(char *sentence, uint32_t now)
{
    char *fields[GPS_MAX_FIELDS];
    int field_count;
    float value_float;
    double latitude;
    double longitude;
    uint8_t status_active;

    field_count = GPS_SplitFields(sentence, fields, GPS_MAX_FIELDS);
    if (field_count < 9)
    {
        return;
    }

    gps_has_rmc = 1U;
    gps_data.last_rmc_ms = now;
    gps_rmc_position_valid = 0U;

    status_active = (uint8_t)((fields[2][0] == 'A') &&
                              (fields[2][1] == '\0'));

    if ((status_active != 0U) &&
        (GPS_ParseCoordinate(fields[3], fields[4], 1U, &latitude) != 0U) &&
        (GPS_ParseCoordinate(fields[5], fields[6], 0U, &longitude) != 0U))
    {
        gps_rmc_latitude = latitude;
        gps_rmc_longitude = longitude;
        gps_rmc_position_valid = 1U;

        if (GPS_ParseFloat(fields[7], &value_float) != 0U)
        {
            gps_data.speed_kmh = value_float * 1.852f;
        }

        if (GPS_ParseFloat(fields[8], &value_float) != 0U)
        {
            gps_data.course = value_float;
        }

        GPS_CopyField(gps_rmc_utc_time,
                      sizeof(gps_rmc_utc_time),
                      fields[1]);

        if (field_count > 9)
        {
            GPS_CopyField(gps_rmc_utc_date,
                          sizeof(gps_rmc_utc_date),
                          fields[9]);
        }
        else
        {
            gps_rmc_utc_date[0] = '\0';
        }
    }

    gps_data.rmc_valid = gps_rmc_position_valid;
    GPS_RefreshFix(now);
}

static void GPS_ProcessSentence(const char *sentence)
{
    char parse_buffer[GPS_NMEA_BUFFER_SIZE];
    char *checksum_separator;
    uint32_t now;

    if (GPS_ChecksumIsValid(sentence) == 0U)
    {
        gps_data.checksum_error_count++;
        return;
    }

    now = HAL_GetTick();
    gps_has_nmea = 1U;
    gps_data.last_nmea_ms = now;

    GPS_CopyField(parse_buffer, sizeof(parse_buffer), sentence);
    checksum_separator = strchr(parse_buffer, '*');
    if (checksum_separator == NULL)
    {
        return;
    }
    *checksum_separator = '\0';

    /* NMEA thuong co dang $ttXXX, trong do tt la talker ID. */
    if ((strlen(parse_buffer) < 7U) ||
        (parse_buffer[0] != '$') ||
        (parse_buffer[6] != ','))
    {
        return;
    }

    if (strncmp(&parse_buffer[3], "GGA", 3U) == 0)
    {
        GPS_ParseGGA(parse_buffer, now);
    }
    else if (strncmp(&parse_buffer[3], "RMC", 3U) == 0)
    {
        GPS_ParseRMC(parse_buffer, now);
    }
}

static uint8_t GPS_PopLine(char *destination, size_t destination_size)
{
    uint8_t tail;

    if ((destination == NULL) || (destination_size == 0U))
    {
        return 0U;
    }

    tail = gps_queue_tail;
    if (tail == gps_queue_head)
    {
        return 0U;
    }

    __DMB();
    GPS_CopyField(destination,
                  destination_size,
                  gps_line_queue[tail]);

    gps_queue_tail = (uint8_t)((tail + 1U) % GPS_LINE_QUEUE_DEPTH);
    return 1U;
}

static void GPS_UpdateState(uint32_t now)
{
    uint8_t gga_fresh;
    uint8_t rmc_fresh;

    if (gps_restart_requested != 0U)
    {
        gps_data.comm_state = GPS_COMM_UART_ERROR;
    }
    else if ((gps_has_nmea != 0U) &&
             (GPS_AgeMs(now, gps_data.last_nmea_ms) <= GPS_TIMEOUT_MS))
    {
        gps_data.comm_state = GPS_COMM_OK;
    }
    else if ((gps_has_nmea == 0U) &&
             (GPS_AgeMs(now, gps_start_ms) <= GPS_TIMEOUT_MS))
    {
        gps_data.comm_state = GPS_COMM_STARTING;
    }
    else
    {
        gps_data.comm_state = GPS_COMM_TIMEOUT;
    }

    gga_fresh = GPS_IsFresh(now,
                            gps_data.last_gga_ms,
                            gps_has_gga);
    rmc_fresh = GPS_IsFresh(now,
                            gps_data.last_rmc_ms,
                            gps_has_rmc);

    if ((gga_fresh == 0U) ||
        (rmc_fresh == 0U) ||
        (gps_gga_valid == 0U) ||
        (gps_rmc_position_valid == 0U))
    {
        gps_data.fix_valid = 0U;
    }

    gps_data.rmc_valid =
        (uint8_t)((rmc_fresh != 0U) &&
                  (gps_rmc_position_valid != 0U));

    if (gps_data.has_position != 0U)
    {
        gps_data.location_age_ms =
            GPS_AgeMs(now, gps_data.last_fix_ms);
    }
    else
    {
        gps_data.location_age_ms = UINT32_MAX;
    }

    if ((gps_data.comm_state == GPS_COMM_TIMEOUT) ||
        (gps_data.comm_state == GPS_COMM_UART_ERROR))
    {
        /* Mat duong truyen GPS la FAULT, du van giu vi tri cu noi bo. */
        gps_data.state = GPS_FAULT;
    }
    else if ((gps_data.fix_valid != 0U) &&
             (gps_data.comm_state == GPS_COMM_OK))
    {
        gps_data.state = GPS_FIXED;
    }
    else if (gps_data.has_position != 0U)
    {
        if (gps_data.location_age_ms <= GPS_LAST_VALID_MS)
        {
            gps_data.state = GPS_LAST_VALID;
        }
        else
        {
            gps_data.state = GPS_STALE;
        }
    }
    else
    {
        gps_data.state = GPS_NO_FIX;
    }
}

static void GPS_RestartReceptionIfNeeded(uint32_t now)
{
    uint32_t interrupt_state;
    HAL_StatusTypeDef receive_status;

    if ((gps_uart == NULL) ||
        (gps_restart_requested == 0U) ||
        (GPS_AgeMs(now, gps_last_restart_attempt_ms) < GPS_RESTART_RETRY_MS))
    {
        return;
    }

    gps_last_restart_attempt_ms = now;

    interrupt_state = __get_PRIMASK();
    __disable_irq();

    gps_restart_requested = 0U;
    gps_rx_index = 0U;
    gps_receiving_sentence = 0U;

    (void)HAL_UART_AbortReceive(gps_uart);
    receive_status = HAL_UART_Receive_IT(gps_uart, &gps_rx_byte, 1U);

    if (receive_status != HAL_OK)
    {
        gps_restart_requested = 1U;
    }

    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
}

void GPS_Init(UART_HandleTypeDef *huart)
{
    memset(&gps_data, 0, sizeof(gps_data));
    memset(gps_rx_buffer, 0, sizeof(gps_rx_buffer));
    memset(gps_line_queue, 0, sizeof(gps_line_queue));

    gps_uart = huart;
    gps_rx_index = 0U;
    gps_receiving_sentence = 0U;
    gps_queue_head = 0U;
    gps_queue_tail = 0U;
    gps_restart_requested = 0U;
    gps_isr_overflow_count = 0U;
    gps_isr_dropped_line_count = 0U;
    gps_isr_uart_error_count = 0U;

    gps_has_nmea = 0U;
    gps_has_gga = 0U;
    gps_has_rmc = 0U;
    gps_gga_valid = 0U;
    gps_rmc_position_valid = 0U;
    gps_rmc_utc_time[0] = '\0';
    gps_rmc_utc_date[0] = '\0';

    gps_start_ms = HAL_GetTick();
    gps_last_restart_attempt_ms = gps_start_ms - GPS_RESTART_RETRY_MS;
    gps_data.state = GPS_NO_FIX;
    gps_data.comm_state = GPS_COMM_STARTING;
    gps_data.location_age_ms = UINT32_MAX;

    if (gps_uart == NULL)
    {
        gps_data.state = GPS_FAULT;
        gps_data.comm_state = GPS_COMM_UART_ERROR;
        return;
    }

    if (HAL_UART_Receive_IT(gps_uart, &gps_rx_byte, 1U) != HAL_OK)
    {
        gps_restart_requested = 1U;
        gps_data.comm_state = GPS_COMM_UART_ERROR;
    }
}

void GPS_Task(void)
{
    char sentence[GPS_NMEA_BUFFER_SIZE];
    uint32_t now = HAL_GetTick();

    GPS_RestartReceptionIfNeeded(now);

    while (GPS_PopLine(sentence, sizeof(sentence)) != 0U)
    {
        GPS_ProcessSentence(sentence);
    }

    gps_data.overflow_count = gps_isr_overflow_count;
    gps_data.dropped_line_count = gps_isr_dropped_line_count;
    gps_data.uart_error_count = gps_isr_uart_error_count;

    GPS_UpdateState(HAL_GetTick());
}

void GPS_RxCallback(UART_HandleTypeDef *huart)
{
    uint8_t head;
    uint8_t next_head;

    if ((gps_uart == NULL) ||
        (huart == NULL) ||
        (huart->Instance != gps_uart->Instance))
    {
        return;
    }

    if (gps_rx_byte == '$')
    {
        gps_rx_index = 0U;
        gps_receiving_sentence = 1U;
        gps_rx_buffer[gps_rx_index++] = '$';
    }
    else if ((gps_rx_byte == '\n') &&
             (gps_receiving_sentence != 0U))
    {
        gps_rx_buffer[gps_rx_index] = '\0';

        head = gps_queue_head;
        next_head = (uint8_t)((head + 1U) % GPS_LINE_QUEUE_DEPTH);

        if (next_head != gps_queue_tail)
        {
            memcpy(gps_line_queue[head],
                   gps_rx_buffer,
                   (size_t)gps_rx_index + 1U);
            __DMB();
            gps_queue_head = next_head;
        }
        else
        {
            gps_isr_dropped_line_count++;
        }

        gps_rx_index = 0U;
        gps_receiving_sentence = 0U;
    }
    else if ((gps_rx_byte != '\r') &&
             (gps_receiving_sentence != 0U))
    {
        if (gps_rx_index < (GPS_NMEA_BUFFER_SIZE - 1U))
        {
            gps_rx_buffer[gps_rx_index++] = (char)gps_rx_byte;
        }
        else
        {
            gps_rx_index = 0U;
            gps_receiving_sentence = 0U;
            gps_isr_overflow_count++;
        }
    }

    if (HAL_UART_Receive_IT(gps_uart, &gps_rx_byte, 1U) != HAL_OK)
    {
        gps_restart_requested = 1U;
    }
}

void GPS_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((gps_uart == NULL) ||
        (huart == NULL) ||
        (huart->Instance != gps_uart->Instance))
    {
        return;
    }

    gps_rx_index = 0U;
    gps_receiving_sentence = 0U;
    gps_isr_uart_error_count++;
    gps_restart_requested = 1U;
}

const GPS_Data_t *GPS_GetData(void)
{
    return &gps_data;
}

const char *GPS_StateToString(GPS_State_t state)
{
    switch (state)
    {
        case GPS_NO_FIX:
            return "NO_FIX";
        case GPS_FIXED:
            return "FIXED";
        case GPS_LAST_VALID:
            return "LAST_VALID";
        case GPS_STALE:
            return "STALE";
        case GPS_FAULT:
            return "GPS_FAULT";
        default:
            return "UNKNOWN";
    }
}

const char *GPS_CommStateToString(GPS_CommState_t state)
{
    switch (state)
    {
        case GPS_COMM_STARTING:
            return "STARTING";
        case GPS_COMM_OK:
            return "OK";
        case GPS_COMM_TIMEOUT:
            return "TIMEOUT";
        case GPS_COMM_UART_ERROR:
            return "UART_ERROR";
        default:
            return "UNKNOWN";
    }
}

void GPS_BuildThingsBoardJSON(char *buffer, size_t buffer_size)
{
    const GPS_Data_t *data = GPS_GetData();
    const char *link_alive;
    const char *position_current;

    if ((buffer == NULL) || (buffer_size == 0U))
    {
        return;
    }

    buffer[0] = '\0';
    link_alive = (data->comm_state == GPS_COMM_OK) ? "true" : "false";
    position_current = (data->state == GPS_FIXED) ? "true" : "false";

    if ((data->has_position != 0U) && (data->state != GPS_FAULT))
    {
        (void)snprintf(
            buffer,
            buffer_size,
            "{\"gps_state\":\"%s\",\"gps_state_code\":%u,"
            "\"gps_link_alive\":%s,\"position_available\":true,"
            "\"position_current\":%s,\"latitude\":%.7f,"
            "\"longitude\":%.7f,\"satellites\":%u,\"hdop\":%.2f,"
            "\"speed_kmh\":%.1f,\"altitude\":%.1f,"
            "\"location_age_s\":%lu}",
            GPS_StateToString(data->state),
            (unsigned int)data->state,
            link_alive,
            position_current,
            data->latitude,
            data->longitude,
            (unsigned int)data->satellites,
            (double)data->hdop,
            (double)data->speed_kmh,
            (double)data->altitude,
            (unsigned long)(data->location_age_ms / 1000U));
    }
    else
    {
        (void)snprintf(
            buffer,
            buffer_size,
            "{\"gps_state\":\"%s\",\"gps_state_code\":%u,"
            "\"gps_link_alive\":%s,\"position_available\":false,"
            "\"position_current\":false}",
            GPS_StateToString(data->state),
            (unsigned int)data->state,
            link_alive);
    }

    buffer[buffer_size - 1U] = '\0';
}
