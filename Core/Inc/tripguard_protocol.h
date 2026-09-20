#ifndef INC_TRIPGUARD_PROTOCOL_H_
#define INC_TRIPGUARD_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TG_PROTOCOL_VERSION       0x01U

#define TG_NODE_GATEWAY           0x01U
#define TG_NODE_REMOTE            0x02U

#define TG_EVENT_REAR_CONFIRM     0x10U
#define TG_EVENT_PERSON           0x11U
#define TG_EVENT_RADAR_CLEAR      0x12U
#define TG_EVENT_RADAR_HEALTH     0x13U
#define TG_EVENT_REMOTE_SOS       0x14U
#define TG_EVENT_ACK              0x80U
#define TG_EVENT_NOP              0x00U

/* Compatibility name for code that describes PERSON as a radar event. */
#define TG_EVENT_RADAR_PERSON     TG_EVENT_PERSON

#define TG_VALUE_ACK_OK           0x0001U
#define TG_VALUE_RADAR_UNHEALTHY  0x0000U
#define TG_VALUE_RADAR_HEALTHY    0x0001U

#define TG_SPI_HEADER_1           0xAAU
#define TG_SPI_HEADER_2           0x55U
#define TG_SPI_PACKET_SIZE        12U

/* New one-way frame: A5,version,command,node,seq,value,flags,crc,end. */
#define TG_SPI_FRAME_MAGIC        0xA5U
#define TG_SPI_FRAME_END          0x5AU

typedef struct
{
  uint8_t version;
  uint8_t node_id;
  uint8_t event;
  uint8_t flags;
  uint16_t sequence;
  uint16_t value;
} tripguard_spi_packet_t;

uint16_t tripguard_crc16_ccitt_false(const uint8_t *data, size_t len);

bool tripguard_spi_encode(const tripguard_spi_packet_t *packet,
                          uint8_t frame[TG_SPI_PACKET_SIZE]);

bool tripguard_spi_encode_framed(const tripguard_spi_packet_t *packet,
                                 uint8_t frame[TG_SPI_PACKET_SIZE]);

bool tripguard_spi_decode(const uint8_t frame[TG_SPI_PACKET_SIZE],
                          tripguard_spi_packet_t *packet);

bool tripguard_spi_validate(const uint8_t frame[TG_SPI_PACKET_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* INC_TRIPGUARD_PROTOCOL_H_ */
