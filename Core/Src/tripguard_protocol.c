#include "tripguard_protocol.h"
#include "tripguard_config.h"

#define TG_SPI_CRC_INPUT_OFFSET  2U
#define TG_SPI_CRC_INPUT_SIZE    8U

uint16_t tripguard_crc16_ccitt_false(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFFU;
  size_t index;
  uint8_t bit;

  if ((data == NULL) && (len != 0U))
  {
    return 0U;
  }

  for (index = 0U; index < len; index++)
  {
    crc ^= (uint16_t)data[index] << 8;
    for (bit = 0U; bit < 8U; bit++)
    {
      crc = ((crc & 0x8000U) != 0U) ?
            (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }

  return crc;
}

bool tripguard_spi_encode(const tripguard_spi_packet_t *packet,
                          uint8_t frame[TG_SPI_PACKET_SIZE])
{
  uint16_t crc;

  if ((packet == NULL) || (frame == NULL))
  {
    return false;
  }

  frame[0] = TG_SPI_HEADER_1;
  frame[1] = TG_SPI_HEADER_2;
  frame[2] = packet->version;
  frame[3] = packet->node_id;
  frame[4] = packet->event;
  frame[5] = packet->flags;
  frame[6] = (uint8_t)(packet->sequence & 0x00FFU);
  frame[7] = (uint8_t)(packet->sequence >> 8);
  frame[8] = (uint8_t)(packet->value & 0x00FFU);
  frame[9] = (uint8_t)(packet->value >> 8);

  crc = tripguard_crc16_ccitt_false(&frame[TG_SPI_CRC_INPUT_OFFSET],
                                    TG_SPI_CRC_INPUT_SIZE);
  frame[10] = (uint8_t)(crc & 0x00FFU);
  frame[11] = (uint8_t)(crc >> 8);
  return true;
}

bool tripguard_spi_encode_framed(const tripguard_spi_packet_t *packet,
                                 uint8_t frame[TG_SPI_PACKET_SIZE])
{
  uint16_t crc;

  if ((packet == NULL) || (frame == NULL))
  {
    return false;
  }

  frame[0] = TG_SPI_FRAME_MAGIC;
  frame[1] = packet->version;
  frame[2] = packet->event;
  frame[3] = packet->node_id;
  frame[4] = (uint8_t)(packet->sequence & 0x00FFU);
  frame[5] = (uint8_t)(packet->sequence >> 8);
  frame[6] = (uint8_t)(packet->value & 0x00FFU);
  frame[7] = (uint8_t)(packet->value >> 8);
  frame[8] = packet->flags;
  crc = tripguard_crc16_ccitt_false(frame, 9U);
  frame[9] = (uint8_t)(crc & 0x00FFU);
  frame[10] = (uint8_t)(crc >> 8);
  frame[11] = TG_SPI_FRAME_END;
  return true;
}

bool tripguard_spi_validate(const uint8_t frame[TG_SPI_PACKET_SIZE])
{
  uint16_t expected_crc;
  uint16_t received_crc;

  if (frame == NULL)
  {
    return false;
  }

  if ((frame[0] == TG_SPI_FRAME_MAGIC) &&
      (frame[1] == TG_PROTOCOL_VERSION) &&
      (frame[11] == TG_SPI_FRAME_END))
  {
    expected_crc = tripguard_crc16_ccitt_false(frame, 9U);
    received_crc = (uint16_t)frame[9] | ((uint16_t)frame[10] << 8);
    return expected_crc == received_crc;
  }

  if ((TRIPGUARD_SPI_ACCEPT_LEGACY_FRAME == 0U) ||
      (frame[0] != TG_SPI_HEADER_1) ||
      (frame[1] != TG_SPI_HEADER_2) ||
      (frame[2] != TG_PROTOCOL_VERSION))
  {
    return false;
  }

  expected_crc = tripguard_crc16_ccitt_false(
      &frame[TG_SPI_CRC_INPUT_OFFSET], TG_SPI_CRC_INPUT_SIZE);
  received_crc = (uint16_t)frame[10] | ((uint16_t)frame[11] << 8);
  return expected_crc == received_crc;
}

bool tripguard_spi_decode(const uint8_t frame[TG_SPI_PACKET_SIZE],
                          tripguard_spi_packet_t *packet)
{
  if ((packet == NULL) || !tripguard_spi_validate(frame))
  {
    return false;
  }

  if (frame[0] == TG_SPI_FRAME_MAGIC)
  {
    packet->version = frame[1];
    packet->event = frame[2];
    packet->node_id = frame[3];
    packet->sequence = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
    packet->value = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
    packet->flags = frame[8];
  }
  else
  {
    packet->version = frame[2];
    packet->node_id = frame[3];
    packet->event = frame[4];
    packet->flags = frame[5];
    packet->sequence = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
    packet->value = (uint16_t)frame[8] | ((uint16_t)frame[9] << 8);
  }
  return true;
}
