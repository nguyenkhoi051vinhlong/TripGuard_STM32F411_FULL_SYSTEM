#ifndef INC_TRIPGUARD_EVENT_H_
#define INC_TRIPGUARD_EVENT_H_

#include <stdint.h>

typedef enum
{
  TG_EVT_NONE = 0,
  TG_EVT_REAR_CONFIRM,
  TG_EVT_REMOTE_SOS,
  TG_EVT_PERSON,
  TG_EVT_RADAR_CLEAR,
  TG_EVT_RADAR_HEALTH
} tripguard_event_type_t;

typedef struct
{
  tripguard_event_type_t type;
  uint8_t source_node;
  uint16_t sequence;
  uint16_t value;
} tripguard_event_t;

typedef struct
{
  uint8_t node_id;
  uint16_t sequence;
  uint16_t value;
} tripguard_ack_t;

#endif /* INC_TRIPGUARD_EVENT_H_ */
