#ifndef COMMS_H
#define COMMS_H

#include "common-defines.h"

#define PACKET_DATA_LEN (32)
#define PACKET_CRC_LEN (1)
#define PACKET_SIZE_LEN (1)
#define PACKET_TOT_LEN (PACKET_SIZE_LEN + PACKET_DATA_LEN + PACKET_SIZE_LEN)

#define PACKET_REXT_DATA0 (0x19)
#define PACKET_ACK_DATA0 (0x15)

#define PACKET_BUFF_LEN (8)

typedef struct comms_packet_t {
  uint8_t length;
  uint8_t data[PACKET_DATA_LEN];
  uint8_t crc;

} comms_packet_t;

typedef enum commms_state_t {
  CommsState_Length,
  CommsState_Data,
  CommsState_CRC
} commms_state_t;

void comms_update(void);

bool comms_packets_available(void);
void comms_write(const comms_packet_t *packet);
void comms_read(comms_packet_t *packet);

#endif // COMMS_H
