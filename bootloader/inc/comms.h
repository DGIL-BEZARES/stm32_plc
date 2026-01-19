#ifndef COMMS_H
#define COMMS_H

#include "common-defines.h"

#define PACKET_DATA_LEN (32)
#define PACKET_CRC_LEN (1)
#define PACKET_SIZE_LEN (1)
#define PACKET_TOT_LEN (PACKET_SIZE_LEN + PACKET_DATA_LEN + PACKET_SIZE_LEN)

#define PACKET_REXT_DATA0 (0x19)
#define PACKET_ACK_DATA0 (0x15)

#define BL_PACKET_SYNC_OBS_DATA0 (0X20)
#define BL_PACKET_FW_UP_REQ_DATA0 (0X31)
#define BL_PACKET_FW_UP_RES_DATA0 (0X37)
#define BL_PACKET_DEV_ID_REQ_DATA0 (0X3C)
#define BL_PACKET_DEV_ID_RES_DATA0 (0X3F)
#define BL_PACKET_FW_LEN_REQ_DATA0 (0X42)
#define BL_PACKET_FW_LEN_RES_DATA0 (0X45)
#define BL_PACKET_RDY_DATA_DATA0 (0X48)
#define BL_PACKET_FW_UP_SCS_DATA0 (0X54)
#define BL_PACKET_NACK_DATA0 (0X58)

#define PACKET_BUFF_LEN (8)

typedef struct comms_packet_t {
  uint8_t length;
  uint8_t data[PACKET_DATA_LEN];
  uint8_t crc;

} comms_packet_t;

typedef enum comms_state_t {
  CommsState_Length,
  CommsState_Data,
  CommsState_CRC
} comms_state_t;

void comms_update(void);
bool comms_packets_available(void);
void comms_write(const comms_packet_t *packet);
void comms_read(comms_packet_t *packet);
uint8_t comms_compute_crc(comms_packet_t *packet);
bool comms_is_single_byte_packet(const comms_packet_t *packet, uint8_t byte);
void comms_create_single_byte_packet(comms_packet_t *packet, uint8_t byte);

#endif // COMMS_H
