#include "comms.h"
#include "core/crc8.h"
#include "core/uart.h"
#include <string.h>

static comms_state_t state = CommsState_Length;
static uint8_t data_byte_count = 0;

static const comms_packet_t retx_packet = {
    .length = 1,
    .data = {[0] = PACKET_REXT_DATA0, [1 ...(PACKET_DATA_LEN - 1)] = 0xff},
    .crc = 0xfc};
static const comms_packet_t ack_packet = {
    .length = 1,
    .data = {[0] = PACKET_ACK_DATA0, [1 ...(PACKET_DATA_LEN - 1)] = 0xff},
    .crc = 0xcc};

static comms_packet_t temp_packet = {.length = 0, .data = {0}, .crc = 0};
static comms_packet_t last_transmitted_packet = {
    .length = 0, .data = {0}, .crc = 0};

static comms_packet_t packet_buffer[PACKET_BUFF_LEN];
static uint32_t packet_read_index = 0;
static uint32_t packet_write_index = 0;
static uint32_t packet_buffer_mask = PACKET_BUFF_LEN - 1;

bool comms_is_single_byte_packet(const comms_packet_t *packet, uint8_t byte) {
  if (packet->length != 1) {
    return false;
  }

  if (packet->data[0] != byte) {
    return false;
  }

  for (uint8_t i = 1; i < PACKET_DATA_LEN; i++) {
    if (packet->data[i] != 0xff) {
      return false;
    }
  }

  return true;
}

void comms_create_single_byte_packet(comms_packet_t *packet, uint8_t byte) {
  memset(packet, 0xff, sizeof(comms_packet_t));
  packet->length = 1;
  packet->data[0] = byte;
  for (uint8_t i = 1; i < PACKET_DATA_LEN; i++) {
    packet->data[i] = 0xff;
  }
  packet->crc = comms_compute_crc(packet);
}

void comms_update(void) {
  while (uart_data_available()) {
    switch (state) {
    case CommsState_Length:
      temp_packet.length = uart_read_byte();
      state = CommsState_Data;
      break;

    case CommsState_Data:
      temp_packet.data[data_byte_count++] = uart_read_byte();
      if (data_byte_count >= PACKET_DATA_LEN) {
        data_byte_count = 0;
        state = CommsState_CRC;
      }
      break;

    case CommsState_CRC:
      temp_packet.crc = uart_read_byte();
      uint8_t computed_crc = crc8_h2f_lut((uint8_t *)(&temp_packet),
                                          PACKET_DATA_LEN + PACKET_SIZE_LEN);
      if (temp_packet.crc != computed_crc) {
        comms_write(&retx_packet);
        state = CommsState_Length;
        break;
      }

      if (comms_is_single_byte_packet(&temp_packet, PACKET_ACK_DATA0)) {
        comms_write(&last_transmitted_packet);
        state = CommsState_Length;
        break;
      }

      if (comms_is_single_byte_packet(&temp_packet, PACKET_REXT_DATA0)) {
        state = CommsState_Length;
        break;
      }

      memcpy(&packet_buffer[packet_write_index], &temp_packet,
             sizeof(comms_packet_t));
      packet_write_index = (packet_write_index + 1) & packet_buffer_mask;
      comms_write(&ack_packet);

      state = CommsState_Length;

      break;

    default: {
      state = CommsState_Length;
    }
    }
  }
}

uint8_t comms_compute_crc(comms_packet_t *packet) {
  return crc8_h2f_lut((uint8_t *)packet, sizeof(comms_packet_t));
}

bool comms_packets_available(void) {
  return packet_read_index != packet_write_index;
}

void comms_write(const comms_packet_t *packet) {
  uart_write((uint8_t *)packet, PACKET_TOT_LEN);
  memcpy(&last_transmitted_packet, packet, sizeof(comms_packet_t));
}

void comms_read(comms_packet_t *packet) {
  memcpy(packet, &packet_buffer[packet_read_index], sizeof(comms_packet_t));
  packet_read_index = (packet_read_index + 1) & packet_buffer_mask;
}
