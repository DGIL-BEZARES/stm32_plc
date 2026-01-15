#include "comms.h"
#include "core/crc8.h"
#include "core/uart.h"

static commms_state_t state = CommsState_Length;
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
static uint32_t packet_buffer_mask = PACKET_BUFF_LEN;

static bool comms_is_ackretx_packet(const comms_packet_t *pckt,
                                    const uint8_t packet_id) {
  const uint8_t *temp = (uint8_t *)pckt;
  const uint8_t *comp =
      (packet_id) ? (uint8_t *)&retx_packet : (uint8_t *)&ack_packet;

  for (uint8_t i = 0; i < PACKET_TOT_LEN; i++) {
    if (*temp++ != *comp++) {
      return false;
    }
  }
  return true;
}

static void comms_memcopy(const comms_packet_t *src, comms_packet_t *dst) {
  uint8_t *tmp_src = (uint8_t *)src;
  uint8_t *tmp_dst = (uint8_t *)dst;

  for (uint8_t i = 0; i < PACKET_TOT_LEN; i++) {
    *tmp_dst++ = *tmp_src++;
  }
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

      if (comms_is_ackretx_packet(&temp_packet, 1)) {
        last_transmitted_packet.crc -= 1;
        comms_write(&last_transmitted_packet);
        state = CommsState_Length;
        break;
      }

      if (comms_is_ackretx_packet(&temp_packet, 0)) {
        state = CommsState_Length;
        break;
      }

      comms_memcopy(&temp_packet, &packet_buffer[packet_write_index]);
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

bool comms_packets_available(void) {
  return packet_read_index != packet_write_index;
}

void comms_write(const comms_packet_t *packet) {
  uart_write((uint8_t *)packet, PACKET_TOT_LEN);
  comms_memcopy(packet, &last_transmitted_packet);
}

void comms_read(comms_packet_t *packet) {
  comms_memcopy(&packet_buffer[packet_read_index], packet);
  packet_read_index = (packet_read_index + 1) & packet_buffer_mask;
}
