#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/rcc.h>

#include "bl-flash.h"
#include "comms.h"
#include "core/simple-timer.h"
#include "core/system.h"
#include "core/uart.h"

#define BOOTLOADER_SIZE (0x4000U) // 16KB
#define APP_START_ADDR (FLASH_BASE + BOOTLOADER_SIZE)
#define MAX_FW_LENGTH ((1024U * 128U) - BOOTLOADER_SIZE)

#define DEVICE_ID (0X42)

#define SYNC_SEQ0 (0XC4)
#define SYNC_SEQ1 (0X55)
#define SYNC_SEQ2 (0X7E)
#define SYNC_SEQ3 (0X10)

#define DEFAULT_TIMEOUT (5000)

typedef enum bl_state_t {
  BL_State_Sync,
  BL_State_WaitForUpdateReq,
  BL_State_DeviceIdReq,
  BL_State_DeviceIdRes,
  BL_State_FWLengthReq,
  BL_State_FWLengthRes,
  BL_State_EraseApp,
  BL_State_ReceiveFW,
  BL_State_Done,
} bl_state_t;

static bl_state_t state = BL_State_Sync;
static uint32_t fw_length = 0;
static uint32_t bytes_written = 0;
static uint8_t sync_seq[4] = {0};

static simple_timer_t timer;
static comms_packet_t temp_packet;
static volatile uint32_t addr[128] = {0};
static uint8_t idx = 0;

static void gpio_setup(void) {
  rcc_periph_clock_enable(RCC_GPIOA);

  gpio_set_mode(USART_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
  gpio_set_mode(USART_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                GPIO_USART2_RX);
}

static void gpio_teardown(void) {
  gpio_set_mode(USART_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG,
                GPIO_USART2_TX | GPIO_USART2_RX);
  rcc_periph_clock_disable(RCC_GPIOA);
}

static void jump_to_app(void) {
  vector_table_t* main_vector_table = (vector_table_t*)APP_START_ADDR;
  main_vector_table->reset();
}

static void bootloader_failure(void) {
  comms_create_single_byte_packet(&temp_packet, BL_PACKET_NACK_DATA0);
  comms_write(&temp_packet);
  state = BL_State_Done;
}

static void check_for_timeout(void) {
  if (simple_timer_has_elapsed(&timer)) {
    bootloader_failure();
  }
}

static bool is_device_id_packet(const comms_packet_t *packet) {
  if (packet->length != 2) {
    return false;
  }

  if (packet->data[0] != BL_PACKET_DEV_ID_RES_DATA0) {
    return false;
  }

  for (uint8_t i = 2; i < PACKET_DATA_LEN; i++) {
    if (packet->data[i] != 0xff) {
      return false;
    }
  }

  return true;
}

static bool is_fw_length_packet(const comms_packet_t *packet) {
  if (packet->length != 5) {
    return false;
  }

  if (packet->data[0] != BL_PACKET_FW_LEN_RES_DATA0) {
    return false;
  }

  for (uint8_t i = 5; i < PACKET_DATA_LEN; i++) {
    if (packet->data[i] != 0xff) {
      return false;
    }
  }

  return true;
}
int main(void) {
  system_setup();
  gpio_setup();
  uart_setup();

  simple_timer_setup(&timer, DEFAULT_TIMEOUT, false);

  while (state != BL_State_Done) {
    if (state == BL_State_Sync) {
      if (uart_data_available()) {
        sync_seq[0] = sync_seq[1];
        sync_seq[1] = sync_seq[2];
        sync_seq[2] = sync_seq[3];
        sync_seq[3] = uart_read_byte();

        bool is_match = sync_seq[0] == SYNC_SEQ0;
        is_match = is_match && (sync_seq[1] == SYNC_SEQ1);
        is_match = is_match && (sync_seq[2] == SYNC_SEQ2);
        is_match = is_match && (sync_seq[3] == SYNC_SEQ3);

        if (is_match) {
          comms_create_single_byte_packet(&temp_packet, BL_PACKET_SYNC_OBS_DATA0);
          comms_write(&temp_packet);
          simple_timer_reset(&timer);
          state = BL_State_WaitForUpdateReq;
        } else {
          check_for_timeout();
        }
      } else {
        check_for_timeout();
      }
      continue;
    }

    comms_update();

    switch (state) {
    case BL_State_WaitForUpdateReq: {
      if (comms_packets_available()) {
        comms_read(&temp_packet);

        if (comms_is_single_byte_packet(&temp_packet, BL_PACKET_FW_UP_REQ_DATA0)) {
          simple_timer_reset(&timer);
          comms_create_single_byte_packet(&temp_packet, BL_PACKET_FW_UP_RES_DATA0);
          comms_write(&temp_packet);
          state = BL_State_DeviceIdReq;

        } else {
          bootloader_failure();
        }

      } else {
        check_for_timeout();
      }

    } break;

    case BL_State_DeviceIdReq: {
      simple_timer_reset(&timer);
      comms_create_single_byte_packet(&temp_packet, BL_PACKET_DEV_ID_REQ_DATA0);
      comms_write(&temp_packet);
      state = BL_State_DeviceIdRes;

    } break;
    case BL_State_DeviceIdRes: {
      if (comms_packets_available()) {
        comms_read(&temp_packet);

        if (is_device_id_packet(&temp_packet) && temp_packet.data[1] == DEVICE_ID) {
          simple_timer_reset(&timer);
          state = BL_State_FWLengthReq;
        } else {
          bootloader_failure();
        }
      } else {
        check_for_timeout();
      }

    } break;
    case BL_State_FWLengthReq: {
      simple_timer_reset(&timer);
      comms_create_single_byte_packet(&temp_packet, BL_PACKET_FW_LEN_REQ_DATA0);
      comms_write(&temp_packet);
      state = BL_State_FWLengthRes;

    } break;
    case BL_State_FWLengthRes: {
      if (comms_packets_available()) {
        comms_read(&temp_packet);

        fw_length = ((temp_packet.data[1]) | (temp_packet.data[2] << 8) |
                     (temp_packet.data[3] << 16) | (temp_packet.data[4] << 24));

        if (is_fw_length_packet(&temp_packet) && (fw_length <= MAX_FW_LENGTH)) {
          simple_timer_reset(&timer);
          state = BL_State_EraseApp;
        } else {
          bootloader_failure();
        }
      } else {
        check_for_timeout();
      }

    } break;
    case BL_State_EraseApp: {
      bl_flash_erase_main_application();

      comms_create_single_byte_packet(&temp_packet, BL_PACKET_RDY_DATA_DATA0);
      comms_write(&temp_packet);
      simple_timer_reset(&timer);

      state = BL_State_ReceiveFW;

    } break;
    case BL_State_ReceiveFW: {
      if (comms_packets_available()) {
        comms_read(&temp_packet);

        const uint8_t packet_length = (temp_packet.length & 0x1f) + 1;

        addr[idx++] = APP_START_ADDR + bytes_written;

        bl_flash_write(APP_START_ADDR + bytes_written, temp_packet.data,
                       packet_length);
        bytes_written += packet_length;
        simple_timer_reset(&timer);

        if (bytes_written >= fw_length) {
          comms_create_single_byte_packet(&temp_packet, BL_PACKET_FW_UP_SCS_DATA0);
          comms_write(&temp_packet);
          state = BL_State_Done;
        } else {
          comms_create_single_byte_packet(&temp_packet, BL_PACKET_RDY_DATA_DATA0);
          comms_write(&temp_packet);
        }

      } else {
        check_for_timeout();
      }
    } break;

    default: {
      state = BL_State_Sync;
    } break;
    }
  }

  system_delay(150);
  uart_teardown();
  gpio_teardown();
  system_teardown();

  jump_to_app();

  return 0;
}
