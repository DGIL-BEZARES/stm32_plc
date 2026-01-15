#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/rcc.h>

#include "comms.h"
#include "core/crc8.h"
#include "core/system.h"
#include "core/uart.h"

#define BOOTLOADER_SIZE (0x4000U) // 16KB
#define APP_START_ADDR (FLASH_BASE + BOOTLOADER_SIZE)

static void gpio_setup(void) {
  rcc_periph_clock_enable(RCC_GPIOA);

  gpio_set_mode(USART_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
  gpio_set_mode(USART_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                GPIO_USART2_RX);
}

static void jump_to_app(void) {
  typedef void (*pFunction)(void);

  uint32_t *reset_vector_entry = (uint32_t *)(APP_START_ADDR + 4U);
  uint32_t *reset_vector = (uint32_t *)(*reset_vector_entry);

  pFunction jump_fn = (pFunction)reset_vector;

  jump_fn();
}

int main(void) {
  system_setup();
  gpio_setup();
  uart_setup();

  comms_packet_t packet = {
      .length = 9,
      .data = {1, 2, 3, 4, 5, 6, 7, 8, 9, [9 ... 31] = 0xff},
      .crc = 0};

  packet.crc = crc8_h2f((uint8_t *)&packet, PACKET_TOT_LEN - PACKET_CRC_LEN);
  packet.crc += 1;

  comms_write(&packet);

  while (true) {
    comms_update();
    system_delay(500);
  }

  jump_to_app();

  return 0;
}
