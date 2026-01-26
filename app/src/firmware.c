
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "core/system.h"
#include "core/uart.h"
#include "timer.h"

#define BOOTLOADER_SIZE (0x4000U)

#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

static void vector_setup(void) { SCB_VTOR = BOOTLOADER_SIZE; }

static void gpio_setup(void) {
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                LED_PIN);

  gpio_set_mode(USART_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
  gpio_set_mode(USART_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                GPIO_USART2_RX);
}

int main(void) {
  vector_setup();
  system_setup();
  gpio_setup();
  // uart_setup();

  gpio_set(LED_PORT, LED_PIN);
  timer_setup();

  timer_pwm_set_duty_cycle(0);

  uint64_t start_time = system_get_ticks();
  uint8_t i = 0;

  while (1) {
    // if (system_get_ticks() - start_time >= 1000)
    //{
    //   gpio_toggle(GPIOA, GPIO5);
    //   start_time = system_get_ticks();
    // }

    if (system_get_ticks() - start_time >= 10) {
      if (i >= 100) {
        i = 0;
      }
      timer_pwm_set_duty_cycle(i++);
      start_time = system_get_ticks();
    }
    //
    // if (uart_data_available()) {
    //  uint8_t data = uart_read_byte();
    //  uart_write_byte(data + 1);
    //}
    // Do useful work
  }

  return 0;
}
