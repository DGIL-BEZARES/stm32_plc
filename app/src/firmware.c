
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>

#include "shared/system.h"
#include "core/timer.h"

#define BOOTLOADER_SIZE (0x4000U)

static void vector_setup(void)
{
  SCB_VTOR = BOOTLOADER_SIZE;
}

static void gpio_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
}

int main(void)
{
  vector_setup();
  system_setup();
  gpio_setup();
  gpio_set(GPIOA, GPIO5);
  timer_setup();

  timer_pwm_set_duty_cycle(0);

  uint64_t start_time = system_get_ticks();
  uint8_t i = 0;

  while (1)
  {
    // if (system_get_ticks() - start_time >= 100)
    //{
    //   gpio_toggle(GPIOA, GPIO5);
    //   start_time = system_get_ticks();
    // }

    if (system_get_ticks() - start_time >= 10)
    {
      if (i >= 100)
      {
        i = 0;
      }
      timer_pwm_set_duty_cycle(i++);
      start_time = system_get_ticks();
    }
  }

  return 0;
}
