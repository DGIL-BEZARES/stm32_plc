
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include "core/system.h"
#include "core/timer.h"

static void gpio_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
}

int main(void)
{
  system_setup();
  gpio_setup();
  timer_setup();

  timer_pwm_set_duty_cycle(0);

  uint64_t start_time = system_get_ticks();

  while (1)
  {
    // if (system_get_ticks() - start_time >= 100)
    //{
    //   gpio_toggle(GPIOA, GPIO5);
    //   start_time = system_get_ticks();
    // }

    for (float i = 0; i <= 100; i++)
    {
      timer_pwm_set_duty_cycle(i);
      while (system_get_ticks() - start_time < 10)
      {
      }
      start_time = system_get_ticks();
    }
    for (float i = 100; i >= 0; i--)
    {
      timer_pwm_set_duty_cycle(i);
      while (system_get_ticks() - start_time < 10)
      {
      }
      start_time = system_get_ticks();
    }
  }

  return 0;
}
