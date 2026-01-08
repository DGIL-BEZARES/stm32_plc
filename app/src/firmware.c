#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static void rcc_setup(void){
  rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSI_48MHZ]);
}

static void gpio_setup(void){
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
}

static void delay_cycles(uint32_t cycles){
  for (uint32_t i = 0; i < cycles; i++)
  {
    __asm__("nop");
  }
  
}

int main(void) {

  rcc_setup();
  gpio_setup();

  while (1) {

    gpio_toggle(GPIOA, GPIO5);	/* LED on/off */
		delay_cycles(48000000/4);

  }

  return 0;
}
