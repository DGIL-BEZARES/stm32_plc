#include "timer.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>

#define PSC (48)
#define ARR_VAL (1000)

#define LED_PORT (GPIOA)
#define LED_PIN (GPIO5)

void timer_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    // Setup OC in No Output Mode
    timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_FROZEN);

    // Enable timer OC output
    timer_enable_oc_output(TIM2, TIM_OC1);

    timer_set_prescaler(TIM2, PSC - 1);
    timer_set_period(TIM2, ARR_VAL - 1);

    nvic_enable_irq(NVIC_TIM2_IRQ);
    nvic_set_priority(NVIC_TIM2_IRQ, 1);

    timer_enable_irq(TIM2, TIM_DIER_CC1IE | TIM_DIER_UIE);

    timer_enable_counter(TIM2);
}

void tim2_isr(void)
{
    if (timer_get_flag(TIM2, TIM_SR_UIF))
    {
        gpio_set(LED_PORT, LED_PIN);
        timer_clear_flag(TIM2, TIM_SR_UIF);
    }
    if (timer_get_flag(TIM2, TIM_SR_CC1IF))
    {
        gpio_clear(LED_PORT, LED_PIN);
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
    }
}

void timer_pwm_set_duty_cycle(float dt)
{
    const float ccr_val = (float)ARR_VAL * (dt / 100.0f);
    timer_set_oc_value(TIM2, TIM_OC1, ccr_val);
}