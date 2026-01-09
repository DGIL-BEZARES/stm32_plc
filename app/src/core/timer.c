#include "core/timer.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>

#define PSC (48)
#define ARR_VAL (1000)

void timer_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    // Setup OC in No Output Mode
    timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_FROZEN);

    // Enable timer OC output
    // timer_enable_oc_output(TIM2, TIM_OC1);

    timer_set_prescaler(TIM2, PSC - 1);
    timer_set_period(TIM2, ARR_VAL - 1);

    nvic_enable_irq(NVIC_TIM2_IRQ);
    nvic_set_priority(NVIC_TIM2_IRQ, 1);

    timer_enable_irq(TIM2, TIM_DIER_UIE | TIM_DIER_CC1IE);

    timer_enable_counter(TIM2);
}

void tim2_isr(void)
{
    if (TIM2_SR & TIM_SR_CC1IF)
    {
        TIM2_SR &= ~TIM_SR_CC1IF;
        gpio_set(GPIOA, GPIO5);
        /* code */
    }
    else
    {
        TIM2_SR &= ~TIM_SR_UIF;
        gpio_clear(GPIOA, GPIO5);
    }
}

void timer_pwm_set_duty_cycle(float dt)
{
    const float ccr_val = (float)ARR_VAL * (dt / 100.0f);
    timer_set_oc_value(TIM2, TIM_OC1, ccr_val);
}