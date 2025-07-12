#include "bg96.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_bus.h"
#include "stm32u545xx.h"

#include "FreeRTOS.h"
#include "task.h"


void bg96_reset_pin_init(void)
{
    // 1. Enable GPIOB peripheral clock
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    // 2. Configure PB4 as output, push-pull, no pull-up/down, low speed
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void bg96_reset_pin_set_high(void)
{
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
}

void bg96_reset_pin_set_low(void)
{
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
}

void bg96_pwrkey_pin_init(void)
{
    // 1. Enable GPIOB clock
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    // 2. Set up PB10 as output, push-pull, no pull-up/down, low speed
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void bg96_pwrkey_pin_set_high(void)
{
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);
}

void bg96_pwrkey_pin_set_low(void)
{
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);
}

void bg96_reset_pin_on(void)
{
    bg96_reset_pin_set_high();
}

void bg96_reset_pin_off(void)
{
    bg96_reset_pin_set_low();
}

void bg96_pwrkey_pin_on(void)
{
    bg96_pwrkey_pin_set_high();
}

void bg96_pwrkey_pin_off(void)
{
    bg96_pwrkey_pin_set_low();
}

void bg96_init(void)
{
    bg96_reset_pin_init();
    bg96_pwrkey_pin_init();

    // Set initial states
    bg96_reset_pin_set_low(); // Reset pin low
    bg96_pwrkey_pin_set_high(); // Power key pin high
}

void bg96_power_on(void)
{
    bg96_pwrkey_pin_on(); // Set power key low to turn on
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
    bg96_pwrkey_pin_off(); // Set power key high to complete power on
}