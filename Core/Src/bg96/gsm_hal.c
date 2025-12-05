#include "gsm_hal.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_bus.h"
#include "stm32u545xx.h"

typedef struct GsmHalPinConfig
{
    GPIO_TypeDef* port;
    uint32_t pin;
} GsmHalPinConfig;


static const GsmHalPinConfig GSM_GPIO_RESET_PIN = {
    .port = GPIOB,
    .pin = LL_GPIO_PIN_4
};

static const GsmHalPinConfig GSM_GPIO_PWRKEY_PIN = {
    .port = GPIOB,
    .pin = LL_GPIO_PIN_10
};

static void gsm_hal_set_pin_output(const GsmHalPinConfig* pin_config);
static void gsm_hal_set_pin_low(const GsmHalPinConfig* pin_config);
static void gsm_hal_set_pin_high(const GsmHalPinConfig* pin_config);



void gsm_hal_gpio_init(void)
{
    gsm_hal_set_pin_output(&GSM_GPIO_RESET_PIN);
    gsm_hal_set_pin_output(&GSM_GPIO_PWRKEY_PIN);
}

void gsm_hal_pwrkey_pin_set_high(void)
{
    gsm_hal_set_pin_high(&GSM_GPIO_PWRKEY_PIN);
}

void gsm_hal_pwrkey_pin_set_low(void)
{
    gsm_hal_set_pin_low(&GSM_GPIO_PWRKEY_PIN);
}

void gsm_hal_reset_pin_set_high(void)
{
    gsm_hal_set_pin_high(&GSM_GPIO_RESET_PIN);
}

void gsm_hal_reset_pin_set_low(void)
{
    gsm_hal_set_pin_low(&GSM_GPIO_RESET_PIN);
}



static void gsm_hal_set_pin_low(const GsmHalPinConfig* pin_config)
{
    if (!pin_config)
    {
        return;
    }
    if (pin_config->port == NULL)
    {
        return;
    }
    LL_GPIO_ResetOutputPin(pin_config->port, pin_config->pin);
}

static void gsm_hal_set_pin_high(const GsmHalPinConfig* pin_config)
{
    if (!pin_config)
    {
        return;
    }
    if (pin_config->port == NULL)
    {
        return;
    }
    LL_GPIO_SetOutputPin(pin_config->port, pin_config->pin);
}

static void gsm_hal_set_pin_output(const GsmHalPinConfig* pin_config)
{
    if (!pin_config)
    {
        return;
    }
    if (pin_config->port == NULL)
    {
        return;
    }
    if (pin_config->port == GPIOB)
    {
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    }
    else if (pin_config->port == GPIOA)
    {
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    }
    else if (pin_config->port == GPIOC)
    {
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    }

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = pin_config->pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(pin_config->port, &GPIO_InitStruct);
}