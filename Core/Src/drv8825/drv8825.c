#include "drv8825.h"
#include "FreeRTOS.h"
#include "task.h"

static void DWT_Delay_Init(void);
__STATIC_INLINE void DWT_Delay_us(uint32_t microseconds);

static void one_step(Drv8825* drv);
static void enable_gpio_clock(GPIO_TypeDef* port);

void motor_init(Drv8825* drv)
{
    DWT_Delay_Init();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    enable_gpio_clock(drv->step_port);
    enable_gpio_clock(drv->dir_port);
    enable_gpio_clock(drv->enable_port);

    GPIO_InitStruct.Pin = drv->dir_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(drv->dir_port , &GPIO_InitStruct);

    GPIO_InitStruct.Pin = drv->step_pin;
    HAL_GPIO_Init(drv->step_port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = drv->enable_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(drv->enable_port, &GPIO_InitStruct);
}

void motor_enable(Drv8825* drv)
{
    HAL_GPIO_WritePin(drv->enable_port, drv->enable_pin, GPIO_PIN_RESET); // Enable the motor driver
}

void motor_disable(Drv8825* drv)
{
    HAL_GPIO_WritePin(drv->enable_port, drv->enable_pin, GPIO_PIN_SET); // Disable the motor driver
}

void motor_steps(Drv8825* drv, uint32_t steps)
{
    for (uint32_t i = 0; i < steps; i++)
    {
        one_step(drv);
        //! Allow motor to settle
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void motor_direction_set(Drv8825* drv, uint8_t direction)
{
}

__STATIC_INLINE void DWT_Delay_us(uint32_t microseconds)
{
    // Check if DWT is enabled, if not — enable it
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
    {
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t clk_cycle_start = DWT->CYCCNT;
    uint32_t delay_cycles = (HAL_RCC_GetHCLKFreq() / 1000000) * microseconds;

    while ((DWT->CYCCNT - clk_cycle_start) < delay_cycles)
        ;
}

static void one_step(Drv8825* drv)
{
    HAL_GPIO_WritePin(drv->step_port, drv->step_pin, GPIO_PIN_SET);
    DWT_Delay_us(2);
    HAL_GPIO_WritePin(drv->step_port, drv->step_pin, GPIO_PIN_RESET);
    DWT_Delay_us(2);
}

static void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void enable_gpio_clock(GPIO_TypeDef* port)
{
    if (port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
#ifdef GPIOE
    else if (port == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
#endif
#ifdef GPIOF
    else if (port == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
#endif
#ifdef GPIOG
    else if (port == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
#endif
#ifdef GPIOH
    else if (port == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
#endif
#ifdef GPIOI
    else if (port == GPIOI)
    {
        __HAL_RCC_GPIOI_CLK_ENABLE();
    }
#endif

}