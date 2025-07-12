#ifndef DRV8825_H
#define DRV8825_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include <stdint.h>

typedef struct
{
    GPIO_TypeDef *step_port;
    uint16_t step_pin;

    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;

    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;

} Drv8825;

void motor_init(Drv8825* drv);
void motor_enable(Drv8825* drv);
void motor_disable(Drv8825* drv);
void motor_steps(Drv8825* drv, uint32_t steps);
void motor_direction_set(Drv8825* drv, uint8_t direction);

#ifdef __cplusplus
}
#endif
#endif // DRV8825_H