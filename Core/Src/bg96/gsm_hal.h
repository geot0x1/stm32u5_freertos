#ifndef GSM_HAL_H
#define GSM_HAL_H
#ifdef __cplusplus
extern "C" {
#endif

void gsm_hal_gpio_init(void);
void gsm_hal_pwrkey_pin_set_high(void);
void gsm_hal_pwrkey_pin_set_low(void);
void gsm_hal_reset_pin_set_high(void);
void gsm_hal_reset_pin_set_low(void);

#ifdef __cplusplus
}
#endif
#endif // GSM_HAL_H