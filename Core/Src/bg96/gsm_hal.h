#ifndef GSM_HAL_H
#define GSM_HAL_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void gsm_hal_gpio_init(void);
void gsm_hal_pwrkey_pin_set_high(void);
void gsm_hal_pwrkey_pin_set_low(void);
void gsm_hal_reset_pin_set_high(void);
void gsm_hal_reset_pin_set_low(void);

int gsm_hal_serial_init(int baudrate);
int gsm_hal_serial_write(const uint8_t* data, uint16_t len);
int gsm_hal_serial_read(uint8_t* data, uint16_t len);
void gsm_hal_serial_deinit(void);

#ifdef __cplusplus
}
#endif
#endif // GSM_HAL_H