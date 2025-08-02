#ifndef MODEM_SERIAL_H
#define MODEM_SERIAL_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct ModemSerial
{
    void* context;
    int (*open)(struct ModemSerial* self, int baudrate);
    int (*write)(struct ModemSerial* self, const uint8_t* data, uint16_t len);
    int (*read)(struct ModemSerial* self, uint8_t* data, uint16_t len);
} ModemSerial;

int modem_serial_open(ModemSerial* self, int baudrate);
int modem_serial_write(ModemSerial* self, const uint8_t* data, uint16_t len);
int modem_serial_read(ModemSerial* self, uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif
