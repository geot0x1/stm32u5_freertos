#include "modem_serial.h"

int modem_serial_open(ModemSerial* self, int baudrate)
{
    if (self && self->open)
    {
        return self->open(self, baudrate);
    }
    return -1;
}

int modem_serial_write(ModemSerial* self, const uint8_t* data, uint16_t len)
{
    if (self && self->write)
    {
        return self->write(self, data, len);
    }
    return -1;

}

int modem_serial_read(ModemSerial* self, uint8_t* data, uint16_t len)
{
    if (self && self->read)
    {
        return self->read(self, data, len);
    }
    return -1;
}