#ifndef BG96_H
#define BG96_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "modem_serial.h"

struct CellularStream;

typedef struct
{
    int (*open)(struct CellularStream* self, int baudrate);
    int (*write)(struct CellularStream* self, const uint8_t* data, uint16_t len);
    int (*read)(struct CellularStream* self, uint8_t* data, uint16_t len);
    int (*close)(struct CellularStream* self);
}CellularStreamVtable;

typedef struct CellularStream
{
    void* context;
    CellularStreamVtable* vtable;
}CellularStream;




typedef struct
{
    ModemSerial* serial; // Pointer to the serial interface for communication

    struct
    {
        CellularStream* s;
        uint8_t size;
    }streams;
    

} Bg96;


int bg96_init(Bg96* module);
void bg96_power_on(Bg96* module);


#ifdef __cplusplus
}
#endif
#endif // BG96_H