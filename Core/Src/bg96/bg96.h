#ifndef BG96_H
#define BG96_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "modem_serial.h"

struct GsmStream;

typedef struct
{
    int (*open)(struct GsmStream* self, int baudrate);
    int (*write)(struct GsmStream* self, const uint8_t* data, uint16_t len);
    int (*read)(struct GsmStream* self, uint8_t* data, uint16_t len);
    int (*close)(struct GsmStream* self);
}GsmStreamVtable;

typedef struct GsmStream
{
    void* context;
    GsmStreamVtable* vtable;
}GsmStream;

// Result for AT+CREG?
typedef struct
{
    bool response_found;
    bool ok_found;
} Bg96CregStatus;



typedef struct
{
    ModemSerial* serial; // Pointer to the serial interface for communication

    struct
    {
        GsmStream* s;
        uint8_t size;
    }streams;
    

} Bg96;


int bg96_init(Bg96* module);
void bg96_power_on(Bg96* module);
int bg96_send_at(Bg96* module, const char* command, uint32_t timeout_ms);


// Query network registration via AT+CREG?; returns 0 on success and fills
// `status` (if non-NULL). Returns negative on error or timeout.
int bg96_query_creg(Bg96* module);



#ifdef __cplusplus
}
#endif
#endif // BG96_H