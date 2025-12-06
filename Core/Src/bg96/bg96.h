#ifndef BG96_H
#define BG96_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "modem_serial.h"

#define EINVAL    -1
#define ETIMEDOUT -2
#define EIO       -3
#define ENODATA   -4

#define AT_OK      0
#define AT_WAITING 1


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

typedef enum
{
    BG96_NETWORK_REG_NOT_REGISTERED = 0,
    BG96_NETWORK_REG_REGISTERED_HOME = 1,
    BG96_NETWORK_REG_SEARCHING = 2,
    BG96_NETWORK_REG_DENIED = 3,
    BG96_NETWORK_REG_UNKNOWN = 4,
    BG96_NETWORK_REG_REGISTERED_ROAMING = 5
} Bg96NetworkRegistrationStatus;


// Result for AT+CREG?
typedef struct
{
    bool response_found;
    bool ok_found;
    Bg96NetworkRegistrationStatus n; // network registration status
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

// High-level AT status common to all AT commands
typedef enum
{
    BG96_AT_STATUS_SUCCESS = 0, // final OK received
    BG96_AT_STATUS_FAIL = 1,    // final ERROR received
    BG96_AT_STATUS_TIMEOUT = 2  // no final result within timeout
} Bg96AtStatus;


typedef struct
{
    union
    {
        Bg96CregStatus creg; // result for AT+CREG?
    } detail;
} Bg96AtResult;

// Send AT command and obtain a structured result. Returns a common status
// (success/fail/timeout). Detailed payload, if any, is placed into `result`.
Bg96AtStatus bg96_send_at_result(Bg96* module, const char* command, Bg96AtResult* result, uint32_t timeout_ms);


// Query network registration via AT+CREG?; returns 0 on success and fills
// `status` (if non-NULL). Returns negative on error or timeout.
// Query network registration via AT+CREG?; fills `status` (must be provided).
// Returns 0 on success, negative on error or timeout.
int bg96_query_creg(Bg96* module, Bg96CregStatus* status);


int bg96_get_network_registration(Bg96* module, Bg96AtResult* result, uint32_t timeout_ms);



#ifdef __cplusplus
}
#endif
#endif // BG96_H