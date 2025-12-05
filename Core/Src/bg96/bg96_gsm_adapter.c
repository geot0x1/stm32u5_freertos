#include "bg96.h"
#include "gsm.h"

// Adapter vtable that maps GsmDriver calls to BG96 functions
static int bg96_gsm_init(GsmDriver* drv, void* ctx)
{
    Bg96* module = (Bg96*)ctx;
    if (!module) return -1;
    drv->ctx = module;
    return bg96_init(module);
}

static int bg96_gsm_power_on(GsmDriver* drv)
{
    Bg96* module = (Bg96*)drv->ctx;
    if (!module) return -1;
    bg96_power_on(module);
    return 0;
}

static int bg96_gsm_power_off(GsmDriver* drv)
{
    // No bg96_power_off implemented at the moment - placeholder
    (void)drv;
    return 0;
}

static int bg96_gsm_send_at(GsmDriver* drv, const char* cmd, char* resp, size_t resp_len, uint32_t timeout_ms)
{
    Bg96* module = (Bg96*)drv->ctx;
    if (!module) return -1;
    return bg96_send_at(module, cmd, resp, resp_len, timeout_ms);
}

static GsmDriverVtable bg96_vtable = {
    .init = bg96_gsm_init,
    .power_on = bg96_gsm_power_on,
    .power_off = bg96_gsm_power_off,
    .send_at = bg96_gsm_send_at
};

// Initialize a GsmDriver structure to use a BG96 instance
void gsm_driver_init_for_bg96(GsmDriver* drv, Bg96* module)
{
    if (!drv) return;
    drv->ctx = module;
    drv->vtable = &bg96_vtable;
}
