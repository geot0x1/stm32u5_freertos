#include "gsm.h"

int gsm_init(GsmDriver* drv, void* ctx)
{
    if (!drv || !drv->vtable || !drv->vtable->init)
        return -1;
    return drv->vtable->init(drv, ctx);
}

int gsm_power_on(GsmDriver* drv)
{
    if (!drv || !drv->vtable || !drv->vtable->power_on)
        return -1;
    return drv->vtable->power_on(drv);
}

int gsm_power_off(GsmDriver* drv)
{
    if (!drv || !drv->vtable || !drv->vtable->power_off)
        return -1;
    return drv->vtable->power_off(drv);
}

int gsm_send_at(GsmDriver* drv, const char* cmd, char* resp, size_t resp_len, uint32_t timeout_ms)
{
    if (!drv || !drv->vtable || !drv->vtable->send_at)
        return -1;
    return drv->vtable->send_at(drv, cmd, resp, resp_len, timeout_ms);
}
