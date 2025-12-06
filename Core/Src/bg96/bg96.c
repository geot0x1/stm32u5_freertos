#include "bg96.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_bus.h"
#include "stm32u545xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "gsm_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>



typedef struct
{
    char response[256];
    size_t length;
    void* context;
    // response_handler should return non-zero when it has fully processed the
    // response and the caller should stop waiting for more data.
    int (*response_handler)(void* ctx, const char* response, size_t length);
} AtHandler;




static int native_modem_serial_init(GsmStream* self, int baudrate)
{
    return gsm_hal_serial_init(baudrate);
}

static int native_modem_serial_write(GsmStream* self, const uint8_t* data, uint16_t len)
{
    return gsm_hal_serial_write(data, len);
}

static int native_modem_serial_read(GsmStream* self, uint8_t* data, uint16_t len)
{
    return gsm_hal_serial_read(data, len);
}

static int native_modem_serial_close(GsmStream* self)
{
    // If there's no close function, we can just return 0
    return 0;
}


GsmStreamVtable native_stream_vtable = {
    .open = native_modem_serial_init,
    .write = native_modem_serial_write,
    .read = native_modem_serial_read,
    .close = native_modem_serial_close
};

GsmStream native_stream = {
    .context = NULL,
    .vtable = &native_stream_vtable
};

static void at_handler_clear(AtHandler* handler)
{
    handler->length = 0;
    handler->response[0] = '\0';
}

static int at_handler_append_char(AtHandler* handler, char c)
{
    if (handler->length < sizeof(handler->response) - 1)
    {
        handler->response[handler->length++] = c;
        handler->response[handler->length] = '\0'; // Null-terminate
        return 0;
    }
    return -1; // Buffer full
}

int send_at_command(GsmStream* stream, const char* command, AtHandler* handler, uint32_t timeout)
{
    if (!stream || !stream->vtable || !stream->vtable->write || !stream->vtable->read || !handler || !command)
    {
        return -1;
    }

    at_handler_clear(handler);
    // Send the command
    printf("Sending: %s", command);
    stream->vtable->write(stream, (const uint8_t*)command, (uint16_t)strlen(command));

    // Read loop: poll for response until timeout (timeout in ms)
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout)
    {
        uint8_t ch;
        int n = stream->vtable->read(stream, &ch, 1);
        if (n > 0)
        {
            if (at_handler_append_char(handler, (char)ch) < 0)
            {
                at_handler_clear(handler);
            }
            if (handler->length >= 2 && 
                handler->response[handler->length - 2] == '\r' &&
                handler->response[handler->length - 1] == '\n')
            {
                printf("Response received: %s", handler->response);
                if (handler->response_handler)
                {
                    size_t resp_len = handler->length;
                    int handled = handler->response_handler(handler->context, handler->response, resp_len);
                    at_handler_clear(handler);
                    if (handled)
                    {
                        // Handler indicates message processed — exit read loop early
                        return (int)resp_len;
                    }
                }
                else
                {
                    at_handler_clear(handler);
                }
            }

        }
        else
        {
            // No data available — small delay
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    // Timeout — return with whatever we have
    return (int)handler->length;
}

static int parse_simple_at(void* ctx, const char* response, size_t length)
{
    // Simple parser example: just print the response
    printf("AT Response (%d bytes): %s\n", (int)length, response);
    if (strstr(response, "OK"))
    {
        printf("Command succeeded.\n");
        return 1; // processed, caller can stop waiting
    }
    else if (strstr(response, "ERROR"))
    {
        printf("Command failed.\n");
        return 1; // processed, caller can stop waiting
    }
    return 0; // not a terminal response yet
}

// Helper: trim leading spaces
static const char* skip_spaces(const char* s)
{
    while (*s == ' ' || *s == '\t') s++;
    return s;
}

// Helper: copy a token from src (stops at comma or end) into dst, strip quotes
static size_t copy_token(char* dst, size_t dst_len, const char* src)
{
    const char* p = src;
    // skip leading spaces
    while ((*p == ' ' || *p == '\t') && *p) p++;

    // optional leading quote
    if (*p == '"') p++;

    size_t i = 0;
    while (*p && *p != ',' && *p != '\r' && *p != '\n' && i + 1 < dst_len)
    {
        if (*p == '"') break; // end quote
        dst[i++] = *p++;
    }
    dst[i] = '\0';
    return i;
}

// Parse +CREG: lines. This handler follows the AtHandler response_handler signature.
static int creg_response_handler(void* ctx, const char* response, size_t length)
{
    if (!ctx || !response) return 0;

    Bg96CregStatus* st = (Bg96CregStatus*)ctx;
    const char* prefix = "+CREG:";
    if (strncmp(response, prefix, strlen(prefix)) == 0)
    {
        st->response_found = true;
        // Further parsing can be done here if needed
        return 0; // not fully processed yet
    }
    else if (strstr(response, "OK"))
    {
        st->ok_found = true;
        return 1; // fully processed
    }
    return 0;
}

int bg96_query_creg(Bg96* module)
{
    AtHandler handler;
    Bg96CregStatus creg_status = {0};
    handler.response_handler = creg_response_handler;
    handler.context = &creg_status;
    handler.length = 0;
    memset(handler.response, 0, sizeof(handler.response));

    // send command; include CR
    const char* cmd = "AT+CREG?\r";
    int r = send_at_command(&native_stream, cmd, &handler, 3000);
    if (r <= 0)
    {
        return -1; // timeout or no data
    }
    // success: status filled by handler
    return 0;
}

int bg96_send_at(Bg96* module, const char* command, uint32_t timeout_ms)
{
    AtHandler handler;
    handler.response_handler = parse_simple_at;
    handler.context = NULL;
    handler.length = 0;
    memset(handler.response, 0, sizeof(handler.response));

    return send_at_command(&native_stream, command, &handler, timeout_ms);
}


// void send_at_command(GsmStream* stream, const char* command, AtHandler* handler, uint32_t timeout)
// {

//     if (!stream || !stream->vtable || !stream->vtable->write || !stream->vtable->read || !handler || !command)
//     {
//         return;
//     }

//     // Clear handler
//     handler->length = 0;
//     handler->response[0] = '\0';

//     // Ensure command ends with CR
//     char cmdbuf[256];
//     size_t cmdlen = strlen(command);
//     if (cmdlen + 2 >= sizeof(cmdbuf))
//     {
//         return; // command too long
//     }
//     strcpy(cmdbuf, command);
//     if (cmdlen == 0 || cmdbuf[cmdlen - 1] != '\r')
//     {
//         cmdbuf[cmdlen++] = '\r';
//         cmdbuf[cmdlen] = '\0';
//     }

//     // Send the command
//     stream->vtable->write(stream, (const uint8_t*)cmdbuf, (uint16_t)cmdlen);

//     // Read loop: poll for response until timeout (timeout in ms)
//     TickType_t start = xTaskGetTickCount();
//     TickType_t timeout_ticks = pdMS_TO_TICKS(timeout);

//     while ((xTaskGetTickCount() - start) < timeout_ticks)
//     {
//         uint8_t ch;
//         int n = stream->vtable->read(stream, &ch, 1);
//         if (n > 0)
//         {
//             if (handler->length < sizeof(handler->response) - 1)
//             {
//                 handler->response[handler->length++] = (char)ch;
//                 handler->response[handler->length] = '\0';
//             }

//             // Simple terminal checks for final response
//             if (strstr(handler->response, "\r\nOK\r\n") || strstr(handler->response, "\r\nERROR\r\n"))
//             {
//                 return;
//             }
//         }
//         else
//         {
//             // No data available — small delay
//             vTaskDelay(pdMS_TO_TICKS(10));
//         }
//     }

//     // Timeout — just return with whatever we have
// }

void bg96_reset_pin_init(void)
{
    // 1. Enable GPIOB peripheral clock
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    // 2. Configure PB4 as output, push-pull, no pull-up/down, low speed
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


void bg96_pwrkey_pin_init(void)
{
    // 1. Enable GPIOB clock
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    // 2. Set up PB10 as output, push-pull, no pull-up/down, low speed
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


void bg96_reset_pin_on(void)
{
    gsm_hal_reset_pin_set_high();
}

void bg96_reset_pin_off(void)
{
    gsm_hal_reset_pin_set_low();
}

void bg96_pwrkey_pin_on(void)
{
    gsm_hal_pwrkey_pin_set_high();
}

void bg96_pwrkey_pin_off(void)
{
    gsm_hal_pwrkey_pin_set_low();
}

int bg96_init(Bg96* module)
{
    gsm_hal_gpio_init();

    // Set initial states
    gsm_hal_reset_pin_set_low(); // Reset pin low
    gsm_hal_pwrkey_pin_set_high(); // Power key pin high

    gsm_hal_serial_init(115200);
    return 0;
}

void bg96_power_on(Bg96* module)
{
    bg96_pwrkey_pin_on(); // Toggle power key (board-specific polarity)
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
    bg96_pwrkey_pin_off(); // Set power key high to complete power on
}


// int bg96_send_at(Bg96* module, const char* command, char* resp_buf, size_t buf_len, uint32_t timeout_ms)
// {
//     if (!module || !module->streams.s)
//     {
//         return -1;
//     }

//     AtHandler handler;
//     handler.length = 0;
//     memset(handler.response, 0, sizeof(handler.response));

//     send_at_command(module->streams.s, command, &handler, timeout_ms);

//     if (resp_buf && buf_len)
//     {
//         size_t copy_len = handler.length < (buf_len - 1) ? handler.length : (buf_len - 1);
//         if (copy_len > 0)
//         {
//             memcpy(resp_buf, handler.response, copy_len);
//         }
//         resp_buf[copy_len] = '\0';
//     }

//     return (int)handler.length;
// }


