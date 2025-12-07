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


static void print_response(const char* response, size_t length);
static bool is_ok(const char* response);
static bool is_error(const char* response);



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
        return AT_ERR_BAD_INPUT;
    }

    at_handler_clear(handler);
    // Send the command
    printf("Sending: %s\r\n", command);
    stream->vtable->write(stream, (const uint8_t*)command, (uint16_t)strlen(command));

    // Read loop: poll for response until timeout (timeout in ms)
    uint32_t start = HAL_GetTick();

    int err = AT_ERR_TIMEOUT;
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
                print_response(handler->response, handler->length);
                
                if (handler->response_handler)
                {
                    size_t resp_len = handler->length;
                    err = handler->response_handler(handler->context, handler->response, resp_len);
                }
                at_handler_clear(handler);
                if (err <= AT_SUCCESS)
                {
                    return err;
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
    return AT_ERR_TIMEOUT;
}

static int parse_simple_at(void* ctx, const char* response, size_t length)
{
    if (strstr(response, "OK\r\n"))
    {
        printf("Command succeeded.\n");
        return AT_SUCCESS;
    }
    else if (strstr(response, "ERROR\r\n"))
    {
        printf("Command failed.\n");
        return AT_SUCCESS;
    }
    return AT_PENDING;
}


// Parse +CREG: lines. This handler follows the AtHandler response_handler signature.
// +CREG: 0,1
static int creg_response_handler(void* ctx, const char* response, size_t length)
{
    if (!ctx || !response) return 0;

    Bg96CregStatus* st = (Bg96CregStatus*)ctx;
    const char* prefix = "+CREG:";
    if (strncmp(response, prefix, strlen(prefix)) == 0)
    {
        st->response_found = true;
        const char net_status = response[9];
        printf("Network registration status: %c\n", net_status);
        st->n = net_status - '0'; // Convert char to int    
        return AT_PENDING;
    }
    else if (strstr(response, "OK\r\n"))
    {
        printf("Received OK response.\n");
        st->ok_found = true;
        return AT_SUCCESS;
    }
    else if (strstr(response, "ERROR\r\n"))
    {
        return AT_ERR_FAIL;
    }
    return AT_PENDING;
}

// int bg96_query_creg(Bg96* module, Bg96CregStatus* status)
// {
//     if (!status)
//         return -1;

//     AtHandler handler;
//     // initialize output
//     memset(status, 0, sizeof(*status));
//     handler.response_handler = creg_response_handler;
//     handler.context = status;
//     handler.length = 0;
//     memset(handler.response, 0, sizeof(handler.response));

//     // send command; include CR
//     const char* cmd = "AT+CREG?\r";
//     int r = send_at_command(&native_stream, cmd, &handler, 2000);
//     if (r <= 0)
//     {
//         return -1; // timeout or no data
//     }

//     // success: status filled by handler (handler sets flags)
//     if (status->response_found && status->ok_found)
//         return 0;
//     return -1;
// }


int bg96_send_at(Bg96* module, const char* command, uint32_t timeout_ms)
{
    AtHandler handler;
    memset(&handler, 0, sizeof(handler));
    handler.response_handler = parse_simple_at;
    handler.context = NULL;
    handler.length = 0;

    return send_at_command(&native_stream, command, &handler, timeout_ms);
}

int bg96_get_network_registration(Bg96* module, Bg96AtResult* result, uint32_t timeout_ms)
{
    if (!result)
        return AT_ERR_BAD_INPUT;

    memset(result, 0, sizeof(*result));
    AtHandler handler;
    memset(&handler, 0, sizeof(handler));
    handler.response_handler = creg_response_handler;
    handler.context = &result->detail.creg;
    handler.length = 0;

    const char* cmd = "AT+CREG?\r";
    int r = send_at_command(&native_stream, cmd, &handler, timeout_ms);

    return 0;
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

static void print_response(const char* response, size_t length)
{
    printf("BG96 Response: ");
    for (size_t i = 0; i < length; i++)
    {
        if (response[i] == '\r')
        {
            printf("\\r");
        }
        else if (response[i] == '\n')
        {
            printf("\\n");
        }
        else
        {
            printf("%c", response[i]);
        }
    }
    printf("\r\n");
}

static bool is_ok(const char* response)
{
    return strstr(response, "OK\r\n") != NULL;
}

static bool is_error(const char* response)
{
    return strstr(response, "ERROR\r\n") != NULL;
}

// Helper: trim leading spaces
static const char* skip_spaces(const char* s)
{
    while (*s == ' ' || *s == '\t') s++;
    return s;
}
