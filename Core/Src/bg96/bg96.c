#include "bg96.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_bus.h"
#include "stm32u545xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "gsm_hal.h"
#include <string.h>



typedef struct
{
    char response[256];
    size_t length;
} AtHandler;




static int native_modem_serial_init(GsmStream* self, int baudrate)
{
    ModemSerial* modem_serial = self->context;
    return modem_serial_open(modem_serial, baudrate);
}

static int native_modem_serial_write(GsmStream* self, const uint8_t* data, uint16_t len)
{
    ModemSerial* modem_serial = self->context;
    return modem_serial_write(modem_serial, data, len);
}

static int native_modem_serial_read(GsmStream* self, uint8_t* data, uint16_t len)
{
    ModemSerial* modem_serial = self->context;
    return modem_serial_read(modem_serial, data, len);
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

    if (!module || !module->serial)
    {
        return -1;
    }

    // Hook up the native stream to the modem serial
    native_stream.context = module->serial;
    module->streams.s = &native_stream;

    int r = modem_serial_open(module->serial, 115200); // Open the serial interface at 115200 baud
    return r;
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


