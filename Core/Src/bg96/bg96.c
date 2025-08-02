#include "bg96.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_bus.h"
#include "stm32u545xx.h"

#include "FreeRTOS.h"
#include "task.h"



typedef struct
{
    char response[256];
    size_t length;
} AtHandler;




static int native_modem_serial_init(CellularStream* self, int baudrate)
{
    ModemSerial* modem_serial = self->context;
    return modem_serial_open(modem_serial, baudrate);
}

static int native_modem_serial_write(CellularStream* self, const uint8_t* data, uint16_t len)
{
    ModemSerial* modem_serial = self->context;
    return modem_serial_write(modem_serial, data, len);
}

static int native_modem_serial_read(CellularStream* self, uint8_t* data, uint16_t len)
{
    ModemSerial* modem_serial = self->context;
    return modem_serial_read(modem_serial, data, len);
}

static int native_modem_serial_close(CellularStream* self)
{
    // If there's no close function, we can just return 0
    return 0;
}


CellularStreamVtable native_stream_vtable = {
    .open = native_modem_serial_init,
    .write = native_modem_serial_write,
    .read = native_modem_serial_read,
    .close = native_modem_serial_close
};

CellularStream native_stream = {
    .context = NULL,
    .vtable = &native_stream_vtable
};

void send_at_command(CellularStream* stream, const char* command, AtHandler* handler, uint32_t timeout)
{

}


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

void bg96_reset_pin_set_high(void)
{
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
}

void bg96_reset_pin_set_low(void)
{
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
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

void bg96_pwrkey_pin_set_high(void)
{
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);
}

void bg96_pwrkey_pin_set_low(void)
{
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);
}

void bg96_reset_pin_on(void)
{
    bg96_reset_pin_set_high();
}

void bg96_reset_pin_off(void)
{
    bg96_reset_pin_set_low();
}

void bg96_pwrkey_pin_on(void)
{
    bg96_pwrkey_pin_set_high();
}

void bg96_pwrkey_pin_off(void)
{
    bg96_pwrkey_pin_set_low();
}

int bg96_init(Bg96* module)
{
    bg96_reset_pin_init();
    bg96_pwrkey_pin_init();

    // Set initial states
    bg96_reset_pin_set_low(); // Reset pin low
    bg96_pwrkey_pin_set_high(); // Power key pin high

    modem_serial_open(module->serial, 115200); // Open the serial interface at 115200 baud
    return 0;
}

void bg96_power_on(Bg96* module)
{
    bg96_pwrkey_pin_on(); // Set power key low to turn on
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
    bg96_pwrkey_pin_off(); // Set power key high to complete power on
}


