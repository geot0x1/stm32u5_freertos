#include "gsm_hal.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_bus.h"
#include "stm32u545xx.h"
#include "stm32u5xx_ll_lpuart.h"
#include "fifo.h"

#include <stddef.h>

typedef struct GsmHalPinConfig
{
    GPIO_TypeDef* port;
    uint32_t pin;
} GsmHalPinConfig;


static void gsm_hal_set_pin_output(const GsmHalPinConfig* pin_config);
static void gsm_hal_set_pin_low(const GsmHalPinConfig* pin_config);
static void gsm_hal_set_pin_high(const GsmHalPinConfig* pin_config);


static const GsmHalPinConfig GSM_GPIO_RESET_PIN = {
    .port = GPIOB,
    .pin = LL_GPIO_PIN_4
};

static const GsmHalPinConfig GSM_GPIO_PWRKEY_PIN = {
    .port = GPIOB,
    .pin = LL_GPIO_PIN_10
};

static uint8_t _fifo_buffer[64]; // Buffer for UART1
static Fifo lpuart_fifo;


void gsm_hal_gpio_init(void)
{
    gsm_hal_set_pin_output(&GSM_GPIO_RESET_PIN);
    gsm_hal_set_pin_output(&GSM_GPIO_PWRKEY_PIN);
}

void gsm_hal_pwrkey_pin_set_high(void)
{
    gsm_hal_set_pin_high(&GSM_GPIO_PWRKEY_PIN);
}

void gsm_hal_pwrkey_pin_set_low(void)
{
    gsm_hal_set_pin_low(&GSM_GPIO_PWRKEY_PIN);
}

void gsm_hal_reset_pin_set_high(void)
{
    gsm_hal_set_pin_high(&GSM_GPIO_RESET_PIN);
}

void gsm_hal_reset_pin_set_low(void)
{
    gsm_hal_set_pin_low(&GSM_GPIO_RESET_PIN);
}



static void gsm_hal_set_pin_low(const GsmHalPinConfig* pin_config)
{
    if (!pin_config)
    {
        return;
    }
    if (pin_config->port == NULL)
    {
        return;
    }
    LL_GPIO_ResetOutputPin(pin_config->port, pin_config->pin);
}

static void gsm_hal_set_pin_high(const GsmHalPinConfig* pin_config)
{
    if (!pin_config)
    {
        return;
    }
    if (pin_config->port == NULL)
    {
        return;
    }
    LL_GPIO_SetOutputPin(pin_config->port, pin_config->pin);
}

static void gsm_hal_set_pin_output(const GsmHalPinConfig* pin_config)
{
    if (!pin_config)
    {
        return;
    }
    if (pin_config->port == NULL)
    {
        return;
    }
    if (pin_config->port == GPIOB)
    {
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    }
    else if (pin_config->port == GPIOA)
    {
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    }
    else if (pin_config->port == GPIOC)
    {
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    }

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = pin_config->pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(pin_config->port, &GPIO_InitStruct);
}

int gsm_hal_serial_init(int baudrate)
{
    fifo_init(&lpuart_fifo, _fifo_buffer, sizeof(_fifo_buffer));

    LL_LPUART_InitTypeDef LPUART_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the peripherals clock
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
    // Use peripheral (PCLK) as the LPUART1 clock source so it follows the
    // configured APB clock. Selecting HSI can block if HSI isn't enabled.
    PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        return -1;
    }

    /* Peripheral clock enable */
    LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_LPUART1);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /**LPUART1 GPIO Configuration
     PA2   ------> LPUART1_TX
    PA3   ------> LPUART1_RX
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    LL_LPUART_DeInit(LPUART1);
    LPUART_InitStruct.PrescalerValue = LL_LPUART_PRESCALER_DIV1;
    LPUART_InitStruct.BaudRate = baudrate;
    LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
    LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
    LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
    LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
    LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;
    LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
    LL_LPUART_SetTXFIFOThreshold(LPUART1, LL_LPUART_FIFOTHRESHOLD_1_8);
    LL_LPUART_SetRXFIFOThreshold(LPUART1, LL_LPUART_FIFOTHRESHOLD_1_8);
    LL_LPUART_DisableFIFO(LPUART1);
    LL_LPUART_Enable(LPUART1);

    LL_LPUART_EnableIT_RXNE(LPUART1);
    NVIC_SetPriority(LPUART1_IRQn, 6);
    NVIC_EnableIRQ(LPUART1_IRQn);
    LL_LPUART_Enable(LPUART1);
    // while (!LL_LPUART_IsActiveFlag_TEACK(LPUART1) || !LL_LPUART_IsActiveFlag_REACK(LPUART1)) {}

    return 0; // Success
}

int gsm_hal_serial_write(const uint8_t* data, uint16_t len)
{
    if (!data || len == 0)
    {
        return -1; // Invalid parameters
    }
    USART_TypeDef* instance = LPUART1;
    
    for (uint16_t i = 0; i < len; ++i)
    {
        while (!LL_LPUART_IsActiveFlag_TXE(instance));
        LL_LPUART_TransmitData8(instance, data[i]);
    }
    // Optional: wait for final transmission to complete
    while (!LL_LPUART_IsActiveFlag_TC(instance)); // Wait for TC (Transmission Complete)
    return len;
}

int gsm_hal_serial_read(uint8_t* data, uint16_t len)
{
    USART_TypeDef* instance = LPUART1;
    uint16_t bytes_read = 0;

    while (bytes_read < len)
    {
        if (fifo_is_empty(&lpuart_fifo))
        {
            break; // Exit if no more data in FIFO
        }

        uint8_t newbyte;
        LL_LPUART_DisableIT_RXNE(LPUART1);
        bool fifo_result = fifo_pop(&lpuart_fifo, &newbyte);
        LL_LPUART_EnableIT_RXNE(LPUART1);
        if (fifo_result)
        {
            data[bytes_read++] = newbyte;
        }
    }

    return bytes_read; // Return number of bytes read
}

void gsm_hal_serial_deinit(void)
{

}



void LPUART1_IRQHandler(void)
{
    uint8_t receivedByte = 0;
    if (LL_LPUART_IsActiveFlag_RXNE(LPUART1) && LL_LPUART_IsEnabledIT_RXNE(LPUART1))
    {
        receivedByte = LL_LPUART_ReceiveData8(LPUART1);
        fifo_push(&lpuart_fifo, receivedByte);
    }
    if (LL_LPUART_IsActiveFlag_ORE(LPUART1))
    {
        LL_LPUART_ClearFlag_ORE(LPUART1); // Clear overrun error
    }
    if (LL_LPUART_IsActiveFlag_FE(LPUART1))
    {
        LL_LPUART_ClearFlag_FE(LPUART1); // Clear framing error
    }
    if (LL_LPUART_IsActiveFlag_NE(LPUART1))
    {
        LL_LPUART_ClearFlag_NE(LPUART1); // Clear noise error
    }
}