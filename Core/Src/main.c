#include "main.h"
#include "FreeRTOS.h"
#include "stm32u545xx.h"
#include "stm32u5xx.h" // Or a more specific system header if needed
#include "stm32u5xx_ll_bus.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_usart.h"
#include "task.h"
#include "drv8825.h"
#include "bg96.h"
#include "fifo.h"
#include "stm32u5xx_ll_lpuart.h"


COM_InitTypeDef BspCOMInit;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_ICACHE_Init(void);

static void LPUART1_Init(int baudrate);
static int lpuart_init(USART_TypeDef* instance, int baudrate);


void vApplicationTickHook(void) { HAL_IncTick(); }

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    // This function is called if a task overflows its stack

    // Optional: print or log task name
    printf("Stack overflow in task: %s\n", pcTaskName);

    // Optional: loop forever (useful for debugging)
    for (;;)
    {
        ;
    }
}

static uint8_t _fifo_buffer[512]; // Buffer for UART1
static Fifo lpuart_fifo;
uint8_t rx_byte;

static modem_serial_lpuart_init(ModemSerial* self, int baudrate);
static int modem_serial_lpuart_write(ModemSerial* self, const uint8_t* data, uint16_t len);
static int modem_serial_lpuart_read(ModemSerial* self, uint8_t* data, uint16_t len);


static ModemSerial lpuart_serial =
{
    .context = LPUART1,
    .open = modem_serial_lpuart_init,
    .write = modem_serial_lpuart_write,
    .read = modem_serial_lpuart_read
};

static Bg96 bg96_module =
{
    .serial = &lpuart_serial // Assign the serial interface to the BG96 module
};

static modem_serial_lpuart_init(ModemSerial* self, int baudrate)
{
    USART_TypeDef* instance = self->context;
    fifo_init(&lpuart_fifo, _fifo_buffer, sizeof(_fifo_buffer));
    lpuart_init(instance, baudrate);
    return 0; // Success
}

static int modem_serial_lpuart_write(ModemSerial* self, const uint8_t* data, uint16_t len)
{
    USART_TypeDef* instance = self->context;
    
    for (uint16_t i = 0; i < len; ++i)
    {
        while (!LL_LPUART_IsActiveFlag_TXE(instance));
        LL_LPUART_TransmitData8(instance, data[i]);
    }

    // Optional: wait for final transmission to complete
    while (!LL_LPUART_IsActiveFlag_TC(instance)); // Wait for TC (Transmission Complete)
    return len; // Return number of bytes written
}

static int modem_serial_lpuart_read(ModemSerial* self, uint8_t* data, uint16_t len)
{
    USART_TypeDef* instance = self->context;
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

static int lpuart_init(USART_TypeDef* instance, int baudrate)
{
    LL_LPUART_InitTypeDef LPUART_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the peripherals clock
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
    PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
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
    while (!LL_LPUART_IsActiveFlag_TEACK(LPUART1) || !LL_LPUART_IsActiveFlag_REACK(LPUART1)) {}

    return 0; // Success
}



void LPUART1_IRQHandler(void)
{
    uint8_t receivedByte = 0;
    if (LL_LPUART_IsActiveFlag_RXNE(LPUART1) && LL_LPUART_IsEnabledIT_RXNE(LPUART1))
    {
        receivedByte = LL_LPUART_ReceiveData8(LPUART1);
        fifo_push(&lpuart_fifo, receivedByte);
        // uart1_send_char_blocking(receivedByte); // Echo back the received byte
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

char* read_string(void)
{
    static char respbuffer[64];
    uint16_t index = 0;
   
    // memset(respbuffer, 0, sizeof(respbuffer));
    uint32_t timeout = 300;
    uint8_t newbyte;
    while (fifo_is_empty(&lpuart_fifo) == false && timeout-- > 0)
    {
        LL_LPUART_DisableIT_RXNE(LPUART1);
        bool fifo_result = fifo_pop(&lpuart_fifo, &newbyte);
        LL_LPUART_EnableIT_RXNE(LPUART1);
        if (fifo_result)
        {
            if (index >= 62)
            {
                break;
            }
            respbuffer[index++] = newbyte;
            respbuffer[index] = 0;
        }
        else
        {
        }
            timeout--;
            vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to allow other tasks to run
    }
    return respbuffer;
}


static void test_task(void* args)
{
    printf("Test task started\n\r");

    bg96_init(&bg96_module);
    bg96_power_on(&bg96_module);

    vTaskDelay(pdMS_TO_TICKS(3500)); // Wait for BG96 to power on


    const char* at = "AT\r";
    const char* at_br = "AT+IPR=230400\r";

    for (int i = 0; i < 3; i++)
    {
        modem_serial_write(&lpuart_serial, (const uint8_t*)at, strlen(at));
        vTaskDelay(pdMS_TO_TICKS(500)); // Wait for response
    }

    modem_serial_write(&lpuart_serial, "AT+CPIN?\r", 9);



    while (1)
    {
        BSP_LED_Toggle(LED_GREEN);

        printf("Sending AT command: \r\n");

        modem_serial_write(&lpuart_serial, "AT+CREG?\r", 9);
        vTaskDelay(pdMS_TO_TICKS(200)); // Wait for response
        while (fifo_is_empty(&lpuart_fifo) == false)
        {
            uint8_t newbyte = 0;
            fifo_pop(&lpuart_fifo, &newbyte);
            uart1_send_char_blocking(newbyte); // Echo back the received byte
            vTaskDelay(1);
        }

    }
}

void uart1_send_char_blocking(char c)
{
    while (!(USART1->ISR & USART_ISR_TXE))
    {
    }
    USART1->TDR = c;
    // while (!(USART1->ISR & USART_ISR_TC))
    // {
    // }
}

void uart_send_blocking(const char* s)
{
    while (*s)
    {
        uart1_send_char_blocking(*s++);
    }
}

void uart1_init(void)
{
    // 1. Enable GPIOA and USART1 clocks
    RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // 2. Configure PA9 as Alternate Function 7 (USART1_TX)
    GPIOA->MODER &= ~(3U << (2 * 9));         // Clear mode bits
    GPIOA->MODER |= (2U << (2 * 9));          // Set to Alternate Function mode
    GPIOA->OTYPER &= ~(1U << 9);              // Push-pull
    GPIOA->OSPEEDR |= (3U << (2 * 9));        // High speed
    GPIOA->AFR[1] &= ~(0xF << (4 * (9 - 8))); // Clear AF
    GPIOA->AFR[1] |= (7U << (4 * (9 - 8)));   // Set AF7 (USART1_TX)

    // 3. Disable USART before configuration
    USART1->CR1 &= ~USART_CR1_UE;

    // 4. Set baud rate (assuming PCLK2 is correctly set)
    USART1->BRR = HAL_RCC_GetPCLK2Freq() / 115200;

    // 5. Enable USART TX and USART
    USART1->CR1 = USART_CR1_TE;
    USART1->CR1 |= USART_CR1_UE;

    // 6. Wait for TE to be acknowledged (optional safety)
    while (!(USART1->ISR & USART_ISR_TEACK))
    {
    }
}




int main(void)
{
    HAL_Init();
    WWDG->CR &= ~(1 << 7);
    __set_PRIMASK(1);

    /* Configure the system clock */
    SystemClock_Config();

    /* Configure the System Power */
    SystemPower_Config();

    /* Initialize all configured peripherals */
    MX_ICACHE_Init();

    /* Initialize led */
    BSP_LED_Init(LED_GREEN);

    // bg96_init();

    BspCOMInit.BaudRate = 230400;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits = COM_STOPBITS_1;
    BspCOMInit.Parity = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
    {
    }
    // uart1_init();

    uart_send_blocking("Board restarted\n\r");

    BSP_LED_On(LED_GREEN);

    xTaskCreate(test_task, "TestTask", 1024, NULL, tskIDLE_PRIORITY, NULL);

    /* Start the scheduler */
    vTaskStartScheduler();
    /* We should never get here as control is now taken by the scheduler */
    while (1)
    {
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief Power Configuration
 * @retval None
 */
static void SystemPower_Config(void)
{
    if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ICACHE Initialization Function
 * @param None
 * @retval None
 */
static void MX_ICACHE_Init(void)
{
    /** Enable instruction cache in 1-way (direct mapped cache)
     */
    if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ICACHE_Enable() != HAL_OK)
    {
        Error_Handler();
    }
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1)
    {
        BSP_LED_Toggle(LED_GREEN);
        for (int i = 0; i < 100000; i++)
        {
            __NOP(); // Simple delay
        }
        BSP_LED_Toggle(LED_GREEN);
        for (int i = 0; i < 100000; i++)
        {
            __NOP(); // Simple delay
        }
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
