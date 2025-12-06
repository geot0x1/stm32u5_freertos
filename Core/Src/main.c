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
#include <string.h>


COM_InitTypeDef BspCOMInit;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_ICACHE_Init(void);



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


static Bg96 bg96_module =
{
};


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




static void test_task(void* args)
{
    printf("Test task started\n\r");

    bg96_init(&bg96_module);
    bg96_power_on(&bg96_module);

    vTaskDelay(pdMS_TO_TICKS(3500)); // Wait for BG96 to power on


    // const char* at = "AT\r";
    // const char* at_br = "AT+IPR=230400\r";

    // for (int i = 0; i < 3; i++)
    // {
    //     modem_serial_write(&lpuart_serial, (const uint8_t*)at, strlen(at));
    //     vTaskDelay(pdMS_TO_TICKS(500)); // Wait for response
    // }

    // modem_serial_write(&lpuart_serial, "AT+CPIN?\r", 9);



    while (1)
    {
        BSP_LED_Toggle(LED_GREEN);

        printf("Sending AT command: \r\n");
        vTaskDelay(pdMS_TO_TICKS(500));
        bg96_send_at(&bg96_module, "AT\r", 2000);

        // modem_serial_write(&lpuart_serial, "AT+CREG?\r", 9);
        // vTaskDelay(pdMS_TO_TICKS(200)); // Wait for response
        // while (fifo_is_empty(&lpuart_fifo) == false)
        // {
        //     uint8_t newbyte = 0;
        //     fifo_pop(&lpuart_fifo, &newbyte);
        //     uart1_send_char_blocking(newbyte); // Echo back the received byte
        //     vTaskDelay(1);
        // }

    }
}

static void background_task(void* args)
{
    while (1)
    {
        printf("tick\r\n");
        // Background processing can be done here
        vTaskDelay(pdMS_TO_TICKS(1000)); // Sleep for 1 second
    }
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

    // uart_send_blocking("Board restarted\n\r");

    BSP_LED_On(LED_GREEN);

    xTaskCreate(test_task, "TestTask", 1024, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(background_task, "BackgroundTask", 512, NULL, tskIDLE_PRIORITY + 1, NULL);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
