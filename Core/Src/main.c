/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "main.h"
#include "FreeRTOS.h"
#include "stm32u545xx.h"
#include "stm32u5xx.h" // Or a more specific system header if needed
#include "stm32u5xx_ll_bus.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_usart.h"
#include "task.h"

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

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

static void test_task(void* args)
{
    while (1)
    {
        // printf("Hello from FreeRTOS task!\n\r");
        uart_send_blocking("STM32U5xx FreeRTOS Example\n\r");
        BSP_LED_Toggle(LED_GREEN);
        vTaskDelay(
            pdMS_TO_TICKS(500)); // half-second delay for visible blinking
    }
}

void uart1_send_char_blocking(char c)
{
    while (!(USART1->ISR & USART_ISR_TXE))
    {
    }
    USART1->TDR = c;
    while (!(USART1->ISR & USART_ISR_TC))
    {
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

    for (int irq = 0; irq < 56; irq++) // 150 is a safe upper bound for STM32U5
    {
        // NVIC_DisableIRQ((IRQn_Type)irq);
    }
    __set_PRIMASK(1);
    // NVIC_DisableIRQ((IRQn_Type)56);

    /* Configure the system clock */
    SystemClock_Config();

    // __disable_irq(); // Disable interrupts globally

    /* Configure the System Power */
    SystemPower_Config();

    /* Initialize all configured peripherals */
    MX_ICACHE_Init();

    /* Initialize led */
    BSP_LED_Init(LED_GREEN);

    /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity
     */
    BspCOMInit.BaudRate = 115200;
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

    xTaskCreate(test_task,
                "TestTask",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL);

    /* Start the scheduler */
    vTaskStartScheduler();
    /* We should never get here as control is now taken by the scheduler */
    while (1)
    {
        uart_send_blocking("STM32U5xx FreeRTOS Example\n\r");
        // printf("Welcome to STM32 world !\n\r");
        BSP_LED_Toggle(LED_GREEN);
        // HAL_Delay(500); // half-second delay for visible blinking
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                  RCC_CLOCKTYPE_PCLK3;
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
 * @brief BSP Push Button callback
 * @param Button Specifies the pressed button
 * @retval None
 */
void BSP_PB_Callback(Button_TypeDef Button)
{
    printf("Button pressed: %d\n\r", Button);
    if (Button == BUTTON_USER)
    {
        BspButtonState = BUTTON_PRESSED;
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
