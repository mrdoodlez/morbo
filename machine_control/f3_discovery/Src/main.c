#include "main.h"

#define I2C_ADDRESS     0xAA
#define I2C_TIMING      0x00201D2C //0x00C4092A

I2C_HandleTypeDef I2cHandle;
UART_HandleTypeDef UartHandle;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);

uint8_t rx_rc_buff;

int main(void) {
    HAL_Init();

    /* Configure the system clock to 72 MHz */
    SystemClock_Config();

    /* Configure LED3, LED4, LED5 and LED6 */
    BSP_LED_Init(LED3);
    BSP_LED_Init(LED4);
    BSP_LED_Init(LED5);
    BSP_LED_Init(LED6);

    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART configured as follows:
        - Word Length = 8 Bits
        - Stop Bit = One Stop bit
        - Parity = None
        - BaudRate = 9600 baud
        - Hardware flow control disabled (RTS and CTS signals) */
    UartHandle.Instance        = USARTx;

    UartHandle.Init.BaudRate   = 9600;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if(HAL_UART_DeInit(&UartHandle) != HAL_OK) {
        Error_Handler();
    }

    if(HAL_UART_Init(&UartHandle) != HAL_OK) {
        Error_Handler();
    }

    I2cHandle.Instance             = I2Cx;
    I2cHandle.Init.Timing          = I2C_TIMING;
    I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
    I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if(HAL_I2C_Init(&I2cHandle) != HAL_OK) {
        /* Initialization Error */
        Error_Handler();
    }

    /* Delay to avoid that possible signal rebound is taken as button release */
    HAL_Delay(50);

    if(HAL_UART_Receive_IT(&UartHandle, &rx_rc_buff, sizeof(rx_rc_buff)) != HAL_OK) {
        Error_Handler();
    }

    mc_task_run();

    while(1);


    
    /*
    grove_speed(MOTOR1, 0xff );    
    grove_speed(MOTOR2, 0xff );
    */

    return 0;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLLMUL                         = RCC_PLL_MUL9 (9)
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK) {
        /* Initialization Error */
        while(1);
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK) {
        /* Initialization Error */
        while(1);
    }
}
/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
    //rcdev_on_new_data(rx);
}


void USART2_IRQHandler(void) {
    //HAL_UART_IRQHandler(&UartHandle);

    if(USART2->ISR & USART_ISR_RXNE) {
        uint8_t rx = (USART2->RDR); // Receive data, clear flag     
        rcdev_on_new_data(rx);
    }    
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
    /* Turn LED6 on: Transfer error in reception/transmission process */
    BSP_LED_On(LED6);
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == USER_BUTTON_PIN) {
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void) {
    /* Turn LED6 on */
    BSP_LED_On(LED6);
    while(1) {
    }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1) {
    }
}
#endif

int i2c_write(const unsigned char *buff, unsigned int len) {
    if(HAL_I2C_Master_Transmit_IT(&I2cHandle, buff[0] << 1, buff+1, len-1)!= HAL_OK) {
        return 0;
    }
    while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY) {      
    }
    
    if(HAL_I2C_GetError(&I2cHandle) == HAL_I2C_ERROR_AF) {
        return 0;
    }

    return len;
}

void delay(unsigned int ms) {
    HAL_Delay(ms);
}

void led_toggle() {
    BSP_LED_Toggle(LED6);
}

void led_on() {
    BSP_LED_On(LED6);
}

void led_off() {
    BSP_LED_Off(LED6);
}

uint32_t get_time() {
    return HAL_GetTick();
}
