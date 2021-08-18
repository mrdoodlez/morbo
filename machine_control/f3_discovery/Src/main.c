#include "main.h"

#define I2C_ADDRESS     0xAA
#define I2C_TIMING      0x00201D2C //0x00C4092A

I2C_HandleTypeDef I2cHandle;
UART_HandleTypeDef UartHandle;

__IO ITStatus UartReady = RESET;

uint8_t aTxBuffer[] = "Hello! Hello! Hello! Hello!";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

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

    grove_begin(0x0F);    

    while (1) {
        for (int i = -10; i < 10; i++) {
            grove_speed(MOTOR2, i * 10 );
            HAL_Delay(1000);
        }
        
        for (int i = 10; i > -10; i--) {
            grove_speed(MOTOR2, i * 10 );
            HAL_Delay(1000);
        }
    }


    do {
        // Set speed of MOTOR1, Clockwise, speed: -100~100
        grove_speed(MOTOR1, 50);

        // Set speed of MOTOR2, Anticlockwise
        grove_speed(MOTOR2, -70);
        HAL_Delay(5000);

        // Change speed and direction of MOTOR1
        grove_speed(MOTOR1, -100);

        // Change speed and direction of MOTOR2
        grove_speed(MOTOR2, 100);
        HAL_Delay(5000);

        // Stop MOTOR1 and MOTOR2
        grove_stop(MOTOR1);
        grove_stop(MOTOR2);
        HAL_Delay(5000);
    } while(1);

    do {
        if(HAL_I2C_Master_Transmit_IT(&I2cHandle, (uint16_t)I2C_ADDRESS,
                                      (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK) {

            /* Error_Handler() function is called when error occurs. */
            Error_Handler();
        }

        while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY) {
        }

        HAL_Delay(50);

    } while(1); //HAL_I2C_GetError(&I2cHandle) == HAL_I2C_ERROR_AF);

    while(1);

    return 0;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int _main(void) {
    /* STM32F3xx HAL library initialization:
         - Configure the Flash prefetch
         - Systick timer is configured by default as source of time base, but user
           can eventually implement his proper time base source (a general purpose
           timer for example or other time source), keeping in mind that Time base
           duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
           handled in milliseconds basis.
         - Set NVIC Group Priority to 4
         - Low Level Initialization
       */
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

#ifdef TRANSMITTER_BOARD

    /* Configure User push-button in Interrupt mode */
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    /* Wait for User push-button press before starting the Communication.
       In the meantime, LED4 is blinking */
    while(UserButtonStatus == 0) {
        /* Toggle LED4*/
        BSP_LED_Toggle(LED4);
        HAL_Delay(100);
    }

    BSP_LED_Off(LED4);

    /* The board sends the message and expects to receive it back */

    /*##-2- Start the transmission process #####################################*/
    /* While the UART in reception process, user can transmit data through
       "aTxBuffer" buffer */
    if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK) {
        Error_Handler();
    }

    /*##-3- Wait for the end of the transfer ###################################*/
    while (UartReady != SET) {
    }

    /* Reset transmission flag */
    UartReady = RESET;

    /*##-4- Put UART peripheral in reception process ###########################*/
    if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK) {
        Error_Handler();
    }

#else

    /* The board receives the message and sends it back */

    /*##-2- Put UART peripheral in reception process ###########################*/
    if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK) {
        Error_Handler();
    }

    /*##-3- Wait for the end of the transfer ###################################*/
    /* While waiting for message to come from the other board, LED4 is
       blinking according to the following pattern: a double flash every half-second */
    while (UartReady != SET) {
        BSP_LED_On(LED4);
        HAL_Delay(100);
        BSP_LED_Off(LED4);
        HAL_Delay(100);
        BSP_LED_On(LED4);
        HAL_Delay(100);
        BSP_LED_Off(LED4);
        HAL_Delay(500);
    }

    /* Reset transmission flag */
    UartReady = RESET;
    BSP_LED_Off(LED4);

    /*##-4- Start the transmission process #####################################*/
    /* While the UART in reception process, user can transmit data through
       "aTxBuffer" buffer */
    if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK) {
        Error_Handler();
    }

#endif /* TRANSMITTER_BOARD */

    /*##-5- Wait for the end of the transfer ###################################*/
    while (UartReady != SET) {
    }

    /* Reset transmission flag */
    UartReady = RESET;

    /*##-6- Compare the sent and received buffers ##############################*/
    if(Buffercmp((uint8_t*)aTxBuffer,(uint8_t*)aRxBuffer,RXBUFFERSIZE)) {
        Error_Handler();
    }

    /* Infinite loop */
    while (1) {
    }
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
    /* Set transmission flag: transfer complete */
    UartReady = SET;

    /* Turn LED3 on: Transfer in transmission process is correct */
    BSP_LED_On(LED3);

}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
    /* Set transmission flag: transfer complete */
    UartReady = SET;

    /* Turn LED5 on: Transfer in reception process is correct */
    BSP_LED_On(LED5);

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
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength) {
    while (BufferLength--) {
        if ((*pBuffer1) != *pBuffer2) {
            return BufferLength;
        }
        pBuffer1++;
        pBuffer2++;
    }

    return 0;
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


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
