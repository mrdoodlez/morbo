/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComIT/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3_discovery.h"
#include "mc_control.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_5
#define USARTx_TX_GPIO_PORT              GPIOD
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_6
#define USARTx_RX_GPIO_PORT              GPIOD
#define USARTx_RX_AF                     GPIO_AF7_USART2

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

#define I2Cx                            I2C1
#define RCC_PERIPHCLK_I2Cx              RCC_PERIPHCLK_I2C1
//#define RCC_I2CxCLKSOURCE_SYSCLK        RCC_I2C1CLKSOURCE_SYSCLK
#define RCC_I2CxCLKSOURCE                RCC_I2C1CLKSOURCE_HSI
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_6
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_7
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C1

/* Definition for I2Cx's NVIC */
#define I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define I2Cx_ER_IRQn                    I2C1_ER_IRQn
#define I2Cx_EV_IRQHandler              I2C1_EV_IRQHandler
#define I2Cx_ER_IRQHandler              I2C1_ER_IRQHandler

/* Definition for TIMx clock resources */
#define TIMx                           TIM3
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE()

/* Definition for TIMx Channel Pins */
#define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOC_CLK_ENABLE()
#define TIMx_GPIO_PORT_CHANNEL1        GPIOC
#define TIMx_GPIO_PORT_CHANNEL2        GPIOC
#define TIMx_GPIO_PORT_CHANNEL3        GPIOC
#define TIMx_GPIO_PORT_CHANNEL4        GPIOC
#define TIMx_GPIO_PIN_CHANNEL1         GPIO_PIN_6
#define TIMx_GPIO_PIN_CHANNEL2         GPIO_PIN_7
#define TIMx_GPIO_PIN_CHANNEL3         GPIO_PIN_8
#define TIMx_GPIO_PIN_CHANNEL4         GPIO_PIN_9
#define TIMx_GPIO_AF_CHANNEL1          GPIO_AF2_TIM3
#define TIMx_GPIO_AF_CHANNEL2          GPIO_AF2_TIM3
#define TIMx_GPIO_AF_CHANNEL3          GPIO_AF2_TIM3
#define TIMx_GPIO_AF_CHANNEL4          GPIO_AF2_TIM3

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

#define  PERIOD_VALUE       (uint32_t)(1330 - 1)  /* Period Value  */

extern TIM_HandleTypeDef  mcTimHandle;
extern TIM_HandleTypeDef  svTimHandle;
extern TIM_OC_InitTypeDef sConfig;

void Error_Handler(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
