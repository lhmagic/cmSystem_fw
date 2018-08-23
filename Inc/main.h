/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DIP_SW2_Pin GPIO_PIN_13
#define DIP_SW2_GPIO_Port GPIOC
#define DIP_SW1_Pin GPIO_PIN_14
#define DIP_SW1_GPIO_Port GPIOC
#define DIP_SW0_Pin GPIO_PIN_15
#define DIP_SW0_GPIO_Port GPIOC
#define AD_CS2_Pin GPIO_PIN_0
#define AD_CS2_GPIO_Port GPIOA
#define CS2_PWR_Pin GPIO_PIN_1
#define CS2_PWR_GPIO_Port GPIOA
#define AD_CS1_Pin GPIO_PIN_2
#define AD_CS1_GPIO_Port GPIOA
#define CS1_PWR_Pin GPIO_PIN_3
#define CS1_PWR_GPIO_Port GPIOA
#define AD_VREF_Pin GPIO_PIN_4
#define AD_VREF_GPIO_Port GPIOA
#define AD_AC_PWR_Pin GPIO_PIN_5
#define AD_AC_PWR_GPIO_Port GPIOA
#define DI_SW1_Pin GPIO_PIN_6
#define DI_SW1_GPIO_Port GPIOA
#define DI_SW2_Pin GPIO_PIN_7
#define DI_SW2_GPIO_Port GPIOA
#define DI_SW3_Pin GPIO_PIN_0
#define DI_SW3_GPIO_Port GPIOB
#define PUMP1_START_Pin GPIO_PIN_1
#define PUMP1_START_GPIO_Port GPIOB
#define PUMP1_STOP_Pin GPIO_PIN_2
#define PUMP1_STOP_GPIO_Port GPIOB
#define PUMP1_FAIL_Pin GPIO_PIN_10
#define PUMP1_FAIL_GPIO_Port GPIOB
#define PUMP2_START_Pin GPIO_PIN_11
#define PUMP2_START_GPIO_Port GPIOB
#define PUMP2_STOP_Pin GPIO_PIN_12
#define PUMP2_STOP_GPIO_Port GPIOB
#define PUMP2_FAIL_Pin GPIO_PIN_13
#define PUMP2_FAIL_GPIO_Port GPIOB
#define PUMP3_START_Pin GPIO_PIN_14
#define PUMP3_START_GPIO_Port GPIOB
#define PUMP3_STOP_Pin GPIO_PIN_15
#define PUMP3_STOP_GPIO_Port GPIOB
#define LED_DS_Pin GPIO_PIN_8
#define LED_DS_GPIO_Port GPIOA
#define LED_OE_Pin GPIO_PIN_9
#define LED_OE_GPIO_Port GPIOA
#define LED_ST_Pin GPIO_PIN_10
#define LED_ST_GPIO_Port GPIOA
#define LED_SH_Pin GPIO_PIN_11
#define LED_SH_GPIO_Port GPIOA
#define USART_DE_Pin GPIO_PIN_12
#define USART_DE_GPIO_Port GPIOA
#define DI_EXT_IN1_Pin GPIO_PIN_6
#define DI_EXT_IN1_GPIO_Port GPIOF
#define DI_EXT_IN2_Pin GPIO_PIN_7
#define DI_EXT_IN2_GPIO_Port GPIOF
#define DBG_TX_SYS_SWCLK_Pin GPIO_PIN_14
#define DBG_TX_SYS_SWCLK_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_15
#define DBG_RX_GPIO_Port GPIOA
#define RS485_TX_LED_Pin GPIO_PIN_4
#define RS485_TX_LED_GPIO_Port GPIOB
#define RS485_RX_LED_Pin GPIO_PIN_5
#define RS485_RX_LED_GPIO_Port GPIOB
#define USART_TX_Pin GPIO_PIN_6
#define USART_TX_GPIO_Port GPIOB
#define USART_RX_Pin GPIO_PIN_7
#define USART_RX_GPIO_Port GPIOB
#define BEEPER_Pin GPIO_PIN_8
#define BEEPER_GPIO_Port GPIOB
#define DIP_SW3_Pin GPIO_PIN_9
#define DIP_SW3_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/