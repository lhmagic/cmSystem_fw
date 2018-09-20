
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "bsp.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 64 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId regularSampleHandle;
uint32_t regularSampleBuffer[ 64 ];
osStaticThreadDef_t regularSampleControlBlock;
osThreadId LedDisplayHandle;
uint32_t LedDisplayBuffer[ 64 ];
osStaticThreadDef_t LedDisplayControlBlock;
osThreadId configParameterHandle;
uint32_t configTaskBuffer[ 64 ];
osStaticThreadDef_t configTaskControlBlock;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const char cmd_read_statu1[]	=	{0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0a};
const char cmd_read_pres1[]		=	{0x0B,0x03,0x00,0x04,0x00,0x01,0xC5,0x61};
const char cmd_read_pres2[]		=	{0x0C,0x03,0x00,0x04,0x00,0x01,0xC4,0xD6};

const char cmd_read_statu2[]	=	{0x02,0x03,0x00,0x00,0x00,0x01,0x84,0x39};
const char cmd_read_pres3[]		=	{0x0D,0x03,0x00,0x04,0x00,0x01,0xC5,0x07};
const char cmd_read_pres4[]		=	{0x0E,0x03,0x00,0x04,0x00,0x01,0xC5,0x34};

s_sys_para sys_para;
uint32_t adc_buf[5];
int32_t avg_vref, avg_ac, avg_v1650, avg_cs1, avg_cs2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void sampleTask(void const * argument);
void DisplayTask(void const * argument);
void configTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t usart_buff[RS485_BUFF_MAX];
uint8_t usart_rx_cplt;
uint8_t debug_buff[DEBUG_BUFF_MAX];
uint8_t debug_rx_cplt;
uint8_t response_buff[64];

u_ac_state ac_state;	
u_dc_state dc_state;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	memcpy(&sys_para, (void *)parama_save_addr, sizeof(s_sys_para));
	if((sys_para.pres1_k <=0) || (sys_para.pres1_k >=5000)) {
		sys_para.pres1_k = 1000;
	}
	if(sys_para.pres1_b >= 1000) {
		sys_para.pres1_b = 0;
	}	
	
	if((sys_para.pres2_k <=0) || (sys_para.pres2_k >=5000)) {
		sys_para.pres1_k = 1000;
	}
	if(sys_para.pres2_b >= 1000) {
		sys_para.pres2_b = 0;
	}		
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM16_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_IWDG_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_ADC_Start_DMA(&hadc, adc_buf, 5);	
	
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_Delay(200);
	HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
	HAL_IWDG_Refresh(&hiwdg);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 64, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of regularSample */
  osThreadStaticDef(regularSample, sampleTask, osPriorityNormal, 0, 64, regularSampleBuffer, &regularSampleControlBlock);
  regularSampleHandle = osThreadCreate(osThread(regularSample), NULL);

  /* definition and creation of LedDisplay */
  osThreadStaticDef(LedDisplay, DisplayTask, osPriorityBelowNormal, 0, 64, LedDisplayBuffer, &LedDisplayControlBlock);
  LedDisplayHandle = osThreadCreate(osThread(LedDisplay), NULL);

  /* definition and creation of configParameter */
  osThreadStaticDef(configParameter, configTask, osPriorityLow, 0, 64, configTaskBuffer, &configTaskControlBlock);
  configParameterHandle = osThreadCreate(osThread(configParameter), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_BACKWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T15_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 300;
  hiwdg.Init.Reload = 300;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 480;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 100;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 15;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 120;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim16);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS2_PWR_Pin|CS1_PWR_Pin|LED_DS_Pin|LED_OE_Pin 
                          |LED_ST_Pin|LED_SH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS485_TX_LED_Pin|RS485_RX_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DIP_SW2_Pin DIP_SW1_Pin DIP_SW0_Pin */
  GPIO_InitStruct.Pin = DIP_SW2_Pin|DIP_SW1_Pin|DIP_SW0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS2_PWR_Pin CS1_PWR_Pin LED_DS_Pin LED_OE_Pin 
                           LED_ST_Pin LED_SH_Pin */
  GPIO_InitStruct.Pin = CS2_PWR_Pin|CS1_PWR_Pin|LED_DS_Pin|LED_OE_Pin 
                          |LED_ST_Pin|LED_SH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_SW1_Pin DI_SW2_Pin */
  GPIO_InitStruct.Pin = DI_SW1_Pin|DI_SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_SW3_Pin PUMP1_START_Pin PUMP1_STOP_Pin PUMP1_FAIL_Pin 
                           PUMP2_START_Pin PUMP2_STOP_Pin PUMP2_FAIL_Pin PUMP3_START_Pin 
                           PUMP3_STOP_Pin */
  GPIO_InitStruct.Pin = DI_SW3_Pin|PUMP1_START_Pin|PUMP1_STOP_Pin|PUMP1_FAIL_Pin 
                          |PUMP2_START_Pin|PUMP2_STOP_Pin|PUMP2_FAIL_Pin|PUMP3_START_Pin 
                          |PUMP3_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_EXT_IN2_Pin DI_EXT_IN1_Pin */
  GPIO_InitStruct.Pin = DI_EXT_IN2_Pin|DI_EXT_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_TX_LED_Pin RS485_RX_LED_Pin */
  GPIO_InitStruct.Pin = RS485_TX_LED_Pin|RS485_RX_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIP_SW3_Pin */
  GPIO_InitStruct.Pin = DIP_SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DIP_SW3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint16_t get_ac220(void) {
	return (uint16_t)(avg_ac*3300.0*221/(4095*8.985*50));
}

int16_t get_pres(uint8_t num) {
int16_t pres=0;	
	if(num == 1) {
		if(avg_cs1 == 0) {
			return 0;
		} else {
			pres = ((3300.0*avg_cs1)/(4095.0)*0.625-250)*(sys_para.pres1_k/1000.0)+sys_para.pres1_b;
		}
	} else if(num==2) {
		if(avg_cs2 == 0) {
			return 0;
		} else {		
			pres = ((3300.0*avg_cs2)/(4095.0)*0.625-250)*(sys_para.pres2_k/1000.0)+sys_para.pres2_b;
		}
	}
	return pres;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
static int32_t sum_vref, sum_ac, sum_v1650, sum_cs1, sum_cs2;	
static int16_t vref[FILTER_BUFF_SIZE], ac[FILTER_BUFF_SIZE];
static int16_t v1650[FILTER_BUFF_SIZE], cs1[FILTER_BUFF_SIZE], cs2[FILTER_BUFF_SIZE];
static int8_t index;
static int32_t ac_temp;
	
	sum_vref -= vref[index]; 
	vref[index] = adc_buf[0];
	sum_vref += vref[index];

	sum_v1650 -= v1650[index]; 
	v1650[index] = adc_buf[2];
	sum_v1650 += v1650[index];			

	sum_cs1 -= cs1[index]; 
	cs1[index] = adc_buf[3];
	sum_cs1 += cs1[index];			

	sum_cs2 -= cs2[index]; 
	cs2[index] = adc_buf[4];
	sum_cs2 += cs2[index];			

	ac_temp += (adc_buf[1]-2060)*(adc_buf[1]-2060);	

	if(++index >= FILTER_BUFF_SIZE) {
	static uint8_t ac_index;
		
		index = 0;
		sum_ac -= ac[ac_index]; 
		ac[ac_index] = sqrt(ac_temp/32);
		sum_ac += ac[ac_index];
		ac_temp = 0;
		
		if(++ac_index >= FILTER_BUFF_SIZE) {
			ac_index = 0;
		}
	}

	avg_vref = sum_vref/FILTER_BUFF_SIZE;
	avg_ac = sum_ac/FILTER_BUFF_SIZE;
	avg_v1650 = sum_v1650/FILTER_BUFF_SIZE;
	avg_cs1 = sum_cs1/FILTER_BUFF_SIZE;
	avg_cs2 = sum_cs2/FILTER_BUFF_SIZE;

	//if voltage greater than 3v, power off output.
	if(avg_cs1 >= 3722) {
		HAL_GPIO_WritePin(CS1_PWR_GPIO_Port, CS1_PWR_Pin, GPIO_PIN_RESET);
	}

	//if voltage greater than 3v, power off output.
	if(avg_cs2 >= 3722) {
		HAL_GPIO_WritePin(CS2_PWR_GPIO_Port, CS2_PWR_Pin, GPIO_PIN_RESET);
	}		
}

int fputc(int ch, FILE *f) {
uint8_t c;
	c = ch;
	HAL_UART_Transmit(&huart2, &c, 1, 1);
	return ch;
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
uint32_t PreviousWakeTime = osKernelSysTick();	
  /* Infinite loop */
  for(;;)
  {
		if(sys_para.cs1_switch != 0) {
			HAL_GPIO_WritePin(CS1_PWR_GPIO_Port, CS1_PWR_Pin, GPIO_PIN_SET);
		}
		if(sys_para.cs2_switch != 0) {
			HAL_GPIO_WritePin(CS2_PWR_GPIO_Port, CS2_PWR_Pin, GPIO_PIN_SET);
		}
		
		HAL_IWDG_Refresh(&hiwdg);
    osDelayUntil(&PreviousWakeTime, 1000);
  }
  /* USER CODE END 5 */ 
}

/* sampleTask function */
void sampleTask(void const * argument)
{
  /* USER CODE BEGIN sampleTask */
uint32_t PreviousWakeTime = osKernelSysTick();
uint8_t response_buff[16];
uint16_t receive_cnt, mb_crc;
uint8_t addr;
	
	HAL_UART_Receive_IT(&huart1, usart_buff, RS485_BUFF_MAX);
	SET_BIT(huart1.Instance->ICR, USART_ICR_IDLECF);
  SET_BIT(huart1.Instance->CR1, USART_CR1_IDLEIE);
	
  /* Infinite loop */
  for(;;)
  {
		if(usart_rx_cplt) {
			HAL_GPIO_WritePin(RS485_RX_LED_GPIO_Port, RS485_RX_LED_Pin, GPIO_PIN_RESET);
			receive_cnt = RS485_BUFF_MAX-huart1.RxXferCount; 
			HAL_UART_AbortReceive_IT(&huart1);
			addr = get_rs485_addr();
			
			if(receive_cnt == 8) {
				if((strncmp((const char *)usart_buff, cmd_read_statu1, receive_cnt) == 0) && (addr == 1)) {
					response_buff[0] = usart_buff[0];
					response_buff[1] = usart_buff[1];
					response_buff[2] = 0x02;
					response_buff[3] = 0;
					response_buff[4] = (ac_state.pump1_stop<<0) | (ac_state.ac_sw1<<1)\
													 | (ac_state.pump1_fail<<2) | (ac_state.pump1_start<<3)\
													 | (ac_state.pump2_stop<<4) | (ac_state.ac_sw2<<5) \
													 | (ac_state.pump2_fail<<6) | (ac_state.pump2_start<<7);
					mb_crc = mb_crc16(response_buff, 5);
					response_buff[5] = mb_crc & 0xFF;
					response_buff[6] = mb_crc >> 8;
					HAL_UART_Transmit_IT(&huart1, response_buff, 7);	
					HAL_GPIO_WritePin(RS485_TX_LED_GPIO_Port, RS485_TX_LED_Pin, GPIO_PIN_RESET);					
				} else if((strncmp((const char *)usart_buff, cmd_read_pres1, receive_cnt) == 0) && (addr == 1)) {
				int16_t pres = get_pres(1);
					pres = (pres < 0) ? 0 : pres;
					response_buff[0] = usart_buff[0];
					response_buff[1] = usart_buff[1];
					response_buff[2] = 0x02;
					response_buff[3] = pres>>8;
					response_buff[4] = pres&0xFF;
					mb_crc = mb_crc16(response_buff, 5);
					response_buff[5] = mb_crc & 0xFF;
					response_buff[6] = mb_crc >> 8;
					HAL_UART_Transmit_IT(&huart1, response_buff, 7);	
					HAL_GPIO_WritePin(RS485_TX_LED_GPIO_Port, RS485_TX_LED_Pin, GPIO_PIN_RESET);	
				} else if((strncmp((const char *)usart_buff, cmd_read_pres2, receive_cnt) == 0) && (addr == 1)) {
				int16_t pres = get_pres(2);
					pres = (pres < 0) ? 0 : pres;
					response_buff[0] = usart_buff[0];
					response_buff[1] = usart_buff[1];
					response_buff[2] = 0x02;
					response_buff[3] = pres>>8;
					response_buff[4] = pres&0xFF;
					mb_crc = mb_crc16(response_buff, 5);
					response_buff[5] = mb_crc & 0xFF;
					response_buff[6] = mb_crc >> 8;
					HAL_UART_Transmit_IT(&huart1, response_buff, 7);	
					HAL_GPIO_WritePin(RS485_TX_LED_GPIO_Port, RS485_TX_LED_Pin, GPIO_PIN_RESET);	
				} else if((strncmp((const char *)usart_buff, cmd_read_statu2, receive_cnt) == 0) && (addr == 2)) {
					response_buff[0] = usart_buff[0];
					response_buff[1] = usart_buff[1];
					response_buff[2] = 0x02;
					response_buff[3] = 0;
					response_buff[4] = (ac_state.pump1_stop<<0) | (ac_state.ac_sw1<<1) | (ac_state.pump1_fail<<2) | (ac_state.pump1_start)\
													 | (ac_state.pump2_stop<<4) | (ac_state.ac_sw2<<5) | (ac_state.pump2_fail<<6) | (ac_state.pump2_start);
					mb_crc = mb_crc16(response_buff, 5);
					response_buff[5] = mb_crc & 0xFF;
					response_buff[6] = mb_crc >> 8;
					HAL_UART_Transmit_IT(&huart1, response_buff, 7);
					HAL_GPIO_WritePin(RS485_TX_LED_GPIO_Port, RS485_TX_LED_Pin, GPIO_PIN_RESET);
				} else if((strncmp((const char *)usart_buff, cmd_read_pres3, receive_cnt) == 0) && (addr == 2)) {
				int16_t pres = get_pres(1);
					pres = (pres < 0) ? 0 : pres;
					response_buff[0] = usart_buff[0];
					response_buff[1] = usart_buff[1];
					response_buff[2] = 0x02;
					response_buff[3] = pres>>8;
					response_buff[4] = pres&0xFF;
					mb_crc = mb_crc16(response_buff, 5);
					response_buff[5] = mb_crc & 0xFF;
					response_buff[6] = mb_crc >> 8;
					HAL_UART_Transmit_IT(&huart1, response_buff, 7);	
					HAL_GPIO_WritePin(RS485_TX_LED_GPIO_Port, RS485_TX_LED_Pin, GPIO_PIN_RESET);
				} else if((strncmp((const char *)usart_buff, cmd_read_pres4, receive_cnt) == 0) && (addr == 2)) {
				int16_t pres = get_pres(2);
					pres = (pres < 0) ? 0 : pres;
					response_buff[0] = usart_buff[0];
					response_buff[1] = usart_buff[1];
					response_buff[2] = 0x02;
					response_buff[3] = pres>>8;
					response_buff[4] = pres&0xFF;
					mb_crc = mb_crc16(response_buff, 5);
					response_buff[5] = mb_crc & 0xFF;
					response_buff[6] = mb_crc >> 8;
					HAL_UART_Transmit_IT(&huart1, response_buff, 7);	
					HAL_GPIO_WritePin(RS485_TX_LED_GPIO_Port, RS485_TX_LED_Pin, GPIO_PIN_RESET);
				} else if(((addr==1)&&((usart_buff[0]==1)||(usart_buff[0]==0x0b)||(usart_buff[0]==0x0c)))\
							|| ((addr==2)&&((usart_buff[0]==2)||(usart_buff[0]==0x0d)||(usart_buff[0]==0x0e)))){
					response_buff[0] = usart_buff[0];
					response_buff[1] = usart_buff[1]|0x80;
					response_buff[2] = 0x01;
					mb_crc = mb_crc16(response_buff, 3);
					response_buff[3] = mb_crc & 0xFF;
					response_buff[4] = mb_crc >> 8;
					HAL_UART_Transmit_IT(&huart1, response_buff, 5);
					HAL_GPIO_WritePin(RS485_TX_LED_GPIO_Port, RS485_TX_LED_Pin, GPIO_PIN_RESET);
				}
			}
			usart_rx_cplt = 0;		
			HAL_UART_Receive_IT(&huart1, usart_buff, 32);
		}
    osDelayUntil(&PreviousWakeTime, 20);
		HAL_GPIO_WritePin(RS485_RX_LED_GPIO_Port, RS485_RX_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RS485_TX_LED_GPIO_Port, RS485_TX_LED_Pin, GPIO_PIN_SET);
  }
  /* USER CODE END sampleTask */
}

/* DisplayTask function */
void DisplayTask(void const * argument)
{
  /* USER CODE BEGIN DisplayTask */
volatile u_led led;
uint32_t PreviousWakeTime = osKernelSysTick();	

	led = hc595_init();
	
  /* Infinite loop */
  for(;;)
  {
		ac_state = get_ac_state();
		//AC voltage 220V with 20% offset
		if((avg_ac>=594) && (avg_ac<=891)) {
			ac_state.ac_pwr = 1;
		}
		dc_state = get_dc_state();
		led.val = 0;
		//AC indicator leds
		
		if(ac_state.ac_pwr) {
			led.pump_pwr = 1;
		} else {
			led.pump_pwr = 0;
		}
		
		if(ac_state.pump1_fail) {
			led.pump1_green = led.pump1_red = 1;
		} else {
			led.pump1_green = ac_state.pump1_start ? 1 : 0;
			led.pump1_red = ac_state.pump1_stop ? 1 : 0;
		}
		if(ac_state.pump2_fail) {
			led.pump2_green = led.pump2_red = 1;
		} else {
			led.pump2_green = ac_state.pump2_start ? 1 : 0;
			led.pump2_red = ac_state.pump2_stop ? 1 : 0;
		}
		//pump3 no fail signal
		led.pump3_green = ac_state.pump3_start ? 1 : 0;
		led.pump3_red = ac_state.pump3_stop ? 1 : 0;
		
		//DC indicator leds
		led.volt_in1 = dc_state.di_ext_in1 ? 1 : 0;
		led.volt_in2 = dc_state.di_ext_in2 ? 1 : 0;
		
		//led on when current between 4~20mA.
		if((avg_cs1>=496)&&(avg_cs1<=2482)) {
			led.curr_in1 = 1;
		} else {
			led.curr_in1 = 0;
		}
		if((avg_cs2>=496)&&(avg_cs2<=2482)) {
			led.curr_in2 = 1;
		} else {
			led.curr_in2 = 0;
		}
		
		//
		if((get_pres(1) < sys_para.pres1_min) || (get_pres(1) > sys_para.pres1_max) || \
			 (get_pres(2) < sys_para.pres2_min) || (get_pres(2) > sys_para.pres2_max)) {
			led.sys_err = 1;
		} else {
			led.sys_err = 0;
		}
		
		hc595_write(led.val);
    osDelayUntil(&PreviousWakeTime, 100);
  }
  /* USER CODE END DisplayTask */
}

/* configTask function */
void configTask(void const * argument)
{
  /* USER CODE BEGIN configTask */
FLASH_EraseInitTypeDef EraseInitStruct;	
uint32_t PageError;
uint32_t PreviousWakeTime = osKernelSysTick();	
uint8_t receive_cnt;
	
	HAL_UART_Receive_IT(&huart2, debug_buff, DEBUG_BUFF_MAX);
	SET_BIT(huart2.Instance->ICR, USART_ICR_IDLECF);
  SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);
	
  /* Infinite loop */
  for(;;)
  {
		if(debug_rx_cplt) {
			receive_cnt = DEBUG_BUFF_MAX-huart2.RxXferCount; 
			HAL_UART_AbortReceive_IT(&huart2);
			
			if((receive_cnt == 6) && (strncmp((const char *)debug_buff, "BEACON", 6) == 0)) {
					memset(response_buff, 0, 40);
					sprintf((char *)response_buff, "B %s %s %03d %04d %04d %04d %02d ", HW_VER, FW_VER, get_ac220(), \
									get_pres(1), get_pres(2), get_ac_state().val, get_dc_state().val);
					HAL_GPIO_WritePin(RS485_TX_LED_GPIO_Port,RS485_TX_LED_Pin, GPIO_PIN_RESET);
					HAL_UART_Transmit_IT(&huart2, response_buff, 40);			
			}
			
			if(receive_cnt == 32) {
				HAL_GPIO_WritePin(RS485_RX_LED_GPIO_Port,RS485_RX_LED_Pin, GPIO_PIN_RESET);
				switch(debug_buff[0]) {
					case 'R':
						if(debug_buff[31] != 'R') {
							break;
						}						
						memset(response_buff, 0, 32);
						response_buff[0] = 'R';
						memcpy(response_buff+1, &sys_para, sizeof(s_sys_para));
						HAL_GPIO_WritePin(RS485_TX_LED_GPIO_Port,RS485_TX_LED_Pin, GPIO_PIN_RESET);
						HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
						HAL_UART_Transmit_IT(&huart2, response_buff, 32);						
						break;
					case 'W':
						if(debug_buff[31] != 'W') {
							break;
						}						
						memset(response_buff, 0, 32);
						response_buff[0] = 'W';
						HAL_FLASH_Unlock();
						EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
						EraseInitStruct.PageAddress = parama_save_addr;
						EraseInitStruct.NbPages = 1;
						HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
					  memcpy(&sys_para, debug_buff+1, sizeof(s_sys_para));
						HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, parama_save_addr, *(uint64_t *)&sys_para);
						HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, parama_save_addr+8, *((uint64_t *)&sys_para+1));
						HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, parama_save_addr+16, *(uint16_t *)(&sys_para+16));
						HAL_FLASH_Lock();
						HAL_GPIO_WritePin(RS485_TX_LED_GPIO_Port,RS485_TX_LED_Pin, GPIO_PIN_RESET);
						HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
						HAL_UART_Transmit_IT(&huart2, response_buff, 32);						
						break;
					default:
						break;
				}
			}
			
			debug_rx_cplt = 0;
			HAL_UART_Receive_IT(&huart2, debug_buff, DEBUG_BUFF_MAX);
		}
    osDelayUntil(&PreviousWakeTime, 100);
		HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(RS485_TX_LED_GPIO_Port,RS485_TX_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RS485_RX_LED_GPIO_Port,RS485_RX_LED_Pin, GPIO_PIN_SET);
  }
  /* USER CODE END configTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
