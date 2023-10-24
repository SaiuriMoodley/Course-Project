/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint32_t curr_millis = 0;
uint32_t prev_millis = 0;
uint32_t start_millis = 0;
uint32_t adc_val = 0;
uint32_t temp = 0;
uint32_t tempCount = 0;


uint32_t dataSize = 12;
uint8_t code = 0;

uint32_t LUT[12] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048};

char adc[16];
char val[16];
uint8_t binary[12];
uint8_t countB[12];
uint8_t count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void write(char[]);
void pollADC();
void sendStart(void);
void receiveStart(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  init_LCD();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t delay = 50;
  lcd_command(CLEAR);
  lcd_putstring("Idle");
  while (1)
  {
	 if (code == 1){

		lcd_command(CLEAR);
		lcd_putstring("Transmitting ADC");
		HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay);
		count++;

		for(uint8_t i=12; i>0; i--){
			if (binary[i-1] == 1)
				HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
			HAL_Delay(delay);
		}
		HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);

		lcd_command(CLEAR);
		sprintf(adc, "ADC value %lu", adc_val);
		lcd_putstring(adc);
		HAL_Delay (5000);
		code=0;
		lcd_command(CLEAR);
		lcd_putstring("Idle");
	 }
	 else if (code == 2){
		lcd_command(CLEAR);
		lcd_putstring("Transmitting CP");
		HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
		HAL_Delay(delay);

		for(uint8_t i=12; i>0; i--){
			if (countB[i-1] == 1)
				HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
			HAL_Delay(delay);
		}
		HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);

		lcd_command(CLEAR);
		sprintf(adc, "Sent %lu", count);
		lcd_putstring(adc);
		HAL_Delay (5000);
		code=0;
		lcd_command(CLEAR);
		lcd_putstring("Idle");
	 	 }


	  //receive code
//	  HAL_Delay (5000);
//	  HAL_GPIO_TogglePin(GPIOB, LED5_Pin);
//	  receiveStart();

	  //send code
//	  HAL_Delay (2000);
//	  sendStart();
	    //HAL_GPIO_TogglePin(GPIOB, LED7_Pin);
//	  	HAL_GPIO_WritePin(GPIOB, LED6_Pin, HAL_GPIO_ReadPin(GPIOA, Button0_Pin));
//	  	HAL_Delay (delay_t);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 47999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED5_Pin|LED7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button0_Pin Button1_Pin */
  GPIO_InitStruct.Pin = Button0_Pin|Button1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED5_Pin LED7_Pin */
  GPIO_InitStruct.Pin = LED5_Pin|LED7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	curr_millis = HAL_GetTick();
	if((curr_millis-prev_millis)>300){
		prev_millis = curr_millis;

		if(GPIO_Pin==Button0_Pin){
			pollADC();
			temp = adc_val;
			for(uint8_t i=12; i>0; i--){

				binary[i-1] = temp/LUT[i-1];
				if ( binary[i-1] == 1)
					temp -= LUT[i-1];
			}
			code=1;
		}
		else{
			tempCount = count;
			for(uint8_t i=12; i>0; i--){

						countB[i-1] = tempCount/LUT[i-1];
						if ( countB[i-1] == 1)
							tempCount -= LUT[i-1];
					}
			code=2;
		}
	}
}

void sendStart(){
	lcd_command(CLEAR);
	lcd_putstring("Start send");
	HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
	HAL_Delay (500);
	HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
	lcd_command(CLEAR);
	lcd_putstring("End send");
}

//void receiveStart(){
//	start_millis = HAL_GetTick();
//	lcd_command(CLEAR);
//	HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
//	while(HAL_GPIO_ReadPin(GPIOA, Input_Pin) == GPIO_PIN_SET){
//		if (HAL_GetTick() - start_millis > 50){
//			lcd_putstring("Received");
//			return;
//		}
//		HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
//	}
//	lcd_putstring("Failed");
//}

void pollADC(){
	HAL_ADC_Start(&hadc);
	adc_val = HAL_ADC_GetValue(&hadc);
//	adc_val = 4095;
	HAL_ADC_Stop(&hadc);
}

void write(char * val){
	lcd_command(CLEAR);
	lcd_putstring(val);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
