/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile unsigned char lookupIndex = 0;
//int lookup[] = {3600, 3599, 3597, 3595, 3591, 3586, 3580, 3573,
//		3565, 3556, 3546, 3534, 3522, 3509, 3494, 3479,
//		3462, 3445, 3427, 3407, 3387, 3366, 3343, 3320,
//		3296, 3271, 3245, 3219, 3191, 3162, 3133, 3103,
//		3072, 3041, 3008, 2975, 2941, 2907, 2872, 2836,
//		2800, 2762, 2725, 2687, 2648, 2609, 2569, 2529,
//		2488, 2447, 2406, 2364, 2322, 2280, 2237, 2194,
//		2151, 2107, 2064, 2020, 1976, 1932, 1888, 1844,
//		1800, 1755, 1711, 1667, 1623, 1579, 1535, 1492,
//		1448, 1405, 1362, 1319, 1277, 1235, 1193, 1152,
//		1111, 1070, 1030, 990, 951, 912, 874, 837,
//		799, 763, 727, 692, 658, 624, 591, 558,
//		527, 496, 466, 437, 408, 380, 354, 328,
//		303, 279, 256, 233, 212, 192, 172, 154,
//		137, 120, 105, 90, 77, 65, 53, 43,
//		34, 26, 19, 13, 8, 4, 2, 0,
//		0, 0, 2, 4, 8, 13, 19, 26,
//		34, 43, 53, 65, 77, 90, 105, 120,
//		137, 154, 172, 192, 212, 233, 256, 279,
//		303, 328, 354, 380, 408, 437, 466, 496,
//		527, 558, 591, 624, 658, 692, 727, 763,
//		799, 837, 874, 912, 951, 990, 1030, 1070,
//		1111, 1152, 1193, 1235, 1277, 1319, 1362, 1405,
//		1448, 1492, 1535, 1579, 1623, 1667, 1711, 1755,
//		1800, 1844, 1888, 1932, 1976, 2020, 2064, 2107,
//		2151, 2194, 2237, 2280, 2322, 2364, 2406, 2447,
//		2488, 2529, 2569, 2609, 2648, 2687, 2725, 2762,
//		2800, 2836, 2872, 2907, 2941, 2975, 3008, 3041,
//		3072, 3103, 3133, 3162, 3191, 3219, 3245, 3271,
//		3296, 3320, 3343, 3366, 3387, 3407, 3427, 3445,
//		3462, 3479, 3494, 3509, 3522, 3534, 3546, 3556,
//		3565, 3573, 3580, 3586, 3591, 3595, 3597, 3599};

int lookup [] = {540, 539, 539, 539, 538, 537, 537, 536,
		534, 533, 531, 530, 528, 526, 524, 521,
		519, 516, 514, 511, 508, 504, 501, 498,
		494, 490, 486, 482, 478, 474, 470, 465,
		460, 456, 451, 446, 441, 436, 430, 425,
		420, 414, 408, 403, 397, 391, 385, 379,
		373, 367, 360, 354, 348, 342, 335, 329,
		322, 316, 309, 303, 296, 289, 283, 276,
		270, 263, 256, 250, 243, 236, 230, 223,
		217, 210, 204, 197, 191, 185, 179, 172,
		166, 160, 154, 148, 142, 136, 131, 125,
		119, 114, 109, 103, 98, 93, 88, 83,
		79, 74, 69, 65, 61, 57, 53, 49,
		45, 41, 38, 35, 31, 28, 25, 23,
		20, 18, 15, 13, 11, 9, 8, 6,
		5, 3, 2, 2, 1, 0, 0, 0,
		0, 0, 0, 0, 1, 2, 2, 3,
		5, 6, 8, 9, 11, 13, 15, 18,
		20, 23, 25, 28, 31, 35, 38, 41,
		45, 49, 53, 57, 61, 65, 69, 74,
		79, 83, 88, 93, 98, 103, 109, 114,
		119, 125, 131, 136, 142, 148, 154, 160,
		166, 172, 179, 185, 191, 197, 204, 210,
		217, 223, 230, 236, 243, 250, 256, 263,
		270, 276, 283, 289, 296, 303, 309, 316,
		322, 329, 335, 342, 348, 354, 360, 367,
		373, 379, 385, 391, 397, 403, 408, 414,
		420, 425, 430, 436, 441, 446, 451, 456,
		460, 465, 470, 474, 478, 482, 486, 490,
		494, 498, 501, 504, 508, 511, 514, 516,
		519, 521, 524, 526, 528, 530, 531, 533,
		534, 536, 537, 537, 538, 539, 539, 539};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void GPIO_Config(void);
void Timer1_Config(void);
void Timer6_Config(void);
void ADC_Config(void);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  GPIO_Config();
  Timer1_Config();
  Timer6_Config();
  ADC_Config();

  TIM6->ARR = 300;
  TIM6->DIER |= TIM_DIER_UIE;

  NVIC_SetPriority(TIM6_DAC_IRQn, 0x03);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);


  /* USER CODE END 2 */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void TIM6_IRQHandler(void){

//	 ADC1->CR |= ADC_CR_ADSTART; //start conversion

//	 // wait for conversion
//	 while(!(ADC1->ISR & ADC_ISR_EOC));
//	 TIM6->ARR = ADC1->DR > 0x10 ? ADC1->DR : 0x10;

//	 while ( !( ADC1->ISR & ADC_ISR_EOS ) ) {};
//	 ADC1->ISR |=  ( ADC_ISR_EOS );

	 TIM1->CCR1 = lookup[lookupIndex];
	 TIM1->CCR2 = lookup[(unsigned char)(lookupIndex + 85)];
	 TIM1->CCR3 = lookup[(unsigned char)(lookupIndex + 170)];
	 lookupIndex++;
	 TIM6->SR &= ~(TIM_SR_UIF);
}

void GPIO_Config(void){
	  /*
	   * GPIO Setup
	   * Phase 1 - Channel 1 - PA8:PA7
	   * Phase 2 - Channel 2 - PA9:PB0
	   * Phase 3 - Channel 3 - PA10:PB1
	   */

	  // Alternate Function Mode
	  //GPIOA->MODER &= ~((1 << MODE100) | (1 << MODE90) | (1 << MODE80) | (1 << MODE70));
	  GPIOA->MODER &= ~(GPIO_MODER_MODE10_0 | GPIO_MODER_MODE9_0 | GPIO_MODER_MODE8_0 | GPIO_MODER_MODE7_0);
	  GPIOB->MODER &= ~(GPIO_MODER_MODE1_0 | GPIO_MODER_MODE0_0);

	  //Set Alternate Function (AF1 for All)
	  GPIOA->AFR[0] |= GPIO_AFRL_AFSEL7_0;
	  GPIOA->AFR[1] |= GPIO_AFRH_AFSEL10_0 | GPIO_AFRH_AFSEL9_0 | GPIO_AFRH_AFSEL8_0;
	  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL1_0 | GPIO_AFRL_AFSEL0_0;
}

void Timer1_Config(void){
	 // Timer1 Setup
	  RCC->APB2ENR = RCC_APB2ENR_TIM1EN; //enable TIM1 clock

	  // frequency = CLK/PSC/ARR
	  //Duty_Cycle = CCR1/ARR
	  TIM1->PSC = 1-1;
	  TIM1->ARR = 3600-1;
	  TIM1->CCR1 = 1800;
	  TIM1->CCR2 = 1800;
	  TIM1->CCR3 = 1800;

	  TIM1->EGR |= TIM_EGR_UG; // Update registers

	  // Enable PWM Mode
	  TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE; //(1 << OC1PE); // Mode 1 | preload enable x2
	  TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE; // Mode 1 | preload enable

	  TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; // enable preload | enable counter

	  TIM1->CCER |= TIM_CCER_CC3NE | TIM_CCER_CC3E | TIM_CCER_CC2NE | TIM_CCER_CC2E | TIM_CCER_CC1NE | TIM_CCER_CC1E; // Enable Channel 1 Output
	  TIM1->CCER |= TIM_CCER_CC3NP | TIM_CCER_CC3P | TIM_CCER_CC2NP | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC1P; // Active Low
	  TIM1->BDTR |= TIM_BDTR_MOE;

	  //TIM1->DIER |= TIM_DIER_UIE;
}

void Timer6_Config(void){
	  RCC->APB1ENR1 |= (1 << 4); //enable clock for timer 6

	  TIM6->PSC = 10-1; //prescaler
	  TIM6->ARR = 0xFFFF;
	  TIM6->CR1 |= (1 << 0); //enable timer
	  TIM6->EGR |= (1 << 0);
}

void ADC_Config(void){
	// adc (PA1 - ADCIN_6)

	  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; // enable adc clock
	  RCC->CCIPR = (0b11 << RCC_CCIPR_ADCSEL_Pos);
	  //GPIOA->MODER &= ~(GPIO_MODER_MODE1_1 | GPIO_MODER_MODE1_0); // PA1 as input

	  //ADC1->CCR |= ADC_CCR_CKMODE_1;

	  ADC1->CR &= ~ADC_CR_DEEPPWD; // exit power down mode
	  ADC1->CR |= ADC_CR_ADVREGEN; //enable voltage regulator

	  //wait for voltage regulator (20us)
	  TIM6->CNT = 0;
	  while ((TIM6->CNT <2000));

	  //ADC1->SQR1 |= ADC_SQR1_L_0; // 2 conversions
	  ADC1->SQR1 |= (6 << ADC_SQR1_SQ1_Pos);// first conversion on channel 6 (PA7)

	  ADC1->SMPR1 |= (2 << ADC_SMPR1_SMP6_Pos); //12 cycles for channel 6

	  ADC1->CFGR |= ADC_CFGR_ALIGN;

	  ADC1->CR |= ADC_CR_ADCAL; //calibrate

	  //wait for calibration
	  while (ADC1->CR & ADC_CR_ADCAL){
	  }

	  ADC1->CR |= ADC_CR_ADEN; //enable ADC
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
