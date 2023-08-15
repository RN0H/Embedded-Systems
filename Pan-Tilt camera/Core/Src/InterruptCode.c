///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2023 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include <stdio.h>
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim3;
//UART_HandleTypeDef huart1;
//
//
//uint8_t Buffer[32];
//uint8_t RxChar[1];
//static short readFlag = 0;
//static uint8_t idx = 0;
//
///* USER CODE BEGIN PV */
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_USART1_UART_Init(void);
//static void MX_TIM2_Init(void);
//static void MX_TIM3_Init(void);
///* USER CODE BEGIN PFP */
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  UNUSED(huart);
//
//// DATA FORMAT : $U\n
//
//  if (huart->Instance == USART1){
//	  if (RxChar[0] == '\n')
//	  {
//		  HAL_UART_Transmit(&huart1, Buffer, sizeof(Buffer), 10);
//		  memset(Buffer, 0x0, sizeof(Buffer));
//		  readFlag = 0;
//		  idx=0;
//	  }
//	  else if (readFlag == 1)
//		  Buffer[idx++] = RxChar[0];
//	  else if (RxChar[0] == '$')
//		  readFlag = 1;
//  }
//}
//
//int _write(int file, char *ptr, int len){
//
//	for (int i = 0; i < len; ++i){
//		ITM_SendChar(*ptr++);
//	}
//	return len;
//
//}
//
//static inline void Yaw(){
//
//	memset(Buffer, 0x0, 32);
//	memcpy(Buffer, "Yaw", 32);
//    HAL_UART_Transmit(&huart1, Buffer, sizeof(Buffer), HAL_MAX_DELAY);
//    HAL_Delay(200);
//
//			  htim2.Instance->CCR1 = 25 ; // (25 % of duty cycle) * 20ms = 0.5ms = 0deg
//			  HAL_Delay(50);
//			  htim2.Instance->CCR1 = 75; // (125 % of duty cycle) * 20ms = 2.5ms = 180deg
//			  HAL_Delay(50);
//			  htim2.Instance->CCR1 = 79;
//			  HAL_Delay(50);
//			  htim2.Instance->CCR1 = 75;
//}
//
//static inline void Pitch(){
//
//	memset(Buffer, 0x0, 32);
//	memcpy(Buffer, "Pitch", 32);
//    HAL_UART_Transmit(&huart1, Buffer, sizeof(Buffer), HAL_MAX_DELAY);
//    HAL_Delay(200);
//
//	  htim3.Instance->CCR1 = 25 ; // (25 % of duty cycle) * 20ms = 0.5ms = 0deg
//	  HAL_Delay(50);
//	  htim3.Instance->CCR1 = 75; // (125 % of duty cycle) * 20ms = 2.5ms = 180deg
//	  HAL_Delay(50);
//	  htim3.Instance->CCR1 = 79;
//	  HAL_Delay(50);
//	  htim3.Instance->CCR1 = 75;
//
//}
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART1_UART_Init();
//  MX_TIM2_Init();
//  MX_TIM3_Init();
//  /* USER CODE BEGIN 2 */
//
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//
//
//  memset(RxChar, 0x0, 1);
//  while (1)
//  {
//
//
//	  HAL_UART_Receive_IT(&huart1, RxChar, 1);
//	  memset(RxChar, 0x0, 1);
//	  HAL_Delay(100);
////	  HAL_UART_Receive_IT(&huart1, Buffer, sizeof(Buffer));
////	        HAL_Delay(100);
////
////            if (Buffer[0] == "U")
////            	htim3.Instance->CCR1 = 25;
////			if (Buffer[0] ==  "D")
////				htim3.Instance->CCR1 = 79;
////			if (Buffer[0] == "L")
////				htim2.Instance->CCR1 = 25;
////			if (Buffer[0] == "R")
////				htim2.Instance->CCR1 = 79;
////			if (Buffer[0] == "S")
////			{
////				htim3.Instance->CCR1 = 75;
////				htim2.Instance->CCR1 = 75;
////			}
//
////	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET){
//
////		  memset(Buffer, 0x0, 32);
////		  HAL_UART_Receive_IT(&huart1, Buffer, sizeof(Buffer));  //interrupt
////		  HAL_Delay(1000);
////		  HAL_UART_Transmit_IT(&huart1, Buffer, sizeof(Buffer));
////		  Yaw();
////		  Pitch();
//
////		  memset(Buffer, 0x0, sizeof(Buffer));
////		  memcpy(Buffer, "0deg\n", 32);
////		  HAL_UART_Transmit(&huart1, Buffer, sizeof(Buffer), HAL_MAX_DELAY);
////		  HAL_Delay(200);
//
//		  //memset(Buffer, 0x0, sizeof(Buffer));
////		  memcpy(Buffer, "90deg\n", 32);
////		  HAL_UART_Transmit(&huart1, Buffer, sizeof(Buffer), HAL_MAX_DELAY);
////		  HAL_Delay(200);
////		  htim2.Instance->CCR1 = 75; // (75 % of duty cycle) * 20ms = 1.5ms = 90deg
////		  HAL_Delay(2000);
////
////		  memset(Buffer, 0x0, sizeof(Buffer));
////		  memcpy(Buffer, "180deg\n", 32);
////		  HAL_UART_Transmit(&huart1, Buffer, sizeof(Buffer), HAL_MAX_DELAY);
////		  HAL_Delay(200);
////		  htim2.Instance->CCR1 = 112; // (125 % of duty cycle) * 20ms = 2.5ms = 180deg
////		  HAL_Delay(2000);
//
////	 }
//
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 4;
//  RCC_OscInitStruct.PLL.PLLN = 160;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 4;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief TIM2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM2_Init(void)
//{
//
//  /* USER CODE BEGIN TIM2_Init 0 */
//
//  /* USER CODE END TIM2_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM2_Init 1 */
//
//  /* USER CODE END TIM2_Init 1 */
//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = 800;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 1000-1;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM2_Init 2 */
//
//  /* USER CODE END TIM2_Init 2 */
//  HAL_TIM_MspPostInit(&htim2);
//
//}
//
///**
//  * @brief TIM3 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM3_Init(void)
//{
//
//  /* USER CODE BEGIN TIM3_Init 0 */
//
//  /* USER CODE END TIM3_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM3_Init 1 */
//
//  /* USER CODE END TIM3_Init 1 */
//  htim3.Instance = TIM3;
//  htim3.Init.Prescaler = 800;
//  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim3.Init.Period = 1000-1;
//  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM3_Init 2 */
//
//  /* USER CODE END TIM3_Init 2 */
//  HAL_TIM_MspPostInit(&htim3);
//
//}
//
///**
//  * @brief USART1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART1_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART1_Init 0 */
//
//  /* USER CODE END USART1_Init 0 */
//
//  /* USER CODE BEGIN USART1_Init 1 */
//
//  /* USER CODE END USART1_Init 1 */
//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART1_Init 2 */
//
//  /* USER CODE END USART1_Init 2 */
//
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
///* USER CODE BEGIN MX_GPIO_Init_1 */
///* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//
//  /*Configure GPIO pin : PC1 */
//  GPIO_InitStruct.Pin = GPIO_PIN_1;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
///* USER CODE BEGIN MX_GPIO_Init_2 */
///* USER CODE END MX_GPIO_Init_2 */
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
