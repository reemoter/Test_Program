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
#include <stdio.h>
#include <string.h>
#include "stdbool.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUF 10
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t rx_data;
char data_arr[RX_BUF] = "";
char data_cmd[RX_BUF] = "";
uint8_t idx;

uint8_t tim_flag;

struct speed{
	uint8_t mt1;
	uint8_t mt2;
	uint8_t mt3;
}motor;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Command_Access();
void Motor_Test_Function();
void Motor_Reset();
void Motor_Drive();
void Motor_Stop();
void Motor_Dir(uint8_t dir);
void Motor_Spd(uint16_t spd);

//update 23.06.09
void Motor_Update();

void Msg_Help();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __cplusplus
extern "C" int _write(int32_t file, uint8_t *ptr, int32_t len) {
#else
	int _write(int32_t file, uint8_t *ptr, int32_t len) {
#endif
		if( HAL_UART_Transmit(&huart2, ptr, len, len) == HAL_OK ) return len;
		else return 0;
	}

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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  HAL_UART_Receive_IT(&huart3, &rx_data, 1);
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);

  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, RESET);
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, SET);
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, SET);

  printf("Program has been reset.\r\n");
  printf("Ver.1.0.0.\r\n");
  Motor_Reset();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //printf("Hello  World\r\n");
	  if(tim_flag >= 10){
		  Motor_Update();
		  tim_flag = 0;
	  }
	  //Motor_Test_Function();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 108-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 108-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 112500;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MRD_Pin|MRB_Pin|MLD_Pin|MLB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MRD_Pin MRB_Pin MLD_Pin MLB_Pin */
  GPIO_InitStruct.Pin = MRD_Pin|MRB_Pin|MLD_Pin|MLB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void Motor_Update(){
	htim3.Instance->CCR1 = motor.mt1;
	htim3.Instance->CCR2 = motor.mt2;
}

void Motor_Test_Function(){
	//Motor Control Code for connection check(Not Essential)
	HAL_GPIO_WritePin(MLD_GPIO_Port, MLD_Pin, SET);
	HAL_GPIO_WritePin(MLB_GPIO_Port, MLB_Pin, RESET);

	HAL_GPIO_WritePin(MRD_GPIO_Port, MRD_Pin, RESET);
	HAL_GPIO_WritePin(MRB_GPIO_Port, MRB_Pin, RESET);
}

void Motor_Reset(){
	HAL_GPIO_WritePin(MLD_GPIO_Port, MLD_Pin, SET);
	HAL_GPIO_WritePin(MLB_GPIO_Port, MLB_Pin, SET);
	htim3.Instance->CCR1 = 50;

	HAL_GPIO_WritePin(MRD_GPIO_Port, MRD_Pin, SET);
	HAL_GPIO_WritePin(MRB_GPIO_Port, MRB_Pin, SET);
	htim3.Instance->CCR2 = 50;
}

void Motor_Drive(){
	HAL_GPIO_WritePin(MLB_GPIO_Port, MLB_Pin, RESET);
	HAL_GPIO_WritePin(MRB_GPIO_Port, MRB_Pin, RESET);
}

void Motor_Stop(){
	HAL_GPIO_WritePin(MLB_GPIO_Port, MLB_Pin, SET);
	HAL_GPIO_WritePin(MRB_GPIO_Port, MRB_Pin, SET);
}

void Motor_Dir(uint8_t dir){
	if(dir == 0){
		HAL_GPIO_WritePin(MRD_GPIO_Port, MRD_Pin, SET);
		HAL_GPIO_WritePin(MLD_GPIO_Port, MLD_Pin, SET);
	}else if(dir == 1){
		HAL_GPIO_WritePin(MRD_GPIO_Port, MRD_Pin, RESET);
		HAL_GPIO_WritePin(MLD_GPIO_Port, MLD_Pin, RESET);
	}else if(dir == 2){
		HAL_GPIO_WritePin(MRD_GPIO_Port, MRD_Pin, SET);
		HAL_GPIO_WritePin(MLD_GPIO_Port, MLD_Pin, RESET);
	}else if(dir == 3){
		HAL_GPIO_WritePin(MRD_GPIO_Port, MRD_Pin, RESET);
		HAL_GPIO_WritePin(MLD_GPIO_Port, MLD_Pin, SET);
	}
}

void Motor_Spd(uint16_t spd){
	htim3.Instance->CCR1 = spd;
	htim3.Instance->CCR2 = spd;
}

void Msg_Help(){
	printf("Command List\r\n");
	printf(">=========================\r\n");
	printf("*Command Example\r\n");
	printf("XXX000	XXX : Command, 000 : Value\r\n");
	printf("\r\n");
	printf("RST	: Motor Reset\r\n");
	printf("RUN	: Motor Operating\r\n");
	printf("BRK	: Brake On\r\n");

	printf("DIR0 	: Direction Changing\r\n");
	printf("   0	: forward \r\n");
	printf("   1	: reverse \r\n");
	printf("   2	: Rotate1 \r\n");
	printf("   3	: Rotate2 \r\n");

	printf("SPD000	: Speed Changing. Max100\r\n");
	printf("STS	: Check Motor Status.\r\n");

	printf("MLD0	: Left Motor Direction Changing\r\n");
	printf("   0	: forward \r\n");
	printf("   1	: reverse \r\n");

	printf("MRD0	: Right Motor Direction Changing\r\n");
	printf("   0	: forward \r\n");
	printf("   1	: reverse \r\n");

	printf("MLS000	: Left Motor Direction Changing\r\n");
	printf("MRS000	: Right Motor Direction Changing\r\n");

	printf("\r\n");
	printf("=========================<\r\n");
}

void Command_Access(){
	strncpy(data_cmd, data_arr+3, 4);
	data_cmd[3] = '\0';

	if(data_arr[idx-1]== '\r' || data_arr[idx-1]== '\n')
	{
		data_arr[idx] = '\0';

		if(strncmp((char*)data_arr, "RST", 3) == 0){
			Motor_Reset();
			printf("Device has been reset...\r\n");
			printf("Brakes	--> On\r\n");
			printf("Dir 	--> forward\r\n");
			printf("Speed	--> 500.\r\n");
			printf("(Maximum 100)\r\n");
		}else if(strncmp((char*)data_arr, "RUN", 3) == 0){
			Motor_Drive();
			printf("Motor Running...!\r\n");
			printf("Brakes	--> On\r\n");
			printf("Speed	--> %d.\r\n", htim3.Instance->CCR1);
			printf("(Maximum 100)\r\n");
		}if(strncmp((char*)data_arr, "BRK", 3) == 0){
			Motor_Stop();
			printf("Motor Stop...\r\n");
			printf("Brakes	--> Off\r\n");
		}else if(strncmp((char*)data_arr, "DIR", 3) == 0){
			if(atoi(data_cmd) <= 3){
				printf("Direction is Setting!!\r\n");
				if(atoi(data_cmd) ==  0)		printf("Forward Mode");
				else if(atoi(data_cmd) ==  1)	printf("Reverse Mode");
				else if(atoi(data_cmd) ==  2)	printf("Right Turn Mode");
				else if(atoi(data_cmd) ==  3)	printf("Left Turn Mode");
				Motor_Dir(atoi(data_cmd));
			}else{
				printf("Wrong Direction Command.\r\n");
			}
		}else if(strncmp((char*)data_arr, "SPD", 3) == 0){
			if(atoi(data_cmd) <= 100){
				motor.mt1 = atoi(data_cmd);
				motor.mt2 = atoi(data_cmd);
				printf("Speed Setting!\r\n");
			}else{
				printf("Wrong Speed Command.\r\n");
			}
		}else if(strncmp((char*)data_arr, "STS", 3) == 0){
			printf("Status Check\r\n\n");
			printf("Left Motor\r\n");
			printf("==================>\r\n");
			printf("Speed = %d\r\n", htim3.Instance->CCR1);
			printf("Brake = %d\r\n", HAL_GPIO_ReadPin(MLB_GPIO_Port, MLB_Pin));
			printf("Direction = %d\r\n", HAL_GPIO_ReadPin(MLD_GPIO_Port, MLD_Pin));
			printf("<==================\r\n");

			printf("Right Motor\r\n");
			printf("==================>\r\n");
			printf("Speed = %d\r\n", htim3.Instance->CCR2);
			printf("Brake = %d\r\n", HAL_GPIO_ReadPin(MRB_GPIO_Port, MRB_Pin));
			printf("Direction = %d\r\n", HAL_GPIO_ReadPin(MRD_GPIO_Port, MRD_Pin));
			printf("<==================\r\n");
		}else if(strncmp((char*)data_arr, "TST", 3) == 0){
			printf("Test Function.\r\n");
			Motor_Test_Function();
		}else if(strncmp((char*)data_arr, "MLD", 3) == 0){
			if(atoi(data_cmd) <= 1){
				HAL_GPIO_WritePin(MLD_GPIO_Port, MLD_Pin, atoi(data_cmd));
				printf("Left Motor Direction Setting!");
			}else printf("Wrong Direction Value...");
		}else if(strncmp((char*)data_arr, "MRD", 3) == 0){
			if(atoi(data_cmd) <= 1){
				HAL_GPIO_WritePin(MRD_GPIO_Port, MRD_Pin, atoi(data_cmd));
				printf("Right Motor Direction Setting!");
			}else printf("Wrong Direction Value...");
		}else if(strncmp((char*)data_arr, "MLS", 3) == 0){
			if(atoi(data_cmd) <= 100){
				motor.mt1 = atoi(data_cmd);
				printf("Left Motor Speed Setting!");
			}else printf("Wrong Speed Value...");
		}else if(strncmp((char*)data_arr, "MRS", 3) == 0){
			if(atoi(data_cmd) <= 100){
				motor.mt2 = atoi(data_cmd);
				printf("Right Motor Speed Setting!");
			}else printf("Wrong Speed Value...");
		}else if(strncmp((char*)data_arr, "ASK", 3) == 0){
			Msg_Help();
		}
		else{
			printf("Wrong Command.\r\n");
		}
		idx = 0;
		printf("\r\n");
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == huart2.Instance)
	{
		data_arr[idx++] = rx_data;
		HAL_UART_Transmit(&huart2, &rx_data, 1, 1); //10
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	}if (huart->Instance == huart3.Instance)
	{
		HAL_UART_Transmit(&huart3, &rx_data, 1, 1); //10
		HAL_UART_Receive_IT(&huart3, &rx_data, 1);
	}
	Command_Access();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance)
	{
		tim_flag++;
	}
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
