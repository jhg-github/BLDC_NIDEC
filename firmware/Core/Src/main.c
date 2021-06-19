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
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_TX_BUFFER_SIZE (80U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



//                  | Forward                | Backward
//# H1 H2 H3 | CODE | OUT1 U  OUT2 V  OUT3 W | OUT1 U  OUT2 V  OUT3 W
//------------------------------------------------------------
//1 H  L  L  |   4  | VS      High Z  PWM    | PWM     High Z  VS
//2 H  H  L  |   6  | High Z  VS      PWM    | High Z  PWM     VS
//3 L  H  L  |   2  | PWM     VS      High Z | VS      PWM     High Z
//4 L  H  H  |   3  | PWM     High Z  VS     | VS      High Z  PWM
//5 L  L  H  |   1  | High Z  PWM     VS     | High Z  VS      PWM
//6 H  L  H  |   5  | VS      PWM     High Z | PWM     VS      High Z

typedef void (*MAIN_StepFunc_t)(void);

static void MAIN_StepX_F(void){
  // all disabled
  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, RESET);
  HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, RESET);
  HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, RESET);
}

void MAIN_Step1_F(void){
  //# H1 H2 H3 | OUT1 U  OUT2 V  OUT3 W
  //1 H  L  L  | VS      High Z  PWM
  HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, RESET);
  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, SET);
  HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, SET);
}

static void MAIN_Step2_F(void){
  //# H1 H2 H3 | OUT1 U  OUT2 V  OUT3 W
  //2 H  H  L  | High Z  VS      PWM
  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, RESET);
  HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, SET);
  HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, SET);
}

static void MAIN_Step3_F(void){
  //# H1 H2 H3 | OUT1 U  OUT2 V  OUT3 W
  //3 L  H  L  | PWM     VS      High Z
  HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, RESET);
  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, SET);
  HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, SET);
}

static void MAIN_Step4_F(void){
  //# H1 H2 H3 | OUT1 U  OUT2 V  OUT3 W
  //4 L  H  H  | PWM     High Z  VS
  HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, RESET);
  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, SET);
  HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, SET);
}

static void MAIN_Step5_F(void){
  //# H1 H2 H3 | OUT1 U  OUT2 V  OUT3 W
  //5 L  L  H  | High Z  PWM     VS
  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, RESET);
  HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, SET);
  HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, SET);
}

static void MAIN_Step6_F(void){
  //# H1 H2 H3 | OUT1 U  OUT2 V  OUT3 W
  //6 H  L  H  | VS      PWM     High Z
  HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, RESET);
  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, SET);
  HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, SET);
}

static MAIN_StepFunc_t MAIN_step_func[8] = {
    MAIN_StepX_F,
    MAIN_Step5_F,
    MAIN_Step3_F,
    MAIN_Step4_F,
    MAIN_Step1_F,
    MAIN_Step6_F,
    MAIN_Step2_F,
    MAIN_StepX_F
};

static uint8_t MAIN_GetHALLCode(void){
  uint8_t code = (HAL_GPIO_ReadPin(HALL_H1_GPIO_Port, HALL_H1_Pin) << 2)
               | (HAL_GPIO_ReadPin(HALL_H2_GPIO_Port, HALL_H2_Pin) << 1)
               |  HAL_GPIO_ReadPin(HALL_H3_GPIO_Port, HALL_H3_Pin);
  return code;
}



static void MAIN_TEST_HALLOnUART(void){
  uint8_t tx_buffer[MAIN_TX_BUFFER_SIZE];
  int len = snprintf((char *)&tx_buffer[0], MAIN_TX_BUFFER_SIZE, "Hello world!\r\n");
  HAL_UART_Transmit(&huart2, &tx_buffer[0], len, 1000);
  while(1){
    int hall_h1 = HAL_GPIO_ReadPin(HALL_H1_GPIO_Port, HALL_H1_Pin);
    int hall_h2 = HAL_GPIO_ReadPin(HALL_H2_GPIO_Port, HALL_H2_Pin);
    int hall_h3 = HAL_GPIO_ReadPin(HALL_H3_GPIO_Port, HALL_H3_Pin);
    uint8_t code = MAIN_GetHALLCode();
    int len = snprintf((char *)&tx_buffer[0], MAIN_TX_BUFFER_SIZE, "\rH1:%d H2:%d H3:%d CODE:%d", hall_h1, hall_h2, hall_h3, code );
    HAL_UART_Transmit(&huart2, &tx_buffer[0], len, 1000);
    HAL_Delay(10);
  }
}

static void MAIN_TEST_HALLAndSteps(void){
  while(1){
    uint8_t code = MAIN_GetHALLCode();
    MAIN_step_func[code]();
  }
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //MAIN_TEST_HALLOnUART();
  MAIN_TEST_HALLAndSteps();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 159;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN1_Pin|EN2_Pin|EN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HALL_H3_Pin HALL_H2_Pin */
  GPIO_InitStruct.Pin = HALL_H3_Pin|HALL_H2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HALL_H1_Pin */
  GPIO_InitStruct.Pin = HALL_H1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HALL_H1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN1_Pin EN2_Pin EN3_Pin */
  GPIO_InitStruct.Pin = EN1_Pin|EN2_Pin|EN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
