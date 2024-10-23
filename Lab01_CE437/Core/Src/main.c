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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_MODE 				3								/* number of mode */
#define NUM_BUTTON				2								/* number of user button */
#define DEBOUNCE_DELAY			50U								/* software debounce delay */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
const int pressTimeThreshold = 500;
const int Button_Pin[NUM_BUTTON] = {BTN1_Pin, BTN2_Pin};
int LED_Blink_Period[] = {500, 500, 500};				/* blink period in different modes */
int LED_Blink_Current_Mode;							/* current mode */
int LED_Blink_Current_Period = 500;						/* current blink period */
volatile uint32_t difTime = 0;

uint32_t timeElapsed = 0;
uint8_t isButtonPressed;
uint32_t pressTime = 0;
uint8_t isButtonPressed	= 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
typedef void (*LED_Blink)(void);
typedef void (*Short_Press)(void);
typedef void (*Long_Press)(void);

extern LED_Blink LED_Blink_Mode[];
extern Short_Press Short_Press_Button[];
extern Long_Press Long_Press_Button[];

void LED_Blink_Mode_Config(int p_mode);
void LED_Blink_Mode_1(void);
void LED_Blink_Mode_2(void);
void LED_Blink_Mode_3(void);
void Short_Press_Button_1(void);
void Short_Press_Button_2(void);
void Long_Press_Button_1(void);
void Long_Press_Button_2(void);
void Buttons_Check(void);
int checkButtonInMode();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief: LED blink effect pointer function
 */
LED_Blink LED_Blink_Mode[NUM_MODE] =
{
	LED_Blink_Mode_1,
	LED_Blink_Mode_2,
	LED_Blink_Mode_3
};
/**
 * @brief: Button short press pointer function
 */
Short_Press Short_Press_Button[NUM_BUTTON] =
{
	Short_Press_Button_1,
	Short_Press_Button_2
};
/**
 * @brief: Button long press pointer function
 */
Long_Press Long_Press_Button[NUM_BUTTON] =
{
	Long_Press_Button_1,
	Long_Press_Button_2
};

/**
 * @brief Config LED blink mode and period
 * @param Current mode
 * @retval None
 */
 void LED_Blink_Mode_Config(int p_mode)
{
	LED_Blink_Current_Mode = p_mode;
	LED_Blink_Current_Period = LED_Blink_Period[LED_Blink_Current_Mode];
}

void LED_All_TurnOff(void)
{
	HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
}

void LED_All_TurnOn(void)
{
	HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
}

/**
 * @brief LED blink effect mode 1
 * @operation LED blink effect mode 1
 * @param None
 * @retval None
 */

void LED_Blink_Mode_1(void)
{
	if (timeElapsed >= LED_Blink_Current_Period)
	{
		LED_All_TurnOff();
		while (timeElapsed < LED_Blink_Current_Period + 300) {
			if (checkButtonInMode()) return;
		}
		LED_All_TurnOn();
		timeElapsed = 0;
	}
}

/**
 * @brief LED blink effect mode 2
 * @operation Blink green LED with duration mode 2
 * @param None
 * @retval None
 */
 void LED_Blink_Mode_2(void)
{
	if (timeElapsed >= LED_Blink_Current_Period)
	{
		HAL_GPIO_TogglePin(GPIOB, LEDB_Pin);
		while (timeElapsed < LED_Blink_Current_Period + 300) {
			if (checkButtonInMode()) return;
		}
		HAL_GPIO_TogglePin(GPIOB, LEDG_Pin);
		while (timeElapsed < LED_Blink_Current_Period + 600) {
			if (checkButtonInMode()) return;
		}
		HAL_GPIO_TogglePin(GPIOB, LEDR_Pin);
		while (timeElapsed < LED_Blink_Current_Period + 900) {
			if (checkButtonInMode()) return;
		}
		timeElapsed = 0;
	}
}

/**
 * @brief LED blink effect mode 3
 * @operation Blink blue LED with duration mode 3
 * @param None
 * @retval None
 */
 void LED_Blink_Mode_3(void)
{
	if (timeElapsed >= LED_Blink_Current_Period)
	{
		HAL_GPIO_TogglePin(GPIOB, LEDR_Pin);
//		while (timeElapsed < LED_Blink_Current_Period + 300) {
//			if (checkButtonInMode()) return;
//		}
		HAL_GPIO_TogglePin(GPIOB, LEDB_Pin);
		while (timeElapsed < LED_Blink_Current_Period + 300) {
			if (checkButtonInMode()) return;
		}
		HAL_GPIO_TogglePin(GPIOB, LEDG_Pin);
		while (timeElapsed < LED_Blink_Current_Period + 300) {
			if (checkButtonInMode()) return;
		}
		timeElapsed = 0;
	}
}

/**
 * @brief short press button 1 process
 * @operation decrease period 100ms
 * @param None
 * @retval None
 */
inline void Short_Press_Button_1(void)
{
	LED_All_TurnOff();
	difTime = -100;
}

/**
 * @brief short press button 2 process
 * @operation change LED blink effect mode
 * @param None
 * @retval None
 */
inline void Short_Press_Button_2(void)
{
	LED_All_TurnOff();
	LED_Blink_Current_Mode = (LED_Blink_Current_Mode + 1) % NUM_MODE;
}

/**
 * @brief long press button 1 process
 * @operation decrease period 100ms every 200ms button pressed
 * @param None
 * @retval None
 */
inline void Long_Press_Button_1(void)
{
	LED_All_TurnOff();
	difTime = -(100 * (int)((pressTime - 500) / 200));
}

/**
 * @brief long press button 2 process
 * @operation increase period 100ms every 200ms button pressed
 * @param None
 * @retval None
 */
inline void Long_Press_Button_2(void)
{
	LED_All_TurnOff();
	difTime = (100 * (int)((pressTime - 500) / 200));
}

/**
 * @brief buttons check
 * @operation check if any buttons is pressed
 * @param None
 * @retval None
 */
void Buttons_Check(void)
{
	for (int _button = 0; _button < NUM_BUTTON; ++ _button)
	{
		if (!HAL_GPIO_ReadPin(GPIOB, Button_Pin[_button]))
		{
			/* software debounce */
			HAL_Delay(DEBOUNCE_DELAY);
			if (!HAL_GPIO_ReadPin(GPIOB, Button_Pin[_button]))
			{
				isButtonPressed = 1;
				pressTime = 0;
				difTime = 0;

				/* wait until the button released */
				while (!HAL_GPIO_ReadPin(GPIOB, Button_Pin[_button]));
				if (pressTime < pressTimeThreshold)
				{
					Short_Press_Button[_button]();
				}
				else
				{
					Long_Press_Button[_button]();
				}
				for (int _mode = 0; _mode < NUM_MODE; ++ _mode)
				{
				  LED_Blink_Period[_mode] += difTime;
				  if (LED_Blink_Period[_mode] <= 0)
				  {
					  LED_Blink_Period[_mode] = 2000;
				  }
				}
				LED_Blink_Mode_Config(LED_Blink_Current_Mode);

				isButtonPressed = 0;
			}
		}
	}
}
int checkButtonInMode()
{
	for (int _button = 0; _button < NUM_BUTTON; ++ _button)
	{
		if (!HAL_GPIO_ReadPin(GPIOB, Button_Pin[_button]))
		{
			/* software debounce */
			HAL_Delay(DEBOUNCE_DELAY);
			if (!HAL_GPIO_ReadPin(GPIOB, Button_Pin[_button])) return 1;
		}
	}
	return 0;
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LED_Blink_Current_Mode = 0;
    while (1)
    {
	    /* blink LED effect current mode */
	    LED_Blink_Mode[LED_Blink_Current_Mode]();
      Buttons_Check();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  htim3.Init.Prescaler = 35999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEDB_Pin|LEDG_Pin|LEDR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN2_Pin BTN1_Pin */
  GPIO_InitStruct.Pin = BTN2_Pin|BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDB_Pin LEDG_Pin LEDR_Pin */
  GPIO_InitStruct.Pin = LEDB_Pin|LEDG_Pin|LEDR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Timer ISR
 * @param htim Timer_HandleTypeDef pointer
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim3.Instance)
	{
		timeElapsed += 100;
		if (isButtonPressed) pressTime += 100;
	}
	else
	{
		__NOP();
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
