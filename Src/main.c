/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "mirage_spi.h" // macros para la conexion spi

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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t g_prevTime = 0 ,g_curTime = 0, g_tDelay = 0; //stores timertick
uint8_t uartByte; //used to send a byte through UART3

/*
 * SPI
 * https://www.bilibili.com/read/cv11912081/ 出处：bilibili
 */
uint16_t leds[24]=
{
512,512,1024,2048,2048,2048,
2048,2048,2048,2048,2048,2048,
2048,2048,2048,2048,2048,2048,
2048,2048,3000,2048,2048,3072
};
int i=0;
int flag=0;
/*
 * SPI
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/*
 * SPI
 */
void SendDataSPI(uint8_t reg, uint8_t data); // función para hablar con el TLC5947
void InitSPI(void);// función para colocar los datos a enviar
void TLC_Update(void);
void TLC_Write(uint8_t data);
/*
 * SPI
 */

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
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  InitSPI();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

/*
 * codigo de prueba de sensor hall via pulling

	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET) // hay lectura en el sensor hall
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // prendo el led
	  else
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // apago el led
	  HAL_Delay(50);
*/
 /*
* SPI
*/
	  if(flag==0)//light fades
	  				i+=5;
	  		else//light fades out

	  				i-=5;

	  		if(flag==0&&i==4095)//the brightest light
	  		{
	  			HAL_GPIO_WritePin(GPIOA, LED_TEST_Pin, GPIO_PIN_SET);
	  			flag=1;
	  		}
	  		if(flag==1&&i==0)//Dimmest light
	  		{
	  			HAL_GPIO_WritePin(GPIOA, LED_TEST_Pin, GPIO_PIN_RESET);
	  			flag=0;
	  		}
	  		leds[0]=i;//update channel 0 PWM
	  		TLC_Update();//renew PWM
	  		HAL_Delay(1);
/*
* SPI
*/
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim3.Init.Period = 2000;
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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TLC5947_BLANK_Pin|TLC5947_XLAT_Pin|LED_TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TLC5947_BLANK_Pin TLC5947_XLAT_Pin LED_TEST_Pin */
  GPIO_InitStruct.Pin = TLC5947_BLANK_Pin|TLC5947_XLAT_Pin|LED_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HALL_Pin */
  GPIO_InitStruct.Pin = HALL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HALL_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */

  HAL_GPIO_TogglePin(BOARD_LED_PORT, BOARD_LED_PIN);//Prendo y apago el pin cada 'x' segundos


}

/*
 * retorno de una INT externa de un pin
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 static int count = 0;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */

  //HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);//buscar valor en stm32f103xb.h
  g_prevTime = g_curTime;
  if(GPIO_Pin == GPIO_PIN_9) // pin del sensor hall
  {
	  HAL_GPIO_TogglePin(GPIOA, LED_TEST_Pin); // prendo/apago el led
	  //HAL_GPIO_WritePin(GPIOA, LED_TEST_Pin, GPIO_PIN_SET);

	  g_curTime = HAL_GetTick(); //Provides a tick value in millisecond
	  count ++;
	  g_tDelay = (g_curTime - g_prevTime);
	  uartByte = (uint8_t)(g_tDelay & 0x00ff); // i keep only the first byte LSB

	  HAL_UART_Transmit(&huart3, &uartByte, 1, 100U);
	  uartByte =(uint8_t)((g_tDelay >>8) & 0x00ff); // i read the second byte MSB
	  HAL_UART_Transmit(&huart3, &uartByte, 1, 100U);



  }

  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/*
 * SPI
 */
void TLC_Update(void)
{
    HAL_GPIO_WritePin(TLC5947_BLANK_PORT, TLC5947_BLANK_Pin, GPIO_PIN_SET);
//		HAL_Delay(1);
    for (int8_t i = 23; i >= 0; i -= 2)
    {
        uint8_t send1 = 0;
        uint8_t send = leds[i] >> 4;
        TLC_Write(send);
        send = (leds[i] & 0x000F);
        send <<= 4;
        send1 = (leds[i-1]) >> 8;
        send |= send1;
        TLC_Write(send);
        send = leds[i-1];
        TLC_Write(send);
    }

    HAL_GPIO_WritePin(TLC5947_XLAT_PORT, TLC5947_XLAT_Pin, GPIO_PIN_SET);
//		HAL_Delay(1);
    HAL_GPIO_WritePin(TLC5947_XLAT_PORT, TLC5947_XLAT_Pin, GPIO_PIN_RESET);
//		HAL_Delay(1);
    HAL_GPIO_WritePin(TLC5947_BLANK_PORT, TLC5947_BLANK_Pin, GPIO_PIN_RESET);

    return ;
}


void TLC_Write(uint8_t data)
{
    HAL_SPI_Transmit(&hspi1, &data, sizeof(data), 0);
    while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_RESET);

    return ;
}
/*
 * envio las configuraciones del TLC5947
 */
void SendDataSPI(uint8_t reg, uint8_t data)
{

/*
	HAL_GPIO_WritePin(SPI_BLANK_PORT, SPI_BLANK_PIN, GPIO_PIN_SET);//deshabilito las salidas del TLC5947
	HAL_GPIO_WritePin(SPI_XLAT_PORT, SPI_XLAT_PIN, GPIO_PIN_RESET);//deshabilito la escritura del GS

	/*
	  * @brief  Transmit an amount of data in blocking mode.
	  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
	  *               the configuration information for SPI module.
	  * @param  pData: pointer to data buffer
	  * @param  Size: amount of data to be sent
	  * @param  Timeout: Timeout duration
	  * @retval HAL status
	  */
/*
	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);//envio registro a configurar, tamaño de la info 1byte, timeout
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);//espero hasta que concluya la comunicación

	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);//envio data a configurar, tamaño de la info 1byte, timeout
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);//espero hasta que concluya la comunicación

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);//de-seleciono dispositivo a usar colocando un '0'
*/
}
/*
 * llama a SendDataSPI y le pasa los valores a usar
 */
void InitSPI(void)
{

}

/*
 * SPI
 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
