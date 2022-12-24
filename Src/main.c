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
uint8_t spi_send[SPI_BYTE_AMOUNT]={}; //acá escribo los valores a mandar via spi, lo incializo con todos los valores en 0

//uint16_t leds[24]=
//{
//512,512,1024,2048,2048,2048,
//2048,2048,2048,2048,2048,2048,
//2048,2048,2048,2048,2048,2048,
//2048,2048,3000,2048,2048,3072
//};

//RED por algun motivo flashero desde [15] hasta  [23] ponia 65355
uint16_t leds[24]=
{}; // todo en 0, le pongo valores con FillArray
uint8_t imain=0;
uint8_t flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
//void TLC_Write(uint8_t data);// prototipo para mandar todo un vector de bytes
void TLC_Write(uint8_t data[]);// prototipo para mandar todo un vector de bytes

void TLC_Update(void);
void FillArray(uint8_t color);// prototipo para llenar el array en la prueba de los canales
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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
//  HAL_TIM_Base_Start_IT(&htim2);

  FillArray(GREEN);
  //InitSPI();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  HAL_GPIO_WritePin(TLC5947_BLANK_GPIO_Port, TLC5947_BLANK_Pin, GPIO_PIN_SET); // este pin en alto apaga los leds
//  FillArray(GREEN);
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

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


//	  HAL_Delay(500);
//	 	  	TLC_Update();//renew PWM
//////	 	  	FillArray(imain);
//	 	  	if(imain > 2 )
//	 	  		imain = 0;
//	 	  	imain ++;
//	 	  	HAL_Delay(1);

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  HAL_GPIO_WritePin(BLUEPILL_LED_GPIO_Port, BLUEPILL_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TLC5947_BLANK_Pin|TLC5947_XLAT_Pin|LED_TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLUEPILL_LED_Pin */
  GPIO_InitStruct.Pin = BLUEPILL_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUEPILL_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TLC5947_BLANK_Pin TLC5947_XLAT_Pin LED_TEST_Pin */
  GPIO_InitStruct.Pin = TLC5947_BLANK_Pin|TLC5947_XLAT_Pin|LED_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
if(htim->Instance == TIM3)
  HAL_GPIO_TogglePin(BLUEPILL_LED_GPIO_Port, BLUEPILL_LED_Pin);//Prendo y apago el pin del board de bluepill cada 'x' segundos
//if(htim->Instance == TIM2)
//{
	  	TLC_Update();//renew PWM
	  	imain ++;
	  	if(imain > 2 )
	  		imain = 0;
	 	FillArray(imain);

//}

  /*
   * Entro en int del timer 3 cada 910.2us ya que
   * Eligo trabajar con el TLC a una frecuencia inferior a 5.6Mhz y tengo
   * clock stm32 = 72Mhz
   * PrescalerSpi = 16 => FSpi = 72M/16 = 4.5Mhz
   * Periodo para resetear GS* = 1/FSpi * 4096 = 910.2us
   * GS* = Greyscale
   * */

//  HAL_GPIO_WritePin(TLC5947_BLANK_GPIO_Port, TLC5947_BLANK_Pin, GPIO_PIN_SET); //Blank high apaga leds
//  HAL_GPIO_WritePin(TLC5947_XLAT_GPIO_Port, TLC5947_XLAT_Pin, GPIO_PIN_SET); // Xlatch high para leer los datos que envie
//  HAL_GPIO_WritePin(TLC5947_XLAT_GPIO_Port, TLC5947_XLAT_Pin, GPIO_PIN_RESET);// Xlatch low porque necesito pasar de high a low para indicar que haga la lectura
//  HAL_GPIO_WritePin(TLC5947_BLANK_GPIO_Port, TLC5947_BLANK_Pin, GPIO_PIN_RESET);//Blank low reinicio cuenta de GS
//  	  //envio data al tlc
//	for(i = 0; i<36; i++)
//	{
//		SendDataSPI(testbyte[i]);
//	}

}

/*
 * retorno de una INT externa de un pin
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 static int count = 0;
 uint8_t sep = 0x69;

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


//	  HAL_UART_Transmit(&huart3, (uint8_t *)"Hello, world!\r\n", 15U, 100U);
	  HAL_UART_Transmit(&huart3, &uartByte, 1, 100U);
	  uartByte =(uint8_t)((g_tDelay >>8) & 0x00ff); // i read the second byte MSB
	  HAL_UART_Transmit(&huart3, &uartByte, 1, 100U);
	  HAL_UART_Transmit(&huart3, &sep, 1, 100U); // lo uso para separar cada medición



  }

  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/*
 * SPI
 */
void TLC_Update(void)
{
	uint8_t si = 0;//Lo uso para el vector a enviar via SPI

    HAL_GPIO_WritePin(TLC5947_BLANK_GPIO_Port, TLC5947_BLANK_Pin, GPIO_PIN_SET);
//		HAL_Delay(1);
//    for (int8_t ji = TOTAL_CHANNELS - 1 ; ji >= 0; ji -= 2)
//    {
//        uint8_t send1 = 0;
//        uint8_t send = leds[ji] >> 4; // mando MSB
//
//        spi_send[si]=send;//
//        si++;
////        TLC_Write(send);
//
//        send = (leds[ji] & 0x000F);
//        send <<= 4;
//        send1 = (leds[ji-1]) >> 8;
//        send |= send1; //me quedo con 4 bits menos significativos del canal i y 4 bits más significativos del canal i-1
//
//        spi_send[si]=send;//
//        si++;
//        //        TLC_Write(send);
//
//        send = leds[ji-1];//borro 4 bits más significativos del canal i-1 y mando LSB del canal i-1
//
//        spi_send[si]=send;//
//        si++;
////        TLC_Write(send);
//    }
    for (int8_t i = 0; i < TOTAL_CHANNELS; i += 2) // lleno
    {
        uint8_t send1 = 0;
        uint8_t send = leds[i] >> 4; // mando MSB

        spi_send[si]=send;//
        si++;

        send = (leds[i] & 0x000F);
        send <<= 4;
        send1 = (leds[i+1]) >> 8;
        send |= send1; //me quedo con 4 bits menos significativos del canal i y 4 bits más significativos del canal i-1

        spi_send[si]=send;//
        si++;


        send = leds[i+1];//borro 4 bits más significativos del canal i-1 y mando LSB del canal i-1

        spi_send[si]=send;//
        si++;

    }

//    for(int m =0;m<SPI_BYTE_AMOUNT;m++)
    TLC_Write(spi_send);

//    imain++;
//	FillArray(imain);
    HAL_GPIO_WritePin(TLC5947_XLAT_GPIO_Port, TLC5947_XLAT_Pin, GPIO_PIN_SET);
//		HAL_Delay(1);
    HAL_GPIO_WritePin(TLC5947_XLAT_GPIO_Port, TLC5947_XLAT_Pin, GPIO_PIN_RESET);
//		HAL_Delay(1);
    HAL_GPIO_WritePin(TLC5947_BLANK_GPIO_Port, TLC5947_BLANK_Pin, GPIO_PIN_RESET);
    return;

}


void TLC_Write(uint8_t data[])
//void TLC_Write(uint8_t *data)
{
//	uint8_t prueba = 0x00;
//	HAL_SPI_Transmit(&hspi1, &data[array_index], sizeof(data), 0); // envio via el sp1 de solo 1 byte
	HAL_SPI_Transmit(&hspi1,data, SPI_BYTE_AMOUNT,1000); // envio via el sp1 de 1 todos los bytes que tenga que mandar
////	HAL_SPI_Transmit(&hspi1,&prueba, sizeof(prueba),1000);
////HAL_SPI_Transmit(&hspi1,&data, sizeof(data),0);
//    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY); // espero a que termine la transferen
    return;

}
/*
 * Uso esta funcion para llenar los vectores de prueba
 * por algún motivo al poner valores en dudo aparecian valores espúreos
 * en las últimas posiciones de 65535
 *
 */
void FillArray(uint8_t color)
{
	uint8_t position = 0, increment = 3,array_index = 0;
	uint16_t intensity = 32;
	switch(color)
	{
	case 0:
		position = 0;
		intensity = 4095;
		break;
	case 1:
		position = 1;
		intensity = 2048;
		break;
	case 2:
		position = 2;
		intensity = 1024;
		break;
	default:
		//RGB
//		increment = 1; // por el momento no lo uso
		break;
	}
	for (array_index= 0; array_index<TOTAL_CHANNELS;array_index++)
		leds[array_index] = 0;//borro todos los valores

	for (array_index=position; array_index<TOTAL_CHANNELS;array_index+=increment)
//		leds[array_index] = 4095*1/(position+1);//intensidad tenue
		leds[array_index] =intensity;
return;
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
