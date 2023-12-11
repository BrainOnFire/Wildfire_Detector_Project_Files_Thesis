/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "rfm95.h"
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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t buf[50];
uint8_t uart_buf[50];
uint8_t temp, hum, monoxido_temp1, monoxido_temp2;
uint8_t verificador1, verificador2;
uint16_t CO;
int RSSI = 0;
float SNR = 0;
uint8_t contador = 0;

rfm95_handle_t rfm95_handle = {
    .spi_handle = &hspi1,
    .nss_port = SPI1_NSS_GPIO_Port,
    .nss_pin = SPI1_NSS_Pin,
    .nrst_port = RESET_GPIO_Port,
    .nrst_pin = RESET_Pin,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void umbral_temperatura(uint8_t temperatura);
void umbral_humedad(uint8_t humedad);
void umbral_monoxido(uint16_t monoxido);
void contador_update(void);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  do{
	  //Inicializar modulo LoRa
	  if(rfm95_init(&rfm95_handle)){
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		  sprintf((char*)uart_buf, "Modulo LoRa Inicializado\n");
		  HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		  verificador1 = 1;
		  verificador2 = 1;
	  }
	  else{
		  rfm95_reset(&rfm95_handle);
	  }
  }
  while(!verificador1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (verificador2 == 1)
  {
	  // Recibir los datos por LoRa
	  rfm95_receive_package(&rfm95_handle, &temp, &hum, &monoxido_temp1, &monoxido_temp2, RSSI, SNR);
	  // Validation message
	  CO = (monoxido_temp1 << 8) | monoxido_temp2;
	  sprintf((char*)uart_buf, "Conexion exitosa\n");
	  HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);

	  //Comparacion de los datos con umbrales
	  umbral_temperatura(temp);
	  umbral_humedad(hum);
	  umbral_monoxido(CO);
	  contador_update();
	  //sprintf((char*)uart_buf, "Temperatura: %u C, Humedad: %u HR, %u PPM \n", temp, hum, CO);
	  //HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
	  //sprintf((char*)uart_buf, "SNR: %.2f C, RSSI: %i  \n", SNR, RSSI);
	  //HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
	  temp = 0;
	  hum = 0;
	  CO = 0;
	  monoxido_temp1 = 0;
	  monoxido_temp2 = 0;
	  contador = 0;
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESET_Pin|SPI1_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_Pin SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = RESET_Pin|SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : G0_LORA_INT_Pin */
  GPIO_InitStruct.Pin = G0_LORA_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(G0_LORA_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void umbral_temperatura(uint8_t temperatura){
	//Comparacion temperatura
	if(37 < temperatura && temperatura < 80){
		sprintf((char*)uart_buf, "Atencion!! Temperatura muy Alta\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		contador++;
	}
	else if(temperatura < 10 || temperatura > 80){
		sprintf((char*)uart_buf, "Atencion!! Valor erroneo de temperatura\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
	}

	else if(10 < temperatura && temperatura < 37){
		sprintf((char*)uart_buf, "Valores normales de temperatura\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
	}
	sprintf((char*)uart_buf, "Temperatura = %u C\n", temperatura);
	HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
}

void umbral_humedad(uint8_t humedad){
	if(5 < humedad && humedad < 20){
		sprintf((char*)uart_buf, "Atencion!! Humedad muy baja\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		contador++;
	}
	else if(humedad < 5 || humedad > 95){
		sprintf((char*)uart_buf, "Atencion!! Valor erroneo de humedad\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
	}
	else if(20 < humedad && humedad < 95){
		sprintf((char*)uart_buf, "Valores normales de humedad\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
	}
	sprintf((char*)uart_buf, "Humedad = %u HR\n", humedad);
	HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
}

void umbral_monoxido(uint16_t monoxido){
	if(800 < monoxido && monoxido < 7000){
		sprintf((char*)uart_buf, "Atencion!! Valores de CO muy altos\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		contador += 2;
	}
	else if(monoxido > 7000){
		sprintf((char*)uart_buf, "Atencion!! Valor erroneos de CO\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
	}
	else if(0 <= monoxido && monoxido < 800){
		sprintf((char*)uart_buf, "Valores normales de CO\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
	}
	sprintf((char*)uart_buf, "CO = %u PPM\n", monoxido);
	HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
}

void contador_update(void){
	switch(contador){
	case 1:
		sprintf((char*)uart_buf, "Existe un bajo riesgo de incendio\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		break;
	case 2:
		sprintf((char*)uart_buf, "Alerta! Existe un riesgo medio de incendio.\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		break;
	case 4:
		sprintf((char*)uart_buf, "Alerta!! Existe un alto riesgo de incendio.\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
		break;
	default:
		sprintf((char*)uart_buf, "No existe riesgo de incendio.\n");
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*)uart_buf), HAL_MAX_DELAY);
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
