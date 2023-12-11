/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define V_REF_IN_Pin GPIO_PIN_0
#define V_REF_IN_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_2
#define GREEN_LED_GPIO_Port GPIOA
#define V_REF_OUT_Pin GPIO_PIN_3
#define V_REF_OUT_GPIO_Port GPIOA
#define V_REF_OUT_IN_Pin GPIO_PIN_4
#define V_REF_OUT_IN_GPIO_Port GPIOA
#define V_GAS_Pin GPIO_PIN_5
#define V_GAS_GPIO_Port GPIOA
#define RESET_RFM9X_Pin GPIO_PIN_6
#define RESET_RFM9X_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_7
#define RED_LED_GPIO_Port GPIOA
#define SPI3_NSS_Pin GPIO_PIN_8
#define SPI3_NSS_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLCK_Pin GPIO_PIN_14
#define SWCLCK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
