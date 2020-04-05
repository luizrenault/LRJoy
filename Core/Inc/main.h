/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LGEN_Pin GPIO_PIN_0
#define LGEN_GPIO_Port GPIOB
#define BATT_Pin GPIO_PIN_1
#define BATT_GPIO_Port GPIOB
#define FLIR_Pin GPIO_PIN_10
#define FLIR_GPIO_Port GPIOB
#define LTDR_Pin GPIO_PIN_11
#define LTDR_GPIO_Port GPIOB
#define LSTNFLR_Pin GPIO_PIN_12
#define LSTNFLR_GPIO_Port GPIOB
#define RGEN_Pin GPIO_PIN_3
#define RGEN_GPIO_Port GPIOB
#define ECSMODE_Pin GPIO_PIN_4
#define ECSMODE_GPIO_Port GPIOB
#define CABINPRESS_Pin GPIO_PIN_5
#define CABINPRESS_GPIO_Port GPIOB
#define PITOT_Pin GPIO_PIN_6
#define PITOT_GPIO_Port GPIOB
#define ANTIICEENG_Pin GPIO_PIN_7
#define ANTIICEENG_GPIO_Port GPIOB
#define LTTEST_Pin GPIO_PIN_8
#define LTTEST_GPIO_Port GPIOB
#define LTMODE_Pin GPIO_PIN_9
#define LTMODE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
