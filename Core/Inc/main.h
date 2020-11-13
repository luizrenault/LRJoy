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
#define L1_Pin GPIO_PIN_0
#define L1_GPIO_Port GPIOA
#define L2_Pin GPIO_PIN_1
#define L2_GPIO_Port GPIOA
#define L3_Pin GPIO_PIN_2
#define L3_GPIO_Port GPIOA
#define L4_Pin GPIO_PIN_3
#define L4_GPIO_Port GPIOA
#define L5_Pin GPIO_PIN_4
#define L5_GPIO_Port GPIOA
#define L6_Pin GPIO_PIN_5
#define L6_GPIO_Port GPIOA
#define L7_Pin GPIO_PIN_6
#define L7_GPIO_Port GPIOA
#define L8_Pin GPIO_PIN_7
#define L8_GPIO_Port GPIOA
#define ENC3A_Pin GPIO_PIN_8
#define ENC3A_GPIO_Port GPIOA
#define ENC3B_Pin GPIO_PIN_9
#define ENC3B_GPIO_Port GPIOA
#define C5_Pin GPIO_PIN_10
#define C5_GPIO_Port GPIOA
#define ENC1A_Pin GPIO_PIN_15
#define ENC1A_GPIO_Port GPIOA
#define ENC1B_Pin GPIO_PIN_3
#define ENC1B_GPIO_Port GPIOB
#define ENC2A_Pin GPIO_PIN_4
#define ENC2A_GPIO_Port GPIOB
#define ENC2B_Pin GPIO_PIN_5
#define ENC2B_GPIO_Port GPIOB
#define C1_Pin GPIO_PIN_6
#define C1_GPIO_Port GPIOB
#define C2_Pin GPIO_PIN_7
#define C2_GPIO_Port GPIOB
#define C3_Pin GPIO_PIN_8
#define C3_GPIO_Port GPIOB
#define C4_Pin GPIO_PIN_9
#define C4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
