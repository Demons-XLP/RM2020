/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define Omron3_Pin GPIO_PIN_13
#define Omron3_GPIO_Port GPIOC
#define Omron2_Pin GPIO_PIN_14
#define Omron2_GPIO_Port GPIOC
#define Omron1_Pin GPIO_PIN_15
#define Omron1_GPIO_Port GPIOC
#define BLIE_Pin GPIO_PIN_0
#define BLIE_GPIO_Port GPIOA
#define RED_Pin GPIO_PIN_1
#define RED_GPIO_Port GPIOA
#define GREEN_Pin GPIO_PIN_2
#define GREEN_GPIO_Port GPIOA
#define BELL_Pin GPIO_PIN_3
#define BELL_GPIO_Port GPIOA
#define OLED_DC_Pin GPIO_PIN_4
#define OLED_DC_GPIO_Port GPIOC
#define OLED_RES_Pin GPIO_PIN_5
#define OLED_RES_GPIO_Port GPIOC
#define Air_Cylinder5_Pin GPIO_PIN_3
#define Air_Cylinder5_GPIO_Port GPIOB
#define Air_Cylinder4_Pin GPIO_PIN_4
#define Air_Cylinder4_GPIO_Port GPIOB
#define Air_Cylinder3_Pin GPIO_PIN_5
#define Air_Cylinder3_GPIO_Port GPIOB
#define Air_Cylinder2_Pin GPIO_PIN_6
#define Air_Cylinder2_GPIO_Port GPIOB
#define Air_Cylinder1_Pin GPIO_PIN_7
#define Air_Cylinder1_GPIO_Port GPIOB
#define Omron4_Pin GPIO_PIN_9
#define Omron4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
