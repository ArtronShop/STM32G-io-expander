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
#include "stm32g0xx_hal.h"

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
#define F1_Pin GPIO_PIN_0
#define F1_GPIO_Port GPIOA
#define F2_Pin GPIO_PIN_1
#define F2_GPIO_Port GPIOA
#define F3_Pin GPIO_PIN_2
#define F3_GPIO_Port GPIOA
#define F4_Pin GPIO_PIN_3
#define F4_GPIO_Port GPIOA
#define F5_Pin GPIO_PIN_4
#define F5_GPIO_Port GPIOA
#define F6_Pin GPIO_PIN_5
#define F6_GPIO_Port GPIOA
#define F7_Pin GPIO_PIN_6
#define F7_GPIO_Port GPIOA
#define F8_Pin GPIO_PIN_7
#define F8_GPIO_Port GPIOA
#define PUMP_Pin GPIO_PIN_0
#define PUMP_GPIO_Port GPIOB
#define HEATER_Pin GPIO_PIN_1
#define HEATER_GPIO_Port GPIOB
#define LIGHT_Pin GPIO_PIN_2
#define LIGHT_GPIO_Port GPIOB
#define NET_Pin GPIO_PIN_5
#define NET_GPIO_Port GPIOB
#define RS485_SEL1_Pin GPIO_PIN_6
#define RS485_SEL1_GPIO_Port GPIOB
#define RS485_SEL0_Pin GPIO_PIN_7
#define RS485_SEL0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
