/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOA
#define DATA_Pin GPIO_PIN_1
#define DATA_GPIO_Port GPIOA
#define SCLK_Pin GPIO_PIN_2
#define SCLK_GPIO_Port GPIOA
#define XLAT_Pin GPIO_PIN_3
#define XLAT_GPIO_Port GPIOA
#define BLANK_Pin GPIO_PIN_4
#define BLANK_GPIO_Port GPIOA
#define SEC_UNID_Pin GPIO_PIN_5
#define SEC_UNID_GPIO_Port GPIOA
#define SEC_DEZ_Pin GPIO_PIN_6
#define SEC_DEZ_GPIO_Port GPIOA
#define MIN_UNID_Pin GPIO_PIN_7
#define MIN_UNID_GPIO_Port GPIOA
#define MIN_DEZ_Pin GPIO_PIN_8
#define MIN_DEZ_GPIO_Port GPIOA
#define HOUR_UNID_Pin GPIO_PIN_9
#define HOUR_UNID_GPIO_Port GPIOA
#define HOUR_DEZ_Pin GPIO_PIN_10
#define HOUR_DEZ_GPIO_Port GPIOA
#define SEL_MODE_Pin GPIO_PIN_11
#define SEL_MODE_GPIO_Port GPIOA
#define CONFIRM_Pin GPIO_PIN_12
#define CONFIRM_GPIO_Port GPIOA
#define ON_OFF_Pin GPIO_PIN_15
#define ON_OFF_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
