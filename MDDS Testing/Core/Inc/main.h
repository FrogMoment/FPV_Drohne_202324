/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RECEIVER_DATA_Pin GPIO_PIN_2
#define RECEIVER_DATA_GPIO_Port GPIOA
#define RECEIVER_PPM_Pin GPIO_PIN_3
#define RECEIVER_PPM_GPIO_Port GPIOA
#define ESC_CH1_Pin GPIO_PIN_6
#define ESC_CH1_GPIO_Port GPIOA
#define ESC_CH2_Pin GPIO_PIN_7
#define ESC_CH2_GPIO_Port GPIOA
#define ESC_CH3_Pin GPIO_PIN_0
#define ESC_CH3_GPIO_Port GPIOB
#define ESC_CH4_Pin GPIO_PIN_1
#define ESC_CH4_GPIO_Port GPIOB
#define TERMINAL_TX_Pin GPIO_PIN_9
#define TERMINAL_TX_GPIO_Port GPIOA
#define TERMINAL_RX_Pin GPIO_PIN_10
#define TERMINAL_RX_GPIO_Port GPIOA
#define DS2438_DQ_Pin GPIO_PIN_12
#define DS2438_DQ_GPIO_Port GPIOC
#define MPU9250_SCL_Pin GPIO_PIN_6
#define MPU9250_SCL_GPIO_Port GPIOB
#define MPU9250_SDA_Pin GPIO_PIN_7
#define MPU9250_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
