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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern UART_HandleTypeDef huart4;
extern char txt[1000];
extern int16_t helpTmp;
extern TIM_HandleTypeDef htim1;

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
#define IMU_TimDelay &htim17
#define ESC_TIM &htim3
#define HCSR04_TIM &htim4
#define DS2438_TimDelay &htim16
#define RTS_TIM &htim15
#define TERMINAL_UART &huart4
#define LED_TIM &htim1
#define LED_BLUE_CHANNEL TIM_CHANNEL_2
#define LED_RED_CHANNEL TIM_CHANNEL_1
#define EXTRA_7_Pin GPIO_PIN_13
#define EXTRA_7_GPIO_Port GPIOC
#define DS2438_DQ_Pin GPIO_PIN_0
#define DS2438_DQ_GPIO_Port GPIOC
#define EXTRA_6_Pin GPIO_PIN_1
#define EXTRA_6_GPIO_Port GPIOC
#define EXTRA_5_Pin GPIO_PIN_2
#define EXTRA_5_GPIO_Port GPIOC
#define EXTRA_4_Pin GPIO_PIN_3
#define EXTRA_4_GPIO_Port GPIOC
#define TERMINAL_TX_Pin GPIO_PIN_0
#define TERMINAL_TX_GPIO_Port GPIOA
#define TERMINAL_RX_Pin GPIO_PIN_1
#define TERMINAL_RX_GPIO_Port GPIOA
#define ESC_CH1_Pin GPIO_PIN_6
#define ESC_CH1_GPIO_Port GPIOA
#define ESC_CH2_Pin GPIO_PIN_7
#define ESC_CH2_GPIO_Port GPIOA
#define EXTRA_2_Pin GPIO_PIN_4
#define EXTRA_2_GPIO_Port GPIOC
#define EXTRA_1_Pin GPIO_PIN_5
#define EXTRA_1_GPIO_Port GPIOC
#define ESC_CH3_Pin GPIO_PIN_0
#define ESC_CH3_GPIO_Port GPIOB
#define ESC_CH4_Pin GPIO_PIN_1
#define ESC_CH4_GPIO_Port GPIOB
#define EXTRA_0_Pin GPIO_PIN_2
#define EXTRA_0_GPIO_Port GPIOB
#define DATA_TRANSMISSION_Pin GPIO_PIN_10
#define DATA_TRANSMISSION_GPIO_Port GPIOB
#define EXTRA_12_Pin GPIO_PIN_12
#define EXTRA_12_GPIO_Port GPIOB
#define EXTRA_11_Pin GPIO_PIN_13
#define EXTRA_11_GPIO_Port GPIOB
#define RECEIVER_DATA_Pin GPIO_PIN_14
#define RECEIVER_DATA_GPIO_Port GPIOB
#define EXTRA_10_Pin GPIO_PIN_15
#define EXTRA_10_GPIO_Port GPIOB
#define RECEIVER_PPM_Pin GPIO_PIN_6
#define RECEIVER_PPM_GPIO_Port GPIOC
#define EXTRA_9_Pin GPIO_PIN_7
#define EXTRA_9_GPIO_Port GPIOC
#define EXTRA_8_Pin GPIO_PIN_12
#define EXTRA_8_GPIO_Port GPIOA
#define IMU_F_SYNC_Pin GPIO_PIN_3
#define IMU_F_SYNC_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_4
#define IMU_INT_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_6
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_7
#define IMU_SDA_GPIO_Port GPIOB
#define HC_SR04_TRIG_Pin GPIO_PIN_8
#define HC_SR04_TRIG_GPIO_Port GPIOB
#define HC_SR04_ECHO_Pin GPIO_PIN_9
#define HC_SR04_ECHO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
