/**
  ******************************************************************************
  * @file    System.h
  * @author  Waveshare Team
  * @version V1.0
  * @date    29-August-2014
  * @brief   Header file for system.c module

  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  ******************************************************************************
  */


#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "Public_StdTypes.h"
#include "LED.h"
#include "USART.h"
#include "I2C.h"
#include "IMU.h"



extern uint8_t UART_UpdataFlag;
extern uint8_t IMU_SampleFlag;
extern uint8_t Press_SampleFlag;
extern void System_Init(void);

#endif

/******************* (C) COPYRIGHT 2014 Waveshare *****END OF FILE*******************/

