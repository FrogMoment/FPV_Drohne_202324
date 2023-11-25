/**
  ******************************************************************************
  * @file    USART.h
  * @author  Waveshare Team
  * @version V1.0
  * @date    29-August-2014
  * @brief   Header file for USART.c module

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


#ifndef _USART_H_
#define _USART_H_

#include "Public_StdTypes.h"

extern void USART1_Init(uint32_t Baudrate);       		
extern void USART_Send_Char(uint8_t ascii_code); 	

#endif

/******************* (C) COPYRIGHT 2014 Waveshare *****END OF FILE*******************/

