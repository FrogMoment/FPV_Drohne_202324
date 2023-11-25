/**
  ******************************************************************************
  * @file    System.c 
  * @author  Waveshare Team
  * @version V1.0
  * @date    29-August-2014
  * @brief   system initialization
  *          This file should be added to the main application to use the provided
  *          functions that initializes system
  *
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


#include "System.h"

uint8_t UART_UpdataFlag = 0;
uint8_t IMU_SampleFlag = 0;
uint8_t Press_SampleFlag = 0;
/**
  * @brief  Configures the NVIC Preemption Priority Bits
  * @param  None
  * @retval None
  */

void Sys_NVICConfiguration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
}


/**
  * @brief  Initializes the system time base
  * @param  None
  * @retval None
  */

void Sys_TimeBaseInit(void)
{
	/* Setup SysTick Timer for 1 msec interrupts.
	 ------------------------------------------
	1. The SysTick_Config() function is a CMSIS function which configure:
	   - The SysTick Reload register with value passed as function parameter.
	   - Configure the SysTick IRQ priority to the lowest value (0x0F).
	   - Reset the SysTick Counter register.
	   - Configure the SysTick Counter clock source to be Core Clock Source (HCLK).
	   - Enable the SysTick Interrupt.
	   - Start the SysTick Counter.

	2. You can change the SysTick Clock source to be HCLK_Div8 by calling the
	   SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8) just after the
	   SysTick_Config() function call. The SysTick_CLKSourceConfig() is defined
	   inside the misc.c file.

	3. You can change the SysTick IRQ priority by calling the
	   NVIC_SetPriority(SysTick_IRQn,...) just after the SysTick_Config() function 
	   call. The NVIC_SetPriority() is defined inside the core_cm3.h file.

	4. To adjust the SysTick time base, use the following formula:
	                        
	     Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)

	   - Reload Value is the parameter to be passed for SysTick_Config() function
	   - Reload Value should not exceed 0xFFFFFF
	*/

	/* Capture error */ 
	if (SysTick_Config(SystemCoreClock / 125))
	{ 
		while (1);
	}
}

/**
  * @brief  Initializes the system
  * @param  None
  * @retval None
  */

void System_Init(void)
{
	Sys_NVICConfiguration();
	
	__disable_irq();
	USART1_Init(115200);
	I2Cx_Init();
	IMU_Init();
	Sys_TimeBaseInit();
	__enable_irq();
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	static uint8_t Tim_48ms, Tim_32ms, Tim_500mS;

	if(++ Tim_48ms >= 6)
	{
		IMU_SampleFlag=1;
		Tim_48ms = 0;
	}

	if(++ Tim_32ms >= 4)
	{
		Press_SampleFlag=1;
		Tim_32ms = 0;
	}

	if(++ Tim_500mS >= 63)
	{
		Tim_500mS = 0;
		UART_UpdataFlag = 1;
	}
	
}

/******************* (C) COPYRIGHT 2014 Waveshare *****END OF FILE*******************/

