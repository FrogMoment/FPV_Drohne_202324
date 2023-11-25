/**
  ******************************************************************************
  * @file    main.c 
  * @author  Waveshare Team
  * @version V1.0
  * @date    29-August-2014
  * @brief   Main program body.
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

int16_t TestmagnBuff[9]={0};
bool Flag_Check_Magn= false; 
extern uint32_t KeyAndJostickValue;
extern MPU9250_TypeDef_Off MPU9250_Magn_Offset;
void TestMagn(void); 
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured, 
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32f10x_xx.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32f10x.c file
	   */ 
	//float Angle;

	System_Init();	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
  LedGpioConfiguration();
 	JoystickGpioConfiguration();
	
	while(1)
	{	
		if(!Flag_Check_Magn)
		{
			if(KyeAndJoystickScan())
			TestMagn();
			LedShow();	
		}
		if(IMU_SampleFlag)
		{
			IMU_GetYawPitchRoll(angles);	
			IMU_SampleFlag=0;
		}
		if(Press_SampleFlag)
		{ 				
			CalTemperatureAndPressureAndAltitude();
			Press_SampleFlag=0;
		}
		if(UART_UpdataFlag)
		{		    
			printf("\r\n /-------------------------------------------------------------/ \r\n");
			printf("\r\n Roll: %.2f     Pitch: %.2f     Yaw: %.2f \r\n",angles[2],angles[1],angles[0]);
			printf("\r\n Acceleration: X: %d     Y: %d     Z: %d \r\n",accel[0],accel[1],accel[2]);
			printf("\r\n Gyroscope: X: %d     Y: %d     Z: %d \r\n",gyro[0],gyro[1],gyro[2]);
 			printf("\r\n Magnetic: X: %d     Y: %d     Z: %d \r\n",magn[0],magn[1],magn[2]);
 			printf("\r\n Pressure: %.2f     Altitude: %.2f \r\n",(float)PressureVal / 100, (float)AltitudeVal / 100);
 			printf("\r\n Temperature: %.1f \r\n", (float)TemperatureVal / 10);
	
			UART_UpdataFlag = 0;
		}
	}
}
/**
  * @brief  check magn
  * @param  None
  * @retval  none
  */
void TestMagn(void)
{
	TestmagnBuff[(KeyAndJostickValue-1)*3]=magn[0];
	TestmagnBuff[(KeyAndJostickValue-1)*3+1]=magn[1];
	TestmagnBuff[(KeyAndJostickValue-1)*3+2]=magn[2];
	
	if(KeyAndJostickValue>=3)
	{
			MPU9250_Magn_Offset.X_Off_Err=(TestmagnBuff[0]+TestmagnBuff[3])/2;
			MPU9250_Magn_Offset.Y_Off_Err=(TestmagnBuff[1]+TestmagnBuff[4])/2;
			MPU9250_Magn_Offset.Z_Off_Err=(TestmagnBuff[5]+TestmagnBuff[8])/2;
			Flag_Check_Magn=true;
	}
}
/******************* (C) COPYRIGHT 2014 Waveshare *****END OF FILE*******************/
