/**
 * @file status_handling.c
 * @author Maximilian Lendl
 * @date 2024-01-03
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief 
 * 
 */

#include "status_handling.h"

/**
 * @brief This funciton completly stops the program 
 * @param sens what sensor has the error
 * @param errorCode 
 * @retval None
 */
void Sensor_ErrorHandler(Sensors sens, int8_t errorCode)
{
  char txt[100];
  switch(sens)
  {
    // case MPU9250:
    //   sprintf(txt, "MPU9250 ERROR | Code: %d\n\r", errorCode);
    //   break;

    case DS2438:
      sprintf(txt, "DS2438 ERROR | Code: %d\n\r", errorCode);
      break;

    case RECEIVER:
      sprintf(txt, "RECEIVER ERROR | Code: %d\n\r", errorCode);
      break;

    case IMU:
      sprintf(txt, "IMU ERROR | Code: %d\n\r", errorCode);
      break;

    case DATA_TRANSMIT:
      sprintf(txt, "DATA TRANSMIT ERROR | Code: %d\n\r", errorCode);
      break;

    default:
      sprintf(txt, "wrong sensor ERROR | Code: %d\n\r", errorCode);
      break;
  }
  HAL_UART_Transmit(&huart4, (uint8_t *)&txt, strlen(txt), HAL_MAX_DELAY);

  __disable_irq();
  while(1);
}