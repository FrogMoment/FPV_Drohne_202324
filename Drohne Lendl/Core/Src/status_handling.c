/**
 * @file status_handling.c
 * @author Maximilian Lendl
 * @date 2024-01-03
 * @version 1
 *
 * @copyright FPV Drohne DA 202324
 *
 * @brief This file provides functions for:
 *          - Sensor error handler
 *          - terminal print
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
    case DS2438:
      sprintf(txt, "DS2438 ERROR | Code: %d\n\r", errorCode);
      break;

    case RECEIVER:
      sprintf(txt, "RECEIVER ERROR | Code: %d\n\r", errorCode);
      break;

    case IMU:
      sprintf(txt, "IMU ERROR | Code: %d\n\r", errorCode);
      break;

    default:
      sprintf(txt, "wrong sensor ERROR | Code: %d\n\r", errorCode);
      break;
  }
  Terminal_Print(txt);

  __disable_irq();
  while(1);
}

/**
 * @brief This function prints a string to the terminal
 * @param string
 * @retval none
 */
void Terminal_Print(char *string)
{
  HAL_UART_Transmit(TERMINAL_UART, (uint8_t *)string, strlen(string), HAL_MAX_DELAY);
}






