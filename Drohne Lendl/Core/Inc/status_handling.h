/**
 * @file status_handling.h
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

#ifndef STATUS_HANDLING_H_INCLUDED
#define STATUS_HANDLING_H_INCLUDED

#include "main.h"
#include <string.h>
#include <stdio.h>

typedef enum Sensors
{
  DATA_TRANSMIT = 1,
  DS2438 = 2,
  IMU = 3,
  RECEIVER = 4,
  DSHOT = 5,
  PID = 6
} Sensors;

/**
 * @brief This funciton completly stops the program
 * @param sens what sensor has the error
 * @param errorCode
 * @retval None
 */
void Sensor_ErrorHandler(Sensors sens, int8_t errorCode);

/**
 * @brief This function prints a string to the terminal
 * @param string
 * @retval none
 */
void Terminal_Print(char *string);

#endif // STATUS_HANDLING_H_INCLUDED

