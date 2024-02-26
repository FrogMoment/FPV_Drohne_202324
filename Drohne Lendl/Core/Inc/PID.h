/**
 * @file PID.h
 * @author Maximilian Lendl
 * @date 2024-01-22
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief This file provides functions for:
 *          - PID algorithm for hover mode
 */

#ifndef PID_H_INCLUDED
#define PID_H_INCLUDED

#include "main.h"
#include <math.h>
#include "IMU_10DOF.h"
#include "dshot_own.h"

typedef enum PID_Status
{
    PID_OK = 0,

    PID_RECEIVER_ERROR = 1
} PID_Status;

/**
 * @brief This function initializes the change Ks system 
 * @param huart pointer to UART_HandleTypeDef (input uart)
 * @retval None
 */
void PID_Init(UART_HandleTypeDef *huart);

/**
 * @brief This function controls hover mode with a PID controller
 * @param inputThrottle throttle value 0-100%
 * @return PID_Status 
 */
PID_Status PID_Hover(float inputThrottle);

/**
 * @brief This function changes a PID controller coefficients via uart
 * @param huart 
 */
void changeKs(UART_HandleTypeDef *huart);



#endif // PID_H_INCLUDED



