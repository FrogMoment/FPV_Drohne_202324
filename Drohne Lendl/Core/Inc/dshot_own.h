/**
 * @file dshot_own.h
 * @author Maximilian Lendl
 * @date 2023-12-01
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief 
 * 
 */

#ifndef DSHOT_H_INCLUDED
#define DSHOT_H_INCLUDED

#include "main.h"
#include <string.h>
#include <stdio.h>

typedef enum DShot_Status
{
    DSHOT_OK = 0,

    DSHOT_TIM_ERROR = 1
} DShot_Status;

/**
 * @brief This function initializes the output ESC DShot signal
 * @param htim pointer to TIM_HandleTypeDef (output timer)
 * @return DShot_Status 
 */
DShot_Status DShot_Init(TIM_HandleTypeDef *htim);

/**
 * @brief This function formats the motor data for the DShot protocol
 * @param motorLF percent of throttle value of left front motor
 * @param motorRF percent of throttle value of right front motor
 * @param motorLR percent of throttle value of left rear motor
 * @param motorRR percent of throttle value of right rear
 * @retval None
 */
void DShot_SendData(double motorLF, double motorRF, double motorLR, double motorRR);

/**
 * @brief This function starts the sending process of the DShot protocol
 * @attention This function sends the data that is stored in data[0-3]
 * @return None 
 */
void DShot_StartSending(void);

#endif

