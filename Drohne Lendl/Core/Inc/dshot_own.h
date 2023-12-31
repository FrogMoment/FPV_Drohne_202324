/**
 * @file dshot_own.h
 * @author Maximilian Lendl
 * @date 2023-12-01
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief This file provides functions for:
 *          - DShot600 output init 
 *          - DShot600 output special command 
 *          - DShot600 output throttle 
 *          - DShot600 ESC / motor test 
 */

#ifndef DSHOT_H_INCLUDED
#define DSHOT_H_INCLUDED

/*******************************************************************************************
----------------------------------------- INCLUDES -----------------------------------------
*******************************************************************************************/

#include "main.h"
#include <string.h>
#include <stdio.h>

/*******************************************************************************************
-------------------------------------- GLOBAL DEFINES --------------------------------------
*******************************************************************************************/

#define ESC_LF_TIM_CH TIM_CHANNEL_4 // timer channel for the left front motor
#define ESC_RF_TIM_CH TIM_CHANNEL_2 // timer channel for the right front motor
#define ESC_LR_TIM_CH TIM_CHANNEL_3 // timer channel for the left rear motor
#define ESC_RR_TIM_CH TIM_CHANNEL_1 // timer channel for the right rear motor

// returns the address of the CCR for the selected channel
#define ESC_TIM_GET_CCR_ADDR(__CHANNEL__) \
    ((__CHANNEL__ == TIM_CHANNEL_1) ? ((uint32_t)&DShot_OutputTim->Instance->CCR1) : \
     (__CHANNEL__ == TIM_CHANNEL_2) ? ((uint32_t)&DShot_OutputTim->Instance->CCR2) : \
     (__CHANNEL__ == TIM_CHANNEL_3) ? ((uint32_t)&DShot_OutputTim->Instance->CCR3) : \
     ((uint32_t)&DShot_OutputTim->Instance->CCR4))

#define ESC_LF_DMA_ID (ESC_LF_TIM_CH / 4) + 1 // TIM_DMA_ID_CCX (X depends on the timer channel, what capture compare)
#define ESC_RF_DMA_ID (ESC_RF_TIM_CH / 4) + 1 // TIM_DMA_ID_CCX (X depends on the timer channel, what capture compare)
#define ESC_LR_DMA_ID (ESC_LR_TIM_CH / 4) + 1 // TIM_DMA_ID_CCX (X depends on the timer channel, what capture compare)
#define ESC_RR_DMA_ID (ESC_RR_TIM_CH / 4) + 1 // TIM_DMA_ID_CCX (X depends on the timer channel, what capture compare)

/*******************************************************************************************
------------------------------------ GLOBAL STRUCTURES ------------------------------------
*******************************************************************************************/

// DShot error codes
typedef enum DShot_Status
{
    DSHOT_OK = 0,

    DSHOT_TIM_ERROR = 1
} DShot_Status;

// DShot special commands (0-47)
typedef enum DShot_Command
{
    DSHOT_MOTOR_STOP = 0,   // disarmes motor
    DSHOT_BEEP_1 = 1,       // wait at least 260ms before next command
    DSHOT_BEEP_2 = 2,       // wait at least 260ms before next command
    DSHOT_BEEP_3 = 3,       // wait at least 260ms before next command
    DSHOT_BEEP_4 = 4,       // wait at least 260ms before next command
    DSHOT_BEEP_5 = 5        // wait at least 260ms before next command

} DShot_Command;

/*******************************************************************************************
----------------------------------- FUNCTION PROTOTYPES -----------------------------------
*******************************************************************************************/

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
void DShot_SendThrottle(double motorLF, double motorRF, double motorLR, double motorRR);

/**
 * @brief This function sends a command to the ESC via DShot protocol
 * @param command 
 * @retval None
 */
void DShot_SendCommand(DShot_Command command);

/**
 * @brief This function formats and sends DShot data via PWM DMA
 * @param throttle Throttle values (0-2047)
 * @param telemetry telemetry request bit
 */
void DShot_SendData(uint16_t *throttle, int8_t telemetry);

/**
 * @brief This function tests the motors
 * @retval None
 */
void DShot_MotorTest(void);

#endif

