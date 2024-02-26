/**
 * @file PID.c
 * @author Maximilian Lendl
 * @date 2024-01-22
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief This file provides functions for:
 *          - PID algorithm for hover mode
 */

#include "PID.h"
#include <stdlib.h>

#define PID_MAX_TURN 10

/**
 * [0] ... KP Roll
 * [1] ... KI Roll
 * [2] ... KD Roll
 * 
 * [3] ... KP Pitch
 * [4] ... KI Pitch
 * [5] ... KD Pitch
 * 
 * [6] ... KP Yaw 
 * [7] ... KI Yaw 
 * [8] ... KD Yaw 
 */
float allKs[9] = {0.0};

char receiveData[7];

/**
 * @brief This function initializes the change Ks system 
 * @param huart pointer to UART_HandleTypeDef (input uart)
 * @retval None
 */
void PID_Init(UART_HandleTypeDef *huart)
{
    // start reception with uart
    HAL_UART_RegisterCallback(huart, HAL_UART_RX_COMPLETE_CB_ID, changeKs);

    HAL_UART_Receive_DMA(huart, (uint8_t *)&receiveData, 6);
}

/**
 * @brief This function controls hover mode with a PID controller
 * @param inputThrottle throttle value 0-100%
 * @return PID_Status 
 */
PID_Status PID_Hover(float inputThrottle)
{
    if(inputThrottle > 30)
        inputThrottle = 30;

    double inputPitch = 0.0f, inputRoll = 0.0f, inputYaw = 0.0f;

    // get current angles + delta time
    IMU_GetAngles();


    // calculate PID roll output 
    double errorRoll = inputRoll - angle.roll;
    
    static double I_Roll = 0, errorRollPrev = 0;
    I_Roll += (errorRoll * imu_DeltaTime) * allKs[1];

    if(I_Roll > PID_MAX_TURN) I_Roll = PID_MAX_TURN;
    else if(I_Roll < -PID_MAX_TURN) I_Roll = -PID_MAX_TURN;

    double rollOutput = allKs[0] * errorRoll + I_Roll + allKs[2] * ((errorRoll - errorRollPrev) / imu_DeltaTime);
    if(rollOutput > PID_MAX_TURN) rollOutput = PID_MAX_TURN;
    else if(rollOutput < -PID_MAX_TURN) rollOutput = -PID_MAX_TURN;

    errorRollPrev = errorRoll;


    // calculate PID pitch output 
    double errorPitch = inputPitch - angle.pitch;

    static double I_Pitch = 0, errorPitchPrev = 0;
    I_Pitch += (errorPitch * imu_DeltaTime) * allKs[4];

    if(I_Pitch > PID_MAX_TURN) I_Pitch = PID_MAX_TURN;
    else if(I_Pitch < -PID_MAX_TURN) I_Pitch = -PID_MAX_TURN;

    double pitchOutput = allKs[3] * errorPitch + I_Pitch + allKs[5] * ((errorPitch - errorPitchPrev) / imu_DeltaTime);
    if(pitchOutput > PID_MAX_TURN) pitchOutput = PID_MAX_TURN;
    else if(pitchOutput < -PID_MAX_TURN) pitchOutput = -PID_MAX_TURN;

    errorPitchPrev = errorPitch;


    // calculate PID yaw output 
    double errorYaw = inputYaw - angle.yaw;

    static double I_Yaw = 0, errorYawPrev = 0;
    I_Yaw += (errorYaw * imu_DeltaTime) * allKs[7];

    if(I_Yaw > PID_MAX_TURN) I_Yaw = PID_MAX_TURN;
    else if(I_Yaw < -PID_MAX_TURN) I_Yaw = -PID_MAX_TURN;

    double yawOutput = allKs[6] * errorYaw + I_Yaw + allKs[8] * ((errorYaw - errorYawPrev) / imu_DeltaTime);
    if(yawOutput > PID_MAX_TURN) yawOutput = PID_MAX_TURN;
    else if(yawOutput < -PID_MAX_TURN) yawOutput = -PID_MAX_TURN;

    errorYawPrev = errorYaw;

    
    // check max/min values + output 
    float motorLF = (inputThrottle < 5) ? 0 : inputThrottle - pitchOutput - rollOutput + yawOutput;
    float motorRF = (inputThrottle < 5) ? 0 : inputThrottle - pitchOutput + rollOutput - yawOutput;
    float motorLR = (inputThrottle < 5) ? 0 : inputThrottle + pitchOutput - rollOutput - yawOutput;
    float motorRR = (inputThrottle < 5) ? 0 : inputThrottle + pitchOutput + rollOutput + yawOutput;

    if(motorLF < 5) motorLF = 0;
    if(motorRF < 5) motorRF = 0;
    if(motorLR < 5) motorLR = 0;
    if(motorRR < 5) motorRR = 0;

    if(motorLF > inputThrottle + PID_MAX_TURN) motorLF = inputThrottle + PID_MAX_TURN;
    if(motorRF > inputThrottle + PID_MAX_TURN) motorRF = inputThrottle + PID_MAX_TURN;
    if(motorLR > inputThrottle + PID_MAX_TURN) motorLR = inputThrottle + PID_MAX_TURN;
    if(motorRR > inputThrottle + PID_MAX_TURN) motorRR = inputThrottle + PID_MAX_TURN;

    DShot_SendThrottle(motorLF, motorRF, motorLR, motorRR);

    return PID_OK;
}

/**
 * @brief This function changes a PID controller coefficients via uart
 * @param huart 
 */
void changeKs(UART_HandleTypeDef *huart)
{
    int index = atoi(&receiveData[0]);
    char value[5];
    sprintf(value, "%c%c%c%c", receiveData[2], receiveData[3], receiveData[4], receiveData[5]);

    allKs[index] = atof(value);
    sprintf(txt, "[%d] = %f\n\r", index, allKs[index]);
    Terminal_Print(txt);
}












