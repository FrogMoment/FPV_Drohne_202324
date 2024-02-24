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

// #define KP_ROLL  0.3f // 0.3f
// #define KI_ROLL  0.05f 
// #define KD_ROLL  0

// #define KP_PITCH 0 // 0.7f
// #define KI_PITCH 0 
// #define KD_PITCH 0

// #define KP_YAW   0
// #define KI_YAW   0
// #define KD_YAW   0

float KP_R = 0.3f; // 0.3f
float KI_R = 0.05f; 
float KD_R = 0;

float KP_P = 0; // 0.7f
float KI_P = 0; 
float KD_P = 0;

float KP_Y = 0;
float KI_Y = 0;
float KD_Y = 0;

float PID_dt = 0;

int8_t uartCheck = 1;


void PID_Init(float deltaTime)
{
    PID_dt = deltaTime;
}


PID_Status PID_Hover(float inputThrottle)
{
    if(inputThrottle > 30)
        inputThrottle = 30;

    if(uartCheck == 1)
    {
        changeKs();
        uartCheck = 0;
    }

    double inputPitch = 0.0f, inputRoll = 0.0f, inputYaw = 0.0f;

    // // read angular velocity
    // IMU_RegCoordinates gyroData = IMU_MPU_ReadGyro();

    // gyro.x = (gyroData.x - gyroOffset.x) / gyroSens;
    // gyro.y = (gyroData.y - gyroOffset.y) / gyroSens;
    // gyro.z = (gyroData.z - gyroOffset.z) / gyroSens;
    // gyro.z = -gyro.z;


    // get current angles + delta time
    IMU_GetAngles();


    // calculate PID roll output 
    // double errorRoll = inputRoll - gyro.y;
    double errorRoll = inputRoll - angle.roll;
    
    static double I_Roll = 0, errorRollPrev = 0;
    I_Roll += (errorRoll * imu_DeltaTime) * KI_R;

    if(I_Roll > PID_MAX_TURN) I_Roll = PID_MAX_TURN;
    else if(I_Roll < -PID_MAX_TURN) I_Roll = -PID_MAX_TURN;

    double rollOutput = KP_R * errorRoll + I_Roll + KD_R * ((errorRoll - errorRollPrev) / imu_DeltaTime);
    if(rollOutput > PID_MAX_TURN) rollOutput = PID_MAX_TURN;
    else if(rollOutput < -PID_MAX_TURN) rollOutput = -PID_MAX_TURN;

    errorRollPrev = errorRoll;


    // calculate PID pitch output 
    // double errorPitch = inputPitch - gyro.x;
    double errorPitch = inputPitch - angle.pitch;

    static double I_Pitch = 0, errorPitchPrev = 0;
    I_Pitch += (errorPitch * imu_DeltaTime) * KI_P;

    if(I_Pitch > PID_MAX_TURN) I_Pitch = PID_MAX_TURN;
    else if(I_Pitch < -PID_MAX_TURN) I_Pitch = -PID_MAX_TURN;

    double pitchOutput = KP_P * errorPitch + I_Pitch + KD_P * ((errorPitch - errorPitchPrev) / imu_DeltaTime);
    if(pitchOutput > PID_MAX_TURN) pitchOutput = PID_MAX_TURN;
    else if(pitchOutput < -PID_MAX_TURN) pitchOutput = -PID_MAX_TURN;

    errorPitchPrev = errorPitch;


    // calculate PID yaw output 
    // double errorYaw = inputYaw - gyro.z;
    double errorYaw = inputYaw - angle.yaw;

    static double I_Yaw = 0, errorYawPrev = 0;
    I_Yaw += (errorYaw * imu_DeltaTime) * KI_Y;

    if(I_Yaw > PID_MAX_TURN) I_Yaw = PID_MAX_TURN;
    else if(I_Yaw < -PID_MAX_TURN) I_Yaw = -PID_MAX_TURN;

    double yawOutput = KP_Y * errorYaw + I_Yaw + KD_Y * ((errorYaw - errorYawPrev) / imu_DeltaTime);
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

void changeKs(void)
{
    char receive[100] = "hi";
    
    if(HAL_UART_Receive(&huart4, (uint8_t *)&receive, 9, HAL_MAX_DELAY) == HAL_OK)
    {
        char variable[5];
        sprintf(variable, "%c%c%c%c", receive[0], receive[1], receive[2], receive[3]);
        char value[5];
        sprintf(value, "%c%c%c%c", receive[5], receive[6], receive[7], receive[8]);

        if(strcmp(variable, "KP_R\0") == 0)
        {
            KP_R = atof(value);
            sprintf(txt, "KP_R = %f\n\r", KP_R);
            Terminal_Print(txt);
        }
        else if(strcmp(variable, "KI_R\0") == 0)
        {
            KI_R = atof(value);
            sprintf(txt, "KI_R = %f\n\r", KI_R);
            Terminal_Print(txt);
        }
        else if(strcmp(variable, "KD_R\0") == 0)
        {
            KD_R = atof(value);
            sprintf(txt, "KD_R = %f\n\r", KD_R);
            Terminal_Print(txt);
        }
        else if(strcmp(variable, "KP_P\0") == 0)
        {
            KP_P = atof(value);
            sprintf(txt, "KP_P = %f\n\r", KP_P);
            Terminal_Print(txt);
        }
        else if(strcmp(variable, "KI_P\0") == 0)
        {
            KI_P = atof(value);
            sprintf(txt, "KI_P = %f\n\r", KI_P);
            Terminal_Print(txt);
        }
        else if(strcmp(variable, "KD_P\0") == 0)
        {
            KD_P = atof(value);
            sprintf(txt, "KD_P = %f\n\r", KD_P);
            Terminal_Print(txt);
        }
        else if(strcmp(variable, "KP_Y\0") == 0)
        {
            KP_Y = atof(value);
            sprintf(txt, "KP_Y = %f\n\r", KP_Y);
            Terminal_Print(txt);
        }
        else if(strcmp(variable, "KI_Y\0") == 0)
        {
            KI_Y = atof(value);
            sprintf(txt, "KI_Y = %f\n\r", KI_Y);
            Terminal_Print(txt);
        }
        else if(strcmp(variable, "KD_Y\0") == 0)
        {
            KD_Y = atof(value);
            sprintf(txt, "KD_Y = %f\n\r", KD_Y);
            Terminal_Print(txt);
        }
    }
}












