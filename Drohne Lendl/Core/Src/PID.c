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

#define PID_MAX_TURN 5

#define KP_ROLL  0.005
#define KI_ROLL  0
#define KD_ROLL  0.02

#define KP_PITCH 0
#define KI_PITCH 0
#define KD_PITCH 0.075

#define KP_YAW   0
#define KI_YAW   0
#define KD_YAW   0

float PID_dt = 0;


void PID_Init(float deltaTime)
{
    PID_dt = deltaTime;
}


PID_Status PID_Hover(float inputThrottle)
{
    if(inputThrottle > 30)
        inputThrottle = 30;

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
    I_Roll += (errorRoll * imu_DeltaTime) * KI_ROLL;

    if(I_Roll > PID_MAX_TURN) I_Roll = PID_MAX_TURN;
    else if(I_Roll < -PID_MAX_TURN) I_Roll = -PID_MAX_TURN;

    double rollOutput = KP_ROLL * errorRoll + I_Roll + KD_ROLL * ((errorRoll - errorRollPrev) / imu_DeltaTime);
    if(rollOutput > PID_MAX_TURN) rollOutput = PID_MAX_TURN;
    else if(rollOutput < -PID_MAX_TURN) rollOutput = -PID_MAX_TURN;

    errorRollPrev = errorRoll;


    // calculate PID pitch output 
    // double errorPitch = inputPitch - gyro.x;
    double errorPitch = inputPitch - angle.pitch;

    static double I_Pitch = 0, errorPitchPrev = 0;
    I_Pitch += (errorPitch * imu_DeltaTime) * KI_PITCH;

    if(I_Pitch > PID_MAX_TURN) I_Pitch = PID_MAX_TURN;
    else if(I_Pitch < -PID_MAX_TURN) I_Pitch = -PID_MAX_TURN;

    double pitchOutput = KP_PITCH * errorPitch + I_Pitch + KD_PITCH * ((errorPitch - errorPitchPrev) / imu_DeltaTime);
    if(pitchOutput > PID_MAX_TURN) pitchOutput = PID_MAX_TURN;
    else if(pitchOutput < -PID_MAX_TURN) pitchOutput = -PID_MAX_TURN;

    errorPitchPrev = errorPitch;


    // calculate PID yaw output 
    // double errorYaw = inputYaw - gyro.z;
    double errorYaw = inputYaw - angle.yaw;

    static double I_Yaw = 0, errorYawPrev = 0;
    I_Yaw += (errorYaw * imu_DeltaTime) * KI_YAW;

    if(I_Yaw > PID_MAX_TURN) I_Yaw = PID_MAX_TURN;
    else if(I_Yaw < -PID_MAX_TURN) I_Yaw = -PID_MAX_TURN;

    double yawOutput = KP_YAW * errorYaw + I_Yaw + KD_YAW * ((errorYaw - errorYawPrev) / imu_DeltaTime);
    if(yawOutput > PID_MAX_TURN) yawOutput = PID_MAX_TURN;
    else if(yawOutput < -PID_MAX_TURN) yawOutput = -PID_MAX_TURN;

    errorYawPrev = errorYaw;

    
    // check max/min values + output 
    float motorLF = (inputThrottle < 5) ? 0 : inputThrottle + pitchOutput - rollOutput + yawOutput;
    float motorRF = (inputThrottle < 5) ? 0 : inputThrottle + pitchOutput + rollOutput - yawOutput;
    float motorLR = (inputThrottle < 5) ? 0 : inputThrottle - pitchOutput - rollOutput - yawOutput;
    float motorRR = (inputThrottle < 5) ? 0 : inputThrottle - pitchOutput + rollOutput + yawOutput;

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












