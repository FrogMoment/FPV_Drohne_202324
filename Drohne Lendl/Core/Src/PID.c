/**
 * @file PID.c
 * @author Maximilian Lendl
 * @date 2024-01-22
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief 
 * 
 */

#include "PID.h"

#define PID_MAX_TURN 5

#define KP_ROLL  0
#define KI_ROLL  0
#define KD_ROLL  0

#define KP_PITCH KP_ROLL
#define KI_PITCH KI_ROLL
#define KD_PITCH KD_ROLL

#define KP_YAW   0
#define KI_YAW   0
#define KD_YAW   0

PID_Status PID_Hover(float inputThrottle)
{
    float dt = 0.01f;
    float inputPitch = 0.0f, inputRoll = 0.0f, inputYaw = 0.0f;

    IMU_RegCoordinates gyroData = IMU_MPU_ReadGyro();

    gyro.x = (gyroData.x - gyroOffset.x) / gyroSens;
    gyro.y = (gyroData.y - gyroOffset.y) / gyroSens;
    gyro.z = (gyroData.z - gyroOffset.z) / gyroSens;
    
    // roll
    float errorRoll = inputRoll - gyro.y;
    
    static float I_Roll = 0, errorRollPrev = 0;
    I_Roll += errorRoll * KI_ROLL;

    if(I_Roll > PID_MAX_TURN) I_Roll = PID_MAX_TURN;
    else if(I_Roll < -PID_MAX_TURN) I_Roll = -PID_MAX_TURN;

    float rollOutput = KP_ROLL * errorRoll + I_Roll + KD_ROLL * (errorRoll - errorRollPrev);
    if(rollOutput > PID_MAX_TURN) rollOutput = PID_MAX_TURN;
    else if(rollOutput < -PID_MAX_TURN) rollOutput = -PID_MAX_TURN;

    errorRollPrev = errorRoll;

    // pitch
    float errorPitch = inputPitch - gyro.x;

    static float I_Pitch = 0, errorPitchPrev = 0;
    I_Pitch += errorPitch * KI_PITCH;

    if(I_Pitch > PID_MAX_TURN) I_Pitch = PID_MAX_TURN;
    else if(I_Pitch < -PID_MAX_TURN) I_Pitch = -PID_MAX_TURN;

    float pitchOutput = KP_PITCH * errorPitch + I_Pitch + KD_PITCH * (errorPitch - errorPitchPrev);
    if(pitchOutput > PID_MAX_TURN) pitchOutput = PID_MAX_TURN;
    else if(pitchOutput < -PID_MAX_TURN) pitchOutput = -PID_MAX_TURN;

    errorPitchPrev = errorPitch;

    // yaw
    float errorYaw = inputYaw - gyro.z;
    static float I_Yaw = 0, errorYawPrev = 0;
    I_Yaw += errorYaw * KI_YAW;

    if(I_Yaw > PID_MAX_TURN) I_Yaw = PID_MAX_TURN;
    else if(I_Yaw < -PID_MAX_TURN) I_Yaw = -PID_MAX_TURN;

    float yawOutput = KP_YAW * errorYaw + I_Yaw + KD_YAW * (errorYaw - errorYawPrev);
    if(yawOutput > PID_MAX_TURN) yawOutput = PID_MAX_TURN;
    else if(yawOutput < -PID_MAX_TURN) yawOutput = -PID_MAX_TURN;

    errorYawPrev = errorYaw;

    // output
    float motorLF = inputThrottle - pitchOutput + rollOutput - yawOutput;
    float motorRF = inputThrottle - pitchOutput - rollOutput + yawOutput;
    float motorLR = inputThrottle + pitchOutput + rollOutput + yawOutput;
    float motorRR = inputThrottle + pitchOutput - rollOutput - yawOutput;

    DShot_SendThrottle(motorLF, motorRF, motorLR, motorRR);

    return PID_OK;
}

