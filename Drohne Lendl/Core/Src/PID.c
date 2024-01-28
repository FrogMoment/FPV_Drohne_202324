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

#define PID_MAX_VALUE 100
#define PID_MAX_TURN 0.2

#define KP_ROLL  PID_MAX_TURN / 180.0
#define KP_PITCH PID_MAX_TURN / 180.0
#define KP_YAW   PID_MAX_TURN / 180.0

#define KI_ROLL  0
#define KI_PITCH 0
#define KI_YAW   0

#define KD_ROLL  0
#define KD_PITCH 0
#define KD_YAW   0

PID_Status PID_Hover(float inputThrottle)
{
    float dt = 0.01f;
    // float kr = 0.1f;
    // float Tn = 0.001f;
    // float Tv = 0.005f;
    float inputPitch = 0.0f, inputRoll = 0.0f, inputYaw = 0.0f;

    IMU_GetAngles();
    
    float errorPitch = inputPitch - angle.pitch;
    float errorRoll = inputRoll - angle.roll;
    float errorYaw = inputYaw - angle.yaw;

    // static float pitchOutPrev = 0.0f, rollOutPrev = 0.0f, yawOutPrev = 0.0f;
    // static float errorPitchPrev[2] = {0}, errorRollPrev[2] = {0}, errorYawPrev[2] = {0};

    // float pitchOut = pitchOutPrev + kr * (errorPitch - errorPitchPrev[0] + dt/Tn * errorPitch + Tv/dt * (errorPitch - 2 * errorPitchPrev[0] + errorPitchPrev[1]));
    // float rollOut = rollOutPrev + kr * (errorRoll - errorRollPrev[0] + dt/Tn * errorRoll + Tv/dt * (errorRoll - 2 * errorRollPrev[0] + errorRollPrev[1]));
    // float yawOut = yawOutPrev + kr * (errorYaw - errorYawPrev[0] + dt/Tn * errorYaw + Tv/dt * (errorYaw - 2 * errorYawPrev[0] + errorYawPrev[1]));
    
    // // save previos data
    // pitchOutPrev = pitchOut;
    // rollOutPrev = rollOut;
    // yawOutPrev = yawOut;

    // errorPitchPrev[1] = errorPitchPrev[0];
    // errorPitchPrev[0] = errorPitch;
    // errorRollPrev[1] = errorRollPrev[0];
    // errorRollPrev[0] = errorRoll;
    // errorYawPrev[1] = errorYawPrev[0];
    // errorYawPrev[0] = errorYaw;

    // if(rollOut > PID_MAX_VALUE) rollOut = PID_MAX_VALUE;
    // else if(rollOut < -PID_MAX_VALUE) rollOut = -PID_MAX_VALUE;

    float P_Pitch = errorPitch * KP_PITCH;
    float P_Roll = errorRoll * KP_ROLL;
    float P_Yaw = errorYaw * KP_YAW;

    static float I_Pitch = 0, I_Roll = 0, I_Yaw = 0;
    I_Pitch += errorPitch * dt;
    I_Roll += errorRoll * dt;
    I_Yaw += errorYaw * dt;

    if(I_Pitch > 100) I_Pitch = 100; 
    else if(I_Pitch < 0) I_Pitch = 0;
    if(I_Roll > 100) I_Roll = 100; 
    else if(I_Roll < 0) I_Roll = 0;
    if(I_Yaw > 100) I_Yaw = 100; 
    else if(I_Yaw < 0) I_Yaw = 0; 

    static float prevErrorPitch = 0, prevErrorRoll = 0, prevErrorYaw = 0;
    float D_Pitch = (errorPitch - prevErrorPitch) / dt;
    float D_Roll = (errorRoll - prevErrorRoll) / dt;
    float D_Yaw = (errorYaw - prevErrorYaw) / dt;
    prevErrorPitch = errorPitch;
    prevErrorRoll = errorRoll;
    prevErrorYaw = errorYaw;

    float pitchOut = P_Pitch + I_Pitch * KI_PITCH + D_Pitch * KD_PITCH;
    float rollOut = P_Roll + I_Roll * KI_ROLL + D_Roll * KD_ROLL;
    float yawOut = P_Yaw + I_Yaw * KI_YAW + D_Yaw * KD_YAW;

    Motor_Position motor = {0};

    // sprintf(txt, "IMU: %.2f  Stell: %.2f\n\r", angle.pitch, pitchOut);
    sprintf(txt, "%.1f, %.1f, %.1f\n\r", angle.pitch, angle.roll, angle.yaw);
    Terminal_Print(txt);

    return PID_OK;
}

