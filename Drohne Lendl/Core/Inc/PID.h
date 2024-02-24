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

extern int8_t uartCheck;

void PID_Init(float deltaTime);

PID_Status PID_Hover(float inputThrottle);

void changeKs(void);




#endif // PID_H_INCLUDED



