/**
 * @file IMU_10DOF.h
 * @author Maximilian Lendl
 * @date 2023-11-17
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief 
 * 
 */

#include "main.h"


typedef enum IMU_Status
{
    IMU_OK = 0,
    IMU_ERROR = 1
} IMU_Status;

IMU_Status IMU_Init(void);