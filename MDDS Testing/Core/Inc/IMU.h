/**
 * @file IMU.h
 * @author Maximilian Lendl
 * @date 2023-11-20
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief 
 * 
 */

#ifndef IMU_H_INCLUDED
#define IMU_H_INCLUDED

/************************************************************************************************
------------------------------------------- INCLUDES -------------------------------------------
************************************************************************************************/

#include "main.h"
#include "MPU9250V3.h"

/************************************************************************************************
--------------------------------------- GLOBAL STRUCTURES ---------------------------------------
************************************************************************************************/

// IMU Errorcodes
typedef enum IMU_Status
{
    IMU_OK = 0,

    IMU_MPU9250_WHOAMI_ERROR = 1,
    IMU_MPU9250_I2C_ERROR = 2
} IMU_Status;

// orientation angels
typedef struct IMU_AnglesTypDef
{
    float pitch, roll, yaw;
} IMU_AnglesTypDef;

// x, y, z coordinates
typedef struct IMU_CoordTypDef
{
    float x, y, z;
} IMU_CoordTypDef;

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

extern IMU_AnglesTypDef MPU9250_Angles;

#define Kp 4.50f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 1.0f    // integral gain governs rate of convergence of gyroscope biases

/************************************************************************************************
-------------------------------------- FUNCTION PROTOTYPES --------------------------------------
************************************************************************************************/

/**
 * @brief This function initializes all IMU components
 * @param hi2c pointer to a I2C_HandleTypeDef structure
 * @param dlpf dlpf bandwidth
 * @param gyroFS full scale range of gyroscope
 * @param accelFS full scale range of accelermeter
 * @retval IMU_Status
 */
IMU_Status IMU_Init(I2C_HandleTypeDef *hi2c, MPU9250_DLPFTypeDef dlpf, MPU9250_FullScale gyroFS, MPU9250_FullScale accelFS);

/**
 * @brief This functions calculates yaw, pitch and roll
 * @param angles IMU_AnglesTypeDef for calculated angles 
 * @retval None
 */
void IMU_GetYawPitchRoll(IMU_AnglesTypDef angles);

/**
 * @brief This function calculates the quarters 
 * @retval None
 */
void IMU_GetQuater(void);

/**
 * @brief update attitude and heading
 * @param gy gyroscope x,y,z
 * @param ac accelerometer x,y,z
 * @param ma magnetometer x,y,z
 * @retval None
 */
void IMU_AHRSupdate(IMU_CoordTypDef gy, IMU_CoordTypDef ac, IMU_CoordTypDef ma); 

/**
 * @brief This function inverts the squareroot of x
 * @param x 
 * @return float 
 */
float invSqrt(float x);



#endif // IMU_H_INCLUDED