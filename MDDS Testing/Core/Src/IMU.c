/**
 * @file IMU.c
 * @author Maximilian Lendl
 * @date 2023-11-20
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief 
 * 
 */

#include "IMU.h"

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

float q0, q1, q2, q3;

/************************************************************************************************
------------------------------------------- FUNCTIONS -------------------------------------------
************************************************************************************************/

/**
 * @brief This function initializes all IMU components
 * @param hi2c pointer to a I2C_HandleTypeDef structure
 * @param dlpf dlpf bandwidth
 * @param gyroFS full scale range of gyroscope
 * @param accelFS full scale range of accelermeter
 * @retval IMU_Status
 */
IMU_Status IMU_Init(I2C_HandleTypeDef *hi2c, MPU9250_DLPFTypeDef dlpf, MPU9250_FullScale gyroFS, MPU9250_FullScale accelFS)
{
    uint8_t status;

    // init MPU9250
    status = MPU9250_Init(hi2c, dlpf, gyroFS, accelFS);
    if(status != MPU9250_OK)
        return status;

    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    return IMU_OK;
}

/**
 * @brief This functions calculates yaw, pitch and roll
 * @param angles IMU_AnglesTypeDef for calculated angles 
 * @retval None
 */
void IMU_GetYawPitchRoll(IMU_AnglesTypDef angles)
{
    IMU_GetQuater();

    angles.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
    angles.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;
    angles.yaw = atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 *q3 - 1) * 57.3;
}

/**
 * @brief This function calculates the quarters 
 * @retval None
 */
void IMU_GetQuater(void)
{
    IMU_CoordTypDef motionGyro, motionAccel, motionMag;
			
    MPU9250_ReadAccel();
    MPU9250_ReadGyro();
    MPU9250_ReadMag();

    motionGyro.x = gyro.x / gyroSens * 0.0175;
    motionGyro.y = gyro.y / gyroSens * 0.0175;
    motionGyro.z = gyro.z / gyroSens * 0.0175;
    motionAccel.x = accel.x / accelSens;
    motionAccel.y = accel.y / accelSens;
    motionAccel.z = accel.z / accelSens;
    motionMag.x = mag.x;
    motionMag.y = mag.y;
    motionMag.z = mag.z;
    
 	IMU_AHRSupdate(motionGyro, motionAccel, motionMag);
}

/**
 * @brief update attitude and heading
 * @param gy gyroscope x,y,z
 * @param ac accelerometer x,y,z
 * @param ma magnetometer x,y,z
 * @retval None
 */
void IMU_AHRSupdate(IMU_CoordTypDef gy, IMU_CoordTypDef ac, IMU_CoordTypDef ma) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;
	float ex, ey, ez, halfT = 0.024f;

	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;   
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;          

	norm = invSqrt(ac.x * ac.x + ac.y * ac.y + ac.z * ac.z);       
	ac.x = ac.x * norm;
	ac.y = ac.y * norm;
	ac.z = ac.z * norm;

	norm = invSqrt(ma.x * ma.x + ma.y * ma.y + ma.z * ma.z);          
	ma.x = ma.x * norm;
	ma.y = ma.y * norm;
	ma.z = ma.z * norm;

	// compute reference direction of flux
	hx = 2 * ma.x * (0.5f - q2q2 - q3q3) + 2 * ma.y * (q1q2 - q0q3) + 2 * ma.z * (q1q3 + q0q2);
	hy = 2 * ma.x * (q1q2 + q0q3) + 2 * ma.y * (0.5f - q1q1 - q3q3) + 2 * ma.z * (q2q3 - q0q1);
	hz = 2 * ma.x * (q1q3 - q0q2) + 2 * ma.y * (q2q3 + q0q1) + 2 * ma.z * (0.5f - q1q1 - q2q2);         
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;     

	// estimated direction of gravity and flux (v and w)
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
	wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
	wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);  

	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ac.y * vz - ac.z * vy) + (ma.y * wz - ma.z * wy);
	ey = (ac.z * vx - ac.x * vz) + (ma.z * wx - ma.x * wz);
	ez = (ac.x * vy - ac.y * vx) + (ma.x * wy - ma.y * wx);

	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;	
		ezInt = ezInt + ez * Ki * halfT;

		gy.x = gy.x + Kp * ex + exInt;
		gy.y = gy.y + Kp * ey + eyInt;
		gy.z = gy.z + Kp * ez + ezInt;
	}

	q0 = q0 + (-q1 * gy.x - q2 * gy.y - q3 * gy.z) * halfT;
	q1 = q1 + (q0 * gy.x + q2 * gy.z - q3 * gy.y) * halfT;
	q2 = q2 + (q0 * gy.y - q1 * gy.z + q3 * gy.x) * halfT;
	q3 = q3 + (q0 * gy.z + q1 * gy.y - q2 * gy.x) * halfT;  

	norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
}

/**
 * @brief This function inverts the squareroot of x
 * @param x 
 * @return float 
 */
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	
	long i = *(long*)&y;                //get bits for floating value
	i = 0x5f3759df - (i >> 1);          //gives initial guss you
	y = *(float*)&i;                    //convert bits back to float
	y = y * (1.5f - (halfx * y * y));   //newtop step, repeating increases accuracy
	
	return y;
}