/**
 * @file MPU9250V3.h
 * @author Maximilian Lendl
 * @date 2023-11-20
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief This file provides functions for:
 *              - initialization of MPU9250 + read/write functions
 *              - measurement for accelerometer (x, y, z)
 *              - measurement for gyroscope (x, y, z)
 *              - measurement for magnetometer (x, y, z)
 */

#ifndef MPU9250V3_H_INCLUDED
#define MPU9250V3_H_INCLUDED

/************************************************************************************************
------------------------------------------- INCLUDES -------------------------------------------
************************************************************************************************/

#include "main.h"
#include <math.h>

/************************************************************************************************
---------------------------------------- GLOBAL DEFINES ----------------------------------------
************************************************************************************************/

#define MPU9250_I2C_SLV 0x69        // I2C slave address
#define MPU9250_I2C_MAG 0x0C        // magnetometer I2C address

// MPU9250 register addresses
#define MPU9250_SMPLRT_DIV 0x19     // Sample Rate Divider register address
#define MPU9250_CONFIG 0x1A         // configuration register address
#define MPU9250_GYRO_CONFIG 0x1B    // gyroscope configuration register address
#define MPU9250_ACCEL_CONFIG 0x1C   // accelerometer configuration 1 register address
#define MPU9250_PWR_MGMT_1 0x6B     // Power Management 1 register address  
#define MPU9250_WHOAMI 0x75         // WHO AM I register address
#define MPU9250_BYPASS_EN 0x37      // bypass enable register

// MPU9250 output registers
#define MPU9250_ACCEL_XOUT_H 0x3B   // Accelerometer measurements X-Axis High register address 
#define MPU9250_TEMP_OUT_H 0x41     // Temperature sensor measurements High register address 
#define MPU9250_GYRO_XOUT_H 0x43    // Gyroscope measurements X-Axis High register address 

// magnetometer registers
#define MAG_XOUT_L 0x03 // Gyroscope measurements X-Axis High register address 
#define MAG_CNTL1 0x0A  // magnetometer control register 1 address

// max sensitivity scale factors (SSF)
#define ACCEL_SENS 16384.0      // SSF of accelerometer
#define GYRO_SENS 131.0         // SSF of gyroscope

/************************************************************************************************
--------------------------------------- GLOBAL STRUCTURES ---------------------------------------
************************************************************************************************/

typedef enum MPU9250_Status
{
    MPU9250_OK = 0,

    MPU9250_WHOAMI_ERROR = 1,
    MPU9250_I2C_ERROR = 2
} MPU9250_Status;

// full scale ranges for gyroscope and accelerometer
typedef enum MPU9250_FullScale
{
    GYRO_250DPS = 0,    // gyroscope full scale range +250°/s
    GYRO_500DPS = 1,    // gyroscope full scale range +500°/s
    GYRO_1000DPS = 2,   // gyroscope full scale range +1000°/s
    GYRO_2000DPS = 3,	// gyroscope full scale range +2000°/s

    ACCEL_2G = 0,       // accelerometer full scale range +-2g
    ACCEL_4G = 1,       // accelerometer full scale range +-4g
    ACCEL_8G = 2,       // accelerometer full scale range +-8g
    ACCEL_16G = 3       // accelerometer full scale range +-16g
} MPU9250_FullScale;

// x, y, z coordinates
typedef struct MPU9250_CoordTypDef
{
    float x, y, z;
} MPU9250_CoordTypDef;

typedef struct MPU9250_AvgTypeDef
{
	uint8_t Index;
	int16_t AvgBuffer[8];
} MPU9250_AvgTypeDef;

// digital lowpass filter configuration (DLPF)
typedef enum MPU9250_DLPFTypeDef
{
    DLPF_250Hz = 0,
    DLPF_184Hz = 1,
    DLPF_92Hz = 2,
    DLPF_41Hz = 3,
    DLPF_20Hz = 4,
    DLPF_10Hz = 5,
    DLPF_5Hz = 6,
    DLPF_3600Hz = 7
} MPU9250_DLPFTypeDef;

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

extern int16_t mpu9250_RawData[14]; // mpu9250 raw data

extern MPU9250_CoordTypDef accel;   // accelerometer values
extern MPU9250_CoordTypDef gyro;    // gyroscope values
extern MPU9250_CoordTypDef mag;     // gyroscope values
extern float mpu9250_Temp;          // temperature values

extern float accelSens;     // sensitivity scale factor of accelerometer
extern float gyroSens;      // sensitivity scale factor of gyroscope

/************************************************************************************************
-------------------------------------- FUNCTION PROTOTYPES --------------------------------------
************************************************************************************************/

/**
 * @brief This funtion initializes the MPU9250
 * @param hi2c pointer to a I2C_HandleTypeDef structure
 * @param dlpf dlpf bandwidth
 * @param gyroFS full scale range of gyroscope
 * @param accelFS full scale range of accelermeter
 * @return MPU9250_Status
 */
MPU9250_Status MPU9250_Init(I2C_HandleTypeDef *hi2c, MPU9250_DLPFTypeDef dlpf, MPU9250_FullScale gyroFS, MPU9250_FullScale accelFS);

/**
 * @brief read register of MPU9250
 * @param i2c_Addr i2c slave address
 * @param addr register address/es
 * @param data received data
 * @param rxBytes amount of reveived bytes
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @retval None
 */
void MPU9250_ReadRegister(uint8_t i2c_Addr, uint8_t addr, uint8_t *data, int8_t rxBytes);

/**
 * @brief write to register of MPU9250
 * @param i2c_Addr i2c slave address
 * @param addr register address/es
 * @param data data to write to register
 * @param txBytes amount of bytes to transmit
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @retval None
 */
void MPU9250_WriteRegister(uint8_t i2c_Addr, uint8_t addr, uint8_t data, int8_t txBytes);

/**
 * @brief This function reads the accelerometer output register
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return None
 */
void MPU9250_ReadAccel(void);

/**
 * @brief This function reads the gyroscope output register 
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return None
 */
void MPU9250_ReadGyro(void);

/**
 * @brief This function reads the magnetoemeter output register
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return None
 */
void MPU9250_ReadMag(void);

/**
 * @brief This function calculates the average of the last 8 input values
 * @param currentIndex current index
 * @param pAvgBuffer store the last 8 values
 * @param inputValue new values
 * @param outputValue average values
 */
void MPU9250_CalcAvgValue(uint8_t *currentIndex, int16_t *pAvgBuffer, int16_t inputValue, int32_t *outputValue);

/**
 * @brief This function reads and formats the temperature measurements
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return float
 */
float MPU9250_ReadTemperature(void);

#endif // MPU9250V3_H_INCLUDED