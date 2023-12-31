/**
 * @file IMU_10DOF.h
 * @author Maximilian Lendl
 * @date 2023-11-18
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief 
 * 
 */

/**********************************************************************************
------------------------------------ INCLUDES ------------------------------------
**********************************************************************************/

#include "main.h"
#include <string.h>
#include <stdio.h>

/**********************************************************************************
--------------------------------- GLOBAL DEFINES ---------------------------------
**********************************************************************************/

// MPU9255 addresses
#define IMU_MPU_I2C_ADDR            0xD0 // MPU9250 I2C slave address
#define IMU_MPU_WHOAMI_ADDR         0x75 // MPU9250 WHO AM I register address
#define IMU_MPU_INT_PIN_CFG_ADDR    0x37 // MPU9250 bypass enable register address
#define IMU_MPU_PWR_MGMT_1_ADDR     0x6B // MPU9250 power management 1 register address
#define IMU_MPU_SMPLRT_DIV_ADDR     0x19 // MPU9250 sample rate divider register address
#define IMU_MPU_CONFIG_ADDR         0x1A // MPU9250 configuration register address
#define IMU_MPU_GYRO_CONFIG_ADDR    0x1B // MPU9250 gyroscope configuration register address
#define IMU_MPU_ACCEL_CONFIG_ADDR   0x1C // MPU9250 accelerometer configuration 1 register address

// magnetometer addresses
#define IMU_MAG_I2C_ADDR    0x18 // magnetometer I2C slave address
#define IMU_MAG_WHOAMI_ADDR 0x00 // magnetometer WHO AM I register address

// barometer addresses
#define IMU_BARO_I2C_ADDR       0xEE // barometer I2C slave address
#define IMU_BARO_CHIPID_ADDR    0xD0 // barometer CHIP ID register address

/**********************************************************************************
-------------------------------- GLOBAL STRUCTURES --------------------------------
**********************************************************************************/

// IMU OK and error codes
typedef enum IMU_Status
{
    IMU_OK = 0,
    IMU_ADDRESS_ERROR = 1,
    IMU_TIMER_ERROR = 2,
    
    IMU_MPU_WHOAMI_ERROR = 10,
    IMU_MAG_WHOAMI_ERROR = 11,
    IMU_BARO_CHIPID_ERROR = 12
} IMU_Status;

// all IMU sensors
typedef enum IMU_Sensor
{
    MPU9250 = 0,
    // MPU = 1,
    AK8963 = 2,
    MAG = 3,
    BMP280 = 4,
    BARO = 5
} IMU_Sensor;

// full scale range values
typedef enum IMU_Fullscale
{
    GYRO_250DPS = 0,
    GYRO_500DPS = 1,
    GYRO_1000DPS = 2,
    GYRO_2000DPS = 3,

    ACCEL_2G = 0,
    ACCEL_4G = 1,
    ACCEL_8G = 2,
    ACCEL_16G = 3
} IMU_Fullscale;

/**********************************************************************************
------------------------------- FUNCTION PROTOTYPES -------------------------------
**********************************************************************************/

/**
 * @brief This function initialzes the 10DOF IMU
 * @param hi2c pointer to I2C_HandleTypeDef
 * @param gyroFS full scale select for gyroscope (GYROGYRO_250DPS / GYRO_500DPS / GYRO_1000DPS / GYRO_2000DPS)
 * @param accelFS full scale selct for accelerometer (ACCEL_2G / ACCEL_4G / ACCEL_8G / ACCEL_16G)
 * @return IMU_Status 
 */
IMU_Status IMU_Init(I2C_HandleTypeDef *hi2c, IMU_Fullscale gyroFS, IMU_Fullscale accelFS);

/**
 * @brief This function reads an amount of bytes from registers from the IMU
 * @param sensor MPU9250, AK8963 (MAG), BMP280 (BARO)
 * @param regAddr register address
 * @param data data pointer
 * @param rxBytes amount of bytes to read
 * @return IMU_Status 
 */
IMU_Status IMU_ReadRegister(IMU_Sensor sensor, uint8_t regAddr, uint8_t *data, uint8_t rxBytes);

/**
 * @brief This function writes an amount of bytes to registers from the IMU
 * @param sensor MPU9250, AK8963 (MAG), BMP280 (BARO)
 * @param regAddr register address
 * @param data data to write
 * @param txBytes amount of bytes to write
 * @return IMU_Status 
 */
IMU_Status IMU_WriteRegister(IMU_Sensor sensor, uint8_t regAddr, uint8_t data, uint8_t txBytes);

/**
 * @brief This function checks the connection of all sensors on the IMU
 * @attention This function enables the bypass mode in the MPU9250
 * @return IMU_Status 
 */
IMU_Status IMU_CheckConnection(void);









