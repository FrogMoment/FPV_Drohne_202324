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
#define IMU_MPU_I2C_ADDR 0xD0
#define IMU_MPU_WHOAMI_ADDR 0x75
#define IMU_MPU_BYPASS_ADDR 0x37

// magnetometer addresses
#define IMU_MAG_I2C_ADDR 0x18
#define IMU_MAG_WHOAMI_ADDR 0x00

// barometer addresses
#define IMU_BARO_I2C_ADDR 0xEE
#define IMU_BARO_CHIPID_ADDR 0xD0 

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

/**********************************************************************************
------------------------------- FUNCTION PROTOTYPES -------------------------------
**********************************************************************************/

/**
 * @brief This function initialzes the 10DOF IMU
 * @param hi2c 
 * @return IMU_Status 
 */
IMU_Status IMU_Init(I2C_HandleTypeDef *hi2c);

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









