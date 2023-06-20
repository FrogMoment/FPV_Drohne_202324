/**
 * @file MPU9250.h
 * @author Maximilian Lendl
 * @date 2023-06-18
 * @version 1
 * 
 * @copyright Speed Junkies DA 202324
 * 
 * @brief This file provides functions for: 
PENDING *              - initialization of MPU9250 + read/write functions
TODO *              - measurement for accelerometer (x, y, z)
TODO *              - measurement for gyroscope (x, y, z)
TODO *              - measurement for magnetometer (x, y, z)
TODO *              - calculation of pitch, roll and yaw angles
 */

#ifndef MPU9250_H_INCLUDED
#define MPU9250_H_INCLUDED

 /*********************************************************
 ------------------------ INCLUDES ------------------------
 *********************************************************/

#include "main.h"
#include "stm32f1xx_hal.h"

 /*********************************************************
 --------------------- GLOBAL DEFINES ---------------------
 *********************************************************/

// MPU9250 register addresses
#define MPU9250_WHOAMI_ADDR 117  // WHO AM I register address

 /*********************************************************
 ------------------- GLOBAL STRUCTURES --------------------
 *********************************************************/

typedef enum MPU_status
{
    MPU_OK = 1,
    MPU_ERROR = 0
} MPU_status;

// full scale ranges for gyroscope and accelerometer
typedef enum fullScale
{
    GYRO_250DPS = 0,    // gyroscope full scale range +250°/s
    GYRO_500DPS = 1,    // gyroscope full scale range +500°/s
    GYRO_1000DPS = 2,   // gyroscope full scale range +1000°/s
    GYRO_2000DPS = 3,	// gyroscope full scale range +2000°/s

    ACCEL_2G = 0,       // accelerometer full scale range +-2g
    ACCEL_4G = 1,       // accelerometer full scale range +-4g
    ACCEL_8G = 2,       // accelerometer full scale range +-8g
    ACCEL_16G = 3       // accelerometer full scale range +-16g
} fullScale;

// x, y, z coordinates
typedef struct coordinates
{
    float x, y, z;
} coordinates;

// orientation angels
typedef struct angles
{
    float pitch, roll, yaw;
} angles;

// digital lowpass filter configuration (DLPF)
typedef enum bandwidthDLPF
{
    DLPF_250Hz = 0,
    DLPF_184Hz = 1,
    DLPF_92Hz = 2,
    DLPF_41Hz = 3,
    DLPF_20Hz = 4,
    DLPF_10Hz = 5,
    DLPF_5Hz = 6,
    DLPF_3600Hz = 7
} bandwidthDLPF;

/*********************************************************
------------------ FUNCTION PROTOTYPES -------------------
*********************************************************/

/**
 * @brief This funtion initializes the MPU9250
 * @param hspi1 pointer to a SPI_HandleTypeDef structure
 */
void MPU9250_Init(SPI_HandleTypeDef *hspi1);

/**
 * @brief read register of MPU9250
 * @param hspi1 pointer to a SPI_HandleTypeDef structure
 * @param addr register address/es
 * @param data received data
 * @param rxBytes amount of reveived bytes
 */
void MPU9250_ReadRegister(SPI_HandleTypeDef *hspi1, uint8_t addr, uint8_t *data, uint8_t rxBytes);

/**
 * @brief check WHO AM I register value
 * @param hspi1 pointer to a SPI_HandleTypeDef structure
 * @return MPU_status
 */
MPU_status MPU9250_ReadWhoAmI(SPI_HandleTypeDef *hspi1);

#endif // MPU9250_H_INCLUDED