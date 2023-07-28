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
 *              - measurement for accelerometer (x, y, z)
 *              - measurement for gyroscope (x, y, z)
TODO *              - calculation of pitch, roll and yaw angles
 */

#ifndef MPU9250_H_INCLUDED
#define MPU9250_H_INCLUDED

 /************************************************************************************************
 ------------------------------------------- INCLUDES -------------------------------------------
 ************************************************************************************************/

#include "main.h"
#include "stm32f1xx_hal.h"

 /************************************************************************************************
 ---------------------------------------- GLOBAL DEFINES ----------------------------------------
 ************************************************************************************************/

 // MPU9250 register addresses
#define MPU9250_WHOAMI 117 // WHO AM I register address
#define MPU9250_PWR_MGMT_1 107     // Power Management 1 register address  
#define MPU9250_PWR_MGMT_2 108     // Power Management 2 register address  
#define MPU9250_SMPLRT_DIV 25      // Sample Rate Divider register address
#define MPU9250_CONFIG 26          // configuration register address
#define MPU9250_INT_PIN_CFG 55     // INT Pin / Bypass Enable Configuration register address
#define MPU9250_INT_ENABLE 56           // interript enable register address
#define MPU9250_GYRO_CONFIG 27     // gyroscope configuration register address
#define MPU9250_ACCEL_CONFIG_1 28  // accelerometer configuration 1 register address
#define MPU9250_ACCEL_CONFIG_2 29  // accelerometer configuration 2 register address
#define MPU9250_USER_CTRL 106      // User Control register address

// MPU9250 output registers
#define MPU9250_ACCEL_XOUT_H 59    // Accelerometer measurements X-Axis High register address 
#define MPU9250_TEMP_OUT_H 65      // Temperature sensor measurements High register address 
#define MPU9250_GYRO_XOUT_H 67     // Gyroscope measurements X-Axis High register address 

// max sensitivity scale factors (SSF)
#define MPU9250_ACCEL_SENS 16384.0      // SSF of accelerometer
#define MPU9250_GYRO_SENS 131.0         // SSF of gyroscope

/************************************************************************************************
--------------------------------------- GLOBAL STRUCTURES ---------------------------------------
************************************************************************************************/

typedef enum Sensor_Status
{
    MPU9250_OK = 0,
    MPU9250_ERROR = 1
} Sensor_Status;

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

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

extern coordinates accel;   // accelerometer values
extern coordinates gyro;    // gyroscope values
extern float temp;          // temperature values

extern angles fusion;       // data after complementary filter has been applied     

extern float accelSens;     // sensitivity scale factor of accelerometer
extern float gyroSens;      // sensitivity scale factor of gyroscope

/************************************************************************************************
-------------------------------------- FUNCTION PROTOTYPES --------------------------------------
************************************************************************************************/

/**
 * @brief This funtion initializes the MPU9250
 * @param hspi pointer to a SPI_HandleTypeDef structure
 * @param dlpf dlpf bandwidth
 * @param gyroFS full scale range of gyroscope
 * @param accelFS full scale range of accelermeter
 * @return Sensor_Status
 */
Sensor_Status MPU9250_Init(SPI_HandleTypeDef *hspi, bandwidthDLPF dlpf, fullScale gyroFS, fullScale accelFS);

/**
 * @brief read register of MPU9250
 * @param hspi pointer to a SPI_HandleTypeDef structure
 * @param addr register address/es
 * @param data received data
 * @param rxBytes amount of reveived bytes
 */
void MPU9250_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t *data, int8_t rxBytes);

/**
 * @brief write to register of MPU9250
 * @param hspi pointer to a SPI_HandleTypeDef structure
 * @param addr register address/es
 * @param data data to write to register
 * @param txBytes amount of bytes to transmit
 */
void MPU9250_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t data, int8_t txBytes);

/**
 * @brief This function reads and formats [g] the accelerometer measurements
 * @param hspi pointer to a SPI_HandleTypeDef structure
 * @return coordinates
 */
coordinates MPU9250_ReadAccel(SPI_HandleTypeDef *hspi);

/**
 * @brief This function reads and formats [°/s] the gyroscope measurements
 * @param hspi pointer to a SPI_HandleTypeDef structure
 * @return coordinates
 */
coordinates MPU9250_ReadGyro(SPI_HandleTypeDef *hspi);

/**
 * @brief This function reads and formats the temperature measurements
 * @param hspi pointer to a SPI_HandleTypeDef structure
 * @return float
 */
float MPU9250_ReadTemperature(SPI_HandleTypeDef *hspi);

/**
 * @brief This function converts rawData to specific data 
 * @param rawData pointer to rawData
 * @return None
 */
void MPU9250_CalcValues(uint8_t *rawData);

#endif // MPU9250_H_INCLUDED