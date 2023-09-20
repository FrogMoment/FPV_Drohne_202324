/**
 * @file MPU9250.h
 * @author Maximilian Lendl
 * @date 2023-06-18
 * @version 1
 *
 * @copyright FPV Drohne DA 202324
 *
 * @brief This file provides functions for:
 *              - initialization of MPU9250 + read/write functions
 *              - measurement for accelerometer (x, y, z)
 *              - measurement for gyroscope (x, y, z)
 *              - calculation of pitch, roll and yaw angles
 */

#ifndef MPU9250_H_INCLUDED
#define MPU9250_H_INCLUDED

 /************************************************************************************************
 ------------------------------------------- INCLUDES -------------------------------------------
 ************************************************************************************************/

#include "main.h"
#include "stm32f1xx_hal.h"
#include <math.h>

 /************************************************************************************************
 ---------------------------------------- GLOBAL DEFINES ----------------------------------------
 ************************************************************************************************/

 // MPU9250 register addresses
#define I2C_SLV_ADDR 0x69       // i2c slave addr
#define WHOAMI_ADDR 117         // WHO AM I register address
#define PWR_MGMT_1_ADDR 107     // Power Management 1 register address  
#define PWR_MGMT_2_ADDR 108     // Power Management 2 register address  
#define SMPLRT_DIV_ADDR 25      // Sample Rate Divider register address
#define CONFIG_ADDR 26          // configuration register address
#define INT_PIN_CFG_ADDR 55     // INT Pin / Bypass Enable Configuration register address
#define INT_ENABLE_ADDR 56      // interrupt enable register address
#define GYRO_CONFIG_ADDR 27     // gyroscope configuration register address
#define ACCEL_CONFIG_1_ADDR 28  // accelerometer configuration 1 register address
#define ACCEL_CONFIG_2_ADDR 29  // accelerometer configuration 2 register address
#define USER_CTRL_ADDR 106      // User Control register address

// MPU9250 output registers
#define ACCEL_XOUT_H_ADDR 59    // Accelerometer measurements X-Axis High register address 
#define TEMP_OUT_H_ADDR 65      // Temperature sensor measurements High register address 
#define GYRO_XOUT_H_ADDR 67     // Gyroscope measurements X-Axis High register address 

// max sensitivity scale factors (SSF)
#define ACCEL_SENS 16384.0      // SSF of accelerometer
#define GYRO_SENS 131.0         // SSF of gyroscope

#define MPU9250_DT 0.5    // delta calculate time
#define RAD_TO_DEG 57.2957795131
#define COMP_GYRO_COEFF .95

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

extern uint8_t mpu9250_RawData[14]; // mpu9250 raw data

extern coordinates accel;   // accelerometer values
extern coordinates gyro;    // gyroscope values
extern float temp;          // temperature values

extern angles fusion;       // data after complementary filter has been applied     

extern float accelSens;     // sensitivity scale factor of accelerometer
extern float gyroSens;      // sensitivity scale factor of gyroscope

extern I2C_HandleTypeDef *mpu9250_InputI2C;

/************************************************************************************************
-------------------------------------- FUNCTION PROTOTYPES --------------------------------------
************************************************************************************************/

/**
 * @brief This funtion initializes the MPU9250
 * @param hi2c pointer to a I2C_HandleTypeDef structure
 * @param dlpf dlpf bandwidth
 * @param gyroFS full scale range of gyroscope
 * @param accelFS full scale range of accelermeter
 * @param htim pointer to TIM_HandleTypeDef structure (for us delay)
 * @return MPU9250_Status
 */
MPU9250_Status MPU9250_Init(I2C_HandleTypeDef *hi2c, bandwidthDLPF dlpf, fullScale gyroFS, fullScale accelFS, TIM_HandleTypeDef *htim);

/**
 * @brief This function starts DMA reading of accel, gyro and temp 
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return MPU9250_Status 
 */
MPU9250_Status MPU9250_StartReading(void);

/**
 * @brief read register of MPU9250
 * @param addr register address/es
 * @param data received data
 * @param rxBytes amount of reveived bytes
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @retval None
 */
void MPU9250_ReadRegister(uint8_t addr, uint8_t *data, int8_t rxBytes);

/**
 * @brief write to register of MPU9250
 * @param addr register address/es
 * @param data data to write to register
 * @param txBytes amount of bytes to transmit
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @retval None
 */
void MPU9250_WriteRegister(uint8_t addr, uint8_t data, int8_t txBytes);

/**
 * @brief This function reads and formats [g] the accelerometer measurements
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return coordinates
 */
coordinates MPU9250_ReadAccel(void);

/**
 * @brief This function reads and formats [°/s] the gyroscope measurements
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return coordinates
 */
coordinates MPU9250_ReadGyro(void);

/**
 * @brief This function reads and formats the temperature measurements
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return float
 */
float MPU9250_ReadTemperature(void);

/**
 * @brief This function converts the raw data of the MPU9250
 * @details
 * Byte[0-5]:  acclerometer H/L byte x,y,z values 
 * Byte[6-7]:  temperature H/L byte  
 * Byte[8-13]: gyroscope H/L byte x,y,z values 
 * The final result gets stored in the variables accel, temp and gyro
 * @attention This function uses the global variable mpu9250_RawData
 * @return None
 */
void MPU9250_CalcValues(void);

/**
 * @brief This function calculates pitch and roll with gyro and accel data
 * @return None
 */
void MPU9250_CompFilter(void);

#endif // MPU9250_H_INCLUDED