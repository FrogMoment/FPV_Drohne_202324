/**
 * @file MPU9250V1.c
 * @author Maximilian Lendl
 * @date 2023-06-18
 * @version 1
 *
 * @copyright Speed Junkies DA 202324
 *
 * @brief This file provides functions for:
 *              - initialization of MPU9250 + read/write functions
 *              - measurement for accelerometer (x, y, z)
 *              - measurement for gyroscope (x, y, z)
 *              - calculation of pitch, roll and yaw angles
 */

#include "MPU9250V2.h"

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

coordinates accel;   // accelerometer values
coordinates gyro;    // gyroscope values
float temp;          // temperature values

angles fusion;       // data after complementary filter has been applied     

float accelSens;     // sensitivity scale factor of accelerometer
float gyroSens;      // sensitivity scale factor of gyroscope

/************************************************************************************************
------------------------------------------- FUNCTIONS -------------------------------------------
************************************************************************************************/

/**
 * @brief This funtion initializes the MPU9250
 * @param hi2c pointer to a I2C_HandleTypeDef structure
 * @param dlpf dlpf bandwidth
 * @param gyroFS full scale range of gyroscope
 * @param accelFS full scale range of accelermeter
 * @return MPU_status
 */
Sensor_Status MPU9250_Init(I2C_HandleTypeDef *hi2c, bandwidthDLPF dlpf, fullScale gyroFS, fullScale accelFS)
{
    uint8_t data[3] = {0}, timeout = 0;

    while(data[0] != 0x71)
    {
        MPU9250_ReadRegister(hi2c, WHOAMI_ADDR, &data[0], 1);
        if(timeout++ > 100)
            return MPU_ERROR;
    }

    MPU9250_WriteRegister(hi2c, PWR_MGMT_1_ADDR, 0x00, 1);
    MPU9250_WriteRegister(hi2c, PWR_MGMT_1_ADDR, 0x01, 1);
    MPU9250_WriteRegister(hi2c, SMPLRT_DIV_ADDR, 0x10, 1);
    MPU9250_WriteRegister(hi2c, CONFIG_ADDR, dlpf, 1);
    MPU9250_WriteRegister(hi2c, INT_PIN_CFG_ADDR, 0x02, 1);
    MPU9250_WriteRegister(hi2c, INT_ENABLE_ADDR, 0x01, 1);

    MPU9250_WriteRegister(hi2c, ACCEL_CONFIG_1_ADDR, accelFS << 3, 1);
    MPU9250_WriteRegister(hi2c, GYRO_CONFIG_ADDR, gyroFS << 3, 1);

    accelSens = ACCEL_SENS / (1 << accelFS);    // calc sensitivity scale factor of accelerometer
    gyroSens = GYRO_SENS / (1 << gyroFS);       // calc sensitivity scale factor of gyroscope

    return SENSOR_OK;
}

/**
 * @brief read register of MPU9250
 * @param hi2c pointer to a I2C_HandleTypeDef structure
 * @param addr register address/es
 * @param data received data
 * @param rxBytes amount of reveived bytes
 */
void MPU9250_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t *data, int8_t rxBytes)
{
    HAL_I2C_Mem_Read(hi2c, I2C_SLV_ADDR << 1, addr, I2C_MEMADD_SIZE_8BIT, data, rxBytes, HAL_MAX_DELAY);
}

/**
 * @brief write to register of MPU9250
 * @param hi2c pointer to a I2C_HandleTypeDef structure
 * @param addr register address/es
 * @param data data to write to register
 * @param txBytes amount of bytes to transmit
 */
void MPU9250_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t data, int8_t txBytes)
{
    HAL_I2C_Mem_Write(hi2c, I2C_SLV_ADDR << 1, addr, I2C_MEMADD_SIZE_8BIT, &data, txBytes, HAL_MAX_DELAY);
}

/**
 * @brief This function reads and formats [g] the accelerometer measurements
 * @param hi2c pointer to a I2C_HandleTypeDef structure
 * @return coordinates
 */
coordinates MPU9250_ReadAccel(I2C_HandleTypeDef *hi2c)
{
    coordinates accelData;
    uint8_t measurements[6];

    MPU9250_ReadRegister(hi2c, ACCEL_XOUT_H_ADDR, measurements, 6);     // read the accel output registers

    // format the measurements to g's
    accelData.x = (((int8_t)measurements[0] << 8) | (int8_t)measurements[1]) / accelSens;
    accelData.y = (((int8_t)measurements[2] << 8) | (int8_t)measurements[3]) / accelSens;
    accelData.z = (((int8_t)measurements[4] << 8) | (int8_t)measurements[5]) / accelSens;

    return accelData;
}

/**
 * @brief This function reads and formats [°/s] the gyroscope measurements
 * @param hi2c pointer to a I2C_HandleTypeDef structure
 * @return coordinates
 */
coordinates MPU9250_ReadGyro(I2C_HandleTypeDef *hi2c)
{
    coordinates gyroData;
    uint8_t measurements[6];

    MPU9250_ReadRegister(hi2c, GYRO_XOUT_H_ADDR, measurements, 6);      // read the accel output registers

    // format the measurements to °/s
    gyroData.x = ((measurements[0] << 8) | measurements[1]) / gyroSens;
    gyroData.y = ((measurements[2] << 8) | measurements[3]) / gyroSens;
    gyroData.z = ((measurements[4] << 8) | measurements[5]) / gyroSens;

    return gyroData;
}

/**
 * @brief This function reads and formats the temperature measurements
 * @param hi2c pointer to a I2C_HandleTypeDef structure
 * @return float
 */
float MPU9250_ReadTemperature(I2C_HandleTypeDef *hi2c)
{
    uint8_t measurements[2];

    MPU9250_ReadRegister(hi2c, TEMP_OUT_H_ADDR, measurements, 2);

    return (((measurements[0] << 8) | measurements[1]) / 333.87) + 21.0;
}

/**
 * @brief This function converts rawData to specific data 
 * @param rawData pointer to rawData
 * @return None
 */
void MPU9250_CalcValues(uint8_t *rawData)
{
    coordinates tmp;

    // accelerometer measurements
    tmp.x = (((int8_t)rawData[0] << 8) | (int8_t)rawData[1]) / accelSens;
    tmp.y = (((int8_t)rawData[2] << 8) | (int8_t)rawData[3]) / accelSens;
    tmp.z = (((int8_t)rawData[4] << 8) | (int8_t)rawData[5]) / accelSens;
    accel = tmp;

    // temperature measurements
    temp = ((((int8_t)rawData[6] << 8) | (int8_t)rawData[7]) / 333.87) + 21.0;

    // gyroscope measurements 
    tmp.x = (((int8_t)rawData[8] << 8) | (int8_t)rawData[9]) / gyroSens;
    tmp.y = (((int8_t)rawData[10] << 8) | (int8_t)rawData[11]) / gyroSens;
    tmp.z = (((int8_t)rawData[12] << 8) | (int8_t)rawData[13]) / gyroSens;
    gyro = tmp;
}

