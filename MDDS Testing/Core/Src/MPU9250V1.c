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

#include "MPU9250V1.h"

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
 * @param hspi pointer to a SPI_HandleTypeDef structure
 * @param dlpf dlpf bandwidth
 * @param gyroFS full scale range of gyroscope
 * @param accelFS full scale range of accelermeter
 * @return MPU_status
 */
Sensor_Status MPU9250_Init(SPI_HandleTypeDef *hspi, bandwidthDLPF dlpf, fullScale gyroFS, fullScale accelFS)
{
    uint8_t data[6] = {0}, timeout = 0;

    HAL_GPIO_WritePin(MPU9250_CS_GPIO_Port, MPU9250_CS_Pin, GPIO_PIN_RESET); // set CS_MPU active

    // check if MPU is connected
    while(data[0] != 0x71)
    {
        MPU9250_ReadRegister(hspi, MPU9250_WHOAMI, &data[0], 1); // read data from WHOAMI register
        if(timeout++ > 100)
            return MPU9250_ERROR;
    }

    // // MPU9250 configuration
    // // reset MPU9250
    // MPU9250_WriteRegister(hspi, PWR_MGMT_1_ADDR, 0x80, 1); 
    // HAL_Delay(100);
    // // MPU9250 set clock source
    // MPU9250_WriteRegister(hspi, PWR_MGMT_1_ADDR, 0x03, 1);
    // HAL_Delay(1);
    // // MPU9250 set interrupt
    // MPU9250_WriteRegister(hspi, INT_PIN_CFG_ADDR, 0x10, 1);
    // HAL_Delay(1);
    // MPU9250_WriteRegister(hspi, INT_ENABLE, ENABLE, 1);
    // HAL_Delay(1);
    // // MPU9250 set sensors
    // MPU9250_WriteRegister(hspi, PWR_MGMT_2_ADDR, 0xC0, 1);
    // HAL_Delay(1);
    // // MPU9250 set sample rate
    // MPU9250_WriteRegister(hspi, SMPLRT_DIV_ADDR, 0x00, 1);
    // HAL_Delay(1);
    // // set full scale range
    // MPU9250_WriteRegister(hspi, GYRO_CONFIG_ADDR, gyroFS << 3, 1);
    // HAL_Delay(1);
    // MPU9250_WriteRegister(hspi, ACCEL_CONFIG_1_ADDR, accelFS << 3, 1);
    // HAL_Delay(1);
    // // set accel dlpf
    // MPU9250_ReadRegister(hspi, ACCEL_CONFIG_2_ADDR, &data[0], 1);
    // HAL_Delay(1);
    // MPU9250_WriteRegister(hspi, ACCEL_CONFIG_2_ADDR, data[0] | dlpf, 1);
    // HAL_Delay(1);
    // // set gyro dlpf
    // MPU9250_WriteRegister(hspi, CONFIG_ADDR, dlpf, 1);
    // HAL_Delay(1);
    // // set SPI mode
    // MPU9250_ReadRegister(hspi, USER_CTRL_ADDR, &data[0], 1);
    // HAL_Delay(1);
    // MPU9250_WriteRegister(hspi, USER_CTRL_ADDR, data[0] | 0x10, 1);
    // HAL_Delay(1);
    // MPU9250_ReadRegister(hspi, USER_CTRL_ADDR, &data[0], 1);
    // HAL_Delay(1);
    // MPU9250_WriteRegister(hspi, USER_CTRL_ADDR, data[0] | 0x20, 1);
    // HAL_Delay(1);

    HAL_GPIO_WritePin(MPU9250_CS_GPIO_Port, MPU9250_CS_Pin, GPIO_PIN_SET);  // set CS_MPU inactive

    accelSens = MPU9250_ACCEL_SENS / (1 << accelFS);    // calc sensitivity scale factor of accelerometer
    gyroSens = MPU9250_GYRO_SENS / (1 << gyroFS);       // calc sensitivity scale factor of gyroscope

    return MPU9250_OK;
}

/**
 * @brief read register of MPU9250
 * @param hspi pointer to a SPI_HandleTypeDef structure
 * @param addr register address/es
 * @param data received data
 * @param rxBytes amount of reveived bytes
 */
void MPU9250_ReadRegister(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t *data, int8_t rxBytes)
{
    // check if the CS port is active
    // int8_t toggle = 0;
    // if(HAL_GPIO_ReadPin(MPU9250_CS_GPIO_Port, MPU9250_CS_Pin) == GPIO_PIN_SET)
    // {
    //     HAL_GPIO_WritePin(MPU9250_CS_GPIO_Port, MPU9250_CS_Pin, GPIO_PIN_RESET);
    //     toggle = 1;
    // }

    uint8_t fullAddr = 0x80 | addr; // add read bit to MSB of address

    HAL_SPI_Transmit(hspi, &fullAddr, 1, HAL_MAX_DELAY);   // transmit register address
    HAL_SPI_Receive(hspi, data, rxBytes, HAL_MAX_DELAY);   // receive register data

    // if(toggle)
    //     HAL_GPIO_WritePin(MPU9250_CS_GPIO_Port, MPU9250_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief write to register of MPU9250
 * @param hspi pointer to a SPI_HandleTypeDef structure
 * @param addr register address/es
 * @param data data to write to register
 * @param txBytes amount of bytes to transmit
 */
void MPU9250_WriteRegister(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t data, int8_t txBytes)
{
    // check if the CS port is active
    int8_t toggle = 0;
    if(HAL_GPIO_ReadPin(MPU9250_CS_GPIO_Port, MPU9250_CS_Pin) == GPIO_PIN_SET)
    {
        HAL_GPIO_WritePin(MPU9250_CS_GPIO_Port, MPU9250_CS_Pin, GPIO_PIN_RESET);
        toggle = 1;
    }

    uint8_t fullAddr = 0x7F & addr; // add write bit to MSB of address

    HAL_SPI_Transmit(hspi, &fullAddr, 1, HAL_MAX_DELAY);    // transmit register address
    HAL_SPI_Transmit(hspi, &data, txBytes, HAL_MAX_DELAY);  // transmit data to register

    if(toggle)
        HAL_GPIO_WritePin(MPU9250_CS_GPIO_Port, MPU9250_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief This function reads and formats [g] the accelerometer measurements
 * @param hspi pointer to a SPI_HandleTypeDef structure
 * @return coordinates
 */
coordinates MPU9250_ReadAccel(SPI_HandleTypeDef *hspi)
{
    coordinates accelData;
    uint8_t measurements[6];

    MPU9250_ReadRegister(hspi, MPU9250_ACCEL_XOUT_H, measurements, 6);     // read the accel output registers

    // format the measurements to g's
    accelData.x = (((int8_t)measurements[0] << 8) | (int8_t)measurements[1]) / accelSens;
    accelData.y = (((int8_t)measurements[2] << 8) | (int8_t)measurements[3]) / accelSens;
    accelData.z = (((int8_t)measurements[4] << 8) | (int8_t)measurements[5]) / accelSens;

    return accelData;
}

/**
 * @brief This function reads and formats [°/s] the gyroscope measurements
 * @param hspi pointer to a SPI_HandleTypeDef structure
 * @return coordinates
 */
coordinates MPU9250_ReadGyro(SPI_HandleTypeDef *hspi)
{
    coordinates gyroData;
    uint8_t measurements[6];

    MPU9250_ReadRegister(hspi, MPU9250_GYRO_XOUT_H, measurements, 6);      // read the accel output registers

    // format the measurements to °/s
    gyroData.x = ((measurements[0] << 8) | measurements[1]) / gyroSens;
    gyroData.y = ((measurements[2] << 8) | measurements[3]) / gyroSens;
    gyroData.z = ((measurements[4] << 8) | measurements[5]) / gyroSens;

    return gyroData;
}

/**
 * @brief This function reads and formats the temperature measurements
 * @param hspi pointer to a SPI_HandleTypeDef structure
 * @return float
 */
float MPU9250_ReadTemperature(SPI_HandleTypeDef *hspi)
{
    uint8_t measurements[2];

    MPU9250_ReadRegister(hspi, MPU9250_TEMP_OUT_H, measurements, 2);

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
    uint8_t tmp_RawData[14];

    for(uint8_t i = 0; i < 14; i++)
        tmp_RawData[i] = rawData[i];

    // accelerometer measurements
    tmp.x = (((int8_t)tmp_RawData[0] << 8) | (int8_t)tmp_RawData[1]) / accelSens;
    tmp.y = (((int8_t)tmp_RawData[2] << 8) | (int8_t)tmp_RawData[3]) / accelSens;
    tmp.z = (((int8_t)tmp_RawData[4] << 8) | (int8_t)tmp_RawData[5]) / accelSens;
    accel = tmp;

    // temperature measurements
    temp = ((((int8_t)tmp_RawData[6] << 8) | (int8_t)tmp_RawData[7]) / 333.87) + 21.0;

    // gyroscope measurements 
    tmp.x = (((int8_t)tmp_RawData[8] << 8) | (int8_t)tmp_RawData[9]) / gyroSens;
    tmp.y = (((int8_t)tmp_RawData[10] << 8) | (int8_t)tmp_RawData[11]) / gyroSens;
    tmp.z = (((int8_t)tmp_RawData[12] << 8) | (int8_t)tmp_RawData[13]) / gyroSens;
    gyro = tmp;
}

