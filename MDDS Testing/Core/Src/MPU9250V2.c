/**
 * @file MPU9250V1.c
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

#include "MPU9250V2.h"

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/


uint8_t mpu9250_RawData[14] = {0}; // mpu9250 raw data

coordinates accel;   // accelerometer values
coordinates gyro;    // gyroscope values
float temp;          // temperature values

angles fusion;       // data after complementary filter has been applied     

float accelSens;     // sensitivity scale factor of accelerometer
float gyroSens;      // sensitivity scale factor of gyroscope

I2C_HandleTypeDef *mpu9250_InputI2C = NULL;
TIM_HandleTypeDef *MPU9250_DelayTimer = NULL;
uint32_t oldTime = 0;

/************************************************************************************************
------------------------------------------- FUNCTIONS -------------------------------------------
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
MPU9250_Status MPU9250_Init(I2C_HandleTypeDef *hi2c, bandwidthDLPF dlpf, fullScale gyroFS, fullScale accelFS, TIM_HandleTypeDef *htim)
{
    uint8_t data[3] = {0}, timeout = 0;
    mpu9250_InputI2C = hi2c;

    while(data[0] != 0x71)
    {
        MPU9250_ReadRegister(WHOAMI_ADDR, &data[0], 1);
        if(timeout++ > 100)
            return MPU9250_WHOAMI_ERROR;
    }

    MPU9250_WriteRegister(PWR_MGMT_1_ADDR, 0x00, 1);
    MPU9250_WriteRegister(PWR_MGMT_1_ADDR, 0x01, 1);
    MPU9250_WriteRegister(SMPLRT_DIV_ADDR, 0x00, 1);
    MPU9250_WriteRegister(CONFIG_ADDR, dlpf, 1);
    // MPU9250_WriteRegister(INT_PIN_CFG_ADDR, 0x02, 1);
    // MPU9250_WriteRegister(INT_ENABLE_ADDR, 0x01, 1);

    MPU9250_WriteRegister(ACCEL_CONFIG_1_ADDR, accelFS << 3, 1);
    // MPU9250_WriteRegister(GYRO_CONFIG_ADDR, gyroFS << 3, 1);

    accelSens = ACCEL_SENS / (1 << accelFS);    // calc sensitivity scale factor of accelerometer
    // gyroSens = GYRO_SENS / (1 << gyroFS);       // calc sensitivity scale factor of gyroscope

    // start timer for us delay (CompFilter)
    MPU9250_DelayTimer = htim;
    HAL_TIM_Base_Start(MPU9250_DelayTimer); 

    return MPU9250_OK;
}

/**
 * @brief This function starts DMA reading of accel, gyro and temp 
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return MPU9250_Status 
 */
MPU9250_Status MPU9250_StartReading(void)
{
    if(HAL_I2C_Mem_Read_DMA(mpu9250_InputI2C, I2C_SLV_ADDR << 1, ACCEL_XOUT_H_ADDR, I2C_MEMADD_SIZE_8BIT, mpu9250_RawData, 14) != HAL_OK)
        return MPU9250_I2C_ERROR;
    
    __HAL_TIM_SET_COUNTER(MPU9250_DelayTimer, 0); // set delay counter to 0

    return MPU9250_OK;
}

/**
 * @brief read register of MPU9250
 * @param addr register address/es
 * @param data received data
 * @param rxBytes amount of reveived bytes
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @retval None
 */
void MPU9250_ReadRegister(uint8_t addr, uint8_t *data, int8_t rxBytes)
{
    HAL_I2C_Mem_Read(mpu9250_InputI2C, I2C_SLV_ADDR << 1, addr, I2C_MEMADD_SIZE_8BIT, data, rxBytes, HAL_MAX_DELAY);
}

/**
 * @brief write to register of MPU9250
 * @param addr register address/es
 * @param data data to write to register
 * @param txBytes amount of bytes to transmit
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @retval None
 */
void MPU9250_WriteRegister(uint8_t addr, uint8_t data, int8_t txBytes)
{
    HAL_I2C_Mem_Write(mpu9250_InputI2C, I2C_SLV_ADDR << 1, addr, I2C_MEMADD_SIZE_8BIT, &data, txBytes, HAL_MAX_DELAY);
}

/**
 * @brief This function reads and formats [g] the accelerometer measurements
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return coordinates
 */
coordinates MPU9250_ReadAccel(void)
{
    coordinates accelData;
    uint8_t measurements[6];

    MPU9250_ReadRegister(ACCEL_XOUT_H_ADDR, measurements, 6);     // read the accel output registers

    // format the measurements to g's
    accelData.x = ((measurements[0] << 8) | measurements[1]) / accelSens;
    accelData.y = ((measurements[2] << 8) | measurements[3]) / accelSens;
    accelData.z = ((measurements[4] << 8) | measurements[5]) / accelSens;

    return accelData;
}

/**
 * @brief This function reads and formats [°/s] the gyroscope measurements
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return coordinates
 */
coordinates MPU9250_ReadGyro(void)
{
    coordinates gyroData;
    uint8_t measurements[6];

    MPU9250_ReadRegister(GYRO_XOUT_H_ADDR, measurements, 6);      // read the accel output registers

    // format the measurements to °/s
    gyroData.x = ((measurements[0] << 8) | measurements[1]) / gyroSens;
    gyroData.y = ((measurements[2] << 8) | measurements[3]) / gyroSens;
    gyroData.z = ((measurements[4] << 8) | measurements[5]) / gyroSens;

    return gyroData;
}

/**
 * @brief This function reads and formats the temperature measurements
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return float
 */
float MPU9250_ReadTemperature(void)
{
    uint8_t measurements[2];

    MPU9250_ReadRegister(TEMP_OUT_H_ADDR, measurements, 2);

    return (((measurements[0] << 8) | measurements[1]) / 333.87) + 21.0;
}

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
void MPU9250_CalcValues(void)
{
    coordinates tmp;

    // accelerometer measurements
    tmp.x = (((int8_t)mpu9250_RawData[0] << 8) | mpu9250_RawData[1]) / accelSens;
    tmp.y = (((int8_t)mpu9250_RawData[2] << 8) | mpu9250_RawData[3]) / accelSens;
    tmp.z = (((int8_t)mpu9250_RawData[4] << 8) | mpu9250_RawData[5]) / accelSens;
    accel = tmp;

    // temperature measurements
    temp = ((((int8_t)mpu9250_RawData[6] << 8) | mpu9250_RawData[7]) / 333.87) + 21.0;

    // gyroscope measurements 
    tmp.x = (((int8_t)mpu9250_RawData[8]  << 8) | mpu9250_RawData[9])  / gyroSens;
    tmp.y = (((int8_t)mpu9250_RawData[10] << 8) | mpu9250_RawData[11]) / gyroSens;
    tmp.z = (((int8_t)mpu9250_RawData[12] << 8) | mpu9250_RawData[13]) / gyroSens;
    gyro = tmp;
}

/**
 * @brief This function calculates pitch and roll with gyro and accel data
 * @attention This funciton resets the counter of the configured delay timer
 * @return None
 */
void MPU9250_CompFilter(void)
{
    // calculate time delay
    float deltaTime = __HAL_TIM_GET_COUNTER(MPU9250_DelayTimer) * 1E-6;
    if(deltaTime <= 0) deltaTime *= -1;

    // check if z axis is inverted
    float accel_z = (accel.z < 0) ? -accel.z : accel.z;

    // calculate pitch and roll angles
    angles tmp;
    if(accel_z != 0)
    {
        tmp.pitch = RAD_TO_DEG * atan2(accel.x, accel_z);
        tmp.roll = RAD_TO_DEG * atan2(accel.y, accel_z);
    }

    fusion.pitch = COMP_GYRO_COEFF * (fusion.pitch + gyro.y * deltaTime) + (1 - COMP_GYRO_COEFF) * tmp.pitch;
    fusion.roll = COMP_GYRO_COEFF * (fusion.roll + gyro.x * deltaTime) + (1 - COMP_GYRO_COEFF) * tmp.roll;

    __HAL_TIM_SET_COUNTER(MPU9250_DelayTimer, 0); // rest timer counter
}

