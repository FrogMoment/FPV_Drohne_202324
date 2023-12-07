/**
 * @file IMU_10DOF.c
 * @author Maximilian Lendl
 * @date 2023-11-18
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief 
 * 
 */

#include "IMU_10DOF.h"

/**********************************************************************
--------------------------- GLOBAL VARIABLE ---------------------------
**********************************************************************/

I2C_HandleTypeDef *imu_ComI2C;

/**********************************************************************
------------------------- FUNCTION PROTOTYPES -------------------------
**********************************************************************/

/**
 * @brief This function initialzes the 10DOF IMU
 * @param hi2c 
 * @return IMU_Status 
 */
IMU_Status IMU_Init(I2C_HandleTypeDef *hi2c)
{
    imu_ComI2C = hi2c;
    if(imu_ComI2C == NULL)
        return IMU_TIMER_ERROR;

    uint8_t errorCode;

    errorCode = IMU_CheckConnection();
    if(errorCode != IMU_OK)
        return errorCode;

    return IMU_OK;
}

/**
 * @brief This function reads an amount of bytes from registers from the IMU
 * @param sensor MPU9250, AK8963 (MAG), BMP280 (BARO)
 * @param regAddr register address
 * @param data data pointer
 * @param rxBytes amount of bytes to read
 * @return IMU_Status 
 */
IMU_Status IMU_ReadRegister(IMU_Sensor sensor, uint8_t regAddr, uint8_t *data, uint8_t rxBytes)
{
    // determine the I2C device address
    uint16_t devAddress;
    switch(sensor)
    {
        case MPU9250:
            devAddress = IMU_MPU_I2C_ADDR;
            break;

        case AK8963:
        case MAG:
            devAddress = IMU_MAG_I2C_ADDR;
            break;
        
        case BMP280:
        case BARO:
            devAddress = IMU_BARO_I2C_ADDR;
            break;

        default:
            return IMU_ADDRESS_ERROR;
    }

    // read register(s)
    HAL_I2C_Mem_Read(imu_ComI2C, devAddress, regAddr, I2C_MEMADD_SIZE_8BIT, data, rxBytes, HAL_MAX_DELAY);

    return IMU_OK;
}

/**
 * @brief This function writes an amount of bytes to registers from the IMU
 * @param sensor MPU9250, AK8963 (MAG), BMP280 (BARO)
 * @param regAddr register address
 * @param data data to write
 * @param txBytes amount of bytes to write
 * @return IMU_Status 
 */
IMU_Status IMU_WriteRegister(IMU_Sensor sensor, uint8_t regAddr, uint8_t data, uint8_t txBytes)
{
    // determine the I2C device address
    uint16_t devAddress;
    switch(sensor)
    {
        case MPU9250:
            devAddress = IMU_MPU_I2C_ADDR;
            break;

        case AK8963:
        case MAG:
            devAddress = IMU_MAG_I2C_ADDR;
            break;
        
        case BMP280:
        case BARO:
            devAddress = IMU_MPU_I2C_ADDR;
            break;

        default:
            return IMU_ADDRESS_ERROR;
    }

    // read register(s)
    HAL_I2C_Mem_Write(imu_ComI2C, devAddress, regAddr, I2C_MEMADD_SIZE_8BIT, &data, txBytes, HAL_MAX_DELAY);

    return IMU_OK;
}

/**
 * @brief This function checks the connection of all sensors on the IMU
 * @attention This function enables the bypass mode in the MPU9250
 * @return IMU_Status 
 */
IMU_Status IMU_CheckConnection(void)
{
    uint8_t regVal[3] = {0x71, 0x48, 0x58};
    uint8_t regAddr[3] = {IMU_MPU_WHOAMI_ADDR, IMU_MAG_WHOAMI_ADDR, IMU_BARO_CHIPID_ADDR};
    uint8_t sensor[3] = {MPU9250, MAG, BARO};
    uint8_t timeout;
    uint8_t data;

    for(uint8_t i = 0; i < 3; i++)
    {
        timeout = 0;

        while(data != regVal[i])
        {
            if(IMU_ReadRegister(sensor[i], regAddr[i], &data, 1) != IMU_OK)
                return IMU_ADDRESS_ERROR;

            if(timeout++ > 100)
                return IMU_MPU_WHOAMI_ERROR + i;
        }

        if(sensor[i] == MPU9250)
        {
            // enable bypass mode
            if(IMU_WriteRegister(MPU9250, IMU_MPU_BYPASS_ADDR, 0x02, 1))
                return IMU_ADDRESS_ERROR;
            HAL_Delay(10);
        }
    }

    return IMU_OK;
}









