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

I2C_HandleTypeDef *imu_ComI2C;

/**
 * @brief This function initialzes the 10DOF IMU
 * @param hi2c 
 * @return IMU_Status 
 */
IMU_Status IMU_Init(I2C_HandleTypeDef *hi2c)
{
    imu_ComI2C = hi2c;
    if(imu_ComI2C == NULL)
        return IMU_ERROR;

    // check if connected via WHO AM I register value (should be 0x6C)
    uint8_t data, timeout = 0;
    while(data != 0x6C)
    {
        IMU_ReadRegister(IMU_WHOAMI_ADDR, &data, 1);
        if(timeout++ > 100)
            return IMU_WHOAMI_ERROR;
    }

    return IMU_OK;
}

/**
 * @brief This function read an amount of bytes from a specific register start
 * @param addr register address
 * @param data data pointer 
 * @param rxBytes amount of bytes received
 */
void IMU_ReadRegister(uint8_t addr, uint8_t *data, uint8_t rxBytes)
{
    HAL_I2C_Mem_Read(imu_ComI2C, IMU_I2C_SLV_ADDR << 1, addr, I2C_MEMADD_SIZE_8BIT, data, rxBytes, HAL_MAX_DELAY);
}

/**
 * @brief This function writes an amount of bytes from a specific register start
 * @param addr register address
 * @param data data pointer 
 * @param txBytes amount of bytes written
 */
void IMU_WriteRegister(uint8_t addr, uint8_t *data, uint8_t txBytes)
{
    HAL_I2C_Mem_Write(imu_ComI2C, IMU_I2C_SLV_ADDR << 1, addr, I2C_MEMADD_SIZE_8BIT, data, txBytes, HAL_MAX_DELAY);
}

