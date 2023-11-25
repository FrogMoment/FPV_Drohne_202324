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

#include "main.h"

#define IMU_I2C_SLV_ADDR 0b1101010
#define IMU_WHOAMI_ADDR 0x0F

typedef enum IMU_Status
{
    IMU_OK = 0,
    IMU_ERROR = 1,
    IMU_WHOAMI_ERROR = 2
} IMU_Status;

/**
 * @brief This function initialzes the 10DOF IMU
 * @param hi2c 
 * @return IMU_Status 
 */
IMU_Status IMU_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief This function read an amount of bytes from a specific register start
 * @param addr register address
 * @param data data pointer 
 * @param rxBytes amount of bytes received
 */
void IMU_ReadRegister(uint8_t addr, uint8_t *data, uint8_t rxBytes);

/**
 * @brief This function writes an amount of bytes from a specific register start
 * @param addr register address
 * @param data data pointer 
 * @param txBytes amount of bytes written
 */
void IMU_WriteRegister(uint8_t addr, uint8_t *data, uint8_t txBytes);