/**
 * @file MPU9250.c
 * @author Maximilian Lendl
 * @brief Functions for MPU9250
 * @version 1
 * @date 2023-06-18
*/

#include "MPU9250.h"

/**
 * @brief This funtion initializes the MPU9250
 * @param hspi1 pointer to a SPI_HandleTypeDef structure
 */
void MPU9250_Init(SPI_HandleTypeDef *hspi1)
{
    // TODO
}

/**
 * @brief read register of MPU9250
 * @param hspi1 pointer to a SPI_HandleTypeDef structure 
 * @param addr register address/es
 * @param data received data
 * @param rxBytes amount of reveived bytes
 */
void MPU9250_ReadRegister(SPI_HandleTypeDef *hspi1, uint8_t addr, uint8_t *data, uint8_t rxBytes)
{
    uint8_t fullAddr = 0x80 | addr; // add read bit to front of address

    HAL_GPIO_WritePin(CS_MS_GPIO_Port, CS_MS_Pin, GPIO_PIN_SET);    // set CS_MS to inactive

    HAL_GPIO_WritePin(CS_MPU_GPIO_Port, CS_MPU_Pin, GPIO_PIN_RESET);// set CS_MPU to active

    HAL_SPI_Transmit(hspi1, &fullAddr, 1, HAL_MAX_DELAY);   // transmit register address
    HAL_SPI_Receive(hspi1, data, rxBytes, HAL_MAX_DELAY);   // receive register data

    HAL_GPIO_WritePin(CS_MPU_GPIO_Port, CS_MPU_Pin, GPIO_PIN_RESET);// set CS_MPU to inactive
}

/**
 * @brief check WHO AM I register value
 * @param hspi1 pointer to a SPI_HandleTypeDef structure
 * @return MPU_status 
 */
MPU_status MPU9250_ReadWhoAmI(SPI_HandleTypeDef *hspi1)
{
    uint8_t data;
    MPU9250_ReadRegister(hspi1, MPU9250_WHOAMI_ADDR, &data, 1); // read data from WHOAMI register
    
    // check if default value
    if(data == 0x71) 
        return MPU_OK;
    else
        return MPU_ERROR;
}