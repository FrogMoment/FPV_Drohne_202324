/**
 * @file datatransmission.c
 * @author Ben Heinicke
 * @date 2023-10-04
 * @version 1
 *
 * @copyright Speed Junkies DA 202324
 *
 * @brief This file provides functions for:
 *          - Arrange sensor data for transmission
 *          - send sensor data to audio input (VTx)
 */

#include "datatransmission.h"
//!delete later, only for testing
#include "string.h"

 /************************************************************************************************
 --------------------------------------- GLOBAL VARIABLES ---------------------------------------
 ************************************************************************************************/

UART_HandleTypeDef *DATA_OUTPUT;
int8_t bytes[4];

/******************************************************
---------------------- FUNCTIONS ----------------------
******************************************************/

/**
 * @brief This function prepares the data for transmission
 * @retval None
 */
void DATA_ARRANGEMENT(float value)  
{
    int32_t datatosend;

    // convert int into floating point number with 32 bits
    memcpy(&datatosend, &value, sizeof(datatosend)); 

    // split into 4 bytes to transmit 1 byte each
    bytes[0] = (int8_t)(datatosend >> 24);
    bytes[1] = (int8_t)((datatosend >> 16) & 0xFF);
    bytes[2] = (int8_t)((datatosend >> 8) & 0xFF);
    bytes[3] = (int8_t)(datatosend & 0xFF);
}

void DATA_SEND()
{
    // send each byte over uart (4 bytes)
    for(int i = 0; i < 4; i++)
    {
        HAL_UART_Transmit(DATA_OUTPUT, (uint8_t*)&bytes[i], sizeof(bytes[0]), HAL_MAX_DELAY);
    }
    // HAL_UART_Transmit_DMA(DATA_OUTPUT, (uint8_t*)bytes, 4);
}

void DATA_INIT(UART_HandleTypeDef *huart)
{
    DATA_OUTPUT = huart;
}

/**
 * @brief This function transmitts the general data to the port where Audio-In (VTx) is connected, from Uart3 PORT PB10
 * @retval None
 */
void DATA_TRANSMISSION_1(float voltage, float grounddistance, int8_t error_code)
{
    // in general each byte is send with LSB first (uart transmit)

    int8_t syncbyte = 0x00;

    // send syncro byte
    HAL_UART_Transmit(DATA_OUTPUT, (uint8_t*)&syncbyte, sizeof(syncbyte), HAL_MAX_DELAY);
    // HAL_UART_Transmit_DMA(DATA_OUTPUT, (uint8_t*)&syncbyte, 1);

    // send battery-voltage over UART
    DATA_ARRANGEMENT(voltage);
    DATA_SEND();

    // send ground distance value over UART
    DATA_ARRANGEMENT(grounddistance);
    DATA_SEND();

    // send error-codes
    HAL_UART_Transmit(DATA_OUTPUT, (uint8_t*)&error_code, sizeof(error_code), HAL_MAX_DELAY);
    // HAL_UART_Transmit_DMA(DATA_OUTPUT, (uint8_t*)&error_code, 1);
}

/**
 * @brief This function transmitts the MPU data to the port where Audio-In (VTx) is connected, from Uart3 PORT PB10
 * @retval None
 */
void DATA_TRANSMISSION_2(float pitch, float roll, float yaw)
{
    // in general each byte is send with LSB first (uart transmit)

    int8_t syncbyte = 0xFF;

    // send syncro byte
    // HAL_UART_Transmit(DATA_OUTPUT, (uint8_t*)&syncbyte, sizeof(syncbyte), HAL_MAX_DELAY);
    // HAL_UART_Transmit_DMA(DATA_OUTPUT, (uint8_t*)&syncbyte, 1);

    // send pitch value over UART
    DATA_ARRANGEMENT(pitch);
    DATA_SEND();
    
    // send roll value over UART
    DATA_ARRANGEMENT(roll);
    DATA_SEND();

    // send yaw value over UART
    DATA_ARRANGEMENT(yaw);
    DATA_SEND();
}




