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
int8_t bytes[3] [4];

/******************************************************
---------------------- FUNCTIONS ----------------------
******************************************************/

// /**
//  * @brief This function prepares the data for transmission
//  * @retval None
//  */
// void DATA_ARRANGEMENT(float value, int8_t j)  
// {
//     int32_t datatosend;

//     j--;

//     // convert int into floating point number with 32 bits
//     memcpy(&datatosend, &value, sizeof(datatosend)); 

//     // split into 4 bytes to transmit 1 byte each
//     bytes[j] [0] = (int8_t)(datatosend >> 24);
//     bytes[j] [1] = (int8_t)((datatosend >> 16) & 0xFF);
//     bytes[j] [2] = (int8_t)((datatosend >> 8) & 0xFF);
//     bytes[j] [3] = (int8_t)(datatosend & 0xFF);
// }

// void DATA_SEND(int8_t j)
// {
//     j--;

//     // send each byte over uart (4 bytes)
//     for(int i = 0; i < 4; i++)
//     {
//         HAL_UART_Transmit(DATA_OUTPUT, (uint8_t*)&bytes[j] [i], sizeof(bytes[0] [0]), HAL_MAX_DELAY);
//     }
// }

// void DATA_INIT(UART_HandleTypeDef *huart)
// {
//     DATA_OUTPUT = huart;
// }

// /**
//  * @brief This function transmitts the general data to the port where Audio-In (VTx) is connected, from Uart3 PORT PB10
//  * @retval None
//  */
// void DATA_TRANSMISSION_1(float voltage, float grounddistance, int8_t error_code)
// {
//     // in general each byte is send with LSB first (uart transmit)

//     int8_t syncbyte = 0x00;

//     // arrange battery-voltage over UART
//     DATA_ARRANGEMENT(voltage, 1);
//     // arrange ground distance value over UART
//     DATA_ARRANGEMENT(grounddistance, 2);

//     // send syncro byte
//     HAL_UART_Transmit(DATA_OUTPUT, (uint8_t*)&syncbyte, sizeof(syncbyte), HAL_MAX_DELAY);
//     // send battery-voltage over UART
//     DATA_SEND(1);
//     // send ground distance value over UART
//     DATA_SEND(2);
//     // send error-codes
//     HAL_UART_Transmit(DATA_OUTPUT, (uint8_t*)&error_code, sizeof(error_code), HAL_MAX_DELAY);
// }

// /**
//  * @brief This function transmitts the MPU data to the port where Audio-In (VTx) is connected, from Uart3 PORT PB10
//  * @retval None
//  */
// void DATA_TRANSMISSION_2(float pitch, float roll, float yaw)
// {
//     // in general each byte is send with LSB first (uart transmit)

//     int8_t syncbyte = 0xFF;

//     // send pitch value over UART
//     DATA_ARRANGEMENT(pitch, 1);
//     // send roll value over UART
//     DATA_ARRANGEMENT(roll, 2);
//     // send yaw value over UART
//     DATA_ARRANGEMENT(yaw, 3);

//     // send syncro byte
//     HAL_UART_Transmit(DATA_OUTPUT, (uint8_t*)&syncbyte, sizeof(syncbyte), HAL_MAX_DELAY);
//     // send pitch value over UART
//     DATA_SEND(1);
//     // send roll value over UART
//     DATA_SEND(2);
//     // send yaw value over UART
//     DATA_SEND(3);

// }

uint8_t transmission_buffer[20];

/**
 * @brief This function prepares the data for transmission
 * @retval None
 */
void DATA_ARRANGEMENT(float value, int8_t j)  
{
    int32_t datatosend;

    j--;

    // convert int into floating point number with 32 bits
    memcpy(&datatosend, &value, sizeof(datatosend)); 

    transmission_buffer[1+4*j] = (int8_t)(datatosend >> 24);
    transmission_buffer[2+4*j] = (int8_t)((datatosend >> 16) & 0xFF);
    transmission_buffer[3+4*j] = (int8_t)((datatosend >> 8) & 0xFF);
    transmission_buffer[4+4*j] = (int8_t)(datatosend & 0xFF);

    // split into 4 bytes to transmit 1 byte each
    // bytes[j] [0] = (int8_t)(datatosend >> 24);
    // bytes[j] [1] = (int8_t)((datatosend >> 16) & 0xFF);
    // bytes[j] [2] = (int8_t)((datatosend >> 8) & 0xFF);
    // bytes[j] [3] = (int8_t)(datatosend & 0xFF);
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

    // add synchronisation byte
    transmission_buffer[0] = syncbyte;
    // arrange battery-voltage over UART
    DATA_ARRANGEMENT(voltage, 1);
    // arrange ground distance value over UART
    DATA_ARRANGEMENT(grounddistance, 2);
    // add errorcode
    transmission_buffer[9] = error_code;

    // send package 1
    HAL_UART_Transmit(DATA_OUTPUT, transmission_buffer, 10, HAL_MAX_DELAY);
}

/**
 * @brief This function transmitts the MPU data to the port where Audio-In (VTx) is connected, from Uart3 PORT PB10
 * @retval None
 */
void DATA_TRANSMISSION_2(float pitch, float roll, float yaw)
{
    // in general each byte is send with LSB first (uart transmit)

    int8_t syncbyte = 0xFF;

    // add synchronisation byte
    transmission_buffer[0] = syncbyte;
    // send pitch value over UART
    DATA_ARRANGEMENT(pitch, 1);
    // send roll value over UART
    DATA_ARRANGEMENT(roll, 2);
    // send yaw value over UART
    DATA_ARRANGEMENT(yaw, 3);

    // send package 2
    HAL_UART_Transmit(DATA_OUTPUT, transmission_buffer, 13, HAL_MAX_DELAY);
}