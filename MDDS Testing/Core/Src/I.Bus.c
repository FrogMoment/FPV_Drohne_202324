/**
 * @file I.Bus.c
 * @author Maximilian Lendl
 * @date 2023-07-29
 * @version 1
 * 
 * @copyright Speed Junkies DA 202324
 * 
 * @brief This file provides functions for: 
 *          - I.Bus read
 *          - I.Bus decode
 */

#include "I.Bus.h"

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

uint8_t ibus_RawData[32] = {0}; // raw data of ibus communication
uint16_t ibus_ChData[14] = {0}; // each channel data

/************************************************************************************************
------------------------------------------- FUNCTIONS -------------------------------------------
************************************************************************************************/

/**
 * @brief This function calibrates and starts uart receive dma
 * @param huart pointer to a UART_HandleTypeDef structure
 * @retval IBUS_Status
 */
IBUS_Status IBUS_Init(UART_HandleTypeDef *huart)
{
    uint8_t tmp[2] = {0};

    // calibrate the UART receive to the time
    while(!(tmp[0] == 0x20 && tmp[1] == 0x40))
        HAL_UART_Receive(huart, tmp, 2, 4);

    HAL_Delay(4);

    // start DMA read i.bus signal
    if(HAL_UART_Receive_DMA(huart, ibus_RawData, 32) != HAL_OK)
        return IBUS_ERROR;

    return IBUS_OK;
}

/**
 * @brief This function decodes the ibus raw data
 * @details
 * channel values from 1070 - 1919
 * Ch1: Aileron
 * Ch2: Elevator
 * Ch3: Throttle
 * Ch4: Rudder
 * Ch5: Gear
 * Ch6: Pitch
 * Ch7: THrottle Hold
 * Ch8: Gear switch
 * @return IBUS_Status 
 */
IBUS_Status IBUS_Decode(void)
{
    // check if protocol header is correct
    if(ibus_RawData[0] != 0x20 || ibus_RawData[1] != 0x40)
        return IBUS_PARAM_ERROR;

    // check if checksum is correct (0xFFFF - sum of other 30 bytes = checksum)
    uint16_t sum = 0;
    for(int8_t i = 0; i < 30; i++)
        sum += ibus_RawData[i];

    uint16_t checksum = (ibus_RawData[31] << 8) | ibus_RawData[30];
    if((0xFFFF - sum) != checksum)
        return IBUS_CHECKSUM_ERROR;

    // decode channel data (little endian byte order)
    for(int8_t i = 0, j = 0; i < 14; i++, j+=2)
        ibus_ChData[i] = (ibus_RawData[j+3] << 8) | ibus_RawData[j+2];

    return IBUS_OK;
}

