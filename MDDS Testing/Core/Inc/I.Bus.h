/**
 * @file I.Bus.h
 * @author Maximilian Lendl
 * @date 2023-07-29
 * @version 1
 * 
 * @copyright Speed Junkies DA 202324
 * 
 * @brief This file provides functions for: 
 *          - I.Bus read
 *          - I.Bus decode
 TODO *          - Receiver disconnect
 */

#ifndef IBUS_H_INCLUDED
#define IBUS_H_INCLUDED

/************************************************************************************************
------------------------------------------- INCLUDES -------------------------------------------
************************************************************************************************/

#include "main.h"
#include <stm32f1xx_hal.h>

/************************************************************************************************
--------------------------------------- GLOBAL STRUCTURES ---------------------------------------
************************************************************************************************/

typedef enum IBUS_Status
{
    IBUS_OK = 0,
    IBUS_ERROR = 1,
    IBUS_PARAM_ERROR = 2,
    IBUS_CHECKSUM_ERROR = 3
} IBUS_Status;

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

extern uint8_t ibus_RawData[32];    // raw data of ibus communication
extern uint16_t ibus_ChData[14];    // each channel data

/************************************************************************************************
-------------------------------------- FUNCTION PROTOTYPES --------------------------------------
************************************************************************************************/

/**
 * @brief This function calibrates and starts uart receive dma
 * @param huart pointer to a UART_HandleTypeDef structure
 * @retval IBUS_Status
 */
IBUS_Status IBUS_Init(UART_HandleTypeDef *huart);

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
IBUS_Status IBUS_Decode(void);

#endif // IBUS_H_INCLUDED
