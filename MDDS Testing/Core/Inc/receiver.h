/**
 * @file receiver.h
 * @author Maximilian Lendl
 * @date 2023-07-29
 * @version 1
 * 
 * @copyright Speed Junkies DA 202324
 * 
 * @brief This file provides functions for: 
 *          - receiver init
 *          - I.Bus read
 *          - I.Bus decode
 TODO *          - S.Bus read
 TODO *          - S.Bus decode
 MAYBE *          - check receiver disconnection
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

// Receiver output protocol selection
typedef enum Receiver_Protocol
{
    NO_PROTO = 0,
    IBUS = 1,
    SBUS = 2
} Receiver_Protocol;

// Receiver OK / ERROR Codes
typedef enum Receiver_Status
{
    RECEIVER_OK = 0,
    RECEIVER_UART_ERROR = 1,
    PROTOCOL_ERROR = 2,

    IBUS_ERROR = 3,
    IBUS_HEADER_ERROR = 4,
    IBUS_CHECKSUM_ERROR = 5,

    SBUS_ERROR = 6,
    SBUS_HEADER_ERROR = 7,
    SBUS_FOOTER_ERROR = 8,
    SBUS_SIGNAL_LOST = 9,
    SBUS_SIGNAL_FAILSAFE = 10
} Receiver_Status;

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

extern uint8_t receiver_RawData[32];    // raw data of receiver communication
extern uint16_t receiver_ChData[16];    // each channel data

/************************************************************************************************
-------------------------------------- FUNCTION PROTOTYPES --------------------------------------
************************************************************************************************/

/**
 * @brief This function calibrates and starts uart receive dma with selected protocol 
 * @param protocol protocol to use (SBUS / IBUS)
 * @param huart pointer to a UART_HandleTypeDef structure
 * @return Receiver_Status 
 */
Receiver_Status Receiver_Init(Receiver_Protocol protocol, UART_HandleTypeDef *huart);

/**
 * @brief This function decodes the receiver raw data depending on the protocol
 * @details
 * i.bus channel values from 1070 - 1920
 * s.bus channel values from
 * 
 * Ch1: Aileron
 * Ch2: Elevator
 * Ch3: Throttle
 * Ch4: Rudder
 * Ch5: Gear
 * Ch6: Pitch
 * Ch7: THrottle Hold
 * Ch8: Gear switch
 * @return Receiver_Status 
 */
Receiver_Status Receiver_Decode(void);

#endif // IBUS_H_INCLUDED
