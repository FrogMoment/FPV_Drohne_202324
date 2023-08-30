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
 *          - S.Bus read
 *          - S.Bus decode
 *          - check receiver disconnection
 *          - motor control
 */

// TODO check disconnect while flying

#ifndef RECEIVER_H_INCLUDED
#define RECEIVER_H_INCLUDED

/************************************************************************************************
------------------------------------------- INCLUDES -------------------------------------------
************************************************************************************************/

#include "main.h"
#include <stm32f1xx_hal.h>

/************************************************************************************************
---------------------------------------- GLOBAL DEFINES ----------------------------------------
************************************************************************************************/

#define PWM_SAFEMODE_DC_MAX 10      // max duty cycle in safe mode
#define PWM_NORMALMODE_DC_MAX 80    // max duty cycle in safe mode
#define PWM_TURN_SPEED_MAX 20       // max addition duty cycle speed when turning 

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
    RECEIVER_TIMEOUT = 2,
    PROTOCOL_ERROR = 3,

    IBUS_ERROR = 4,
    IBUS_HEADER_ERROR = 5,
    IBUS_CHECKSUM_ERROR = 6,

    SBUS_ERROR = 7,
    SBUS_HEADER_ERROR = 8,
    SBUS_FOOTER_ERROR = 9,
    SBUS_SIGNAL_LOST = 10,
    SBUS_SIGNAL_FAILSAFE = 11
} Receiver_Status;

// min and max values of receiver raw data
typedef struct Receiver_Values
{
    uint16_t min, max, delta, half;
} Receiver_Values;

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
 * @param huart pointer to a UART_HandleTypeDef structure (input usart)
 * @param htim pointer to a TIM_HandleTypeDef structure (output pwm timer)
 * @return Receiver_Status 
 */
Receiver_Status Receiver_Init(Receiver_Protocol proto, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);

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

/**
 * @brief this function controls the output pwm signals accourding to the receiver input
 * @retval None
 */
void Receiver_MotorControl(void);

#endif // RECEIVER_H_INCLUDED
