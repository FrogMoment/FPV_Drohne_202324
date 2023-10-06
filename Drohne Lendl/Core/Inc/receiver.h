/**
 * @file receiver.h
 * @author Maximilian Lendl
 * @date 2023-07-29
 * @version 1
 *
 * @copyright FPV Drohne DA 202324
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

#ifndef RECEIVER_H_INCLUDED
#define RECEIVER_H_INCLUDED

 /************************************************************************************************
 ------------------------------------------- INCLUDES -------------------------------------------
 ************************************************************************************************/

#include "main.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

 /************************************************************************************************
 ---------------------------------------- GLOBAL DEFINES ----------------------------------------
 ************************************************************************************************/

// duty cycles in various modes (*10 because of value range of 0-1000 / 0.0%-100.0%)
#define PWM_SAFEMODE_DC_MAX     05      // max duty cycle in safe mode
#define PWM_NORMALMODE_DC_MAX   80      // max duty cycle in safe mode
#define PWM_TURN_OFFSET_MAX     10      // max addition duty cycle speed when turning
#define PWM_OFFMODE_DC          .1 * 10 // duty cycle when on/off switch is on off
#define PWM_MOTORTEST_DC        50 * 10 // duty cycle for motor test

#define PWM_HOVER_DC            50 * 10 // duty cycle for hovering
#define PWM_SIGNAL_LOST_OFFSET  10 * 10 // duty cycle offset for landing while receiver disconnected

// channel indices (-1 because of array indices)
#define YAW_CHANNEL             1 - 1   // yaw channel index
#define PITCH_CHANNEL           2 - 1   // pitch channel index
#define THROTTLE_CHANNEL        3 - 1   // throttle channel index
#define ROLL_CHANNEL            4 - 1   // roll channel index
#define ONOFF_SWITCH_CHANNEL    5 - 1   // on/off switch channel index
#define MODESEL_SWTICH_CHANNEL  6 - 1   // mode select channel index

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
    RECEIVER_OK = 0,                // receiver ok
    RECEIVER_UART_ERROR = 1,        // uart configuration doesnt match selected protocol
    RECEIVER_PWM_ERROR = 2,         // pwm timer not set
    RECEIVER_PPM_ERROR = 3,         // IBUS selected: PPM not configured correctly
    RECEIVER_TIMEOUT = 4,           // no data signal found
    PROTOCOL_ERROR = 5,             // selected protocol wrong

    IBUS_ERROR = 6,                 // IBUS UART DMA not starting 
    IBUS_HEADER_ERROR = 7,          // IBUS header is wrong
    IBUS_CHECKSUM_ERROR = 8,        // IBUS checksum is wrong
    IBUS_SIGNAL_LOST_ERROR = 9,     // IBUS signal lost

    SBUS_ERROR = 10,                // SBUS UART DMA not starting 
    SBUS_HEADER_ERROR = 11,         // SBUS header is wrong
    SBUS_FOOTER_ERROR = 12,         // SBUS footer is wrong
    SBUS_SIGNAL_LOST = 13,          // SBUS signal lost flag is set
    SBUS_SIGNAL_FAILSAFE = 14       // SBUS signal failsafe flag is set
} Receiver_Status;

// min and max values of receiver raw data
typedef struct Receiver_Values
{
    uint16_t min, max, delta, half;
} Receiver_Values;

// motor position (right/left front/rear)
typedef struct Motor_Position
{
    double RF, LF, RR, LR;
} Motor_Position;

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

extern UART_HandleTypeDef *receiver_InputUART;
extern uint8_t receiver_RawData[32];    // raw data of receiver communication
extern uint16_t receiver_ChData[16];    // each channel data

/************************************************************************************************
-------------------------------------- FUNCTION PROTOTYPES --------------------------------------
************************************************************************************************/

/**
 * @brief This function calibrates and starts uart receive dma with selected protocol
 * @param protocol protocol to use (SBUS / IBUS)
 * @param huart pointer to a UART_HandleTypeDef structure (input usart)
 * @param htim_out pointer to a TIM_HandleTypeDef structure (output pwm timer)
 * @return Receiver_Status
 */
Receiver_Status Receiver_Init(Receiver_Protocol proto, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim_out);

/**
 * @brief This function decodes the receiver raw data depending on the protocol
 * @details
 * i.bus channel values from 1070 - 1920
 * s.bus channel values from 350 - 1680
 *
 * Ch1: Yaw (rotate left / rotate right)
 * Ch2: Pitch (forwards / backwards)
 * Ch3: Throttle (up / down)
 * Ch4: Roll (left / right)
 * Ch5: on/off switch
 * Ch6: mode select (3 way switch)
 * Ch7: not used
 * Ch8: not used
 * @return Receiver_Status
 */
Receiver_Status Receiver_Decode(void);

/**
 * @brief this function controls the output pwm output signals according to the receiver input
 * @return Receiver_Status
 */
Receiver_Status Receiver_MotorControl(void);

/**
 * @brief This functions sets the duty cycle to the std value 0.1%
 * @return Receiver_Status
 */
Receiver_Status Receiver_SetStdDC(void);

/**
 * @brief This function output all receiver channels side by side
 * @attention This function uses the global array receiver_ChData[]
 * @param huart pointer to a UART_HandleTypeDef structure (for output)
 * @retval None
 */
void Receiver_OutputChValues(UART_HandleTypeDef *huart);

/**
 * @brief This function test each motor
 * @details
 * turn motor 1 for 2 seconds on then next motor etc
 * @retval None
 */

/**
 * @brief This function test each motor
 * @param htim pointer to a TIM_HandleTypeDef structure (output pwm timer)
 * @details turn motor 1 for 2 seconds on then next motor etc
 * @retval None
 */
void Receiver_MotorTest(TIM_HandleTypeDef *htim);

/**
 * @brief This function sets the drone motors under the hover duty cycle -> landing
 * @retval Receiver_Status
 */
Receiver_Status Receiver_SignalLostHandler(void);

/**
 * @brief This function saves the current channel data and check if its the same as before
 * @retval None
 */
void Receiver_SaveChannelData(void);

#endif // RECEIVER_H_INCLUDED
