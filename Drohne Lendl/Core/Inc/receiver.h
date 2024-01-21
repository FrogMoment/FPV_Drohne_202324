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
 *          - S.Bus / I.Bus read and decode
 *          - Motor control via DShot
 *          - failsafe detection
 *          - failsafe handler
 *          - output input data
 */

#ifndef RECEIVER_H_INCLUDED
#define RECEIVER_H_INCLUDED

 /************************************************************************************************
 ------------------------------------------- INCLUDES -------------------------------------------
 ************************************************************************************************/

#include "main.h"
#include "dshot_own.h"
#include <string.h>
#include <stdio.h>
#include "status_handling.h"

 /************************************************************************************************
 ---------------------------------------- GLOBAL DEFINES ----------------------------------------
 ************************************************************************************************/

#define ESC_SAFEMODE_THR_MAX    10      // max throttle in safe mode
#define ESC_NORMALMODE_THR_MAX  80      // max throttle in safe mode
#define ESC_TURN_OFFSET_MAX     10      // max addition throttle speed when turning
#define ESC_OFFMODE_THR         0       // throttle when on/off switch is on off
#define ESC_MOTORTEST_THR       30      // throttle for motor test
#define ESC_LANDING_THR         50      // throttle to land drone in case of failsafe

// channel indices (-1 because of array indices)
#define RECEIVER_YAW_CHANNEL             1 - 1   // yaw channel index
#define RECEIVER_PITCH_CHANNEL           2 - 1   // pitch channel index
#define RECEIVER_THROTTLE_CHANNEL        3 - 1   // throttle channel index
#define RECEIVER_ROLL_CHANNEL            4 - 1   // roll channel index
#define RECEIVER_ONOFF_SWITCH_CHANNEL    5 - 1   // on/off switch channel index
#define RECEIVER_MODESEL_SWTICH_CHANNEL  6 - 1   // mode select channel index

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

// limits of receiver raw data
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
 * @param proto protocol to use (SBUS / IBUS)
 * @param huart pointer to a UART_HandleTypeDef structure (input usart)
 * @param htim_out pointer to a TIM_HandleTypeDef structure (output pwm timer)
 * @param speed_out DSHOT150, DSHOT300 or DSHOT600
 * @return Receiver_Status
 */
Receiver_Status Receiver_Init(Receiver_Protocol proto, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim_out, ESC_OutputProtocol speed_out);

/**
 * @brief This function decodes the receiver raw data depending on the protocol
 * @details
 * i.bus channel values from 1070 - 1920
 * s.bus channel values from 350 - 1680
 *
 * what channel does what depends on the defines found in receiver.h:
 *  - RECEIVER_YAW_CHANNEL
 *  - RECEIVER_PITCH_CHANNEL
 *  - RECEIVER_THROTTLE_CHANNEL
 *  - RECEIVER_ROLL_CHANNEL
 *  - RECEIVER_ONOFF_SWITCH_CHANNEL
 *  - RECEIVER_MODESEL_SWTICH_CHANNEL
 * @return Receiver_Status
 */
Receiver_Status Receiver_Decode(void);

/**
 * @brief This function calculates the stick positions according to the receiver input
 * @details
 * The max throttle values per mode can be changed in receiver.h with:
 *  - ESC_SAFEMODE_THR_MAX
 *  - ESC_NORMALMODE_THR_MAX
 *  - ESC_OFFMODE_THR
 *  - ESC_TURN_OFFSET_MAX
 * @return Receiver_Status
 */
Receiver_Status Receiver_MotorControl(void);

/**
 * @brief This functions sets the throttle values to offmode throttle
 * @details throttle value can be changed with define 'ESC_OFFMODE_THR' found in receiver.h
 * @return Receiver_Status
 */
Receiver_Status Receiver_SetStdDC(void);

/**
 * @brief This function outputs all receiver channels side by side
 * @attention This function uses the global array receiver_ChData[]
 * @param huart pointer to a UART_HandleTypeDef structure (for output)
 * @retval None
 */
void Receiver_OutputChValues(UART_HandleTypeDef *huart);

/**
 * @brief This function sets the drone motors to a throttle that slowly lands the drone
 * @retval Receiver_Status
 */
Receiver_Status Receiver_FailsafeHandler(void);

/**
 * @brief This function saves the current channel data and check if its the same as before
 * @param huart pointer to UART_HandleTypeDef
 * @retval None
 */
void Receiver_CheckEqChData(UART_HandleTypeDef *huart);

#endif // RECEIVER_H_INCLUDED



