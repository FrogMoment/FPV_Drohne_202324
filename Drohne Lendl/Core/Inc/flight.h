/**
 * @file flight.h
 * @author Maximilian Lendl
 * @date 2024-02-22
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief This file provides functions for:
 *          - 
 */

#ifndef FLIGHT_H_INCLUDED
#define FLIGHT_H_INCLUDED

#include "main.h"
#include "receiver.h"
#include "dshot_own.h"
#include "PID.h"

typedef struct flightTypeDef
{
    // receiver settings
    Receiver_Protocol inputProtocol;
    UART_HandleTypeDef *inputUART;

    // DShot output settings
    TIM_HandleTypeDef *outputTIM;
    ESC_OutputProtocol outputProtocol;
    TIM_HandleTypeDef *outputUpdateTIM;

    // PID settings
    float deltaTime;

} flightTypeDef;

typedef enum Flight_Status
{
    FLIGHT_OK = 0
} Flight_Status;

/**
 * @brief This function initilizes the input and output of the drone
 * @param hflight pointer to REceiver_FlightTypeDef
 * @return uint8_t (errorCode) 
 */
uint8_t Flight_Init(flightTypeDef *hflight);

#endif // FLIGHT_H_INCLUDED





