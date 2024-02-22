/**
 * @file flight.c
 * @author Maximilian Lendl
 * @date 2024-02-22
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief This file provides functions for:
 *          - 
 */

#include "flight.h"

/**
 * @brief This function initilizes the input and output of the drone
 * @param hflight pointer to REceiver_FlightTypeDef
 * @return uint8_t (errorCode) 
 */
uint8_t Flight_Init(flightTypeDef *hflight)
{
    uint8_t errorCode;

    // initiliaze input via receiver
    errorCode = Receiver_Init(hflight->inputProtocol, hflight->inputUART);
    if(errorCode != RECEIVER_OK)
        return errorCode;

    // initililize output via DSHOT protocol
    errorCode = DShot_Init(hflight->outputTIM, hflight->outputProtocol, hflight->outputUpdateTIM);
    if(errorCode != DSHOT_OK)
        return errorCode;

    // initilize PID
    PID_Init(hflight->deltaTime);

    return FLIGHT_OK;
}












