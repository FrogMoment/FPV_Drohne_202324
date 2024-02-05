/**
 * @file datatransmission.h
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


#include "main.h"

 /************************************************************************************************
 ---------------------------------------- GLOBAL DEFINES ----------------------------------------
 ************************************************************************************************/

//  #define 


 /************************************************************************************************
 --------------------------------------- GLOBAL STRUCTURES ---------------------------------------
 ************************************************************************************************/


/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/


/******************************************************
----------------- FUNCTION PROTOTYPES -----------------
******************************************************/

/**
 * @brief This function prepares the data for transmission
 * @retval None
 */
void DATA_ARRANGEMENT(float value, int8_t j);

/**
 * @brief This function sends the data over uart
 * @retval None
 */
void DATA_SEND(int8_t j);

void DATA_INIT(UART_HandleTypeDef *huart);

/**
 * @brief This function transmitts the first package (00) to the port where Audio-In (VTx) is connected
 * @retval None
 */
void DATA_TRANSMISSION_1(float voltage, float grounddistance, int8_t error_code);

/**
 * @brief This function transmitts the second package (FF) to the port where Audio-In (VTx) is connected
 * @retval None
 */
void DATA_TRANSMISSION_2(float pitch, float roll, float yaw);