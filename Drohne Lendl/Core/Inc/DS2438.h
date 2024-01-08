/**
 * @file DS2438.c
 * @author Maximilian Lendl
 * @date 2023-07-11
 * @version 1
 *
 * @copyright Speed Junkies DA 202324
 *
 * @brief This file provides functions for:
 *          - OneWire read
 *          - OneWire write
 *          - DS2438 communication
 *          - Current measurement
 *          - Voltage measurement
 *          - Temperature measurement
 *          - capacity measurement
 */

#ifndef DS2438_H_INCLUDED
#define DS2438_H_INCLUDED

#include "main.h"

 /************************************************************************************************
 ---------------------------------------- GLOBAL DEFINES ----------------------------------------
 ************************************************************************************************/

#define DS2438_SKIP_ROM 0xCC    // Skip ROM function command
#define DS2438_RECALL_MEM 0xB8  // Recall Memory function command
#define DS2438_CONVERT_T 0x44   // Convert Temperature command
#define DS2438_CONVERT_V 0xB4   // Convert Voltage command
#define DS2438_READ_SP 0xBE     // read scratchpad command
#define DS2438_WRITE_SP 0x4E    // write scratchpad command
#define DS2438_COPY_SP 0x48     // write scratchpad command

#define DS2438_RSENS 0.150      // value of sense resistor [ohm]

 /************************************************************************************************
 --------------------------------------- GLOBAL STRUCTURES ---------------------------------------
 ************************************************************************************************/

typedef enum DS2438_Status
{
    DS2438_OK = 0,
    DS2438_ERROR = 1
} DS2438_Status;

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

extern float ds2438_current;        // DS2438 current value
extern float ds2438_voltage;        // DS2438 voltage value
extern float ds2438_temperature;    // DS2438 temperature value    
extern float ds2438_capacity;       // DS2438 capacity value

/************************************************************************************************
-------------------------------------- FUNCTION PROTOTYPES --------------------------------------
************************************************************************************************/

/**
 * @brief This function delays the program in us
 * @param us delay in us
 * @retval None
 */
void Delay_us(uint32_t us);

/**
 * @brief This function initializes the DS2438 (reset + presence pulse)
 * @param htim pointer to TIM_HandleTypeDef (timer for us delay)
 * @return DS2438_Status 
 */
DS2438_Status DS2438_Init(TIM_HandleTypeDef *htim);

/**
 * @brief This function resets / checks device presence
 * @return DS2438_Status 
 */
DS2438_Status DS2438_Reset(void);

/**
 * @brief This function writes one byte to the DS2438
 * @param byte byte to write
 * @retval None
 */
void DS2438_WriteByte(uint8_t byte);

/**
 * @brief This function writes one bit to the DS2438
 * @param bit bit to write
 * @retval None
 */
void DS2438_WriteBit(int8_t bit);

/**
 * @brief This function reads one byte from the DS2438 (LSB first)
 * @return uint8_t
 */
uint8_t DS2438_ReadByte(void);

/**
 * @brief This function reads one bit from the DS2438
 * @return int8_t
 */
int8_t DS2438_ReadBit(void);

/**
 * @brief This function writes the data to one page of the DS2438
 * @param page page number (0 - 7)
 * @param pageData data of page
 * @return DS2438_Status 
 */
DS2438_Status DS2438_WritePage(uint8_t page, int16_t *pageData);

/**
 * @brief This function reads the data from one page of the DS2438
 * @param page page number (0 - 7)
 * @param pageData data of page
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadPage(uint8_t page, int16_t *pageData);

/**
 * @brief This function starts voltage measurement
 * @return DS2438_Status
 */
DS2438_Status DS2438_StartVoltageMeasurement(void);

/**
 * @brief This function returns the control voltage flag bit
 * @return int8_t
 */
int8_t DS2438_ControlVoltageFlag(void);

/**
 * @brief This function reads vica
 * @param data
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadVICA(int8_t *data);

/**
 * @brief This function reads the current current value of the DS2438
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadCurrent(void);

/**
 * @brief This function reads the current voltage value of the DS2438
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadVoltage(void);

/**
 * @brief This function reads the current temperature value of the DS2438
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadTemperature(void);

/**
 * @brief This function reads the current capacity value of the DS2438
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadCapacity(void);

/**
 * @brief This function reads current, voltage, temperature and capacity of the DS2438
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadAllSensors(void);

#endif

