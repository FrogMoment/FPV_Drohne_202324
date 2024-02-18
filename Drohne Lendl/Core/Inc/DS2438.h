/**
 * @file DS2438.h
 * @author Maximilian Lendl, Marcel Bieder, Lukas Lindmayr
 * @date 2023-07-11
 * @version 1
 *
 * @copyright FPV Drohne DA 202324
 *
 * @brief This file provides functions for:
 *          - OneWire read
 *          - OneWire write
 *          - DS2438 communication
 *          - Voltage measurement
 *          - Temperature measurement
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
#define DS2438_COPY_SP 0x48     // copy scratchpad command

#define DS2438_RSENS 0.150      // value of sense resistor [ohm]

#define DS2438_MIN_VOLTAGE 19   // voltage to send errors and switch to failsafe

/************************************************************************************************
--------------------------------------- GLOBAL STRUCTURES ---------------------------------------
************************************************************************************************/

typedef enum DS2438_Status
{
    DS2438_OK = 0,
    DS2438_ERROR = 1,
    DS2438_VOLTAGE_ERROR = 2
} DS2438_Status;

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

extern float ds2438_Current;        // DS2438 current value
extern float ds2438_Voltage;        // DS2438 voltage value
extern float ds2438_Temperature;    // DS2438 temperature value    
extern float ds2438_Capacity;       // DS2438 capacity value

/************************************************************************************************
-------------------------------------- FUNCTION PROTOTYPES --------------------------------------
************************************************************************************************/

/**
 * @brief This function delays the program in us
 * @param us delay in us
 * @retval None
 */
void DS2438_DelayUs(uint32_t us);

/**
 * @brief This function initializes the DS2438
 * @param htim pointer to TIM_HandleTypeDef (timer for us delay)
 * @param gpio_Port GPIOx (x dependend on port)
 * @param gpio_Pin GPIO_PIN_x (x dependend on pin)
 * @return DS2438_Status
 */
DS2438_Status DS2438_Init(TIM_HandleTypeDef *htim, GPIO_TypeDef *gpio_Port, uint16_t gpio_Pin);

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
int8_t DS2438_ReadControlVoltageFlag(void);

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

#endif // DS2438_H_INCLUDED

