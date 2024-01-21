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

#include "DS2438.h"

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

float ds2438_Current = 0;        // DS2438 current value
float ds2438_Voltage = 0;        // DS2438 voltage value
float ds2438_Temperature = 0;    // DS2438 temperature value    
float ds2438_Capacity = 0;       // DS2438 capacity value

TIM_HandleTypeDef *DS2438_DelayTimer;

/************************************************************************************************
------------------------------------------- FUNCTIONS -------------------------------------------
************************************************************************************************/

/**
 * @brief This function delays the program in us
 * @param us delay in us
 * @retval None
 */
void DS2438_DelayUs(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(DS2438_DelayTimer, 0);
    while(__HAL_TIM_GET_COUNTER(DS2438_DelayTimer) < us);
}

/**
 * @brief This function initializes the DS2438
 * @param htim pointer to TIM_HandleTypeDef (timer for us delay)
 * @return DS2438_Status
 */
DS2438_Status DS2438_Init(TIM_HandleTypeDef *htim)
{
    if(htim == NULL)
        return DS2438_ERROR;

    DS2438_DelayTimer = htim;
    HAL_TIM_Base_Start(DS2438_DelayTimer); // start timer for DS2438_DelayUs

    if(DS2438_Reset() == DS2438_ERROR)
        return DS2438_ERROR;

    // set Vad as A/D converter input
    int16_t pageData[9] = {0x00};

    if(DS2438_ReadPage(0x00, pageData) == DS2438_ERROR)
        return DS2438_ERROR;

    // pageData[0] |= 0x08;
    pageData[0] &= 0xF7;

    if(DS2438_WritePage(0x00, pageData) == DS2438_ERROR)
        return DS2438_ERROR;

    return DS2438_OK;
}

/**
 * @brief This function resets / checks device presence
 * @return DS2438_Status
 */
DS2438_Status DS2438_Reset(void)
{
    // reset DS2438
    HAL_GPIO_WritePin(DS2438_DQ_GPIO_Port, DS2438_DQ_Pin, GPIO_PIN_RESET);  // send reset pulse (min 480us)
    Delay_us(480);
    HAL_GPIO_WritePin(DS2438_DQ_GPIO_Port, DS2438_DQ_Pin, GPIO_PIN_SET);    // release line -> change to receive mode

    Delay_us(70); // wait until slave sends presence pulse
    int8_t pin = HAL_GPIO_ReadPin(DS2438_DQ_GPIO_Port, DS2438_DQ_Pin);
    Delay_us(410);

    // read current pin level (0 -> found, 1 -> not found)
    if(pin == GPIO_PIN_SET)
        return DS2438_ERROR;

    return DS2438_OK;
}

/**
 * @brief This function writes one byte to the DS2438
 * @param byte byte to write
 * @retval None
 */
void DS2438_WriteByte(uint8_t byte)
{
    for(int8_t i = 0; i < 8; i++)
    {
        DS2438_WriteBit(byte & 0x01);
        byte >>= 1;
    }
}

/**
 * @brief This function writes one bit to the DS2438
 * @param bit bit to write
 * @retval None
 */
void DS2438_WriteBit(int8_t bit)
{
    if(bit == 1)
    {
        HAL_GPIO_WritePin(DS2438_DQ_GPIO_Port, DS2438_DQ_Pin, GPIO_PIN_RESET);
        Delay_us(10);
        HAL_GPIO_WritePin(DS2438_DQ_GPIO_Port, DS2438_DQ_Pin, GPIO_PIN_SET);
        Delay_us(70);
    }
    else
    {
        HAL_GPIO_WritePin(DS2438_DQ_GPIO_Port, DS2438_DQ_Pin, GPIO_PIN_RESET);
        Delay_us(60);
        HAL_GPIO_WritePin(DS2438_DQ_GPIO_Port, DS2438_DQ_Pin, GPIO_PIN_SET);
        Delay_us(10);
    }
}

/**
 * @brief This function reads one byte from the DS2438 (LSB first)
 * @return uint8_t
 */
uint8_t DS2438_ReadByte(void)
{
    uint8_t byte = 0;

    for(int8_t i = 0; i < 8; i++)
        byte |= (DS2438_ReadBit() << i);

    return byte;
}

/**
 * @brief This function reads one bit from the DS2438
 * @return int8_t
 */
int8_t DS2438_ReadBit(void)
{
    int8_t bit = 0;

    HAL_GPIO_WritePin(DS2438_DQ_GPIO_Port, DS2438_DQ_Pin, GPIO_PIN_RESET);
    Delay_us(10);
    HAL_GPIO_WritePin(DS2438_DQ_GPIO_Port, DS2438_DQ_Pin, GPIO_PIN_SET);
    Delay_us(10);

    // read current pin level
    bit = HAL_GPIO_ReadPin(DS2438_DQ_GPIO_Port, DS2438_DQ_Pin) == GPIO_PIN_SET;

    Delay_us(60);

    return bit;
}

/**
 * @brief This function writes the data to one page of the DS2438
 * @param page page number (0 - 7)
 * @param pageData data of page
 * @return DS2438_Status
 */
DS2438_Status DS2438_WritePage(uint8_t page, int16_t *pageData)
{
    // reset + presence pulse
    if(DS2438_Reset() == DS2438_ERROR)
        return DS2438_ERROR;

    // copy current data to scratchpad
    DS2438_WriteByte(DS2438_SKIP_ROM);
    DS2438_WriteByte(DS2438_WRITE_SP);
    DS2438_WriteByte(page);

    for(uint8_t i = 0; i < 9; i++)
        DS2438_WriteByte(pageData[i]);

    // reset + presence pulse
    if(DS2438_Reset() == DS2438_ERROR)
        return DS2438_ERROR;

    DS2438_WriteByte(DS2438_SKIP_ROM);
    DS2438_WriteByte(DS2438_COPY_SP);
    DS2438_WriteByte(page);

    return DS2438_OK;
}

/**
 * @brief This function reads the data from one page of the DS2438
 * @param page page number (0 - 7)
 * @param pageData data of page
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadPage(uint8_t page, int16_t *pageData)
{
    // reset + presence pulse
    if(DS2438_Reset() == DS2438_ERROR)
        return DS2438_ERROR;

    // copy current data to scratchpad
    DS2438_WriteByte(DS2438_SKIP_ROM);
    DS2438_WriteByte(DS2438_RECALL_MEM);
    DS2438_WriteByte(page);

    // reset + presence pulse
    if(DS2438_Reset() == DS2438_ERROR)
        return DS2438_ERROR;

    // read scratchpad data
    DS2438_WriteByte(DS2438_SKIP_ROM);
    DS2438_WriteByte(DS2438_READ_SP);
    DS2438_WriteByte(page);

    for(int8_t i = 0; i < 9; i++)
        pageData[i] = DS2438_ReadByte();

    return DS2438_OK;
}

/**
 * @brief This function starts voltage measurement
 * @return DS2438_Status
 */
DS2438_Status DS2438_StartVoltageMeasurement(void)
{
    // reset + presence pulse
    if(DS2438_Reset() == DS2438_ERROR)
        return DS2438_ERROR;

    // read voltage data
    DS2438_WriteByte(DS2438_SKIP_ROM);
    DS2438_WriteByte(DS2438_CONVERT_V);

    return DS2438_OK;
}

/**
 * @brief This function returns the control voltage flag bit
 * @return int8_t
 */
int8_t DS2438_ControlVoltageFlag(void)
{
    int16_t pageData[9] = {0x00};

    DS2438_ReadPage(0x00, pageData);

    return pageData[0] & 0x40; // 1 = busy, 0 = ready
}

/**
 * @brief This function reads vica
 * @param data
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadVICA(int8_t *data)
{
    int16_t pageData[9] = {0x00};

    if(DS2438_ReadPage(0x01, pageData) == DS2438_ERROR)
        return DS2438_ERROR;

    *data = pageData[4];

    return DS2438_OK;
}

/**
 * @brief This function reads the current current value of the DS2438
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadCurrent(void)
{
    int16_t pageData[9] = {0x00};

    if(DS2438_ReadPage(0x00, pageData) == DS2438_ERROR)
        return DS2438_ERROR;

    // reading current 
    int16_t currentLSB = pageData[5];
    int16_t currentMSB = pageData[6];

    int8_t tmp = (((currentMSB & 0x3) << 8) | (currentLSB));

    if(currentMSB & ~0x3)
        tmp *= -1;

    ds2438_Current = tmp / (4096 * DS2438_RSENS);

    return DS2438_OK;
}

/**
 * @brief This function reads the current voltage value of the DS2438
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadVoltage(void)
{
    if(DS2438_StartVoltageMeasurement() == DS2438_ERROR)
        return DS2438_ERROR;

    // wait for measurement to be complete (1 = busy, 0 = ready)
    while(DS2438_ControlVoltageFlag());

    int16_t pageData[9] = {0x00};

    if(DS2438_ReadPage(0x00, pageData) == DS2438_ERROR)
        return DS2438_ERROR;

    // extracting voltage bits 
    int16_t voltageLSB = pageData[3];
    int16_t voltageMSB = pageData[4];

    ds2438_Voltage = (((voltageMSB & 0x3) << 8) | (voltageLSB)) / 100.0;
    // TODO ds2438_voltage *= 2; // because of resistor voltage divider 

    return DS2438_OK;
}

/**
 * @brief This function reads the current temperature value of the DS2438
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadTemperature(void)
{
    int16_t pageData[9] = {0x00};

    if(DS2438_ReadPage(0x00, pageData) == DS2438_ERROR)
        return DS2438_ERROR;

    // extracting temp bits 
    int16_t tempLSB = pageData[1];
    int16_t tempMSB = pageData[2];

    float tmp = tempMSB;
    tmp += (tempLSB >> 3) * 0.03125;

    if(tempMSB & 0x80)
        tmp *= -1;

    ds2438_Temperature = tmp;

    return DS2438_OK;
}

/**
 * @brief This function reads the current capacity value of the DS2438
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadCapacity(void)
{
    int8_t vica = 0;

    if(DS2438_ReadVICA(&vica) == DS2438_ERROR)
        return DS2438_ERROR;

    ds2438_Capacity = vica / (2048 * DS2438_RSENS);

    return DS2438_OK;
}

/**
 * @brief This function reads current, voltage, temperature and capacity of the DS2438
 * @return DS2438_Status
 */
DS2438_Status DS2438_ReadAllSensors(void)
{
    DS2438_ReadCurrent();
    DS2438_ReadVoltage();
    DS2438_ReadTemperature();
    DS2438_ReadCapacity();

    return DS2438_OK;
}

