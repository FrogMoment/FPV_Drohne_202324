/**
 * @file dshot_own.c
 * @author Maximilian Lendl
 * @date 2023-12-01
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief 
 * 
 */

#include "dshot_own.h"

/**********************************************************************
--------------------------- GLOBAL VARIABLE ---------------------------
**********************************************************************/

uint16_t data[4] = {0};
int8_t send = 0;        // sending flag
uint16_t oneDC, zeroDC;

TIM_HandleTypeDef *DShot_OutputTim = NULL;

/**********************************************************************
------------------------- FUNCTIONS VARIABLE -------------------------
**********************************************************************/

/**
 * @brief Timer PWM Pulse finished callback
 * @param htim timer pointer
 */
// void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
void DShot_TimPWMPulseFinCallback(TIM_HandleTypeDef *htim)
{
    // check pointer and sending flag
    if(!send || htim != DShot_OutputTim)
        return;

    static int8_t index = 0;
    static uint16_t extractingBit = 0x8000;

    // check if bit needs to be extracted
    if(extractingBit != 0x0000)
    {
        // sending bit 1
        if(data[index] & extractingBit)
            __HAL_TIM_SET_COMPARE(DShot_OutputTim, index * 4, oneDC);
        
        // sending bit 0
        else
            __HAL_TIM_SET_COMPARE(DShot_OutputTim, index * 4, zeroDC);
        
        extractingBit >>= 1;
    }
    else
    {
        // set idle value (0%)
        __HAL_TIM_SET_COMPARE(DShot_OutputTim, index * 4, 0);

        // increment index and check for max value
        if(index++ >= 4)
        {
            index = 0;  // reset index
            send = 0;   // reset sending flag
        }

        extractingBit = 0x8000; // reset extracting bit
    }
}

/**
 * @brief This function initializes the output ESC DShot signal
 * @param htim pointer to TIM_HandleTypeDef (output timer)
 * @return DShot_Status 
 */
DShot_Status DShot_Init(TIM_HandleTypeDef *htim)
{
    // set and check timer pointer
    DShot_OutputTim = htim;
    if(DShot_OutputTim == NULL)
        return DSHOT_TIM_ERROR;

    oneDC = __HAL_TIM_GET_AUTORELOAD(DShot_OutputTim) * 0.74;   // calculate DC for sending bit 1
    zeroDC = __HAL_TIM_GET_AUTORELOAD(DShot_OutputTim) * 0.37;  // calculate DC for sending bit 0

    // set custom callback function
    HAL_TIM_RegisterCallback(DShot_OutputTim, HAL_TIM_PWM_PULSE_FINISHED_CB_ID, DShot_TimPWMPulseFinCallback);

    // set default duty cycle (0%)
    __HAL_TIM_SET_COMPARE(DShot_OutputTim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(DShot_OutputTim, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(DShot_OutputTim, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(DShot_OutputTim, TIM_CHANNEL_4, 0);

    // start output timer channels
    HAL_TIM_PWM_Start(DShot_OutputTim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(DShot_OutputTim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(DShot_OutputTim, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start_IT(DShot_OutputTim, TIM_CHANNEL_1); // interrupt for duty cycle change while transmitting
    
    return DSHOT_OK;
}

/**
 * @brief This function formats the motor data for the DShot protocol
 * @param motorLF percent of throttle value of left front motor
 * @param motorRF percent of throttle value of right front motor
 * @param motorLR percent of throttle value of left rear motor
 * @param motorRR percent of throttle value of right rear
 * @retval None
 */
void DShot_SendData(double motorLF, double motorRF, double motorLR, double motorRR)
{
    // get throttle integar value
    uint16_t throttle[4];
    throttle[0] = round(motorLF);
    throttle[1] = round(motorRF);
    throttle[2] = round(motorLR);
    throttle[3] = round(motorRR);

    int telemetry = 0, withoutCS;

    for(int8_t i = 0; i < 4; i++)
    {
        throttle[i] = 48 + 20 * throttle[i]; // get value (48 - 2047)
        if(throttle[i] > 2047) // check max value
            throttle[i] = 2047;

        // first 12 bits (without Checksum)
        withoutCS = (throttle[i] << 1) | telemetry; 

        // format whole data frame
        data[i] = withoutCS << 4 | (withoutCS ^ (withoutCS >> 4) ^ (withoutCS >> 8)) & 0x0F;     
    }

    DShot_StartSending(); // start sending cycle for all channels
}

/**
 * @brief This function starts the sending process of the DShot protocol
 * @attention This function sends the data that is stored in data[0-3]
 * @return None 
 */
void DShot_StartSending(void)
{  
    send = 1; // set sending flag
}