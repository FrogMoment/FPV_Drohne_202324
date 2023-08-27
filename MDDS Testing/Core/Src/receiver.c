/**
 * @file receiver.c
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

#include "receiver.h"

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

// receiver variables
Receiver_Protocol protocol = NO_PROTO; // selected serial protocol
uint8_t receiver_RawData[32] = {0}; // raw data of receiver communication
uint16_t receiver_ChData[16] = {0}; // each channel data

// pwm output variables
TIM_HandleTypeDef *pwm_Timer; // pointer to TIM_HandleTypeDef of output pwm signal
Receiver_Values receiver_Input = {0};

/************************************************************************************************
------------------------------------------- FUNCTIONS -------------------------------------------
************************************************************************************************/

/**
 * @brief This function calibrates and starts uart receive dma with selected protocol 
 * @param protocol protocol to use (SBUS / IBUS)
 * @param huart pointer to a UART_HandleTypeDef structure (input usart)
 * @param htim pointer to a TIM_HandleTypeDef structure (output pwm timer)
 * @return Receiver_Status 
 */
Receiver_Status Receiver_Init(Receiver_Protocol proto, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim)
{
    protocol = proto; // select serial protocol
    switch(protocol)
    {
        /**
         * 115200 baud
         * 8 data bits, 1 stop bit, no parity
         * LSB first, not inverted
         * 32 Bytes:
         *      Byte[0]: protocol length, 0x20
         *      Byte[1]: command code, 0x40
         *      Byte[2-29]: channel data, 14 channels, 2 byte each, little endian
         *      Byte[30-31]: checksum, little endian, 0xFFFF - sum of other 30 bytes = checksum
         */
        case IBUS:
        {
            // check if uart is configured via baudrate
            if(huart->Init.BaudRate != 115200)
                return RECEIVER_UART_ERROR;

            uint8_t tmp[2] = {0}, timeout = 0;
        
            // calibrate reception to begin of protocol
            while(!(tmp[0] == 0x20 && tmp[1] == 0x40))
            {
                HAL_UART_Receive(huart, tmp, 2, 4);
                if(timeout++ > 100)
                    return RECEIVER_TIMEOUT;
            }
            HAL_Delay(4);

            // start DMA read i.bus signal
            if(HAL_UART_Receive_DMA(huart, receiver_RawData, 32) != HAL_OK)
                return IBUS_ERROR;

            receiver_Input.min = 1070;  // set min value of receiver input data
            receiver_Input.max = 1920;  // set max value of receiver input data
            break;
        }

        /**
         * 100000 baud
         * 8 data bits, 2 stop bit, even parity
         * LSB first, inverted
         * 25 Bytes:    
         *      Byte[0]: protocol header, 0x0F
         *      Byte[1-22]: channel data, 16 channels, 11 bits each
         *      Byte[23]:
         *          bit[4]: signal failsafe flag
         *          bit[5]: signal lost flag
         *          bit[6]: digital channel 18
         *          bit[7]: digital channel 17
         *      Byte[24]: protocol footer, 0x00
         */
        case SBUS:
        {
            // check if uart is configured via baudrate
            if(huart->Init.BaudRate != 100000)
                return RECEIVER_UART_ERROR;

            uint8_t tmp = 0, timeout = 0;

            // calibrate reception to begin of protocol
            while(tmp != 0x0F)
            {
                HAL_UART_Receive(huart, &tmp, 1, 4);
                if(timeout++ > 100)
                    return RECEIVER_TIMEOUT;
            }
            HAL_Delay(4);

            // start DMA read s.bus signal
            if(HAL_UART_Receive_DMA(huart, receiver_RawData, 25) != HAL_OK)
                return SBUS_ERROR;

            receiver_Input.min = 350;   // set min value of receiver input data
            receiver_Input.max = 1680;  // set max value of receiver input data
            break;
        }

        // wrong or no protocol selected
        case NO_PROTO:
        default:
            return PROTOCOL_ERROR;           
            break;
    }

    receiver_Input.delta = receiver_Input.max - receiver_Input.min;
    receiver_Input.half = (receiver_Input.max + receiver_Input.min) / 2;

    // set start duty cycle (0.1%) and start pwm output
    pwm_Timer = htim;
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_1, 1);
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_2, 1);
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_3, 1);
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_4, 1);

    HAL_TIM_PWM_Start(pwm_Timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(pwm_Timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(pwm_Timer, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(pwm_Timer, TIM_CHANNEL_4);

    return RECEIVER_OK;
}

/**
 * @brief This function decodes the receiver raw data depending on the protocol
 * @details
 * i.bus channel values from 1070 - 1920
 * s.bus channel values from 350 - 1680
 * 
 * Ch1: Aileron
 * Ch2: Elevator
 * Ch3: Throttle
 * Ch4: Rudder
 * Ch5: Gear
 * Ch6: Pitch
 * Ch7: Throttle Hold
 * Ch8: Gear switch
 * @return Receiver_Status 
 */
Receiver_Status Receiver_Decode(void)
{
    switch(protocol)
    {
        case IBUS:
            // check if protocol header is correct
            if(receiver_RawData[0] != 0x20 || receiver_RawData[1] != 0x40)
                return IBUS_HEADER_ERROR;

            // check if checksum is correct (0xFFFF - sum of other 30 bytes = checksum)
            uint16_t sum = 0;
            for(int8_t i = 0; i < 30; i++)
                sum += receiver_RawData[i];

            uint16_t checksum = (receiver_RawData[31] << 8) | receiver_RawData[30];
            if((0xFFFF - sum) != checksum)
                return IBUS_CHECKSUM_ERROR;

            // decode channel data (14 channels, 2 bytes each, little endian byte order)
            for(int8_t i = 0, j = 0; i < 14; i++, j+=2)
                receiver_ChData[i] = (receiver_RawData[j+3] << 8) | receiver_RawData[j+2];

            break;


        case SBUS:
            // check if protocol header is correct
            if(receiver_RawData[0] != 0x0F)
                return SBUS_HEADER_ERROR;

            // check if protocol footer is correct
            if(receiver_RawData[24] != 0x00)
                return SBUS_FOOTER_ERROR;

            // check signal lost flags
            if(receiver_RawData[23] & 0x20)
                return SBUS_SIGNAL_LOST;
            
            // check signal failsafe flag
            if(receiver_RawData[23] & 0x10)
                return SBUS_SIGNAL_FAILSAFE;

            // decode channel data (16 channels, 11 bits each, lsb first)
            for(int8_t i = 0, j = 0; i < 16; i+=8, j+=11)
            {
                receiver_ChData[i+0] = ((receiver_RawData[j+1]  >> 0) | (receiver_RawData[j+2]  << 8)) & 0x7FF;
                receiver_ChData[i+1] = ((receiver_RawData[j+2]  >> 3) | (receiver_RawData[j+3]  << 5)) & 0x7FF;
                receiver_ChData[i+2] = ((receiver_RawData[j+3]  >> 6) | (receiver_RawData[j+4]  << 2) | (receiver_RawData[5] << 10)) & 0x7FF;
                receiver_ChData[i+3] = ((receiver_RawData[j+5]  >> 1) | (receiver_RawData[j+6]  << 7)) & 0x7FF;
                receiver_ChData[i+4] = ((receiver_RawData[j+6]  >> 4) | (receiver_RawData[j+7]  << 4)) & 0x7FF;
                receiver_ChData[i+5] = ((receiver_RawData[j+7]  >> 7) | (receiver_RawData[j+8]  << 1) | (receiver_RawData[9] <<  9)) & 0x7FF;
                receiver_ChData[i+6] = ((receiver_RawData[j+9]  >> 2) | (receiver_RawData[j+10] << 6)) & 0x7FF;
                receiver_ChData[i+7] = ((receiver_RawData[j+10] >> 5) | (receiver_RawData[j+11] << 3)) & 0x7FF;
            }

            break;


        case NO_PROTO:
        default:
            return PROTOCOL_ERROR;
            break;
    }
    
    return RECEIVER_OK;
}

/**
 * @brief this function controls the output pwm signals accourding to the receiver input
 * @retval None
 */
void Receiver_MotorControl(void)
{
    // check save mode (one switch)
    uint8_t pwm_MaxDutyCycle = PWM_NORMALMODE_DC_MAX;
    if(receiver_ChData[7] < receiver_Input.half)
        pwm_MaxDutyCycle = PWM_SAFEMODE_DC_MAX;


    double channel[4] = {0.0}; 


    // throttle (up / down)
    float throttle = (float)(receiver_ChData[2] - receiver_Input.min) / receiver_Input.delta;   // get joystick position
    throttle *= pwm_MaxDutyCycle;                               // get percent of max duty cycle addition
    for(uint8_t i = 0; i < 4; i++)
        channel[i] = throttle;


    // pitch (forward / backwards)
    float pitch = (float)(receiver_ChData[1] - receiver_Input.min) / receiver_Input.delta;      // get joystick position
    pitch = (pitch < .5) ? (.5 - pitch) * 2 : (pitch - .5) * 2; // get difference from 50%
    pitch *= PWM_TURN_SPEED_MAX;                                // get percent of max duty cycle addition
    
    // flying backwards, front motors faster
    if(receiver_ChData[1] < receiver_Input.half)
    {
        channel[0] += pitch;
        channel[1] += pitch;
    }
    // flying forward, rear motors faster
    else
    {
        channel[2] += pitch;
        channel[3] += pitch;
    }


    // roll (left / right)
    float roll = (float)(receiver_ChData[3] - receiver_Input.min) / receiver_Input.delta;       // get joystick position
    roll = (roll < .5) ? (.5 - roll) * 2 : (roll - .5) * 2; // get difference from 50%
    roll *= PWM_TURN_SPEED_MAX;                             // get percent of max duty cycle addition

    // flying left, right motors faster
    if(receiver_ChData[3] < receiver_Input.half)
    {
        channel[1] += roll;
        channel[3] += roll;
    }
    // flying right, left motors faster
    else
    {
        channel[0] += roll;
        channel[2] += roll;
    }


    // yaw (rotate left / rotate right)
    float yaw = (float)(receiver_ChData[0] - receiver_Input.min) / receiver_Input.delta;       // get joystick position
    yaw = (yaw < .5) ? (.5 - yaw) * 2 : (yaw - .5) * 2;     // get difference from 50%
    yaw *= PWM_TURN_SPEED_MAX;                              // get percent of max duty cycle addition

    // rotate left, right front and left rear motors faster
    if(receiver_ChData[0] < receiver_Input.half)
    {
        channel[1] += yaw;
        channel[2] += yaw;
    }
    // rotate right, left front and right rear motors faster
    else
    {
        channel[0] += yaw;
        channel[3] += yaw;
    }


    // MAYBE hover mode


    // check if the value is larger then the max value
    for(uint8_t i = 0; i < 4; i++)
        if(channel[i] > throttle + PWM_TURN_SPEED_MAX)
            channel[i] = throttle + PWM_TURN_SPEED_MAX;

    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_1, (uint16_t)(channel[0] * 10));
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_2, (uint16_t)(channel[1] * 10));
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_3, (uint16_t)(channel[2] * 10));
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_4, (uint16_t)(channel[3] * 10));

}

