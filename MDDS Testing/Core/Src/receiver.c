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
 */

#include "receiver.h"

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

Receiver_Protocol protocol = NO_PROTO; // selected serial protocol
uint8_t receiver_RawData[32] = {0}; // raw data of receiver communication
uint16_t receiver_ChData[16] = {0}; // each channel data

/************************************************************************************************
------------------------------------------- FUNCTIONS -------------------------------------------
************************************************************************************************/

/**
 * @brief This function calibrates and starts uart receive dma with selected protocol 
 * @param protocol protocol to use (SBUS / IBUS)
 * @param huart pointer to a UART_HandleTypeDef structure
 * @return Receiver_Status 
 */
Receiver_Status Receiver_Init(Receiver_Protocol proto, UART_HandleTypeDef *huart)
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

            uint8_t tmp[2] = {0};
        
            // calibrate reception to begin of protocol
            while(!(tmp[0] == 0x20 && tmp[1] == 0x40))
                HAL_UART_Receive(huart, tmp, 2, 4);

            HAL_Delay(4);

            // start DMA read i.bus signal
            if(HAL_UART_Receive_DMA(huart, receiver_RawData, 32) != HAL_OK)
                return IBUS_ERROR;

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

            uint8_t tmp = 0;

            // calibrate reception to begin of protocol
            while(tmp != 0x0F)
                HAL_UART_Receive(huart, &tmp, 1, 4);

            HAL_Delay(4);

            // start DMA read s.bus signal
            if(HAL_UART_Receive_DMA(huart, receiver_RawData, 25) != HAL_OK)
                return SBUS_ERROR;

            break;
        }

        // wrong or no protocol selected
        case NO_PROTO:
        default:
            return PROTOCOL_ERROR;           
            break;
    }

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

