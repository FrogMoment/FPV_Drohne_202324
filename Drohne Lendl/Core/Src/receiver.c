/**
 * @file receiver.c
 * @author Maximilian Lendl
 * @date 2023-07-29
 * @version 1
 *
 * @copyright FPV Drohne DA 202324
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
UART_HandleTypeDef *receiver_InputUART = NULL;
Receiver_Protocol protocol = NO_PROTO;  // selected serial protocol
uint8_t receiver_RawData[32] = {0};     // raw data of receiver communication
uint16_t receiver_ChData[16] = {0};     // each channel data
TIM_HandleTypeDef *ppm_Timer;           // pointer to TIM_HandleTypeDef of input ppm signal

// pwm output variables
TIM_HandleTypeDef *pwm_Timer = NULL;    // pointer to TIM_HandleTypeDef of output pwm signal
Receiver_Values receiver_Input = {0};   // max values of receiver input

uint16_t receiver_OldChData[16] = {0};  // previous channel data
uint16_t receiver_ChDataCheck = 0;        // counter amount of same channel data after another


/************************************************************************************************
------------------------------------------- FUNCTIONS -------------------------------------------
************************************************************************************************/

/**
 * @brief This function calibrates and starts uart receive dma with selected protocol
 * @param protocol protocol to use (SBUS / IBUS)
 * @param huart pointer to a UART_HandleTypeDef structure (input usart)
 * @param htim_out pointer to a TIM_HandleTypeDef structure (output pwm timer)
 * @return Receiver_Status
 */
Receiver_Status Receiver_Init(Receiver_Protocol proto, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim_out)
{
    receiver_InputUART = huart; // set input uart
    protocol = proto;           // set serial protocol
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
            if(receiver_InputUART->Init.BaudRate != 115200)
                return RECEIVER_UART_ERROR;

            // check if trnasmitter is connected (ppm signal reception) 
            uint16_t timeout = 0;
            int8_t tmp_PinState = HAL_GPIO_ReadPin(RECEIVER_PPM_GPIO_Port, RECEIVER_PPM_Pin);
            while(tmp_PinState == HAL_GPIO_ReadPin(RECEIVER_PPM_GPIO_Port, RECEIVER_PPM_Pin))
            {
                if(timeout++ > 10)
                    return RECEIVER_PPM_ERROR;
                HAL_Delay(1);
            }

            uint8_t tmp[2] = {0};
            timeout = 0;

            // calibrate reception to begin of protocol
            while(!(tmp[0] == 0x20 && tmp[1] == 0x40))
            {
                HAL_UART_Receive(receiver_InputUART, tmp, 2, 4);
                if(timeout++ > 100)
                    return RECEIVER_TIMEOUT;
            }
            HAL_Delay(4);

            // start DMA read i.bus signal
            if(HAL_UART_Receive_DMA(receiver_InputUART, receiver_RawData, 32) != HAL_OK)
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
            if(receiver_InputUART->Init.BaudRate != 100000)
                return RECEIVER_UART_ERROR;

            uint8_t tmp = 0, timeout = 0;

            // calibrate reception to begin of protocol
            while(tmp != 0x0F)
            {
                HAL_UART_Receive(receiver_InputUART, &tmp, 1, 4);
                if(timeout++ > 100)
                    return RECEIVER_TIMEOUT;
            }
            HAL_Delay(4);

            // start DMA read s.bus signal
            if(HAL_UART_Receive_DMA(receiver_InputUART, receiver_RawData, 25) != HAL_OK)
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

    // set std duty cycle (0.1%) and start timer pwm
    pwm_Timer = htim_out;
    Receiver_SetStdDC();

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
 * Ch1: Yaw (rotate left / rotate right)
 * Ch2: Pitch (forwards / backwards)
 * Ch3: Throttle (up / down)
 * Ch4: Roll (left / right)
 * Ch5: on/off switch
 * Ch6: mode select (3 way switch)
 * Ch7: not used
 * Ch8: not used
 * @return Receiver_Status
 */
Receiver_Status Receiver_Decode(void)
{
    switch(protocol)
    {
        case IBUS:
        {
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
            for(int8_t i = 0, j = 0; i < 14; i++, j += 2)
                receiver_ChData[i] = (receiver_RawData[j + 3] << 8) | receiver_RawData[j + 2];

            // check disconnection
            if(receiver_ChDataCheck > 250)
                return IBUS_SIGNAL_LOST_ERROR;

            break;
        }


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
            for(int8_t i = 0, j = 0; i < 16; i += 8, j += 11)
            {
                receiver_ChData[i + 0] = ((receiver_RawData[j + 1] >> 0) | (receiver_RawData[j + 2] << 8)) & 0x7FF;
                receiver_ChData[i + 1] = ((receiver_RawData[j + 2] >> 3) | (receiver_RawData[j + 3] << 5)) & 0x7FF;
                receiver_ChData[i + 2] = ((receiver_RawData[j + 3] >> 6) | (receiver_RawData[j + 4] << 2) | (receiver_RawData[5] << 10)) & 0x7FF;
                receiver_ChData[i + 3] = ((receiver_RawData[j + 5] >> 1) | (receiver_RawData[j + 6] << 7)) & 0x7FF;
                receiver_ChData[i + 4] = ((receiver_RawData[j + 6] >> 4) | (receiver_RawData[j + 7] << 4)) & 0x7FF;
                receiver_ChData[i + 5] = ((receiver_RawData[j + 7] >> 7) | (receiver_RawData[j + 8] << 1) | (receiver_RawData[9] << 9)) & 0x7FF;
                receiver_ChData[i + 6] = ((receiver_RawData[j + 9] >> 2) | (receiver_RawData[j + 10] << 6)) & 0x7FF;
                receiver_ChData[i + 7] = ((receiver_RawData[j + 10] >> 5) | (receiver_RawData[j + 11] << 3)) & 0x7FF;
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
 * @brief this function controls the output pwm output signals according to the receiver input
 * @return Receiver_Status
 */
Receiver_Status Receiver_MotorControl(void)
{
    Receiver_Status status = Receiver_Decode();
    if(status != RECEIVER_OK)
        return status;

    // check on/off switch
    if(receiver_ChData[ONOFF_SWITCH_CHANNEL] < receiver_Input.half)
        return Receiver_SetStdDC();

    // check mode select (3 way switch)
    uint16_t pwm_MaxDutyCycle;
    if(receiver_ChData[MODESEL_SWTICH_CHANNEL] < receiver_Input.half - 10)
        pwm_MaxDutyCycle = PWM_SAFEMODE_DC_MAX;
    else if(receiver_ChData[MODESEL_SWTICH_CHANNEL] >= receiver_Input.half - 10 && receiver_ChData[MODESEL_SWTICH_CHANNEL] <= receiver_Input.half + 10)
        pwm_MaxDutyCycle = PWM_NORMALMODE_DC_MAX;
    else
        pwm_MaxDutyCycle = PWM_NORMALMODE_DC_MAX; // MAYBE third flight mode select


    Motor_Position motor;


    // throttle (up / down)
    float throttle = (float)(receiver_ChData[THROTTLE_CHANNEL] - receiver_Input.min) / receiver_Input.delta;   // get joystick position
    throttle *= pwm_MaxDutyCycle;   // get percent of max duty cycle addition
    motor.LF = throttle;            // set motor speed to throttle
    motor.RF = throttle;            // set motor speed to throttle
    motor.LR = throttle;            // set motor speed to throttle
    motor.RR = throttle;            // set motor speed to throttle


    // pitch (forwards / backwards)
    float pitch = (float)(receiver_ChData[PITCH_CHANNEL] - receiver_Input.min) / receiver_Input.delta;      // get joystick position
    pitch = (pitch < .5) ? (.5 - pitch) * 2 : (pitch - .5) * 2; // get difference from 50%
    pitch *= PWM_TURN_OFFSET_MAX;                               // get percent of max duty cycle addition

    // flying backwards, front motors faster
    if(receiver_ChData[PITCH_CHANNEL] < receiver_Input.half)
    {
        motor.LF += pitch;  // add motor speed to left front
        motor.RF += pitch;  // add motor speed to right front
    }
    // flying forward, rear motors faster
    else
    {
        motor.LR += pitch;  // add motor speed to left rear
        motor.RR += pitch;  // add motor speed to right rear
    }


    // roll (left / right)
    float roll = (float)(receiver_ChData[ROLL_CHANNEL] - receiver_Input.min) / receiver_Input.delta;       // get joystick position
    roll = (roll < .5) ? (.5 - roll) * 2 : (roll - .5) * 2; // get difference from 50%
    roll *= PWM_TURN_OFFSET_MAX;                             // get percent of max duty cycle addition

    // flying left, right motors faster
    if(receiver_ChData[ROLL_CHANNEL] < receiver_Input.half)
    {
        motor.RF += roll;   // add motor speed to right front
        motor.RR += roll;   // add motor speed to right rear 
    }
    // flying right, left motors faster
    else
    {
        motor.LF += roll;   // add motor speed to left front 
        motor.LR += roll;   // add motor speed to left rear 
    }


    // yaw (rotate left / rotate right)
    float yaw = (float)(receiver_ChData[YAW_CHANNEL] - receiver_Input.min) / receiver_Input.delta;       // get joystick position
    yaw = (yaw < .5) ? (.5 - yaw) * 2 : (yaw - .5) * 2;     // get difference from 50%
    yaw *= PWM_TURN_OFFSET_MAX;                             // get percent of max duty cycle addition

    // rotate left, right front and left rear motors faster
    if(receiver_ChData[YAW_CHANNEL] < receiver_Input.half)
    {
        motor.RF += yaw;    // add motor speed to right front 
        motor.LR += yaw;    // add motor speed to left rear 
    }
    // rotate right, left front and right rear motors faster
    else
    {
        motor.LF += yaw;    // add motor speed to left front 
        motor.RR += yaw;    // add motor speed to right rear 
    }


    // check if the value is larger then the max value
    if(motor.LF > throttle + PWM_TURN_OFFSET_MAX) motor.LF = throttle + PWM_TURN_OFFSET_MAX;
    if(motor.RF > throttle + PWM_TURN_OFFSET_MAX) motor.RF = throttle + PWM_TURN_OFFSET_MAX;
    if(motor.LR > throttle + PWM_TURN_OFFSET_MAX) motor.LR = throttle + PWM_TURN_OFFSET_MAX;
    if(motor.RR > throttle + PWM_TURN_OFFSET_MAX) motor.RR = throttle + PWM_TURN_OFFSET_MAX;
    

    /** 
     * change the duty cycle of the pwm signals
     * channel1 = right rear
     * channel2 = right front
     * channel3 = left rear
     * channel4 = left front
     */
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_1, (uint16_t)(motor.RR * 10));
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_2, (uint16_t)(motor.RF * 10));
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_3, (uint16_t)(motor.LR * 10));
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_4, (uint16_t)(motor.LF * 10));

    return RECEIVER_OK;
}

/**
 * @brief This functions sets the duty cycle to the std value 0.1%
 * @return Receiver_Status
 */
Receiver_Status Receiver_SetStdDC(void)
{
    // check if pwm_Timer is set
    if(pwm_Timer == NULL)
        return RECEIVER_PWM_ERROR;

    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_1, (uint16_t)(PWM_OFFMODE_DC));
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_2, (uint16_t)(PWM_OFFMODE_DC));
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_3, (uint16_t)(PWM_OFFMODE_DC));
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_4, (uint16_t)(PWM_OFFMODE_DC));

    return RECEIVER_OK;
}

/**
 * @brief This function output all receiver channels side by side
 * @attention This function uses the global array receiver_ChData[]
 * @param huart pointer to a UART_HandleTypeDef structure (for output)
 * @retval None
 */
void Receiver_OutputChValues(UART_HandleTypeDef *huart)
{
    char txt[100], txt2[1000];

    // get amount of channels per protocol
    int8_t len = (protocol == IBUS) ? 14 : 16;

    // convert channel data to string
    for(int8_t i = 0; i < len; i++)
    {
      sprintf(txt, "%d  ", receiver_ChData[i]);
      strcat(txt2, txt);
    }
    strcat(txt2, "\n\r");

    // output string 
    HAL_UART_Transmit(huart, (uint8_t *)&txt2, strlen(txt2), HAL_MAX_DELAY);
}

/**
 * @brief This function test each motor
 * @param htim pointer to a TIM_HandleTypeDef structure (output pwm timer)
 * @details turn motor 1 for 2 seconds on then next motor etc
 * @retval None
 */
void Receiver_MotorTest(TIM_HandleTypeDef *htim)
{
    // set std duty cycle
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint16_t)(PWM_OFFMODE_DC));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint16_t)(PWM_OFFMODE_DC));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint16_t)(PWM_OFFMODE_DC));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, (uint16_t)(PWM_OFFMODE_DC));
    
    // start pwm output
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
    HAL_Delay(3000);

    // start motortest
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint16_t)(PWM_MOTORTEST_DC));
    HAL_Delay(2000);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint16_t)(PWM_MOTORTEST_DC));
    HAL_Delay(2000);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint16_t)(PWM_MOTORTEST_DC));
    HAL_Delay(2000);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, (uint16_t)(PWM_MOTORTEST_DC));
    HAL_Delay(2000);

    // set std duty cycle
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint16_t)(PWM_OFFMODE_DC));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint16_t)(PWM_OFFMODE_DC));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint16_t)(PWM_OFFMODE_DC));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, (uint16_t)(PWM_OFFMODE_DC));
}

/**
 * @brief This function sets the drone motors under the hover duty cycle -> landing
 * @retval Receiver_Status
 */
Receiver_Status Receiver_SignalLostHandler(void)
{
    // check if pwm_Timer is set
    if(pwm_Timer == NULL)
        return RECEIVER_PWM_ERROR;

    // set motors to less then hover mode -> slowing flying downward
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_1, (uint16_t)(PWM_HOVER_DC - PWM_SIGNAL_LOST_OFFSET));
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_2, (uint16_t)(PWM_HOVER_DC - PWM_SIGNAL_LOST_OFFSET));
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_3, (uint16_t)(PWM_HOVER_DC - PWM_SIGNAL_LOST_OFFSET));
    __HAL_TIM_SET_COMPARE(pwm_Timer, TIM_CHANNEL_4, (uint16_t)(PWM_HOVER_DC - PWM_SIGNAL_LOST_OFFSET));

    // MAYBE check IMU for stability

    return RECEIVER_OK;
}

/**
 * @brief This function saves the current channel data and check if its the same as before
 * @retval None
 */
void Receiver_SaveChannelData(void)
{
    // TODO Fehlerbehebung (einmal error 9 dann immer) (löst nicht richtig aus)

    if(protocol != IBUS)
        return;

    if(receiver_ChData[ONOFF_SWITCH_CHANNEL] < receiver_Input.half)
    {
        receiver_ChDataCheck = 0;
        goto save;
    }

    for(int8_t i = 0; i < 14; i++)
    {
        // check if the old channel data is not the same as the current
        if(receiver_OldChData[i] != receiver_ChData[i])
        {    
            receiver_ChDataCheck = 0; // reset channel data check
            goto save;
        }
        else if(i == 14 - 1)
            receiver_ChDataCheck = (receiver_ChDataCheck == UINT16_MAX - 10) ? 260 : receiver_ChDataCheck + 1; // increment channel data check
    }

    // save current channel data
    save: 
    for(int8_t i = 0; i < 14; i++)
        receiver_OldChData[i] = receiver_ChData[i];
}
