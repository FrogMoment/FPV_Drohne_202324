/**
 * @file dshot_own.c
 * @author Maximilian Lendl
 * @date 2023-12-01
 * @version 1
 *
 * @copyright FPV Drohne DA 202324
 *
 * @brief This file provides functions for:
 *          - DShot init for DShot150, DShot300 or DShot600
 *          - DShot send throttle value
 *          - DShot send command
 *          - DShot ESC / motor test
 */

#include "dshot_own.h"

/**********************************************************************
--------------------------- GLOBAL VARIABLE ---------------------------
**********************************************************************/

uint16_t oneDC, zeroDC;

TIM_HandleTypeDef *DShot_OutputTim = NULL;

/**********************************************************************
------------------------------ FUNCTIONS ------------------------------
**********************************************************************/

/**
 * @brief This function is the ISR for DMA transmit complete
 * @param hdma
 * @retval None
 */
static void Dshot_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
    // diable DMA to get rid of the delay between channels
    if(hdma == DShot_OutputTim->hdma[TIM_DMA_ID_CC1])
    {
        __HAL_TIM_DISABLE_DMA(DShot_OutputTim, TIM_DMA_CC1);
    }
    else if(hdma == DShot_OutputTim->hdma[TIM_DMA_ID_CC2])
    {
        __HAL_TIM_DISABLE_DMA(DShot_OutputTim, TIM_DMA_CC2);
    }
    else if(hdma == DShot_OutputTim->hdma[TIM_DMA_ID_CC3])
    {
        __HAL_TIM_DISABLE_DMA(DShot_OutputTim, TIM_DMA_CC3);
    }
    else if(hdma == DShot_OutputTim->hdma[TIM_DMA_ID_CC4])
    {
        __HAL_TIM_DISABLE_DMA(DShot_OutputTim, TIM_DMA_CC4);
    }
}

/**
 * @brief This function initializes the output ESC DShot signal
 * @param htim pointer to TIM_HandleTypeDef (output timer)
 * @param protocol DSHOT150, DSHOT300, DSHOT600
 * @return DShot_Status
 */
DShot_Status DShot_Init(TIM_HandleTypeDef *htim, ESC_OutputProtocol protocol)
{
    // set and check timer pointer
    DShot_OutputTim = htim;
    if(DShot_OutputTim == NULL)
        return DSHOT_TIM_ERROR;

    // set the right timer frequency, 279MHz / (prescaler * autoreload)
    __HAL_TIM_SET_PRESCALER(DShot_OutputTim, 1 - 1);
    __HAL_TIM_SET_AUTORELOAD(DShot_OutputTim, protocol - 1);

    oneDC = protocol * 0.74850;   // calculate DC for sending bit 1
    zeroDC = protocol * 0.37425;  // calculate DC for sending bit 0

    // define custom transfer complete ISR
    DShot_OutputTim->hdma[ESC_LF_DMA_ID]->XferCpltCallback = Dshot_DMA_XferCpltCallback;
    DShot_OutputTim->hdma[ESC_RF_DMA_ID]->XferCpltCallback = Dshot_DMA_XferCpltCallback;
    DShot_OutputTim->hdma[ESC_LR_DMA_ID]->XferCpltCallback = Dshot_DMA_XferCpltCallback;
    DShot_OutputTim->hdma[ESC_RR_DMA_ID]->XferCpltCallback = Dshot_DMA_XferCpltCallback;

    // start all timers in pwm output mode
    HAL_TIM_PWM_Start(DShot_OutputTim, ESC_LF_TIM_CH);
    HAL_TIM_PWM_Start(DShot_OutputTim, ESC_RF_TIM_CH);
    HAL_TIM_PWM_Start(DShot_OutputTim, ESC_LR_TIM_CH);
    HAL_TIM_PWM_Start(DShot_OutputTim, ESC_RR_TIM_CH);

    return DSHOT_OK;
}

/**
 * @brief This function formats the motor data for the DShot protocol
 * @param motorLF percent of throttle value of left front motor (0-100)
 * @param motorRF percent of throttle value of right front motor (0-100)
 * @param motorLR percent of throttle value of left rear motor (0-100)
 * @param motorRR percent of throttle value of right rear motor (0-100)
 * @retval None
 */
void DShot_SendThrottle(double motorLF, double motorRF, double motorLR, double motorRR)
{
    /**
     * timer channel 1 = left front motor
     * timer channel 2 = right front motor
     * timer channel 3 = left rear motor
     * timer channel 4 = right rear motor
     */

    // convert to dshot throttle format (48 = 0% throttle, 2047 = 100% throttle)
    uint16_t throttle[4] = {0};
    throttle[0] = 48 + 20 * motorLF;
    throttle[1] = 48 + 20 * motorRF;
    throttle[2] = 48 + 20 * motorLR;
    throttle[3] = 48 + 20 * motorRR;

    DShot_SendData(throttle, 0);
}

/**
 * @brief This function sends a command to the ESC via DShot protocol
 * @param command
 * @retval None
 */
void DShot_SendCommand(DShot_Command command)
{
    // get command integar value
    uint16_t commands[4] = {command, command, command, command};

    DShot_SendData(commands, 0);
}

/**
 * @brief This function formats and sends DShot data via PWM DMA
 * @param throttle Throttle values (0-2047)
 * @param telemetry telemetry request bit
 */
void DShot_SendData(uint16_t *throttle, int8_t telemetry)
{
    uint16_t data[4][18] = {0};
    uint16_t withoutCS, complete, div;

    // format the data to packets
    for(int8_t i = 0; i < 4; i++)
    {
        // first 12 bits (without Checksum)
        withoutCS = (throttle[i] << 1) | telemetry;

        // format whole data frame
        complete = withoutCS << 4 | (withoutCS ^ (withoutCS >> 4) ^ (withoutCS >> 8)) & 0x0F;

        // convert each bit to the specific duty cycle length   
        div = 0x8000;
        for(int8_t j = 0; j < 16; j++)
        {
            data[i][j] = (complete & div) ? oneDC : zeroDC;
            div >>= 1;
        }
    }

    // reset counter to get rid of delay between channels
    __HAL_TIM_SET_COUNTER(DShot_OutputTim, 0);

    // start dma transfer to the capture compare register
    HAL_DMA_Start_IT(DShot_OutputTim->hdma[ESC_LF_DMA_ID], (uint32_t)&data[0][0], ESC_TIM_GET_CCR_ADDR(ESC_LF_TIM_CH), 18);
    HAL_DMA_Start_IT(DShot_OutputTim->hdma[ESC_RF_DMA_ID], (uint32_t)&data[1][0], ESC_TIM_GET_CCR_ADDR(ESC_RF_TIM_CH), 18);
    HAL_DMA_Start_IT(DShot_OutputTim->hdma[ESC_LR_DMA_ID], (uint32_t)&data[2][0], ESC_TIM_GET_CCR_ADDR(ESC_LR_TIM_CH), 18);
    HAL_DMA_Start_IT(DShot_OutputTim->hdma[ESC_RR_DMA_ID], (uint32_t)&data[3][0], ESC_TIM_GET_CCR_ADDR(ESC_RR_TIM_CH), 18);

    // enable dma
    __HAL_TIM_ENABLE_DMA(DShot_OutputTim, TIM_DMA_CC1);
    __HAL_TIM_ENABLE_DMA(DShot_OutputTim, TIM_DMA_CC2);
    __HAL_TIM_ENABLE_DMA(DShot_OutputTim, TIM_DMA_CC3);
    __HAL_TIM_ENABLE_DMA(DShot_OutputTim, TIM_DMA_CC4);
}

/**
 * @brief This function tests the motors
 * @retval None
 */
void DShot_MotorTest(void)
{
    // for 5s send command 0 every 1ms
    for(uint64_t i = 0; i < 5000; i++)
    {
        DShot_SendCommand(0);
        HAL_Delay(1);
    }

    for(uint64_t i = 0; i < 2000; i++)
    {
        DShot_SendThrottle(5,5,5,5);
        HAL_Delay(1);
    }

    while(1)
    {
        DShot_SendThrottle(0,0,0,0);
        HAL_Delay(1);
    }
}







