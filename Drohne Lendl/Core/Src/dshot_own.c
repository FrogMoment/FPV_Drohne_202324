/**
 * @file dshot_own.c
 * @author Maximilian Lendl
 * @date 2023-12-01
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief This file provides functions for:
 *          - DShot600 output init 
 *          - DShot600 output special command 
 *          - DShot600 output throttle 
 *          - DShot600 ESC / motor test 
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
 * @param protocol DSHOT150, DSHOT300, DSHOT600 or PWM
 * @return DShot_Status 
 */
DShot_Status DShot_Init(TIM_HandleTypeDef *htim, ESC_OutputProtocol protocol)
{
    // set and check timer pointer
    DShot_OutputTim = htim;
    if(DShot_OutputTim == NULL)
        return DSHOT_TIM_ERROR;

    if(protocol == PWM)
    {
        __HAL_TIM_SET_PRESCALER(DShot_OutputTim, 558 - 1);
        __HAL_TIM_SET_AUTORELOAD(DShot_OutputTim, protocol - 1);
        __HAL_TIM_SET_COMPARE(DShot_OutputTim, TIM_CHANNEL_1, protocol * 0.05);
        __HAL_TIM_SET_COMPARE(DShot_OutputTim, TIM_CHANNEL_2, protocol * 0.05);
        __HAL_TIM_SET_COMPARE(DShot_OutputTim, TIM_CHANNEL_3, protocol * 0.05);
        __HAL_TIM_SET_COMPARE(DShot_OutputTim, TIM_CHANNEL_4, protocol * 0.05);
    }
    else
    {
        __HAL_TIM_SET_PRESCALER(DShot_OutputTim, 1 - 1);
        __HAL_TIM_SET_AUTORELOAD(DShot_OutputTim, protocol - 1);

        oneDC = protocol * 0.74850;   // calculate DC for sending bit 1
        zeroDC = protocol * 0.37425;  // calculate DC for sending bit 0

        // set custom transfer complete ISR
        DShot_OutputTim->hdma[ESC_LF_DMA_ID]->XferCpltCallback = Dshot_DMA_XferCpltCallback;
        DShot_OutputTim->hdma[ESC_RF_DMA_ID]->XferCpltCallback = Dshot_DMA_XferCpltCallback;
        DShot_OutputTim->hdma[ESC_LR_DMA_ID]->XferCpltCallback = Dshot_DMA_XferCpltCallback;
        DShot_OutputTim->hdma[ESC_RR_DMA_ID]->XferCpltCallback = Dshot_DMA_XferCpltCallback;
    }

    HAL_TIM_PWM_Start(DShot_OutputTim, ESC_LF_TIM_CH);
  	HAL_TIM_PWM_Start(DShot_OutputTim, ESC_RF_TIM_CH);
	HAL_TIM_PWM_Start(DShot_OutputTim, ESC_LR_TIM_CH);
	HAL_TIM_PWM_Start(DShot_OutputTim, ESC_RR_TIM_CH);

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
void DShot_SendThrottle(double motorLF, double motorRF, double motorLR, double motorRR)
{
    // get throttle integar value
    uint16_t throttle[4];
    throttle[0] = 48 + 20 * round(motorLF);
    throttle[1] = 48 + 20 * round(motorRF);
    throttle[2] = 48 + 20 * round(motorLR);
    throttle[3] = 48 + 20 * round(motorRR);

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

    for(int8_t i = 0; i < 4; i++)
    {
        // first 12 bits (without Checksum)
        withoutCS = (throttle[i] << 1) | telemetry;
        
        // format whole data frame
        complete = withoutCS << 4 | (withoutCS ^ (withoutCS >> 4) ^ (withoutCS >> 8)) & 0x0F;

        sprintf(txt, "%d\n\r", throttle[i]);
        HAL_UART_Transmit(&huart4, (uint8_t*)&txt, strlen(txt), HAL_MAX_DELAY);     

        div = 0x8000;
        for(int8_t j = 0; j < 16; j++)
        {
            data[i][j] = (complete & div) ? oneDC : zeroDC;
            div >>= 1; 
        }
    }
    HAL_UART_Transmit(&huart4, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);     

    HAL_DMA_Start_IT(DShot_OutputTim->hdma[ESC_LF_DMA_ID], (uint32_t)&data[0][0], ESC_TIM_GET_CCR_ADDR(ESC_LF_TIM_CH), 18);
	HAL_DMA_Start_IT(DShot_OutputTim->hdma[ESC_RF_DMA_ID], (uint32_t)&data[1][0], ESC_TIM_GET_CCR_ADDR(ESC_RF_TIM_CH), 18);
	HAL_DMA_Start_IT(DShot_OutputTim->hdma[ESC_LR_DMA_ID], (uint32_t)&data[2][0], ESC_TIM_GET_CCR_ADDR(ESC_LR_TIM_CH), 18);
	HAL_DMA_Start_IT(DShot_OutputTim->hdma[ESC_RR_DMA_ID], (uint32_t)&data[3][0], ESC_TIM_GET_CCR_ADDR(ESC_RR_TIM_CH), 18);

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
    DShot_SendThrottle(0, 0, 0, 0);
    HAL_Delay(1000);
    while(1)
    {
        DShot_SendThrottle(40, 40, 40, 40);
        HAL_Delay(1);
    }
}












