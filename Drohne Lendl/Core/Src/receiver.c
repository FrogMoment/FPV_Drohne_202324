/**
 * @file receiver.c
 * @author Maximilian Lendl
 * @date 2023-07-29
 * @version 1
 *
 * @copyright FPV Drohne DA 202324
 *
 * @brief This file provides functions for:
 *          - receiver init for S.Bus or I.Bus
 *          - S.Bus / I.Bus reception via DMA
 *          - S.Bus / I.Bus decoding
 * 			- failsafe handler
 * 			- reception interrupt control
 */

/**********************************************************************************
------------------------------------ INCLUDES ------------------------------------
**********************************************************************************/

#include "receiver.h"

/**********************************************************************************
-------------------------------- GLOBAL VARIABLES --------------------------------
**********************************************************************************/

 // receiver variables
UART_HandleTypeDef *receiver_InputUART = NULL;          // pointer to UART_HandleTypeDef of input signal
TIM_HandleTypeDef *ppm_Timer = NULL;                    // pointer to TIM_HandleTypeDef of input ppm signal
Receiver_Protocol receiver_SelectedProtocol = NO_PROTO; // selected serial protocol

uint8_t receiver_RawData[32] = {0};     // raw data of receiver communication
uint16_t receiver_ChData[16] = {0};     // each channel data
uint16_t receiver_SameDataCounter = 0;  // counter amount of same channel data after another

// pwm output variables
Receiver_Values receiver_InputLimits = {0}; // input limits dependend on selected protocol

uint8_t failsafeFlag = 0;
uint8_t droneOffModeFlag = 1;

Receiver_Status currentStatus;

/**********************************************************************************
------------------------------------ FUNCTIONS ------------------------------------
**********************************************************************************/

/**
 * @brief This function calibrates and starts uart receive dma with selected protocol
 * @param proto protocol to use (SBUS / IBUS)
 * @param huart pointer to a UART_HandleTypeDef structure (input u(s)art)
 * @return Receiver_Status
 */
Receiver_Status Receiver_Init(Receiver_Protocol proto, UART_HandleTypeDef *huart)
{
	receiver_InputUART = huart;         // set input uart
	receiver_SelectedProtocol = proto;  // set serial protocol

	// set custom reception complete ISR
	HAL_UART_RegisterCallback(receiver_InputUART, HAL_UART_RX_COMPLETE_CB_ID, Receiver_ReceptionCallback);

	switch(receiver_SelectedProtocol)
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
			uint8_t timeout = 0;
			int8_t tmp_PinState = HAL_GPIO_ReadPin(RECEIVER_PPM_GPIO_Port, RECEIVER_PPM_Pin);
			while(tmp_PinState == HAL_GPIO_ReadPin(RECEIVER_PPM_GPIO_Port, RECEIVER_PPM_Pin))
			{
				// if the ppm signal doesn't change in 10ms -> error
				if(timeout++ > 10)
					return RECEIVER_PPM_ERROR;
				HAL_Delay(1);
			}

			uint8_t tmp[2] = {0};
			timeout = 0;

			// calibrate reception to begin of protocol
			while(!(tmp[0] == 0x20 && tmp[1] == 0x40))
			{
				// if the header is wrong 100x -> error
				if(timeout++ > 100)
					return RECEIVER_TIMEOUT;

				HAL_UART_Receive(receiver_InputUART, tmp, 2, 3);
			}
			HAL_Delay(4); // wait to sync to next data packet

			// start DMA read i.bus signal
			if(HAL_UART_Receive_DMA(receiver_InputUART, receiver_RawData, 32) != HAL_OK)
				return IBUS_ERROR;

			// set min/max values of receiver input data
			receiver_InputLimits.min = 1070;
			receiver_InputLimits.max = 1920;
			break;
		}

		/**
		 * 100000 baud
		 * 9 data bits, 2 stop bit, even parity
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

			// check if transmitter is connected (ppm signal reception) 
			uint8_t timeout = 0;
			int8_t tmp_PinState = HAL_GPIO_ReadPin(RECEIVER_PPM_GPIO_Port, RECEIVER_PPM_Pin);
			while(tmp_PinState == HAL_GPIO_ReadPin(RECEIVER_PPM_GPIO_Port, RECEIVER_PPM_Pin))
			{
				// if the ppm signal doesn't change in 10ms -> error
				if(timeout++ > 10)
					return RECEIVER_PPM_ERROR;
				HAL_Delay(1);
			}

			uint8_t tmp = 0;
			timeout = 0;

			// calibrate reception to begin of protocol
			while(tmp != 0x0F)
			{
				// if the header is wrong 100x -> error
				if(timeout++ > 100)
					return RECEIVER_TIMEOUT;

				HAL_UART_Receive(receiver_InputUART, &tmp, 1, 4);
			}
			HAL_Delay(4); // wait to sync to next data packet

			// start DMA read s.bus signal
			if(HAL_UART_Receive_DMA(receiver_InputUART, receiver_RawData, 25) != HAL_OK)
				return SBUS_ERROR;

			// set min/max values of receiver input data
			receiver_InputLimits.min = 350;
			receiver_InputLimits.max = 1680;
			break;
		}

		// wrong or no protocol selected
		case NO_PROTO:
		default:
			return PROTOCOL_ERROR;
			break;
	}

	// set value range and half value
	receiver_InputLimits.delta = receiver_InputLimits.max - receiver_InputLimits.min;
	receiver_InputLimits.half = (receiver_InputLimits.max + receiver_InputLimits.min) / 2;

	return RECEIVER_OK;
}

/**
 * @brief This function decodes the receiver raw data depending on the protocol
 * @details
 * i.bus channel values from 1070 - 1920
 * s.bus channel values from 350 - 1680
 *
 * what channel does what depends on the defines found in receiver.h:
 *  - RECEIVER_YAW_CHANNEL
 *  - RECEIVER_PITCH_CHANNEL
 *  - RECEIVER_THROTTLE_CHANNEL
 *  - RECEIVER_ROLL_CHANNEL
 *  - RECEIVER_ONOFF_SWITCH_CHANNEL
 *  - RECEIVER_MODESEL_SWTICH_CHANNEL
 * @return Receiver_Status
 */
Receiver_Status Receiver_Decode(void)
{
	switch(receiver_SelectedProtocol)
	{
		case IBUS:
		{
			// if reception input start at the last byte -> reorder for correct order (rotate left)
			if(receiver_RawData[1] == 0x20 && receiver_RawData[2] == 0x40)
			{
				uint8_t tmp = receiver_RawData[0];
				for(int8_t i = 1; i < 32; i++)
					receiver_RawData[i - 1] = receiver_RawData[i];
				receiver_RawData[31] = tmp;
			}

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
			Receiver_IBusFailsafeCheck();
			if(receiver_SameDataCounter > 250)
				return IBUS_SIGNAL_LOST_ERROR;

			break;
		}


		case SBUS:
		{
			// if reception input start at the last byte -> reorder for correct order (rotate left)
			if(receiver_RawData[0] == 0x00 && receiver_RawData[1] == 0x0F)
			{
				uint8_t tmp = receiver_RawData[0];
				for(int8_t i = 1; i < 25; i++)
					receiver_RawData[i - 1] = receiver_RawData[i];
				receiver_RawData[24] = tmp;
			}

			// check if protocol header is correct
			if(receiver_RawData[0] != 0x0F)
				return SBUS_HEADER_ERROR;

			// check if protocol footer is correct
			if(receiver_RawData[24] != 0x00)
				return SBUS_FOOTER_ERROR;

			// check signal lost flags
			if(receiver_RawData[23] & 0x04)
				return SBUS_SIGNAL_LOST;

			// check signal failsafe flag
			if(receiver_RawData[23] & 0x08)
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
 * @brief This function convert the input from the receiver to throttle percentage
 * @details
 * The max throttle values per mode can be changed in receiver.h with:
 *  - ESC_SAFEMODE_THR_MAX
 *  - ESC_NORMALMODE_THR_MAX
 *  - ESC_OFFMODE_THR
 *  - ESC_TURN_OFFSET_MAX
 * @return None
 */
void Receiver_ConvertInput(void)
{
	__HAL_TIM_SET_COMPARE(LED_TIM, LED_RED_CHANNEL, 0);

	float throttle = 0, pitch = 0, roll = 0, yaw = 0;


	/**************************************************************************************************************************************
	-------------------------------------------------------- check ON / OFF switch --------------------------------------------------------
	***************************************************************************************************************************************/
	// top position (< half) = off (set standard throttle)
	if(receiver_ChData[RECEIVER_ONOFF_SWITCH_CHANNEL] < receiver_InputLimits.half)
	{
		droneOffModeFlag = 1;
		PID_Normal(0, 0, 0, 0);
		return;
	}

	if(droneOffModeFlag == 1 && receiver_ChData[RECEIVER_THROTTLE_CHANNEL] > receiver_InputLimits.min + 10)
	{
		PID_Normal(0, 0, 0, 0);
		return;
	}

	droneOffModeFlag = 0;


	/**************************************************************************************************************************************
	------------------------------------------------ check 3 position switch (mode select) ------------------------------------------------
	***************************************************************************************************************************************/
	uint16_t esc_MaxThr = ESC_SAFEMODE_THR_MAX;
	uint8_t hoverModeFlag = 0;

	// top position (< half) = safemode
	if(receiver_ChData[RECEIVER_MODESEL_SWTICH_CHANNEL] < receiver_InputLimits.half - 10)
		esc_MaxThr = ESC_SAFEMODE_THR_MAX;

	// middle position (half +- 10) = normalmode
	else if(receiver_ChData[RECEIVER_MODESEL_SWTICH_CHANNEL] >= receiver_InputLimits.half - 10 && receiver_ChData[RECEIVER_MODESEL_SWTICH_CHANNEL] <= receiver_InputLimits.half + 10)
		esc_MaxThr = ESC_NORMALMODE_THR_MAX;

	// down position = extra mode hover mode
	else
		hoverModeFlag = 1;


	/**************************************************************************************************************************************
	------------------------------------------------ calculate throttle input (up / down) ------------------------------------------------
	***************************************************************************************************************************************/
	throttle = (float)(receiver_ChData[RECEIVER_THROTTLE_CHANNEL] - receiver_InputLimits.min) / receiver_InputLimits.delta; // get joystick position
	throttle *= esc_MaxThr; // get percent of max duty cycle addition

	if(hoverModeFlag == 1)
	{
		PID_Hover(throttle);
		return;
	}


	/**************************************************************************************************************************************
	-------------------------------------------- calculate pitch input (forwards / backwards) --------------------------------------------
	***************************************************************************************************************************************/
	pitch = (float)(receiver_ChData[RECEIVER_PITCH_CHANNEL] - receiver_InputLimits.min) / receiver_InputLimits.delta; // get joystick position
	pitch = (pitch - 0.5f) * 2;
	pitch *= ESC_TURN_OFFSET_MAX; // get percent of max duty cycle addition


	/**************************************************************************************************************************************
	------------------------------------------------- calculate roll input (left / right) -------------------------------------------------
	***************************************************************************************************************************************/
	roll = (float)(receiver_ChData[RECEIVER_ROLL_CHANNEL] - receiver_InputLimits.min) / receiver_InputLimits.delta; // get joystick position
	roll = (roll - 0.5f) * 2;
	roll *= ESC_TURN_OFFSET_MAX; // get percent of max duty cycle addition


	/**************************************************************************************************************************************
	------------------------------------------ calculate yaw input (rotate left / rotate right) ------------------------------------------
	***************************************************************************************************************************************/
	yaw = (float)(receiver_ChData[RECEIVER_YAW_CHANNEL] - receiver_InputLimits.min) / receiver_InputLimits.delta; // get joystick position
	yaw = (yaw - 0.5f) * 2;


	/**************************************************************************************************************************************
	-------------------------------------------------------- check and send values --------------------------------------------------------
	***************************************************************************************************************************************/
	PID_Normal(throttle, pitch, roll, yaw);
}

/**
 * @brief This function outputs all receiver channels side by side
 * @attention This function uses the global array receiver_ChData[]
 * @retval None
 */
void Receiver_OutputChValues(void)
{
	char tmp[100], finalString[1000];

	// get amount of channels per protocol
	int8_t len = (receiver_SelectedProtocol == IBUS) ? 14 : 16;

	// convert channel data to string
	for(int8_t i = 0; i < len; i++)
	{
		sprintf(tmp, "%d  ", receiver_ChData[i]);
		strcat(finalString, tmp);
	}
	strcat(finalString, "\n\r");

	// output string to terminal
	Terminal_Print(finalString);
}

/**
 * @brief This function sets the drone motors to a throttle that slowly lands the drone
 * @retval Receiver_Status
 */
Receiver_Status Receiver_FailsafeHandler(void)
{
	PID_Normal(0, 0, 0, 0);
	failsafeFlag = 1;

	// TODO hover mode with a little less then take off throttle
	// something like that 
	// PID_Hover(30);

	__HAL_TIM_SET_COMPARE(LED_TIM, LED_RED_CHANNEL, 5000);

	return RECEIVER_OK;
}

/**
 * @brief This function saves the current channel data and check if its the same as before
 * @param huart pointer to UART_HandleTypeDef
 * @retval None
 */
void Receiver_IBusFailsafeCheck(void)
{
	// only used for IBUS because SBUS does have a signal lost / failsafe flag
	if(receiver_SelectedProtocol != IBUS)
		return;

	static uint16_t receiver_OldChData[16] = {0};   // previous channel data

	// when off -> don't check
	if(receiver_ChData[RECEIVER_ONOFF_SWITCH_CHANNEL] < receiver_InputLimits.half)
	{
		receiver_SameDataCounter = 0; // reset counter
	}
	else
	{
		int8_t same = 1; // data is same flag
		for(int8_t i = 0; i < 14 && same == 1; i++)
		{
			// check if the old channel data is not the same as the current
			if(receiver_OldChData[i] != receiver_ChData[i])
			{
				receiver_SameDataCounter = 0; // reset channel data check
				same = 0;
			}

			// increment receiver_SameDataCounter when data is the same
			else if(i == 14 - 1)
				(receiver_SameDataCounter == UINT16_MAX - 10) ? receiver_SameDataCounter = 260 : receiver_SameDataCounter++;
		}
	}

	// save current channel data
	for(int8_t i = 0; i < 14; i++)
		receiver_OldChData[i] = receiver_ChData[i];
}

/**
 * @brief This function is the ISR for DMA receiver reception complete
 * @details all data gets decoded, the motor speed gets regulated and data gets transmitted
 * @param huart
 */
void Receiver_ReceptionCallback(UART_HandleTypeDef *huart)
{
	/************************************************************************************************
	--------------------------------------- RECEIVER + OUTPUT ---------------------------------------
	************************************************************************************************/
	uint8_t errorCode;
	errorCode = Receiver_Decode();

	if(errorCode != RECEIVER_OK)
	{
		if(errorCode == IBUS_SIGNAL_LOST_ERROR || errorCode == SBUS_SIGNAL_LOST || errorCode == SBUS_SIGNAL_FAILSAFE)
			Receiver_FailsafeHandler();

		sprintf(txt, "Receiver Error %d\n\r", errorCode);
		Terminal_Print(txt);
	}
	else
	{
		failsafeFlag = 0;
		Receiver_ConvertInput();
	}


	/************************************************************************************************
	---------------------------------------------- IMU ----------------------------------------------
	************************************************************************************************/
	IMU_GetAngles();
	IMU_BARO_ReadBaro();


	/************************************************************************************************
	--------------------------------------- data transmission ---------------------------------------
	************************************************************************************************/
	static int8_t dataTransmitDelay = 0;

	if(dataTransmitDelay++ >= 60)
	{
		static int8_t packetSelect = 0;

		if(packetSelect == 0)
			DATA_TRANSMISSION_1(ds2438_Voltage, baroAltitude, 0x00);
		else
			DATA_TRANSMISSION_2(angle.pitch, angle.roll, angle.yaw);

		packetSelect = packetSelect == 0;
		dataTransmitDelay = 0;
	}
}




