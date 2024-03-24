/**
 * @file PID.c
 * @author Maximilian Lendl
 * @date 2024-01-22
 * @version 1
 *
 * @copyright FPV Drohne DA 202324
 *
 * @brief This file provides functions for:
 *          - PID algorithm for hover mode
 */

 /**********************************************************************************
 ------------------------------------ INCLUDES ------------------------------------
 **********************************************************************************/

#include "PID.h"
#include <stdlib.h>

 /**********************************************************************************
 -------------------------------- GLOBAL VARIABLES --------------------------------
 **********************************************************************************/

#define PID_MAX_TURN 15

 /**
	* [0] ... KP Roll
	* [1] ... KI Roll
	* [2] ... KD Roll
	*
	* [3] ... KP Pitch
	* [4] ... KI Pitch
	* [5] ... KD Pitch
	*
	* [6] ... KP Yaw
	* [7] ... KI Yaw
	* [8] ... KD Yaw
	*/
double PID_AllKs[9] = {
	0.120000, 0.006000, 0.001000,
	0.150000, 0.006000, 0.001000,
	0.100000, 0.000000, 0.000000
};

char receiveData[10];

/**********************************************************************************
------------------------------------ FUNCTIONS ------------------------------------
**********************************************************************************/

/**
 * @brief This function initializes the change Ks system
 * @param huart pointer to UART_HandleTypeDef (input uart)
 * @retval None
 */
void PID_Init(UART_HandleTypeDef *huart)
{
	// start reception with uart
	HAL_UART_RegisterCallback(huart, HAL_UART_RX_COMPLETE_CB_ID, PID_ChangeKs);

	// read 10 Byte (for 10 chars for format "0 0.123456")
	HAL_UART_Receive_DMA(huart, (uint8_t *)&receiveData, 10);
}

/**
 * @brief This function controls the flight PID controller
 * @details
 * The max throttle value can be altered with the define PID_MAX_TURN
 * @param inputThrottle throttle value from joysticks
 * @param inputPitch pitch value from joysticks
 * @param inputRoll roll value from joysticks
 * @param inputYaw yaw value from joysticks
 * @return PID_Status
 */
PID_Status PID_Update(float inputThrottle, float inputPitch, float inputRoll, float inputYaw)
{
	static double I_Roll = 0, errorRollPrev = 0;
	static double I_Pitch = 0, errorPitchPrev = 0;
	static double I_Yaw = 0, errorYawPrev = 0;


	/***********************************************************************************
	----------------------------- get current angles + dt -----------------------------
	***********************************************************************************/
	IMU_GetAngles();


	/***********************************************************************************
	---------------------------------- check off mode ----------------------------------
	***********************************************************************************/
	if(inputThrottle < 5)
	{
		I_Roll = 0;
		errorRollPrev = 0;

		I_Pitch = 0;
		errorPitchPrev = 0;

		I_Yaw = 0;
		errorYawPrev = 0;

		DShot_SendThrottle(0, 0, 0, 0);

		return PID_OK;
	}


	/***********************************************************************************
	--------------------------------- calc roll output ---------------------------------
	***********************************************************************************/
	double errorRoll = -inputRoll - angle.roll;

	I_Roll += (errorRoll * imu_DeltaTime) * PID_AllKs[1];

	// limit I value to +/- max throttle addition
	if(I_Roll > PID_MAX_TURN) I_Roll = PID_MAX_TURN;
	else if(I_Roll < -PID_MAX_TURN) I_Roll = -PID_MAX_TURN;

	double rollOutput = PID_AllKs[0] * errorRoll + I_Roll + PID_AllKs[2] * ((errorRoll - errorRollPrev) / imu_DeltaTime);

	// limit output value to +/- max throttle addition
	if(rollOutput > PID_MAX_TURN) rollOutput = PID_MAX_TURN;
	else if(rollOutput < -PID_MAX_TURN) rollOutput = -PID_MAX_TURN;

	errorRollPrev = errorRoll;


	/***********************************************************************************
	-------------------------------- calc pitch output --------------------------------
	***********************************************************************************/
	double errorPitch = inputPitch - angle.pitch;

	I_Pitch += (errorPitch * imu_DeltaTime) * PID_AllKs[4];

	// limit I value to +/- max throttle addition
	if(I_Pitch > PID_MAX_TURN) I_Pitch = PID_MAX_TURN;
	else if(I_Pitch < -PID_MAX_TURN) I_Pitch = -PID_MAX_TURN;

	double pitchOutput = PID_AllKs[3] * errorPitch + I_Pitch + PID_AllKs[5] * ((errorPitch - errorPitchPrev) / imu_DeltaTime);

	// limit output value to +/- max throttle addition
	if(pitchOutput > PID_MAX_TURN) pitchOutput = PID_MAX_TURN;
	else if(pitchOutput < -PID_MAX_TURN) pitchOutput = -PID_MAX_TURN;

	errorPitchPrev = errorPitch;


	/***********************************************************************************
	--------------------------------- calc yaw output ---------------------------------
	***********************************************************************************/
	static double controlYaw = 0;
	static int8_t changeYawFlag = 1;

	double yawOutput = inputYaw * PID_MAX_TURN;

	if(yawOutput >= -1 && yawOutput <= 1)
	{
		if(changeYawFlag == 1)
			controlYaw = angle.yaw;

		changeYawFlag = 0;

		double errorYaw = controlYaw - angle.yaw;

		I_Yaw += (errorYaw * imu_DeltaTime) * PID_AllKs[7];

		// limit I value to +/- max throttle addition
		if(I_Yaw > PID_MAX_TURN) I_Yaw = PID_MAX_TURN;
		else if(I_Yaw < -PID_MAX_TURN) I_Yaw = -PID_MAX_TURN;

		yawOutput = PID_AllKs[6] * errorYaw + I_Yaw + PID_AllKs[8] * ((errorYaw - errorYawPrev) / imu_DeltaTime);

		// limit output value to +/- max throttle addition
		if(yawOutput > PID_MAX_TURN) yawOutput = PID_MAX_TURN;
		else if(yawOutput < -PID_MAX_TURN) yawOutput = -PID_MAX_TURN;

		errorYawPrev = errorYaw;
	}
	else
	{
		changeYawFlag = 1;
	}


	/***********************************************************************************
	-------------------------- check min/max values + output --------------------------
	***********************************************************************************/
	// calc throttle value
	float motorLF = inputThrottle - pitchOutput - rollOutput - yawOutput;
	float motorRF = inputThrottle - pitchOutput + rollOutput + yawOutput;
	float motorLR = inputThrottle + pitchOutput - rollOutput + yawOutput;
	float motorRR = inputThrottle + pitchOutput + rollOutput - yawOutput;

	// if value is less then 5 -> turn motors off to hinder motor tremble
	if(motorLF < 5) motorLF = 0;
	if(motorRF < 5) motorRF = 0;
	if(motorLR < 5) motorLR = 0;
	if(motorRR < 5) motorRR = 0;

	// check if value is higher than max value
	if(motorLF > inputThrottle + PID_MAX_TURN) motorLF = inputThrottle + PID_MAX_TURN;
	if(motorRF > inputThrottle + PID_MAX_TURN) motorRF = inputThrottle + PID_MAX_TURN;
	if(motorLR > inputThrottle + PID_MAX_TURN) motorLR = inputThrottle + PID_MAX_TURN;
	if(motorRR > inputThrottle + PID_MAX_TURN) motorRR = inputThrottle + PID_MAX_TURN;

	DShot_SendThrottle(motorLF, motorRF, motorLR, motorRR);

	return PID_OK;
}

/**
 * @brief This function changes a PID controller coefficients via uart
 * @param huart
 */
void PID_ChangeKs(UART_HandleTypeDef *huart)
{
	// get index
	int index = atoi(&receiveData[0]);

	// check if index is in range
	if(index < 0 || index > 8)
	{
		Terminal_Print("index out of range\n\r");
		return;
	}

	// get value
	char value[9];
	sprintf(value, "%c%c%c%c%c%c%c%c", receiveData[2], receiveData[3], receiveData[4], receiveData[5], receiveData[6], receiveData[7], receiveData[8], receiveData[9]);

	// check if value is in range
	double tmp = atof(value);
	if(tmp > 5 || tmp < 0)
	{
		Terminal_Print("value out of range\n\r");
		return;
	}

	// write value to array
	PID_AllKs[index] = tmp;

	// write confirmation to terminal (eg: 0 = 0.250000)
	sprintf(txt, "[%d] = %f\n\r", index, PID_AllKs[index]);
	Terminal_Print(txt);
}












