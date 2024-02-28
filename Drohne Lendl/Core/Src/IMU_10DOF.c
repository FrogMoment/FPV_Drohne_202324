/**
 * @file IMU_10DOF.c
 * @author Maximilian Lendl
 * @date 2023-11-18
 * @version 1
 *
 * @copyright FPV Drohne DA 202324
 *
 * @brief This file provides functions for:
 *          - MPU9250 + AK8963 initialization
 *          - BMP280 initialization
 *          - read / write registers via I2C
 *          - connection check
 *          - read accerometer, gyroscope, magnetometer, barometer data
 *          - calculate pitch, roll and yaw with complementary filter
 */

#include "IMU_10DOF.h"

/**********************************************************************
--------------------------- GLOBAL VARIABLES ---------------------------
**********************************************************************/

I2C_HandleTypeDef *imu_ComI2C;
TIM_HandleTypeDef *imu_DelayTIM;
float imu_DeltaTime = 0;

float accelSens = 0;
float gyroSens = 0;
IMU_Coordinates gyroOffset = {0};

uint8_t magAdjust[3] = {0};
IMU_BARO_CompensationVal baroCompensation = {0};

IMU_Coordinates accel = {0};
IMU_Coordinates gyro = {0};
IMU_Coordinates mag = {0};

float baroTemperature = 0;
float baroPressure = 0;
float baroAltitude = 0;
float baroAltitudeOffset = 0;

IMU_Angles angle = {0};

/**********************************************************************
------------------------- FUNCTION PROTOTYPES -------------------------
**********************************************************************/

/**
 * @brief This function delay the program in us
 * @param us time to delay
 * @retval None
 */
void IMU_DelayUs(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(imu_DelayTIM, 0);
	while(__HAL_TIM_GET_COUNTER(imu_DelayTIM) < us);
}

/**
 * @brief This function initialzes the 10DOF IMU (accel, gyro, mag, baro)
 * @param imuInit pointer to IMU_InitTypeDef
 * @return IMU_Status
 */
IMU_Status IMU_Init(IMU_InitTypeDef *imuInit)
{
	imu_ComI2C = imuInit->hi2c;
	if(imu_ComI2C == NULL)
		return IMU_I2C_ERROR;

	imu_DelayTIM = imuInit->htim;
	if(imu_DelayTIM == NULL)
		return IMU_TIM_ERROR;

// IMU_dt = imuInit->dt;

	HAL_TIM_Base_Start(imu_DelayTIM); // start delay timer


	uint8_t errorCode;

	// check if all IMU sensors are connected
	errorCode = IMU_CheckConnection();
	if(errorCode != IMU_OK)
		return errorCode;

/*************************************************************************************
------------------------------- MPU9250 initialization -------------------------------
*************************************************************************************/
// reset MPU
	IMU_WriteRegister(MPU9250, IMU_MPU_PWR_MGMT_1_ADDR, 0x00);
	IMU_DelayUs(10000);
	// auto select best clk source
	IMU_WriteRegister(MPU9250, IMU_MPU_PWR_MGMT_1_ADDR, 0x01);
	// enable gyro and accel
	IMU_WriteRegister(MPU9250, IMU_MPU_PWR_MGMT_2_ADDR, 0x00);

	// select full scale range for gyro and accel
	IMU_WriteRegister(MPU9250, IMU_MPU_ACCEL_CONFIG_ADDR, imuInit->accelFS << 3);
	IMU_WriteRegister(MPU9250, IMU_MPU_GYRO_CONFIG_ADDR, imuInit->gyroFS << 3);

	// select digital low pass filter for gyro and accel
	IMU_WriteRegister(MPU9250, IMU_MPU_ACCEL_CONFIG_2_ADDR, imuInit->accelDLPF);
	IMU_WriteRegister(MPU9250, IMU_MPU_CONFIG_ADDR, imuInit->gyroDLPF);

	// select fastest sample rate
	IMU_WriteRegister(MPU9250, IMU_MPU_SMPLRT_DIV_ADDR, 0x00);

	// calculate sensitivity scale factor (LSB/g and LSB/(°/s))
	accelSens = IMU_ACCEL_RES_MAX / (1 << imuInit->accelFS);
	gyroSens = IMU_GYRO_RES_MAX / (1 << imuInit->gyroFS);

	// calibrate gyro
	uint16_t amount = 2000;
	IMU_RegCoordinates tempGyro = {0};
	int32_t tempX = 0, tempY = 0, tempZ = 0;
	for(uint16_t i = 0; i < amount; i++)
	{
		tempGyro = IMU_MPU_ReadGyro();
		tempX += tempGyro.x;
		tempY += tempGyro.y;
		tempZ += tempGyro.z;
		IMU_DelayUs(3000);
	}

	gyroOffset.x = (float)tempX / (float)amount;
	gyroOffset.y = (float)tempY / (float)amount;
	gyroOffset.z = (float)tempZ / (float)amount;

	/*************************************************************************************
	------------------------------- AK8963 initialization -------------------------------
	*************************************************************************************/
	// power down AK8963
	IMU_WriteRegister(AK8963, IMU_MAG_CNTL1_ADDR, 0x00);
	IMU_DelayUs(10000);
	// soft reset the sensor
	IMU_WriteRegister(AK8963, IMU_MAG_CNTL2_ADDR, 0x01);

	// power down AK8963
	IMU_WriteRegister(AK8963, IMU_MAG_CNTL1_ADDR, 0x00);
	HAL_Delay(100);
	// set Fuse ROM access mode + 16-bit output
	IMU_WriteRegister(AK8963, IMU_MAG_CNTL1_ADDR, 0x1F);
	HAL_Delay(100);
	// read sensitivity adjustment values
	IMU_ReadRegister(AK8963, IMU_MAG_ASAX_ADDR, magAdjust, 3);
	// power down AK8963
	IMU_WriteRegister(AK8963, IMU_MAG_CNTL1_ADDR, 0x00);
	HAL_Delay(100);
	// set Continuous measurement mode 2 + 16-bit output 
	IMU_WriteRegister(AK8963, IMU_MAG_CNTL1_ADDR, 0x16);
	HAL_Delay(100);
	IMU_WriteRegister(MPU9250, IMU_MPU_PWR_MGMT_1_ADDR, 0x01);

	/*************************************************************************************
	------------------------------- BMP280 initialization -------------------------------
	*************************************************************************************/
	IMU_WriteRegister(BMP280, IMU_BARO_RESET_ADDR, 0xB6); // reset barometer

	// check status of device
	uint8_t timeout = 0, status = 1;
	while(status != 0x00)
	{
		IMU_ReadRegister(BMP280, IMU_BARO_STATUS_ADDR, &status, 1);
		if(timeout++ > 100)
			return IMU_BARO_INIT_ERROR;
	}

	IMU_BARO_ReadCompensationValues();

	// set standby time and time constant of IIR filter
	uint8_t config = ((imuInit->baroSBT << 5) | (imuInit->baroCoeff << 2));
	IMU_WriteRegister(BMP280, IMU_BARO_CONFIG_ADDR, config);

	// set oversampling settings for temperature and pressure measurement and set normal mode
	uint8_t ctrl = (imuInit->baroTempOS << 5) | (imuInit->baroPressOS << 2) | 0x03;
	IMU_WriteRegister(BMP280, IMU_BARO_CTRL_MEAS_ADDR, ctrl);

	// get current altitude level
	IMU_DelayUs(UINT16_MAX - 1);
	float baroSum = 0;
	for(uint16_t i = 0; i < amount; i++)
	{
		IMU_BARO_ReadBaro();
		baroSum += baroAltitude;
		IMU_DelayUs(1000);
	}
	baroAltitudeOffset = baroSum / amount;

	return IMU_OK;
}

/**
 * @brief This function reads an amount of bytes from registers from the IMU
 * @param sensor MPU9250, AK8963 (MAG), BMP280 (BARO)
 * @param regAddr register address
 * @param data data pointer
 * @param rxBytes amount of bytes to read
 * @return IMU_Status
 */
IMU_Status IMU_ReadRegister(IMU_Sensor sensor, uint8_t regAddr, uint8_t *data, uint8_t rxBytes)
{
		// determine the I2C device address
	uint16_t devAddress;
	switch(sensor)
	{
		case MPU9250:
			devAddress = IMU_MPU_I2C_ADDR;
			break;

		case AK8963:
			devAddress = IMU_MAG_I2C_ADDR;
			break;

		case BMP280:
			devAddress = IMU_BARO_I2C_ADDR;
			break;

		default:
			return IMU_ADDRESS_ERROR;
	}

	// read register(s)
	HAL_I2C_Mem_Read(imu_ComI2C, devAddress, regAddr, I2C_MEMADD_SIZE_8BIT, data, rxBytes, 1000);

	return IMU_OK;
}

/**
 * @brief This function writes an amount of bytes to registers from the IMU
 * @param sensor MPU9250, AK8963 (MAG), BMP280 (BARO)
 * @param regAddr register address
 * @param data data to write
 * @return IMU_Status
 */
IMU_Status IMU_WriteRegister(IMU_Sensor sensor, uint8_t regAddr, uint8_t data)
{
		// determine the I2C device address
	uint16_t devAddress;
	switch(sensor)
	{
		case MPU9250:
			devAddress = IMU_MPU_I2C_ADDR;
			break;

		case AK8963:
			devAddress = IMU_MAG_I2C_ADDR;
			break;

		case BMP280:
			devAddress = IMU_BARO_I2C_ADDR;
			break;

		default:
			return IMU_ADDRESS_ERROR;
	}

	// read register(s)
	HAL_I2C_Mem_Write(imu_ComI2C, devAddress, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	return IMU_OK;
}

/**
 * @brief This function checks the connection of all sensors on the IMU
 * @attention This function enables the bypass mode in the MPU9250
 * @return IMU_Status
 */
IMU_Status IMU_CheckConnection(void)
{
	uint8_t regVal[3] = {0x71, 0x48, 0x58};
	uint8_t regAddr[3] = {IMU_MPU_WHOAMI_ADDR, IMU_MAG_WHOAMI_ADDR, IMU_BARO_CHIPID_ADDR};
	uint8_t sensor[3] = {MPU9250, MAG, BARO};
	uint8_t timeout;
	uint8_t data = 0x00;

	for(uint8_t i = 0; i < 3; i++)
	{
		timeout = 0;

		while(data != regVal[i])
		{
			if(IMU_ReadRegister(sensor[i], regAddr[i], &data, 1) != IMU_OK)
				return IMU_ADDRESS_ERROR;

			if(timeout++ > 100)
				return IMU_MPU_WHOAMI_ERROR + i;
		}

		if(sensor[i] == MPU9250)
		{
				// enable bypass mode
			if(IMU_WriteRegister(MPU9250, IMU_MPU_INT_PIN_CFG_ADDR, 0x02) != IMU_OK)
				return IMU_ADDRESS_ERROR;
			IMU_DelayUs(10000);
		}
	}

	return IMU_OK;
}

/**
 * @brief This function reads gyroscope register data (x,y,z)
 * @return IMU_RegCoordinates
 */
IMU_RegCoordinates IMU_MPU_ReadGyro(void)
{
	uint8_t buffer[6] = {0};
	IMU_ReadRegister(MPU9250, IMU_MPU_GYRO_XOUT_H_ADDR, buffer, 6);

	IMU_RegCoordinates gyroData = {0};
	gyroData.x = ((int16_t)buffer[0] << 8) | (int16_t)buffer[1];
	gyroData.y = ((int16_t)buffer[2] << 8) | (int16_t)buffer[3];
	gyroData.z = ((int16_t)buffer[4] << 8) | (int16_t)buffer[5];

	return gyroData;
}

/**
 * @brief This function reads accelerometer register data (x,y,z)
 * @return IMU_RegCoordinates
 */
IMU_RegCoordinates IMU_MPU_ReadAccel(void)
{
	uint8_t buffer[6] = {0};
	IMU_ReadRegister(MPU9250, IMU_MPU_ACCEL_XOUT_H_ADDR, buffer, 6);

	IMU_RegCoordinates accelData = {0};
	accelData.x = ((int16_t)buffer[0] << 8) | buffer[1];
	accelData.y = ((int16_t)buffer[2] << 8) | buffer[3];
	accelData.z = ((int16_t)buffer[4] << 8) | buffer[5];

	return accelData;
}

/**
 * @brief This function reads magnetometer register data (x,y,z)
 * @return IMU_RegCoordinates
 */
IMU_RegCoordinates IMU_MAG_ReadMag(void)
{
	uint8_t check = 0x00;
	do
	{
		IMU_ReadRegister(AK8963, IMU_MAG_ST1_ADDR, &check, 1);
	}
	while(!(check & 0x01));


	uint8_t buffer[6] = {0};
	IMU_ReadRegister(AK8963, IMU_MAG_HXL_ADDR, buffer, 6);

	IMU_RegCoordinates magData = {0};
	magData.x = ((int16_t)buffer[1] << 8) | (int16_t)buffer[0];
	magData.y = ((int16_t)buffer[3] << 8) | (int16_t)buffer[2];
	magData.z = ((int16_t)buffer[5] << 8) | (int16_t)buffer[4];

	return magData;
}

/**
 * @brief This function calculates pitch,roll and yaw
 * @details data gets stored in the global variable 'angle'
 * @retval None
 */
void IMU_GetAngles(void)
{
	// read counter value since last function call
	uint16_t tmpTime = __HAL_TIM_GET_COUNTER(imu_DelayTIM);
	// reset timer
	__HAL_TIM_SET_COUNTER(imu_DelayTIM, 0);

	// start first time call
	static int8_t firstTimeFlag = 0;
	if(firstTimeFlag == 0)
	{
		tmpTime = 0;
		firstTimeFlag = 1;
	}

	// calculate delta time
	imu_DeltaTime = (float)tmpTime / 1E6f;

	// read sensors
	IMU_RegCoordinates gyroData = IMU_MPU_ReadGyro();
	IMU_RegCoordinates accelData = IMU_MPU_ReadAccel();
	// IMU_RegCoordinates magData = IMU_MAG_ReadMag();

	// calculate actual values
	gyro.x = (gyroData.x - gyroOffset.x) / gyroSens;
	gyro.y = (gyroData.y - gyroOffset.y) / gyroSens;
	gyro.z = (gyroData.z - gyroOffset.z) / gyroSens;

	accel.x = (accelData.x / accelSens) - 0.01f;
	accel.y = (accelData.y / accelSens) - 0.02f;
	accel.z = (accelData.z / accelSens) - 0.1f;

	// invert axis because the sensor is upside down
	accel.z = -accel.z;
	// gyro.y = -gyro.y;
	// gyro.x = -gyro.x;

	// mag.x = (float)magData.x * ((((float)magAdjust[0] - 128.0f) / 256.0f) + 1.0f);
	// mag.y = (float)magData.y * ((((float)magAdjust[1] - 128.0f) / 256.0f) + 1.0f);
	// mag.z = (float)magData.z * ((((float)magAdjust[2] - 128.0f) / 256.0f) + 1.0f);

	// apply complementary filter to calc angles
	float accelPitch = atan2(accel.y, accel.z) * RAD2DEG;
	float accelRoll = atan2(accel.x, accel.z) * RAD2DEG;

	angle.roll = 0.98f * (angle.roll + gyro.y * imu_DeltaTime) + 0.02f * accelRoll;
	angle.pitch = 0.98f * (angle.pitch - gyro.x * imu_DeltaTime) + 0.02f * accelPitch;
	angle.yaw += gyro.z * imu_DeltaTime;
}

/**
 * @brief This function reads the temperature and pressure compensation values
 * @details values get stored in variable "baroCompensation"
 * @retval None
 */
void IMU_BARO_ReadCompensationValues(void)
{
	uint8_t buffer[24];
	IMU_ReadRegister(BMP280, IMU_BARO_DIG_T1_L_ADDR, buffer, 24);

	baroCompensation.T1 = (buffer[1] << 8) | buffer[0];
	baroCompensation.T2 = (buffer[3] << 8) | buffer[2];
	baroCompensation.T3 = (buffer[5] << 8) | buffer[4];

	baroCompensation.P1 = (buffer[7] << 8) | buffer[6];
	baroCompensation.P2 = (buffer[9] << 8) | buffer[8];
	baroCompensation.P3 = (buffer[11] << 8) | buffer[10];
	baroCompensation.P4 = (buffer[13] << 8) | buffer[12];
	baroCompensation.P5 = (buffer[15] << 8) | buffer[14];
	baroCompensation.P6 = (buffer[17] << 8) | buffer[16];
	baroCompensation.P7 = (buffer[19] << 8) | buffer[18];
	baroCompensation.P8 = (buffer[21] << 8) | buffer[20];
	baroCompensation.P9 = (buffer[23] << 8) | buffer[22];
}

/**
 * @brief This function reads the barometer values and calculates temperature, pressure and altitude
 * @details values gets stored in global variables 'baroTemperature', 'baroPressure' and 'baroAltitude'
 * @retval None
 */
void IMU_BARO_ReadBaro(void)
{
	uint8_t buffer[6] = {0};
	IMU_ReadRegister(BMP280, IMU_BARO_PRESS_ADDR, buffer, 6);

	int32_t adcPress = ((int32_t)buffer[0] << 12) | ((int32_t)buffer[1] << 4) | ((int32_t)buffer[2] >> 4);
	int32_t adcTemp = ((int32_t)buffer[3] << 12) | ((int32_t)buffer[4] << 4) | ((int32_t)buffer[5] >> 4);

	int32_t fineTemp;
	int32_t temp = IMU_BARO_CompensateTemp(adcTemp, &fineTemp);
	uint32_t press = IMU_BARO_CompensatePress(adcPress, fineTemp);

	baroTemperature = (float)temp / 100.0;
	baroPressure = (float)press / 256.0;

	// convert pressure to altitude according to datasheet
	float presshPa = baroPressure / 100;
	baroAltitude = (44330.0 * (1.0 - pow(presshPa / 1013.25, 1.0 / 5.255))) - baroAltitudeOffset;
}

/**
 * @brief This function compensates the temperature according to the datasheet
 * @details
 * Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
 * @param adcTemp measured temperature
 * @param fineTemp
 * @return int32_t (temperature)
 */
int32_t IMU_BARO_CompensateTemp(int32_t adcTemp, int32_t *fineTemp)
{
	int32_t var1, var2, T;
	var1 = ((((adcTemp >> 3) - ((int32_t)baroCompensation.T1 << 1))) * ((int32_t)baroCompensation.T2)) >> 11;
	var2 = (((((adcTemp >> 4) - ((int32_t)baroCompensation.T1)) * ((adcTemp >> 4) - ((int32_t)baroCompensation.T1))) >> 12) * ((int32_t)baroCompensation.T3)) >> 14;
	*fineTemp = var1 + var2;
	T = (*fineTemp * 5 + 128) >> 8;
	return T;
}

/**
 * @brief This function compensates the pressure according to the datasheet
 * @details
 * Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
 * Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 * @param adcPress measured pressure
 * @param fineTemp
 * @return uint32_t (pressure)
 */
uint32_t IMU_BARO_CompensatePress(int32_t adcPress, int32_t fineTemp)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)fineTemp) - 128000;
	var2 = var1 * var1 * (int64_t)baroCompensation.P6;
	var2 = var2 + ((var1 * (int64_t)baroCompensation.P5) << 17);
	var2 = var2 + (((int64_t)baroCompensation.P4) << 35);
	var1 = ((var1 * var1 * (int64_t)baroCompensation.P3) >> 8) + ((var1 * (int64_t)baroCompensation.P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)baroCompensation.P1) >> 33;
	if(var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adcPress;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)baroCompensation.P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)baroCompensation.P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)baroCompensation.P7) << 4);
	return (uint32_t)p;
}








