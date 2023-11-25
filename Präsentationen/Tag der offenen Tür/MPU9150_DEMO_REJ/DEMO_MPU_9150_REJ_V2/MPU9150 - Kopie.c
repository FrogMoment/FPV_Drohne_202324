/**
  ******************************************************************************
  * @file    SBR_MPU9150.c
  * @author  Manuel Lang
  * @version V1.1
  * @date    12.10.2017
  * @brief   This file provides functions for:          
  *           + Initialization of the MPU9150 sensors	
	*						+ measurement for temperature	
	*						+ measurement for acceleration of x,y,z-axis
	*						+ measurement for gyroskope of x,y,z-axis
	* @History 12.10.2017 V1.0 created Manuel Lang
	*          23.03.2019 V1.1 REJ: transfer to SerBus Library 
  ******************************************************************************
  */
#define MPU9150_MOD	
#include <stm32f10x_i2c.h>
#include <armv30_i2c.h>
#include <armv30_std.h>
#include <stdio.h>
#include <math.h>
#include "MPU9150.h"

float acc_sensitivity,gyro_sensitivity;  


/**
  * @brief Iinitializes the MPU9150
	* @param	I2Cx   pointer to I2C Interface	
	* @param	dlpf   Digital-Lowpass-Filter-Configuration
  * @retval I2C Error Code
  */
char MPU9150_Init(I2C_TypeDef *I2Cx,mpu9150_bandwidth_dlpf dlpf) {
char ret;
	
	/* ------------------------------ Initialize I2C Interface --------------------------------*/
	if((ret=I2C_Master_Init(I2Cx))!=0) {
    return(ret);	
	  }
  /* ----- Clear sleep mode, Enable all Sensors ---*/	
  if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_PWR_MGMT_1, 0x00))!=0){
    return(ret);	
	  }
	wait_ms(100);	
		
	/* ----- PLL with X axis gyroscope reference, disable temperature sensor and sleep mode ---*/	
  //if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_PWR_MGMT_1, 0x0A))!=0){
    //return(ret);	
	  //}
		
  /* ----- Auto select clock source to be PLL gyroscope reference 	---------- */
  if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_PWR_MGMT_1, 0x01))!=0){
    return(ret);	
	  }
	wait_ms(200);

	/* ---------------------Sample Rate 8kHz sample rate  ------------------------*/	
  if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_RA_SMPLRT_DIV, 0x4))!=0){
    return(ret);	
	  }
	
	/* ---------------------Sample Rate 500Hz sample rate ~ Period 2ms ------------------------*/	
  //if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_RA_SMPLRT_DIV, 0x10))!=0){
    //return(ret);	
	  //}
		
	/* ------------- Configure digital low pass filter bandwith (5-250Hz) ------------------*/	
  if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_CONFIG, dlpf))!=0){
    return(ret);	
	  }
	

	//set i2c bypass enable pin to true to access magnetometer
	//The logic level for INT pin is active high, INT pin indicates interrupt pulse’s is width 50us. 	
  //if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_RA_INT_PIN_CFG, 0x02))!=0){
    //return(ret);	
	  //}

	//set i2c bypass enable pin to true to access magnetometer
	//The logic level for INT pin is active high, INT pin level held until interrupt status is cleared
  //Interrupt status is cleared only by reading INT_STATUS register		
  //if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_RA_INT_PIN_CFG, 0x22))!=0){
    //return(ret);	
	  //}

  // set i2c bypass enable pin to true to access magnetometer
	// INT Pin is configured as push pull
	// INT pin indicates interrupt pulse’s is width 50us (active high),
	// Interrupt status is cleared only by reading INT_STATUS register	
  if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_RA_INT_PIN_CFG, 0x02))!=0){
    return(ret);	
	  }
		
	/* ---------------------Interrupt status bits are cleared on any read operation ------------*/	
//  if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_RA_INT_PIN_CFG, 1<<4))!=0){
//    return(ret);	
//	  }
		
	/* --------------------- Interupt occurs when data is ready  ------------*/	
  if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_RA_INT_ENABLE, 0x1))!=0){
    return(ret);	
	  }
return(0);		
}


/**
  * @brief Iinitializes the MPU9150 acellerometer
	* @param	I2Cx   pointer to I2C Interface	
	* @param	mr ...... specifies the full scale range of the accelerometer
  * @retval I2C Error Code
  */
char MPU9150_Accel_Config(I2C_TypeDef *I2Cx,mpu9150_acc_measurement_range mr) {
char ret;	

	/* -------------- Accelerometer configuration : full-scale range [g] ---------*/
  if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_ACCEL_CONFIG, (mr<<3)))!=0){
    return(ret);	
	  }
	acc_sensitivity = MPU9150_ACC_SENSITIVITY / (1 << mr);	/* calculate LSB/g */
	return(0);	
}


/**
  * @brief Iinitializes the MPU9150 gyroskope
	* @param	I2Cx   pointer to I2C Interface	
	* @param	mr ...... specifies the full scale range of the gyroskope
  * @retval I2C Error Code
  */
char MPU9150_Gyro_Config(I2C_TypeDef *I2Cx,mpu9150_gyro_measurement_range mr) {
char ret;	

	/* ------------- Gyroscope configu//ration - full-scale range [°/sec] --------*/
  if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_GYRO_CONFIG, (mr<<3)))!=0){
    return(ret);	
	  }
	gyro_sensitivity = MPU9150_GYRO_SENSITIVITY / (1 << mr);	/* calculate LSB per °/sec*/
  return(0);	
}

/**
  * @brief Iinitializes the MPU9150 Magentometer
	* @param	I2Cx   pointer to I2C Interface	
	* @param	pointer to read sensitivity adjustment values  on x,y,z-axis
  * @retval I2C Error Code
  */
char MPU9150_Mag_Config(I2C_TypeDef *I2Cx, xyz *mag_sens) {
char ret;
uint8_t data[3];	
uint8_t reg_adress;	
	
  /* --------------- enable the magnetometer , 16 Bit, Single Measurement Mode --------*/
  //if((ret=I2C_Write_Reg(I2Cx, I2C_AK8963_ADDRESS,AK8963_CNTL, 0x01))!=0){
    //return(ret);	
	  //}


  /* ------------------------------Power Down Magnetometer ----------------------------*/
  if((ret=I2C_Write_Reg(I2Cx, I2C_AK8963_ADDRESS,AK8963_CNTL, 0x00))!=0){
    return(ret);	
	  }
  wait_ms(10);  // Wait for Mode Change 
	
	/* --------------- Set Magentometer: Fuse ROM access Mode --------------*/
  if((ret=I2C_Write_Reg(I2Cx, I2C_AK8963_ADDRESS,AK8963_CNTL, 0x0F))!=0){
    return(ret);	
	  }
		
	/* --------------- Set Magentometer: Fuse ROM access Mode with 16 Bit output -------*/
  //if((ret=I2C_Write_Reg(I2Cx, I2C_AK8963_ADDRESS,AK8963_CNTL, 0x1F))!=0){
    //return(ret);	
	  //}
	
	/* --------------- Read Sensitivity Values  Magnetometer  -----------*/
	reg_adress = AK8963_ASAX;
	ret=I2C_Master_Transmit(I2Cx, I2C_AK8963_ADDRESS, &reg_adress,1); 
	if(ret !=0) {
		return(ret);
	  }
	ret = I2C_Master_Receive(I2Cx,I2C_AK8963_ADDRESS,&data[0], 3);
	if(ret !=0) {
	  return(ret);
	  }
	// Read the x-, y-, and z-axis calibration values and calc sensitivity adjustment values
  mag_sens->x =  (float)(data[0] - 128)/256.0 + 1.0;   
  mag_sens->y =  (float)(data[1] - 128)/256.0 + 1.0;  
  mag_sens->z =  (float)(data[2] - 128)/256.0 + 1.0; 
		
	/* --------------- Reset Magentometer: Power down Mode -------*/
  if((ret=I2C_Write_Reg(I2Cx, I2C_AK8963_ADDRESS,AK8963_CNTL, 0x00))!=0){
    return(ret);	
	  }
  wait_ms(10);     // Wait for Mode Change 

		
	/* --------------- Set Magentometer: Continous Mode 1 and 16 Bit Output -------*/
  if((ret=I2C_Write_Reg(I2Cx, I2C_AK8963_ADDRESS,AK8963_CNTL, 0x12))!=0){
    return(ret);	
	  }
  wait_ms(10);    // Wait for Mode change		
  return(0);
}


/**
  * @brief read temperature
	* @param	I2Cx   pointer to I2C Interface	
	* @param	pointer to measured temperature value
  * @retval I2C error code
  */
char MPU9150_Read_Temp(I2C_TypeDef *I2Cx,float *temperatur) {
	
uint8_t i2cBuffer[2];
uint8_t reg_adress;	
short int help;
char ret=0;
//char buffer[100];
	
	/* -------------------- read TEMP_OUT_Registers -------------*/
	reg_adress = MPU9150_TEMP_OUT_H;
	ret=I2C_Master_Transmit(I2Cx, I2C_MPU9250_ADDRESS, &reg_adress,1); 
	if(ret !=0) {
	  }
	
	ret = I2C_Master_Receive(I2Cx,I2C_MPU9250_ADDRESS,i2cBuffer,2);
	if(ret !=0) {
	  }
	help = ((i2cBuffer[0] << 8) | i2cBuffer[1]);	/* calculate temperture value */
	//*temperatur = (((float)help)/ 340) + 35;
	*temperatur = (double)(help + 12412) / 340.0;	

	return(ret);
}

/**
  * @brief reads the accelerometer x,y,z-values
	* @param	I2Cx   pointer to I2C Interface	
	* @param	pointer measured accelerometer values on x,y,z-axis
  * @retval I2C error code
  */
char MPU9150_Read_Acc_XYZ(I2C_TypeDef *I2Cx,xyz *acc) {
uint8_t i2cBuffer[6];
uint8_t reg_adress;	
short int help;
char ret=0;
//char buffer[100];
	
	reg_adress = MPU9150_ACCEL_XOUT_H;
	ret=I2C_Master_Transmit(I2Cx, I2C_MPU9250_ADDRESS, &reg_adress,1); 
	if(ret !=0) {
	  }
	
	ret = I2C_Master_Receive(I2Cx,I2C_MPU9250_ADDRESS,i2cBuffer,6);
	if(ret !=0) {
	  }
		
	/* calculate acceleration x-value */
	help = ((i2cBuffer[0] << 8) | (i2cBuffer[1]));
	acc->x= ((float)help)/ acc_sensitivity;
	
	/* calculate acceleration y-value */
	help = ((i2cBuffer[2] << 8) | (i2cBuffer[3]));
	acc->y = ((float)help)/ acc_sensitivity;
	
	/* calculate acceleration z-value */
	help = ((i2cBuffer[4] << 8) | (i2cBuffer[5]));	
	acc->z = ((float)help)/ acc_sensitivity;

  return(ret);
}


/**
  * @brief reads the gyro x,y,z-values
	* @param	I2Cx   pointer to I2C Interface	
	* @param	pointer measured gyro values on x,y,z-axis
  * @retval I2C error code
  */
char MPU9150_Read_Gyro_XYZ(I2C_TypeDef *I2Cx,xyz *gyro) {
	
	uint8_t i2cBuffer[6];
	uint8_t reg_adress;	
	short int help;
	char ret=0;
	
	reg_adress = MPU9150_GYRO_XOUT_H;
	ret=I2C_Master_Transmit(I2Cx, I2C_MPU9250_ADDRESS, &reg_adress,1); 
	if(ret !=0) {
	  }
	
	ret = I2C_Master_Receive(I2Cx,I2C_MPU9250_ADDRESS,i2cBuffer, 6);
	if(ret !=0) {
	  }
	
	/* calculate gyroskop x-value */
	help = ((i2cBuffer[0] << 8) | (i2cBuffer[1]));	
	gyro->x = ((float)help)/ gyro_sensitivity;
	
	/* calculate gyroskop y-value */
	help = ((i2cBuffer[2] << 8) | (i2cBuffer[3]));	
	gyro->y = ((float)help)/ gyro_sensitivity;
	
	/* calculate gyroskop z-value */
	help = ((i2cBuffer[4] << 8) | (i2cBuffer[5]));	
	gyro->z = ((float)help)/ gyro_sensitivity;
		
	return(ret);	
}


/**
  * @brief reads data of magnetometer
	* @param	I2Cx   pointer to I2C Interface	
	* @param	mag_sens sensitivity adjustment values  on x,y,z-axis
	* @param	mag(xyz) ...... measured magenetometer Values on x,y,z-axis 
  * @retval I2C error code
  */
char MPU9150_ReadMagData_Old(I2C_TypeDef *I2Cx, xyz mag_sens, xyz *mag) {
uint8_t rawData[7];
uint8_t status;
uint8_t reg_adress;	
uint8_t c; 
float mRes;	
	
char ret=0;
	
	/* -------------------- read Status Register Magnetometer ----------------*/
	reg_adress = AK8963_ST1;
  ret = I2C_Read_Reg(I2Cx, I2C_AK8963_ADDRESS,reg_adress, &status);	
	if(ret !=0) {
		return(ret);
	  }
	
	/* -------- Check Data ready bit is set ?  Yes --> Read Data Magentometer ----- */	
	if (status & 0x01) {
	  reg_adress = AK8963_XOUT_L;
	  ret=I2C_Master_Transmit(I2Cx, I2C_AK8963_ADDRESS, &reg_adress,1); 
	  if(ret !=0) {
		  return(ret);
	    }
	  ret = I2C_Master_Receive(I2Cx,I2C_AK8963_ADDRESS,&rawData[0], 7);
	  if(ret !=0) {
		  return(ret);
		  }
    c = rawData[6]; // End data read by reading ST2 register
    // Check if magnetic sensor overflow set, if not then report data
    if (!(c & 0x08)) {
			mRes = 10.0*4912.0/32760.0; // Resolution Magenetometer (16 Bit ADC)
      // Turn the MSB and LSB into a signed 16-bit value
      // Data stored as little Endian
      mag->x = (float)(((int16_t)rawData[1] << 8) | rawData[0]);
		  mag->x = mag->x*mRes*mag_sens.x;   // Calc Value in milli Gauß 
			
      mag->y = (float)(((int16_t)rawData[3] << 8) | rawData[2]);
			mag->y = mag->y*mRes*mag_sens.y;   // Calc Value in milli Gauß 
			
      mag->z = (float)(((int16_t)rawData[5] << 8) | rawData[4]);
			mag->z = mag->z*mRes*mag_sens.z;   // Calc Value in milli Gauß 
      }
	  }		
  return(ret);	
}

/**
  * @brief reads data of magnetometer
	* @param	I2Cx   pointer to I2C Interface	
	* @param	mag(xyz) ...... measured magenetometer Values on x,y,z-axis 
  * @retval I2C error code
  */
char MPU9150_ReadMagData(I2C_TypeDef *I2Cx, xyz *mag) {
uint8_t rawData[7];
uint8_t reg_adress;	
uint8_t status;	
	
char ret=0;

	/* -------------------- read Status Register Magnetometer ----------------*/
	do {
	  reg_adress = AK8963_ST1;
    ret = I2C_Read_Reg(I2Cx, I2C_AK8963_ADDRESS,reg_adress, &status);	
	  if(ret !=0) {
		  return(ret);
	    }
	  } while (!(status&0x1));   // Wait till ready Bit is set
	
	reg_adress = AK8963_XOUT_L;
	ret=I2C_Master_Transmit(I2Cx, I2C_AK8963_ADDRESS, &reg_adress,1); 
	if(ret !=0) {
		return(ret);
	  }
	ret = I2C_Master_Receive(I2Cx,I2C_AK8963_ADDRESS,&rawData[0], 6);
	if(ret !=0) {
	 return(ret);
	 }
	
  // Turn the MSB and LSB into a signed 16-bit value
  mag->x = (float)(((int16_t)rawData[1] << 8) | rawData[0]);  // mx
  // Data stored as little Endian
  mag->y = (float)(((int16_t)rawData[3] << 8) | rawData[2]);  //my
  mag->z = (float)(((int16_t)rawData[5] << 8) | rawData[4]);  //mz
	
return(ret);	
}


/**
  * @brief reads all gyro and Acc Values of MPU 9150
	* @param	I2Cx   pointer to I2C Interface	
	* @param	acc(xyz) ...... measured acceleration on x,y,z-axis 
	* @param	gyro(xyz) ...... measured gyro values on x,y,z-axis 
	* @param	temperature ...... measured temperature value 
  * @retval I2C error code
  */
char MPU9150_Read_All(I2C_TypeDef *I2Cx,xyz *acc, xyz *gyro, float *temperature) 
{
	uint8_t i2cBuffer[14];
	uint8_t reg_adress;	
	short int help;
	char ret=0;
	
	reg_adress = MPU9150_ACCEL_XOUT_H;
	ret=I2C_Master_Transmit(I2Cx, I2C_MPU9250_ADDRESS, &reg_adress,1); 
	if(ret !=0) {
	  }
	
	ret = I2C_Master_Receive(I2Cx,I2C_MPU9250_ADDRESS,i2cBuffer, 14);
	if(ret !=0) {
	  }
		
	/* calculate acceleration x-value */
	help = ((i2cBuffer[0] << 8) | (i2cBuffer[1]));
	acc->x= ((float)help)/ acc_sensitivity;
	
	/* calculate acceleration y-value */
	help = ((i2cBuffer[2] << 8) | (i2cBuffer[3]));
	acc->y = ((float)help)/ acc_sensitivity;
	
	/* calculate acceleration z-value */
	help = ((i2cBuffer[4] << 8) | (i2cBuffer[5]));	
	acc->z = ((float)help)/ acc_sensitivity;
	
  /**
     * The following parameters are taken from the documentation [MPU-6000/MPU-6050 Product Specification, p.14]:
     *
     * Temperature sensor is -40°C to +85°C (signed integer)
     * 340 per °C
     * Offset = -512 at 35°C
     * At 0°C: -512 - (340 * 35) = -12412
     */
	help = ((i2cBuffer[6] << 8) | (i2cBuffer[7]));	
	*temperature = (((float)help)/ 340) + 35;
  //*temperature = (double)(help + 12412) / 340.0;
		
	/* calculate gyroskop x-value */
	help = ((i2cBuffer[8] << 8) | (i2cBuffer[9]));	
	gyro->x = ((float)help)/ gyro_sensitivity;
	
	/* calculate gyroskop y-value */
	help = ((i2cBuffer[10] << 8) | (i2cBuffer[11]));	
	gyro->y = ((float)help)/ gyro_sensitivity;
	
	/* calculate gyroskop z-value */
	help = ((i2cBuffer[12] << 8) | (i2cBuffer[13]));	
	gyro->z = ((float)help)/ gyro_sensitivity;
	
	return(ret);
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic
// sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based 
// estimate of absolute device orientation -- which can be converted to yaw, pitch, and roll. 
// Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based 
// filtering algorithms but is much less computationally intensive ---it can be performed on 
// a 3.3 V Pro Mini operating at 8 MHz!

/**
  * @brief Implementation of Sebastian Madgwick's " orientation filter
	* @param	acc(xyz) ...... measured acceleration on x,y,z-axis 
	* @param	gyro(xyz) ...... measured gyro values on x,y,z-axis 
	* @param	mag(xyz) ...... measured magnetomter values on x,y,z-axis 
	* @param	deltat ...... time between updates in µs 
	* @param	q ......  updated quaternion 
  * @retval I2C error code
  */
void MadgwickQuaternionUpdate(xyz acc,xyz gyro,xyz mag, float deltat,float *q) {
float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
float norm;
float hx, hy, _2bx, _2bz;
float s1, s2, s3, s4;
float qDot1, qDot2, qDot3, qDot4;
            
float _2q1mx;  // Auxiliary variables to avoid repeated arithmetic
float _2q1my;
float _2q1mz;
float _2q2mx;
float _4bx;
float _4bz;
float _2q1 = 2.0f * q1;
float _2q2 = 2.0f * q2;
float _2q3 = 2.0f * q3;
float _2q4 = 2.0f * q4;
float _2q1q3 = 2.0f * q1 * q3;
float _2q3q4 = 2.0f * q3 * q4;
float q1q1 = q1 * q1;
float q1q2 = q1 * q2;
float q1q3 = q1 * q3;
float q1q4 = q1 * q4;
float q2q2 = q2 * q2;
float q2q3 = q2 * q3;
float q2q4 = q2 * q4;
float q3q3 = q3 * q3;
float q3q4 = q3 * q4;
float q4q4 = q4 * q4;

float GyroMeasError;  // gyroscope measurement error in rads/s (start at 40 deg/s)
float beta;   

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was
// found to give optimal accuracy. However, with this value, the LSM9SD0 response time is about 
// 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a 
// quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is 
// reduced to ~2 sec I haven't noticed any reduction in solution accuracy. This is essentially the 
// I coefficient in a PID control sense; the bigger the feedback coefficient, the faster the solution
// converges, usually at the expense of accuracy. In any case, this is the free parameter in the 
// Madgwick filtering and fusion scheme.

	GyroMeasError = PI * (40.0f / 180.0f);
  beta = sqrtf(3.0f / 4.0f) * GyroMeasError;  // compute beta

  // Normalise accelerometer measurement
  norm = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  acc.x *= norm;
  acc.y *= norm;
  acc.z *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mag.x *= norm;
  mag.y *= norm;
  mag.z *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mag.x;
  _2q1my = 2.0f * q1 * mag.y;
  _2q1mz = 2.0f * q1 * mag.z;
  _2q2mx = 2.0f * q2 * mag.x;
  hx = mag.x * q1q1 - _2q1my * q4 + _2q1mz * q3 + 
	     mag.x * q2q2 + _2q2 * mag.y * q3 +
    	_2q2 * mag.z * q4 - mag.x * q3q3 - mag.x * q4q4;
  hy = _2q1mx * q4 + mag.y * q1q1 - _2q1mz * q2 + _2q2mx * q3 - mag.y * q2q2 + mag.y * q3q3 + 
	    _2q3 * mag.z * q4 - mag.y * q4q4;
  _2bx = sqrtf(hx * hx + hy * hy);
	
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mag.z * q1q1 +
         	_2q2mx * q4 - mag.z * q2q2 +
        	_2q3 * mag.y * q4 - mag.z * q3q3 + mag.z * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q2 * (2.0f * q1q2 + _2q3q4 - acc.y) - 
	      _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) + 
				(-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) + 
				_2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q1 * (2.0f * q1q2 + _2q3q4 - acc.y) -
       4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - acc.z) +
			 _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) + 
			 (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) + 
			 (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q4 * (2.0f * q1q2 + _2q3q4 - acc.y) -
	      4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - acc.z) +
				(-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
				(_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
				(_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - acc.x) + _2q3 * (2.0f * q1q2 + _2q3q4 - acc.y) + 
	     (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mag.x) +
			 (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - mag.y) +
			 _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mag.z);
			 
  norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gyro.x - q3 * gyro.y - q4 * gyro.z) - beta * s1;
  qDot2 = 0.5f * (q1 * gyro.x + q3 * gyro.z - q4 * gyro.y) - beta * s2;
  qDot3 = 0.5f * (q1 * gyro.y - q2 * gyro.z + q4 * gyro.x) - beta * s3;
  qDot4 = 0.5f * (q1 * gyro.z + q2 * gyro.y - q3 * gyro.x) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
  }


