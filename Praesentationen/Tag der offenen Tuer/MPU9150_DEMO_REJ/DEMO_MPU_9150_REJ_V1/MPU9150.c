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
	//The logic level for INT pin is active high, INT pin indicates interrupt pulseís is width 50us. 	
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
	// INT pin indicates interrupt pulseís is width 50us (active high),
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

	/* ------------- Gyroscope configu//ration - full-scale range [∞/sec] --------*/
  if((ret=I2C_Write_Reg(I2Cx, I2C_MPU9250_ADDRESS,MPU9150_GYRO_CONFIG, (mr<<3)))!=0){
    return(ret);	
	  }
	gyro_sensitivity = MPU9150_GYRO_SENSITIVITY / (1 << mr);	/* calculate LSB per ∞/sec*/
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
char MPU9150_ReadMagData(I2C_TypeDef *I2Cx, xyz mag_sens, xyz *mag) {
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
		  mag->x = mag->x*mRes*mag_sens.x;   // Calc Value in milli Gauﬂ 
			
      mag->y = (float)(((int16_t)rawData[3] << 8) | rawData[2]);
			mag->y = mag->y*mRes*mag_sens.y;   // Calc Value in milli Gauﬂ 
			
      mag->z = (float)(((int16_t)rawData[5] << 8) | rawData[4]);
			mag->z = mag->z*mRes*mag_sens.z;   // Calc Value in milli Gauﬂ 
      }
	  }		
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
     * Temperature sensor is -40∞C to +85∞C (signed integer)
     * 340 per ∞C
     * Offset = -512 at 35∞C
     * At 0∞C: -512 - (340 * 35) = -12412
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



