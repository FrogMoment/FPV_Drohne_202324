/**
  ******************************************************************************
  * @file    MPU9150.c
  * @author  Manuel Lang
  * @version V2.0
  * @date    12.10.2017
  * @brief   This file provides functions for:          
  *           + Initialization of the MPU9150 sensore	
	*						+ measurement for temperature	
	*						+ measurement for acceleration of x,y,z-axis
	*						+ measurement for gyroskope of x,y,z-axis
  ******************************************************************************
  */
	
/* Define to prevent recursive inclusion */
#ifndef __MPU9150_H
#define __MPU9150_H

#ifndef  MPU9150_MOD
#define EXPORT
#else
#define EXPORT extern
#endif
	
/********* Defines *********/
/* Use DMA for i2C reading 1=True/0=FALSE */
#define USE_DMA		1	
	
/* maths-functions */	
#define	PI	3.141592653589793f 
	
/* Offest angle */		
#define	OFFSET	-0.8
	
/* MPU-I2C-Address */
#define I2C_MPU9250_ADDRESS 0xD2   // Address of IMU MPU9250
#define I2C_AK8963_ADDRESS  0x18   // Address of Magnetometer

//Magnetometer Registers
#define WHO_AM_I_AK8963  0x00 // (AKA WIA) should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

	
/* MPU-Config-Registers */
#define MPU9150_CONFIG             0x1A
#define MPU9150_PWR_MGMT_1         0x6B      
#define MPU9150_GYRO_CONFIG        0x1B   
#define MPU9150_ACCEL_CONFIG       0x1C 
#define MPU9150_RA_SMPLRT_DIV      0x19
#define MPU9150_RA_INT_PIN_CFG     0x37
#define MPU9150_RA_INT_PIN_CFG     0x37
#define MPU9150_RA_INT_ENABLE      0x38
	
/* MPU-ACCEL-MEASUREMENT-Registers */
#define MPU9150_ACCEL_XOUT_H       0x3B    
#define MPU9150_ACCEL_XOUT_L       0x3C     
#define MPU9150_ACCEL_YOUT_H       0x3D     
#define MPU9150_ACCEL_YOUT_L       0x3E     
#define MPU9150_ACCEL_ZOUT_H       0x3F    
#define MPU9150_ACCEL_ZOUT_L       0x40
	
/* MPU-GYRO-MEASUREMENT-Registers */
#define MPU9150_GYRO_XOUT_H        0x43     
#define MPU9150_GYRO_XOUT_L        0x44    
#define MPU9150_GYRO_YOUT_H        0x45     
#define MPU9150_GYRO_YOUT_L        0x46   
#define MPU9150_GYRO_ZOUT_H        0x47    
#define MPU9150_GYRO_ZOUT_L        0x48

/* MPU-TEMP-MEASUREMENT-Registers */
#define MPU9150_TEMP_OUT_H         0x41     
#define MPU9150_TEMP_OUT_L         0x42 

/* MPU-ACCEL-Sensivity */
#define MPU9150_ACC_SENSITIVITY 16384.0f   

/* MPU-GYRO-Sensivity */
#define MPU9150_GYRO_SENSITIVITY 131.0f

/* Digital-Lowpass-Filter-Configuration */
typedef enum
{
  DLPF_250_Hz = 0,
  DLPF_184_Hz = 1,
  DLPF_94_Hz = 2,
  DLPF_44_Hz = 3,
  DLPF_21_Hz = 4,
  DLPF_10_Hz = 5,
  DLPF_5_Hz = 6
} mpu9150_bandwidth_dlpf;


/* ACCEL-Measurement-Range */
typedef enum													/****************************************/
{																			/*	 AFS_SEL	 *	 Full Scale Range			*/
	MR_2_g = 0,													/*			0		 	 *	     +/- 2 g					*/
	MR_4_g = 1,											  	/*			1		   *	     +/- 4 g					*/
	MR_8_g = 2,													/*			2		   *	     +/- 8 g					*/
	MR_16_g = 3													/*			3		   *	     +/- 16 g					*/
} mpu9150_acc_measurement_range;			/****************************************/


/* GYRO-Measurement-Range */
typedef enum								 					/***************************************/
{											 			 					/*	   FS_SEL	  *	  Full Scale Range	 */
  MR_250 = 0,								 					/*			 0		  *	    +/- 250 °/s	   	 */
  MR_500 = 1, 												/*			 1		  *	    +/- 500 °/s		   */
  MR_1000 = 2, 												/*			 2		  *	    +/- 1000 °/s		 */
  MR_2000 = 3												  /*			 3		  *	    +/- 2000 °/s		 */
} mpu9150_gyro_measurement_range;			/***************************************/	


/* xyz-structure */
typedef struct
{
  float x,y,z;
} xyz;

/* ------------------------ Global variables -----------------------*/
//EXPORT xyz acc;
//EXPORT xyz gyro;
//EXPORT double acc_sensitivity;
//EXPORT double gyro_sensitivity;

/* ---------------------- MPU9150 Functions ------------------------*/
/**
  * @brief Iinitializes the MPU9150
	* @param	I2Cx   pointer to I2C Interface	
	* @param	dlpf   Digital-Lowpass-Filter-Configuration
  * @retval I2C Error Code
  */
EXPORT char MPU9150_Init(I2C_TypeDef *I2Cx,mpu9150_bandwidth_dlpf dlpf);
/**
  * @brief Iinitializes the MPU9150 acellerometer
	* @param	I2Cx   pointer to I2C Interface	
	* @param	mr ...... specifies the full scale range of the accelerometer
  * @retval I2C Error Code
  */
EXPORT char MPU9150_Accel_Config(I2C_TypeDef *I2Cx,mpu9150_acc_measurement_range mr);
/**
  * @brief Iinitializes the MPU9150 gyroskope
	* @param	I2Cx   pointer to I2C Interface	
	* @param	mr ...... specifies the full scale range of the gyroskope
  * @retval I2C Error Code
  */
EXPORT char MPU9150_Gyro_Config(I2C_TypeDef *I2Cx,mpu9150_gyro_measurement_range mr);

/**
  * @brief Iinitializes the MPU9150 Magentometer
	* @param	I2Cx   pointer to I2C Interface	
	* @param	pointer to read sensitivity adjustment values  on x,y,z-axis
  * @retval I2C Error Code
  */
EXPORT char MPU9150_Mag_Config(I2C_TypeDef *I2Cx, xyz *mag_sens);


/**
  * @brief reads the accelerometer x,y,z-values
	* @param	I2Cx   pointer to I2C Interface	
	* @param	pointer measured accelerometer values on x,y,z-axis
  * @retval I2C error code
  */
EXPORT char MPU9150_Read_Acc_XYZ(I2C_TypeDef *I2Cx,xyz *acc);
/**
  * @brief reads the gyro x,y,z-values
	* @param	I2Cx   pointer to I2C Interface	
	* @param	pointer measured Acc values on x,y,z-axis
  * @retval I2C error code
  */
EXPORT char MPU9150_Read_Gyro_XYZ(I2C_TypeDef *I2Cx,xyz *gyro); 

/**
  * @brief reads all gyro and Acc Values of MPU 9150
	* @param	I2Cx   pointer to I2C Interface	
	* @param	gyro(xyz) ...... measured gyro values on x,y,z-axis 
  * @retval I2C error code
  */

/**
  * @brief read temperature
	* @param	I2Cx   pointer to I2C Interface	
	* @param	pointer to temperature value
  * @retval I2C error code
  */
EXPORT char MPU9150_Read_Temp(I2C_TypeDef *I2Cx, float *temperatur); 


/**
  * @brief reads data of magnetometer
	* @param	I2Cx   pointer to I2C Interface	
	* @param	mag_sens sensitivity adjustment values  on x,y,z-axis
	* @param	mag(xyz) ...... measured magenetometer Values on x,y,z-axis 
  * @retval I2C error code
  */
EXPORT char MPU9150_ReadMagData_Old(I2C_TypeDef *I2Cx, xyz mag_sens, xyz *mag);


/**
  * @brief reads data of magnetometer
	* @param	I2Cx   pointer to I2C Interface	
	* @param	mag(xyz) ...... measured magenetometer Values on x,y,z-axis 
  * @retval I2C error code
  */
EXPORT char MPU9150_ReadMagData(I2C_TypeDef *I2Cx, xyz *mag);


/**
  * @brief reads all gyro and Acc Values of MPU 9150
	* @param	I2Cx   pointer to I2C Interface	
	* @param	acc(xyz) ...... measured acceleration on x,y,z-axis 
	* @param	gyro(xyz) ...... measured gyro values on x,y,z-axis 
	* @param	temperature ...... measured temperature value 
  * @retval I2C error code
  */
EXPORT char MPU9150_Read_All(I2C_TypeDef *I2Cx,xyz *acc, xyz *gyro, float *temperature); 

//EXPORT void MPU9150_calibrate_gyro (void);


/**
  * @brief Implementation of Sebastian Madgwick's " orientation filter
	* @param	acc(xyz) ...... measured acceleration on x,y,z-axis 
	* @param	gyro(xyz) ...... measured gyro values on x,y,z-axis 
	* @param	mag(xyz) ...... measured magnetomter values on x,y,z-axis 
	* @param	deltat ...... time between updates in µs 
	* @param	q ......  updated Quaternion 
  * @retval I2C error code
  */
EXPORT void MadgwickQuaternionUpdate(xyz acc,xyz gyro,xyz mag, float deltat,float *q);


#undef  EXPORT
#endif
