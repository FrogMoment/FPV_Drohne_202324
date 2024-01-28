/**
 * @file IMU_10DOF.h
 * @author Maximilian Lendl
 * @date 2023-11-18
 * @version 1
 *
 * @copyright FPV Drohne DA 202324
 *
 * @brief
 *
 */

#ifndef IMU_10DOF_INCLUDED
#define IMU_10DOF_INCLUDED

/**********************************************************************************
------------------------------------ INCLUDES ------------------------------------
**********************************************************************************/

#include "main.h"
#include <string.h>
#include <stdio.h>
#include "status_handling.h"
#include <math.h>

/**********************************************************************************
--------------------------------- GLOBAL DEFINES ---------------------------------
**********************************************************************************/

// MPU9255 addresses
#define IMU_MPU_I2C_ADDR            0xD0    // MPU9250 I2C slave address
#define IMU_MPU_WHOAMI_ADDR         0x75    // MPU9250 WHO AM I register address
#define IMU_MPU_INT_PIN_CFG_ADDR    0x37    // MPU9250 bypass enable register address
#define IMU_MPU_USER_CTRL_ADDR      0x6A    // MPU9250 user control register address
#define IMU_MPU_PWR_MGMT_1_ADDR     0x6B    // MPU9250 power management 1 register address
#define IMU_MPU_PWR_MGMT_2_ADDR     0x6C    // MPU9250 power management 2 register address
#define IMU_MPU_INT_ENABLE_ADDR     0x38    // MPU9250 interrupt enable register address
#define IMU_MPU_FIFO_EN_ADDR        0x23    // MPU9250 FIFO enable register address
#define IMU_MPU_I2C_MST_CTRL_ADDR   0x24    // MPU9250 I2C master control register address
#define IMU_MPU_SMPLRT_DIV_ADDR     0x19    // MPU9250 sample rate divider register address
#define IMU_MPU_CONFIG_ADDR         0x1A    // MPU9250 configuration register address
#define IMU_MPU_GYRO_CONFIG_ADDR    0x1B    // MPU9250 gyroscope configuration register address
#define IMU_MPU_ACCEL_CONFIG_ADDR   0x1C    // MPU9250 accelerometer configuration 1 register address
#define IMU_MPU_ACCEL_CONFIG_2_ADDR 0x1D    // MPU9250 accelerometer configuration 2 register address
#define IMU_MPU_FIFO_COUNTH_ADDR    0x72    // MPU9250 FIFO count registers register address
#define IMU_MPU_FIFO_R_W_ADDR       0x74    // MPU9250 FIFO read write register address
#define IMU_MPU_XG_OFFSET_H_ADDR    0x13    // MPU9250 gyro offset X axis high register address
#define IMU_MPU_XG_OFFSET_L_ADDR    0x14    // MPU9250 gyro offset X axis low register address
#define IMU_MPU_YG_OFFSET_H_ADDR    0x15    // MPU9250 gyro offset Y axis high register address
#define IMU_MPU_YG_OFFSET_L_ADDR    0x16    // MPU9250 gyro offset Y axis Low register address
#define IMU_MPU_ZG_OFFSET_H_ADDR    0x17    // MPU9250 gyro offset Z axis high register address
#define IMU_MPU_ZG_OFFSET_L_ADDR    0x18    // MPU9250 gyro offset Z axis low register address
#define IMU_MPU_XA_OFFSET_H_ADDR    0x77    // MPU9250 accel offset X axis high register address
#define IMU_MPU_XA_OFFSET_L_ADDR    0x78    // MPU9250 accel offset X axis low register address
#define IMU_MPU_YA_OFFSET_H_ADDR    0x7A    // MPU9250 accel offset Y axis high register address
#define IMU_MPU_YA_OFFSET_L_ADDR    0x7B    // MPU9250 accel offset Y axis low register address
#define IMU_MPU_ZA_OFFSET_H_ADDR    0x7D    // MPU9250 accel offset Z axis high register address
#define IMU_MPU_ZA_OFFSET_L_ADDR    0x7E    // MPU9250 accel offset Z axis low register address

// magnetometer addresses
#define IMU_MAG_I2C_ADDR    0x18    // magnetometer I2C slave address
#define IMU_MAG_WHOAMI_ADDR 0x00    // magnetometer WHO AM I register address
#define IMU_MAG_CNTL1_ADDR  0x0A    // magnetometer control 1 register address 
#define IMU_MAG_CNTL2_ADDR  0x0B    // magnetometer control 2 register address 
#define IMU_MAG_ASAX_ADDR   0x10    // magnetometer sensitivity adjustment register address 
#define IMU_MAG_HXL_ADDR    0x03    // magnetometer measurement data x axis low register address 
#define IMU_MAG_ST1_ADDR    0x02    // magnetometer status 1 register

// barometer addresses
#define IMU_BARO_I2C_ADDR       0xEE    // barometer I2C slave address
#define IMU_BARO_CHIPID_ADDR    0xD0    // barometer CHIP ID register address
#define IMU_BARO_RESET_ADDR     0xE0    // barometer reset resgister address
#define IMU_BARO_STATUS_ADDR    0xF3    // barometer reset resgister address
#define IMU_BARO_CTRL_MEAS_ADDR 0xF4    // barometer control measurement register address
#define IMU_BARO_CONFIG_ADDR    0xF5    // barometer config register address
#define IMU_BARO_DIG_T1_L_ADDR  0x88    // barometer temperature compensation value T1 low byte (next 24 are all temperture and pressure)
#define IMU_BARO_PRESS_ADDR     0xF7    // barometer pressure measurements register address

// output registers
#define IMU_MPU_GYRO_XOUT_H_ADDR    0x43    // MPU9250 gyroscope output register X axis high
#define IMU_MPU_ACCEL_XOUT_H_ADDR   0x3B    // MPU9250 accelerometer ouput register X axis high 
#define IMU_MAG_XOUT_L_ADDR         0x03    // AK8963 output register X axis high

// max resolution
#define IMU_GYRO_RES_MAX 131.0f
#define IMU_ACCEL_RES_MAX 16384.0f
#define RAD2DEG 57.29578f

/**********************************************************************************
-------------------------------- GLOBAL STRUCTURES --------------------------------
**********************************************************************************/

// IMU OK and error codes
typedef enum IMU_Status
{
    IMU_OK = 0,
    IMU_ADDRESS_ERROR = 1,
    IMU_I2C_ERROR = 2,
    IMU_TIM_ERROR = 3,

    IMU_MPU_WHOAMI_ERROR = 10,
    IMU_MAG_WHOAMI_ERROR = 11,
    IMU_BARO_CHIPID_ERROR = 12,

    IMU_BARO_INIT_ERROR = 13
} IMU_Status;

// all IMU sensors
typedef enum IMU_Sensor
{
    MPU9250 = 0,
    AK8963 = 1,
    MAG = 1,
    BMP280 = 2,
    BARO = 2
} IMU_Sensor;

// full scale range values
typedef enum IMU_Fullscale
{
    GYRO_250DPS = 0,
    GYRO_500DPS = 1,
    GYRO_1000DPS = 2,
    GYRO_2000DPS = 3,

    ACCEL_2G = 0,
    ACCEL_4G = 1,
    ACCEL_8G = 2,
    ACCEL_16G = 3
} IMU_Fullscale;

typedef enum IMU_DLPF
{
    GYRO_DLPF_250HZ = 0,
    GYRO_DLPF_184HZ = 1,
    GYRO_DLPF_92HZ = 2,
    GYRO_DLPF_41HZ = 3,
    GYRO_DLPF_20HZ = 4,
    GYRO_DLPF_10HZ = 5,
    GYRO_DLPF_5HZ = 6,
    GYRO_DLPF_3600HZ = 7,

    ACCEL_DLPF_218HZ = 1,
    ACCEL_DLPF_99HZ = 2,
    ACCEL_DLPF_44HZ = 3,
    ACCEL_DLPF_21HZ = 4,
    ACCEL_DLPF_10HZ = 5,
    ACCEL_DLPF_5HZ = 6,
    ACCEL_DLPF_420HZ = 7
} IMU_DLPF;

// x, y, z coordinates for register values
typedef struct IMU_RegCoordinates
{
    int16_t x;
    int16_t y;
    int16_t z;
} IMU_RegCoordinates;

// x, y, z coordinates (float)
typedef struct IMU_Coordinates
{
    float x;
    float y;
    float z;
} IMU_Coordinates;

// BMP280 compensation values
typedef struct IMU_BARO_CompensationVal
{
    // temperature compensation values
    uint16_t T1;
    int16_t T2, T3;

    // pressure compensation values
    uint16_t P1;
    int16_t P2, P3, P4, P5, P6, P7, P8, P9;
} IMU_BARO_CompensationVal;

typedef enum IMU_BARO_StandByTime
{
    BARO_STANDBY_0P5MS = 0,
    BARO_STANDBY_62P5MS = 1,
    BARO_STANDBY_125MS = 2,
    BARO_STANDBY_250MS = 3,
    BARO_STANDBY_500MS = 4,
    BARO_STANDBY_1000MS = 5,
    BARO_STANDBY_2000MS = 6,
    BARO_STANDBY_4000MS = 7
} IMU_BARO_StandByTime;

typedef enum IMU_BARO_IIRFilterCoefficient
{
    IMU_BARO_FILTER_OFF = 0,
    IMU_BARO_FILTER_COEFF_2 = 1,
    IMU_BARO_FILTER_COEFF_4 = 2,
    IMU_BARO_FILTER_COEFF_8 = 3,
    IMU_BARO_FILTER_COEFF_16 = 5
} IMU_BARO_IIRFilterCoefficient;

typedef struct IMU_Angles
{
    float pitch;
    float roll;
    float yaw;
} IMU_Angles;

typedef enum IMU_BARO_Oversampling
{
    BARO_TEMP_OS_SKIP = 0,
    BARO_TEMP_OS_1X = 1,
    BARO_TEMP_OS_2X = 2,
    BARO_TEMP_OS_4X = 3,
    BARO_TEMP_OS_8X = 4,
    BARO_TEMP_OS_16X = 5,

    BARO_PRESS_OS_SKIP = 0,
    BARO_PRESS_OS_1X = 1,
    BARO_PRESS_OS_2X = 2,
    BARO_PRESS_OS_4X = 3,
    BARO_PRESS_OS_8X = 4,
    BARO_PRESS_OS_16X = 5

} IMU_BARO_Oversampling;

typedef enum IMU_BARO_Mode
{
    BARO_MODE_SLEEP = 0,
    BARO_MODE_FORCE = 1,
    BARO_MODE_NORMAL = 3
} IMU_BARO_Mode;

typedef struct IMU_InitTypeDef
{
    I2C_HandleTypeDef *hi2c;

    IMU_Fullscale gyroFS;
    IMU_Fullscale accelFS;

    IMU_DLPF gyroDLPF;
    IMU_DLPF accelDLPF;

    IMU_BARO_StandByTime baroSBT;
    IMU_BARO_IIRFilterCoefficient baroCoeff;
    IMU_BARO_Oversampling baroTempOS;
    IMU_BARO_Oversampling baroPressOS;
    IMU_BARO_Mode baroMode;

    TIM_HandleTypeDef *htim; // timer for us delay
    float dt; // time between measurements 
} IMU_InitTypeDef;

/**********************************************************************************
-------------------------------- GLOBAL VARIABLES --------------------------------
**********************************************************************************/

extern IMU_Coordinates accel;
extern IMU_Coordinates gyro;
extern IMU_Coordinates mag;

extern IMU_Angles angle;

extern float baroTemperature;
extern float baroPressure;
extern float baroAltitude;

/**********************************************************************************
------------------------------- FUNCTION PROTOTYPES -------------------------------
**********************************************************************************/

/**
 * @brief This function delay the program in us
 * @param us time to delay
 * @retval None
 */
void IMU_DelayUs(uint16_t us);

/**
 * @brief This function initialzes the 10DOF IMU (accelerometer, gyroscope, magnetometer, barometer
 * @param imuInit pointer to IMU_InitTypeDef
 * @return IMU_Status 
 */
IMU_Status IMU_Init(IMU_InitTypeDef *imuInit);

/**
 * @brief This function reads an amount of bytes from registers from the IMU
 * @param sensor MPU9250, AK8963 (MAG), BMP280 (BARO)
 * @param regAddr register address
 * @param data data pointer
 * @param rxBytes amount of bytes to read
 * @return IMU_Status
 */
IMU_Status IMU_ReadRegister(IMU_Sensor sensor, uint8_t regAddr, uint8_t *data, uint8_t rxBytes);

/**
 * @brief This function writes an amount of bytes to registers from the IMU
 * @param sensor MPU9250, AK8963 (MAG), BMP280 (BARO)
 * @param regAddr register address
 * @param data data to write
 * @return IMU_Status
 */
IMU_Status IMU_WriteRegister(IMU_Sensor sensor, uint8_t regAddr, uint8_t data);

/**
 * @brief This function checks the connection of all sensors on the IMU
 * @attention This function enables the bypass mode in the MPU9250
 * @return IMU_Status
 */
IMU_Status IMU_CheckConnection(void);

/**
 * @brief This function reads gyroscope register data (x,y,z)
 * @return IMU_RegCoordinates
 */
IMU_RegCoordinates IMU_MPU_ReadGyro(void);

/**
 * @brief This function reads accelerometer register data (x,y,z)
 * @return IMU_RegCoordinates
 */
IMU_RegCoordinates IMU_MPU_ReadAccel(void);

/**
 * @brief This function reads magnetometer register data (x,y,z)
 * @return IMU_RegCoordinates
 */
IMU_RegCoordinates IMU_MAG_ReadMag(void);

/**
 * @brief This function calculates pitch,roll and yaw
 * @details data gets stored in the global variable 'anges'
 * @retval None
 */
void IMU_GetAngles(void);

/**
 * @brief This function reads the temperature and pressure compensation values
 * @details values get stored in variable "baroCompensation"
 * @retval None
 */
void IMU_BARO_ReadCompensationValues(void);

/**
 * @brief This function reads the barometer values and calculates temperature, pressure and altitude
 * @details values gets stored in global variables 'baroTemperature', 'baroPressure' and 'baroAltitude'
 * @retval None
 */
void IMU_BARO_ReadBaro(void);

/**
 * @brief This function compensates the temperature according to the datasheet
 * @details
 * Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
 * @param adcTemp measured temperature
 * @param fineTemp
 * @return int32_t (temperature)
 */
int32_t IMU_BARO_CompensateTemp(int32_t adcTemp, int32_t *fineTemp);

/**
 * @brief This function compensates the pressure according to the datasheet
 * @details
 * Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
 * Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 * @param adcPress measured pressure
 * @param fineTemp
 * @return uint32_t (pressure)
 */
uint32_t IMU_BARO_CompensatePress(int32_t adcPress, int32_t fineTemp);





#endif // IMU_10DOF_INCLUDED







