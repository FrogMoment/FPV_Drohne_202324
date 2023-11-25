/**
 * @file MPU9250V3.c
 * @author Maximilian Lendl
 * @date 2023-11-20
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief This file provides functions for:
 *              - initialization of MPU9250 + read/write functions
 *              - measurement for accelerometer (x, y, z)
 *              - measurement for gyroscope (x, y, z)
 *              - measurement for magnetometer (x, y, z)
 */

#include "MPU9250V3.h"

/************************************************************************************************
--------------------------------------- GLOBAL VARIABLES ---------------------------------------
************************************************************************************************/

int16_t mpu9250_RawData[14] = {0}; // mpu9250 raw data

MPU9250_CoordTypDef accel;  // accelerometer values
MPU9250_CoordTypDef gyro;   // gyroscope values
MPU9250_CoordTypDef mag;    // magnetometer values
float mpu9250_Temp;         // temperature values

MPU9250_CoordTypDef MPU9250_Offset = {0};
MPU9250_CoordTypDef MPU9250_MagOffset = {0};

float accelSens;     // sensitivity scale factor of accelerometer
float gyroSens;      // sensitivity scale factor of gyroscope

I2C_HandleTypeDef *mpu9250_InputI2C = NULL;

/************************************************************************************************
------------------------------------------- FUNCTIONS -------------------------------------------
************************************************************************************************/

/**
 * @brief This funtion initializes the MPU9250
 * @param hi2c pointer to a I2C_HandleTypeDef structure
 * @param dlpf dlpf bandwidth
 * @param gyroFS full scale range of gyroscope
 * @param accelFS full scale range of accelermeter
 * @return MPU9250_Status
 */
MPU9250_Status MPU9250_Init(I2C_HandleTypeDef *hi2c, MPU9250_DLPFTypeDef dlpf, MPU9250_FullScale gyroFS, MPU9250_FullScale accelFS)
{
    mpu9250_InputI2C = hi2c; // set I2C pointer

    uint8_t data[3] = {0}, timeout = 0;

    // check connection
    while(data[0] != 0x71)
    {
        MPU9250_ReadRegister(MPU9250_I2C_SLV, MPU9250_WHOAMI, &data[0], 1);
        if(timeout++ > 100) // check max 100 times
            return MPU9250_WHOAMI_ERROR;
    }

    MPU9250_WriteRegister(MPU9250_I2C_SLV, MPU9250_PWR_MGMT_1, 0x00, 1);
    MPU9250_WriteRegister(MPU9250_I2C_SLV, MPU9250_SMPLRT_DIV, 0x07, 1);
    MPU9250_WriteRegister(MPU9250_I2C_SLV, MPU9250_CONFIG, dlpf, 1);
    MPU9250_WriteRegister(MPU9250_I2C_SLV, MPU9250_GYRO_CONFIG, gyroFS << 3, 1);
    MPU9250_WriteRegister(MPU9250_I2C_SLV, MPU9250_ACCEL_CONFIG, accelFS << 3, 1);

    HAL_Delay(10);

    // calculate gyro offset
    MPU9250_CoordTypDef tmp = {0};
    for(uint8_t i = 0; i < 32; i++)
    {
        MPU9250_ReadGyro();
        tmp.x += gyro.x;
        tmp.y += gyro.y;
        tmp.z += gyro.z;

        HAL_Delay(1);
    }

    MPU9250_Offset.x = tmp.x / 32.0;    
    MPU9250_Offset.y = tmp.y / 32.0;    
    MPU9250_Offset.z = tmp.z / 32.0;  

    // set sensitivity factors
    accelSens = ACCEL_SENS / (1 << accelFS);    // calc sensitivity scale factor of accelerometer
    gyroSens = GYRO_SENS / (1 << gyroFS);       // calc sensitivity scale factor of gyroscopes      

    return MPU9250_OK;
}

/**
 * @brief read register of MPU9250
 * @param i2c_Addr i2c slave address
 * @param addr register address/es
 * @param data received data
 * @param rxBytes amount of reveived bytes
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @retval None
 */
void MPU9250_ReadRegister(uint8_t i2c_Addr, uint8_t addr, uint8_t *data, int8_t rxBytes)
{
    HAL_I2C_Mem_Read(mpu9250_InputI2C, i2c_Addr << 1, addr, I2C_MEMADD_SIZE_8BIT, data, rxBytes, HAL_MAX_DELAY);
}

/**
 * @brief write to register of MPU9250
 * @param i2c_Addr i2c slave address
 * @param addr register address/es
 * @param data data to write to register
 * @param txBytes amount of bytes to transmit
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @retval None
 */
void MPU9250_WriteRegister(uint8_t i2c_Addr, uint8_t addr, uint8_t data, int8_t txBytes)
{
    HAL_I2C_Mem_Write(mpu9250_InputI2C, i2c_Addr << 1, addr, I2C_MEMADD_SIZE_8BIT, &data, txBytes, HAL_MAX_DELAY);
}

/**
 * @brief This function reads the gyroscope output register 
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return None
 */
void MPU9250_ReadGyro(void)
{
    static int32_t outBuffer[3] = {0};
    static MPU9250_AvgTypeDef MPU9250_Filter[3];
    
    int16_t gyroData[3] = {0};
    uint8_t measurements[6];

    MPU9250_ReadRegister(MPU9250_I2C_SLV, MPU9250_GYRO_XOUT_H, measurements, 6); // read the accel output registers

    // format the measurements to °/s
    gyroData[0] = ((measurements[0] << 8) | measurements[1]);
    gyroData[1] = ((measurements[2] << 8) | measurements[3]);
    gyroData[2] = ((measurements[4] << 8) | measurements[5]);

    for(uint8_t i = 0; i < 3; i++)
    {
        MPU9250_CalcAvgValue(&MPU9250_Filter[i].Index, MPU9250_Filter[i].AvgBuffer, gyroData[i], &outBuffer[i+1]);
    }

    gyro.x = outBuffer[0] - MPU9250_Offset.x;
    gyro.y = outBuffer[1] - MPU9250_Offset.y;
    gyro.z = outBuffer[2] - MPU9250_Offset.z;
}

/**
 * @brief This function reads the accelerometer output register
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return None
 */
void MPU9250_ReadAccel(void)
{
    static int32_t outBuffer[3] = {0};
    static MPU9250_AvgTypeDef MPU9250_Filter[3];
    
    int16_t accelData[3] = {0};
    uint8_t measurements[6];

    MPU9250_ReadRegister(MPU9250_I2C_SLV, MPU9250_ACCEL_XOUT_H, measurements, 6);     // read the accel output registers

    // format the measurements to g's
    accelData[0] = ((measurements[0] << 8) | measurements[1]);
    accelData[1] = ((measurements[2] << 8) | measurements[3]);
    accelData[2] = ((measurements[4] << 8) | measurements[5]);

    for(uint8_t i = 0; i < 3; i++)
    {
        MPU9250_CalcAvgValue(&MPU9250_Filter[i].Index, MPU9250_Filter[i].AvgBuffer, accelData[i], &outBuffer[i+1]);
    }

    accel.x = outBuffer[0];
    accel.y = outBuffer[1];
    accel.z = outBuffer[2];
}

/**
 * @brief This function reads the magnetoemeter output register
 * @attention This function uses the global I2C structure mpu9250_InputI2C
 * @return None
 */
void MPU9250_ReadMag(void)
{
    static int32_t outBuffer[3] = {0};
    static MPU9250_AvgTypeDef MPU9250_Filter[3];
    
    int16_t magData[3] = {0};
    uint8_t measurements[6];

    MPU9250_WriteRegister(MPU9250_I2C_SLV, MPU9250_BYPASS_EN, 0x02, 1); // turn on bypass mode
    // HAL_Delay(10);
    MPU9250_WriteRegister(MPU9250_I2C_MAG, MAG_CNTL1, 0x01, 1); // single measurement mode
    // HAL_Delay(10);

    MPU9250_ReadRegister(MPU9250_I2C_MAG, MAG_XOUT_L, measurements, 6);     // read the accel output registers

    // format the measurements to g's
    magData[0] = ((measurements[1] << 8) | measurements[0]);
    magData[1] = ((measurements[3] << 8) | measurements[2]);
    magData[2] = ((measurements[5] << 8) | measurements[4]);
    magData[2] = -magData[2];

    for(uint8_t i = 0; i < 3; i++)
    {
        MPU9250_CalcAvgValue(&MPU9250_Filter[i].Index, MPU9250_Filter[i].AvgBuffer, magData[i], &outBuffer[i+1]);
    }

    mag.x = outBuffer[0] - MPU9250_MagOffset.x;
    mag.y = outBuffer[1] - MPU9250_MagOffset.y;
    mag.z = outBuffer[2] - MPU9250_MagOffset.z;
}

/**
 * @brief This function calculates the average of the last 8 input values
 * @param currentIndex current index
 * @param pAvgBuffer store the last 8 values
 * @param inputValue new values
 * @param outputValue average values
 */
void MPU9250_CalcAvgValue(uint8_t *currentIndex, int16_t *pAvgBuffer, int16_t inputValue, int32_t *outputValue)
{
    // Update circular buffer with new input value
    pAvgBuffer[(*currentIndex)++] = inputValue;
    *currentIndex &= 0x07; // Ensure index is 0 to 7

    *outputValue = 0;

    // Calculate sum of values in the circular buffer
    for (uint8_t i = 0; i < 8; i++)
    {
        *outputValue += pAvgBuffer[i];
    }

    // Calculate the average by dividing the sum by 8
    *outputValue >>= 3;
}




