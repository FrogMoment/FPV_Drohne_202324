/**
 * @file IMU_10DOF.c
 * @author Maximilian Lendl
 * @date 2023-11-18
 * @version 1
 * 
 * @copyright FPV Drohne DA 202324
 * 
 * @brief 
 * 
 */

#include "IMU_10DOF.h"

/**********************************************************************
--------------------------- GLOBAL VARIABLES ---------------------------
**********************************************************************/

I2C_HandleTypeDef *imu_ComI2C;
TIM_HandleTypeDef *imu_DelayTIM;

float accelSens = 0;
float gyroSens = 0;
IMU_Coordinates gyroOffset = {0};

uint8_t magAdjust[3] = {0};

IMU_Coordinates accel = {0};
IMU_Coordinates gyro = {0};
IMU_Coordinates mag = {0};

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
 * @brief This function initialzes the 10DOF IMU
 * @param hi2c communication I2C, pointer to I2C_HandleTypeDef
 * @param gyroFS full scale select for gyroscope (GYROGYRO_250DPS / GYRO_500DPS / GYRO_1000DPS / GYRO_2000DPS)
 * @param accelFS full scale selct for accelerometer (ACCEL_2G / ACCEL_4G / ACCEL_8G / ACCEL_16G)
 * @param gyroDLPF bandwidth of gyroscope digital low pass filter (GYRO_DLPF_250HZ, GYRO_DLPF_184HZ, GYRO_DLPF_92HZ, GYRO_DLPF_41HZ)
 * @param accelDLPF bandwidth of accelerometer digital low pass filter (ACCEL_DLPF_460HZ, ACCEL_DLPF_184HZ, ACCEL_DLPF_92HZ, ACCEL_DLPF_41HZ)
 * @param htim us Delay timer, pointer to TIM_HandleTypeDef
 * @return IMU_Status 
 */
IMU_Status IMU_Init(I2C_HandleTypeDef *hi2c, IMU_Fullscale gyroFS, IMU_Fullscale accelFS, IMU_DLPF gyroDLPF, IMU_DLPF accelDLPF, TIM_HandleTypeDef *htim)
{
    imu_ComI2C = hi2c;
    if(imu_ComI2C == NULL)
        return IMU_I2C_ERROR;

    imu_DelayTIM = htim;
    if(imu_DelayTIM == NULL)
        return IMU_TIM_ERROR;
        
    HAL_TIM_Base_Start(imu_DelayTIM); // start delay timer

    
    uint8_t errorCode;

    // check if all IMU sensors are connected
    errorCode = IMU_CheckConnection();
    if(errorCode != IMU_OK)
        return errorCode;

    IMU_WriteRegister(MPU9250, IMU_MPU_PWR_MGMT_1_ADDR, 0x00); // reset MPU
    IMU_DelayUs(10000);
    IMU_WriteRegister(MPU9250, IMU_MPU_PWR_MGMT_1_ADDR, 0x01);
    IMU_WriteRegister(MPU9250, IMU_MPU_PWR_MGMT_2_ADDR, 0x00);
    IMU_WriteRegister(MPU9250, IMU_MPU_ACCEL_CONFIG_ADDR, accelFS << 3);
    IMU_WriteRegister(MPU9250, IMU_MPU_GYRO_CONFIG_ADDR, gyroFS << 3);
    IMU_WriteRegister(MPU9250, IMU_MPU_ACCEL_CONFIG_2_ADDR, accelDLPF);
    IMU_WriteRegister(MPU9250, IMU_MPU_CONFIG_ADDR, gyroDLPF);
    IMU_WriteRegister(MPU9250, IMU_MPU_SMPLRT_DIV_ADDR, 0x00);

    accelSens = IMU_ACCEL_RES_MAX / (1 << accelFS);
    gyroSens = IMU_GYRO_RES_MAX / (1 << gyroFS);

    // calibrate gyro
    uint16_t amount = 1500;
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

    // AK8963 init
    IMU_WriteRegister(AK8963, IMU_MAG_CNTL1_ADDR, 0x00);
    IMU_DelayUs(10000);
    IMU_WriteRegister(AK8963, IMU_MAG_CNTL2_ADDR, 0x01);

    // magnetometer calibration
    IMU_WriteRegister(AK8963, IMU_MAG_CNTL1_ADDR, 0x00);
    HAL_Delay(100);
    IMU_WriteRegister(AK8963, IMU_MAG_CNTL1_ADDR, 0x0F);
    HAL_Delay(100);
    IMU_ReadRegister(AK8963, IMU_MAG_ASAX_ADDR, magAdjust, 3);
    IMU_WriteRegister(AK8963, IMU_MAG_CNTL1_ADDR, 0x00);
    HAL_Delay(100);
    IMU_WriteRegister(AK8963, IMU_MAG_CNTL1_ADDR, 0x16);
    HAL_Delay(100);
    IMU_WriteRegister(MPU9250, IMU_MPU_PWR_MGMT_1_ADDR, 0x01);

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
        case MAG:
            devAddress = IMU_MAG_I2C_ADDR;
            break;
        
        case BMP280:
        case BARO:
            devAddress = IMU_BARO_I2C_ADDR;
            break;

        default:
            return IMU_ADDRESS_ERROR;
    }

    // read register(s)
    HAL_I2C_Mem_Read(imu_ComI2C, devAddress, regAddr, I2C_MEMADD_SIZE_8BIT, data, rxBytes, HAL_MAX_DELAY);

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
        case MAG:
            devAddress = IMU_MAG_I2C_ADDR;
            break;
        
        case BMP280:
        case BARO:
            devAddress = IMU_BARO_I2C_ADDR;
            break;

        default:
            return IMU_ADDRESS_ERROR;
    }

    // read register(s)
    HAL_I2C_Mem_Write(imu_ComI2C, devAddress, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

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
    uint8_t data;

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
 * @return IMU_RegCoordinates 1
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
    accelData.x = ((int16_t)buffer[0] << 8) | (int16_t)buffer[1];
    accelData.y = ((int16_t)buffer[2] << 8) | (int16_t)buffer[3];
    accelData.z = ((int16_t)buffer[4] << 8) | (int16_t)buffer[5];

    return accelData;
}

/**
 * @brief This function reads magnetometer register data (x,y,z)
 * @return IMU_RegCoordinates 
 */
IMU_RegCoordinates IMU_MAG_ReadMag(void)
{
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
    IMU_RegCoordinates gyroData = IMU_MPU_ReadGyro();
    IMU_RegCoordinates accelData = IMU_MPU_ReadAccel();
    IMU_RegCoordinates magData = IMU_MAG_ReadMag();

    gyro.x = (gyroData.x - gyroOffset.x) / gyroSens;
    gyro.y = (gyroData.y - gyroOffset.y) / gyroSens;
    gyro.z = (gyroData.z - gyroOffset.z) / gyroSens;

    accel.x = accelData.x / accelSens;
    accel.y = accelData.y / accelSens;
    accel.z = accelData.z / accelSens;

    mag.x = (float)magData.x * ((((float)magAdjust[0] - 128.0f) / 256.0f) + 1.0f);
    mag.y = (float)magData.y * ((((float)magAdjust[1] - 128.0f) / 256.0f) + 1.0f);
    mag.z = (float)magData.z * ((((float)magAdjust[2] - 128.0f) / 256.0f) + 1.0f);

    // Complementary filter
    float accelPitch = atan2(accel.y, accel.z) * RAD2DEG;
    float accelRoll = atan2(accel.x, accel.z) * RAD2DEG;
    float dt = 0.01f;

    angle.roll = 0.98 * (angle.roll - gyro.y * dt) + (1 - 0.98) * accelRoll;
    angle.pitch = 0.98 * (angle.pitch + gyro.x * dt) + (1 - 0.98) * accelPitch;
    angle.yaw += gyro.z * dt;
}


/**
 * @brief This function reads the temperature and pressure compensation values 
 * @details values get stored in variable "baroCompensation"
 * @retval None
 */
/*void IMU_BARO_ReadCompensationValues(void)
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
}*/








