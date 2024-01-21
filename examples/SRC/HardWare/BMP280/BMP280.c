/**
  ******************************************************************************
  * @file    BMP280.c
  * @author  Waveshare Team
  * @version V1.0
  * @date    12-12-2016
  * @brief   This file includes the BMP280 driver functions
  
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  ******************************************************************************
  */
#include "BMP280.h"

#define dig_T1 bmp280.T1  
#define dig_T2 bmp280.T2  
#define dig_T3 bmp280.T3  
#define dig_P1 bmp280.P1  
#define dig_P2 bmp280.P2  
#define dig_P3 bmp280.P3  
#define dig_P4 bmp280.P4  
#define dig_P5 bmp280.P5  
#define dig_P6 bmp280.P6  
#define dig_P7 bmp280.P7  
#define dig_P8 bmp280.P8  
#define dig_P9 bmp280.P9  
#define t_fine bmp280.T_fine  

BMP280_HandleTypeDef bmp280;
int32_t gs32Pressure0 = MSLP; 

void BMP280_ReadReg(uint8_t RegAddr, uint8_t Num, uint8_t *pBuffer) 
{
  	I2C_ReadBuff(BMP280_ADDR, RegAddr, Num, pBuffer);
}
void BMP280_WriteReg(uint8_t RegAddr, uint8_t Val) 
{
	I2C_WriteOneByte(BMP280_ADDR, RegAddr, Val);
}

void BMP280_Read_Calibration(void)
{
	uint8_t lsb, msb; 
	
	/* read the temperature calibration parameters */  
    BMP280_ReadReg(BMP280_DIG_T1_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_T1_MSB_REG, 1, &msb);
	dig_T1 = msb << 8 | lsb;  
    BMP280_ReadReg(BMP280_DIG_T2_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_T2_MSB_REG, 1, &msb);
	dig_T2 = msb << 8 | lsb;  
    BMP280_ReadReg(BMP280_DIG_T3_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_T3_MSB_REG, 1, &msb);
	dig_T3 = msb << 8 | lsb;  
	
	/* read the pressure calibration parameters */  
    BMP280_ReadReg(BMP280_DIG_P1_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P1_MSB_REG, 1, &msb);    
	dig_P1 = msb << 8 | lsb;  
    BMP280_ReadReg(BMP280_DIG_P2_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P2_MSB_REG, 1, &msb);      
	dig_P2 = msb << 8 | lsb;  
    BMP280_ReadReg(BMP280_DIG_P3_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P3_MSB_REG, 1, &msb);
	dig_P3 = msb << 8 | lsb;  
    BMP280_ReadReg(BMP280_DIG_P4_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P4_MSB_REG, 1, &msb);    
	dig_P4 = msb << 8 | lsb;  
    BMP280_ReadReg(BMP280_DIG_P5_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P5_MSB_REG, 1, &msb);     
	dig_P5 = msb << 8 | lsb;  
    BMP280_ReadReg(BMP280_DIG_P6_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P6_MSB_REG, 1, &msb);     
	dig_P6 = msb << 8 | lsb;  
    BMP280_ReadReg(BMP280_DIG_P7_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P7_MSB_REG, 1, &msb);      
	dig_P7 = msb << 8 | lsb;  
    BMP280_ReadReg(BMP280_DIG_P8_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P8_MSB_REG, 1, &msb);     
	dig_P8 = msb << 8 | lsb;  
    BMP280_ReadReg(BMP280_DIG_P9_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P9_MSB_REG, 1, &msb);      
	dig_P9 = msb << 8 | lsb; 

//	printf("dig_T1 = %d\r\n",dig_T1);
//	printf("dig_T2 = %d\r\n",dig_T2);
//	printf("dig_T3 = %d\r\n",dig_T3);
//	printf("dig_P1 = %d\r\n",dig_P1);
//	printf("dig_P2 = %d\r\n",dig_P2);
//	printf("dig_P3 = %d\r\n",dig_P3);
//	printf("dig_P4 = %d\r\n",dig_P4);
//	printf("dig_P5 = %d\r\n",dig_P5);
//	printf("dig_P6 = %d\r\n",dig_P6);
//	printf("dig_P7 = %d\r\n",dig_P7);
//	printf("dig_P8 = %d\r\n",dig_P8);
//	printf("dig_P9 = %d\r\n",dig_P9);

}
/* Returns temperature in DegC, double precision. Output value of "1.23"equals 51.23 DegC. */  
double BMP280_Compensate_Temperature(int32_t adc_T)  
{  
	double var1, var2, temperature;  
	var1 = (((double) adc_T) / 16384.0 - ((double) dig_T1) / 1024.0) * ((double) dig_T2); 
	var2 = ((((double) adc_T) / 131072.0 - ((double) dig_T1) / 8192.0)  * (((double) adc_T) / 131072.0  
					- ((double) dig_T1) / 8192.0)) * ((double) dig_T3);  
	t_fine = (int32_t) (var1 + var2);  
	temperature = (var1 + var2) / 5120.0;  
	
	return temperature;  
}  
  
  
/* Returns pressure in Pa as double. Output value of "6386.2"equals 96386.2 Pa = 963.862 hPa */  
double BMP280_Compensate_Pressure(int32_t adc_P)  
{  
	double var1, var2, pressure;  

	var1 = ((double)t_fine / 2.0) - 64000.0; 
	var2 = var1 * var1 * ((double) dig_P6) / 32768.0;  
	var2 = var2 + var1 * ((double) dig_P5) * 2.0;  
	var2 = (var2 / 4.0) + (((double) dig_P4) * 65536.0);  
	var1 = (((double) dig_P3) * var1 * var1 / 524288.0  + ((double) dig_P2) * var1) / 524288.0;  
	var1 = (1.0 + var1 / 32768.0) * ((double) dig_P1);  

	if (var1 == 0.0) {  
		return 0; // avoid exception caused by division by zero  
	}  

	pressure = 1048576.0 - (double) adc_P;  
	pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;  
	var1 = ((double) dig_P9) * pressure * pressure / 2147483648.0;  
	var2 = pressure * ((double) dig_P8) / 32768.0;  
	pressure = pressure + (var1 + var2 + ((double) dig_P7)) / 16.0;  

	return pressure;  
}  
void BMP280_Get_Temperature_And_Pressure(double *temperature, double *pressure)  
{  
	uint8_t lsb, msb, xlsb;  
	int32_t adc_P,adc_T;
    
    BMP280_ReadReg(BMP280_TEMP_XLSB_REG, 1, &xlsb);
    BMP280_ReadReg(BMP280_TEMP_LSB_REG,  1, &lsb);
    BMP280_ReadReg(BMP280_TEMP_MSB_REG,  1, &msb);
	adc_T = (msb << 12) | (lsb << 4) | (xlsb >> 4); 
	//adc_T = 415148;
	* temperature = BMP280_Compensate_Temperature(adc_T);

    BMP280_ReadReg(BMP280_PRESS_XLSB_REG, 1, &xlsb);
    BMP280_ReadReg(BMP280_PRESS_LSB_REG,  1, &lsb);
    BMP280_ReadReg(BMP280_PRESS_MSB_REG,  1, &msb);
	adc_P = (msb << 12) | (lsb << 4) | (xlsb >> 4); 
	//adc_P = 51988;
	* pressure = BMP280_Compensate_Pressure(adc_P);     
} 

void BMP280_CalAvgValue(uint8_t *pIndex, int32_t *pAvgBuffer, int32_t InVal, int32_t *pOutVal)
{	
	uint8_t i;
	
  	*(pAvgBuffer + ((*pIndex) ++)) = InVal;
  	*pIndex &= 0x07;
  	
  	*pOutVal = 0;
	for(i = 0; i < 8; i ++) 
  	{
    	*pOutVal += *(pAvgBuffer + i);
  	}
  	*pOutVal >>= 3;
}

void BMP280_CalculateAbsoluteAltitude(int32_t *pAltitude, int32_t PressureVal)
{
	*pAltitude = 4433000 * (1 - pow((PressureVal / (float)gs32Pressure0), 0.1903)); 
}

extern uint8_t BMP280_Init()
{
    uint8_t u8Ret = BMP280_RET_OK;
    uint8_t u8ChipID, u8CtrlMod, u8Status;

    BMP280_ReadReg(BMP280_REGISTER_CHIPID, 1, &u8ChipID);
    BMP280_ReadReg(BMP280_REGISTER_CONTROL, 1, &u8CtrlMod);
    BMP280_ReadReg(BMP280_REGISTER_STATUS, 1, &u8Status);

    if(u8ChipID == 0x58)
    {
        printf("\r\nBMP280 init successful : ChipID [0x%x] CtrlMod [0x%x] Status [0x%x] \r\n", u8ChipID,u8CtrlMod,u8Status);
        BMP280_WriteReg(BMP280_REGISTER_CONTROL, 0xFF); 
        BMP280_WriteReg(BMP280_REGISTER_CONFIG, 0x14); 
        BMP280_Read_Calibration();

    }
    else
    {
        u8Ret = BMP280_RET_NG;
    }
    return  u8Ret;
}


extern void BMP280_CalTemperatureAndPressureAndAltitude(int32_t *temperature, int32_t *pressure, int32_t *Altitude)
{
    double CurPressure, CurTemperature;
    int32_t CurAltitude;
    static BMP280_AvgTypeDef BMP280_Filter[3];
    
    BMP280_Get_Temperature_And_Pressure(&CurTemperature, &CurPressure);
    BMP280_CalAvgValue(&BMP280_Filter[0].Index, BMP280_Filter[0].AvgBuffer, (int32_t)(CurPressure), pressure);

    BMP280_CalculateAbsoluteAltitude(&CurAltitude, (*pressure));
    BMP280_CalAvgValue(&BMP280_Filter[1].Index, BMP280_Filter[1].AvgBuffer, CurAltitude, Altitude);
    BMP280_CalAvgValue(&BMP280_Filter[2].Index, BMP280_Filter[2].AvgBuffer, (int32_t)CurTemperature*10, temperature);
    
    return;
}

