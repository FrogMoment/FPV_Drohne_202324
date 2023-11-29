/**
  ******************************************************************************
  * @file    Demo_MPU_9150.c
  * @author  Josef Reisinger
  * @version V1.0
  * @date    13.05.2023
  * @brief   Demoprogramm für I2C Funktionalitäten für ARM Cortex M3  
  * @History 13.05.2023 REJ V1.0 created
	* @note
  *  An den I2C1-Bus (PB6,PB7) wird der MPU 9150 angeschlossen.   
  *  Außerdem sind Pullup-Widerstände von 4,7kOhm an den I2C-Leitungen         
  *  empfehlenswert.                                                           
  */
#include <math.h>	
#include <armv30_std.h>
#include <armv30_i2c.h>
#include "SvVis3.h"
#include "MPU9150.h"

#define USART_LOG USART1
#define USART_SvVis USART2

#define sample_num_mdate  5000

/* ------------------------------- Function Prototypes -----------------*/
static void TIM1_Config(void);
static void Get_Mag_Calibration_Data(void);
static void Mag_Init_Calibrated(void);

/* -------------------------------- Init Global Variables ----------------------------*/
static int sekunden;
xyz acc;   // actual values of Acceleratometer
xyz gyro;   // actual values of Gyrosscope in rad/s
xyz mag;    // actual Values of Magnetometer
float temperature=0.0; // actual values of Temperature Sensor
xyz mag_sens;  // Sensitivity Adjustment Values for Magnetometer


xyz mag_centre;  // Compass Calibration Values
xyz mag_max;  
xyz mag_min;  


float mx_sample[3]; // Magnetometer calibration parameter
float my_sample[3];
float mz_sample[3];


float sampling_time = 0.1;     // Sampling time 100ms
float pitch, bank = 0.0;
float pitch_acc, bank_acc = 0.0;
float pitch_gyro, bank_gyro = 0.0;


#define COMP_GYRO_COEFF 0.95
#define RAD_TO_DEG 57.29577951 /* 180/pi*/

/******************************************************************************/
/*           Interrupt Service Routine  Timer1 (General Purpose Timer)        */
/******************************************************************************/
void TIM1_UP_IRQHandler(void)	//Timer 1, löst alle 1000ms aus
{
	TIM_ClearFlag(TIM1,TIM_FLAG_Update); // Clear Update Interrupt Flag (UIF) Flag
	LED0=~LED0;
	sekunden++;
}

/**
  *   @brief Configure Timer 1, Intertupt every 1s
  *                                                                            
  *   @return none                                
  */
static void TIM1_Config(void)
{
TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
NVIC_InitTypeDef nvic;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	TIM_DeInit(TIM1);
	TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	/* T_INT = 13,8ns, Annahme: Presc = 1350 ---> Auto Reload Wert = 52801 (=0xCE41) --> 1s Update Event*/
	TIM_TimeBase_InitStructure.TIM_Period = 0xCE41;	//Auto-Reload Wert = Maximaler Zaehlerstand des Upcounters
	//TIM_TimeBase_InitStructure.TIM_Prescaler = 1350; //Wert des prescalers (Taktverminderung)
	TIM_TimeBase_InitStructure.TIM_Prescaler = 135; //Wert des prescalers (Taktverminderung)
	TIM_TimeBase_InitStructure.TIM_RepetitionCounter = 0; //Repetition Counter deaktivieren
	TIM_TimeBaseInit(TIM1, &TIM_TimeBase_InitStructure);
		
	TIM_ITConfig (TIM1, TIM_DIER_UIE,ENABLE);   // Update Interrupt Enable 

	// Init NVIC for Timer 1 Update Interrupt 
	nvic.NVIC_IRQChannel = TIM1_UP_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&nvic);

	TIM_Cmd(TIM1, ENABLE); //Counter-Enable bit (CEN) setzen
}


/**
  * @brief Calibration of Compass(Magentometer)
  * @retval I2C error code
  */
static void Mag_Init_Calibrated() {
char c;
	
	UARTx_printf(USART_LOG,"Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes.\n");
	UARTx_printf(USART_LOG,"During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes.\n");
	UARTx_printf(USART_LOG,"If you are ready ,please press any character to start sample and calibrate.\n");
  //c = UARTx_get_char(USART_LOG);
	UARTx_printf(USART_LOG,"Sampling started in 5s......");
	wait_ms(5000);
	UARTx_printf(USART_LOG,"Calibrating ......");
	
	Get_Mag_Calibration_Data();  // Get Compass Calibration data
	
	UARTx_printf(USART_LOG,"Compass Calibration Parameter ");
	UARTx_printf(USART_LOG,"Centre x = %2.2f",mag_centre.x);
	UARTx_printf(USART_LOG,"Centre y = %2.2f",mag_centre.y);
	UARTx_printf(USART_LOG,"Centre z = %2.2f",mag_centre.z);

	wait_ms(5000);
	
}


/**
  * @brief Get Calibration Data of Magentometer
  * @retval I2C error code
  */
static void Get_Mag_Calibration_Data(void)
{
char ret;
xyz mag_sample;	
float mx_sample[3];
float my_sample[3];
float mz_sample[3];

	
		for (int i=0; i<sample_num_mdate;i++)			{	
			/* --------  Get one sample from magnetometer ---- */	
      ret = MPU9150_ReadMagData(I2C1, mag_sens, &mag_sample);				
		  mx_sample[2] = mag_sample.x;
		  my_sample[2] = mag_sample.y;
		  mz_sample[2] = mag_sample.z;

			
			if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];			
			if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //find max value			
			if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];		
			
			if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
			if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//find min value
			if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];	

			wait_ms(10);
			}
			
			mag_max.x = mx_sample[1];
			mag_max.y = my_sample[1];
			mag_max.z = mz_sample[1];			
					
			mag_min.x = mx_sample[0];
			mag_min.y = my_sample[0];
			mag_min.z = mz_sample[0];
	
			mag_centre.x = (mag_max.x + mag_min.x)/2;
			mag_centre.y = (mag_max.y + mag_min.y)/2;
			mag_centre.z = (mag_max.z + mag_min.z)/2;	
	
}

/**
  *   @brief Hauptprogramm                            
  *          Liest Werte Beschleunigungswerte und Gyrowerte des MPU 9150 aus
  *                                                                            
  *   @return none                                
  */

int main (void)
{
	char ret;
	int log_sec;
static float acc_pitch_angle = 0.0;	
static float acc_bank_angle = 0.0;	
	
  //unsigned char buffer[100];
	

  init_leds_switches();
	set_leds(0x0);

	UARTx_Init(USART_LOG,115200,8,1,NO_PARITY);
	UARTx_put_string(USART_LOG,"\n\r --------- MPU Demo V1.0 ---------\n\r");

	SvVis3_Init(USART_SvVis,115200);
	SvVis3_Message("MPU9150-Demo V1.0");
	
	/*----------------------------- Init MPU 9150 -----------------------------*/
	ret = MPU9150_Init(I2C1,DLPF_250_Hz);
	if (ret != 0) {
		goto exit;
	  }
	ret = MPU9150_Accel_Config(I2C1,MR_2_g);
	if (ret != 0) {
		goto exit;
	  }
	ret = MPU9150_Gyro_Config(I2C1,MR_250);
	if (ret != 0) {
		goto exit;
	  }
  
	mag_sens.x=0;  mag_sens.y=0;	mag_sens.z=0;	
	ret = MPU9150_Mag_Config(I2C1,&mag_sens);
	if (ret != 0) {
		goto exit;
	  }
	
  acc.x=0; acc.y=0; 	acc.z=0;
	gyro.x=0; gyro.y=0; gyro.z=0;
	mag.x=0;  mag.y=0;	mag.z=0;
	
  Mag_Init_Calibrated(); // Calibrate Compass (Magnetometer)
		
	TIM1_Config();   // Configure Timer1, Interrupt every 1s
		
	for (;;)
	{
		if (log_sec != sekunden) {
		  log_sec = sekunden;	
      ret=MPU9150_Read_All(I2C1,&acc, &gyro, &temperature); 
	    if (ret != 0) {
		    goto exit;
	      }
			/* --------  read values of magnetometer ---- */	
      ret = MPU9150_ReadMagData(I2C1, mag_sens, &mag);				
	    if (ret != 0) {
		    goto exit;
	      }
				
	    UARTx_printf(USART_LOG,"aX = %2.2f", acc.x);
	    UARTx_printf(USART_LOG," | aY = %2.2f", acc.y);
	    UARTx_printf(USART_LOG," | aZ = %2.2f\r\n", acc.z);
	
				
	    UARTx_printf(USART_LOG,"gX = %2.2f", gyro.x);
	    UARTx_printf(USART_LOG," | gY = %2.2f", gyro.y);
	    UARTx_printf(USART_LOG," | gZ = %2.2f\r\n", gyro.z);

			UARTx_printf(USART_LOG,"mX = %2.2f", mag.x);
	    UARTx_printf(USART_LOG," | mY = %2.2f", mag.y);
	    UARTx_printf(USART_LOG," | mZ = %2.2f\r\n", mag.z);
					
	    UARTx_printf(USART_LOG,"temp = %2.2f\r\n", temperature);

      acc.z = -acc.z;    // Sensor Up Side Down
      if (acc.z != 0) {
				acc_pitch_angle = RAD_TO_DEG * atan(acc.x/acc.z);
				acc_bank_angle = - RAD_TO_DEG*atan(acc.y/acc.z);
				//acc_pitch_angle = RAD_TO_DEG * atan2(acc.x,acc.z);
				//acc_bank_angle = - RAD_TO_DEG*atan2(acc.y,acc.z);
			 }
      pitch = COMP_GYRO_COEFF*(pitch + gyro.y*sampling_time) + (1.0 - COMP_GYRO_COEFF)*acc_pitch_angle;
			bank = COMP_GYRO_COEFF*(bank + gyro.x*sampling_time) + (1.0 - COMP_GYRO_COEFF)*acc_bank_angle;
    
			pitch_gyro += gyro.y*sampling_time; 
			bank_gyro += gyro.x*sampling_time; 
			 
			UARTx_printf(USART_LOG,"pitch = %2.2f", pitch);
	    UARTx_printf(USART_LOG," | bank = %2.2f\r\n", bank);
			 
			SvVis3_WriteSV2(1,pitch);
			SvVis3_WriteSV2(2,bank);
		  SvVis3_WriteSV2(3,pitch_gyro);
		  SvVis3_WriteSV2(4,bank_gyro);
		  }	
		wait_ms(10);	
	}	
	
exit:
	UARTx_printf(USART_LOG, "Error:%x\n", ret);

end:	
	UARTx_printf(USART_LOG, "END\n");
	while(1){};
	  
}

