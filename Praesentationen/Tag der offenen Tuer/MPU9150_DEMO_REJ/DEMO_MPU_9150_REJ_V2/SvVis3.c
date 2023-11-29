//
//	(C) Copyright HTL - HOLLABRUNN  2009-2009 All rights reserved. AUSTRIA   
//                                                                            
// 	File Name:  SvVis3.c	                                                 
// 	Autor: 			Patrick Weißkirchner                                        
// 	Version: 		V1.00                                                        
// 	Date: 			21/12/2012                                                    
// 	Description:  Library zur Kommunikation mit PC-Software "SvVis3" zu Visualisierung von Messdaten			                                      
//
// 	History: 		V1.00  creation
//					    V1.1   REJ: Convert to Maier Jakob UART Library (uart_lib.c)
//					    V1.2   REJ: Convert to ST-Firmware Library
//              V1.3   22.05.2023 REJ: Convert to REJ UART Library (ARMV30_HTL_PACK_V32)	
#define SVVIS3_MOD
#include <armv30_std.h>
#include "SvVis3.h"


/* -------------------------------- Gloabl Varibales -------------------------*/
uint8_t acqON = 0;
USART_TypeDef* SvVisUARTx=0;


/* ----------------------------Function Prototypes -------------------------*/
static void Puts(char* string);
static void write(void* aData, uint32_t aLenBytes); 
static void read(void* aData, uint32_t aLenBytes);

static char SvVis3_UARTx_get_char(void); 		
static void SvVis3_UARTx_put_char(char c);
static void SvVis3_UARTx_put_string(char *string); 						
static void SvVis3_UARTx_clear(void);
static void SvVis3_UARTx_saveCursorPos(void);									
static void SvVis3_UARTx_restoreCursorPos(void);
static void SvVis3_UARTx_setpos(char x,char y);
static void SvVis3_UARTx_printf(const char *format, ...);

/******************************************************************************/
/*                 Initialisiert UARTx für SVis3 Übertragung                  */
/******************************************************************************/
void SvVis3_Init(USART_TypeDef* UARTx, unsigned long baudrate)
{
	UARTx_Init(UARTx,baudrate,8,1,NO_PARITY);
	SvVisUARTx = UARTx;
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
static void Puts(char* string)
{
	while(*string != '\0')	{
		if(*string == '\n')	{
			SvVis3_UARTx_put_char('\r'); // REJ:
			//put_char('\r');
			}
		SvVis3_UARTx_put_char(*string); // REJ:
		//put_char(*string);
		string++;
		}
	SvVis3_UARTx_put_char(0); // REJ:
	//put_char(0);
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
static void write(void* aData, uint32_t aLenBytes) 
{
	int i;
  	uint8_t* ptr = (uint8_t*)aData;

  	for(i=0; i<aLenBytes; i++) 	{
	    SvVis3_UARTx_put_char(*ptr); // REJ:
    	//put_char(*ptr); 
		ptr++;
  		}
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
static void read(void* aData, uint32_t aLenBytes)
{
	int i;
  	uint8_t* ptr = (uint8_t*)aData;

  	for(i=0; i<aLenBytes; i++)	{
		*ptr = SvVis3_UARTx_get_char();  // REJ:
    	//*ptr=get_char(); 
		ptr++;
  		}
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
char SvVis3_Get_Command(void) 
{
char cmd;
	
	//char cmd = get_char();
	if(UARTx_received(SvVisUARTx) !=0 ) {
 		cmd = SvVis3_UARTx_get_char();  // REJ:

		if(cmd==1)
		{
			acqON = SvVis3_UARTx_get_char();  // REJ:
			//acqON = get_char();
			if(acqON)
			{
				SvVis3_Message("AcqON");
			}
			else
			{
				SvVis3_Message("AcqOFF");
			}
			return(0);
		}
		return(cmd);
	}
	else
	{
		return 0;
	}
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
void SvVis3_Message(char* aTxt) 
{
	SvVis3_UARTx_put_char(10); // REJ:
  	//put_char(10); 
	Puts(aTxt);
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
void SvVis3_WriteSV3p13(uint8_t aId, float aData) 
{
  	int16_t val = aData;	 // hoffentlich funktioniert das auch

	SvVis3_UARTx_put_char(aId); // REJ:
  	//put_char(aId); 
  	write(&val,2);
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
void SvVis3_Write(uint8_t aId, char* aData) 
{
    SvVis3_UARTx_put_char(aId); // REJ:
  	//put_char(aId); 
	Puts(aData); 
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
void SvVis3_VectHeader(uint8_t aId, uint8_t aNVals)
{ 
	SvVis3_UARTx_put_char(aId); // REJ:
	//put_char(aId); 
	SvVis3_UARTx_put_char(aNVals); // REJ:
	//put_char(aNVals);
}
    
/******************************************************************************/
/*                                                                            */
/******************************************************************************/
void SvVis3_Write_I16(uint8_t aId, int16_t aData)
{ 
	SvVis3_UARTx_put_char(aId+10); // REJ:
	//put_char(aId+10); 
	write(&aData,2); 
}
    
/******************************************************************************/
/*                                                                            */
/******************************************************************************/
void SvVis3_Write_I32(uint8_t aId, int32_t aData)
{ 
	SvVis3_UARTx_put_char(aId); // REJ:
	//put_char(aId); 
	write(&aData,4); 
}
    
/******************************************************************************/
/*                                                                            */
/******************************************************************************/
void SvVis3_WriteSV2(uint8_t aId, float aData)
{ 
	SvVis3_UARTx_put_char(aId+20); // REJ:
	//put_char(aId+20); 
	write(&aData,4); 
}
    
/******************************************************************************/
/*                                                                            */
/******************************************************************************/
int16_t SvVis3_Read_I16()
{ 
int16_t ret; 

	read(&ret,2); 
	return(ret); 
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
int32_t SvVis3_Read_I32()
{ 
int32_t ret; 

	read(&ret,4); 
	return ret; 
}
    
/******************************************************************************/
/*                                                                            */
/******************************************************************************/
float SvVis3_ReadF()
{ 
float ret; 

	read(&ret,4); 
	return ret; 
}    


//---------------------------- internal functions --------------------------------

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
static char SvVis3_UARTx_get_char(void) 		//liest ein Zeichen von UART1 ein
{
	while(UARTx_received(SvVisUARTx) == 0){}; 
  	return (UARTx_get_char(SvVisUARTx));
}



/******************************************************************************/
/*                                                                            */
/******************************************************************************/
static void SvVis3_UARTx_put_char(char c) 		//gibt ein Zeichen auf  UART1 aus
{
	//while(USART_GetFlagStatus(SvVisUARTx, USART_FLAG_TC) == RESET);	
	UARTx_put_char(SvVisUARTx,c);
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
static void SvVis3_UARTx_put_string(char *string) 	//gibt einen Text auf einem der 5 UARTs aus
{
  	while (*string)
	{
		SvVis3_UARTx_put_char(*string++);  //solange ein Zeichen im String ist Zeichen ausgeben und Pointer erhöhen	
	}
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
static void SvVis3_UARTx_clear()					//löscht den Inhalt der Konsole
{
	SvVis3_UARTx_put_string("\x1b[2J");				//Sendet diese Zeichenfolge
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
static void SvVis3_UARTx_saveCursorPos()										//speichert die derzeitige Cursorposition
{
	SvVis3_UARTx_put_string("\x1b[s");												//Sendet diese Zeichenfolge
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
static void SvVis3_UARTx_restoreCursorPos()									//geht zur gespeicherten Cursorposition
{
	SvVis3_UARTx_put_string("\x1b[u");												//Sendet diese Zeichenfolge
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
static void SvVis3_UARTx_setpos(char x,char y)								//Setzt den Cursor eines UARTs auf eine bestimmte Position
{
	SvVis3_UARTx_printf("\x1b[%d;%dH",y,x);											//Sendet die Escape-Sequenz
}

/******************************************************************************/
/*                                                                            */
/******************************************************************************/
static void SvVis3_UARTx_printf(const char *format, ...)						//UART-Printf - ACHTUNG: max. Länge!
{
	static char buffer[1024];												//Buffer
	va_list  argptr;														//Argument-Liste
	va_start( argptr, format );
	vsprintf( buffer, format, argptr );										//Mit sprintf-Funktion in Buffer übertragen
	va_end  ( argptr );
	buffer[1024-1]=0;														//Zur Sicherheit
	SvVis3_UARTx_put_string(buffer);
}

