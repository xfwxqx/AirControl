/******************************************************************************
 * @file     	N74HC595.c
 * @version  	V1.00
 * $Revision: 3 $
 * $Date: 		17/07/03 11:45a $
 * @brief    	74HC595 chip
 * @auther		ljj
 * @note
 * Copyright (C) 2017 Jsyaao Technology Corp. All rights reserved.
*****************************************************************************/
#include "N74HC595.h"


unsigned char HC595_LEDCode[] = {0xFC,0x60,0xDA,0xF2,0x66,0xB6,0xBE,0xE0,0xFE,0xF6,0xEE,0x3E,0x9C,0x7A,0x9E,0x8E,0x80,0x1,0x2,0x4,0x88,0x10,0x20,0xC0,0x80,0x90,0x98};   
static unsigned int HC595LedVal;

	
static void HC595_SendData(unsigned char k)  
{  
    unsigned char i,data,j;  

    for(i=0;i<8;i++)  
    {  
        data=k&0x01;
		
        if(data)
            P06 = 0;
        else
            P06 = 1;

        P07 = 0;	
        for(j=0;j<30;j++)
        ; 
        P07 = 1; 
        k>>=1;  
    }  
}
//学习灯状态控制:mode=0时，根据IsRun控制学习灯亮灭；mode=1时，学习灯闪烁；
extern void LearnLightCtrl(char mode,char IsRun)
{
		/*uint8_t  temp_data,j;
	
		P20 = 0;
   
		if(IsRun)
				HC595_SendData(HC595_LEDCode[26]&0xff);
		else
				HC595_SendData(HC595_LEDCode[25]&0xff);
		
		temp_data	= (HC595LedVal%100)/10;
		HC595_SendData(HC595_LEDCode[temp_data]&0xff);
		
		temp_data	= (HC595LedVal%1000)/100;
		HC595_SendData(HC595_LEDCode[temp_data]&0xff);
		
		temp_data	= HC595LedVal/1000;
		HC595_SendData(HC595_LEDCode[temp_data]&0xff);
		
		for(j=0;j<30;j++)
				;  
		P20 = 1;
		for(j=0;j<120;j++)
				;     */
		if(mode==0){
			GPIO_SetMode(P0, BIT2, GPIO_PMD_OUTPUT);
        	P02 = (IsRun?0x0:0x01);
		}else if(mode==1){
			GPIO_SetMode(P0, BIT2, GPIO_PMD_OUTPUT);
        	P02 = !P02;
		}
		
}
extern void RunLightCtrl(char IsRun)
{
		/*uint8_t  temp_data,j;
	
		P20 = 0;
   
		if(IsRun)
				HC595_SendData(HC595_LEDCode[20]&0xff);
		else
				HC595_SendData(HC595_LEDCode[16]&0xff);
		
		temp_data	= (HC595LedVal%100)/10;
		HC595_SendData(HC595_LEDCode[temp_data]&0xff);
		
		temp_data	= (HC595LedVal%1000)/100;
		HC595_SendData(HC595_LEDCode[temp_data]&0xff);
		
		temp_data	= HC595LedVal/1000;
		HC595_SendData(HC595_LEDCode[temp_data]&0xff);
		
		for(j=0;j<30;j++)
				;  
		P20 = 1;
		for(j=0;j<120;j++)
				; */
    	GPIO_SetMode(P2, BIT7, GPIO_PMD_OUTPUT);
        P27 = (IsRun?0x0:0x01);
}
extern void FailureLightCtrl(char IsRun)
{
    	GPIO_SetMode(P0, BIT0, GPIO_PMD_OUTPUT);
        P00 = (IsRun?0x0:0x01);
}
extern void CommunicationLightCtrl(char IsRun)
{
    	GPIO_SetMode(P1, BIT0, GPIO_PMD_OUTPUT);
        P10 = (IsRun?0x0:0x01);
}

extern void HC595_LedDisp(unsigned int LedNumVal)  
{  
		uint8_t  temp_data,j;
	
		P20 = 0;
   
		if(LedNumVal==0xff)
		{
				HC595_SendData(HC595_LEDCode[15]);
				HC595_SendData(HC595_LEDCode[15]);
				HC595_SendData(HC595_LEDCode[15]);
				HC595_SendData(HC595_LEDCode[15]);
		}
		else
		{
				HC595LedVal= LedNumVal;
				temp_data	= (LedNumVal%10)/1;
				HC595_SendData(HC595_LEDCode[20]&0xff);
			
				temp_data	= (LedNumVal%100)/10;
				HC595_SendData(HC595_LEDCode[temp_data]&0xff);

				temp_data	= (LedNumVal%1000)/100;
				HC595_SendData(HC595_LEDCode[temp_data]&0xff);

				temp_data	= LedNumVal/1000;
				HC595_SendData(HC595_LEDCode[temp_data]&0xff);
		}
		for(j=0;j<30;j++)
				;  
		P20 = 1;
		for(j=0;j<120;j++)
				;     
} 
extern void HC595_Init(void)
{
   	GPIO_SetMode(P0, BIT7, GPIO_PMD_OUTPUT);
	GPIO_SetMode(P0, BIT6, GPIO_PMD_OUTPUT);
	GPIO_SetMode(P2, BIT0, GPIO_PMD_OUTPUT);
    HC595_LedDisp(0);
}


