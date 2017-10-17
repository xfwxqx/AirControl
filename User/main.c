/******************************************************************************
 * @file     	main.c
 * @version  	V1.00
 * $Revision: 3 $
 * $Date: 		17/07/03 11:45a $
 * @brief    	Template project for NUC029 series MCU
 * @auther		ljj
 * @note
 * Copyright (C) 2017 Jsyaao Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC029xAN.h"
#include "Usart.h"
#include "RN8209C.h"
#include "N74HC595.h"
#include "NRX8010.h"
//#include "infra_modbus.h"
#include "Ydt1363.h"
#include "AT45DBXXX.h" 
#include "UserCommon.h"
#include "SoftTimer.h"
#include "InfraRecord.h"

#define		RN8209C_MAX_CHIP_NUM															3

#define		DOWNCOM1_RECV_TIMEOUT_S														3
#define		DOWNCOM2_RECV_TIMEOUT_S														3
#define		DOWNCOM3_RECV_TIMEOUT_S														3
#define		DOWNCOM4_RECV_TIMEOUT_S														3
#define		DOWNCOM_MAX_RECV_TIMEOUT													0xFFFFFFFF

#define		PM5KT_COMMON_TIMEZONE_UTC													0
#define		PM5KT_COMMON_TIMEZONE_BEIJING											8
#define		PM5KT_COMMON_TIMEZONE_LOCAL												PM5KT_COMMON_TIMEZONE_BEIJING

#define		PM5KT_HARDWARE_VERSION														0x10				
#define		PM5KT_SOFTWARE_VERSION														0x170713

#define		PM5KT_LOCALRESPONSE_EXTCMD_GET_VER								0xA1
#define		PM5KT_LOCALRESPONSE_EXTCMD_GET_DATA								0xA2
#define		PM5KT_LOCALRESPONSE_EXTCMD_SET_RTC								0xA3
#define		PM5KT_LOCALRESPONSE_EXTCMD_GET_RTC								0xA4

#define		AIR_COND_MENU_TYPE_LEARN													0x1
#define		AIR_COND_MENU_TYPE_CTRL														0x2
#define		AIR_COND_MENU_TYPE_READ_LEARN_STATUS							0x3
#define		AIR_COND_MENU_TYPE_READ_R_TEMP								0x4
#define		AIR_COND_MENU_TYPE_READ_S_TEMP								0x5


#define		AIR_COND_LEARN_MAX_TIMEOUT_S											30

#define   AIR_COND_POWER_ALARM_THRESHOLD_MA									50

#define   AIR_COND_MAX_QUERY_STATUS													32

#define		COMMON_HIBYTE(x)																	(uint8_t)(((x)>>8)&0xFF)
#define		COMMON_LOBYTE(x)																	(uint8_t)((x)&0xFF)
#define		COMMON_WORD_TO_BCD(wdata)													(((wdata)%10)|((((wdata)/10)%10)<<4)|((((wdata)/100)%10)<<8)|(((wdata)/1000)<<12))		

//#define		AIR_COND_DEBUG

#define		COMMON_GETTABLEN(pTab)														(sizeof(pTab)/sizeof(pTab[0]))	

#define		SYS_RESET_WAIT_S_MAX	3



typedef struct _PM5KT_MAIN_LOCAL_MANAGER
{		
		uint32_t			TimSysTimeCountS;				
		uint8_t				UpComNeedSend;
		uint8_t				DownCom1NeedSend;
		uint32_t 			DownCom1RecvTimeOut;
		uint8_t				DownCom2NeedSend;
		uint32_t 			DownCom2RecvTimeOut;
		uint8_t				DownCom3NeedSend;
		uint32_t 			DownCom3RecvTimeOut;
		uint8_t				DownCom4NeedSend;
		uint32_t 			DownCom4RecvTimeOut;
		uint8_t				DownComIsBeingUsed;
		uint8_t				IsSysTimeUpdate_1S;
		uint8_t				IsSysTimeUpdate_5S;
		uint8_t				IsSysTimeUpdate_15S;
		uint8_t				IsSysTimeUpdate_60S;
		uint8_t				IsSysTimeUpdate_250MS;
		uint8_t				IsSysRunLight;
		uint8_t				IsSysWathdogSet;
		uint8_t				IsK1ButtonPressed;
		uint8_t				IsK2ButtonPressed;
		uint8_t				IsK3ButtonPressed;
		uint8_t				IsK4ButtonPressed;
		uint8_t				AirCondLearnStatus;
		uint8_t				AirCondLearnValue;
		uint8_t				AirCondCtrlValue;		//空调控制命令临时缓存，按键显示
		uint8_t				AirCondQueryLearnStatus;
		uint8_t				AirCondQueryRunStatus;
		uint8_t				AirCondMenuType;
		uint32_t			AirCondLearnedMaskFlag;
		uint8_t				IsAirCondLearnWaiting;
		uint8_t				AirCondLearnWaitTimeOut;
		uint8_t				IsShowAirCondLearnStatus;
		uint8_t				IsReadAirCondRunStatus;
		uint8_t				AirCondSwitchOnOffStatus;
		uint8_t				AirCondSettingTempStatus;
		uint8_t				AirCondRunModeStatus;
		uint8_t				AirCondWindSpeedStatus;
		uint8_t				IsAirCondReturnTempAlarm;
		uint8_t				IsAirCondOutletTempAlarm;
		uint8_t				IsAirCondHotAlarm;
		uint8_t				IsAirCondCoolAlarm;
		uint8_t				IsAirCondPowerAlarm;
		uint16_t			AirCondOnDurationTime_S;
		uint16_t			AirCondCoolDurationTime_S;
		uint16_t			AirCondHotDurationTime_S;
		uint8_t				IsSysStablility;
		uint8_t				IsAirCondSelfLearnClean;
		uint8_t				IsAirCondLearn1363Proc;
		uint8_t				AirCondLastLearnCmdId;
		uint8_t				IsAirCondCtrl1363Proc;	//上位机空调控制命令	
		//uint8_t				AirCondLastCtrlCmdId;
		uint8_t				IsAirCondQuery1363Proc;
		uint8_t				IsShowRTemperature; 	//显示回风温度
		uint8_t				IsShowSTemperature; 	//显示送风温度
		uint8_t				IsSysNeedReset;
		uint8_t				SysResetWaitS;
		COMM_DATA     CommData;  
}PM5KT_MAIN_LOCAL_MANAGER, *PPM5KT_MAIN_LOCAL_MANAGER;

static PM5KT_MAIN_LOCAL_MANAGER		Pm5ktLocalManager;

void TMR0_IRQHandler(void)
{
    volatile uint32_t TDRVal;
    
    if(TIMER_GetCaptureIntFlag(TIMER0) == 1)
    {
        TIMER_ClearCaptureIntFlag(TIMER0);
        TDRVal = TIMER_GetCounter(TIMER0);
        InfraDealPulse(P32,TDRVal);
    }
}
void TMR1_IRQHandler(void)
{
    static int8_t Count;
    
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
        Pm5ktLocalManager.IsSysTimeUpdate_250MS = 1;
        Count++;
        if(Count>=4){
            Pm5ktLocalManager.TimSysTimeCountS				++;
            Pm5ktLocalManager.IsSysTimeUpdate_1S 			= 1;
            
            if(Pm5ktLocalManager.TimSysTimeCountS%5 == 0)
                    Pm5ktLocalManager.IsSysTimeUpdate_5S 	= 1;
            if(Pm5ktLocalManager.TimSysTimeCountS%15 == 0)
                    Pm5ktLocalManager.IsSysTimeUpdate_15S = 1;
            if(Pm5ktLocalManager.TimSysTimeCountS%60 == 0)
                    Pm5ktLocalManager.IsSysTimeUpdate_60S = 1;
            Count=0;
        }
    }
}
void TMR2_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER2) == 1)
    {
        TIMER_ClearIntFlag(TIMER2);
    
		/*Pm5ktLocalManager.TimSysTimeCountS				++;
		Pm5ktLocalManager.IsSysTimeUpdate_1S 			= 1;
        
		if(Pm5ktLocalManager.TimSysTimeCountS%5 == 0)
				Pm5ktLocalManager.IsSysTimeUpdate_5S 	= 1;
		if(Pm5ktLocalManager.TimSysTimeCountS%15 == 0)
				Pm5ktLocalManager.IsSysTimeUpdate_15S = 1;
		if(Pm5ktLocalManager.TimSysTimeCountS%60 == 0)
				Pm5ktLocalManager.IsSysTimeUpdate_60S = 1;*/
    }
    
}
void TMR3_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        TIMER_ClearIntFlag(TIMER3);
        SoftTimerIntervalProcess();
        /*debug("%llx %x\n",RmtSta.RmtRecFront35Bit,RmtSta.RmtRecBehind32Bit);
        RmtSta.TiomeOutCnt++;
        if(RmtSta.TiomeOutCnt>1){
            memset(&RmtSta,0,sizeof(RmtSta));
            TIMER_Close(TIMER3);
        }*/
    }
    
}
void GPIOP2P3P4_IRQHandler(void)
{
    if(GPIO_GET_INT_FLAG(P4, BIT0)){
        GPIO_CLR_INT_FLAG(P4, BIT0);
		Pm5ktLocalManager.IsK1ButtonPressed = 1;
    }else if(GPIO_GET_INT_FLAG(P2, BIT4)){
        GPIO_CLR_INT_FLAG(P2, BIT4);
		Pm5ktLocalManager.IsK2ButtonPressed = 1;
    }else if(GPIO_GET_INT_FLAG(P2, BIT3)){
        GPIO_CLR_INT_FLAG(P2, BIT3);
		Pm5ktLocalManager.IsK3ButtonPressed = 1;
    }else if(GPIO_GET_INT_FLAG(P2, BIT2)){
        GPIO_CLR_INT_FLAG(P2, BIT2);
		Pm5ktLocalManager.IsK4ButtonPressed = 1;
    }else{
        P2->ISRC = P2->ISRC;
        P3->ISRC = P3->ISRC;
        P4->ISRC = P4->ISRC;
    }
}

void PWMB_IRQHandler(void)
{
    uint32_t u32PwmIntFlag;

    /* Handle PWMA Timer function */
    u32PwmIntFlag = PWMB->PIIR;

    /* PWMB channel 1 PWM timer interrupt */
    if(u32PwmIntFlag & PWM_PIIR_PWMIF1_Msk)
    {
        PWMA->PIIR = PWM_PIIR_PWMIF1_Msk;
        //PWM_PwmIRQHandler();
    }
}

extern void WatchDogFeed(void)
{
		Pm5ktLocalManager.IsSysWathdogSet = Pm5ktLocalManager.IsSysWathdogSet ? 0:1;
	
		GPIO_SetMode(P0, BIT1, GPIO_PMD_OUTPUT);
		P01 = Pm5ktLocalManager.IsSysWathdogSet;
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

		/*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    //CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock 
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);*/

    /* Waiting for external XTAL clock ready 
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);*/

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_50MHZ);
	
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
	CLK_EnableModuleClock(TMR3_MODULE);	
    CLK_EnableModuleClock(PWM45_MODULE);
    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HCLK, CLK_CLKDIV_HCLK(1));
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HIRC, CLK_CLKDIV_HCLK(1));
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2_S_HIRC, CLK_CLKDIV_HCLK(1));
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3_S_HIRC, CLK_CLKDIV_HCLK(1)); 
    CLK_SetModuleClock(PWM45_MODULE, CLK_CLKSEL2_PWM45_S_HIRC, CLK_CLKDIV_HCLK(1));    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
     
    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk | SYS_MFP_P32_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0 | SYS_MFP_P32_T0EX);

	SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_RXD1 | SYS_MFP_P13_TXD1);
    
    SYS->P2_MFP &= ~(SYS_MFP_P25_Msk);
    SYS->P2_MFP |= (SYS_MFP_P25_PWM5);
    
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}
static void Timer0Init(void)
{
    TIMER_Close(TIMER0);
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_FALLING_AND_RISING_EDGE);
    TIMER_EnableCaptureInt(TIMER0);
    TIMER_EnableCaptureDebounce(TIMER0);//防抖动
    /* Enable Timer1 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);
    TIMER_Start(TIMER0);
}
static void Timer1Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 4);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);
    TIMER_Start(TIMER1);
}
static void Timer2Init(void)
{
    TIMER_Close(TIMER2);
    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 1);
    TIMER_EnableInt(TIMER2);
    NVIC_EnableIRQ(TMR2_IRQn);
    TIMER_Start(TIMER2);
}
void Timer3Init(void)
{
    TIMER_Close(TIMER3);
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 10);
    TIMER_EnableInt(TIMER3);
    NVIC_EnableIRQ(TMR3_IRQn);
    TIMER_Start(TIMER3);
}
static void GpioIntInit(void)
{
		GPIO_SetMode(P2, BIT2, GPIO_PMD_INPUT);
    GPIO_EnableInt(P2, 2, GPIO_INT_RISING);
		GPIO_SetMode(P2, BIT3, GPIO_PMD_INPUT);
    GPIO_EnableInt(P2, 3, GPIO_INT_RISING);
		GPIO_SetMode(P2, BIT4, GPIO_PMD_INPUT);
    GPIO_EnableInt(P2, 4, GPIO_INT_RISING);
    GPIO_SetMode(P4, BIT0, GPIO_PMD_INPUT);
    GPIO_EnableInt(P4, 0, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPIO_P2P3P4_IRQn);
}
static void UartsInit(void)
{
	UPCOM_UnInit();
	DOWNCOM_UnInit();
	UPCOM_Init(PM5KT_DEFAULT_UPCOM_BAUDRATE);
    DOWNCOM_Init_DisenableIRQ(PM5KT_DEFAULT_DOWNCOM1_BAUDRATE,PM5KT_DEFAULT_DOWNCOM1_PARITY);
}
static void SysDataInit(void)
{	
	Pm5ktLocalManager.DownCom1RecvTimeOut 		= DOWNCOM_MAX_RECV_TIMEOUT;
	Pm5ktLocalManager.DownCom2RecvTimeOut 		= DOWNCOM_MAX_RECV_TIMEOUT;
	Pm5ktLocalManager.DownCom3RecvTimeOut 		= DOWNCOM_MAX_RECV_TIMEOUT;
	Pm5ktLocalManager.DownCom4RecvTimeOut 		= DOWNCOM_MAX_RECV_TIMEOUT;
	Pm5ktLocalManager.IsReadAirCondRunStatus 	= 1;
	Pm5ktLocalManager.AirCondLearnedMaskFlag    = 0;

	InfraGetLearningStatus(&Pm5ktLocalManager.AirCondLearnedMaskFlag);
    debug("%s AirCondLearnedMaskFlag=%d\n",__FUNCTION__,Pm5ktLocalManager.AirCondLearnedMaskFlag);
	switch(Pm5ktLocalManager.CommData.AirCondLastCtrlCmdId)
	{
		case 1:
			Pm5ktLocalManager.AirCondSwitchOnOffStatus = PowerOff;
			Pm5ktLocalManager.AirCondRunModeStatus = COLD;
			Pm5ktLocalManager.AirCondWindSpeedStatus = HIGH;
			Pm5ktLocalManager.AirCondSettingTempStatus	= 20;
			break;
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
			Pm5ktLocalManager.AirCondSwitchOnOffStatus = PowerOn;
			Pm5ktLocalManager.AirCondRunModeStatus = COLD;
			Pm5ktLocalManager.AirCondWindSpeedStatus = HIGH;
			Pm5ktLocalManager.AirCondSettingTempStatus	= Pm5ktLocalManager.CommData.AirCondLastCtrlCmdId-2+20;
			break;
		case 13:
			Pm5ktLocalManager.AirCondSwitchOnOffStatus = PowerOn;
			Pm5ktLocalManager.AirCondRunModeStatus = WIND;
			Pm5ktLocalManager.AirCondWindSpeedStatus = HIGH;
			Pm5ktLocalManager.AirCondSettingTempStatus	= 20;
			break;
		case 14:
			Pm5ktLocalManager.AirCondSwitchOnOffStatus = PowerOn;
			Pm5ktLocalManager.AirCondRunModeStatus = HEAT;
			Pm5ktLocalManager.AirCondWindSpeedStatus = HIGH;
			Pm5ktLocalManager.AirCondSettingTempStatus	= 16;
			break;
		case 15:
			Pm5ktLocalManager.AirCondSwitchOnOffStatus = PowerOn;
			Pm5ktLocalManager.AirCondRunModeStatus = COLD;
			Pm5ktLocalManager.AirCondWindSpeedStatus = HIGH;
			Pm5ktLocalManager.AirCondSettingTempStatus	= 20;
			break;
		default:
			Pm5ktLocalManager.AirCondSwitchOnOffStatus = PowerOff;
			Pm5ktLocalManager.AirCondRunModeStatus = COLD;
			Pm5ktLocalManager.AirCondWindSpeedStatus = HIGH;
			Pm5ktLocalManager.AirCondSettingTempStatus	= 20;
			break;
	}
	Pm5ktLocalManager.AirCondRunModeStatus = Pm5ktLocalManager.CommData.YdtData->Param.RunMode;
	Pm5ktLocalManager.AirCondWindSpeedStatus = Pm5ktLocalManager.CommData.YdtData->Param.WindMode;
	Pm5ktLocalManager.AirCondSettingTempStatus	= Pm5ktLocalManager.CommData.YdtData->Param.SetTemp;
}
//PWM0-7分为PWMA(0-3)、PWMB(0-3)
static void PWM5Init(uint32_t u32Frequency,uint32_t u32DutyCycle)
{
    SYS_ResetModule(PWM47_RST);
    
    PWM_Stop(PWMB, 1<<PWM_CH1);
    /* Enable PWM Output pin */
    PWM_EnableOutput(PWMB, 1<<PWM_CH1);

    PWM_ConfigOutputChannel(PWMB, PWM_CH1, u32Frequency, u32DutyCycle);

    /* Enable Timer period Interrupt */
    //PWM_EnablePeriodInt(PWMB, PWM_CH0, PWM_PERIOD_INT_UNDERFLOW);

    /* Enable PWMB NVIC */
    //NVIC_EnableIRQ((IRQn_Type)(PWMB_IRQn));

    /* Enable PWM Timer */
    PWM_Start(PWMB, 1<<PWM_CH1);

}


extern uint8_t Comm_MakeSum(const void *pBuf, uint16_t Len)
{
	uint8_t		*ptr=(uint8_t *)pBuf, Sum=0;

	while (Len--)
	{
			Sum		+= *ptr;
			ptr		++;
	}

	return	Sum;
}
static void AsciiToHex(const uint8_t *oBuf, uint8_t *dBuf, const uint32_t oLen)
{
    uint8_t i=0,cnt;

		for(cnt=0;cnt<oLen;)
		{
				if((oBuf[cnt] >= '0')&&(oBuf[cnt] <= '9'))
				{
						dBuf[i] = oBuf[cnt]-0x30;
				}
				else if((oBuf[cnt] >= 'a')&&(oBuf[cnt] <= 'f'))
				{
						dBuf[i] = oBuf[cnt]-0x61 + 10;
				}
				else if ((oBuf[cnt] >= 'A')&&(oBuf[cnt] <= 'F'))
				{
						dBuf[i] = oBuf[cnt]-0x41 + 10;
				}

				cnt++;
				dBuf[i] *= 0x10;

				if((oBuf[cnt] >= '0')&&(oBuf[cnt] <= '9'))
				{
						dBuf[i] += oBuf[cnt]-0x30;
				}
				else if((oBuf[cnt] >= 'a')&&(oBuf[cnt] <= 'f'))
				{
						dBuf[i] += oBuf[cnt] - 0x61 + 10;
				}
				else if ((oBuf[cnt] >= 'A')&&(oBuf[cnt] <= 'F'))
				{
						dBuf[i] += oBuf[cnt]-0x41 + 10;
				}
				
				cnt++;
				i++;
		}
}
extern void Comm_CycBufferToLineBuffer(uint8_t *pCycBuffer, uint16_t PushOffset, uint16_t *pPopOffset, uint16_t CycBufferSize, uint8_t *pLineBuffer, uint16_t *pLineDataLen, uint16_t LineBufferSize)
{
	uint16_t		CycDataLen, LineLeftLen;

	if ((pCycBuffer!=NULL) && (pLineBuffer!=NULL) && (pLineDataLen!=NULL) && (pPopOffset!=NULL))
	{
			if (PushOffset>=*pPopOffset)
			{
					CycDataLen		= PushOffset-*pPopOffset;
					LineLeftLen		= LineBufferSize-*pLineDataLen;
					if (CycDataLen<=LineLeftLen)
					{
							memcpy(&pLineBuffer[*pLineDataLen], &pCycBuffer[*pPopOffset], CycDataLen);	
							*pLineDataLen	+= CycDataLen;
					}
					else
					{
							memcpy(&pLineBuffer[*pLineDataLen], &pCycBuffer[*pPopOffset], LineLeftLen);	
							*pLineDataLen	= LineBufferSize;
					}
			}
			else				
			{
					CycDataLen		= PushOffset+CycBufferSize-*pPopOffset;
					LineLeftLen		= LineBufferSize-*pLineDataLen;
					if (CycDataLen<=LineLeftLen)
					{
							memcpy(&pLineBuffer[*pLineDataLen], &pCycBuffer[*pPopOffset], CycBufferSize-*pPopOffset);	
							*pLineDataLen	+= CycBufferSize-*pPopOffset;
							memcpy(&pLineBuffer[*pLineDataLen], pCycBuffer, PushOffset);	
							*pLineDataLen	+= PushOffset;	
					}
					else
					{
							if (LineLeftLen<(CycBufferSize-*pPopOffset))
							{
									memcpy(&pLineBuffer[*pLineDataLen], &pCycBuffer[*pPopOffset], LineLeftLen);	
							}
							else
							{
									memcpy(&pLineBuffer[*pLineDataLen], &pCycBuffer[*pPopOffset], CycBufferSize-*pPopOffset);	
									*pLineDataLen	+= CycBufferSize-*pPopOffset;
									LineLeftLen		-= CycBufferSize-*pPopOffset;
									memcpy(&pLineBuffer[*pLineDataLen], pCycBuffer, LineLeftLen);	
							}
							*pLineDataLen	= LineBufferSize;
					}
			}
			*pPopOffset		= PushOffset;
	}
}
extern uint8_t Comm_LineBufferToCycBuffer(uint8_t *pLineBuffer, uint16_t LineDataLen,  uint8_t *pCycBuffer, uint16_t *pPushOffset, uint16_t PopOffset, uint16_t CycBufferSize)
{
	if((pLineBuffer!=NULL) && (pCycBuffer!=NULL) && (pPushOffset!=NULL))
	{
			if (((PopOffset+CycBufferSize-*pPushOffset-1)%CycBufferSize)>LineDataLen)
			{
					if ((*pPushOffset+LineDataLen)>CycBufferSize)
					{
							memcpy(&pCycBuffer[*pPushOffset], pLineBuffer, CycBufferSize-*pPushOffset);
							memcpy(&pCycBuffer[0], &pLineBuffer[CycBufferSize-*pPushOffset], LineDataLen+*pPushOffset-CycBufferSize);
					}
					else
					{
							memcpy(&pCycBuffer[*pPushOffset], pLineBuffer, LineDataLen);
					}
					*pPushOffset	= (*pPushOffset+LineDataLen)%CycBufferSize;	
					return	1;
			}
	}

	return	0;
}
extern uint8_t Uart_ComSendFromCycBuffer(uint8_t comindex,uint8_t *pSendBuffer, uint16_t PushOffset, uint16_t *pPopOffset, uint16_t BufferSize)
{
		if( (pSendBuffer!=NULL) && (pPopOffset!=NULL))
		{
				if(PushOffset>*pPopOffset)
				{
						if (comindex == 0)
								UPCOM_Write(&pSendBuffer[*pPopOffset],PushOffset-*pPopOffset);
						else 
								DOWNCOM_Write(&pSendBuffer[*pPopOffset],PushOffset-*pPopOffset);
					
						*pPopOffset			= PushOffset;
						return	1;
				}
				else if (PushOffset<*pPopOffset)
				{
						if (comindex == 0)
								UPCOM_Write(&pSendBuffer[*pPopOffset],BufferSize-*pPopOffset);
						else
								DOWNCOM_Write(&pSendBuffer[*pPopOffset],BufferSize-*pPopOffset);
						*pPopOffset			= 0;
						return	1;
				}
		}

		return	0;
}
extern uint8_t Comm_ProtocolAnalyse7E0D(const uint8_t *pBuf, uint16_t *pLen, uint8_t *pIsCutted, uint16_t MtuSize, PPROTOCAL_YDT1363_3 pGeneralData)
{
		uint8_t			LenChecksum, ConvertBuf[2*YDT1363_3_DATABUF_MAX];
		uint16_t		DataChecksum=0;
		uint32_t		Len,Count;

		if ((pBuf!=NULL) && (pLen!=NULL))
		{
				if (pBuf[0]==0x7E)		
				{
						if (*pLen<18)
						{
								if (pIsCutted!=NULL)
								{
										*pIsCutted	= TRUE;
								}
								return	1;
						}
						
						AsciiToHex(&pBuf[9], ConvertBuf, 4);
						Len	= SWAP_WORD(GetWord(ConvertBuf));
						
						if (((Len & 0x0FFF) / 2) > YDT1363_3_DATABUF_MAX)
								return	0;
						
						if (*pLen < 18 + (Len & 0x0FFF))	
						{
								if (pIsCutted!=NULL)
								{
										*pIsCutted	= TRUE;
								}
								return	1;
						}
						
						LenChecksum		= (Len & 0xF) + ((Len >> 4) & 0xF) + ((Len >> 8) & 0xF);
						LenChecksum		= LenChecksum % 16;
						LenChecksum		= (~LenChecksum + 1) & 0xF;
						
						if (LenChecksum != ((Len >> 12) & 0xF))		
								return	0;
						
						for (Count = 1; Count < (13 + (Len & 0x0FFF)); Count ++)
								DataChecksum	+= pBuf[Count];

						DataChecksum		= ~DataChecksum + 1;
						AsciiToHex(&pBuf[13 + (Len & 0x0FFF)], ConvertBuf, 4);
						
						if (DataChecksum != SWAP_WORD(GetWord(ConvertBuf)))	
								return	0;
						
						if (pBuf[17 + (Len & 0x0FFF)] != 0x0D)
								return	0;	

						if (pGeneralData != NULL)
						{
								AsciiToHex(&pBuf[1], ConvertBuf, 8);
								pGeneralData->Ver			= ConvertBuf[0];
								pGeneralData->Addr		= ConvertBuf[1];
								pGeneralData->CID1		= ConvertBuf[2];
								pGeneralData->CID2		= ConvertBuf[3];
								if ((Len & 0x0FFF)%2)
										pGeneralData->Length = (Len & 0x0FFF)/2+1;
								else
										pGeneralData->Length = (Len & 0x0FFF)/2;
								AsciiToHex(&pBuf[13], ConvertBuf, (Len & 0x0FFF));
								memset(pGeneralData->DataBuf, 0, sizeof(pGeneralData->DataBuf));
								memcpy(pGeneralData->DataBuf, ConvertBuf, (Len & 0x0FFF) / 2);
								
								return 1;
						}
						
				}
		}

		if (pIsCutted!=NULL)
		{
				*pIsCutted	= 0;
		}

		return	0;
}
extern uint8_t Comm_ProtocolAnalyse55AA(const uint8_t *pBuf, uint16_t *pLen, uint8_t *pIsCutted, uint8_t *pIsLong55AA, uint16_t MtuSize)
{
		uint16_t		DataLen;

		if ((pBuf!=NULL) && (pLen!=NULL))
		{
				if (pBuf[0]==0x55)		
				{
						if (*pLen<3)
						{
								if (pIsCutted!=NULL)
								{
										*pIsCutted	= TRUE;
								}

								return	1;
						}
					
						if (pBuf[2]==0)		
						{
								if (*pLen<7)
								{
										if (pIsCutted!=NULL)
										{
												*pIsCutted	= TRUE;
										}

										return	1;
								}
								else if (((pBuf[3]+pBuf[5])==0xFF) && ((pBuf[4]+pBuf[6])==0xFF))
								{
										DataLen		= (pBuf[4]<<8)|pBuf[3];
										if (DataLen>MtuSize)
										{
												return	0;
										}

										if (*pLen<(uint16_t)(DataLen+9))
										{
												if (pIsCutted!=NULL)
												{
														*pIsCutted	= 1;
												}

												return	1;
										}
										else
										{
												if ((Comm_MakeSum(&pBuf[7], DataLen)==pBuf[7+DataLen]) && (pBuf[8+DataLen]==0xAA))
												{
														*pLen		= DataLen+9;
														if (pIsCutted!=NULL)
														{
																*pIsCutted		= 0;
														}

														if (pIsLong55AA!=NULL)
														{
																*pIsLong55AA	= 1;
														}
														
														return	1;
												}
										}
								}
						}
						else
						{
								DataLen		= pBuf[2];
								if (*pLen<(uint16_t)(DataLen+5))	
								{
										if (pIsCutted!=NULL)
										{
												*pIsCutted	= 1;
										}

										return	1;
								}
								else
								{
										if ((Comm_MakeSum(&pBuf[3], DataLen)==pBuf[3+DataLen]) && (pBuf[4+DataLen]==0xAA))
										{
												*pLen		= DataLen+5;
												if (pIsCutted!=NULL)
												{
														*pIsCutted		= 0;
												}

												if (pIsLong55AA!=NULL)
												{
														*pIsLong55AA	= 0;
												}

												return	1;
										}
								}
						}
				}
		}

		if (pIsCutted!=NULL)
		{
				*pIsCutted	= 0;
		}

		return	0;
}
extern uint8_t AirCondGetPowerAlarmStatus(void)
{
		return Pm5ktLocalManager.IsAirCondPowerAlarm;
}
extern uint8_t AirCondGetHotAlarmStatus(void)
{
		return Pm5ktLocalManager.IsAirCondHotAlarm;
}
extern uint8_t AirCondGetCoolAlarmStatus(void)
{
		return Pm5ktLocalManager.IsAirCondCoolAlarm;
}
extern uint8_t AirCondGetReturnTempAlarmStatus(void)
{
		return Pm5ktLocalManager.IsAirCondReturnTempAlarm;
}
extern uint8_t AirCondGetOutletTempAlarmStatus(void)
{
		return Pm5ktLocalManager.IsAirCondOutletTempAlarm;
}
extern uint8_t AirCondGetSwitchOnOffStatus(void)
{
		Pm5ktLocalManager.IsAirCondQuery1363Proc = 1;
		return Pm5ktLocalManager.AirCondSwitchOnOffStatus;
}
extern uint8_t AirCondGetSettingTempStatus(void)
{
		Pm5ktLocalManager.IsAirCondQuery1363Proc = 1;
		return Pm5ktLocalManager.AirCondSettingTempStatus;
}
extern uint8_t AirCondGetRunModeStatus(void)
{
		return Pm5ktLocalManager.AirCondRunModeStatus;
}
extern uint8_t AirCondGetWindSpeedStatus(void)
{
		return Pm5ktLocalManager.AirCondWindSpeedStatus;
}
extern void AirCondCtrlOperation(uint8_t OpCmd)
{
		Pm5ktLocalManager.IsAirCondCtrl1363Proc = OpCmd;
}
extern void AirCondSelfLearnClean(void)
{
		Pm5ktLocalManager.IsAirCondSelfLearnClean = 1;
}

extern void AirCondLearnOperation(uint8_t OpCmd)
{
		Pm5ktLocalManager.IsAirCondLearnWaiting 	= 1;
		Pm5ktLocalManager.AirCondLearnWaitTimeOut = 0;
		Pm5ktLocalManager.IsAirCondLearn1363Proc 	= OpCmd;
}

static void AirCondAlarmJudge(void)
{
		if(Pm5ktLocalManager.AirCondSwitchOnOffStatus)
		{
				if(Pm5ktLocalManager.CommData.YdtData->Analog.CurrentPhaseA*1000 <= AIR_COND_POWER_ALARM_THRESHOLD_MA
					&&Pm5ktLocalManager.CommData.YdtData->Analog.CurrentPhaseB*1000 <= AIR_COND_POWER_ALARM_THRESHOLD_MA
					&&Pm5ktLocalManager.CommData.YdtData->Analog.CurrentPhaseC*1000 <= AIR_COND_POWER_ALARM_THRESHOLD_MA)
						Pm5ktLocalManager.IsAirCondPowerAlarm = DCMETER_ALARM_FAILURE;
				else	
						Pm5ktLocalManager.IsAirCondPowerAlarm = DCMETER_ALARM_NORMAL;
				if(Pm5ktLocalManager.AirCondOnDurationTime_S > 6*60)
				{
						if(Pm5ktLocalManager.AirCondRunModeStatus == 0x01)	//制冷
						{
								if(Pm5ktLocalManager.CommData.YdtData->Analog.ReturnTemperature > Pm5ktLocalManager.AirCondSettingTempStatus +3)
								{
										
										if(Pm5ktLocalManager.CommData.YdtData->Analog.ReturnTemperature - Pm5ktLocalManager.CommData.YdtData->Analog.SupplyTemperature <5)
										{
												if(Pm5ktLocalManager.AirCondCoolDurationTime_S > 5*60)
														Pm5ktLocalManager.IsAirCondCoolAlarm = DCMETER_ALARM_FAILURE;
												else
														Pm5ktLocalManager.AirCondCoolDurationTime_S += 5;
										}
										else
										{
												Pm5ktLocalManager.IsAirCondCoolAlarm 				= DCMETER_ALARM_NORMAL;
												Pm5ktLocalManager.AirCondCoolDurationTime_S = 0;
										}
								}
								else
								{
										Pm5ktLocalManager.IsAirCondCoolAlarm 				= DCMETER_ALARM_NORMAL;
										Pm5ktLocalManager.AirCondCoolDurationTime_S = 0;
								}
								
								Pm5ktLocalManager.IsAirCondHotAlarm 				= DCMETER_ALARM_NORMAL;
						}
						else
						{
								Pm5ktLocalManager.AirCondCoolDurationTime_S = 0;
						}
						if(Pm5ktLocalManager.AirCondRunModeStatus == 0x04)	//制热
						{
								if(Pm5ktLocalManager.CommData.YdtData->Analog.ReturnTemperature < Pm5ktLocalManager.AirCondSettingTempStatus -3)
								{
										if(Pm5ktLocalManager.CommData.YdtData->Analog.SupplyTemperature - Pm5ktLocalManager.CommData.YdtData->Analog.ReturnTemperature <5)
										{
												if(Pm5ktLocalManager.AirCondHotDurationTime_S > 15*60)
														Pm5ktLocalManager.IsAirCondHotAlarm = DCMETER_ALARM_FAILURE;
												else
														Pm5ktLocalManager.AirCondHotDurationTime_S += 5;
										}
										else
										{
												Pm5ktLocalManager.IsAirCondHotAlarm 				= DCMETER_ALARM_NORMAL;
												Pm5ktLocalManager.AirCondHotDurationTime_S 	= 0;
										}
								}
								else
								{
										Pm5ktLocalManager.IsAirCondHotAlarm 				= DCMETER_ALARM_NORMAL;
										Pm5ktLocalManager.AirCondHotDurationTime_S 	= 0;
								}
								
								Pm5ktLocalManager.IsAirCondCoolAlarm 				= DCMETER_ALARM_NORMAL;
						}
						else
						{
								Pm5ktLocalManager.AirCondHotDurationTime_S 	= 0;
						}
				}
				else
				{
						if((Pm5ktLocalManager.CommData.YdtData->Analog.ReturnTemperature > Pm5ktLocalManager.AirCondSettingTempStatus +3)
							&& (Pm5ktLocalManager.AirCondRunModeStatus == 0x01))
						{
								Pm5ktLocalManager.AirCondOnDurationTime_S += 5;
								Pm5ktLocalManager.IsAirCondCoolAlarm 			= DCMETER_ALARM_NORMAL;
								Pm5ktLocalManager.IsAirCondHotAlarm 			= DCMETER_ALARM_NORMAL;
						}
						else if((Pm5ktLocalManager.CommData.YdtData->Analog.ReturnTemperature < Pm5ktLocalManager.AirCondSettingTempStatus -3)
							&& (Pm5ktLocalManager.AirCondRunModeStatus == 0x04))
						{
								Pm5ktLocalManager.AirCondOnDurationTime_S += 5;
								Pm5ktLocalManager.IsAirCondCoolAlarm 			= DCMETER_ALARM_NORMAL;
								Pm5ktLocalManager.IsAirCondHotAlarm 			= DCMETER_ALARM_NORMAL;
						}
						else
						{
								Pm5ktLocalManager.AirCondOnDurationTime_S = 0;
								Pm5ktLocalManager.IsAirCondCoolAlarm 			= DCMETER_ALARM_NORMAL;
								Pm5ktLocalManager.IsAirCondHotAlarm 			= DCMETER_ALARM_NORMAL;
						}
				}
		}
		else
		{
				if(Pm5ktLocalManager.CommData.YdtData->Analog.CurrentPhaseA*1000 > AIR_COND_POWER_ALARM_THRESHOLD_MA
					||Pm5ktLocalManager.CommData.YdtData->Analog.CurrentPhaseB*1000 > AIR_COND_POWER_ALARM_THRESHOLD_MA
					||Pm5ktLocalManager.CommData.YdtData->Analog.CurrentPhaseC*1000 > AIR_COND_POWER_ALARM_THRESHOLD_MA)
						Pm5ktLocalManager.IsAirCondPowerAlarm = DCMETER_ALARM_FAILURE;
				else	
						Pm5ktLocalManager.IsAirCondPowerAlarm = DCMETER_ALARM_NORMAL;
				
				Pm5ktLocalManager.AirCondOnDurationTime_S = 0;
				Pm5ktLocalManager.IsAirCondCoolAlarm 			= DCMETER_ALARM_NORMAL;
				Pm5ktLocalManager.IsAirCondHotAlarm 			= DCMETER_ALARM_NORMAL;
		}
		
		if(Pm5ktLocalManager.CommData.YdtData->Analog.ReturnTemperature > 100.0)
				Pm5ktLocalManager.IsAirCondReturnTempAlarm = DCMETER_ALARM_FAILURE;
		else
				Pm5ktLocalManager.IsAirCondReturnTempAlarm = DCMETER_ALARM_NORMAL;
		if(Pm5ktLocalManager.CommData.YdtData->Analog.SupplyTemperature > 100.0)
				Pm5ktLocalManager.IsAirCondOutletTempAlarm = DCMETER_ALARM_FAILURE;
		else
				Pm5ktLocalManager.IsAirCondOutletTempAlarm = DCMETER_ALARM_NORMAL;
}
static void LocalResProcess(uint16_t OpCmd, uint8_t *pCmdBuf, uint8_t CmdLen)		
{
		uint8_t						ResBuf[128];
		uint16_t					ResLen=0;
		PM5KT_COMM_TM  		TmBuf;
	
		memset(ResBuf, 0, sizeof(ResBuf));
		ResBuf[0]	= 0x55;
		ResBuf[1]	= PM5KT_COM_PROTOCOL_ADDR_LOCAL;

		switch (OpCmd)
		{
			case PM5KT_LOCALRESPONSE_EXTCMD_GET_VER:
				ResBuf[2]		= 5;
				ResBuf[3]		= OpCmd;
				ResBuf[4]		= PM5KT_HARDWARE_VERSION;
				ResBuf[5]		= COMMON_LOBYTE(PM5KT_SOFTWARE_VERSION>>16);
				ResBuf[6]		= COMMON_LOBYTE(PM5KT_SOFTWARE_VERSION>>8);
				ResBuf[7]		= COMMON_LOBYTE(PM5KT_SOFTWARE_VERSION);
				ResBuf[8]		= Comm_MakeSum(&ResBuf[3], ResBuf[2]);
				ResBuf[9]		= 0xAA;
				ResLen			= 10;
				break;
			case PM5KT_LOCALRESPONSE_EXTCMD_GET_DATA:
				ResLen						= 3;
				ResBuf[ResLen++]	= OpCmd;
				memcpy(&ResBuf[ResLen],&Pm5ktLocalManager.TimSysTimeCountS, sizeof(Pm5ktLocalManager.TimSysTimeCountS));
				ResLen						+= sizeof(Pm5ktLocalManager.TimSysTimeCountS);
				ResBuf[2]					= ResLen-3;
				ResBuf[ResLen++]	= Comm_MakeSum(&ResBuf[3], ResBuf[2]);
				ResBuf[ResLen++]	= 0xAA;
				break;
			case PM5KT_LOCALRESPONSE_EXTCMD_SET_RTC:
				if ((CmdLen>=9) && (pCmdBuf!=NULL))
				{
						memset(&TmBuf,0,sizeof(TmBuf));
						TmBuf.Year		= pCmdBuf[2]|(pCmdBuf[3]<<8);
						TmBuf.Month		= pCmdBuf[4];
						TmBuf.Day			= pCmdBuf[5];
						TmBuf.Hour		= pCmdBuf[6];
						TmBuf.Minute	= pCmdBuf[7];
						TmBuf.Second	= pCmdBuf[8];
						//RX8010_SetTime(&TmBuf);
				}
				ResLen			= 0;
				break;
			case PM5KT_LOCALRESPONSE_EXTCMD_GET_RTC:
				memset(&TmBuf,0,sizeof(TmBuf));
				//RX8010_GetTime(&TmBuf);
				ResBuf[2]		= 9;
				ResBuf[3]		= OpCmd;
				ResBuf[4]		= PM5KT_COMMON_TIMEZONE_LOCAL;	
				ResBuf[5]		= COMMON_LOBYTE(TmBuf.Year);
				ResBuf[6]		= COMMON_HIBYTE(TmBuf.Year);
				ResBuf[7]		= TmBuf.Month;
				ResBuf[8]		= TmBuf.Day;
				ResBuf[9]		= TmBuf.Hour;
				ResBuf[10]	= TmBuf.Minute;
				ResBuf[11]	= TmBuf.Second;
				ResBuf[12]	= Comm_MakeSum(&ResBuf[3], ResBuf[2]);
				ResBuf[13]	= 0xAA;
				ResLen			= 14;
				break;
			default:
				ResLen			= 0;
				break;
		}
		
		if (ResLen!=0)
		{
				uint8_t retval;

				retval= Comm_LineBufferToCycBuffer(ResBuf, ResLen, gUart_Pm5ktApi.UpComSendBuffer, 
											&gUart_Pm5ktApi.UpComSendBufferPushOffset, gUart_Pm5ktApi.UpComSendBufferPopOffset, PM5KT_UPCOM_SEND_BUFFER_SIZE);
				if(retval)
						Pm5ktLocalManager.UpComNeedSend	= 1;
		}
}

//7E 31 30 30 31 36 36 34 46 30 30 30 30 46 44 39 38 0D
static void LocalResProcessEx(PPROTOCAL_YDT1363_3 pProcBuf)		
{
		uint8_t						ResBuf[256];
		uint32_t					ResLen=0;
		int32_t 					Ret=1;
		memset(ResBuf, 0, sizeof(ResBuf));
		ResLen = sizeof(ResBuf);
		
		Ret = ProtocolProc_YDT1363_3(pProcBuf,ResBuf,&ResLen);
		if (Ret==0 && ResLen!=0)
		{
				uint8_t retval;

				retval= Comm_LineBufferToCycBuffer(ResBuf, ResLen, gUart_Pm5ktApi.UpComSendBuffer, 
											&gUart_Pm5ktApi.UpComSendBufferPushOffset, gUart_Pm5ktApi.UpComSendBufferPopOffset, PM5KT_UPCOM_SEND_BUFFER_SIZE);
				if(retval){
					Pm5ktLocalManager.UpComNeedSend	= 1;
					if(NeedReset(GetCID2())){
						Pm5ktLocalManager.IsSysNeedReset = 1;
						Pm5ktLocalManager.SysResetWaitS = 0;
					}
				}
						
		}
}
static void ProtBufProcessEx(void)
{
		uint16_t						i, Len;
		uint8_t							IsCutted=0;
		PROTOCAL_YDT1363_3 	ProcBuf;
	
		for (i=0; i<gUart_Pm5ktApi.ProtDataLen; )	
		{
				Len		= gUart_Pm5ktApi.ProtDataLen-i;
				if (Comm_ProtocolAnalyse7E0D(&gUart_Pm5ktApi.ProtBuffer[i], &Len, &IsCutted, PM5KT_UPCOM_MTU_SIZE, &ProcBuf))
				{
						if (IsCutted)
						{
								memcpy(gUart_Pm5ktApi.ProtBuffer, &gUart_Pm5ktApi.ProtBuffer[i],gUart_Pm5ktApi.ProtDataLen-i);
								gUart_Pm5ktApi.ProtDataLen	-= i;
								break;
						}
						else
						{
								LocalResProcessEx(&ProcBuf);	
								
								if(gUart_Pm5ktApi.ProtDataLen-i-Len)
								{
										memcpy(gUart_Pm5ktApi.ProtBuffer, &gUart_Pm5ktApi.ProtBuffer[i+Len],gUart_Pm5ktApi.ProtDataLen-i-Len);
										gUart_Pm5ktApi.ProtDataLen = gUart_Pm5ktApi.ProtDataLen-i-Len;
								}
								else
								{
										gUart_Pm5ktApi.ProtDataLen	= 0;
										memset(gUart_Pm5ktApi.ProtBuffer,0,sizeof(gUart_Pm5ktApi.ProtBuffer));
								}
								i		+= Len;
						}
				}
				else
				{
						i		++;
				}
			
				if (i==gUart_Pm5ktApi.ProtDataLen)
				{
						gUart_Pm5ktApi.ProtDataLen	= 0;
						memset(gUart_Pm5ktApi.ProtBuffer,0,sizeof(gUart_Pm5ktApi.ProtBuffer));
				}
		}
		if(gUart_Pm5ktApi.ProtDataLen == 0x100)
		{
				gUart_Pm5ktApi.ProtDataLen	= 0;
				memset(gUart_Pm5ktApi.ProtBuffer,0,sizeof(gUart_Pm5ktApi.ProtBuffer));
		}    
}
static void ProtBufProcess(void)
{
		uint16_t		i, Len;
		uint8_t			IsCutted=0, IsLong55AA=0, RetVal=0;
		
		for (i=0; i<gUart_Pm5ktApi.ProtDataLen; )	
		{
				Len		= gUart_Pm5ktApi.ProtDataLen-i;
				if (Comm_ProtocolAnalyse55AA(&gUart_Pm5ktApi.ProtBuffer[i], &Len, &IsCutted, &IsLong55AA, PM5KT_UPCOM_MTU_SIZE))
				{
						if (IsCutted)
						{
								memcpy(gUart_Pm5ktApi.ProtBuffer, &gUart_Pm5ktApi.ProtBuffer[i], gUart_Pm5ktApi.ProtDataLen-i);
								gUart_Pm5ktApi.ProtDataLen	-= i;
								break;
						}
						else
						{
								if (IsLong55AA)
								{
										if (gUart_Pm5ktApi.ProtBuffer[i+1]==PM5KT_COM_PROTOCOL_ADDR_COM1)
										{
												uint16_t	reslen	= 0;
												reslen		= gUart_Pm5ktApi.ProtBuffer[i+3]|(gUart_Pm5ktApi.ProtBuffer[i+4]<<8);
												RetVal		= Comm_LineBufferToCycBuffer(&gUart_Pm5ktApi.ProtBuffer[i+7], reslen,
															gUart_Pm5ktApi.DownComSendBuffer, &gUart_Pm5ktApi.DownComSendBufferPushOffset, gUart_Pm5ktApi.DownComSendBufferPopOffset, PM5KT_DOWNCOM_SEND_BUFFER_SIZE);
												if(RetVal)
														Pm5ktLocalManager.DownCom1NeedSend 	= 1;
												
										}
										else if (gUart_Pm5ktApi.ProtBuffer[i+1]==PM5KT_COM_PROTOCOL_ADDR_COM2)
										{
												uint16_t	reslen	= 0;
												reslen		= gUart_Pm5ktApi.ProtBuffer[i+3]|(gUart_Pm5ktApi.ProtBuffer[i+4]<<8);
												RetVal		= Comm_LineBufferToCycBuffer(&gUart_Pm5ktApi.ProtBuffer[i+7], reslen,
															gUart_Pm5ktApi.DownComSendBuffer, &gUart_Pm5ktApi.DownComSendBufferPushOffset, gUart_Pm5ktApi.DownComSendBufferPopOffset, PM5KT_DOWNCOM_SEND_BUFFER_SIZE);
												if(RetVal)
														Pm5ktLocalManager.DownCom2NeedSend 	= 1;
												
										}
										else if (gUart_Pm5ktApi.ProtBuffer[i+1]==PM5KT_COM_PROTOCOL_ADDR_COM3)
										{
												uint16_t	reslen	= 0;
												reslen		= gUart_Pm5ktApi.ProtBuffer[i+3]|(gUart_Pm5ktApi.ProtBuffer[i+4]<<8);
												RetVal		= Comm_LineBufferToCycBuffer(&gUart_Pm5ktApi.ProtBuffer[i+7], reslen,
															gUart_Pm5ktApi.DownComSendBuffer, &gUart_Pm5ktApi.DownComSendBufferPushOffset, gUart_Pm5ktApi.DownComSendBufferPopOffset, PM5KT_DOWNCOM_SEND_BUFFER_SIZE);
												if(RetVal)
														Pm5ktLocalManager.DownCom3NeedSend 	= 1;
												
										}
										else if (gUart_Pm5ktApi.ProtBuffer[i+1]==PM5KT_COM_PROTOCOL_ADDR_COM4)
										{
												uint16_t	reslen	= 0;
												reslen		= gUart_Pm5ktApi.ProtBuffer[i+3]|(gUart_Pm5ktApi.ProtBuffer[i+4]<<8);
												RetVal		= Comm_LineBufferToCycBuffer(&gUart_Pm5ktApi.ProtBuffer[i+7], reslen,
															gUart_Pm5ktApi.DownComSendBuffer, &gUart_Pm5ktApi.DownComSendBufferPushOffset, gUart_Pm5ktApi.DownComSendBufferPopOffset, PM5KT_DOWNCOM_SEND_BUFFER_SIZE);
												if(RetVal)
														Pm5ktLocalManager.DownCom4NeedSend 	= 1;
										}
								
								}
								else
								{
										if (gUart_Pm5ktApi.ProtBuffer[i+1]==PM5KT_COM_PROTOCOL_ADDR_LOCAL)
										{
												LocalResProcess(gUart_Pm5ktApi.ProtBuffer[i+3], &gUart_Pm5ktApi.ProtBuffer[i+3], gUart_Pm5ktApi.ProtBuffer[i+2]);
										}
									
								}

								i		+= Len;
						}
				}
				else
				{
						i		++;
				}
			
				if (i==gUart_Pm5ktApi.ProtDataLen)
				{
						gUart_Pm5ktApi.ProtDataLen	= 0;
				}
		}
}


static void SysTimeProcess_250MS(void)
{
		unsigned int 	disp_val=0;
		uint8_t				IrInBuf[3];
		uint16_t			IrInLen=3,i;
		uint8_t				IrOutBuf[10];
		uint16_t			IrOutLen=sizeof(IrOutBuf);
    SaveRecordFormatDef *pTemp=NULL;
    
		#ifdef LEARNING_TEST  
		if(FUNC_RET_TRUE == InfraLearning(MANUFACTURER_MEIDI,CMDID3_ON_COOL_TEMP21_STRONG)){
				//InfraInit();
		}
		#endif
		WatchDogFeed();

		if(Pm5ktLocalManager.IsK1ButtonPressed && Pm5ktLocalManager.IsSysStablility)
		{
				Pm5ktLocalManager.IsK1ButtonPressed = 0;
				Pm5ktLocalManager.IsShowRTemperature = 0;
				Pm5ktLocalManager.IsShowSTemperature = 0;
                debug("K1 Button Pressed\n");
				LearnLightCtrl(0,0);
				switch(Pm5ktLocalManager.AirCondMenuType)
				{
						case 0:
								break;
						case AIR_COND_MENU_TYPE_LEARN:		
								if(Pm5ktLocalManager.AirCondLearnValue > 1)
										Pm5ktLocalManager.AirCondLearnValue --;
								if(Pm5ktLocalManager.AirCondLearnValue <= 1)
										Pm5ktLocalManager.AirCondLearnValue = 1;
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000 + Pm5ktLocalManager.AirCondLearnValue*10;
								break;
						case AIR_COND_MENU_TYPE_CTRL:			
								if(Pm5ktLocalManager.AirCondCtrlValue > 1)
										Pm5ktLocalManager.AirCondCtrlValue --;
								if(Pm5ktLocalManager.AirCondCtrlValue <= 1)
										Pm5ktLocalManager.AirCondCtrlValue = 1;
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000 + Pm5ktLocalManager.AirCondCtrlValue*10;
								break;
						case AIR_COND_MENU_TYPE_READ_LEARN_STATUS:	
								if(Pm5ktLocalManager.AirCondQueryLearnStatus > 1)
										Pm5ktLocalManager.AirCondQueryLearnStatus --;
								if(Pm5ktLocalManager.AirCondQueryLearnStatus <= 1)
										Pm5ktLocalManager.AirCondQueryLearnStatus = 1;
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000 + Pm5ktLocalManager.AirCondQueryLearnStatus*10;
								break;
						case AIR_COND_MENU_TYPE_READ_R_TEMP:	
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000;
								break;
						case AIR_COND_MENU_TYPE_READ_S_TEMP:	
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000;
								break;
						default:
								break;
				}
				HC595_LedDisp(disp_val);

		}
		if(Pm5ktLocalManager.IsK2ButtonPressed && Pm5ktLocalManager.IsSysStablility)
		{
				Pm5ktLocalManager.IsK2ButtonPressed = 0;
				Pm5ktLocalManager.IsShowRTemperature = 0;
				Pm5ktLocalManager.IsShowSTemperature = 0;
                debug("K2 Button Pressed\n");
				LearnLightCtrl(0,0);
				switch(Pm5ktLocalManager.AirCondMenuType)
				{
						case 0:
								break;
						case AIR_COND_MENU_TYPE_LEARN:		
								if(Pm5ktLocalManager.AirCondLearnValue < AIR_COND_MAX_QUERY_STATUS)
										Pm5ktLocalManager.AirCondLearnValue ++;
								if(Pm5ktLocalManager.AirCondLearnValue >= AIR_COND_MAX_QUERY_STATUS)
										Pm5ktLocalManager.AirCondLearnValue = AIR_COND_MAX_QUERY_STATUS;
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000 + Pm5ktLocalManager.AirCondLearnValue*10;
								break;
						case AIR_COND_MENU_TYPE_CTRL:		
								if(Pm5ktLocalManager.AirCondCtrlValue < AIR_COND_MAX_QUERY_STATUS)
										Pm5ktLocalManager.AirCondCtrlValue ++;
								if(Pm5ktLocalManager.AirCondCtrlValue >= AIR_COND_MAX_QUERY_STATUS)
										Pm5ktLocalManager.AirCondCtrlValue = AIR_COND_MAX_QUERY_STATUS;
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000 + Pm5ktLocalManager.AirCondCtrlValue*10;
								break;
						case AIR_COND_MENU_TYPE_READ_LEARN_STATUS:		
								if(Pm5ktLocalManager.AirCondQueryLearnStatus < AIR_COND_MAX_QUERY_STATUS)
										Pm5ktLocalManager.AirCondQueryLearnStatus ++;
								if(Pm5ktLocalManager.AirCondQueryLearnStatus >= AIR_COND_MAX_QUERY_STATUS)
										Pm5ktLocalManager.AirCondQueryLearnStatus = AIR_COND_MAX_QUERY_STATUS;
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000 + Pm5ktLocalManager.AirCondQueryLearnStatus*10;
								break;
						case AIR_COND_MENU_TYPE_READ_R_TEMP:	
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000;
								break;
						case AIR_COND_MENU_TYPE_READ_S_TEMP:	
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000;
								break;
						default:
								break;
				}
				HC595_LedDisp(disp_val);
		
		}
		if(Pm5ktLocalManager.IsK3ButtonPressed && Pm5ktLocalManager.IsSysStablility)
		{
				Pm5ktLocalManager.IsK3ButtonPressed = 0;
                debug("K3 Button Pressed\n");
				LearnLightCtrl(0,0);
				Pm5ktLocalManager.IsShowRTemperature = 0;
				Pm5ktLocalManager.IsShowSTemperature = 0;
				Pm5ktLocalManager.AirCondMenuType++;
				Pm5ktLocalManager.AirCondMenuType=Pm5ktLocalManager.AirCondMenuType%6;
				switch(Pm5ktLocalManager.AirCondMenuType)
				{
						case 0:
								Pm5ktLocalManager.IsShowAirCondLearnStatus = 0;
								break;
						case AIR_COND_MENU_TYPE_LEARN:		
								Pm5ktLocalManager.IsShowAirCondLearnStatus = 0;
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000 + Pm5ktLocalManager.AirCondLearnValue*10;
								break;
						case AIR_COND_MENU_TYPE_CTRL:		
								Pm5ktLocalManager.IsShowAirCondLearnStatus = 0;
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000 + Pm5ktLocalManager.AirCondCtrlValue*10;
								break;
						case AIR_COND_MENU_TYPE_READ_LEARN_STATUS:		
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000 + Pm5ktLocalManager.AirCondQueryLearnStatus*10;
								break;
						case AIR_COND_MENU_TYPE_READ_R_TEMP:	
								Pm5ktLocalManager.IsShowAirCondLearnStatus = 0;
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000;
								break;
						case AIR_COND_MENU_TYPE_READ_S_TEMP:	
								Pm5ktLocalManager.IsShowAirCondLearnStatus = 0;
								disp_val = Pm5ktLocalManager.AirCondMenuType*1000;
								break;
						default:
								break;
				}
				
				HC595_LedDisp(disp_val);
		}
		if(Pm5ktLocalManager.IsK4ButtonPressed && Pm5ktLocalManager.IsSysStablility && !Pm5ktLocalManager.DownComIsBeingUsed && !Pm5ktLocalManager.DownCom4NeedSend)
		{
				Pm5ktLocalManager.IsK4ButtonPressed = 0;
                debug("K4 Button Pressed\n");
				if(Pm5ktLocalManager.AirCondMenuType == AIR_COND_MENU_TYPE_LEARN)
				{
						/*IrInBuf[0] = Pm5ktLocalManager.AirCondLearnValue;
						IrInBuf[1] = 0x00;
						IrInBuf[2] = 0x00;
						memset(IrOutBuf,0,sizeof(IrOutBuf));
						if(modbus_tx_fun(PROTOCAL_ADDR,PROTOCAL_CODE_LEARN,IrInBuf,IrInLen,IrOutBuf,IrOutLen) == PROTOCAL_NORMAL)
						{	
								memcpy(gUart_Pm5ktApi.DownComSendBuffer,IrOutBuf,IrOutLen);
								gUart_Pm5ktApi.DownComSendBufferPushOffset	= IrOutLen;
								gUart_Pm5ktApi.DownComSendBufferPopOffset		= 0;
								Pm5ktLocalManager.DownCom4NeedSend 					= 1;
								Pm5ktLocalManager.AirCondLearnStatus 				= 1;
								Pm5ktLocalManager.IsShowAirCondLearnStatus 	= 1;
						}*/
						
						Pm5ktLocalManager.IsShowAirCondLearnStatus 	= 1;
						Pm5ktLocalManager.IsAirCondLearnWaiting = 1;
						Pm5ktLocalManager.AirCondLearnStatus = 0;//标记学习命令来自于电总协议
				}
				else if(Pm5ktLocalManager.AirCondMenuType == AIR_COND_MENU_TYPE_CTRL)
				{
						/*IrInBuf[0] = Pm5ktLocalManager.AirCondCtrlValue;
						IrInBuf[1] = 0x00;
						IrInBuf[2] = 0x00;
						memset(IrOutBuf,0,sizeof(IrOutBuf));
						if(modbus_tx_fun(PROTOCAL_ADDR,PROTOCAL_CODE_CTRL,IrInBuf,IrInLen,IrOutBuf,IrOutLen) == PROTOCAL_NORMAL)
						{
								memcpy(gUart_Pm5ktApi.DownComSendBuffer,IrOutBuf,IrOutLen);
								gUart_Pm5ktApi.DownComSendBufferPushOffset	= IrOutLen;
								gUart_Pm5ktApi.DownComSendBufferPopOffset		= 0;
								Pm5ktLocalManager.DownCom4NeedSend 					= 1;
						}*/
						
						if(FUNC_RET_TRUE == InfraGetRecordTemp(&pTemp)){
							if(FUNC_RET_TRUE == GetInfraLearningCmdFromFlash(InfraGetManuID(),Pm5ktLocalManager.AirCondCtrlValue-1,pTemp,sizeof(SaveRecordFormatDef))){
								CommunicationLightCtrl(1);
								SendInfraCmd(pTemp);//发送命令
								debug("######### SendInfraCmd Cnt=%u ########\n",pTemp->RecordCnt);
								for(i=0;i<pTemp->RecordCnt;i++){
										if(i>=INFRA_RECORD_MAX)
												break;
										debug("{%d,%d} ",pTemp->Record[i].PinLevel,pTemp->Record[i].DurationUS);
								}
								debug("\n");
								debug("%s %u SendInfraCmd Succ,CMDID=%u\n",__FUNCTION__,__LINE__,Pm5ktLocalManager.AirCondCtrlValue);
								Pm5ktLocalManager.CommData.AirCondLastCtrlCmdId = Pm5ktLocalManager.AirCondCtrlValue;
								CommunicationLightCtrl(0);
							}
							InfraFreeRecordTemp(&pTemp);
						}
						InfraFreeRecordTemp(&pTemp);
				}
				else if(Pm5ktLocalManager.AirCondMenuType == AIR_COND_MENU_TYPE_READ_LEARN_STATUS)
				{
						/*IrInBuf[0] = 0x01;
						IrInBuf[1] = 0xff;
						IrInBuf[2] = 0x01;
						memset(IrOutBuf,0,sizeof(IrOutBuf));
						if(modbus_tx_fun(PROTOCAL_ADDR,PROTOCAL_CODE_CHECK_STA,IrInBuf,IrInLen,IrOutBuf,IrOutLen) == PROTOCAL_NORMAL)
						{
								memcpy(gUart_Pm5ktApi.DownComSendBuffer,IrOutBuf,IrOutLen);
								gUart_Pm5ktApi.DownComSendBufferPushOffset	= IrOutLen;
								gUart_Pm5ktApi.DownComSendBufferPopOffset		= 0;
								Pm5ktLocalManager.DownCom4NeedSend 					= 1;
								Pm5ktLocalManager.IsShowAirCondLearnStatus 	= 1;
						}*/
						Pm5ktLocalManager.IsShowAirCondLearnStatus 	= 1;
				}
				else if(Pm5ktLocalManager.AirCondMenuType == AIR_COND_MENU_TYPE_READ_R_TEMP)
				{
						/*IrInBuf[0] = 0x01;
						IrInBuf[1] = 0xff;
						IrInBuf[2] = 0x01;
						memset(IrOutBuf,0,sizeof(IrOutBuf));
						if(modbus_tx_fun(PROTOCAL_ADDR,PROTOCAL_CODE_QUERY_START,IrInBuf,IrInLen,IrOutBuf,IrOutLen) == PROTOCAL_NORMAL)
						{
								memcpy(gUart_Pm5ktApi.DownComSendBuffer,IrOutBuf,IrOutLen);
								gUart_Pm5ktApi.DownComSendBufferPushOffset	= IrOutLen;
								gUart_Pm5ktApi.DownComSendBufferPopOffset		= 0;
								Pm5ktLocalManager.DownCom4NeedSend 					= 1;
						}*/
						Pm5ktLocalManager.IsShowRTemperature = 1;
				}
				else if(Pm5ktLocalManager.AirCondMenuType == AIR_COND_MENU_TYPE_READ_S_TEMP)
				{
						/*IrInBuf[0] = 0x01;
						IrInBuf[1] = 0xff;
						IrInBuf[2] = 0x01;
						memset(IrOutBuf,0,sizeof(IrOutBuf));
						if(modbus_tx_fun(PROTOCAL_ADDR,PROTOCAL_CODE_QUERY_START,IrInBuf,IrInLen,IrOutBuf,IrOutLen) == PROTOCAL_NORMAL)
						{
								memcpy(gUart_Pm5ktApi.DownComSendBuffer,IrOutBuf,IrOutLen);
								gUart_Pm5ktApi.DownComSendBufferPushOffset	= IrOutLen;
								gUart_Pm5ktApi.DownComSendBufferPopOffset		= 0;
								Pm5ktLocalManager.DownCom4NeedSend 					= 1;
						}*/
						Pm5ktLocalManager.IsShowSTemperature = 1;
				}
		}
		/*if(Pm5ktLocalManager.AirCondLearnStatus == 0 && !Pm5ktLocalManager.DownComIsBeingUsed && !Pm5ktLocalManager.DownCom4NeedSend)
		{
				IrInBuf[0] = 0x01;
				IrInBuf[1] = 0xff;
				IrInBuf[2] = 0x01;
				memset(IrOutBuf,0,sizeof(IrOutBuf));
				if(modbus_tx_fun(PROTOCAL_ADDR,PROTOCAL_CODE_CHECK_STA,IrInBuf,IrInLen,IrOutBuf,IrOutLen) == PROTOCAL_NORMAL)
				{
						memcpy(gUart_Pm5ktApi.DownComSendBuffer,IrOutBuf,IrOutLen);
						gUart_Pm5ktApi.DownComSendBufferPushOffset	= IrOutLen;
						gUart_Pm5ktApi.DownComSendBufferPopOffset		= 0;
						Pm5ktLocalManager.DownCom4NeedSend 					= 1;
				}
				Pm5ktLocalManager.AirCondLearnStatus = 3;
		}
		if(Pm5ktLocalManager.AirCondLearnStatus == 2 && !Pm5ktLocalManager.DownComIsBeingUsed && !Pm5ktLocalManager.DownCom4NeedSend)
		{
				IrInBuf[0] = 0x01;
				IrInBuf[1] = 0xff;
				IrInBuf[2] = 0x01;
				memset(IrOutBuf,0,sizeof(IrOutBuf));
				if(modbus_tx_fun(PROTOCAL_ADDR,PROTOCAL_CODE_CHECK_STA,IrInBuf,IrInLen,IrOutBuf,IrOutLen) == PROTOCAL_NORMAL)
				{
						memcpy(gUart_Pm5ktApi.DownComSendBuffer,IrOutBuf,IrOutLen);
						gUart_Pm5ktApi.DownComSendBufferPushOffset	= IrOutLen;
						gUart_Pm5ktApi.DownComSendBufferPopOffset		= 0;
						Pm5ktLocalManager.DownCom4NeedSend 					= 1;
						Pm5ktLocalManager.AirCondLearnStatus 				= 3;
				}
				
		}*/
		
		
		if(Pm5ktLocalManager.IsAirCondLearn1363Proc && !Pm5ktLocalManager.DownComIsBeingUsed && !Pm5ktLocalManager.DownCom4NeedSend)
		{
				/*IrInBuf[0] = Pm5ktLocalManager.IsAirCondLearn1363Proc;
				IrInBuf[1] = 0x00;
				IrInBuf[2] = 0x00;
				memset(IrOutBuf,0,sizeof(IrOutBuf));
				if(modbus_tx_fun(PROTOCAL_ADDR,PROTOCAL_CODE_LEARN,IrInBuf,IrInLen,IrOutBuf,IrOutLen) == PROTOCAL_NORMAL)
				{	
						memcpy(gUart_Pm5ktApi.DownComSendBuffer,IrOutBuf,IrOutLen);
						gUart_Pm5ktApi.DownComSendBufferPushOffset	= IrOutLen;
						gUart_Pm5ktApi.DownComSendBufferPopOffset		= 0;
						Pm5ktLocalManager.DownCom4NeedSend 					= 1;
						Pm5ktLocalManager.AirCondLearnStatus 				= 1;
						Pm5ktLocalManager.IsShowAirCondLearnStatus 	= 1;
						Pm5ktLocalManager.AirCondLastLearnCmdId			= Pm5ktLocalManager.IsAirCondLearn1363Proc;
						Pm5ktLocalManager.IsAirCondLearn1363Proc		= 0;
				}*/

				Pm5ktLocalManager.IsShowAirCondLearnStatus 	= 1;
				Pm5ktLocalManager.IsAirCondLearnWaiting = 1;
				Pm5ktLocalManager.AirCondLastLearnCmdId	= Pm5ktLocalManager.IsAirCondLearn1363Proc;
				Pm5ktLocalManager.AirCondLearnStatus  = 1;
				Pm5ktLocalManager.IsAirCondLearn1363Proc = 0;
				/*		
				if(FUNC_RET_TRUE == InfraLearning(InfraGetManuID(),Pm5ktLocalManager.IsAirCondLearn1363Proc-1)){
					Pm5ktLocalManager.AirCondLearnedMaskFlag |= (1<<(Pm5ktLocalManager.IsAirCondLearn1363Proc-1));
					Pm5ktLocalManager.IsAirCondLearn1363Proc = 0;
					Pm5ktLocalManager.AirCondLearnStatus  = 1;
					Pm5ktLocalManager.IsShowAirCondLearnStatus 	= 1;
					Pm5ktLocalManager.AirCondLastLearnCmdId			= Pm5ktLocalManager.IsAirCondLearn1363Proc;
				}else{
					Pm5ktLocalManager.AirCondLearnedMaskFlag &= ~(1<<(Pm5ktLocalManager.IsAirCondLearn1363Proc-1));
				}*/
		}
		if(Pm5ktLocalManager.IsAirCondCtrl1363Proc && !Pm5ktLocalManager.DownComIsBeingUsed && !Pm5ktLocalManager.DownCom4NeedSend)
		{
				/*if(Pm5ktLocalManager.DownCom4NeedSend == 0)
				{
						IrInBuf[0] = Pm5ktLocalManager.IsAirCondCtrl1363Proc;
						IrInBuf[1] = 0x00;
						IrInBuf[2] = 0x00;
						memset(IrOutBuf,0,sizeof(IrOutBuf));
						if(modbus_tx_fun(PROTOCAL_ADDR,PROTOCAL_CODE_CTRL,IrInBuf,IrInLen,IrOutBuf,IrOutLen) == PROTOCAL_NORMAL)
						{
								memcpy(gUart_Pm5ktApi.DownComSendBuffer,IrOutBuf,IrOutLen);
								gUart_Pm5ktApi.DownComSendBufferPushOffset	= IrOutLen;
								gUart_Pm5ktApi.DownComSendBufferPopOffset		= 0;
								Pm5ktLocalManager.DownCom4NeedSend 					= 1;
								Pm5ktLocalManager.IsAirCondCtrl1363Proc			= 0;
						}
				}*/
				if(FUNC_RET_TRUE == InfraGetRecordTemp(&pTemp)){
					if(FUNC_RET_TRUE == GetInfraLearningCmdFromFlash(InfraGetManuID(),Pm5ktLocalManager.IsAirCondCtrl1363Proc-1,pTemp,sizeof(SaveRecordFormatDef))){
						CommunicationLightCtrl(1);
						SendInfraCmd(pTemp);//发送命令
						debug("%s %u SendInfraCmd Succ,CMDID=%u\n",__FUNCTION__,__LINE__,Pm5ktLocalManager.IsAirCondCtrl1363Proc);
						CommunicationLightCtrl(0);
						Pm5ktLocalManager.IsAirCondCtrl1363Proc	= 0;
						Pm5ktLocalManager.CommData.AirCondLastCtrlCmdId = Pm5ktLocalManager.IsAirCondCtrl1363Proc;
					}
					InfraFreeRecordTemp(&pTemp);
				}
				InfraFreeRecordTemp(&pTemp);
		}
		if(Pm5ktLocalManager.IsAirCondQuery1363Proc && !Pm5ktLocalManager.DownComIsBeingUsed && !Pm5ktLocalManager.DownCom4NeedSend)
		{
				/*if(Pm5ktLocalManager.DownCom4NeedSend == 0)
				{
						IrInBuf[0] = 0x01;
						IrInBuf[1] = 0xff;
						IrInBuf[2] = 0x01;
						memset(IrOutBuf,0,sizeof(IrOutBuf));
						if(modbus_tx_fun(PROTOCAL_ADDR,PROTOCAL_CODE_QUERY_START,IrInBuf,IrInLen,IrOutBuf,IrOutLen) == PROTOCAL_NORMAL)
						{
								memcpy(gUart_Pm5ktApi.DownComSendBuffer,IrOutBuf,IrOutLen);
								gUart_Pm5ktApi.DownComSendBufferPushOffset	= IrOutLen;
								gUart_Pm5ktApi.DownComSendBufferPopOffset		= 0;
								Pm5ktLocalManager.DownCom4NeedSend 					= 1;
								Pm5ktLocalManager.IsAirCondQuery1363Proc		= 0;
						}
				}*/
				Pm5ktLocalManager.IsAirCondQuery1363Proc		= 0;
		}
		if(Pm5ktLocalManager.IsAirCondSelfLearnClean && !Pm5ktLocalManager.DownComIsBeingUsed && !Pm5ktLocalManager.DownCom4NeedSend)
		{
				/*if(Pm5ktLocalManager.DownCom4NeedSend == 0)
				{
						memcpy(gUart_Pm5ktApi.DownComSendBuffer,IrOutBuf,IrOutLen);
						gUart_Pm5ktApi.DownComSendBuffer[0]					= 0x01;
						gUart_Pm5ktApi.DownComSendBuffer[1]					= 0x22;
						gUart_Pm5ktApi.DownComSendBuffer[2]					= 0x00;
						gUart_Pm5ktApi.DownComSendBuffer[3]					= 0x03;
						gUart_Pm5ktApi.DownComSendBuffer[4]					= 0x01;
						gUart_Pm5ktApi.DownComSendBuffer[5]					= 0x01;
						gUart_Pm5ktApi.DownComSendBuffer[6]					= 0x01;
						gUart_Pm5ktApi.DownComSendBuffer[7]					= 0x9c;
						gUart_Pm5ktApi.DownComSendBuffer[8]					= 0x96;
					
						gUart_Pm5ktApi.DownComSendBufferPushOffset	= 9;
						gUart_Pm5ktApi.DownComSendBufferPopOffset		= 0;
						Pm5ktLocalManager.DownCom4NeedSend 					= 1;
						Pm5ktLocalManager.IsAirCondSelfLearnClean		= 0;
				}*/
				if(FUNC_RET_TRUE == CleanInfraCmd()){
					Pm5ktLocalManager.IsAirCondSelfLearnClean		= 0;
					if(NeedReset(GetCID2())){
						debug("CleanInfraCmd Succ,\n");
						Pm5ktLocalManager.IsSysNeedReset = 1;
						Pm5ktLocalManager.SysResetWaitS = 0;
					}
				}
		}
		if(Pm5ktLocalManager.IsReadAirCondRunStatus==1 && !Pm5ktLocalManager.DownComIsBeingUsed && !Pm5ktLocalManager.DownCom4NeedSend)
		{
				/*IrInBuf[0] = 0x01;
				IrInBuf[1] = 0xff;
				IrInBuf[2] = 0x01;
				memset(IrOutBuf,0,sizeof(IrOutBuf));
				if(modbus_tx_fun(PROTOCAL_ADDR,PROTOCAL_CODE_QUERY_START,IrInBuf,IrInLen,IrOutBuf,IrOutLen) == PROTOCAL_NORMAL)
				{
						memcpy(gUart_Pm5ktApi.DownComSendBuffer,IrOutBuf,IrOutLen);
						gUart_Pm5ktApi.DownComSendBufferPushOffset	= IrOutLen;
						gUart_Pm5ktApi.DownComSendBufferPopOffset		= 0;
						Pm5ktLocalManager.DownCom4NeedSend 					= 1;
				}*/
				Pm5ktLocalManager.IsReadAirCondRunStatus++;
		}
		if(Pm5ktLocalManager.IsAirCondLearnWaiting == 1)
		{
				uint32_t LearnCmdID;
				uint8_t 			Rtn=0;
				uint8_t				ResBuf[30];
				uint32_t			ResLen=0;
				int32_t 			Ret=1;
				
                memset(ResBuf, 0, sizeof(ResBuf));
                ResLen = sizeof(ResBuf);
				LearnLightCtrl(1,0);
				if(Pm5ktLocalManager.AirCondLearnWaitTimeOut>=AIR_COND_LEARN_MAX_TIMEOUT_S)
				{
						
						
						Pm5ktLocalManager.IsAirCondLearnWaiting 	= 0;
						Pm5ktLocalManager.AirCondLearnWaitTimeOut = 0;
						if(Pm5ktLocalManager.AirCondLearnStatus){
						
							if((Pm5ktLocalManager.AirCondLearnedMaskFlag>>(Pm5ktLocalManager.AirCondLastLearnCmdId-1))&0x01)
									Rtn = YDT1363_3_PROTOCAL_RTN_LEARNING_SUCC;
							else
									Rtn = YDT1363_3_PROTOCAL_RTN_LEARNING_FAILURE;
							Ret = ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,ResBuf,&ResLen);
							if (Ret==0 && ResLen!=0)
							{
									uint8_t retval;

									retval= Comm_LineBufferToCycBuffer(ResBuf, ResLen, gUart_Pm5ktApi.UpComSendBuffer, 
																&gUart_Pm5ktApi.UpComSendBufferPushOffset, gUart_Pm5ktApi.UpComSendBufferPopOffset, PM5KT_UPCOM_SEND_BUFFER_SIZE);
									if(retval)
											Pm5ktLocalManager.UpComNeedSend	= 1;
							}
						}
						Pm5ktLocalManager.AirCondLearnStatus = 0;

						//UartsInit();
				}else{
				
					//学习命令来自于电总协议，学习命令号保存在AirCondLastLearnCmdId中
					if(Pm5ktLocalManager.AirCondLearnStatus){
						LearnCmdID = Pm5ktLocalManager.AirCondLastLearnCmdId-1;
					}else{
						LearnCmdID = Pm5ktLocalManager.AirCondLearnValue-1;
					}

					//UPCOM_UnInit();
					
					if(FUNC_RET_TRUE == InfraLearning(InfraGetManuID(),LearnCmdID)){
						Pm5ktLocalManager.AirCondLearnedMaskFlag |= (1<<(LearnCmdID));
						debug("%s AirCondLearnedMaskFlag:0x%08x\n",__FUNCTION__,Pm5ktLocalManager.AirCondLearnedMaskFlag);
						Pm5ktLocalManager.IsAirCondLearnWaiting 	= 0;

						if(Pm5ktLocalManager.AirCondLearnStatus){

							if((Pm5ktLocalManager.AirCondLearnedMaskFlag>>LearnCmdID)&0x01)
								Rtn = YDT1363_3_PROTOCAL_RTN_LEARNING_SUCC;
							else
								Rtn = YDT1363_3_PROTOCAL_RTN_LEARNING_FAILURE;
							Ret = ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(YDT1363_3_PROTOCAL_RTN_LEARNING_SUCC,ResBuf,&ResLen);
							if (Ret==0 && ResLen!=0)
							{
									uint8_t retval;

									retval= Comm_LineBufferToCycBuffer(ResBuf, ResLen, gUart_Pm5ktApi.UpComSendBuffer, 
																&gUart_Pm5ktApi.UpComSendBufferPushOffset, gUart_Pm5ktApi.UpComSendBufferPopOffset, PM5KT_UPCOM_SEND_BUFFER_SIZE);
									if(retval)
											Pm5ktLocalManager.UpComNeedSend	= 1;
							}
						}

					
						Pm5ktLocalManager.IsAirCondLearnWaiting 	= 0;
						Pm5ktLocalManager.AirCondLearnWaitTimeOut = 0;
						Pm5ktLocalManager.AirCondLearnStatus = 0;

						//UartsInit();
					}else{
						Pm5ktLocalManager.AirCondLearnedMaskFlag &= ~(1<<(LearnCmdID));
					}
						
				}
				
		}
}
static void SysTimeProcess_1S(void)
{
		if(Pm5ktLocalManager.IsAirCondLearnWaiting == 1)
		{
				Pm5ktLocalManager.AirCondLearnWaitTimeOut++;
				debug("AirCondLearnWaiting:%d\n",Pm5ktLocalManager.AirCondLearnWaitTimeOut);
		}
		//控制学习指示灯亮灭
		if((Pm5ktLocalManager.IsShowAirCondLearnStatus)&&(!Pm5ktLocalManager.IsAirCondLearnWaiting))
		{
				debug("IsShowAirCondLearnStatus AirCondMenuType=%d AirCondLearnedMaskFlag=0x%08x\n",Pm5ktLocalManager.AirCondMenuType,Pm5ktLocalManager.AirCondLearnedMaskFlag);
				if(Pm5ktLocalManager.AirCondMenuType == AIR_COND_MENU_TYPE_READ_LEARN_STATUS)
				{
					
						if((Pm5ktLocalManager.AirCondLearnedMaskFlag>>(Pm5ktLocalManager.AirCondQueryLearnStatus-1))&0x01)
						{
								//Pm5ktLocalManager.IsSysRunLight = !Pm5ktLocalManager.IsSysRunLight;
							LearnLightCtrl(0,1);
							debug("LearnLightCtrl On\n");
							//return;
						}else{
							LearnLightCtrl(0,0);	
							debug("LearnLightCtrl Off\n");
						}
				}
				if(Pm5ktLocalManager.AirCondMenuType == AIR_COND_MENU_TYPE_LEARN)
				{
						if( (Pm5ktLocalManager.AirCondLearnedMaskFlag>>(Pm5ktLocalManager.AirCondLearnValue-1))&0x01)
						{
							//Pm5ktLocalManager.IsSysRunLight = !Pm5ktLocalManager.IsSysRunLight;
							LearnLightCtrl(0,1);
							debug("LearnLightCtrl On\n");
							//return;
						}else{
							LearnLightCtrl(0,0);	
							debug("LearnLightCtrl Off\n");
						}
				}
				Pm5ktLocalManager.IsShowAirCondLearnStatus = 0;
		}
		
		Pm5ktLocalManager.IsSysRunLight = !Pm5ktLocalManager.IsSysRunLight;
		RunLightCtrl(Pm5ktLocalManager.IsSysRunLight);
		
		if(!Pm5ktLocalManager.DownComIsBeingUsed && !Pm5ktLocalManager.DownCom4NeedSend){
			if(SoftTimerIsStart(TIMER_GET_REAL_DATA)){
				SoftTimerStop(TIMER_GET_REAL_DATA);
			}
			SoftTimerStart(TIMER_GET_REAL_DATA,20,0,NULL,NULL);
			GetRealTimeData(&Pm5ktLocalManager.CommData);
			if(SoftTimerIsOver(TIMER_GET_REAL_DATA)){
				FailureLightCtrl(1);
			}else{
				FailureLightCtrl(0);
			}
		}
				
		if(Pm5ktLocalManager.IsSysStablility==0 && Pm5ktLocalManager.TimSysTimeCountS >10)
				Pm5ktLocalManager.IsSysStablility = 1;
}
static void SysTimeProcess_5S(void)
{	
		AirCondAlarmJudge();
}
static void SysTimeProcess_15S(void)
{
		
}
static void SysTimeProcess_60S(void)
{	
		if(!Pm5ktLocalManager.DownComIsBeingUsed && !Pm5ktLocalManager.DownCom4NeedSend)
				SaveFrequentUpdateData(&Pm5ktLocalManager.CommData);
}
static uint8_t ProcessDownCom1RecvData(uint8_t *pRecvBuffer, uint16_t RecvLen)
{
		if(pRecvBuffer==NULL || RecvLen==0)
				return 0;
		
		return 1;
}
static uint8_t ProcessDownCom2RecvData(uint8_t *pRecvBuffer, uint16_t RecvLen)
{
		if(pRecvBuffer==NULL || RecvLen==0)
				return 0;
		
		return 1;
}
static uint8_t ProcessDownCom3RecvData(uint8_t *pRecvBuffer, uint16_t RecvLen)
{
		if(pRecvBuffer==NULL || RecvLen==0)
				return 0;
		
		return 1;
}
/*
static uint8_t ProcessDownCom4RecvData(uint8_t *pRecvBuffer, uint16_t RecvLen)
{
		uint8_t				IrOutBuf[256],i=0;
		unsigned int	IrOutLen=sizeof(IrOutBuf);
        static uint8_t Sta;
	
		if(pRecvBuffer==NULL || RecvLen<=3)
				return 0;

		if(pRecvBuffer[0]==0xc3)
		{
				pRecvBuffer=pRecvBuffer+1;
				RecvLen=RecvLen-1;
		}
#ifdef AIR_COND_DEBUG
		{
				uint8_t retval;
	
				retval= Comm_LineBufferToCycBuffer(pRecvBuffer, RecvLen, gUart_Pm5ktApi.UpComSendBuffer, 
											&gUart_Pm5ktApi.UpComSendBufferPushOffset, gUart_Pm5ktApi.UpComSendBufferPopOffset, PM5KT_UPCOM_SEND_BUFFER_SIZE);
				if(retval)
						Pm5ktLocalManager.UpComNeedSend	= 1;
		}
#endif
		Comm_CtrlLightOnOff(LED_COMMUNICATION,Sta);
        Sta = (Sta==0?0x1:0x0);
		memset(IrOutBuf,0,sizeof(IrOutBuf));
		if(modbus_rx_fun(pRecvBuffer,RecvLen,IrOutBuf,&IrOutLen) == PROTOCAL_NORMAL)
		{

#ifdef AIR_COND_DEBUG
				{
						uint8_t retval;
			
						retval= Comm_LineBufferToCycBuffer(IrOutBuf, IrOutLen, gUart_Pm5ktApi.UpComSendBuffer, 
													&gUart_Pm5ktApi.UpComSendBufferPushOffset, gUart_Pm5ktApi.UpComSendBufferPopOffset, PM5KT_UPCOM_SEND_BUFFER_SIZE);
						if(retval)
								Pm5ktLocalManager.UpComNeedSend	= 1;
				}
#endif
				switch(*(pRecvBuffer+1))
				{
						case PROTOCAL_CODE_CTRL:       																					//05
								Pm5ktLocalManager.IsReadAirCondRunStatus = 1;
								break;
						case PROTOCAL_CODE_LEARN:      																					//06
								if(Pm5ktLocalManager.AirCondLearnStatus == 1)
										Pm5ktLocalManager.AirCondLearnStatus = 2;
								break;
						case PROTOCAL_CODE_CHECK_STA:  																					//21H
								Pm5ktLocalManager.AirCondLearnedMaskFlag = 0;
								for(i=0;i<IrOutLen && i<31;i++)
								{
										Pm5ktLocalManager.AirCondLearnedMaskFlag |= 1<<IrOutBuf[i];
								}
								if(Pm5ktLocalManager.IsAirCondLearnWaiting == 1)
										Pm5ktLocalManager.AirCondLearnWaitTimeOut = AIR_COND_LEARN_MAX_TIMEOUT_S;
								break;
						case PROTOCAL_CODE_QUERY_START:    																			//28H
								Pm5ktLocalManager.IsReadAirCondRunStatus 		= 22;
								Pm5ktLocalManager.AirCondSwitchOnOffStatus 	= IrOutBuf[1];
								Pm5ktLocalManager.AirCondSettingTempStatus	= IrOutBuf[2];
								Pm5ktLocalManager.AirCondRunModeStatus			= IrOutBuf[3];
								Pm5ktLocalManager.AirCondWindSpeedStatus		= IrOutBuf[4];
								
								break;
						default:
								return 0;
				}
		}	
		
		return 1;
}

*/

//#define TEST_TIMER_INIT_PARAM
int main()
{
	PCOMM_DATA pData;
	uint8_t LastMinute=0;
    uint32_t SaveHtyFlag=0;
    uint32_t u32Temp,i;
    //InfraCodeDef InfraRecv;
    SaveRecordFormatDef *pSaveRecord=NULL;
    
    SYS_Init();
    Timer0Init();
	Timer1Init();
    //Timer2Init();
    Timer3Init();
    UartsInit();
	HC595_Init();
	GpioIntInit();
	
	RX8010_Init();
	AT45DBXXX_Init();
    SoftTimerInit();
    InfraInit();
    
	debug("Sys Satrt\n");
	Pm5ktLocalManager.CommData.YdtData = ProtocolProc_YDT1363_3_CallBack();
	Pm5ktLocalManager.CommData.YdtData->pRn8209c = &Pm5ktLocalManager.CommData.Rn8209c;
	pData = &Pm5ktLocalManager.CommData;
    //添加2s初始化参数超时，超时后故障灯亮
	SoftTimerStart(TIMER_INIT_PARAM,20,0,NULL,NULL);
	MeterDataInit(1,pData);
	RN8209C_Init(&pData->Rn8209c);
	//Pm5ktLocalManager.CommData.AirCondLastCtrlCmdId = (uint8_t)pData->AirCondLastCtrlCmdId; //保存上次控制命令序号，掉电不易失
	SysDataInit(); 
	
    #ifdef TEST_TIMER_INIT_PARAM
        DelayUs(3000*1000);
    #endif
	
	if(SoftTimerIsOver(TIMER_INIT_PARAM)){
		FailureLightCtrl(1);
	}else{
		FailureLightCtrl(0);
	}
	PWM5Init((uint32_t)pData->YdtData->InfraredParam.TransmitFreq,50);
    GPIO_SetMode(P2, BIT6, GPIO_PMD_OUTPUT);
    IRTX = IRTX_HIGH;
	
    do
	{
        
		if(Pm5ktLocalManager.IsSysTimeUpdate_250MS)
		{
				Pm5ktLocalManager.IsSysTimeUpdate_250MS = 0;
				SysTimeProcess_250MS();
				
		}
		if(Pm5ktLocalManager.IsSysTimeUpdate_1S)
		{
				Pm5ktLocalManager.IsSysTimeUpdate_1S 		= 0;
                //HC595_RunLight(Pm5ktLocalManager.IsSysRunLight);
				SysTimeProcess_1S();
                
				if(LastMinute != pData->YdtData->Date.Minute)
				{
						LastMinute = pData->YdtData->Date.Minute;
						SaveHtyFlag = 0;
				}
				if(pData->YdtData->Date.Year>1900)
						SaveHistoryData(&SaveHtyFlag,pData);
				
				if(Pm5ktLocalManager.IsSysNeedReset){
					Pm5ktLocalManager.SysResetWaitS++;
					if(Pm5ktLocalManager.SysResetWaitS>=SYS_RESET_WAIT_S_MAX){
						Pm5ktLocalManager.IsSysNeedReset = 0;
						debug("Sys Reset!\n");
						Sys_Soft_Reset();
					}
				}
		}
		if(Pm5ktLocalManager.IsSysTimeUpdate_5S)
		{
				Pm5ktLocalManager.IsSysTimeUpdate_5S 		= 0;
				GetReturnTemperature(&pData->YdtData->RTempRadio,&pData->YdtData->Analog.ReturnTemperature);
                GetSupplyTemperature(&pData->YdtData->STempRadio,&pData->YdtData->Analog.SupplyTemperature);
				if(Pm5ktLocalManager.IsShowRTemperature){
					HC595_LedDisp(4000+(uint8_t)pData->YdtData->Analog.ReturnTemperature*10);
				}
				if(Pm5ktLocalManager.IsShowSTemperature){
					HC595_LedDisp(5000+(uint8_t)pData->YdtData->Analog.SupplyTemperature*10);
				}
				SysTimeProcess_5S();
		}
		if(Pm5ktLocalManager.IsSysTimeUpdate_15S)
		{
				Pm5ktLocalManager.IsSysTimeUpdate_15S 	= 0;
				SysTimeProcess_15S();
		}
		if(Pm5ktLocalManager.IsSysTimeUpdate_60S)
		{
				Pm5ktLocalManager.IsSysTimeUpdate_60S 	= 0;
				SysTimeProcess_60S();
				RN8209C_RegularCheckParam(&pData->Rn8209c);
		}
		
		if(gUart_Pm5ktApi.IsUpComRecv)
		{
				if(gUart_Pm5ktApi.ProtDataLen<64)
						gUart_Pm5ktApi.IsUpComRecv 					= 0;
				Comm_CycBufferToLineBuffer(gUart_Pm5ktApi.UpComRecvBuffer, gUart_Pm5ktApi.UpComRecvBufferPushOffset, &gUart_Pm5ktApi.UpComRecvBufferPopOffset, PM5KT_UPCOM_RECV_BUFFER_SIZE, 
								gUart_Pm5ktApi.ProtBuffer, &gUart_Pm5ktApi.ProtDataLen, PM5KT_PROTOCOL_BUFFER_SIZE);
				ProtBufProcessEx();
		}
		if(Pm5ktLocalManager.UpComNeedSend)
		{
				Pm5ktLocalManager.UpComNeedSend 				= 0;
				Uart_ComSendFromCycBuffer(0, gUart_Pm5ktApi.UpComSendBuffer,gUart_Pm5ktApi.UpComSendBufferPushOffset, &gUart_Pm5ktApi.UpComSendBufferPopOffset, PM5KT_UPCOM_SEND_BUFFER_SIZE);
				if(gUart_Pm5ktApi.UpComSendBufferPushOffset != gUart_Pm5ktApi.UpComSendBufferPopOffset)
						Uart_ComSendFromCycBuffer(0, gUart_Pm5ktApi.UpComSendBuffer,gUart_Pm5ktApi.UpComSendBufferPushOffset, &gUart_Pm5ktApi.UpComSendBufferPopOffset, PM5KT_UPCOM_SEND_BUFFER_SIZE);
				
		}
		if(Pm5ktLocalManager.DownCom1NeedSend && !Pm5ktLocalManager.DownComIsBeingUsed)
		{
				UART_CS(0);
				gUart_Pm5ktApi.DownComRecvBufferPushOffset = gUart_Pm5ktApi.DownComRecvBufferPopOffset = 0;
				UART_CS(1);
				Uart_ComSendFromCycBuffer(1, gUart_Pm5ktApi.DownComSendBuffer,gUart_Pm5ktApi.DownComSendBufferPushOffset, &gUart_Pm5ktApi.DownComSendBufferPopOffset, PM5KT_DOWNCOM_SEND_BUFFER_SIZE);
				if(gUart_Pm5ktApi.DownComSendBufferPushOffset != gUart_Pm5ktApi.DownComSendBufferPopOffset)
						Uart_ComSendFromCycBuffer(1, gUart_Pm5ktApi.DownComSendBuffer,gUart_Pm5ktApi.DownComSendBufferPushOffset, &gUart_Pm5ktApi.DownComSendBufferPopOffset, PM5KT_DOWNCOM_SEND_BUFFER_SIZE);
				Pm5ktLocalManager.DownCom1NeedSend 			= 0;
				Pm5ktLocalManager.DownCom1RecvTimeOut 	= Pm5ktLocalManager.TimSysTimeCountS + DOWNCOM1_RECV_TIMEOUT_S;
				Pm5ktLocalManager.DownComIsBeingUsed 		= 1;
				
		}
		if(Pm5ktLocalManager.TimSysTimeCountS > Pm5ktLocalManager.DownCom1RecvTimeOut)
		{
				uint16_t tmpRecvlen = gUart_Pm5ktApi.DownComRecvBufferPushOffset - gUart_Pm5ktApi.DownComRecvBufferPopOffset;
				UART_CS(0);
				Pm5ktLocalManager.DownCom1RecvTimeOut 	= DOWNCOM_MAX_RECV_TIMEOUT;
				ProcessDownCom1RecvData(gUart_Pm5ktApi.DownComRecvBuffer,tmpRecvlen);
				Pm5ktLocalManager.DownComIsBeingUsed 		= 0;
		}
		if(Pm5ktLocalManager.DownCom2NeedSend && !Pm5ktLocalManager.DownComIsBeingUsed)
		{
				UART_CS(0);
				gUart_Pm5ktApi.DownComRecvBufferPushOffset = gUart_Pm5ktApi.DownComRecvBufferPopOffset = 0;
				UART_CS(2);
				Uart_ComSendFromCycBuffer(2, gUart_Pm5ktApi.DownComSendBuffer,gUart_Pm5ktApi.DownComSendBufferPushOffset, &gUart_Pm5ktApi.DownComSendBufferPopOffset, PM5KT_DOWNCOM_SEND_BUFFER_SIZE);
				if(gUart_Pm5ktApi.DownComSendBufferPushOffset != gUart_Pm5ktApi.DownComSendBufferPopOffset)
						Uart_ComSendFromCycBuffer(2, gUart_Pm5ktApi.DownComSendBuffer,gUart_Pm5ktApi.DownComSendBufferPushOffset, &gUart_Pm5ktApi.DownComSendBufferPopOffset, PM5KT_DOWNCOM_SEND_BUFFER_SIZE);
				Pm5ktLocalManager.DownCom2NeedSend 			= 0;
				Pm5ktLocalManager.DownCom2RecvTimeOut 	= Pm5ktLocalManager.TimSysTimeCountS + DOWNCOM2_RECV_TIMEOUT_S;
				Pm5ktLocalManager.DownComIsBeingUsed 		= 1;
				
		}
		if(Pm5ktLocalManager.TimSysTimeCountS > Pm5ktLocalManager.DownCom2RecvTimeOut)
		{
				uint32_t tmpRecvlen = gUart_Pm5ktApi.DownComRecvBufferPushOffset - gUart_Pm5ktApi.DownComRecvBufferPopOffset;
				UART_CS(0);
				Pm5ktLocalManager.DownCom2RecvTimeOut 	= DOWNCOM_MAX_RECV_TIMEOUT;
				ProcessDownCom2RecvData(gUart_Pm5ktApi.DownComRecvBuffer,tmpRecvlen);
				Pm5ktLocalManager.DownComIsBeingUsed 		= 0;
		}
		if(Pm5ktLocalManager.DownCom3NeedSend && !Pm5ktLocalManager.DownComIsBeingUsed)
		{
				UART_CS(0);
				gUart_Pm5ktApi.DownComRecvBufferPushOffset = gUart_Pm5ktApi.DownComRecvBufferPopOffset = 0;
				UART_CS(3);
				Uart_ComSendFromCycBuffer(3, gUart_Pm5ktApi.DownComSendBuffer,gUart_Pm5ktApi.DownComSendBufferPushOffset, &gUart_Pm5ktApi.DownComSendBufferPopOffset, PM5KT_DOWNCOM_SEND_BUFFER_SIZE);
				if(gUart_Pm5ktApi.DownComSendBufferPushOffset != gUart_Pm5ktApi.DownComSendBufferPopOffset)
						Uart_ComSendFromCycBuffer(3, gUart_Pm5ktApi.DownComSendBuffer,gUart_Pm5ktApi.DownComSendBufferPushOffset, &gUart_Pm5ktApi.DownComSendBufferPopOffset, PM5KT_DOWNCOM_SEND_BUFFER_SIZE);
				Pm5ktLocalManager.DownCom3NeedSend 			= 0;
				Pm5ktLocalManager.DownCom3RecvTimeOut 	= Pm5ktLocalManager.TimSysTimeCountS + DOWNCOM3_RECV_TIMEOUT_S;
				Pm5ktLocalManager.DownComIsBeingUsed 		= 1;
				
		}
		if(Pm5ktLocalManager.TimSysTimeCountS > Pm5ktLocalManager.DownCom3RecvTimeOut)
		{
				uint16_t tmpRecvlen = gUart_Pm5ktApi.DownComRecvBufferPushOffset - gUart_Pm5ktApi.DownComRecvBufferPopOffset;
				UART_CS(0);
				Pm5ktLocalManager.DownCom3RecvTimeOut 	= DOWNCOM_MAX_RECV_TIMEOUT;
				ProcessDownCom3RecvData(gUart_Pm5ktApi.DownComRecvBuffer,tmpRecvlen);
				Pm5ktLocalManager.DownComIsBeingUsed 		= 0;
		}
		if(Pm5ktLocalManager.DownCom4NeedSend && !Pm5ktLocalManager.DownComIsBeingUsed)
		{
#ifdef AIR_COND_DEBUG
				if(gUart_Pm5ktApi.DownComSendBufferPushOffset != gUart_Pm5ktApi.DownComSendBufferPopOffset)
				{
						uint8_t retval;

						retval= Comm_LineBufferToCycBuffer(gUart_Pm5ktApi.DownComSendBuffer, gUart_Pm5ktApi.DownComSendBufferPushOffset-gUart_Pm5ktApi.DownComSendBufferPopOffset, gUart_Pm5ktApi.UpComSendBuffer, 
													&gUart_Pm5ktApi.UpComSendBufferPushOffset, gUart_Pm5ktApi.UpComSendBufferPopOffset, PM5KT_UPCOM_SEND_BUFFER_SIZE);
						if(retval)
								Pm5ktLocalManager.UpComNeedSend	= 1;
				}
#endif
				UART_CS(0);
				gUart_Pm5ktApi.DownComRecvBufferPushOffset = gUart_Pm5ktApi.DownComRecvBufferPopOffset = 0;
				UART_CS(4);
				Uart_ComSendFromCycBuffer(4, gUart_Pm5ktApi.DownComSendBuffer,gUart_Pm5ktApi.DownComSendBufferPushOffset, &gUart_Pm5ktApi.DownComSendBufferPopOffset, PM5KT_DOWNCOM_SEND_BUFFER_SIZE);
				if(gUart_Pm5ktApi.DownComSendBufferPushOffset != gUart_Pm5ktApi.DownComSendBufferPopOffset)
						Uart_ComSendFromCycBuffer(4, gUart_Pm5ktApi.DownComSendBuffer,gUart_Pm5ktApi.DownComSendBufferPushOffset, &gUart_Pm5ktApi.DownComSendBufferPopOffset, PM5KT_DOWNCOM_SEND_BUFFER_SIZE);
				Pm5ktLocalManager.DownCom4NeedSend 			= 0;
				Pm5ktLocalManager.DownCom4RecvTimeOut 	= Pm5ktLocalManager.TimSysTimeCountS + DOWNCOM4_RECV_TIMEOUT_S;
				Pm5ktLocalManager.DownComIsBeingUsed 		= 1;
			
		}
		if(Pm5ktLocalManager.TimSysTimeCountS > Pm5ktLocalManager.DownCom4RecvTimeOut)
		{
				uint16_t tmpRecvlen = gUart_Pm5ktApi.DownComRecvBufferPushOffset - gUart_Pm5ktApi.DownComRecvBufferPopOffset;
				UART_CS(0);
				Pm5ktLocalManager.DownCom4RecvTimeOut 	= DOWNCOM_MAX_RECV_TIMEOUT;
				//ProcessDownCom4RecvData(gUart_Pm5ktApi.DownComRecvBuffer,tmpRecvlen);
				Pm5ktLocalManager.DownComIsBeingUsed 		= 0;
		}
	}while(1);

}

/*** (C) COPYRIGHT 2017	Jsyaao Technology Corp. ***/
