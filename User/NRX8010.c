/******************************************************************************
 * @file     	NRX8010.c
 * @version  	V1.00
 * $Revision: 3 $
 * $Date: 		17/07/04 11:45a $
 * @brief    	RX8010 chip
 * @auther		ljj
 * @note
 * Copyright (C) 2017 Jsyaao Technology Corp. All rights reserved.
*****************************************************************************/

#include "NRX8010.h"


#define BCD2BIN(val)			(((val) & 0x0f) + ((val)>>4)*10)
#define BIN2BCD(val)			((((val)/10)<<4) + (val)%10)


#define DATE_YEAR_IS_VALID(x) (((x)>=2000)&&((x)<=2099))
#define DATE_MONTH_IS_VALID(x) (((x)>=1)&&((x)<=12))
#define DATE_DAY_IS_VALID(x) (((x)>=1)&&((x)<=31))
#define DATE_HOUR_IS_VALID(x) (((x)>=0)&&((x)<=23))
#define DATE_MINUTE_OR_SECOND_IS_VALID(x) (((x)>=0)&&((x)<=59))

static void Comm_DelayUs(char MicroS)
{
    int i;
    for(i=0;i<10*MicroS;i++);
}

static void RX8010_IIC_Init(void) //???IIC
{
    //EESDA P45
    GPIO_SetMode(P4, BIT5, GPIO_PMD_QUASI);
    //EESCL P44
    GPIO_SetMode(P4, BIT4, GPIO_PMD_OUTPUT);
    RX8010_PIN_SDA = 1;
    RX8010_PIN_SCL = 1;
}

static void RX8010_I2C_Start(void)
{
    Comm_DelayUs(2);
	
	RX8010_PIN_SCL = 1;
	RX8010_PIN_SDA = 1;
    Comm_DelayUs(2);
	RX8010_PIN_SDA = 0;
    Comm_DelayUs(4);
}

static void RX8010_I2C_Stop(void)
{
    Comm_DelayUs(2);
	RX8010_PIN_SDA = 0;
    Comm_DelayUs(2);
	RX8010_PIN_SCL = 1;
    Comm_DelayUs(8);
	RX8010_PIN_SDA = 1;
    Comm_DelayUs(2);
}

static void RX8010_I2C_SendOneByte(unsigned char c)
{
    int i; 
    for (i=0; i<8; i++)
    {
        Comm_DelayUs(2);
		RX8010_PIN_SCL = 0;
        Comm_DelayUs(2);

        if (c & (1<<(7-i)))
			RX8010_PIN_SDA = 1;
        else
			RX8010_PIN_SDA = 0;

        Comm_DelayUs(2);
        RX8010_PIN_SCL = 1;
        Comm_DelayUs(2);
        RX8010_PIN_SCL = 0;
    }
    Comm_DelayUs(4); 
}

static unsigned char RX8010_I2C_RecvOneByte(void)
{
    unsigned char j=0;
    int i;
	int data1,data2,data3;

    for (i=0; i<8; i++)
    {
        Comm_DelayUs(2);
        RX8010_PIN_SCL = 0;
		Comm_DelayUs(2);
        RX8010_PIN_SCL = 1;
        Comm_DelayUs(2);

		data1 = RX8010_PIN_SDA;
		Comm_DelayUs(4);
		data2 = RX8010_PIN_SDA;
		Comm_DelayUs(4);
		data3 = RX8010_PIN_SDA;
        if (data1 && data2 && data3)
            j+=(1<<(7-i));

        Comm_DelayUs(2);
        RX8010_PIN_SCL = 0;
    }
    
    Comm_DelayUs(2);

    return j;
}

static int RX8010_I2C_RecvAck(void)
{
    int nack;
	int nack2;
	int nack3;
    
	Comm_DelayUs(2);

    RX8010_PIN_SCL = 0;
    Comm_DelayUs(2);
    RX8010_PIN_SCL = 1;
    //Comm_DelayUs(1);
	Comm_DelayUs(8);
    nack = RX8010_PIN_SDA;
	Comm_DelayUs(4);
	nack2 = RX8010_PIN_SDA;
	Comm_DelayUs(4);
	nack3 = RX8010_PIN_SDA;

    Comm_DelayUs(2);
    RX8010_PIN_SCL = 0;
    Comm_DelayUs(2);

    if (nack == 0 && nack2 == 0 && nack3 == 0)
        return 1; 

    return 0;
}

static void RX8010_I2C_SendAck(void)
{
    Comm_DelayUs(2);
    RX8010_PIN_SDA = 0;
    Comm_DelayUs(2);
    RX8010_PIN_SCL = 1;
    Comm_DelayUs(4);
    RX8010_PIN_SCL = 0;
    Comm_DelayUs(2);
	RX8010_PIN_SDA = 1;
    Comm_DelayUs(2);
}

static void RX8010_I2C_SendNack(void)
{
    Comm_DelayUs(2);
    RX8010_PIN_SDA = 1;
    Comm_DelayUs(2);
    RX8010_PIN_SCL = 1;
    Comm_DelayUs(4);
    RX8010_PIN_SCL = 0;
    Comm_DelayUs(2);
    RX8010_PIN_SDA = 0;
    Comm_DelayUs(2);
}

static unsigned char GetRX8010(unsigned char firsttype,unsigned char count,unsigned char *buff)
{
		unsigned char i; 	
		
		RX8010_I2C_Start();
		RX8010_I2C_SendOneByte((unsigned char)(0x64));
		if (!RX8010_I2C_RecvAck())
		{
				RX8010_I2C_Stop();
				debug("GetRX8010 function step1 no ack\n");
				return	0;
		}
		RX8010_I2C_SendOneByte((unsigned char)(firsttype));
		if (!RX8010_I2C_RecvAck())
		{
				RX8010_I2C_Stop();
				debug("GetRX8010 function step2 no ack\n");
				return	0;
		}
		RX8010_I2C_Start();
		RX8010_I2C_SendOneByte((unsigned char)(0x65));
		if (!RX8010_I2C_RecvAck())
		{
				RX8010_I2C_Stop();
				debug("GetRX8010 function step3 no ack\n");
				return	0;
		}
		for (i=0;i<count;i++)
		{
				buff[i]=RX8010_I2C_RecvOneByte();
				if (i!=count-1) 
					RX8010_I2C_SendAck();
		}
		RX8010_I2C_SendNack();
		RX8010_I2C_Stop();
		
		return	1;
}
static unsigned char SetRX8010(unsigned char timetype,unsigned char value)
{		
		RX8010_I2C_Start();
		RX8010_I2C_SendOneByte((unsigned char)(0x64));
		if (!RX8010_I2C_RecvAck())
		{
				RX8010_I2C_Stop();
				debug("SetRX8010 function step1 no ack\n");
				return	0;
		}
		RX8010_I2C_SendOneByte((unsigned char)(timetype));
		if (!RX8010_I2C_RecvAck())
		{
				RX8010_I2C_Stop();
				debug("SetRX8010 function step1 no ack\n");
				return	0;
		}
		RX8010_I2C_SendOneByte((unsigned char)(value));
		if (!RX8010_I2C_RecvAck())
		{
				RX8010_I2C_Stop();
				debug("SetRX8010 function step1 no ack\n");
				return	0;
		}
		RX8010_I2C_Stop();
		
		return	1;
}

extern unsigned char RX8010_Check(void)
{
    unsigned char cTemp;
    return !GetRX8010(0x10,1,&cTemp);
}

extern unsigned char RX8010_GetTime(PPM5KT_COMM_TM rtc_tm)
{
    PM5KT_COMM_TM Tm={0};
    uint8_t u8Temp;
    
    if(GetRX8010(0x10,1,&Tm.Second) == 0)
            return 0; 
    if(GetRX8010(0x11,1,&Tm.Minute) == 0)
            return 0; 
    if(GetRX8010(0x12,1,&Tm.Hour) == 0)
            return 0; 
    if(GetRX8010(0x14,1,&Tm.Day) == 0)
            return 0; 
    if(GetRX8010(0x15,1,&Tm.Month) == 0)
            return 0; 
    if(GetRX8010(0x16,1,&u8Temp) == 0)
            return 0;  
    
    Tm.Second	&=0x7f;
    Tm.Minute	&=0x7f;
    Tm.Hour		&=0x3f;
    Tm.Day		&=0x3f;
    Tm.Month	&=0x1f;
    Tm.Year = u8Temp;
    
    Tm.Second 	= BCD2BIN(Tm.Second);
    Tm.Minute 	= BCD2BIN(Tm.Minute);
    Tm.Hour 	= BCD2BIN(Tm.Hour);
    Tm.Day 		= BCD2BIN(Tm.Day);
    Tm.Month 	= BCD2BIN(Tm.Month);
    Tm.Year 	= 2000+BCD2BIN(Tm.Year);
    //rtc_tm->Month--;

    /*if(!DATE_YEAR_IS_VALID(Tm.Year))
        return 0;
    if(!DATE_MONTH_IS_VALID(Tm.Month))
        return 0;
    if(!DATE_DAY_IS_VALID(Tm.Day))
        return 0;
    if(!DATE_HOUR_IS_VALID(Tm.Hour))
        return 0;
    if(!DATE_MINUTE_OR_SECOND_IS_VALID(Tm.Minute))
        return 0;
    if(!DATE_MINUTE_OR_SECOND_IS_VALID(Tm.Second))
        return 0;
    */
    memcpy((void *)rtc_tm,(void *)&Tm,sizeof(PM5KT_COMM_TM));
    return 1;
}

extern unsigned char RX8010_SetTime(PPM5KT_COMM_TM rtc_tm)
{
		unsigned char mon, day, hrs, min, sec;
		unsigned int yrs;

		yrs = rtc_tm->Year - 2000;
		//mon = rtc_tm->Month + 1;   /* tm_mon starts at zero */
        mon = rtc_tm->Month;
		day = rtc_tm->Day;
		hrs = rtc_tm->Hour;
		min = rtc_tm->Minute;
		sec = rtc_tm->Second;

		
		sec = BIN2BCD(sec);
		min = BIN2BCD(min);
		hrs = BIN2BCD(hrs);
		day = BIN2BCD(day);
		mon = BIN2BCD(mon);
		yrs = BIN2BCD(yrs);

		if(SetRX8010(0x10,sec)==0)
				return 0;
		if(SetRX8010(0x11,min)==0)
				return 0;
		if(SetRX8010(0x12,hrs)==0)
				return 0;
		if(SetRX8010(0x14,day)==0)
				return 0;
		if(SetRX8010(0x15,mon)==0)
				return 0;
		if(SetRX8010(0x16,yrs)==0)
				return 0;

		return 1;
}

extern void RX8010_Init(void)
{
    RX8010_IIC_Init();
    /*SetRX8010(0x17,0xd8);
    SetRX8010(0x30,0x00);
    SetRX8010(0x31,0x08);
    SetRX8010(0x32,0x00);
    SetRX8010(0x1d,0x04);
    SetRX8010(0x1e,0x00);
    SetRX8010(0x1f,0x00);*/
    #ifdef RX8010_TEST
        PM5KT_COMM_TM TmTemp;
        RX8010_Check();
        TmTemp.Year = 2017;
        TmTemp.Month = 9;
        TmTemp.Day = 12;
        TmTemp.Hour = 14;
        TmTemp.Minute = 3;
        RX8010_SetTime(&TmTemp);
    #endif
    
}
