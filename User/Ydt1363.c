/**
  *****************************************************************************
  *                            电总YDT1363_3协议
  *
  *                       (C) Copyright 2017-2018, ***
  *                            All Rights Reserved
  *****************************************************************************
  *
  * @File    : Ydt1363.c
  * @By      : XFW
  * @Version : V1.0
  * @Date    : 2017 / 03 / 28
  *
  *****************************************************************************
  *                                   Update
  * @Version : V1.0
  * @By      : XFW
  * @Date    : 2017 / 03 / 08
  * @Brief   : The Fisrt Edition
  *
  *
  *****************************************************************************
**/

#include "Ydt1363.h"
#include <string.h>
#include <stdlib.h>
#include "NRX8010.h"
#include "InfraRecord.h"
#include "UserCommon.h"
#include "Usart.h"
#include "SoftTimer.h"


#define TRUE 					1
#define FALSE 				0


YDT1363_DATA 	gYdtData;
uint8_t 			gu8Prot_CID2;
uint8_t 			gu8DataFlag=0;

#define APROM_TO_LDROM      0x5aa5aa50 //APROM跳入LDROM标记（最后四位为Boot波特率）
#define APROM_TO_LDROM_MSK  0xfffffff0
/*#define LDROM_BAUDRATE_1200      0x0
#define LDROM_BAUDRATE_2400      0x01
#define LDROM_BAUDRATE_4800      0x02*/
#define LDROM_BAUDRATE_9600      0x03
#define LDROM_BAUDRATE_19200      0x04
#define MIN(x,y) (((x)>(y))?(y):(x))
#define MAX(x,y) (((x)>(y))?(x):(y))

/*
 ************************************************************************************************************************************************************************     
 *函数名称:	HexToAscii
 *功能描述:	二进制转换为ASCII
 *输入描述:	原始数据，转换缓冲区，原始数据长度，字母是否大写
 *输出描述:	转换数据
 *返回描述:	
 *作者日期:	
 *全局声明: 
 *特殊说明: 转换缓冲区长度必须大于等于原始数据长度的2倍，转换后顺序为高位在前
 ************************************************************************************************************************************************************************       
 */
 
extern void HexToAscii(const uint8_t *oBuf, uint8_t *dBuf, const uint32_t oLen, const uint32_t IsCapital)
{
		uint32_t		Count;
		uint8_t			Add;

		if (IsCapital)
				Add		= 0;
		else
				Add		= 0x20;
		
		for (Count = 0; Count < oLen; Count ++)
		{
				if (((*oBuf >> 4) & 0xF) > 9)
						*dBuf		= Add + 0x41 + ((*oBuf >> 4) & 0xF) - 10;
				else
						*dBuf		= 0x30 + ((*oBuf >> 4) & 0xF);

				dBuf			++;

				if ((*oBuf & 0xF) > 9)
						*dBuf		= Add + 0x41 + (*oBuf & 0xF) - 10;
				else
						*dBuf		= 0x30 + (*oBuf & 0xF);

				oBuf			++;
				dBuf			++;
		}
}
/*
 ************************************************************************************************************************************************************************     
 *函数名称:	AsciiToHex
 *功能描述:	ASCII转换为二进制
 *输入描述:	原始数据，转换缓冲区，原始数据长度
 *输出描述:	
 *返回描述:	
 *作者日期:	
 *全局声明: 
 *特殊说明: 转换缓冲区长度必须大于等于原始数据长度的1/2倍，将2个ASCII字符组成一个字节放入缓冲区，剩余单数ASCII字符放入低4位，高4位填充0
 ************************************************************************************************************************************************************************       
 */
extern void AsciiToHex(const uint8_t *oBuf, uint8_t *dBuf, const uint32_t oLen)
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

static uint32_t GetDevAddr(void)
{
		return 0x01;
}

extern uint8_t GetCID2(void)
{
    return gu8Prot_CID2;
}

/*
DATA_FLAG:
bit4: 1-有未读取的开关量变化
bit0: 1-有未读取的告警量变化
*/
extern void SetDataFlag(uint8_t IsSwitchChange,uint8_t IsAlarmChange)
{   
    volatile uint8_t u8Temp=0;
    
    if(IsSwitchChange)
       u8Temp |= 0x10;

    if(IsAlarmChange)
       u8Temp |= 0x01;

    gu8DataFlag = u8Temp;
}

extern uint8_t GetDataFlag(void)
{
    return gu8DataFlag;
}
/*
 ************************************************************************************************************************************************************************     
 *函数名称:	ProtocolProc_YDT1363_3_Make
 *功能描述:	YDT1363电总协议组包
 *输入描述:	转换方向，电总协议结构体，协议包缓冲区，缓冲区长度(最大长度)
 *输出描述:	电总协议结构体，协议包缓冲区，缓冲区大小
 *返回描述:	成功(YDT1363_FUNC_RET_SUCCESS)/失败(YDT1363_FUNC_RET_FALSE)
 *作者日期:	
 *全局声明: 
 *特殊说明: 
 ************************************************************************************************************************************************************************       
 */
extern int32_t ProtocolProc_YDT1363_3_Make(PPROTOCAL_YDT1363_3 pGeneralData,uint8_t *pBuf,uint32_t *pLen)
{
		uint8_t		LenChecksum, ConvertBuf[2*YDT1363_3_DATABUF_MAX];
		uint16_t	DataChecksum=0;
		uint32_t	Len,Count;

		if (pBuf == NULL)
				return	YDT1363_FUNC_RET_FALSE;

		if (pGeneralData == NULL)
				return	YDT1363_FUNC_RET_FALSE;

		if (pGeneralData->Length > YDT1363_3_DATABUF_MAX)
				return	YDT1363_FUNC_RET_FALSE;
		
		Len	= pGeneralData->Length * 2;

		if (Len > 0xFFF)
				return	YDT1363_FUNC_RET_FALSE;

		if ((Len + 18) > *pLen)			
				return	YDT1363_FUNC_RET_FALSE;
		
		LenChecksum		= (Len & 0xF) + ((Len >> 4) & 0xF) + ((Len >> 8) & 0xF);	
		LenChecksum		= LenChecksum % 16;
		LenChecksum		= (~LenChecksum + 1) & 0xF;
		Len						|= (LenChecksum << 12);

		ConvertBuf[0]	= pGeneralData->Ver;
		ConvertBuf[1]	= pGeneralData->Addr;
		ConvertBuf[2]	= pGeneralData->CID1;
		ConvertBuf[3]	= pGeneralData->CID2;
		GetWord(ConvertBuf + 4)	= SWAP_WORD(Len);
		memcpy(&ConvertBuf[6], pGeneralData->DataBuf, pGeneralData->Length);

		pBuf[0]				= 0x7E;
		HexToAscii(ConvertBuf, &pBuf[1], (6 + pGeneralData->Length), TRUE);
		
		DataChecksum	= 0;
		for (Count = 0; Count < (uint32_t)(12 + pGeneralData->Length * 2); Count ++)
			DataChecksum	+= pBuf[1 + Count];

		DataChecksum	= ~DataChecksum + 1;
		GetWord(ConvertBuf)		= SWAP_WORD(DataChecksum);
		HexToAscii(ConvertBuf, &pBuf[13 + pGeneralData->Length * 2], 2, TRUE);
		
		pBuf[17 + 2 * pGeneralData->Length]		= 0x0D;
		*pLen			= 18 + pGeneralData->Length * 2;

		return	YDT1363_FUNC_RET_SUCCESS;
}

extern int32_t ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(const uint8_t Rtn,uint8_t *pBuf,uint32_t *pLen)
{
    PROTOCAL_YDT1363_3 Prot;
	
		if((pBuf == NULL)||(pLen == NULL))
				return YDT1363_FUNC_RET_FALSE;

		//memset(&Prot,0,sizeof(PROTOCAL_YDT1363_3));
		Prot.Addr 	= (uint8_t)GetDevAddr();
		Prot.Ver 		= YDT1363_3_PROTOCAL_VER;
		Prot.CID1 	= YDT1363_3_PROTOCAL_CID1_DCMETER;
		Prot.CID2 	= Rtn;
		Prot.Length = 0;
		
		return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}

static int32_t ProtocolProc_YDT1363_3_Make_Vender(uint8_t *pBuf,uint32_t *pLen)
{
		PROTOCAL_YDT1363_3 Prot;
		
		if((pBuf == NULL)||(pLen == NULL))
				return YDT1363_FUNC_RET_FALSE;
		/*
		//memset(&Prot,0,sizeof(PROTOCAL_YDT1363_3));
		Prot.Addr 	= (uint8_t)GetDevAddr();
		Prot.Ver 		= YDT1363_3_PROTOCAL_VER;
		Prot.CID1 	= YDT1363_3_PROTOCAL_CID1_DCMETER;
		Prot.CID2 	= YDT1363_3_PROTOCAL_RTN_NORMAL;
		Prot.Length = 30+20+20;

		Len = Prot.Length;
		LenChecksum		= (Len & 0xF) + ((Len >> 4) & 0xF) + ((Len >> 8) & 0xF);	
		LenChecksum		= LenChecksum % 16;
		LenChecksum		= (~LenChecksum + 1) & 0xF;
		Len						|= (LenChecksum << 12);
		
		
		ConvertBuf[0]	= Prot.Ver;
		ConvertBuf[1]	= Prot.Addr;
		ConvertBuf[2]	= Prot.CID1;
		ConvertBuf[3]	= Prot.CID2;
		GetWord(ConvertBuf + 4)	= SWAP_WORD(Len);

		pBuf[0]				= 0x7E;
		HexToAscii(ConvertBuf, &pBuf[1], 6, TRUE);
		memset(ConvertBuf,0x20,Prot.Length);
		strncpy((void *)ConvertBuf,UNIT_TYPE,strlen(UNIT_TYPE));
		strncpy((void *)&ConvertBuf[30],SOFTWARE_VERSION,strlen(SOFTWARE_VERSION));
		strncpy((void *)&ConvertBuf[50],VENDER,strlen(VENDER));
		memcpy(&pBuf[13],ConvertBuf,Prot.Length);
		
		DataChecksum	= 0;
		for (Count = 0; Count < (uint32_t)(12 + Prot.Length); Count ++)
				DataChecksum	+= pBuf[1 + Count];

		DataChecksum			= ~DataChecksum + 1;
		GetWord(ConvertBuf)		= SWAP_WORD(DataChecksum);
		HexToAscii(ConvertBuf, &pBuf[13 + Prot.Length], 2, TRUE);
		
		pBuf[17 + Prot.Length]		= 0x0D;
		*pLen	= 18 + Prot.Length;
		
		return YDT1363_FUNC_RET_SUCCESS;*/
        memset(&Prot,0x20,sizeof(PROTOCAL_YDT1363_3));
        Prot.Addr 	= (uint8_t)GetDevAddr();
		Prot.Ver 	= YDT1363_3_PROTOCAL_VER;
		Prot.CID1 	= YDT1363_3_PROTOCAL_CID1_DCMETER;
		Prot.CID2 	= YDT1363_3_PROTOCAL_RTN_NORMAL;
        Prot.Length = 0;
        strncpy((void *)&Prot.DataBuf[Prot.Length],UNIT_TYPE,strlen(UNIT_TYPE));
		Prot.Length += 30;
        strncpy((void *)&Prot.DataBuf[Prot.Length],SOFTWARE_VERSION,strlen(SOFTWARE_VERSION));
		Prot.Length += 20;
		strncpy((void *)&Prot.DataBuf[Prot.Length],VENDER,strlen(VENDER));
		Prot.Length += 20;
        
		return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}



static int32_t ProtocolProc_YDT1363_3_Make_Anolog(const PYDT1363_ANALOG pData,uint8_t *pBuf,uint32_t *pLen)
{
		uint8_t		Offset;
		uint16_t	u16Temp;
		int16_t 	s16Temp;
		PROTOCAL_YDT1363_3 Prot;
		
		if((pData == NULL)||(pBuf == NULL)||(pLen == NULL))
				return YDT1363_FUNC_RET_FALSE;
		
        memset(&Prot,0x20,sizeof(PROTOCAL_YDT1363_3));
        Prot.Addr 	= (uint8_t)GetDevAddr();
		Prot.Ver 	= YDT1363_3_PROTOCAL_VER;
		Prot.CID1 	= YDT1363_3_PROTOCAL_CID1_DCMETER;
		Prot.CID2 	= YDT1363_3_PROTOCAL_RTN_NORMAL;
        Prot.Length = 0;
        
        Offset = 0;
        GetWord(&Prot.DataBuf[Offset]) = SWAP_WORD((uint16_t)(pData->VoltagePhaseA*YDT1363_3_DATA_RATIO_VOLTAGE)); 
		Offset += 2;
        //若B相电压小于10V，则未检测，显示20H
        u16Temp = pData->VoltagePhaseB*YDT1363_3_DATA_RATIO_VOLTAGE;
        GetWord(&Prot.DataBuf[Offset]) = ((u16Temp<(10*YDT1363_3_DATA_RATIO_VOLTAGE))?SWAP_WORD(0x2020):SWAP_WORD(u16Temp));
        Offset += 2;
        //若C相电压小于10V，则未检测，显示20H
        u16Temp = pData->VoltagePhaseC*YDT1363_3_DATA_RATIO_VOLTAGE;
        GetWord(&Prot.DataBuf[Offset]) = ((u16Temp<(10*YDT1363_3_DATA_RATIO_VOLTAGE))?SWAP_WORD(0x2020):SWAP_WORD(u16Temp));
        Offset += 2;
        GetWord(&Prot.DataBuf[Offset]) = SWAP_WORD((uint16_t)(pData->CurrentPhaseA*YDT1363_3_DATA_RATIO_CURRENT)); 
		Offset += 2;
        //若B相电流小于0.01A，则未检测，显示20H
        u16Temp = pData->CurrentPhaseB*YDT1363_3_DATA_RATIO_CURRENT;
        GetWord(&Prot.DataBuf[Offset]) = ((u16Temp<1)?SWAP_WORD(0x2020):SWAP_WORD(u16Temp));
        Offset += 2;
        //若C相电流小于0.01A，则未检测，显示20H
        u16Temp = pData->CurrentPhaseC*YDT1363_3_DATA_RATIO_CURRENT;
        GetWord(&Prot.DataBuf[Offset]) = ((u16Temp<1)?SWAP_WORD(0x2020):SWAP_WORD(u16Temp));
        Offset += 2;
        
        s16Temp = (int16_t)(pData->SupplyTemperature*YDT1363_3_DATA_RATIO_TEMPERATUE);
        GetWord(&Prot.DataBuf[Offset]) = ((s16Temp<(-20))?SWAP_WORD(0x2020):SWAP_WORD(s16Temp));
        Offset += 2;
        s16Temp = (int16_t)(pData->ReturnTemperature*YDT1363_3_DATA_RATIO_TEMPERATUE);
        GetWord(&Prot.DataBuf[Offset]) = ((s16Temp<(-20))?SWAP_WORD(0x2020):SWAP_WORD(s16Temp));
        Offset += 2;
        Prot.Length = Offset;
        return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}
static int32_t ProtocolProc_YDT1363_3_Make_Anolog_EX(const PYDT1363_ANALOG pData,uint8_t *pBuf,uint32_t *pLen)
{
		uint8_t		Offset;
		uint16_t	u16Temp;
		int16_t 	s16Temp;
		PROTOCAL_YDT1363_3 Prot;
		
		if((pData == NULL)||(pBuf == NULL)||(pLen == NULL))
				return YDT1363_FUNC_RET_FALSE;
		
        memset(&Prot,0x20,sizeof(PROTOCAL_YDT1363_3));
        Prot.Addr 	= (uint8_t)GetDevAddr();
		Prot.Ver 	= YDT1363_3_PROTOCAL_VER;
		Prot.CID1 	= YDT1363_3_PROTOCAL_CID1_DCMETER;
		Prot.CID2 	= YDT1363_3_PROTOCAL_RTN_NORMAL;
        Prot.Length = 0;
        
        Offset = 0;
        GetWord(&Prot.DataBuf[Offset]) = SWAP_WORD((uint16_t)(pData->VoltagePhaseA*YDT1363_3_DATA_RATIO_VOLTAGE)); 
		Offset += 2;
        //若B相电压小于10V，则未检测，显示20H
        u16Temp = pData->VoltagePhaseB*YDT1363_3_DATA_RATIO_VOLTAGE;
        GetWord(&Prot.DataBuf[Offset]) = ((u16Temp<(10*YDT1363_3_DATA_RATIO_VOLTAGE))?SWAP_WORD(0x2020):SWAP_WORD(u16Temp));
        Offset += 2;
        //若C相电压小于10V，则未检测，显示20H
        u16Temp = pData->VoltagePhaseC*YDT1363_3_DATA_RATIO_VOLTAGE;
        GetWord(&Prot.DataBuf[Offset]) = ((u16Temp<(10*YDT1363_3_DATA_RATIO_VOLTAGE))?SWAP_WORD(0x2020):SWAP_WORD(u16Temp));
        Offset += 2;
        GetWord(&Prot.DataBuf[Offset]) = SWAP_WORD((uint16_t)(pData->CurrentPhaseA*YDT1363_3_DATA_RATIO_CURRENT)); 
		Offset += 2;
        //若B相电流小于0.01A，则未检测，显示20H
        u16Temp = pData->CurrentPhaseB*YDT1363_3_DATA_RATIO_CURRENT;
        GetWord(&Prot.DataBuf[Offset]) = ((u16Temp<1)?SWAP_WORD(0x2020):SWAP_WORD(u16Temp));
        Offset += 2;
        //若C相电流小于0.01A，则未检测，显示20H
        u16Temp = pData->CurrentPhaseC*YDT1363_3_DATA_RATIO_CURRENT;
        GetWord(&Prot.DataBuf[Offset]) = ((u16Temp<1)?SWAP_WORD(0x2020):SWAP_WORD(u16Temp));
        Offset += 2;
        
        s16Temp = (int16_t)(pData->SupplyTemperature*YDT1363_3_DATA_RATIO_TEMPERATUE_EX);
        GetWord(&Prot.DataBuf[Offset]) = ((s16Temp<(-20*YDT1363_3_DATA_RATIO_TEMPERATUE_EX))?SWAP_WORD(0x2020):SWAP_WORD(s16Temp));
        Offset += 2;
        s16Temp = (int16_t)(pData->ReturnTemperature*YDT1363_3_DATA_RATIO_TEMPERATUE_EX);
        GetWord(&Prot.DataBuf[Offset]) = ((s16Temp<(-20*YDT1363_3_DATA_RATIO_TEMPERATUE_EX))?SWAP_WORD(0x2020):SWAP_WORD(s16Temp));
        Offset += 2;
        Prot.Length = Offset;
        return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}

static int32_t ProtocolProc_YDT1363_3_Make_Switch(const PYDT1363_SWITCH pData,uint8_t *pBuf,uint32_t *pLen)
{
		uint8_t Offset;
		PROTOCAL_YDT1363_3 Prot;
			
		if((pData == NULL)||(pBuf == NULL)||(pLen == NULL))
				return YDT1363_FUNC_RET_FALSE;

		//memset(&Prot,0,sizeof(PROTOCAL_YDT1363_3));
		Prot.Addr = (uint8_t)GetDevAddr();
		Prot.Ver 	= YDT1363_3_PROTOCAL_VER;
		Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
		Prot.CID2 = YDT1363_3_PROTOCAL_RTN_NORMAL;
	
		pData->PowerOnOff 		= (SWITCH_DEFINE)AirCondGetSwitchOnOffStatus();
    Offset 		= 0;
    Prot.DataBuf[Offset] 	= GetDataFlag();
    Offset 		+= 1;
    Prot.DataBuf[Offset] 	= pData->PowerOnOff; 
    Offset 		+= 1;
    Prot.DataBuf[Offset] 	= 0; 
    Offset 		+= 1;
    
    Prot.Length 					= Offset;

		return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}


static int32_t ProtocolProc_YDT1363_3_Make_Alarm(const PYDT1363_ALARM pData,uint8_t *pBuf,uint32_t *pLen)
{
		uint8_t Offset;
		PROTOCAL_YDT1363_3 Prot;
			
		if((pData == NULL)||(pBuf == NULL)||(pLen == NULL))
				return YDT1363_FUNC_RET_FALSE;

		Prot.Addr = (uint8_t)GetDevAddr();
		Prot.Ver 	= YDT1363_3_PROTOCAL_VER;
		Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
		Prot.CID2 = YDT1363_3_PROTOCAL_RTN_NORMAL;

		Offset	 	= 0;
		Prot.DataBuf[Offset] 		= GetDataFlag();
		Offset 		+= 1;
		pData->ColdFailure 			= (ALARM_DEFINE)AirCondGetCoolAlarmStatus();
		pData->HeatFailure 			= (ALARM_DEFINE)AirCondGetHotAlarmStatus();
		pData->ReturnTempSensor = (ALARM_DEFINE)AirCondGetReturnTempAlarmStatus();
		pData->SupplyTempSensor = (ALARM_DEFINE)AirCondGetOutletTempAlarmStatus();
		pData->Power						= (ALARM_DEFINE)AirCondGetPowerAlarmStatus();
		Prot.DataBuf[Offset] 		= pData->ColdFailure; 
		Offset 		+= 1;
		Prot.DataBuf[Offset] 		= pData->HeatFailure; 
		Offset 		+= 1;
		Prot.DataBuf[Offset] 		= pData->Power; 
		Offset 		+= 1;
		Prot.DataBuf[Offset] 		= pData->CurrSensor; 
		Offset 		+= 1;
		Prot.DataBuf[Offset] 		= pData->ReturnTempSensor; 
		Offset 		+= 1;
		Prot.DataBuf[Offset] 		= pData->SupplyTempSensor; 
		Offset 		+= 1;
		Prot.Length 						= Offset;
	
		return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}

static int32_t ProtocolProc_YDT1363_3_Make_Param(const PYDT1363_PARAM pData,uint8_t *pBuf,uint32_t *pLen)
{
	uint8_t Offset;
	PROTOCAL_YDT1363_3 Prot;
		
	if((pData == NULL)||(pBuf == NULL)||(pLen == NULL))
			return YDT1363_FUNC_RET_FALSE;

	//memset(&Prot,0,sizeof(PROTOCAL_YDT1363_3));
	Prot.Addr = (uint8_t)GetDevAddr();
	Prot.Ver 	= YDT1363_3_PROTOCAL_VER;
	Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
	Prot.CID2 = YDT1363_3_PROTOCAL_RTN_NORMAL;

	Offset 		= 0;
    
	//pData->SetTemp 	= AirCondGetSettingTempStatus();
	//pData->RunMode 	= (MODE_DEFINE)AirCondGetRunModeStatus();
	//pData->WindMode = (WIND_MODE_DEFINE)AirCondGetWindSpeedStatus();
	
	GetWord(&Prot.DataBuf[Offset]) = SWAP_SHORT((int16_t)(pData->PowerOnTemp*YDT1363_3_DATA_RATIO_TEMPERATUE)); 
	Offset 					+= 2;
	GetWord(&Prot.DataBuf[Offset]) = SWAP_SHORT((int16_t)(pData->PowerOffTemp*YDT1363_3_DATA_RATIO_TEMPERATUE)); 
	Offset 					+= 2;
	GetWord(&Prot.DataBuf[Offset]) = SWAP_SHORT((int16_t)(pData->ReturnTempUpper*YDT1363_3_DATA_RATIO_TEMPERATUE)); 
	Offset 					+= 2;
    GetWord(&Prot.DataBuf[Offset]) = SWAP_SHORT((int16_t)(pData->ReturnTempLower*YDT1363_3_DATA_RATIO_TEMPERATUE)); 
	Offset 					+= 2;
    GetWord(&Prot.DataBuf[Offset]) = SWAP_SHORT((int16_t)(pData->SetTemp*YDT1363_3_DATA_RATIO_TEMPERATUE)); 
	Offset 					+= 2;
    GetWord(&Prot.DataBuf[Offset]) = SWAP_SHORT((uint16_t)(pData->RunMode)); 
	Offset 					+= 2;
    GetWord(&Prot.DataBuf[Offset]) = SWAP_SHORT((uint16_t)(pData->WindMode)); 
	Offset 					+= 2;
	Prot.Length 	= Offset;
	
	return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}

static int32_t ProtocolProc_YDT1363_3_Set_Param(PYDT1363_PARAM pData,uint8_t *pDataBuf,uint8_t *pBuf,uint32_t *pLen)
{
    uint8_t Rtn=0;
    uint8_t CommandType=pDataBuf[0];
    int16_t DataI=SWAP_SHORT(GetWord(&pDataBuf[1]));
    int			tmpVal=0,i;
	
	if((pData == NULL))
		return YDT1363_FUNC_RET_FALSE;
    
    Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
    switch(CommandType)
    {
        case 0x80:
            pData->PowerOnTemp = (float)DataI;
			if(DataI<=16)
					AirCondCtrlOperation(14);
			else if(DataI<=20)
					AirCondCtrlOperation(2);
			else if(DataI==21)
					AirCondCtrlOperation(3);
			else if(DataI==22)
					AirCondCtrlOperation(4);
			else if(DataI==23)
					AirCondCtrlOperation(5);
			else if(DataI==24)
					AirCondCtrlOperation(6);
			else if(DataI==25)
					AirCondCtrlOperation(7);
			else if(DataI==26)
					AirCondCtrlOperation(8);
			else if(DataI==27)
					AirCondCtrlOperation(9);
			else if(DataI==28)
					AirCondCtrlOperation(10);
			else if(DataI==29)
					AirCondCtrlOperation(11);
			else if(DataI>=30)
					AirCondCtrlOperation(12);
            break;
        case 0x81:
            pData->PowerOffTemp = (float)DataI;
			AirCondCtrlOperation(1);
            break;
        case 0x86:
            pData->SetTemp = (float)DataI;
			if(DataI<=16)
					AirCondCtrlOperation(14);
			else if(DataI<=20)
					AirCondCtrlOperation(2);
			else if(DataI==21)
					AirCondCtrlOperation(3);
			else if(DataI==22)
					AirCondCtrlOperation(4);
			else if(DataI==23)
					AirCondCtrlOperation(5);
			else if(DataI==24)
					AirCondCtrlOperation(6);
			else if(DataI==25)
					AirCondCtrlOperation(7);
			else if(DataI==26)
					AirCondCtrlOperation(8);
			else if(DataI==27)
					AirCondCtrlOperation(9);
			else if(DataI==28)
					AirCondCtrlOperation(10);
			else if(DataI==29)
					AirCondCtrlOperation(11);
			else if(DataI>=30)
					AirCondCtrlOperation(12);
            break;
        case 0xc0:
            if((DataI>=0)&&(DataI<=4))
            {
				pData->RunMode = (uint32_t)DataI;
				tmpVal = pData->SetTemp;
				if(DataI == 0x2 || DataI == 0x1)	//除湿制冷
				{
						if(tmpVal<=16)
								AirCondCtrlOperation(14);
						else if(tmpVal<=20)
								AirCondCtrlOperation(2);
						else if(tmpVal==21)
								AirCondCtrlOperation(3);
						else if(tmpVal==22)
								AirCondCtrlOperation(4);
						else if(tmpVal==23)
								AirCondCtrlOperation(5);
						else if(tmpVal==24)
								AirCondCtrlOperation(6);
						else if(tmpVal==25)
								AirCondCtrlOperation(7);
						else if(tmpVal==26)
								AirCondCtrlOperation(8);
						else if(tmpVal==27)
								AirCondCtrlOperation(9);
						else if(tmpVal==28)
								AirCondCtrlOperation(10);
						else if(tmpVal==29)
								AirCondCtrlOperation(11);
						else if(tmpVal>=30)
								AirCondCtrlOperation(12);
				}
				else if(DataI == 0x3)
						AirCondCtrlOperation(13);
				else if(DataI == 0x4)
						AirCondCtrlOperation(14);
						}
            else
                Rtn = YDT1363_3_PROTOCAL_RTN_DATA_INVALID;
            break;
        case 0xc1:
            if((DataI>=1)&&(DataI<=3))
                pData->WindMode = (uint32_t)DataI;
            else
                Rtn = YDT1363_3_PROTOCAL_RTN_DATA_INVALID;
            break;
        default:
            Rtn = YDT1363_3_PROTOCAL_RTN_DATA_INVALID;
            break;
    }
		
		i=0;
    do
	{
        i++;
		WatchDogFeed();
        Rtn = MeterData_WaitWriteSucc(1,(uint8_t *)pData,FLASH_ADDR_PARAM,FLASH_ADDR_PARAM_SIZE);
    }while((i<3)&&(Rtn!=0));
		
    return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pBuf,pLen);
}

static int32_t ProtocolProc_YDT1363_3_Set_ParamInfrared(PYDT1363_PARAM_INFRARED pData,uint8_t *pDataBuf,uint8_t *pBuf,uint32_t *pLen)
{
    uint8_t Rtn=0,InvalidData=0,i;
    uint16_t DataI1,DataI2,DataI3;
    
		if((pData == NULL))
				return YDT1363_FUNC_RET_FALSE;
    
    DataI1 = SWAP_WORD(GetWord(&pDataBuf[0]));
    if((DataI1<30000)||(DataI1>50000))
		{
        InvalidData = 1;
    }
    pData->TransmitFreq = (float)DataI1;
    
    DataI2 = SWAP_WORD(GetWord(&pDataBuf[2]));
    if(DataI2>2)
		{
        InvalidData = 1;
    }
    pData->CommandAction = DataI2;
    DataI3 = SWAP_WORD(GetWord(&pDataBuf[4]));
    if(DataI3>32)
		{
        InvalidData = 1;
    }
    pData->CommandNum = DataI3;
    if(InvalidData==1)
		{
        Rtn = YDT1363_3_PROTOCAL_RTN_DATA_INVALID;
        return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pBuf,pLen);
    }
    
    if(DataI2==0)
	{
        //无动作
    }
	else if(DataI2==1)
	{
    //学习红外命令
			AirCondLearnOperation(DataI3);
			Rtn = YDT1363_3_PROTOCAL_RTN_LEARNING_SUCC;
			*pLen = 0;
    }
	else
	{
    //发射红外命令
			AirCondCtrlOperation(DataI3);
			Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
    }

	i=0;
    do
	{
        i++;
		WatchDogFeed();
        Rtn = MeterData_WaitWriteSucc(1,(uint8_t *)pData,FLASH_ADDR_PARAM_INFRARED,FLASH_ADDR_PARAM_INFRARED_SIZE);

    }while((i<3)&&(Rtn!=0));
	
    return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pBuf,pLen);
}
static int32_t ProtocolProc_YDT1363_3_Make_Energy(const PYDT1363_ENERGY pData,uint8_t *pBuf,uint32_t *pLen)
{
		uint8_t 						Offset;
		PROTOCAL_YDT1363_3 	Prot;
			
		if((pData == NULL)||(pBuf == NULL)||(pLen == NULL))
				return YDT1363_FUNC_RET_FALSE;

		Prot.Addr = (uint8_t)GetDevAddr();
		Prot.Ver 	= YDT1363_3_PROTOCAL_VER;
		Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
		Prot.CID2 = YDT1363_3_PROTOCAL_RTN_NORMAL;

		Offset 		= 0;
		GetDWord(&Prot.DataBuf[Offset]) = SWAP_DWORD((uint32_t)(pData->LastDayEgy*YDT1363_3_DATA_RATIO_ENERGY)); 
		Offset 		+= 4;
		GetDWord(&Prot.DataBuf[Offset]) = SWAP_DWORD((uint32_t)(pData->TodayEgy*YDT1363_3_DATA_RATIO_ENERGY)); 
		Offset 		+= 4;
		Prot.Length = Offset;
		
		return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}

static int32_t ProtocolProc_YDT1363_3_Make_Date(const PREAL_DATE pData,uint8_t *pBuf,uint32_t *pLen)
{
		uint8_t 						Offset;
		PROTOCAL_YDT1363_3 	Prot;
		
		if((pData == NULL)||(pBuf == NULL)||(pLen == NULL))
				return YDT1363_FUNC_RET_FALSE;

		//memset(&Prot,0,sizeof(PROTOCAL_YDT1363_3));
		Prot.Addr = (uint8_t)GetDevAddr();
		Prot.Ver 	= YDT1363_3_PROTOCAL_VER;
		Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
		Prot.CID2 = YDT1363_3_PROTOCAL_RTN_NORMAL;
	
		Offset 		= 0;
		GetWord(&Prot.DataBuf[Offset]) = SWAP_WORD((uint16_t)pData->Year); 
		Offset 		+= 2;
		Prot.DataBuf[Offset] = pData->Month; 
		Offset 		+= 1;
		Prot.DataBuf[Offset] = pData->Day; 
		Offset 		+= 1;
		Prot.DataBuf[Offset] = pData->Hour; 
		Offset 		+= 1;
		Prot.DataBuf[Offset] = pData->Minute; 
		Offset 		+= 1;
		Prot.DataBuf[Offset] = pData->Second; 
		Offset 		+= 1;
		Prot.Length = Offset;
	
		return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}

//pData为NULL时，仅检查日期时间的有效性
static int32_t ProtocolProc_YDT1363_3_SetDate(const PREAL_DATE pSetData,PREAL_DATE pData)
{
		if((pSetData == NULL))
				return YDT1363_FUNC_RET_FALSE;
		
		if(!DATE_YEAR_IS_VALID(pSetData->Year))
				return YDT1363_FUNC_RET_FALSE;
		if(!DATE_MONTH_IS_VALID(pSetData->Month))
				return YDT1363_FUNC_RET_FALSE;
		if(!DATE_DAY_IS_VALID(pSetData->Day))
				return YDT1363_FUNC_RET_FALSE;
		if(!DATE_HOUR_IS_VALID(pSetData->Hour))
				return YDT1363_FUNC_RET_FALSE;
		if(!DATE_MINUTE_OR_SECOND_IS_VALID(pSetData->Minute))
				return YDT1363_FUNC_RET_FALSE;
		if(!DATE_MINUTE_OR_SECOND_IS_VALID(pSetData->Second))
				return YDT1363_FUNC_RET_FALSE;

		if(pData != NULL)
				memcpy(pData,pSetData,sizeof(REAL_DATE));
		
		return YDT1363_FUNC_RET_SUCCESS;
}

#define YDT1363_3_RET_HISTORY_END 						0x01
#define YDT1363_3_RET_HISTORY_NORMAL 					0x00

static int32_t ProtocolProc_YDT1363_3_Make_HistoryDayData(uint8_t Command,PYDT1363_DATA pData,uint8_t *pBuf,uint32_t *pLen)
{
    static uint32_t 		Count;
		PROTOCAL_YDT1363_3 	Prot;
    
		if((pData == NULL)||(pBuf == NULL)||(pLen == NULL))
				return YDT1363_FUNC_RET_FALSE;

    Prot.Addr = (uint8_t)GetDevAddr();
		Prot.Ver 	= YDT1363_3_PROTOCAL_VER;
		Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
		Prot.CID2 = YDT1363_3_PROTOCAL_RTN_NORMAL;
    
    if(Command==0x10)
		{
        Count = 0;
        if(pData->SaveCnt.DayEgyCnt==1)
            Prot.DataBuf[0] = YDT1363_3_RET_HISTORY_END;
        else
            Prot.DataBuf[0] = YDT1363_3_RET_HISTORY_NORMAL;
        HistoryDayData_GetRecord(pData->SaveCnt.DayEgyCnt,(PHistoryDayData)&Prot.DataBuf[1]);
    }
		else
		{
        (Command==0x11)?++Count:Count;
        if(Count>(SAVE_HISTORY_DATA_CNT_MAX-1))
				{
            Count = 0;
            return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(YDT1363_3_PROTOCAL_RTN_DATA_NONE,pBuf,pLen);
        }
				else if(Count==(pData->SaveCnt.DayEgyCnt-1))
				{
            Prot.DataBuf[0] = YDT1363_3_RET_HISTORY_END;
            HistoryDayData_GetRecord(pData->SaveCnt.DayEgyCnt-Count,(PHistoryDayData)&Prot.DataBuf[1]); 
        }
				else
				{
            Prot.DataBuf[0] = YDT1363_3_RET_HISTORY_NORMAL;
            HistoryDayData_GetRecord(pData->SaveCnt.DayEgyCnt-Count,(PHistoryDayData)&Prot.DataBuf[1]);
        }          
    }
    Prot.Length = 1+SAVE_HISTORY_DATA_BYTES;
	
		return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}

static int32_t ProtocolProc_YDT1363_3_Make_TEMP_RATIO(const PTEMP_RATIO pData,uint8_t *pBuf,uint32_t *pLen)
{
		uint8_t 						Offset;
    int32_t 						s32Temp;
		PROTOCAL_YDT1363_3 	Prot;
		
		if((pData == NULL)||(pBuf == NULL)||(pLen == NULL))
				return YDT1363_FUNC_RET_FALSE;

		Prot.Addr = (uint8_t)GetDevAddr();
		Prot.Ver 	= YDT1363_3_PROTOCAL_VER;
		Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
		Prot.CID2 = YDT1363_3_PROTOCAL_RTN_NORMAL;
	
		Offset 		= 0;
    s32Temp 	= pData->k*100000000;
		GetDWord(&Prot.DataBuf[Offset]) = SWAP_LONG(s32Temp); 
		Offset 		+= 4;
		s32Temp 	= pData->b*100000000;
		GetDWord(&Prot.DataBuf[Offset]) = SWAP_LONG(s32Temp); 
		Offset 		+= 4;
    s32Temp 	= pData->R0*100000000;
		GetDWord(&Prot.DataBuf[Offset]) = SWAP_LONG(s32Temp); 
		Offset 		+= 4;
		Prot.Length = Offset;
	
		return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}
static int32_t ProtocolProc_YDT1363_3_Set_TEMP_RATIO(uint8_t *pDataBuf,PTEMP_RATIO pData)
{
		uint8_t Offset;
    int32_t s32Temp;
		
		if((pData == NULL)||(pDataBuf == NULL))
				return YDT1363_FUNC_RET_FALSE;

		Offset 		= 0;
		s32Temp 	= SWAP_LONG(GetDWord(&pDataBuf[Offset])); 
		pData->k 	= (float)s32Temp/100000000.0;
		Offset 		+= 4;
		s32Temp 	= SWAP_LONG(GetDWord(&pDataBuf[Offset])); 
    pData->b 	= (float)s32Temp/100000000.0;
		Offset 		+= 4;
		s32Temp 	= SWAP_LONG(GetDWord(&pDataBuf[Offset])); 
		pData->R0 = (float)s32Temp/100000000.0;
		Offset 		+= 4;
	
		return YDT1363_FUNC_RET_SUCCESS;
}
static int32_t ProtocolProc_YDT1363_3_Make_RN8209C_INITPARAM(uint8_t Rn8209Num,const PRN8209C pData,uint8_t *pBuf,uint32_t *pLen)
{
		uint8_t 						Offset;
		PROTOCAL_YDT1363_3 	Prot;
		PRN8209C_DEF 				pRn8209 = NULL;
    
		if((pData == NULL)||(pBuf == NULL)||(pLen == NULL))
				return YDT1363_FUNC_RET_FALSE;
		if((Rn8209Num<1)||(Rn8209Num>3))
				return YDT1363_FUNC_RET_FALSE;
        
		Prot.Addr = (uint8_t)GetDevAddr();
		Prot.Ver 	= YDT1363_3_PROTOCAL_VER;
		Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
		Prot.CID2 = YDT1363_3_PROTOCAL_RTN_NORMAL;
        
		pRn8209 	= &pData->Def[Rn8209Num-1];
		
		Offset 		= 0;
		Prot.DataBuf[Offset] = Rn8209Num;
		Offset 		+= 1;
		memcpy((void *)&Prot.DataBuf[Offset],(void *)pRn8209,sizeof(RN8209C_DEF));
		Offset 		+= sizeof(RN8209C_DEF);

		Prot.Length = Offset;
	
		return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}
static int32_t ProtocolProc_YDT1363_3_Set_RN8209C_INITPARAM(uint8_t *pDataBuf,PRN8209C pData)
{
		uint8_t     	Count,Rn8209Num;
    uint16_t    	CheckSum;
		PRN8209C_DEF 	pRn8209 = NULL;
    
		if((pData == NULL)||(pDataBuf == NULL))
				return YDT1363_FUNC_RET_FALSE;
        
		Rn8209Num = pDataBuf[0];
		if((Rn8209Num<1)||(Rn8209Num>3))
				return YDT1363_FUNC_RET_FALSE;
		
		pRn8209 = (PRN8209C_DEF)&pDataBuf[1];

		for(Count=0,CheckSum=0;Count<24;Count++)
		{
				CheckSum += pRn8209->InitParam.UNION.PARAM_Union[Count];
		}
		CheckSum += 0x1600;
		CheckSum = ~CheckSum;
        
		if(CheckSum != pRn8209->InitParam.UNION.PARAM_List.CHKSUM)
				return YDT1363_FUNC_RET_FALSE;
		
		memcpy(&pData->Def[Rn8209Num-1],pRn8209,sizeof(RN8209C_DEF));
        
		return YDT1363_FUNC_RET_SUCCESS;
}

static int32_t ProtocolProc_YDT1363_3_Make_SoftWare(uint8_t *pBuf,uint32_t *pLen)
{
	PROTOCAL_YDT1363_3 Prot;
	
	if((pBuf == NULL)||(pLen == NULL))
		return YDT1363_FUNC_RET_FALSE;
	
    memset(&Prot,0x20,sizeof(PROTOCAL_YDT1363_3));
    Prot.Addr = (uint8_t)GetDevAddr();
	Prot.Ver = YDT1363_3_PROTOCAL_VER;
	Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
	Prot.CID2 = YDT1363_3_PROTOCAL_RTN_NORMAL;
    Prot.Length = 0;
    strncpy((void *)&Prot.DataBuf[Prot.Length],USER_DEFINE_SOFT_VERSION,strlen(USER_DEFINE_SOFT_VERSION));
    Prot.Length += strlen(USER_DEFINE_SOFT_VERSION);
    
    return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}

static int32_t ProtocolProc_YDT1363_3_Make_CtrlManufacturer(uint8_t *pBuf,uint32_t *pLen)
{
	PROTOCAL_YDT1363_3 Prot;
	uint8_t Offset;
	
	if((pBuf == NULL)||(pLen == NULL))
		return YDT1363_FUNC_RET_FALSE;
	
    memset(&Prot,0x20,sizeof(PROTOCAL_YDT1363_3));
    Prot.Addr = (uint8_t)GetDevAddr();
	Prot.Ver = YDT1363_3_PROTOCAL_VER;
	Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
	Prot.CID2 = YDT1363_3_PROTOCAL_RTN_NORMAL;
	Offset = 0;
	GetDWord(&Prot.DataBuf[Offset]) = SWAP_DWORD(InfraGetManuID()); 
	Offset += 4;
    Prot.Length = Offset;
    return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}
static int32_t ProtocolProc_YDT1363_3_Set_CtrlManufacturer(uint8_t *pDataBuf,uint8_t *pBuf,uint32_t *pLen)
{
	uint8_t Offset;
    uint32_t u32Temp,Rtn;
		
	if((pDataBuf == NULL)||(pBuf == NULL)||(pLen == NULL))
		return YDT1363_FUNC_RET_FALSE;

	Offset 		= 0;
	u32Temp 	= SWAP_DWORD(GetDWord(&pDataBuf[Offset])); 

	Rtn = YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL;
	if(FUNC_RET_TRUE==InfraSetManuID(u32Temp)){
		if(0==MeterData_WaitWriteSucc(1,(uint8_t *)&u32Temp,FLASH_ADDR_AIR_CON_MANUFACTURER,FLASH_ADDR_AIR_CON_MANUFACTURER_SIZE))
			Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
	}
	return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pBuf,pLen);
}

int FMC_Read_User(unsigned int u32Addr, unsigned int * data)
{
    unsigned int Reg;
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADR = u32Addr;
    FMC->ISPDAT = 0;
    FMC->ISPTRG = 0x1;

    __ISB();

    Reg = FMC->ISPCON;

    if(Reg & FMC_ISPCON_ISPFF_Msk) {
        FMC->ISPCON = Reg;
        return -1;
    }

    *data = FMC->ISPDAT;
    return 0;
}

uint32_t GetApromSize(void)
{
    uint32_t size = 0x4000, data;
    int result;

    do {
        result = FMC_Read_User(size, &data);
        if(result < 0) {
            return size;
        } else
            size *= 2;
    } while(1);
}

static int32_t ProtocolProc_YDT1363_3_AI_Cali_ReturnTemperature(float y,PTEMP_RATIO pData)
{
    static float x1,x2,y1,y2;
    static uint8_t SecondFlag=0;
	uint32_t u32Temp;
	float fTemp;
    
    if(y>2.5)
        return YDT1363_FUNC_RET_FALSE;
    
	WatchDogFeed();
   	UART_CS(1);
    DelayUs(10);		
    if(0!=GetSPLIBAverage(&u32Temp))
        return YDT1363_FUNC_RET_FALSE;  
    UART_CS(0);
    if(u32Temp&0x80000)
	{
        u32Temp = (~u32Temp)&0x7ffff;
        fTemp 	= u32Temp;
        fTemp 	= -fTemp; 
    }else{
        fTemp 	= u32Temp;
    } 
    if(SecondFlag==0){
        SecondFlag = 1;
        y1 = y;
        x1 = fTemp;  
		return 0x5a;
    }else{
        SecondFlag = 0;
        y2 = y;
        x2 = fTemp;
        if(pData!=NULL){
            pData->k = (y2-y1)/(x2-x1);
            pData->b = y1-x1*pData->k;
        }
		return YDT1363_FUNC_RET_SUCCESS;
    }
    
}
static int32_t ProtocolProc_YDT1363_3_AI_Cali_SupplyTemperature(float y,PTEMP_RATIO pData)
{
    static float x1,x2,y1,y2;
    static uint8_t SecondFlag=0;
	uint32_t u32Temp;
	float fTemp;
    
    if(y>2.5)
        return YDT1363_FUNC_RET_FALSE;
    
	WatchDogFeed();
   	UART_CS(2);
    DelayUs(10);		
    if(0!=GetSPLIBAverage(&u32Temp))
        return YDT1363_FUNC_RET_FALSE;  
    UART_CS(0);
    if(u32Temp&0x80000)
	{
        u32Temp = (~u32Temp)&0x7ffff;
        fTemp 	= u32Temp;
        fTemp 	= -fTemp; 
    }else{
        fTemp 	= u32Temp;
    } 
    if(SecondFlag==0){
        SecondFlag = 1;
        y1 = y;
        x1 = fTemp; 
		return 0x5a;
    }else{
        SecondFlag = 0;
        y2 = y;
        x2 = fTemp;
        if(pData!=NULL){
            pData->k = (y2-y1)/(x2-x1);
            pData->b = y1-x1*pData->k;
        }
		return YDT1363_FUNC_RET_SUCCESS;
    }
    
}
//本函数会调用传入的串口发送函数指针，分多个包将红外信号编码发送到上位机
static int32_t ProtocolProc_YDT1363_3_Make_UpLoadCmd(
				uint8_t CMDID,
				uint8_t *pBuf,
				uint32_t *pLen,uint32_t (*pUartWrite)(uint8_t *pu8TxBuf, uint32_t u32WriteBytes))
{
	PROTOCAL_YDT1363_3 Prot;
	uint8_t Offset,Step;
	SaveRecordFormatDef *pTemp;
	uint32_t i=0,SendRcdNum;
		
	if((pBuf == NULL)||(pLen == NULL)||(pUartWrite==NULL))
		return YDT1363_FUNC_RET_FALSE;
	
   
	
	if(FUNC_RET_TRUE == InfraGetRecordTemp(&pTemp)){
		if((0 == GetInfraLearningCmdFromFlash(InfraGetManuID(),CMDID,pTemp,sizeof(SaveRecordFormatDef)))
			&&(pTemp->RecordCnt>2)){
			
			for(SendRcdNum=0;SendRcdNum<((pTemp->RecordCnt+YDT1363_LOAD_RECORD_MAX-1)/YDT1363_LOAD_RECORD_MAX);SendRcdNum++){
				
				memset(&Prot,0,sizeof(PROTOCAL_YDT1363_3));
				Prot.Addr = (uint8_t)GetDevAddr();
				Prot.Ver = YDT1363_3_PROTOCAL_VER;
				Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
				Prot.CID2 = YDT1363_3_PROTOCAL_RTN_NORMAL;
				Offset = 0;
				GetWord(&Prot.DataBuf[Offset]) = SWAP_WORD(pTemp->RecordCnt); 
				Offset += 2;
				GetWord(&Prot.DataBuf[Offset]) = SWAP_WORD(SendRcdNum); 
				Offset += 2;
				Step = (((pTemp->RecordCnt-YDT1363_LOAD_RECORD_MAX*SendRcdNum)>YDT1363_LOAD_RECORD_MAX)?YDT1363_LOAD_RECORD_MAX:pTemp->RecordCnt-YDT1363_LOAD_RECORD_MAX*SendRcdNum);
				for(i=0;i<Step;i++){
					GetWord(&Prot.DataBuf[Offset]) = SWAP_WORD(pTemp->Record[SendRcdNum*YDT1363_LOAD_RECORD_MAX+i].DurationUS); 
					Offset += 2;
				}
				Prot.Length = Offset;
				if(YDT1363_FUNC_RET_SUCCESS == ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen)){
					(*pUartWrite)(pBuf,*pLen);	
				}
			
			}
		}
	}
	InfraFreeRecordTemp(&pTemp);

	return YDT1363_FUNC_RET_FALSE;
}

static void ProtocolProc_YDT1363_3_Set_DownLoadCmd_TimeOutCallBack(void *pHandle)
{
	uint16_t *pSavedRcd=(uint16_t *)pHandle;
    
	debug("ProtocolProc_YDT1363_3_Set_DownLoadCmd_TimeOutCallBack\n");
	InfraFreeRecordTemp(NULL);
	*pSavedRcd = 0;
}
static int32_t ProtocolProc_YDT1363_3_Set_DownLoadCmd(PROTOCAL_YDT1363_3 *pYdtBuf,uint8_t *pBuf,uint32_t *pLen)
{
	uint8_t Offset,CmdID,i;
	uint16_t RecordCnt,OrderNum;
    uint32_t Rtn;
	static uint8_t PinLevel;
	static uint16_t LastOrderNum,SavedRcd;	
	static SaveRecordFormatDef *pTemp=NULL;
	PROTOCAL_YDT1363_3 Prot;
	
	if((pYdtBuf == NULL)||(pBuf == NULL)||(pLen == NULL))
		return YDT1363_FUNC_RET_FALSE;

	
	if(SavedRcd==0){
		if(FUNC_RET_TRUE != InfraGetRecordTemp(&pTemp)){
			Rtn = YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL;
			return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pBuf,pLen);
		}
		
		if(SoftTimerIsStart(TIMER_DOWNLOAD_INFRA_CMD)){
			SoftTimerStop(TIMER_DOWNLOAD_INFRA_CMD);
		}
		SoftTimerStart(TIMER_DOWNLOAD_INFRA_CMD,600,1,ProtocolProc_YDT1363_3_Set_DownLoadCmd_TimeOutCallBack,&SavedRcd);//1min定时
		
	}
	Offset = 0;
	CmdID = pYdtBuf->DataBuf[Offset]-1;
	if(!CMDIDIsValid(CmdID)){
		Rtn = YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL;
		return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pBuf,pLen);
	}
	Offset += 1;
	RecordCnt = SWAP_WORD(GetWord(&pYdtBuf->DataBuf[Offset]));
	Offset += 2;
	OrderNum = SWAP_WORD(GetWord(&pYdtBuf->DataBuf[Offset]));
	Offset += 2;

	if(SavedRcd==0){
		pTemp->RecordCnt = RecordCnt;
		PinLevel = 0;
		LastOrderNum = 0;
	}else{
		if((LastOrderNum+1)!=OrderNum){

			memset(&Prot,0x20,sizeof(PROTOCAL_YDT1363_3));
			Prot.Addr = (uint8_t)GetDevAddr();
			Prot.Ver = YDT1363_3_PROTOCAL_VER;
			Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
			Prot.CID2 = YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL;
			Prot.DataBuf[0] = CmdID+1;
			Prot.DataBuf[1] = LastOrderNum;
			GetWord(&Prot.DataBuf[2]) = SWAP_WORD(SavedRcd); 
			Prot.Length = 4;
            
            //InfraFreeRecordTemp(&pTemp);
            //SavedRcd = 0;
            
			return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
		}
	}
	
	for(i=0;i<(pYdtBuf->Length-5)/2;i++){
		if(SavedRcd>=MIN(RecordCnt,INFRA_RECORD_MAX))
			break;
		pTemp->Record[SavedRcd].PinLevel = PinLevel;
		PinLevel = (PinLevel==0?1:0);
		pTemp->Record[SavedRcd].DurationUS = SWAP_WORD(GetWord(&pYdtBuf->DataBuf[Offset]));
		Offset += 2;
		SavedRcd++;
	}
    LastOrderNum = OrderNum;
	if(SavedRcd>=MIN(RecordCnt,INFRA_RECORD_MAX)){
		i=0;
		do{
			i++;
			Rtn = SaveInfraLearningCmdToFlash(InfraGetManuID(),CmdID,pTemp,sizeof(SaveRecordFormatDef));
		}while((i<=3)&&(0!=Rtn));
		if(Rtn==0){
			Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
			return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pBuf,pLen);
		}else{
			Rtn = YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL;
			return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pBuf,pLen);
		}
		InfraFreeRecordTemp(&pTemp);
		SavedRcd = 0;
	}else{
		Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
		return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pBuf,pLen);
	}
}
static int32_t ProtocolProc_YDT1363_3_Make_LearningStatus(uint8_t *pBuf,uint32_t *pLen)
{
	PROTOCAL_YDT1363_3 Prot;
	uint8_t Offset;
	uint32_t StaFlag=0;
		
	if((pBuf == NULL)||(pLen == NULL))
		return YDT1363_FUNC_RET_FALSE;


	InfraGetLearningStatus(&StaFlag);
	
    memset(&Prot,0x20,sizeof(PROTOCAL_YDT1363_3));
    Prot.Addr = (uint8_t)GetDevAddr();
	Prot.Ver = YDT1363_3_PROTOCAL_VER;
	Prot.CID1 = YDT1363_3_PROTOCAL_CID1_DCMETER;
	Prot.CID2 = YDT1363_3_PROTOCAL_RTN_NORMAL;
	Offset = 0;
	GetDWord(&Prot.DataBuf[Offset]) = SWAP_DWORD(StaFlag); 
	Offset += 4;
    Prot.Length = Offset;
    return ProtocolProc_YDT1363_3_Make(&Prot,pBuf,pLen);
}

/*
 ************************************************************************************************************************************************************************     
 *函数名称:	ProtocolProc_YDT1363_3
 *功能描述:	协议处理进程
 *输入描述:	收到的协议数据
 *输出描述:	协议回包缓冲区，长度
 *返回描述:	0成功 否则失败
 *作者日期:	20170712
 *全局声明: gYdtData
 *特殊说明: 无
 ************************************************************************************************************************************************************************       
 */
extern int32_t ProtocolProc_YDT1363_3(PPROTOCAL_YDT1363_3 pProcBuf, uint8_t *pSendBuf, uint32_t *pSendLen)
{
	uint8_t 	Rtn;
    REAL_DATE TmEx={0};
	volatile float fTemp;
	uint8_t i=0;
    int32_t AiCaliRet;
	
	if((pProcBuf==NULL)||(pSendBuf==NULL)||(pSendLen==NULL))
			return YDT1363_FUNC_RET_FALSE;
		
    //首先校验地址
    if(pProcBuf->Addr == (uint8_t)GetDevAddr())
    {
            //公共命令解析,VER与CID1可以为任意值的命令有:获取讯协议版本号
            if(pProcBuf->Ver == (uint8_t)YDT1363_3_PROTOCAL_VER)
            {
                    //假如CID1错误
                    if(pProcBuf->CID1 == YDT1363_3_PROTOCAL_CID1_DCMETER)
                    {
                    		gu8Prot_CID2 = pProcBuf->CID2;
                            //针对CID2处理
                            switch(pProcBuf->CID2)
                            {
                                    case YDT1363_3_PROTOCAL_CID2_GET_ANALOG:
                                        if(pProcBuf->Length==0)
                                        {
                                                 return ProtocolProc_YDT1363_3_Make_Anolog(&gYdtData.Analog,pSendBuf,pSendLen);
                                        }
                                        break;
                                    case YDT1363_3_PROTOCAL_CID2_GET_ANALOG_EX:
                                        if(pProcBuf->Length==0)
                                        {
                                                 return ProtocolProc_YDT1363_3_Make_Anolog_EX(&gYdtData.Analog,pSendBuf,pSendLen);
                                        }
                                        break;
                                    case YDT1363_3_PROTOCAL_CID2_GET_SWITCH:
                                        if(pProcBuf->Length==0)
                                        {
                                                 return ProtocolProc_YDT1363_3_Make_Switch(&gYdtData.Switch,pSendBuf,pSendLen);
                                        }
                                        break;
                                    case YDT1363_3_PROTOCAL_CID2_GET_ALARM:
                                        if(pProcBuf->Length==0)
                                        {
                                                return ProtocolProc_YDT1363_3_Make_Alarm(&gYdtData.Alarm,pSendBuf,pSendLen);
                                        }
                                        break;
                                    case YDT1363_3_PROTOCAL_CID2_REMOTE_CTRL:
                                        if((pProcBuf->Length==1)&&((pProcBuf->DataBuf[0]==0x10)||(pProcBuf->DataBuf[0]==0x1f)))
                                        {
                                            if(pProcBuf->DataBuf[0]==0x10)
                                            {
                                                    //空调开机
                                                    AirCondCtrlOperation(CMDID15_CTRLORDER_ON);
                                            }
                                            else
                                            {
                                                    //空调关机
                                                    AirCondCtrlOperation(CMDID1_CTRLORDER_OFF);
                                            }
                                            return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(YDT1363_3_PROTOCAL_RTN_NORMAL,pSendBuf,pSendLen);
                                        }
                                        break;
                                    case YDT1363_3_PROTOCAL_CID2_GET_PARAM:		//获取表参数
                                        if(pProcBuf->Length==0)
                                        {
                                                return ProtocolProc_YDT1363_3_Make_Param(&gYdtData.Param,pSendBuf,pSendLen);
                                        }
                                        break;             
                                    case YDT1363_3_PROTOCAL_CID2_SET_PARAM:		//设置表参数
                                        if(pProcBuf->Length==3)
                                        {
                                                 return ProtocolProc_YDT1363_3_Set_Param(&gYdtData.Param,pProcBuf->DataBuf,pSendBuf,pSendLen); 
                                        }
                                        break; 
                                    case YDT1363_3_PROTOCAL_CID2_SET_PARAM_INFRARED:
                                        if(pProcBuf->Length==6)
                                        {
                                                 return ProtocolProc_YDT1363_3_Set_ParamInfrared(&gYdtData.InfraredParam,pProcBuf->DataBuf,pSendBuf,pSendLen); 
                                        }
                                        break; 
                                    case YDT1363_3_PROTOCAL_CID2_GET_ENERGY:
                                        if(pProcBuf->Length==0)
                                        {
                                                return ProtocolProc_YDT1363_3_Make_Energy(&gYdtData.Egy,pSendBuf,pSendLen); 
                                        }
                                        break; 
                                    case YDT1363_3_PROTOCAL_CID2_GET_VERSION:
                                        if(pProcBuf->Length==0)
                                        {
                                                Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
                                                return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
                                        }
                                        else
                                        {
                                            
                                        }
                                        break;
                                    case YDT1363_3_PROTOCAL_CID2_FACTORY_RESET:
                                        if(pProcBuf->Length==0)
                                        {          
                                                MeterFactoryReset();
                                                Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
                                                return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
                                        }
                                        else
                                        {
                                            
                                        }
                                        break;  
                                    case YDT1363_3_PROTOCAL_CID2_CLEAN_SELF_LEARN:
                                        if(pProcBuf->Length==0)
                                        {
                                                AirCondSelfLearnClean();
                                                Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
                                                return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
                                        }
                                        break;
                                    case YDT1363_3_PROTOCAL_CID2_GET_DATE:
                                        if(pProcBuf->Length==0)
                                        {
                                                PM5KT_COMM_TM  		TmBuf;
                                            
                                                memset(&TmBuf,0,sizeof(TmBuf));
                                                if(RX8010_GetTime(&TmBuf)==1)
                                                {
                                                        //gYdtData.Date.Year 		= 1970+TmBuf.Year;
                                                        gYdtData.Date.Year 		= TmBuf.Year;
                                                        gYdtData.Date.Month		= TmBuf.Month;
                                                        gYdtData.Date.Day 		= TmBuf.Day;
                                                        gYdtData.Date.Hour		= TmBuf.Hour;
                                                        gYdtData.Date.Minute	= TmBuf.Minute;
                                                        gYdtData.Date.Second	= TmBuf.Second;
                                                }
                                                else
                                                {
                                                        if(RX8010_GetTime(&TmBuf)==1)
                                                        {
                                                                //gYdtData.Date.Year 		= 1970+TmBuf.Year;
                                                                gYdtData.Date.Year 		= TmBuf.Year;
                                                                gYdtData.Date.Month		= TmBuf.Month;
                                                                gYdtData.Date.Day 		= TmBuf.Day;
                                                                gYdtData.Date.Hour		= TmBuf.Hour;
                                                                gYdtData.Date.Minute	= TmBuf.Minute;
                                                                gYdtData.Date.Second	= TmBuf.Second;
                                                        }
                                                }
                                                return ProtocolProc_YDT1363_3_Make_Date(&gYdtData.Date,pSendBuf,pSendLen);
                                        }
                                        else
                                        {
                                            
                                        }
                                        break;
                                    case YDT1363_3_PROTOCAL_CID2_GET_DEVICE_ADDR:
                                        if(pProcBuf->Length==0)
                                        {
                                                Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
                                                return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
                                        }
                                        else
                                        {
                                            
                                        }
                                        break;
                                    case YDT1363_3_PROTOCAL_CID2_GET_VENDER:
                                        if(pProcBuf->Length==0)
                                        {
                                                return ProtocolProc_YDT1363_3_Make_Vender(pSendBuf,pSendLen);
                                        }
                                        else
                                        {
                                            
                                        }
                                        break; 
                                    case YDT1363_3_PROTOCAL_CID2_SET_DATE:
                                        if(pProcBuf->Length==7)
                                        {
                                                TmEx.Year 	= SWAP_WORD(GetWord(&pProcBuf->DataBuf[0]));
                                                TmEx.Month 	= pProcBuf->DataBuf[2];
                                                TmEx.Day 	= pProcBuf->DataBuf[3];
                                                TmEx.Hour 	= pProcBuf->DataBuf[4];
                                                TmEx.Minute = pProcBuf->DataBuf[5];
                                                TmEx.Second = pProcBuf->DataBuf[6];
                                                
                                                if(0 == ProtocolProc_YDT1363_3_SetDate(&TmEx,NULL))
                                                {
                                                        /*PM5KT_COMM_TM  		TmBuf;
                                                        
                                                        memset(&TmBuf,0,sizeof(TmBuf));
                                                        TmBuf.Year		= TmEx.Year-1970;
                                                        TmBuf.Month		= TmEx.Month;
                                                        TmBuf.Day			= TmEx.Day;
                                                        TmBuf.Hour		= TmEx.Hour;
                                                        TmBuf.Minute	= TmEx.Minute;
                                                        TmBuf.Second	= TmEx.Second;*/
                                                        RX8010_SetTime((PPM5KT_COMM_TM)&TmEx);
                                                    
                                                        Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
                                                        return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
                                                }
                                                else
                                                {
                                                        Rtn = YDT1363_3_PROTOCAL_RTN_DATA_INVALID;
                                                        return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
                                                }
                                        }
                                        else
                                        {
                                            
                                        }
                                        break;   
                                    case YDT1363_3_PROTOCAL_CID2_GET_HISTORY_DATA:
                                            if((pProcBuf->Length==1)&&(pProcBuf->DataBuf[0]>=0x10)&&(pProcBuf->DataBuf[0]<=0x12))
                                            {
                                                    if(gYdtData.SaveCnt.DayEgyCnt==0)
                                                    {
                                                            Rtn = YDT1363_3_PROTOCAL_RTN_DATA_NONE;
                                                            return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
                                                    }
                                                    else
                                                    {
                                                            return ProtocolProc_YDT1363_3_Make_HistoryDayData(pProcBuf->DataBuf[0],&gYdtData,pSendBuf,pSendLen);
                                                    }
                                            }
                                            else
                                            {
                                                    
                                            }
                                            break;   
                                    /*case YDT1363_3_PROTOCAL_CID2_GET_R_TEMP_RATIO:
                                    case YDT1363_3_PROTOCAL_CID2_GET_S_TEMP_RATIO:
                                        if(pProcBuf->Length==0)
                                        {
                                                if(pProcBuf->CID2==YDT1363_3_PROTOCAL_CID2_GET_R_TEMP_RATIO)
                                                        return ProtocolProc_YDT1363_3_Make_TEMP_RATIO(&gYdtData.RTempRadio,pSendBuf,pSendLen);
                                                else
                                                        return ProtocolProc_YDT1363_3_Make_TEMP_RATIO(&gYdtData.STempRadio,pSendBuf,pSendLen);
                                        }
                                        else
                                        {
                                            
                                        }
                                        break;
                                    case YDT1363_3_PROTOCAL_CID2_SET_R_TEMP_RATIO:
                                    case YDT1363_3_PROTOCAL_CID2_SET_S_TEMP_RATIO:
                                        if(pProcBuf->Length==12)
                                        {
                                                
                                            
                                                if(pProcBuf->CID2==YDT1363_3_PROTOCAL_CID2_SET_R_TEMP_RATIO)
                                                {
                                                        ProtocolProc_YDT1363_3_Set_TEMP_RATIO(pProcBuf->DataBuf,&gYdtData.RTempRadio);
                                                        i=0;
                                                        do
                                                        {
                                                                i++;
                                                                WatchDogFeed();
                                                                Rtn = MeterData_WaitWriteSucc(1,(uint8_t *)&gYdtData.RTempRadio,FLASH_ADDR_R_TEMP_RATIO,FLASH_ADDR_R_TEMP_RATIO_SIZE);
                                                                WatchDogFeed();
                                                        }while((i<3)&&(Rtn!=0));
                                                        
                                                }
                                                else
                                                {
                                                        ProtocolProc_YDT1363_3_Set_TEMP_RATIO(pProcBuf->DataBuf,&gYdtData.STempRadio);
                                                        i=0;
                                                        do
                                                        {
                                                                i++;
                                                                WatchDogFeed();
                                                                Rtn = MeterData_WaitWriteSucc(1,(uint8_t *)&gYdtData.STempRadio,FLASH_ADDR_S_TEMP_RATIO,FLASH_ADDR_S_TEMP_RATIO_SIZE);
                                                                WatchDogFeed();
                                                        }while((i<3)&&(Rtn!=0));
                                                }
                                                return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(YDT1363_3_PROTOCAL_RTN_NORMAL,pSendBuf,pSendLen);
                                        }
                                        else
                                        {
                                            
                                        }
                                        break;  */
                                    case YDT1363_3_PROTOCAL_CID2_RN8209C_AC_GAIN:
                                    //case YDT1363_3_PROTOCAL_CID2_RN8209C_AC_PHASE:
                                    //case YDT1363_3_PROTOCAL_CID2_RN8209C_AC_OFFSET:
                                    //case YDT1363_3_PROTOCAL_CID2_RN8209C_AC_REACTIVE:
                                    //case YDT1363_3_PROTOCAL_CID2_RN8209C_AC_CURR:
                                        if(pProcBuf->Length==0)
                                        {
                                                if(pProcBuf->CID2==YDT1363_3_PROTOCAL_CID2_RN8209C_AC_GAIN)
                                                {
                                                        WatchDogFeed();
                                                        Rtn = ACGainCalibration_ALL(gYdtData.pRn8209c);
                                                        WatchDogFeed();
                                                        WriteRn8209CalidataToFlash(gYdtData.pRn8209c);
                                                        WatchDogFeed();
                                                }
                                                else if(pProcBuf->CID2==YDT1363_3_PROTOCAL_CID2_RN8209C_AC_PHASE)
                                                {
                                                        WatchDogFeed();
                                                        Rtn = ACPhaseCalibration_ALL(gYdtData.pRn8209c);
                                                        WatchDogFeed();
                                                        WriteRn8209CalidataToFlash(gYdtData.pRn8209c);
                                                        WatchDogFeed();
                                                }
                                                else if(pProcBuf->CID2==YDT1363_3_PROTOCAL_CID2_RN8209C_AC_OFFSET)
                                                {
                                                        WatchDogFeed();
                                                        Rtn = ACOffsetCalibration_ALL(gYdtData.pRn8209c);
                                                        WatchDogFeed();
                                                        WriteRn8209CalidataToFlash(gYdtData.pRn8209c);
                                                        WatchDogFeed();
                                                }
                                                else if(pProcBuf->CID2==YDT1363_3_PROTOCAL_CID2_RN8209C_AC_REACTIVE)
                                                {
                                                        WatchDogFeed();
                                                        Rtn = ACReactiveCalibration_ALL(gYdtData.pRn8209c);
                                                        WatchDogFeed();
                                                        WriteRn8209CalidataToFlash(gYdtData.pRn8209c);
                                                        WatchDogFeed();
                                                }
                                                else
                                                {
                                                        WatchDogFeed();
                                                        Rtn = ACCurrentOffsetCalibration_ALL(gYdtData.pRn8209c);
                                                        WatchDogFeed();
                                                        WriteRn8209CalidataToFlash(gYdtData.pRn8209c);
                                                        WatchDogFeed();
                                                }
                                                    
                                                return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(YDT1363_3_PROTOCAL_RTN_NORMAL,pSendBuf,pSendLen);
                                        }
                                        else
                                        {
                                            
                                        }
                                        break;
                                    case YDT1363_3_PROTOCAL_CID2_GET_RN8209C_INITPARAM:
                                        if(pProcBuf->Length==1)
                                        {
                                                if((pProcBuf->DataBuf[0]>=1)&&(pProcBuf->DataBuf[0]<=3))
                                                        return ProtocolProc_YDT1363_3_Make_RN8209C_INITPARAM(pProcBuf->DataBuf[0],gYdtData.pRn8209c,pSendBuf,pSendLen);
                                                
                                                Rtn = YDT1363_3_PROTOCAL_RTN_DATA_INVALID;
                                                return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
                                        }
                                        break;    
                                    case YDT1363_3_PROTOCAL_CID2_SET_RN8209C_INITPARAM:
                                        if(pProcBuf->Length>sizeof(RN8209C_DEF))
                                        {
                                                if(0==ProtocolProc_YDT1363_3_Set_RN8209C_INITPARAM(pProcBuf->DataBuf,gYdtData.pRn8209c))
                                                {
                                                        WatchDogFeed();
                                                        WriteRn8209CalidataToFlash(gYdtData.pRn8209c);
                                                        WatchDogFeed();
                                                        return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(YDT1363_3_PROTOCAL_RTN_NORMAL,pSendBuf,pSendLen);
                                                }                      
                                        }
                                        break;     
                                    case YDT1363_3_PROTOCAL_CID2_GET_SOFT_VER:
                                        if(pProcBuf->Length==0){
                                            return ProtocolProc_YDT1363_3_Make_SoftWare(pSendBuf,pSendLen);
                                        }
                                        else{
                                            
                                        }
                                        break;
                                    case YDT1363_3_PROTOCAL_CID2_AI_CALI:
                                        if(pProcBuf->Length==5){
                                            if(pProcBuf->DataBuf[0]==1){
												
                                                fTemp = (float)SWAP_DWORD(GetDWord(&pProcBuf->DataBuf[1]))/1000.0;
												AiCaliRet = ProtocolProc_YDT1363_3_AI_Cali_ReturnTemperature(fTemp,&gYdtData.RTempRadio);
												if(AiCaliRet==0x5a){
													Rtn = YDT1363_3_PROTOCAL_RTN_AI_CALI_FIRST;
													return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
												}else if(AiCaliRet==YDT1363_FUNC_RET_FALSE){
													Rtn = YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL;
													return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
												}else if(AiCaliRet==YDT1363_FUNC_RET_SUCCESS){
													i = 0;
													do
	                                                {
	                                                    WatchDogFeed();
	                                                    Rtn = MeterData_WaitWriteSucc(1,(uint8_t *)&gYdtData.RTempRadio,FLASH_ADDR_R_TEMP_RATIO,FLASH_ADDR_R_TEMP_RATIO_SIZE);
	                                                    WatchDogFeed();
	                                                }while((i++<3)&&(Rtn!=0));
													if((i>=3)&&(Rtn!=0)){
														Rtn = YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL;
														return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
													}else{
														Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
														return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
													}
												}
                                                
                                            }else if(pProcBuf->DataBuf[0]==2){
                                               
												fTemp = (float)SWAP_DWORD(GetDWord(&pProcBuf->DataBuf[1]))/1000.0;
												AiCaliRet = ProtocolProc_YDT1363_3_AI_Cali_SupplyTemperature(fTemp,&gYdtData.STempRadio);
												if(AiCaliRet==0x5a){
													Rtn = YDT1363_3_PROTOCAL_RTN_AI_CALI_FIRST;
													return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
												}else if(AiCaliRet==YDT1363_FUNC_RET_FALSE){
													Rtn = YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL;
													return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
												}else if(AiCaliRet==YDT1363_FUNC_RET_SUCCESS){
													i = 0;
													do
	                                                {
	                                                    WatchDogFeed();
	                                                    Rtn = MeterData_WaitWriteSucc(1,(uint8_t *)&gYdtData.STempRadio,FLASH_ADDR_S_TEMP_RATIO,FLASH_ADDR_S_TEMP_RATIO_SIZE);
	                                                    WatchDogFeed();
	                                                }while((i++<3)&&(Rtn!=0));
													if((i>=3)&&(Rtn!=0)){
														Rtn = YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL;
														return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
													}else{
														Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
														return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
													}
												}
												
                                            }
                                        }
                                        else if((pProcBuf->Length==9)&&(pProcBuf->DataBuf[0]==0xff)){
                                            fTemp = (float)SWAP_DWORD(GetDWord(&pProcBuf->DataBuf[1]))/1000.0;
                                            
                                            AiCaliRet = ProtocolProc_YDT1363_3_AI_Cali_ReturnTemperature(fTemp,&gYdtData.RTempRadio);
                                            fTemp = (float)SWAP_DWORD(GetDWord(&pProcBuf->DataBuf[5]))/1000.0;
                                            AiCaliRet += ProtocolProc_YDT1363_3_AI_Cali_SupplyTemperature(fTemp,&gYdtData.STempRadio);
											if(AiCaliRet==(0x5a+0x5a)){
												Rtn = YDT1363_3_PROTOCAL_RTN_AI_CALI_FIRST;
												return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
											}else if(AiCaliRet<0){
												Rtn = YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL;
												return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
											}else if(AiCaliRet==YDT1363_FUNC_RET_SUCCESS){
												i = 0;
												Rtn = 0;
												do{
													WatchDogFeed();
													Rtn += MeterData_WaitWriteSucc(1,(uint8_t *)&gYdtData.RTempRadio,FLASH_ADDR_R_TEMP_RATIO,FLASH_ADDR_R_TEMP_RATIO_SIZE);
													WatchDogFeed();
												}while((i++<3)&&(Rtn!=0));
												i = 0; 
												do
												{
													WatchDogFeed();
													Rtn += MeterData_WaitWriteSucc(1,(uint8_t *)&gYdtData.STempRadio,FLASH_ADDR_S_TEMP_RATIO,FLASH_ADDR_S_TEMP_RATIO_SIZE);
													WatchDogFeed();
												}while((i++<3)&&(Rtn!=0));
												
												Rtn = (Rtn==0?YDT1363_3_PROTOCAL_RTN_NORMAL:YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL);
												return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
											}
                                        }
                                        break;    
                                        
                                    case YDT1363_3_PROTOCAL_CID2_SET_TO_BOOT:
                                        if(pProcBuf->Length==4){
                                            uint32_t APROM_SIZE,Flag,SetBaudRate;
                                            SetBaudRate = SWAP_DWORD(GetDWord(pProcBuf->DataBuf));
                                            SYS_UnlockReg();
                                            FMC_Open();
                                            FMC_ENABLE_AP_UPDATE();
                                            APROM_SIZE = GetApromSize();
                                            Flag = APROM_TO_LDROM;
                                            
                                            switch(SetBaudRate)
                                            {
                                                /*case 1200:
                                                    Flag |= LDROM_BAUDRATE_1200;
                                                    break;
                                                case 2400:
                                                    Flag |= LDROM_BAUDRATE_2400;
                                                    break;
                                                case 4800:
                                                    Flag |= LDROM_BAUDRATE_4800;
                                                    break;*/
                                                case 19200:
                                                    Flag |= LDROM_BAUDRATE_19200;
                                                    break;
                                                default:
                                                    Flag |= LDROM_BAUDRATE_9600;
                                                    break;
                                            }
                                            
                                            FMC_Write(APROM_SIZE-8,Flag);
                                            Flag = 0;
                                            Flag = FMC_Read(APROM_SIZE-8);
                                            if((Flag & APROM_TO_LDROM_MSK) != APROM_TO_LDROM){
                                                Rtn = YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL;
                                                return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(\
                                                    Rtn,pSendBuf,pSendLen);
                                            }
                                            FMC_DISABLE_AP_UPDATE();

                                            FMC_Close();
                                            SYS_LockReg();
                                            Rtn = YDT1363_3_PROTOCAL_RTN_NORMAL;
                                            Sys_Hard_Reset();
                                            return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(\
                                                    Rtn,pSendBuf,pSendLen);
                                        }
                                        else{
                                            
                                        }
                                        break; 
									case YDT1363_3_PROTOCAL_CID2_GET_MANUFACTURER:
                                        if(pProcBuf->Length==0){
                                            return ProtocolProc_YDT1363_3_Make_CtrlManufacturer(pSendBuf,pSendLen);
                                        }
                                        else{
                                            
                                        }
                                        break;
									case YDT1363_3_PROTOCAL_CID2_SET_MANUFACTURER:
                                        if(pProcBuf->Length==4)
                                        {
                                        	return ProtocolProc_YDT1363_3_Set_CtrlManufacturer(pProcBuf->DataBuf,pSendBuf,pSendLen);
                                        }
                                        else
                                        {
                                            
                                        }
                                        break;
									case YDT1363_3_PROTOCAL_CID2_UPLOAD_CMD:
                                        if((pProcBuf->Length==1)&&(CMDIDIsValid(pProcBuf->DataBuf[0]-1)))
                                        {
                                        	return ProtocolProc_YDT1363_3_Make_UpLoadCmd(pProcBuf->DataBuf[0]-1,pSendBuf,pSendLen,UPCOM_Write);
                                        }
                                        else
                                        {
                                            
                                        }
                                        break;
									case YDT1363_3_PROTOCAL_CID2_DOWNLOAD_CMD:
                                        if((pProcBuf->Length>=5)&&(CMDIDIsValid(pProcBuf->DataBuf[0]-1)))
                                        {
                                        	return ProtocolProc_YDT1363_3_Set_DownLoadCmd(pProcBuf,pSendBuf,pSendLen);
                                        }
                                        else
                                        {
                                            
                                        }
                                        break;
									case YDT1363_3_PROTOCAL_CID2_GET_LEARNING_STATUS:
                                        if(pProcBuf->Length==0){
                                            return ProtocolProc_YDT1363_3_Make_LearningStatus(pSendBuf,pSendLen);
                                        }
                                        else{
                                            
                                        }
                                        break;
										
                                    default:
                                        Rtn = YDT1363_3_PROTOCAL_RTN_CID2_INVALID;
                                        return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
                                        //break;
                            }
                            Rtn = YDT1363_3_PROTOCAL_RTN_FORMAT_ERROR;
                            return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
                    }

            }
            else
            {
                    //9.3.1获取讯协议版本号（4FH）无需校验VER 假如是获取设备地址命令
                    if((pProcBuf->CID2 == YDT1363_3_PROTOCAL_CID2_GET_VERSION)&&(pProcBuf->Length==0))
                    {
                            return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(YDT1363_3_PROTOCAL_RTN_VER_ERROR,pSendBuf,pSendLen);
                    }
                    else if((pProcBuf->CID2 == YDT1363_3_PROTOCAL_CID2_GET_DEVICE_ADDR)&&(pProcBuf->Length==0))
                    {
                            return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(YDT1363_3_PROTOCAL_RTN_NORMAL,pSendBuf,pSendLen);
                    }
                    else if(pProcBuf->CID2 == YDT1363_3_PROTOCAL_CID2_GET_VERSION)
                    {
                            Rtn = YDT1363_3_PROTOCAL_RTN_FORMAT_ERROR;
                            return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
                    }
                    else
                    {
                            Rtn = YDT1363_3_PROTOCAL_RTN_VER_ERROR;
                            return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
                    }  
            }
		}
		else
		{
				//假如是获取设备地址命令
				if((pProcBuf->CID2 == YDT1363_3_PROTOCAL_CID2_GET_DEVICE_ADDR)&&(pProcBuf->Length==0))
				{
						return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(YDT1363_3_PROTOCAL_RTN_NORMAL,pSendBuf,pSendLen);
				}
				else if(pProcBuf->CID2 == YDT1363_3_PROTOCAL_CID2_GET_DEVICE_ADDR)
				{
						Rtn = YDT1363_3_PROTOCAL_RTN_FORMAT_ERROR;
						return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
				}
				else
				{
						//假如地址不对,CID2又不是获取地址命令，那就不是发给本设备的
						return	YDT1363_FUNC_RET_FALSE;
				}	
		}
		
		Rtn = YDT1363_3_PROTOCAL_RTN_FORMAT_ERROR;
		return ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(Rtn,pSendBuf,pSendLen);
}

extern PYDT1363_DATA ProtocolProc_YDT1363_3_CallBack(void)
{
    return (PYDT1363_DATA)&gYdtData;
}

