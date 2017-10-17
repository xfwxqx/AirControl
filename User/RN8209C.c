//#include "PM5000T.h"

#include "Usart.h"
#include <string.h>
#include "NUC029xAN.h"
#include "RN8209C.h"
#include "UserCommon.h"
#include <math.h>
/**************************************
	全局变量声明
****************************************/
RN8209C_INIT_PARAM	sInitParam;

#define EN1 				P03
#define EN2 				P41
#define EN3 				P04
#define EN4 				P05


/*****************************************************************************
** Function name:RN8209C_ReadReg(uint8_t RegAddr,uint8_t *pBuf,uint8_t Len)
**
** Description:读RN8209寄存器
**
** Parameters:RegAddr 寄存器地址，*pBuf读出值的存放地址，Len：待读值的长度   
**
** Returned value:	操作标识-0成功,1-输入参数错误 2-读取失败
**
******************************************************************************/
//累积和不能超过uint32_t的最大值
static uint64_t GetSum_u32(uint32_t *s,uint8_t Cnt)
{
    uint8_t i;
    uint64_t Sum;
    
    for(i=0,Sum=0;i<Cnt;i++)
		{
        Sum += s[i];
    }
		
    return Sum;
}

extern uint32_t RN8209C_ReadReg(uint8_t RegAddr,uint8_t *pBuf,uint8_t Len)
{
		uint8_t i,j,CheckSum=0,Timeout;
		uint8_t ReadBuf[5]={0};
		
		if(pBuf==NULL)
				return FUNC_RET_PARAM_ERR;
		if((Len==0)||(Len>4))
				return FUNC_RET_PARAM_ERR;

		Timeout 	= 0;
		CheckSum 	= RegAddr;
				
		do
		{
				WatchDogFeed();
				DOWNCOM_Write(&RegAddr,1);
				WatchDogFeed();
				if(FALSE == UART_Read(PM5KT_DOWNCOM,ReadBuf,Len+1))
						return FUNC_RET_FAIL;
				WatchDogFeed();
				for(i=0;i<Len;i++)
						CheckSum += ReadBuf[i];
				CheckSum = ~CheckSum;
				if(CheckSum == ReadBuf[Len])
				{
						for(j=0;j<Len;j++)
							 pBuf[Len-j-1] = ReadBuf[j];
						
						return FUNC_RET_SUCC;
				}
				Timeout++;
		}
		while(Timeout<3);
		
		return FUNC_RET_FAIL;
}

/*****************************************************************************
** Function name:RN8209C_WiteReg(uint8_t RegAddr,uint8_t *pBuf,uint8_t Len)
**
** Description:写RN8209寄存器
**
** Parameters:RegAddr 寄存器地址，*pBuf读出值的存放地址，Len：待读值的长度   
**
** Returned value:	操作标识-0写入成功,1-输入参数错误 2-写入失败
**
******************************************************************************/
extern uint32_t RN8209C_WiteReg(uint8_t RegAddr,uint8_t *pBuf,uint8_t Len)
{
		uint8_t 	i,CheckSum=0;
    uint8_t 	WriteBuf[5]	={0};
    uint8_t 	ReadBuf[5]	={0};
	
		if(pBuf==NULL)
				return FUNC_RET_PARAM_ERR;
		if((Len==0)||(Len>2))
				return FUNC_RET_PARAM_ERR;

		CheckSum = 0;
		
		WriteBuf[0] = RegAddr|0x80;
		for(i=0;i<Len;i++)
        WriteBuf[Len-i] = pBuf[i];
		for(i=0;i<(Len+1);i++)
			CheckSum += WriteBuf[i];
		
		CheckSum = ~CheckSum;
		WriteBuf[Len+1] = CheckSum;
		
		i=0;
		
		do
		{	
				WatchDogFeed();
				DOWNCOM_Write(WriteBuf,Len+2);
				WatchDogFeed();
				if(FUNC_RET_SUCC==RN8209C_ReadReg(RN8209C_REG_WData,ReadBuf,RN8209C_REG_WData_LEN))
				{
						WatchDogFeed();
            if(0==memcmp(ReadBuf,pBuf,Len))
								return FUNC_RET_SUCC;
				}
				WatchDogFeed();
				i++;
		}while(i<3);
		
		return FUNC_RET_FAIL;
}
extern uint32_t RN8209C_SoftReset(void) 
{
    uint8_t Buf[1]= {0xfa};
		
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_WiteReg(0xea,Buf,1))
        return FUNC_RET_FAIL;
		
		WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_ReadReg(RN8209C_REG_SysStatus,Buf,RN8209C_REG_SysStatus_LEN))
        return FUNC_RET_FAIL;
    WatchDogFeed();
		
    return ((Buf[0]&(1<<1))?FUNC_RET_SUCC:FUNC_RET_FAIL);
}
extern uint32_t RN8209C_WriteEnable(void) 
{
    uint8_t Buf[1]= {0xe5};
		
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_WiteReg(0xea,Buf,1))
        return FUNC_RET_FAIL;
		
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_ReadReg(RN8209C_REG_SysStatus,Buf,RN8209C_REG_SysStatus_LEN))
        return FUNC_RET_FAIL;
		WatchDogFeed();
		
    return ((Buf[0]&(1<<4))?FUNC_RET_SUCC:FUNC_RET_FAIL);
}
extern uint32_t RN8209C_WriteProtect(void) 
{
    uint8_t Buf[1]={0xdc};

    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_WiteReg(0xea,Buf,1))
        return FUNC_RET_FAIL;
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_ReadReg(RN8209C_REG_SysStatus,Buf,RN8209C_REG_SysStatus_LEN))
        return FUNC_RET_FAIL;
		WatchDogFeed();
		
    return ((Buf[0]&(1<<4))?FUNC_RET_FAIL:FUNC_RET_SUCC);
}
extern void RN8209C_WaitDataUpdate(void) 
{
    uint8_t Buf[1]={0}; 
		
    do
    {
				WatchDogFeed();
        RN8209C_ReadReg(RN8209C_REG_IF,Buf,RN8209C_REG_IF_LEN);
    }while(0==(Buf[0]&0x01));
		WatchDogFeed();
}
extern uint8_t RN8209C_TestDataUpdate(void) 
{
    uint8_t Buf[1]={0}; 
    
    RN8209C_ReadReg(RN8209C_REG_IF,Buf,RN8209C_REG_IF_LEN);
		
		return (Buf[0]&0x01);
}
extern uint16_t RN8209C_GetCheckSum(void) 
{
    uint8_t Buf[3]={0}; 
		
    do
    {
        ////等待校验值可用，11.2us
				WatchDogFeed();
        RN8209C_ReadReg(RN8209C_REG_EMUStatus,Buf,RN8209C_REG_EMUStatus_LEN);
    }while(0!=(Buf[2]&0x01));
		WatchDogFeed();
		
    return GetWord(&Buf[0]);
}

extern uint32_t RN8209C_Init_ResetParam(PRN8209C_INIT_PARAM pInitParam)
{
    if(pInitParam==NULL)
        return FUNC_RET_PARAM_ERR;
    
    memset((void *)pInitParam,0,sizeof(RN8209C_INIT_PARAM));
    //pInitParam->UNION.PARAM_List.SYSCON 	= 0x0040;
    pInitParam->UNION.PARAM_List.SYSCON  		&= (~REG_SYSCON_PGAIA_MSK);
    pInitParam->UNION.PARAM_List.SYSCON 		|= REG_SYSCON_PGAIA_1;
    pInitParam->UNION.PARAM_List.SYSCON  		&= (~REG_SYSCON_PGAU_MSK);
    pInitParam->UNION.PARAM_List.SYSCON 		|= REG_SYSCON_PGAU_1;
    pInitParam->UNION.PARAM_List.SYSCON  		&= (~REG_SYSCON_PGAIB_MSK);
    pInitParam->UNION.PARAM_List.SYSCON 		|= REG_SYSCON_PGAIB_1;
    pInitParam->UNION.PARAM_List.SYSCON  		&= (~REG_SYSCON_ADC2ON_MSK);
    pInitParam->UNION.PARAM_List.SYSCON 		|= REG_SYSCON_ADC2ON_ON;
    //关闭IB通道数字高通滤波器;
    pInitParam->UNION.PARAM_List.EMUCON 		= 0x4003;
		
    //选择电表脉冲常数EC=3600,
    //pInitParam->UNION.PARAM_List.HFConst 	= 0x1c7f;//
    //pInitParam->UNION.PARAM_List.HFConst 	= 0x1000;
    pInitParam->UNION.PARAM_List.HFConst 		= 0x2dad;
    pInitParam->UNION.PARAM_List.PStart 		= 0x0060;
    pInitParam->UNION.PARAM_List.DStart  		= 0x0120;
   
    pInitParam->UNION.PARAM_List.GPQA 			= 0x0000;
    pInitParam->UNION.PARAM_List.GPQB 			= 0x0000;
    pInitParam->UNION.PARAM_List.PhsA				= 0x0000;
    pInitParam->UNION.PARAM_List.PhsB				= 0x0000;
    pInitParam->UNION.PARAM_List.QPhsCal 		= 0x0000;
    pInitParam->UNION.PARAM_List.APOSA 			= 0x0000;
    pInitParam->UNION.PARAM_List.APOSB 			= 0x0000;
    pInitParam->UNION.PARAM_List.RPOSA 			= 0x0000;
    pInitParam->UNION.PARAM_List.RPOSB 			= 0x0000;
    pInitParam->UNION.PARAM_List.IARMSOS 		= 0x0000;
    pInitParam->UNION.PARAM_List.IBRMSOS 		= 0x0000;
    pInitParam->UNION.PARAM_List.IBGain 		= 0x0000;
    pInitParam->UNION.PARAM_List.D2FPL 			= 0x0000;
    pInitParam->UNION.PARAM_List.D2FPH 			= 0x0000;
    pInitParam->UNION.PARAM_List.DCIAH 			= 0x0000;
    pInitParam->UNION.PARAM_List.DCIBH 			= 0x0000;
    pInitParam->UNION.PARAM_List.DCUH 			= 0x0000;
    pInitParam->UNION.PARAM_List.DCL 				= 0x0000;
    pInitParam->UNION.PARAM_List.EMUCON2 		= 0x0000;//自定义电能输入选择为无功功率
    
    return FUNC_RET_SUCC;
}

static uint32_t RN8209C_Init_Chip(PRN8209C_DEF pRnDef)
{
		uint8_t			i;
		uint16_t 		CheckSum=0;
    PRN8209C_INIT_PARAM pInitParam=NULL;
    
    if(pRnDef==NULL)
        return FUNC_RET_PARAM_ERR;
    
		WatchDogFeed();
    if(FUNC_RET_SUCC != RN8209C_WriteEnable())
        return FUNC_RET_FAIL;
    WatchDogFeed();
    if(FUNC_RET_SUCC != RN8209C_SoftReset())
        goto Err;

		WatchDogFeed();
		if(FUNC_RET_SUCC != RN8209C_WriteEnable())
        return FUNC_RET_FAIL;
		
    pInitParam = &pRnDef->InitParam;
		CheckSum=0;
		for(i=0;i<24;i++)
		{
				CheckSum += (uint16_t)pInitParam->UNION.PARAM_Union[i];
				if((i==7)||(i==8))
				{
						WatchDogFeed();
						if(FUNC_RET_SUCC!=RN8209C_WiteReg(i,(uint8_t *)&pInitParam->UNION.PARAM_Union[i],1))
							goto Err;
				}
				else
				{
						WatchDogFeed();
						if(FUNC_RET_SUCC!=RN8209C_WiteReg(i,(uint8_t *)&pInitParam->UNION.PARAM_Union[i],2))
							goto Err;
				}
		}
		WatchDogFeed();
		CheckSum += 0x1600;
		CheckSum = ~CheckSum;
		
    pInitParam->UNION.PARAM_List.CHKSUM = CheckSum;
    if(pInitParam->UNION.PARAM_List.CHKSUM != RN8209C_GetCheckSum())
        goto Err;
    WatchDogFeed();
		
    RN8209C_WriteProtect(); 
		WatchDogFeed();
		
    return FUNC_RET_SUCC;
	
Err:RN8209C_WriteProtect(); 
		WatchDogFeed();
		
    return FUNC_RET_FAIL;
}

extern uint32_t RN8209C_Init(PRN8209C pRn8209)
{
    uint8_t i;
    uint32_t Ret=0;
    
    if(pRn8209==NULL)
        return FUNC_RET_PARAM_ERR;
    
    for(i=0;i<3;i++)
		{
        UART_CS(i+1);
        DelayUs(10);
				WatchDogFeed();
        Ret += RN8209C_Init_Chip(&pRn8209->Def[i]);
        UART_CS(0);
    }
		
    return Ret;
}

extern uint32_t RN8209C_GetRegValue(uint8_t RegAddr,uint32_t *pVal)
{
    uint8_t RegAddrTemp=0,RegLen=0;
    uint32_t u32Temp=0;
    
    if(pVal==NULL)
        return FUNC_RET_PARAM_ERR;
    
    switch(RegAddr)
    {
        case RN8209C_REG_PhsA:
        case RN8209C_REG_PhsB:
        case RN8209C_REG_IE:
        case RN8209C_REG_IF:
        case RN8209C_REG_SysStatus:
            RegAddrTemp = RegAddr;
            RegLen = 1;
            break;
        case RN8209C_REG_IARMS:
        case RN8209C_REG_IBRMS:  
        case RN8209C_REG_URMS:
        case RN8209C_REG_EnergyP:
        case RN8209C_REG_EnergyP2:
        case RN8209C_REG_EnergyD:
        case RN8209C_REG_EnergyD2:
        case RN8209C_REG_EMUStatus:
        case RN8209C_REG_SPL_IA:
        case RN8209C_REG_SPL_IB:
        case RN8209C_REG_SPL_U:
        case RN8209C_REG_UFreq2:
        case RN8209C_REG_DeviceID:
            RegAddrTemp = RegAddr;
            RegLen = 3;
						break;
        case RN8209C_REG_SYSCON:
        case RN8209C_REG_EMUCON:  
        case RN8209C_REG_HFConst:
        case RN8209C_REG_PStart:
        case RN8209C_REG_DStart:
        case RN8209C_REG_GPQA:
        case RN8209C_REG_GPQB:
        case RN8209C_REG_QPHSCAL:
        case RN8209C_REG_APOSA:
        case RN8209C_REG_APOSB:
        case RN8209C_REG_RPOSA:
        case RN8209C_REG_RPOSB:
        case RN8209C_REG_IARMSOS:
        case RN8209C_REG_IBRMSOS:
        case RN8209C_REG_IBGain:  
        case RN8209C_REG_D2FPL:
        case RN8209C_REG_D2FPH:
        case RN8209C_REG_DCIAH:
        case RN8209C_REG_DCIBH:
        case RN8209C_REG_DCUH:
        case RN8209C_REG_DCL:
        case RN8209C_REG_EMUCON2:
        case RN8209C_REG_PFCnt:
        case RN8209C_REG_DFcnt:
        case RN8209C_REG_UFreq:
        case RN8209C_REG_WData:
            RegAddrTemp = RegAddr;
            RegLen = 2;
            break;
        case RN8209C_REG_PowerPA:
        case RN8209C_REG_PowerPB:
        case RN8209C_REG_PowerQ:
        case RN8209C_REG_RDataS:
            RegAddrTemp = RegAddr;
            RegLen = 4;
            break;
        
        default:
            RegAddrTemp = RN8209C_REG_DeviceID;
            RegLen = RN8209C_REG_DeviceID_LEN;
            break;
    }
    WatchDogFeed();
    if(0==RN8209C_ReadReg(RegAddrTemp,(uint8_t *)&u32Temp,RegLen)){
        *pVal = u32Temp;
        return FUNC_RET_SUCC;
    }
		WatchDogFeed();
		
    return FUNC_RET_FAIL;
}

//多次测量取Ia、Ib、U平均值
extern uint32_t RN8209C_GetCurrVolAverage(uint32_t *pIa,uint32_t *pIb,uint32_t *pU)
{
    uint8_t 	Cnt;
    uint32_t 	IaTemp[11]={0};
    uint32_t 	IbTemp[11]={0};
    uint32_t 	UTemp[11]={0};
    uint64_t 	Sum;
    
    for(Cnt=0;Cnt<11;Cnt++)
		{
        //TIMER_Delay(TIMER1,700000);		//接近数据更新频率
				WatchDogFeed();
        RN8209C_WaitDataUpdate();
				WatchDogFeed();
        if((pIa!=NULL)&&(FUNC_RET_SUCC != RN8209C_GetRegValue(RN8209C_REG_IARMS,&IaTemp[Cnt])))
            return FUNC_RET_FAIL;
				WatchDogFeed();
        if((pIb!=NULL)&&(FUNC_RET_SUCC != RN8209C_GetRegValue(RN8209C_REG_IBRMS,&IbTemp[Cnt])))
            return FUNC_RET_FAIL;
				WatchDogFeed();
        if((pU!=NULL)&&(FUNC_RET_SUCC != RN8209C_GetRegValue(RN8209C_REG_URMS,&UTemp[Cnt])))
            return FUNC_RET_FAIL;
				WatchDogFeed();
    }
    Sum = GetSum_u32(&IaTemp[1],10);
    Sum /= 10;
		
    if(pIa!=NULL)
        *pIa = Sum;
    Sum = GetSum_u32(&IbTemp[1],10);
    Sum /= 10;
		
    if(pIb!=NULL)
        *pIb = Sum;
    Sum = GetSum_u32(&UTemp[1],10);
    Sum /= 10;
		
    if(pU!=NULL)
        *pU = Sum;
    WatchDogFeed();
		
    return FUNC_RET_SUCC;
}
//多次测量取Pa、Pb平均值
static uint32_t RN8209C_GetPowerAverage(int32_t *pPa,int32_t *pPb)
{
    uint8_t Cnt;
    int32_t PaTemp[11]={0};
    int32_t PbTemp[11]={0};
    int64_t Sum;
    
    for(Cnt=0;Cnt<11;Cnt++)
		{
        //TIMER_Delay(TIMER1,700000);//接近数据更新频率
				WatchDogFeed();
        RN8209C_WaitDataUpdate();
				WatchDogFeed();
        if(FUNC_RET_SUCC != RN8209C_GetRegValue(RN8209C_REG_PowerPA,(uint32_t *)&PaTemp[Cnt]))
            return FUNC_RET_FAIL;
				WatchDogFeed();
        if(FUNC_RET_SUCC != RN8209C_GetRegValue(RN8209C_REG_PowerPB,(uint32_t *)&PbTemp[Cnt]))
            return FUNC_RET_FAIL;
        WatchDogFeed();
    }
		Sum = 0;
		for(Cnt=1;Cnt<11;Cnt++)
		{
				Sum += PaTemp[Cnt];
		}
    Sum /= 10;
    if(pPa!=NULL)
        *pPa = (int32_t)Sum;
	
    Sum = 0;
		for(Cnt=1;Cnt<11;Cnt++)
		{
				Sum += PbTemp[Cnt];
		}
    Sum /= 10;
    if(pPb!=NULL)
        *pPb = (int32_t)Sum;
    
    return FUNC_RET_SUCC;
}

extern uint32_t GetSPLIB(uint32_t *pIb)
{
    RN8209C_WaitDataUpdate();
    if((pIb!=NULL)&&(FUNC_RET_SUCC != RN8209C_GetRegValue(RN8209C_REG_SPL_IB,pIb)))
        return FUNC_RET_FAIL;
    return FUNC_RET_SUCC;
}

extern uint32_t GetSPLIBAverage(uint32_t *pIb)
{
		uint8_t 	Cnt;
    uint32_t 	IaTemp[11]={0};
    uint64_t 	Sum;
    
    for(Cnt=0;Cnt<11;Cnt++){
        //TIMER_Delay(TIMER1,700000);		//接近数据更新频率
		WatchDogFeed();
        RN8209C_WaitDataUpdate();
		WatchDogFeed();
        if((pIb!=NULL)&&(FUNC_RET_SUCC != RN8209C_GetRegValue(RN8209C_REG_SPL_IB,&IaTemp[Cnt])))
            return FUNC_RET_FAIL;
		WatchDogFeed();
    }
    Sum = 0;
    Rank(IaTemp,11);
    for(Cnt=0;Cnt<7;Cnt++)
    {
         
         Sum += IaTemp[2];
    }
    if(pIb!=NULL)
        *pIb = (uint32_t)(Sum/7);
		
    return FUNC_RET_SUCC;
}
//频率计算公式：f=CLKIN/8/UFREQ,其中CLKIN=3.579545MHz
extern uint32_t RN8209C_GetFreqHz(float *pHz)
{
    uint32_t 	u32Temp=0;
    float 		fTemp=0.0;
    
		WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_GetRegValue(RN8209C_REG_UFreq,&u32Temp))
        return FUNC_RET_FAIL;
		
    //printf("IARMS:%u\n",u32Temp&0xffffff);
    u32Temp &= 0xffff;
    fTemp 	= (float)u32Temp;
    fTemp 	= 447443.125/fTemp;
		WatchDogFeed();
		
    if(pHz!=NULL)
        *pHz = fTemp;
    
    return FUNC_RET_SUCC;
}

extern uint32_t RN8209C_GetIa(PRN8209C_CALI_PARAM pCali,float *pI)
{
    uint32_t 	u32Temp=0;
    float 		fTemp=0.0;
    
    if(pCali==NULL)
        return FUNC_RET_PARAM_ERR;
		
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_GetRegValue(RN8209C_REG_IARMS,&u32Temp))
        return FUNC_RET_FAIL;
		
    //printf("IARMS:%u\n",u32Temp&0xffffff);
    u32Temp &= 0x7fffff;
    fTemp 	= (float)u32Temp;
    fTemp 	/= 1000000.0;
    fTemp 	*= pCali->Ki;
		
    if(pI!=NULL)
        *pI = fTemp;
		WatchDogFeed();
    
    return FUNC_RET_SUCC;
}

extern uint32_t RN8209C_GetIb(PRN8209C_CALI_PARAM pCali,float *pI)
{
    uint32_t 	u32Temp=0;
    float 		fTemp=0.0;
    
    if(pCali==NULL)
        return FUNC_RET_PARAM_ERR;
		
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_GetRegValue(RN8209C_REG_IBRMS,&u32Temp))
        return FUNC_RET_FAIL;
    //printf("IARMS:%u\n",u32Temp&0xffffff);
		
    u32Temp &= 0x7fffff;
		
    fTemp 	= (float)u32Temp;
    fTemp 	/= 1000000.0;
    fTemp 	*= pCali->Ki;
		
    if(pI!=NULL)
        *pI = fTemp;
		WatchDogFeed();
   
    return FUNC_RET_SUCC;
}

extern uint32_t RN8209C_GetU(PRN8209C_CALI_PARAM pCali,float *pU)
{
    uint32_t 	u32Temp=0;
    float 		fTemp=0.0;
    
    
    if(pCali==NULL)
        return FUNC_RET_PARAM_ERR;
		
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_GetRegValue(RN8209C_REG_URMS,&u32Temp))
        return FUNC_RET_FAIL;
		
    u32Temp &= 0xffffff;
    fTemp 	= (float)u32Temp;
    fTemp 	/= 1000000.0;
    fTemp 	*= pCali->Kv;
		
    if(pU!=NULL)
        *pU = fTemp;
		WatchDogFeed();
    
    return FUNC_RET_SUCC;
}
extern uint32_t RN8209C_GetPa(PRN8209C_CALI_PARAM pCali,float *pP)
{
    int32_t s32Temp=0;
    float fTemp=0.0;
    
    if(pCali==NULL)
        return FUNC_RET_PARAM_ERR;
		
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_GetRegValue(RN8209C_REG_PowerPA,(uint32_t *)&s32Temp))
        return FUNC_RET_FAIL;
		
    fTemp = (float)s32Temp;
    fTemp *= pCali->Kp;
    fTemp /= 1000.0;
		
    if(pP!=NULL)
        *pP = fTemp;
		WatchDogFeed();
    
    return FUNC_RET_SUCC;
}
extern uint32_t RN8209C_GetEa(PRN8209C_CALI_PARAM pCali,double *pE)
{
    uint32_t 	u32Temp=0;
    float 		fTemp=0.0;
    
    if(pCali==NULL)
        return FUNC_RET_PARAM_ERR;
		
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_GetRegValue(RN8209C_REG_EnergyP2,&u32Temp))
        return FUNC_RET_FAIL;
		
    fTemp = (double)u32Temp;
    fTemp /= 3600.0;
		
    if(pE!=NULL)
        *pE = fTemp;
		WatchDogFeed();
    
    return FUNC_RET_SUCC;
}
extern uint32_t RN8209C_GetEa_NoRet(PRN8209C_CALI_PARAM pCali,double *pE)
{
    uint32_t 	u32Temp=0;
    float 		fTemp=0.0;
    
    if(pCali==NULL)
        return FUNC_RET_PARAM_ERR;
		
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_GetRegValue(RN8209C_REG_EnergyP,&u32Temp))
        return FUNC_RET_FAIL;
		
    fTemp = (double)u32Temp;
    fTemp /= 3600.0;
		
    if(pE!=NULL)
        *pE = fTemp;
		WatchDogFeed();
    
    return FUNC_RET_SUCC;
}
extern uint32_t RN8209C_GetPb(PRN8209C_CALI_PARAM pCali,float *pP)
{
    int32_t s32Temp=0;
    float 	fTemp=0.0;
	
    if(pCali==NULL)
        return FUNC_RET_PARAM_ERR;
		
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_GetRegValue(RN8209C_REG_PowerPB,(uint32_t *)&s32Temp))
        return FUNC_RET_FAIL;
		
		WatchDogFeed();
    //printf("IARMS:%u\n",u32Temp&0xffffff);
    fTemp 	= (float)s32Temp;
    fTemp 	/= 1000000.0;
    fTemp 	*= pCali->Kp;
    fTemp 	/= 1000.0;
		
    if(pP!=NULL)
        *pP = fTemp;
    
    return FUNC_RET_SUCC;
}
extern uint32_t RN8209C_GetEb(PRN8209C_CALI_PARAM pCali,double *pE)
{
    uint32_t 	u32Temp=0;
    float 		fTemp=0.0;
    
    if(pCali==NULL)
        return FUNC_RET_PARAM_ERR;
		
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_GetRegValue(RN8209C_REG_EnergyD2,&u32Temp))
        return FUNC_RET_FAIL;
		
    fTemp = (double)u32Temp;
    fTemp /= 360000.0;
		
    if(pE!=NULL)
        *pE = fTemp;
		WatchDogFeed();
    
    return FUNC_RET_SUCC;
}
extern uint32_t RN8209C_GetEb_NoRet(PRN8209C_CALI_PARAM pCali,double *pE)
{
    uint32_t 	u32Temp=0;
    float 		fTemp=0.0;
    
    if(pCali==NULL)
        return FUNC_RET_PARAM_ERR;
		
    WatchDogFeed();
    if(FUNC_RET_SUCC!=RN8209C_GetRegValue(RN8209C_REG_EnergyD,&u32Temp))
        return FUNC_RET_FAIL;
		
		WatchDogFeed();
    fTemp = (double)u32Temp;
    fTemp /= 360000.0;
		
    if(pE!=NULL)
        *pE = fTemp;
    
    return FUNC_RET_SUCC;
}

extern uint32_t RN8209C_DCOffsetCalibration(PRN8209C_DEF pRnDef)
{
    uint32_t 			IARMS1	= 0;
    uint32_t 			IARMS2	= 0;
    uint32_t 			IBRMS1	= 0;
    uint32_t 			IBRMS2	= 0;
    uint32_t 			URMS1		= 0;
    uint32_t 			URMS2		= 0;
    uint8_t 			NeedInitFlag=0;
    PRN8209C_INIT_PARAM pInitParam=NULL;
    
    if(pRnDef==NULL)
        return FUNC_RET_PARAM_ERR;
		
    WatchDogFeed();
    pInitParam = &pRnDef->InitParam;
    RN8209C_Init_ResetParam(pInitParam);
		
		WatchDogFeed();
    if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRnDef))
        return FUNC_RET_FAIL;
		WatchDogFeed();
		
    RN8209C_GetCurrVolAverage(&IARMS1,&IBRMS1,&URMS1);
		WatchDogFeed();
		
    //?IARMS1????BIT[23:8]??DCIAH???,BIT[7:4]??DCL????BIT[3:0]
    //printf("IARMS1=%x\n",IARMS1);
    pInitParam->UNION.PARAM_List.DCIAH 	= (uint16_t)(IARMS1>>8);
    pInitParam->UNION.PARAM_List.DCL 		&= (uint16_t)(~0x000f);
    pInitParam->UNION.PARAM_List.DCL 		|= (uint16_t)((IARMS1>>4)&0xf);
    //?IBRMS1????BIT[23:8]??DCIBH???,BIT[7:4]??DCL????BIT[7:4]
    //printf("IBRMS1=%x\n",IBRMS1);
    pInitParam->UNION.PARAM_List.DCIBH 	= (uint16_t)(IBRMS1>>8);
    pInitParam->UNION.PARAM_List.DCL 		&= (uint16_t)(~0x00f0);
    pInitParam->UNION.PARAM_List.DCL 		|= (uint16_t)((IBRMS1>>4)&0xf)<<4;
    //?URMS1????BIT[23:8]??DCUH???,BIT[7:4]??DCL????BIT[11:8]
    //printf("URMS1=%x\n",URMS1);
    pInitParam->UNION.PARAM_List.DCUH 	= (uint16_t)(URMS1>>8);
    pInitParam->UNION.PARAM_List.DCL 		&= (uint16_t)(~0x0f00);
    pInitParam->UNION.PARAM_List.DCL 		|= (uint16_t)((URMS1>>4)&0xf)<<8;
    
		WatchDogFeed();
    if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRnDef))
        return FUNC_RET_FAIL;
		WatchDogFeed();
		
    //printf("Delay Start\n");
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
    //printf("Delay Stop\n");
    //printf("RN8209C_GetCurrVolAverage --2\n");
		
		WatchDogFeed();
    RN8209C_GetCurrVolAverage(&IARMS2,&IBRMS2,&URMS2);
    //printf("IARMS2=%x\n",IARMS2);
    if(IARMS2>IARMS1)
		{
        IARMS2 = ~IARMS1;
        pInitParam->UNION.PARAM_List.DCIAH = (uint16_t)(IARMS2>>8);
        pInitParam->UNION.PARAM_List.DCL 	&= (uint16_t)(~0x000f);
        pInitParam->UNION.PARAM_List.DCL 	|= (uint16_t)((IARMS2>>4)&0xf);
        NeedInitFlag = 1;
    }
    WatchDogFeed();
    //printf("IBRMS2=%x\n",IBRMS2);
    if(IBRMS2>IBRMS1)
		{
        IBRMS2 = ~IBRMS1;
        pInitParam->UNION.PARAM_List.DCIBH = (uint16_t)(IBRMS2>>8);
        pInitParam->UNION.PARAM_List.DCL 	&= (uint16_t)(~0x00f0);
        pInitParam->UNION.PARAM_List.DCL 	|= (uint16_t)((IBRMS2>>4)&0xf)<<4;
        NeedInitFlag = 1;
    }
    
    //printf("URMS2=%x\n",URMS2);
    if(URMS2>URMS1)
		{
        URMS2 = ~URMS1;
        pInitParam->UNION.PARAM_List.DCIBH = (uint16_t)(URMS2>>8);
        pInitParam->UNION.PARAM_List.DCL 	&= (uint16_t)(~0x00f0);
        pInitParam->UNION.PARAM_List.DCL 	|= (uint16_t)((URMS2>>4)&0xf)<<4;
        NeedInitFlag = 1;
    }
		
		WatchDogFeed();
    if(1==NeedInitFlag)
		{
        if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRnDef))
            return FUNC_RET_FAIL;
    }
		
		WatchDogFeed();
    //printf("Delay Start\n");
    
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
		
    //printf("Delay Stop\n");
    RN8209C_GetCurrVolAverage(&IARMS1,&IBRMS1,NULL);
		WatchDogFeed();
		
    //printf("IARMS1=%x\n",IARMS1);
    IARMS1 	= IARMS1*IARMS1;
    IARMS1	>>=8;
    IARMS1 	= ~IARMS1;
    pInitParam->UNION.PARAM_List.IARMSOS = IARMS1&0xffff;
    //printf("IBRMS1=%x\n",IBRMS1);
    IBRMS1 	= IBRMS1*IBRMS1;
    IBRMS1	>>=8;
    IBRMS1 	= ~IBRMS1;
    pInitParam->UNION.PARAM_List.IBRMSOS = IBRMS1&0xffff;
		
		WatchDogFeed();
    if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRnDef))
        return FUNC_RET_FAIL;
    WatchDogFeed();
		
    return FUNC_RET_SUCC;
}
static uint32_t RN8209C_GetPGAIN(int32_t P,RN8209C_CALI_PARAM *pCali,uint16_t *pPGAIN)
{
		float ftemp;
		float ERR=0.0;

		if(pCali==NULL)
				return FUNC_RET_PARAM_ERR;
		
		ftemp 	= (float)P;
		ftemp 	/= 1000000.0;
		ftemp 	*= pCali->Kp;
		ftemp 	/= (pCali->In*pCali->Un);
		
		ERR = ftemp - 1.0;
		
		ftemp = -ERR/(1.0+ERR);			//PGAIN
		
		if(ftemp>0.0001)
				*pPGAIN = (uint16_t)(ftemp*32768);
		else
				*pPGAIN = (uint16_t)(65536+ftemp*0x8000);
		
		return FUNC_RET_SUCC;
}
//直流增益校准
extern uint32_t RN8209C_DCGainCalibration(PRN8209C_DEF pRn8209c)
{
    RN8209C_CALI_PARAM *pCali=NULL;
    RN8209C_INIT_PARAM *pInit=NULL;
    uint32_t 		Ia=0,Ib=0,U=0;
		int32_t 		Pa=0,Pb=0;
    float 			ftemp;
    uint16_t 		u16Temp=0;
	
    if(pRn8209c==NULL)
        return FUNC_RET_PARAM_ERR;
		
    pCali = &pRn8209c->CaliParam;
    pInit = &pRn8209c->InitParam;
    
    pInit->UNION.PARAM_List.IBGain 	= 0;
    pInit->UNION.PARAM_List.GPQA 		= 0;
    pInit->UNION.PARAM_List.GPQB 		= 0;
    WatchDogFeed();
		
    if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
		
    //printf("RN8209C_GetCurrVolAverage\n");
    if(FUNC_RET_SUCC != RN8209C_GetCurrVolAverage(&Ia,&Ib,NULL))
        return FUNC_RET_FAIL;
		
		//printf("IARMS1=%x\n",Ia);
    //printf("IBRMS2=%x\n",Ib);
    WatchDogFeed();
		ftemp 	= (float)Ia;
		ftemp 	/= (float)Ib;
    //printf("%u ftemp=%f\n",__LINE__,ftemp);
		if(ftemp>1.0)
		{
				ftemp -= 1.0;
				ftemp *= 32768.0;
				//printf("%u ftemp=%f\n",__LINE__,ftemp);
		}
		else
		{
				ftemp 	-= 1.0;
				ftemp 	*= 32768.0;
				ftemp 	+= 65536.0;
        //printf("%u ftemp=%f\n",__LINE__,ftemp);
		}
		pInit->UNION.PARAM_List.IBGain = (uint16_t)ftemp;
    //printf("IBGain=%x\n",pInit->UNION.PARAM_List.IBGain);
		
    WatchDogFeed();
		if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		
		WatchDogFeed();
		DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
		
    RN8209C_WaitDataUpdate();
		WatchDogFeed();
    if(FUNC_RET_SUCC != RN8209C_GetCurrVolAverage(&Ia,&Ib,&U))
        return FUNC_RET_FAIL;
		WatchDogFeed();
		
    //printf("Ia=%x Ib=%x U=%x\n",Ia,Ib,U);
		//比例系数都扩大了1000000倍
    Ia &= 0x7fffff;
    ftemp 	= (float)Ia;
    ftemp 	/= 1000000.0;
    pCali->Ki = pCali->In/ftemp;
    //printf("Ki=%f\n",pCali->Ki);
    U &= 0x7fffff;
    ftemp 	= (float)U;
    ftemp 	/= 1000000.0;
    pCali->Kv = pCali->Un/ftemp;
    //printf("Kv=%f\n",pCali->Kv);
    pCali->Kp= 0.19180;
		
		WatchDogFeed();
		if(FUNC_RET_SUCC != RN8209C_GetPowerAverage(&Pa,&Pb))
        return FUNC_RET_FAIL;
    //printf("Pa=%d Pb=%d\n",Pa,Pb);
		
		WatchDogFeed();
		if(FUNC_RET_SUCC != RN8209C_GetPGAIN(Pa,pCali,&u16Temp))
        return FUNC_RET_FAIL;
    WatchDogFeed();
		pInit->UNION.PARAM_List.GPQA = u16Temp;
		//printf("GPQA=%x\n",u16Temp);
		
    if(FUNC_RET_SUCC != RN8209C_GetPGAIN(Pb,pCali,&u16Temp))
        return FUNC_RET_FAIL;
		pInit->UNION.PARAM_List.GPQB = u16Temp;
		//printf("GPQB=%x\n",u16Temp);
		
		WatchDogFeed();
    if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		WatchDogFeed();
		
    return FUNC_RET_SUCC;
}


//A通道增益校正：功率源220V，15A，PF=1，计算有功功率误差
//PGAIN=-ERR/( 1+ERR)
// PGAIN>0,GPQA=PGAIN*2^15;
// PGAIN<0,GPQA=PGAIN*2^15+2^16;
extern uint32_t RN8209C_ACGainCalibration(PRN8209C_DEF pRn8209c)
{
    RN8209C_CALI_PARAM *pCali=NULL;
    RN8209C_INIT_PARAM *pInit=NULL;
    uint32_t 						Ia=0,U=0;
		int32_t 						Pa=0;
    volatile float 			ftemp,err,Pgain;
    volatile uint16_t 	u16Temp=0;
	
    if(pRn8209c==NULL)
        return FUNC_RET_PARAM_ERR;
		
    pCali = &pRn8209c->CaliParam;
    pInit = &pRn8209c->InitParam;
		
		WatchDogFeed();
    RN8209C_Init_ResetParam(pInit);
    WatchDogFeed();
		
    if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
		
    //printf("RN8209C_GetCurrVolAverage\n");
    if(FUNC_RET_SUCC != RN8209C_GetCurrVolAverage(&Ia,NULL,&U))
        return FUNC_RET_FAIL;
		WatchDogFeed();
		
    if(FUNC_RET_SUCC != RN8209C_GetPowerAverage(&Pa,NULL))
        return FUNC_RET_FAIL;
		WatchDogFeed();
		
    //printf("Ia=%x(%d) ",Ia,Ia);
    //printf("U=%x(%d) ",U,U);
    //printf("Pa=%x(%d) \n",Pa,Pa);
		
    //比例系数都扩大了1000000倍
    Ia 			&= 0x7fffff;
    ftemp 	= (float)Ia;
    ftemp 	/= 1000000.0;
    pCali->Ki = pCali->In/ftemp;
    //printf("Ki=%f\n",pCali->Ki);
    U 			&= 0x7fffff;
    ftemp 	= (float)U;
    ftemp 	/= 1000000.0;
    pCali->Kv = pCali->Un/ftemp;
    //printf("Kv=%f\n",pCali->Kv);
    pCali->Kp= 0.00001781608;
		
    ftemp 	= (float)Pa;
    ftemp 	*= pCali->Kp;
    //printf("ftemp=%f\n",ftemp);
    err = (pCali->Un*pCali->In)/ftemp-1.0;
    if(err>0.00001)
		{
        u16Temp = (uint16_t)(err*32768);
        pInit->UNION.PARAM_List.GPQA = u16Temp;
        //printf("GPQA=%x\n",u16Temp);
    }
		else
		{
        err *= 32768.0;
        err += 65536.0;
				u16Temp = (uint16_t)err;
        pInit->UNION.PARAM_List.GPQA = u16Temp;
        //printf("GPQA=%x\n",u16Temp);
    }
    
		WatchDogFeed();
		if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		WatchDogFeed();
		
    return FUNC_RET_SUCC;
}
//A通道相位校正：功率源220V，15A，PF=0.5，计算有功功率误差
//相位= [ArcSin (-ERR/1.732) ]/0.02 ; 
extern uint32_t RN8209C_ACPhaseCalibration(PRN8209C_DEF pRn8209c)
{
    RN8209C_CALI_PARAM *pCali=NULL;
    RN8209C_INIT_PARAM *pInit=NULL;
		int32_t Pa=0;
    float 	ftemp,err;
	
    if(pRn8209c==NULL)
        return FUNC_RET_PARAM_ERR;
		
    pCali = &pRn8209c->CaliParam;
    pInit = &pRn8209c->InitParam;

    //printf("RN8209C_ACPhaseCalibration\n");
    
    pInit->UNION.PARAM_List.PhsA = 0;
    WatchDogFeed();
    if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		
    WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
    WatchDogFeed();
		
    if(FUNC_RET_SUCC != RN8209C_GetPowerAverage(&Pa,NULL))
        return FUNC_RET_FAIL;
		
    //printf("Pa=%x(%d) \n",Pa,Pa);
		WatchDogFeed();
    ftemp 	= (float)Pa;
    ftemp 	*= pCali->Kp;
    //printf("ftemp=%f\n",ftemp);
    err 		= ftemp/(pCali->Un*pCali->In)-1.0;
    ftemp 	= (float)asin((-err/(double)1.732));
    ftemp 	/= 0.02;
    if(ftemp>0.00001)
		{
        pInit->UNION.PARAM_List.PhsA = (uint16_t)ftemp;
    }
		else
		{
        pInit->UNION.PARAM_List.PhsA = (uint16_t)(ftemp+256.0);
    }
		
		WatchDogFeed();
		if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		
    return FUNC_RET_SUCC;
}

//A通道有功OFFSET校正：功率源220V，0A，PF=1
extern uint32_t RN8209C_ACOffsetCalibration(PRN8209C_DEF pRn8209c)
{
    RN8209C_INIT_PARAM *pInit=NULL;
		int32_t Pa=0;
    
	
    if(pRn8209c==NULL)
        return FUNC_RET_PARAM_ERR;
		
    pInit = &pRn8209c->InitParam;

    //printf("RN8209C_ACOffsetCalibration\n");
    
    pInit->UNION.PARAM_List.APOSA = 0;
    WatchDogFeed();
    if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		
    WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
    WatchDogFeed();
		
    if(FUNC_RET_SUCC != RN8209C_GetPowerAverage(&Pa,NULL))
        return FUNC_RET_FAIL;
		
    //printf("Pa=%x(%d) \n",Pa,Pa);
    if(Pa&0x80000000)
        Pa = ~Pa;
    pInit->UNION.PARAM_List.APOSA = (uint16_t)(Pa&0xffff);
    WatchDogFeed();
		
		if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		
    return FUNC_RET_SUCC;
}
//A通道无功校正：功率源220V，15A，PF=0.5
extern uint32_t RN8209C_ACReactiveCalibration(PRN8209C_DEF pRn8209c)
{
    RN8209C_CALI_PARAM *pCali=NULL;
    RN8209C_INIT_PARAM *pInit=NULL;
		int32_t 	Pa=0;
    float 		ftemp,err;
    uint16_t 	u16Temp=0;
	
    if(pRn8209c==NULL)
        return FUNC_RET_PARAM_ERR;
		
    pCali = &pRn8209c->CaliParam;
    pInit = &pRn8209c->InitParam;

    //printf("RN8209C_ACReactiveCalibration\n");
    //无功相位补偿
    pInit->UNION.PARAM_List.QPhsCal = 0;
    WatchDogFeed();
    if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		
    WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
    WatchDogFeed();
		
    if(FUNC_RET_SUCC != RN8209C_GetPowerAverage(&Pa,NULL))
        return FUNC_RET_FAIL;
		
    //printf("Pa=%x(%d) \n",Pa,Pa);
    WatchDogFeed();
    ftemp 	= (float)Pa;
    ftemp 	*= pCali->Kp;
    err 		= ftemp - pCali->Un*pCali->In;
    err 		*= 0.5774;
    if(err>0.00001)
		{
        u16Temp = (uint16_t)(err*32768);
    }
		else
		{
        ftemp 	= err*32768.0;
        ftemp 	+= 65536.0;
        u16Temp = (uint16_t)ftemp;
    }
    pInit->UNION.PARAM_List.QPhsCal = u16Temp;
		
    WatchDogFeed();
		if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		
    return FUNC_RET_SUCC;
}
//电流有效值校正：功率源220V，0A，PF=1.0
extern uint32_t RN8209C_ACCurrentOffsetCalibration(PRN8209C_DEF pRn8209c)
{
    RN8209C_INIT_PARAM *pInit=NULL;
		uint32_t 	Ia=0;
	
    if(pRn8209c==NULL)
        return FUNC_RET_PARAM_ERR;

    pInit = &pRn8209c->InitParam;
    pInit->UNION.PARAM_List.IARMSOS = 0;
		
    WatchDogFeed();
    if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		
    WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
		WatchDogFeed();
    DelayUs(500000);
    WatchDogFeed();
		
    if(FUNC_RET_SUCC != RN8209C_GetCurrVolAverage(&Ia,NULL,NULL))
        return FUNC_RET_FAIL;
		
    //printf("Ia=%x(%d) \n",Ia,Ia);
    WatchDogFeed();
    Ia = Ia*Ia;
    Ia = ~Ia;
    Ia>>=8;
   
    pInit->UNION.PARAM_List.IARMSOS = Ia&0xffff;
		
    WatchDogFeed();
		if(FUNC_RET_SUCC != RN8209C_Init_Chip(pRn8209c))
        return FUNC_RET_FAIL;
		
    return FUNC_RET_SUCC;
}
//定时检查下电能芯片的参数是否被篡改
extern uint32_t RN8209C_RegularCheckParam(PRN8209C pRn8209)
{
	uint8_t i;

	if(pRn8209==NULL)
        return FUNC_RET_PARAM_ERR;
	
	for(i=0;i<3;i++){
        WatchDogFeed();
		UART_CS(i+1);
		DelayUs(10);
		if(pRn8209->Def[i].InitParam.UNION.PARAM_List.CHKSUM != RN8209C_GetCheckSum()){
			RN8209C_Init_Chip(&pRn8209->Def[i]);
			debug("Find RN8209C Chip%d CheckSum Error,Reinit it\n",i+1);
		}
		UART_CS(0);
	}
	return FUNC_RET_SUCC;
}




