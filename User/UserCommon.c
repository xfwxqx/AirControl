#include "UserCommon.h"
#include "Ydt1363.h"
#include "NUC029xAN.h"
#include "AT45DBXXX.h"
#include "RN8209C.h"
#include "NRX8010.h"
#include "Usart.h"

#define FUNC_SUCCESS 0
#define FUNC_FAILURE !FUNC_SUCCESS
#define SUCCESS 0
#define FAILURE !SUCCESS


extern void DelayUs(uint32_t time)
{
		uint32_t i=0;
	
		for(i=0;i<time*10;i++)
		;
}
extern uint8_t GetSum(uint8_t *buf,uint8_t cnt)
{
    uint8_t i,sum=0;
    
    for(i=0;i<cnt;i++)
        sum += buf[i];
    
    return sum;
}
extern void Rank(uint32_t average[],int n)
{
		uint8_t m = 0;
		uint8_t j = 0;
		uint32_t average_temp = 0;
		//average[0] = 0;										//数组第一个数据赋值为0（冒泡排序需要）
	
		for(m = 0;m < n - 1;m++)                           //冒泡排序
		{
				for(j = 0;j < n - m - 1;j++)
				{
						if(average[j+1] < average[j])
						{
								average_temp 	= average[j];
								average[j] 		= average[j+1];
								average[j+1] 	= average_temp;
						}
				}
		}
}
//输入：pData为NULL时，仅检查数据有效性
//0 ：成功 2:数据错误 3:输入参数错误
static uint32_t MeterData_GetValidDataFromAT24CXX(uint8_t *pData,uint32_t StartAddr,uint16_t ReadBytes)
{
    uint8_t CheckSum;
    uint8_t TempBuf[151]={0};
    
    if(ReadBytes>150)
        return 3;
    WatchDogFeed();
    AT45DBxxxD_Read(TempBuf,StartAddr,(uint32_t)ReadBytes+1);
    CheckSum = TempBuf[ReadBytes];
		if(CheckSum == GetSum(TempBuf,ReadBytes))
		{
				if(pData!=NULL)
						memcpy(pData,TempBuf,ReadBytes);
				return 0;
		}
		WatchDogFeed();
		
		return 2;     
}

//Input: IsReadBackup为1时,读取备份区，0时不读取备份区
//0 ：成功  2:失败 3:输入参数错误
extern uint32_t MeterData_WaitReadSucc(uint8_t IsReadBackup,uint8_t *pData,uint32_t StartAddr,uint16_t ReadBytes)
{
    uint8_t i=0;
    uint32_t ret;
    uint32_t StartAddrTemp;
    
    
    if(pData==NULL)
        return 3;
    if(0 != AT45DBXXX_Check())
        return 3;

    StartAddrTemp = StartAddr;
    
    //超时获取数据
    i=0;
    do
		{
        i++;
				WatchDogFeed();
        ret = MeterData_GetValidDataFromAT24CXX(pData,StartAddrTemp,ReadBytes);
				WatchDogFeed();
    }while((ret==2)&&(i<3));
    
    if((ret==2)&&(IsReadBackup))
		{
        //获取失败，则从备份区获取
        StartAddrTemp = StartAddr + FLASH_ADDR_BACKUP_OFFSET;
        i=0;
        do
				{
            i++;
						WatchDogFeed();
            ret = MeterData_GetValidDataFromAT24CXX(pData,StartAddrTemp,ReadBytes);
						WatchDogFeed();
        }while((i<3)&&(ret==2));
        if(ret==2)
            return 2;
        return 0; 
    }
		else if(ret==2)
    {
				return 2;
		}
		
    return 0; 
}
//0:成功 1：参数错误 2：写数据失败 
static uint32_t MeterData_SetValidDataToAT24CXX(uint8_t *pData,uint32_t StartAddr,uint16_t WriteBytes)
{
    uint8_t Sum=0;
    uint8_t TempBuf[151]={0};
    
    if(pData==NULL)
        return 1;
    
    if(WriteBytes>150)
        return 1;
    WatchDogFeed();
    memcpy(TempBuf,pData,WriteBytes);
    Sum = GetSum(TempBuf,WriteBytes);
    TempBuf[WriteBytes] = Sum;
    //printf("MeterData_SetValidDataToAT24CXX StartAddr=%x WriteBytes=%x\n",StartAddr,WriteBytes);
    return AT45DBxxxD_Write(TempBuf,StartAddr,(uint32_t)WriteBytes+1);
}
//0:成功 1：参数错误 2：写数据失败 
static uint32_t MeterData_SetAndCheckToAT24CXX(uint8_t *pData,uint32_t StartAddr,uint16_t WriteBytes)
{
    uint8_t i=0;
    uint32_t ret=0;
    
    if(pData==NULL)
        return 1;
    
    i=0;
    do
		{
        i++;
				WatchDogFeed();
        ret = MeterData_SetValidDataToAT24CXX(pData,StartAddr,WriteBytes);
        if(ret == 0)
				{
						WatchDogFeed();
						ret = MeterData_GetValidDataFromAT24CXX(NULL,StartAddr,WriteBytes);
        }
    }while((i<3)&&(ret!=0));
    WatchDogFeed();
		
    if(ret!=0)
       return 2;
		
    return 0;
}

//Input: IsWriteBackup为1时,读取备份区，0时不读取备份区
//0:成功 1：参数错误 2：写数据失败 3：写备份数据失败
extern uint32_t MeterData_WaitWriteSucc(uint8_t IsWriteBackup,uint8_t *pData,uint32_t StartAddr,uint16_t WriteBytes)
{
    uint32_t ret=0;
    uint32_t StartAddrTemp;
    
    if(pData==NULL)
        return 1;
		
    //首先检查AT24CXX是否存在
    if(0 != AT45DBXXX_Check())
        return 1;
		
    WatchDogFeed();
    StartAddrTemp = StartAddr;
    //超时等待写成功
    ret = MeterData_SetAndCheckToAT24CXX(pData,StartAddrTemp,WriteBytes);
    if(ret != 0)
       return 2;
		
    WatchDogFeed();
    if(IsWriteBackup)
		{
        StartAddrTemp = StartAddr+FLASH_ADDR_BACKUP_OFFSET;
        //超时等待写备份数据成功
        ret = MeterData_SetAndCheckToAT24CXX(pData,StartAddrTemp,WriteBytes);
        if(ret != 0)
            return 3;
    }
		WatchDogFeed();
		
    return 0;
}

extern uint32_t WriteDataFlash(uint32_t StartAddr,__packed uint32_t *pData,uint8_t WriteCnt)
{
    uint8_t i;
    uint32_t Data,Addr;
    
    if(pData==NULL)
        return 2;
    
		WatchDogFeed();
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    for(i=0;i<WriteCnt;i++)
		{
        Addr = StartAddr+4*i;
				WatchDogFeed();
        FMC_Write(Addr,pData[i]);
				WatchDogFeed();
        Data = FMC_Read(Addr);
        //printf("Read Data:0x%x From Addr:0x%x\n",Data,Addr);
        if(Data!=pData[i])
            return 1;
    }
    FMC_DISABLE_AP_UPDATE();
    FMC_Close();
    SYS_LockReg();
		WatchDogFeed();
		
    return 0;
}
extern uint32_t ReadDataFlash(uint32_t StartAddr,__packed uint32_t *pData,uint8_t ReadCnt)
{
    uint8_t i;
    uint32_t Addr;
    
    if(pData==NULL)
        return 2;
		
    WatchDogFeed();
    SYS_UnlockReg();
    FMC_Open();
    for(i=0;i<ReadCnt;i++)
		{
        Addr = StartAddr+4*i;
				WatchDogFeed();
        pData[i] = FMC_Read(Addr);
    }
    FMC_Close();
    SYS_LockReg();
		WatchDogFeed();
		
    return 0;
}  
extern uint32_t EraseDataFlash_Page(uint32_t StartAddr)
{
    SYS_UnlockReg();
    FMC_Open();
		WatchDogFeed();
    FMC_Erase(StartAddr);
		WatchDogFeed();
    FMC_Close();
    SYS_LockReg();
	
    return 0;
}  

extern uint32_t RN8209C_Calibration_GN_ALL(PRN8209C_DEF pRn8209c)
{
    uint32_t i,ret=0;
   
    for(i=0;i<3;i++)
		{
        UART_CS(i+1);
        DelayUs(10);
				WatchDogFeed();
        ret += RN8209C_DCGainCalibration(&pRn8209c[i]);
				WatchDogFeed();
        UART_CS(0);					//拉高所有片选
        DelayUs(10);
    }
		
    return ret;
}
//将Rn8209校准后数据写入DataFlash(起始地址：0x1f000-0x1fffff)
extern uint32_t WriteRn8209CalidataToFlash(PRN8209C pRn8209c)
{
    uint32_t DWCnt;
    uint32_t StartAddr = FMC_ReadDataFlashBaseAddr();
    
		WatchDogFeed();
    EraseDataFlash_Page(StartAddr);
    DWCnt = sizeof(RN8209C_DEF)/4;
    DWCnt *= 3;
		WatchDogFeed();
	
    return WriteDataFlash(StartAddr,(__packed uint32_t *)pRn8209c,(uint8_t)DWCnt);
}
//从DataFlash(起始地址：0x1f000-0x1fffff)读出Rn8209校准数据
extern uint32_t ReadRn8209CalidataFromFlash(PRN8209C pRn8209c)
{
    uint32_t DWCnt;
    uint32_t StartAddr = FMC_ReadDataFlashBaseAddr();
   
    DWCnt = sizeof(RN8209C_DEF)/4;
    DWCnt *= 3;
		WatchDogFeed();
	
    return ReadDataFlash(StartAddr,(__packed uint32_t *)pRn8209c,(uint8_t)DWCnt);
}

extern uint32_t GetRealTimeData(PCOMM_DATA pData)
{
    uint8_t 						i,ret;
    float 							fTemp=0.0;
    double  						dfTemp=0.0;
    PM5KT_COMM_TM  			TmBuf;
    PYDT1363_DATA   		pYdtData=NULL;
    PRN8209C_CALI_PARAM pRn8209Cali=NULL;
	
    if(pData==NULL)
        return FUNC_FAILURE;
										
		WatchDogFeed();
    memset(&TmBuf,0,sizeof(TmBuf));
    if(RX8010_GetTime(&TmBuf) == 1)
		{
				pYdtData = pData->YdtData;
				//pYdtData->Date.Year 	= 1970+TmBuf.Year;
                pYdtData->Date.Year 	= TmBuf.Year;
				pYdtData->Date.Month	= TmBuf.Month;
				pYdtData->Date.Day 		= TmBuf.Day;
				pYdtData->Date.Hour		= TmBuf.Hour;
				pYdtData->Date.Minute	= TmBuf.Minute;
				pYdtData->Date.Second	= TmBuf.Second;
		}
		else if(RX8010_GetTime(&TmBuf) == 1)
		{
				pYdtData = pData->YdtData;
				//pYdtData->Date.Year 	= 1970+TmBuf.Year;
                pYdtData->Date.Year 	= TmBuf.Year;
				pYdtData->Date.Month	= TmBuf.Month;
				pYdtData->Date.Day 		= TmBuf.Day;
				pYdtData->Date.Hour		= TmBuf.Hour;
				pYdtData->Date.Minute	= TmBuf.Minute;
				pYdtData->Date.Second	= TmBuf.Second;
		}

    for(i=0;i<3;i++)
		{
        pRn8209Cali = &pData->Rn8209c.Def[i].CaliParam;
        WatchDogFeed();
        UART_CS(i+1);
        DelayUs(10);
        if(RN8209C_TestDataUpdate())
				{
						WatchDogFeed();
            ret += RN8209C_GetIa(pRn8209Cali,&fTemp);
            pData->RN8209Data[i].Current 	= fTemp;
						WatchDogFeed();
            ret += RN8209C_GetU(pRn8209Cali,&fTemp);
            pData->RN8209Data[i].Voltage 	= fTemp;
						WatchDogFeed();
            ret += RN8209C_GetPa(pRn8209Cali,&fTemp);
            pData->RN8209Data[i].PowerP 	= fTemp;
						WatchDogFeed();
            ret += RN8209C_GetEa(pRn8209Cali,&dfTemp);
            pData->RN8209Data[i].EnergyP += dfTemp;
						WatchDogFeed();
            ret += RN8209C_GetFreqHz(&fTemp);
            pData->RN8209Data[i].Freq 		= fTemp;
        }
        UART_CS(0);
        WatchDogFeed();
    }
	
    dfTemp = 0.0;
    for(i=0;i<3;i++)
        dfTemp += pData->RN8209Data[i].EnergyP;
		
    pData->NowAllEnergyP 									= (float)dfTemp;
    pData->YdtData->Egy.TodayEgy 					= pData->NowAllEnergyP;
		pData->YdtData->Analog.CurrentPhaseA 	= pData->RN8209Data[0].Current;
		pData->YdtData->Analog.CurrentPhaseB 	= pData->RN8209Data[1].Current;
		pData->YdtData->Analog.CurrentPhaseC 	= pData->RN8209Data[2].Current;
		pData->YdtData->Analog.VoltagePhaseA 	= pData->RN8209Data[0].Voltage;
		pData->YdtData->Analog.VoltagePhaseB 	= pData->RN8209Data[1].Voltage;
		pData->YdtData->Analog.VoltagePhaseC 	= pData->RN8209Data[2].Voltage;
		
    return ret;
}
static uint32_t GetPT1000_Temp(float R,__packed float *pT)
{
		float R0=0.0;

		if(pT==NULL)
			return FUNC_FAILURE;
		R0 = 2.0*R;
		if((R0>=901.923)&&(R0<=905.861)){
			*pT = -25.0;
			*pT += (R0-901.923)/(905.861-901.923);
		}else if((R0>905.861)&&(R0<=909.798)){
			*pT = -24.0;
			*pT += (R0-905.861)/(909.798-905.861);
		}else if((R0>909.798)&&(R0<=913.733)){
			*pT = -23.0;
			*pT += (R0-909.798)/(913.733-909.798);
		}else if((R0>913.733)&&(R0<=917.666)){
			*pT = -22.0;
			*pT += (R0-913.733)/(917.666-913.733);
		}else if((R0>917.666)&&(R0<=921.599)){
			*pT = -21.0;
			*pT += (R0-917.666)/(921.599-917.666);
		}else if((R0>921.599)&&(R0<=925.531)){
			*pT = -20.0;
			*pT += (R0-921.599)/(925.531-921.599);
		}else if((R0>925.531)&&(R0<=929.46)){
			*pT = -19.0;
			*pT += (R0-925.531)/(929.46-925.531);
		}else if((R0>929.46)&&(R0<=933.39)){
			*pT = -18.0;
			*pT += (R0-929.46)/(933.39-929.46);
		}else if((R0>933.39)&&(R0<=937.317)){
			*pT = -17.0;
			*pT += (R0-933.39)/(937.317-933.39);
		}else if((R0>937.317)&&(R0<=941.244)){
			*pT = -16.0;
			*pT += (R0-937.317)/(941.244-937.317);
		}else if((R0>941.244)&&(R0<=945.17)){
			*pT = -15.0;
			*pT += (R0-941.244)/(945.17-941.244);
		}else if((R0>945.17)&&(R0<=949.094)){
			*pT = -14.0;
			*pT += (R0-945.17)/(949.094-945.17);
		}else if((R0>949.094)&&(R0<=953.016)){
			*pT = -13.0;
			*pT += (R0-949.094)/(953.016-949.094);
		}else if((R0>953.016)&&(R0<=956.938)){
			*pT = -12.0;
			*pT += (R0-953.016)/(956.938-953.016);
		}else if((R0>956.938)&&(R0<=960.859)){
			*pT = -11.0;
			*pT += (R0-956.938)/(960.859-956.938);
		}else if((R0>960.859)&&(R0<=964.779)){
			*pT = -10.0;
			*pT += (R0-960.859)/(964.779-960.859);
		}else if((R0>964.779)&&(R0<=968.697)){
			*pT = -9.0;
			*pT += (R0-964.779)/(968.697-964.779);
		}else if((R0>968.697)&&(R0<=972.614)){
			*pT = -8.0;
			*pT += (R0-968.697)/(972.614-968.697);
		}else if((R0>972.614)&&(R0<=976.529)){
			*pT = -7.0;
			*pT += (R0-972.614)/(976.529-972.614);
		}else if((R0>976.529)&&(R0<=980.444)){
			*pT = -6.0;
			*pT += (R0-976.529)/(980.444-976.529);
		}else if((R0>980.444)&&(R0<=984.358)){
			*pT = -5.0;
			*pT += (R0-980.444)/(984.358-980.444);
		}else if((R0>984.358)&&(R0<=988.27)){
			*pT = -4.0;
			*pT += (R0-984.358)/(988.27-984.358);
		}else if((R0>988.27)&&(R0<=992.181)){
			*pT = -3.0;
			*pT += (R0-988.27)/(992.181-988.27);
		}else if((R0>992.181)&&(R0<=996.091)){
			*pT = -2.0;
			*pT += (R0-992.181)/(996.091-992.181);
		}else if((R0>996.091)&&(R0<=1000.0)){
			*pT = -1.0;
			*pT += (R0-996.091)/(1000.0-996.091);
		}else if((R0>1000.0)&&(R0<=1003.908)){
			*pT = 0.0;
			*pT += (R0-1000.0)/(1003.908-1000.0);
		}else if((R0>1003.908)&&(R0<=1007.814)){
			*pT = 1.0;
			*pT += (R0-1003.908)/(1007.814-1003.908);
		}else if((R0>1007.814)&&(R0<=1011.72)){
			*pT = 2.0;
			*pT += (R0-1007.814)/(1011.72-1007.814);
		}else if((R0>1011.72)&&(R0<=1015.624)){
			*pT = 3.0;
			*pT += (R0-1011.72)/(1015.624-1011.72);
		}else if((R0>1015.624)&&(R0<=1019.527)){
			*pT = 4.0;
			*pT += (R0-1015.624)/(1019.527-1015.624);
		}else if((R0>1019.527)&&(R0<=1023.429)){
			*pT = 5.0;
			*pT += (R0-1019.527)/(1023.429-1019.527);
		}else if((R0>1023.429)&&(R0<=1027.33)){
			*pT = 6.0;
			*pT += (R0-1023.429)/(1027.33-1023.429);
		}else if((R0>1027.33)&&(R0<=1031.229)){
			*pT = 7.0;
			*pT += (R0-1027.33)/(1031.229-1027.33);
		}else if((R0>1031.229)&&(R0<=1035.128)){
			*pT = 8.0;
			*pT += (R0-1031.229)/(1035.128-1031.229);
		}else if((R0>1035.128)&&(R0<=1039.025)){
			*pT = 9.0;
			*pT += (R0-1035.128)/(1039.025-1035.128);
		}else if((R0>1039.025)&&(R0<=1042.921)){
			*pT = 10.0;
			*pT += (R0-1039.025)/(1042.921-1039.025);
		}else if((R0>1042.921)&&(R0<=1046.816)){
			*pT = 11.0;
			*pT += (R0-1042.921)/(1046.816-1042.921);
		}else if((R0>1046.816)&&(R0<=1050.71)){
			*pT = 12.0;
			*pT += (R0-1046.816)/(1050.71-1046.816);
		}else if((R0>1050.71)&&(R0<=1054.603)){
			*pT = 13.0;
			*pT += (R0-1050.71)/(1054.603-1050.71);
		}else if((R0>1054.603)&&(R0<=1058.495)){
			*pT = 14.0;
			*pT += (R0-1054.603)/(1058.495-1054.603);
		}else if((R0>1058.495)&&(R0<=1062.385)){
			*pT = 15.0;
			*pT += (R0-1058.495)/(1062.385-1058.495);
		}else if((R0>1062.385)&&(R0<=1066.274)){
			*pT = 16.0;
			*pT += (R0-1062.385)/(1066.274-1062.385);
		}else if((R0>1066.274)&&(R0<=1070.162)){
			*pT = 17.0;
			*pT += (R0-1066.274)/(1070.162-1066.274);
		}else if((R0>1070.162)&&(R0<=1074.049)){
			*pT = 18.0;
			*pT += (R0-1070.162)/(1074.049-1070.162);
		}else if((R0>1074.049)&&(R0<=1077.935)){
			*pT = 19.0;
			*pT += (R0-1074.049)/(1077.935-1074.049);
		}else if((R0>1077.935)&&(R0<=1081.82)){
			*pT = 20.0;
			*pT += (R0-1077.935)/(1081.82-1077.935);
		}else if((R0>1081.82)&&(R0<=1085.703)){
			*pT = 21.0;
			*pT += (R0-1081.82)/(1085.703-1081.82);
		}else if((R0>1085.703)&&(R0<=1089.585)){
			*pT = 22.0;
			*pT += (R0-1085.703)/(1089.585-1085.703);
		}else if((R0>1089.585)&&(R0<=1093.467)){
			*pT = 23.0;
			*pT += (R0-1089.585)/(1093.467-1089.585);
		}else if((R0>1093.467)&&(R0<=1097.347)){
			*pT = 24.0;
			*pT += (R0-1093.467)/(1097.347-1093.467);
		}else if((R0>1097.347)&&(R0<=1101.225)){
			*pT = 25.0;
			*pT += (R0-1097.347)/(1101.225-1097.347);
		}else if((R0>1101.225)&&(R0<=1105.103)){
			*pT = 26.0;
			*pT += (R0-1101.225)/(1105.103-1101.225);
		}else if((R0>1105.103)&&(R0<=1108.98)){
			*pT = 27.0;
			*pT += (R0-1105.103)/(1108.98-1105.103);
		}else if((R0>1108.98)&&(R0<=1112.855)){
			*pT = 28.0;
			*pT += (R0-1108.98)/(1112.855-1108.98);
		}else if((R0>1112.855)&&(R0<=1116.729)){
			*pT = 29.0;
			*pT += (R0-1112.855)/(1116.729-1112.855);
		}else if((R0>1116.729)&&(R0<=1120.602)){
			*pT = 30.0;
			*pT += (R0-1116.729)/(1120.602-1116.729);
		}else if((R0>1120.602)&&(R0<=1124.474)){
			*pT = 31.0;
			*pT += (R0-1120.602)/(1124.474-1120.602);
		}else if((R0>1124.474)&&(R0<=1128.345)){
			*pT = 32.0;
			*pT += (R0-1124.474)/(1128.345-1124.474);
		}else if((R0>1128.345)&&(R0<=1132.215)){
			*pT = 33.0;
			*pT += (R0-1128.345)/(1132.215-1128.345);
		}else if((R0>1132.215)&&(R0<=1136.083)){
			*pT = 34.0;
			*pT += (R0-1132.215)/(1136.083-1132.215);
		}else if((R0>1136.083)&&(R0<=1139.95)){
			*pT = 35.0;
			*pT += (R0-1136.083)/(1139.95-1136.083);
		}else if((R0>1139.95)&&(R0<=1143.817)){
			*pT = 36.0;
			*pT += (R0-1139.95)/(1143.817-1139.95);
		}else if((R0>1143.817)&&(R0<=1147.681)){
			*pT = 37.0;
			*pT += (R0-1143.817)/(1147.681-1143.817);
		}else if((R0>1147.681)&&(R0<=1151.545)){
			*pT = 38.0;
			*pT += (R0-1147.681)/(1151.545-1147.681);
		}else if((R0>1151.545)&&(R0<=1155.408)){
			*pT = 39.0;
			*pT += (R0-1151.545)/(1155.408-1151.545);
		}else if((R0>1155.408)&&(R0<=1159.27)){
			*pT = 40.0;
			*pT += (R0-1155.408)/(1159.27-1155.408);
		}else if((R0>1159.27)&&(R0<=1163.13)){
			*pT = 41.0;
			*pT += (R0-1159.27)/(1163.13-1159.27);
		}else if((R0>1163.13)&&(R0<=1166.989)){
			*pT = 42.0;
			*pT += (R0-1163.13)/(1166.989-1163.13);
		}else if((R0>1166.989)&&(R0<=1170.847)){
			*pT = 43.0;
			*pT += (R0-1166.989)/(1170.847-1166.989);
		}
		else if((R0>1170.847)&&(R0<=1174.704 ))
		{
			*pT = 44.0;
			*pT += (R0-1170.847)/(1174.704-1170.847);
		}
		else if((R0>1174.704)&&(R0<=1178.560  ))
		{
			*pT = 45.0;
			*pT += (R0-1174.704)/(1178.560 -1174.704);
		}
		else if((R0>1178.560)&&(R0<=1182.414   ))
		{
			*pT = 46.0;
			*pT += (R0-1178.560)/(1182.414 -1178.560);
		}
		else if((R0>1182.414)&&(R0<=1186.268   ))
		{
			*pT = 47.0;
			*pT += (R0-1182.414)/(1186.268 -1182.414);
		}
		else if((R0>1186.268)&&(R0<=1190.120  ))
		{
			*pT = 48.0;
			*pT += (R0-1186.268)/(1190.120 -1186.268);
		}
		else if((R0>1190.120)&&(R0<=1193.971  ))
		{
			*pT = 49.0;
			*pT += (R0-1190.120)/(1193.971 -1190.120);
		}
		else if((R0>1193.971)&&(R0<=1197.821  ))
		{
			*pT = 50.0;
			*pT += (R0-1193.971)/(1197.821 -1193.971);
		}
		else if((R0>1197.821)&&(R0<=1201.670  ))
		{
			*pT = 51.0;
			*pT += (R0-1197.821)/(1201.670 -1197.821);
		}
		else if((R0>1201.670)&&(R0<=1205.518  ))
		{
			*pT = 52.0;
			*pT += (R0-1201.670)/(1205.518 -1201.670);
		}
		else if((R0>1205.518)&&(R0<=1209.364  ))
		{
			*pT = 53.0;
			*pT += (R0-1205.518)/(1209.364 -1205.518);
		}
		else if((R0>1209.364)&&(R0<=1213.210  ))
		{
			*pT = 54.0;
			*pT += (R0-1209.364)/(1213.210 -1209.364);
		}
		else if((R0>1213.210)&&(R0<=1217.054))
		{
			*pT = 55.0;
			*pT += (R0-1213.210)/(1217.054 -1213.210);
		}
		else if((R0>1217.054)&&(R0<=1220.897))
		{
			*pT = 56.0;
			*pT += (R0-1217.054)/(1220.897 - 1217.054);
		}
		else if((R0>1220.897)&&(R0<=1224.739))
		{
			*pT = 57.0;
			*pT += (R0-1220.897)/(1224.739 -1220.897);
		}
		else if((R0>1224.739)&&(R0<=1228.579))
		{
			*pT = 58.0;
			*pT += (R0-1224.739)/(1228.579 -1224.739);
		}
		else if((R0>1228.579)&&(R0<=1232.419))
		{
			*pT = 59.0;
			*pT += (R0-1228.579)/(1232.419 -1228.579);
		}
		else if((R0>1232.419)&&(R0<=1236.257))
		{
			*pT = 60.0;
			*pT += (R0-1232.419)/(1236.257 -1232.419);
		}
		else if((R0>1236.257)&&(R0<=1240.095))
		{
			*pT = 61.0;
			*pT += (R0-1236.257)/(1240.095 -1236.257);
		}
		else if((R0>1240.095)&&(R0<=1243.931))
		{
			*pT = 62.0;
			*pT += (R0-1240.095)/(1243.931 -1240.095);
		}
		else if((R0>1243.931)&&(R0<=1247.766 ))
		{
			*pT = 63.0;
			*pT += (R0-1243.931)/(1247.766 -1243.931);
		}
		else if((R0>1247.766)&&(R0<=1251.600 ))
		{
			*pT = 64.0;
			*pT += (R0-1247.766)/(1251.600 -1247.766);
		}
		else if((R0>1251.600)&&(R0<=1255.432 ))
		{
			*pT = 65.0;
			*pT += (R0-1251.600)/(1255.432 -1251.600);
		}
		else if((R0>1255.432)&&(R0<=1259.264 ))
		{
			*pT = 66.0;
			*pT += (R0-1255.432)/(1259.264 -1255.432);
		}
		else if((R0>1259.264)&&(R0<=1263.094 ))
		{
			*pT = 67.0;
			*pT += (R0-1259.264)/(1263.094 -1259.264);
		}
		else if((R0>1263.094)&&(R0<=1266.923  ))
		{
			*pT = 68.0;
			*pT += (R0-1263.094)/(1266.923 -1263.094);
		}
		else if((R0>1266.923)&&(R0<=1270.751 ))
		{
			*pT = 69.0;
			*pT += (R0-1266.923)/(1270.751 -1266.923);
		}
		else if((R0>1270.751)&&(R0<=1274.578))
		{
			*pT = 70.0;
			*pT += (R0-1270.751)/(1274.578 -1270.751);
		}
		else if((R0>1274.578)&&(R0<=1278.404))
		{
			*pT = 71.0;
			*pT += (R0-1274.578)/(1278.404 -1274.578);
		}
		else if((R0>1278.404)&&(R0<=1282.228))
		{
			*pT = 72.0;
			*pT += (R0-1278.404)/(1282.228 -1278.404);
		}
		else if((R0>1282.228)&&(R0<=1286.052 ))
		{
			*pT = 73.0;
			*pT += (R0-1282.228)/(1286.052 -1282.228);
		}

		else if((R0>1286.052)&&(R0<= 1289.874))
		{
			*pT = 74.0;
			*pT += (R0-1286.052)/(1289.874 -1286.052);
		}
		else if((R0>1289.874)&&(R0<= 1293.695 ))
		{
			*pT = 75.0;
			*pT += (R0-1289.874)/(1293.695 -1289.874);
		}
		else if((R0>1293.695)&&(R0<=1297.515 ))
		{
			*pT = 76.0;
			*pT += (R0-1293.695)/(1297.515  -1293.695);
		}
		else if((R0>1297.515)&&(R0<= 1301.334))
		{
			*pT = 77.0;
			*pT += (R0-1297.515)/(1301.334 -1297.515);
		}
		else if((R0>1301.334)&&(R0<= 1305.152))
		{
			*pT = 78.0;
			*pT += (R0-1301.334)/(1305.152 -1301.334);
		}
		else if((R0>1305.152)&&(R0<= 1308.968))
		{
			*pT = 79.0;
			*pT += (R0-1305.152)/(1308.968 -1305.152);
		}
		else if((R0>1308.968)&&(R0<= 1312.783))
		{
			*pT = 80.0;
			*pT += (R0-1308.968)/(1312.783 -1308.968);
		}
		else if((R0>1312.783)&&(R0<= 1316.597))
		{
			*pT = 81.0;
			*pT += (R0-1312.783)/(1316.597 -1312.783);
		}
		else if((R0>1316.597)&&(R0<= 1320.411))
		{
			*pT = 82.0;
			*pT += (R0-1316.597)/(1320.411 -1316.597);
		}
		else if((R0>1320.411)&&(R0<= 1324.222))
		{
			*pT = 83.0;
			*pT += (R0-1320.411)/(1324.222  -1320.411);
		}
		else if((R0>1324.222)&&(R0<= 1328.033))
		{
			*pT = 84.0;
			*pT += (R0-1324.222)/(1328.033 -1324.222);
		}

		else if((R0>1328.033)&&(R0<=1331.843  ))
		{
			*pT = 85.0;
			*pT += (R0-1328.033)/(1331.843 -1328.033);
		}
		return FUNC_SUCCESS;
}

//获取回风温度
extern uint32_t GetReturnTemperature(PTEMP_RATIO pRatio,__packed float *pT)
{
    uint32_t u32Temp=0;
    volatile float fTemp=0.0;
    
    if((pRatio==NULL)||(pT==NULL))
        return FUNC_FAILURE;
    
	WatchDogFeed();
   	UART_CS(1);
    DelayUs(10);	
    if(0!=GetSPLIB(&u32Temp))
        return FUNC_FAILURE;  
    UART_CS(0);
    if(u32Temp&0x80000)
	{
        u32Temp = (~u32Temp)&0x7ffff;
        fTemp 	= u32Temp;
        fTemp 	= -fTemp; 
    }else{
        fTemp 	= u32Temp;
    }  
    fTemp = fTemp*pRatio->k + pRatio->b;
    //debug("V:%f ",fTemp);
    //MCP9700A温度传感器，0.01V/摄氏度 -40——125对应10mV-1.75V
    *pT = (fTemp-0.5)*100.0;
    WatchDogFeed();
		
    return FUNC_SUCCESS;
}
//获取送风温度
extern uint32_t GetSupplyTemperature(PTEMP_RATIO pRatio,__packed float *pT)
{
    uint32_t u32Temp=0;
    volatile float fTemp=0.0,R=0.0;
    
    if((pRatio==NULL)||(pT==NULL))
        return FUNC_FAILURE;
    
	WatchDogFeed();
   	UART_CS(2);
    DelayUs(10);	
    if(0!=GetSPLIB(&u32Temp))
        return FUNC_FAILURE;  
    UART_CS(0);
    if(u32Temp&0x80000)
	{
        u32Temp = (~u32Temp)&0x7ffff;
        fTemp 	= u32Temp;
        fTemp 	= -fTemp; 
    }else{
        fTemp 	= u32Temp;
    }  
    fTemp = fTemp*pRatio->k + pRatio->b;
    //debug("V:%f ",fTemp);
    *pT = (fTemp-0.5)*100.0;
    WatchDogFeed();
		
    return FUNC_SUCCESS;	
}


extern uint32_t SaveFrequentUpdateData(PCOMM_DATA pData)
{
    uint8_t 							i,len;
    volatile float 				fTemp;
    volatile uint32_t 		Addr;
    COMM_FREQUENT_UPDATE 	FreqUpdate={0};
    
    if(pData==NULL)
        return FUNC_FAILURE;
    
    len = 0;
    
    memcpy(&FreqUpdate.Date,&pData->YdtData->Date,sizeof(REAL_DATE));
    len += sizeof(REAL_DATE);
    
    for(i=0;i<3;i++)
    {
        FreqUpdate.LastDayEnergyP[i] = pData->LastDayEnergyP[i];
        len += 4;
    }
    FreqUpdate.LastDayAllEnergyP 	= pData->LastDayAllEnergyP;
    len += 4;
    
    for(i=0;i<3;i++)
    {
        fTemp = (float)pData->RN8209Data[i].EnergyP;
        FreqUpdate.NowEnergyP[i] = fTemp;
        len += 4;
    }
    
    FreqUpdate.NowAllEnergyP 			= pData->NowAllEnergyP;
    len += 4;
    
    FreqUpdate.AirCondLastCtrlCmdId	= pData->AirCondLastCtrlCmdId;
    len += 4;
    
    Addr = FLASH_ADDR_FREQUENT_UPDATE;
    Addr += 0x200*pData->YdtData->Date.Minute;
    WatchDogFeed();
		
    return MeterData_WaitWriteSucc(0,(uint8_t *)&FreqUpdate,Addr,SAVE_FREQUENT_UPDATE_SIZE);
}


extern uint32_t GetFrequentUpdateData(PCOMM_DATA pData)
{
    uint8_t 							i,ret;
    volatile float 				fTemp=0.0,fMaxEnergy=0.0;
    volatile uint32_t 		Addr;
    COMM_FREQUENT_UPDATE 	FreqUpdate={0};
    
    if(pData==NULL)
        return FUNC_FAILURE;
    
    fMaxEnergy = 0.0;
    for(i=0;i<60;i++)
    {   
        Addr = FLASH_ADDR_FREQUENT_UPDATE;
        Addr += 0x200*i;
			
        WatchDogFeed();
        ret = MeterData_WaitReadSucc(0,(uint8_t *)&FreqUpdate,Addr,SAVE_FREQUENT_UPDATE_SIZE);
        if(ret==0)
		{
            fTemp = FreqUpdate.NowAllEnergyP;
            if(fTemp>fMaxEnergy)
            {
                fMaxEnergy = fTemp;
                      
                for(i=0;i<3;i++)
                {
                    fTemp = FreqUpdate.NowEnergyP[i];
                    pData->RN8209Data[i].EnergyP 	= (double)fTemp;
                    pData->LastDayEnergyP[i] 			= FreqUpdate.LastDayEnergyP[i];
                }
                pData->LastDayAllEnergyP 					= FreqUpdate.LastDayAllEnergyP;
                pData->NowAllEnergyP 							= FreqUpdate.NowAllEnergyP;
                pData->YdtData->Egy.TodayEgy 			= pData->NowAllEnergyP;
                pData->AirCondLastCtrlCmdId	= FreqUpdate.AirCondLastCtrlCmdId;
            }
        }
		WatchDogFeed();
    }

    return 0;
}

extern void MeterDataInit(uint8_t IsInit,PCOMM_DATA pData)
{ 
    int32_t 	Ret;
    uint32_t 	i;
    uint8_t  	FirstInitFlag=0,UseBackup=1;
    uint32_t 	FirstInitDW;
    uint32_t    ManuTemp;
    
    if(pData==NULL)
        return;

    if(0 == AT45DBXXX_Check())
		{    
        //FirstInitFlag
				WatchDogFeed();
        Ret = MeterData_WaitReadSucc(UseBackup,(uint8_t *)&FirstInitDW,FLASH_ADDR_FIRST_INIT,FLASH_ADDR_FIRST_INIT_SIZE);
        if(Ret != 0)
				{
            FirstInitFlag = 1;
            FirstInitDW 	= FIRST_INIT_FLAG;
            WatchDogFeed();
            MeterData_WaitWriteSucc(UseBackup,(uint8_t *)&FirstInitDW,FLASH_ADDR_FIRST_INIT,FLASH_ADDR_FIRST_INIT_SIZE);
        }
        else
				{
						FirstInitFlag	= 0;
				}   
				
        //初始化电表校准参数
        /**/
        if(IsInit)
				{
            do
						{
                i++;
								WatchDogFeed();
            }while((0 != ReadRn8209CalidataFromFlash(&pData->Rn8209c))&&(i<5));
            if(i==5)
						{
                RN8209C_Init_ResetParam(&pData->Rn8209c.Def[0].InitParam);
                RN8209C_Init_ResetParam(&pData->Rn8209c.Def[1].InitParam);
                RN8209C_Init_ResetParam(&pData->Rn8209c.Def[2].InitParam);
								WatchDogFeed();
                WriteRn8209CalidataToFlash(&pData->Rn8209c);
            }
        }
				WatchDogFeed();
        Ret = MeterData_WaitReadSucc(UseBackup,(uint8_t *)&pData->YdtData->Param,FLASH_ADDR_PARAM,FLASH_ADDR_PARAM_SIZE);
        if(Ret != 0)
				{
            if(FirstInitFlag == 1)
						{
                pData->YdtData->Param.PowerOnTemp 		= (float)PARAM_POWER_ON_TEMP_DEFAULT;
                pData->YdtData->Param.PowerOffTemp 		= (float)PARAM_POWER_OFF_TEMP_DEFAULT;
                pData->YdtData->Param.ReturnTempUpper = (float)PARAM_RETURN_TEMP_UPPER_DEFAULT;
                pData->YdtData->Param.ReturnTempLower = (float)PARAM_RETURN_TEMP_LOWER_DEFAULT;
                pData->YdtData->Param.SetTemp 				= (float)PARAM_POWER_OFF_TEMP_DEFAULT;
                pData->YdtData->Param.RunMode 				= PARAM_RUN_MODE_DEFAULT;
                pData->YdtData->Param.WindMode 				= PARAM_WIND_MODE_DEFAULT;
								WatchDogFeed();
                MeterData_WaitWriteSucc(UseBackup,(uint8_t *)&pData->YdtData->Param,FLASH_ADDR_PARAM,FLASH_ADDR_PARAM_SIZE);
            }
            //printf("%s %d Read Error(Ret=%ld)\n",__FUNCTION__,__LINE__,Ret);
        }  
        WatchDogFeed();
        Ret = MeterData_WaitReadSucc(UseBackup,(uint8_t *)&pData->YdtData->RTempRadio,FLASH_ADDR_R_TEMP_RATIO,FLASH_ADDR_R_TEMP_RATIO_SIZE);
        if(Ret != 0)
				{
            if(FirstInitFlag == 1)
						{
                pData->YdtData->RTempRadio.k 	= 1.0;
                pData->YdtData->RTempRadio.b 	= 0;
                pData->YdtData->RTempRadio.R0 = 0;
								WatchDogFeed();
                MeterData_WaitWriteSucc(UseBackup,(uint8_t *)&pData->YdtData->RTempRadio,FLASH_ADDR_R_TEMP_RATIO,FLASH_ADDR_R_TEMP_RATIO_SIZE);
            }
            //printf("%s %d Read Error(Ret=%ld)\n",__FUNCTION__,__LINE__,Ret);
        } 
        WatchDogFeed();
        Ret = MeterData_WaitReadSucc(UseBackup,(uint8_t *)&pData->YdtData->STempRadio,FLASH_ADDR_S_TEMP_RATIO,FLASH_ADDR_S_TEMP_RATIO_SIZE);
        if(Ret != 0)
				{
            if(FirstInitFlag == 1)
						{
                pData->YdtData->STempRadio.k 	= 1.0;
                pData->YdtData->STempRadio.b 	= 0;
                pData->YdtData->STempRadio.R0 = 0;
								WatchDogFeed();
                MeterData_WaitWriteSucc(UseBackup,(uint8_t *)&pData->YdtData->STempRadio,FLASH_ADDR_S_TEMP_RATIO,FLASH_ADDR_S_TEMP_RATIO_SIZE);
            }
            //printf("%s %d Read Error(Ret=%ld)\n",__FUNCTION__,__LINE__,Ret);
        } 
        WatchDogFeed();
        Ret = MeterData_WaitReadSucc(UseBackup,(uint8_t *)&pData->YdtData->SaveCnt.DayEgyCnt,FLASH_ADDR_HISTORY_DAY_CNT,FLASH_ADDR_HISTORY_DAY_CNT_SIZE);
        if(Ret != 0)
				{
            if(FirstInitFlag == 1)
						{
                pData->YdtData->SaveCnt.DayEgyCnt = 0;
								WatchDogFeed();
                MeterData_WaitWriteSucc(1,(uint8_t *)&pData->YdtData->SaveCnt.DayEgyCnt,FLASH_ADDR_HISTORY_DAY_CNT,FLASH_ADDR_HISTORY_DAY_CNT_SIZE);
            }
            //printf("%s %d Read Error(Ret=%ld)\n",__FUNCTION__,__LINE__,Ret);
        } 
        WatchDogFeed();
		Ret = MeterData_WaitReadSucc(UseBackup,(uint8_t *)&pData->YdtData->Egy.LastDayEgy,FLASH_ADDR_LAST_DAY_CONSUME,FLASH_ADDR_LAST_DAY_CONSUME_SIZE);
        if(Ret != 0)
		{
			WatchDogFeed();
            if(FirstInitFlag == 1)
			{
                pData->YdtData->Egy.LastDayEgy = 0;
                MeterData_WaitWriteSucc(1,(uint8_t *)&pData->YdtData->Egy.LastDayEgy,FLASH_ADDR_LAST_DAY_CONSUME,FLASH_ADDR_LAST_DAY_CONSUME_SIZE);
            }
            //printf("%s %d Read Error(Ret=%ld)\n",__FUNCTION__,__LINE__,Ret);
        } 
		WatchDogFeed();
        Ret = MeterData_WaitReadSucc(UseBackup,(uint8_t *)&pData->YdtData->InfraredParam,FLASH_ADDR_PARAM_INFRARED,FLASH_ADDR_PARAM_INFRARED_SIZE);
        if(Ret != 0)
		{
            if(FirstInitFlag == 1)
			{
                pData->YdtData->InfraredParam.TransmitFreq = 40000;
                pData->YdtData->InfraredParam.CommandAction = NoAction;
                pData->YdtData->InfraredParam.CommandNum = 0;
								WatchDogFeed();
                MeterData_WaitWriteSucc(UseBackup,(uint8_t *)&pData->YdtData->InfraredParam,FLASH_ADDR_PARAM_INFRARED,FLASH_ADDR_PARAM_INFRARED_SIZE);
            }
            //printf("%s %d Read Error(Ret=%ld)\n",__FUNCTION__,__LINE__,Ret);
        } 
        
		WatchDogFeed();
        Ret = MeterData_WaitReadSucc(UseBackup,(uint8_t *)&ManuTemp,FLASH_ADDR_AIR_CON_MANUFACTURER,FLASH_ADDR_AIR_CON_MANUFACTURER_SIZE);
        if(Ret != 0)
		{
            if(FirstInitFlag == 1)
			{
                ManuTemp = MANUFACTURER_WHIRLPOOL;
				InfraSetManuID(ManuTemp);
				WatchDogFeed();
                MeterData_WaitWriteSucc(UseBackup,(uint8_t *)&ManuTemp,FLASH_ADDR_AIR_CON_MANUFACTURER,FLASH_ADDR_AIR_CON_MANUFACTURER_SIZE);
            }
            //printf("%s %d Read Error(Ret=%ld)\n",__FUNCTION__,__LINE__,Ret);
        } 
        InfraSetManuID(ManuTemp);

		
		WatchDogFeed();
        if(FirstInitFlag != 1)
            GetFrequentUpdateData(pData);
				WatchDogFeed();
     }

    //初始化其余无需存储到EEPROM中的数据
    for(i=0;i<3;i++)
		{
        pData->Rn8209c.Def[i].CaliParam.In = 15.0;
        pData->Rn8209c.Def[i].CaliParam.Un = 220.0;
    }
}


extern void Sys_Soft_Reset(void) 
{    
    //SCB->AIRCR =0X05FA0000|(uint32_t)0x04;	
    SYS_UnlockReg();
    //SYS_ResetChip();
    SYS_ResetCPU();
    SYS_LockReg();
}

//恢复出厂设置
extern void MeterFactoryReset(void)
{
    if(0 == AT45DBXXX_Check())
		{
		debug("Satrt MeterFactoryReset\n");
        AT45DB161_BulkErase();
    }
}


/*********************************历史数据***********************************************/
extern uint32_t HistoryDayData_MakeRecord(PHistoryDayData pData,uint8_t *pRecord,uint8_t *plen)
{
    uint8_t Offset=0;
    uint32_t u32Temp;
    
    if((pData==NULL)||(pRecord==NULL)||(plen == NULL))
        return 1;
    if(*plen<11)
        return 1;
		
    GetWord(&pRecord[Offset]) = SWAP_WORD(pData->Date.Year);
    Offset += 2;
    pRecord[Offset] = pData->Date.Month;
    Offset += 1;
    pRecord[Offset] = pData->Date.Day;
    Offset += 1;
    pRecord[Offset] = pData->Date.Hour;
    Offset += 1;
    pRecord[Offset] = pData->Date.Minute;
    Offset += 1;
    pRecord[Offset] = pData->Date.Second;
    Offset += 1;
    u32Temp = pData->EnergyP*YDT1363_3_DATA_RATIO_ENERGY;
    GetDWord(&pRecord[Offset]) = SWAP_DWORD(u32Temp);
    Offset += 4;
    
    *plen = Offset;
		
    return 0;
}

//获取指定保存计数的历史日电能记录
extern uint32_t HistoryDayData_GetRecord(uint32_t AssignCnt,PHistoryDayData pDayData)
{
    uint8_t 					Temp[20]={0};
    volatile uint32_t ReadAddr;
    
    if((pDayData==NULL)||(AssignCnt==0))
        return 1;
    
    ReadAddr 	= AssignCnt%SAVE_HISTORY_DATA_CNT_MAX;
    ReadAddr 	-= 1;
    ReadAddr 	*= 0x200;
    ReadAddr 	+= SAVE_HISTORY_DATA_START_ADDR;
		
		WatchDogFeed();
    if(0 == MeterData_WaitReadSucc(1,Temp,ReadAddr,SAVE_HISTORY_DATA_BYTES))
		{
        memcpy(pDayData,Temp,SAVE_HISTORY_DATA_BYTES);
				WatchDogFeed();
        return  0;
    }
		WatchDogFeed();
		
    return 1;     
}

//功能：存储历史日电能记录，环形存储  参数：pSaveCnt 保存当前存储计数 pData 存储数据
extern uint32_t HistoryDayData_SaveRecord(__packed uint32_t *pSaveCnt,PCOMM_DATA pData)
{
    uint8_t 					Len;
    uint8_t 					Temp[20]={0};
    HistoryDayData 		DayData={0};
    uint32_t				 	Addr;
   
    if((pSaveCnt==NULL)||(pData==NULL))
        return 1;
    
    memcpy((uint8_t *)&DayData.Date,(uint8_t *)&pData->YdtData->Date,sizeof(REAL_DATE));
    DayData.EnergyP = pData->YdtData->Egy.TodayEgy;
		
    WatchDogFeed();
    Len = sizeof(Temp);
    if(0 != HistoryDayData_MakeRecord(&DayData,Temp,&Len))
        return 1;
		
		WatchDogFeed();
    Addr = SAVE_HISTORY_DATA_START_ADDR;
    Addr += (*pSaveCnt%SAVE_HISTORY_DATA_CNT_MAX)*0x200;
    WatchDogFeed();
    if(0 != MeterData_WaitWriteSucc(1,Temp,Addr,Len))
		{
				WatchDogFeed();
        return 1;
    }
   
    *pSaveCnt += 1;
    WatchDogFeed();
    if(0== MeterData_WaitWriteSucc(1,(uint8_t *)pSaveCnt,FLASH_ADDR_HISTORY_DAY_CNT,FLASH_ADDR_HISTORY_DAY_CNT_SIZE))
        return 0;
    else
        *pSaveCnt -= 1;
    WatchDogFeed();
		
    return 1;
}

//根据标志位判断哪些记录是否需要保存
extern void SaveHistoryData(uint32_t *pSaveFlag,PCOMM_DATA pData)
{ 
    uint8_t 	i=0;
    uint32_t 	SaveFlag = *pSaveFlag;
    PYDT1363_DATA pYdtData=NULL;
    
    if(pData==NULL)
        return;
    pYdtData = pData->YdtData;
    
    if((pYdtData->Date.Hour == 0)&&(pYdtData->Date.Minute==0))
		{
        if((SaveFlag&SAVE_FLAG_HISTORY_DAY_DATA_MASK)!=SAVE_FLAG_HISTORY_DAY_DATA_MASK)
				{
						WatchDogFeed();
            if(0==HistoryDayData_SaveRecord(&pYdtData->SaveCnt.DayEgyCnt,pData))
						{
                SaveFlag |= SAVE_FLAG_HISTORY_DAY_DATA_MASK;
                for(i=0;i<3;i++)
								{
                    pData->LastDayEnergyP[i] 		= (float)pData->RN8209Data[i].EnergyP;
                }
                pData->YdtData->Egy.LastDayEgy 	= pData->NowAllEnergyP - pData->LastDayAllEnergyP;
                pData->LastDayAllEnergyP 				= pData->NowAllEnergyP;
								WatchDogFeed();
								MeterData_WaitWriteSucc(1,(uint8_t *)&pData->YdtData->Egy.LastDayEgy,FLASH_ADDR_LAST_DAY_CONSUME,FLASH_ADDR_LAST_DAY_CONSUME_SIZE);
								WatchDogFeed();
            } 
        }    
    }
    *pSaveFlag = SaveFlag;
}

extern uint32_t ACGainCalibration_ALL(PRN8209C pRn8209c)
{
    uint8_t i=0;
    uint32_t Ret=0;
    
    for(i=0;i<3;i++)
		{
				UART_CS(i+1);
				DelayUs(10);	
				WatchDogFeed();
				Ret += RN8209C_ACGainCalibration(&pRn8209c->Def[i]);
				UART_CS(0);
				WatchDogFeed();
		}
        
    return Ret;
}

extern uint32_t ACPhaseCalibration_ALL(PRN8209C pRn8209c)
{
    uint8_t i=0;
    uint32_t Ret=0;
    
    for(i=0;i<3;i++)
		{
				UART_CS(i+1);
				DelayUs(10);	
				WatchDogFeed();
				Ret += RN8209C_ACPhaseCalibration(&pRn8209c->Def[i]);
				UART_CS(0);
				WatchDogFeed();
		}
        
    return Ret;
}

extern uint32_t ACOffsetCalibration_ALL(PRN8209C pRn8209c)
{
    uint8_t i=0;
    uint32_t Ret=0;
    
    for(i=0;i<3;i++)
		{
				UART_CS(i+1);
				DelayUs(10);	
				WatchDogFeed();
				Ret += RN8209C_ACOffsetCalibration(&pRn8209c->Def[i]);
				UART_CS(0);
				WatchDogFeed();
		}
        
    return Ret;
}

extern uint32_t ACReactiveCalibration_ALL(PRN8209C pRn8209c)
{
    uint8_t i=0;
    uint32_t Ret=0;
    
    for(i=0;i<3;i++)
		{
				UART_CS(i+1);
				DelayUs(10);	
				WatchDogFeed();
				Ret += RN8209C_ACReactiveCalibration(&pRn8209c->Def[i]);
				UART_CS(0);
				WatchDogFeed();
		}
        
    return Ret;
}

extern uint32_t ACCurrentOffsetCalibration_ALL(PRN8209C pRn8209c)
{   
    uint8_t i=0;
    uint32_t Ret=0;
    
    for(i=0;i<3;i++)
		{
				UART_CS(i+1);
				DelayUs(10);	
				WatchDogFeed();
				Ret += RN8209C_ACCurrentOffsetCalibration(&pRn8209c->Def[i]);
				UART_CS(0);
				WatchDogFeed();
		}
        
    return Ret;
}

extern void Comm_CtrlLightOnOff(uint8_t LightNum,uint8_t IsOn)
{
	
		GPIO_SetMode(P0, BIT0, GPIO_PMD_OUTPUT);
        GPIO_SetMode(P1, BIT0, GPIO_PMD_OUTPUT);
        GPIO_SetMode(P0, BIT2, GPIO_PMD_OUTPUT);
   
		if((LightNum == LED_ERROR)&& IsOn){
            P00 = 1;
        }else{
            P00 = 0;   
        }
		if((LightNum == LED_COMMUNICATION)&& IsOn){
            P10 = 1;
        }else{
            P10 = 0;   
        }
        if((LightNum == LED_LEARNING)&& IsOn){
            P02 = 1;
        }else{
            P02 = 0;   
        }        
}

//复位从LDROM开始
extern void Sys_Hard_Reset(void) 
{    
    //SCB->AIRCR =0X05FA0000|(uint32_t)0x04;	
    SYS_UnlockReg();
    SYS_ResetChip();
    //SYS_ResetCPU();
    SYS_LockReg();
}
//连续写入数据到FLASH,返回0成功，返回2失败
static uint8_t WriteLearningCmdToSPIFlash(uint8_t *pucBuffer, uint32_t ulWriteAddr, uint32_t ulNumByteToWrite)
{
    uint8_t Ret,TryCnt;
    
    TryCnt = 0;
    do{
        WatchDogFeed();
        Ret = AT45DBxxxD_Write(pucBuffer,ulWriteAddr,(uint32_t)ulNumByteToWrite);
    }while((Ret!=0)&&(++TryCnt<3));
    
    if(Ret != 0){
        Ret = 2;
    }
    return Ret;
}
//连续从FLASH读出数据,返回0成功，返回2失败
static uint8_t ReadLearningCmdFromSPIFlash(uint8_t *pucBuffer, uint32_t ulReadAddr, uint32_t ulNumByteToRead)
{
    uint8_t Ret,TryCnt;
    
    TryCnt = 0;
    do{
        WatchDogFeed();
        Ret = AT45DBxxxD_Read(pucBuffer,ulReadAddr,(uint32_t)ulNumByteToRead);
    }while((Ret!=0)&&(++TryCnt<3));
    
    if(Ret != 0){
        Ret = 2;
    }
    return Ret;
}
//从FLASH读出数据并校验,返回0成功，返回2失败
static uint8_t ReadLearningCmdAndCheck(SaveRecordFormatDef *pucBuffer, uint32_t ulReadAddr, uint32_t ulNumByteToRead)
{
    uint8_t Ret,TryCnt,Sum;
    
    TryCnt = 0;
    do{
        WatchDogFeed();
        Ret = AT45DBxxxD_Read((uint8_t *)pucBuffer,ulReadAddr,(uint32_t)ulNumByteToRead);
    }while((Ret!=0)&&(++TryCnt<3));
    
    if(Ret != 0){
        return 2; 
    }
    
    Sum = GetSum((uint8_t *)pucBuffer,ulNumByteToRead-1);
    if(Sum == pucBuffer->CheckSum){
        Ret = 0;
    }else{
        Ret = 2;
    }
    return Ret;
}
//保存指定的命令号对应红外波形到Falsh中
//返回0代表成功 1代表参数无效 2代表写入FLASH失败 3代表写入备份数据失败 4代表写入数据和备份数据都失败
extern uint32_t SaveInfraLearningCmdToFlash(const uint32_t ManuID,const uint32_t CmdID,SaveRecordFormatDef *pSaveCmd,const uint32_t SaveBytes)
{
    uint8_t Sum=0,Ret,RetBak;
    uint32_t SaveStartAddr;
    
    if(pSaveCmd==NULL)
        return FUNC_RET_PARAM_ERROR;
    if(!ManuIDIsValid(ManuID))
        return FUNC_RET_PARAM_ERROR;
    if(!CMDIDIsValid(CmdID))
        return FUNC_RET_PARAM_ERROR;
    
    WatchDogFeed();
   
    Sum = GetSum((uint8_t *)pSaveCmd,SaveBytes-1);
    pSaveCmd->CheckSum = Sum;
    SaveStartAddr = FLASH_ADDR_INFRA_LEARNING_RECORD_START;
    //SaveStartAddr += 0x400*ManuID*CMDID_MAX;
    SaveStartAddr += 0x400*CmdID;
    //debug("MeterData_SetValidDataToAT24CXX StartAddr=%x WriteBytes=%x\n",StartAddr,WriteBytes);

    Ret = WriteLearningCmdToSPIFlash((uint8_t *)pSaveCmd,SaveStartAddr,SaveBytes);
    WatchDogFeed();
    RetBak = WriteLearningCmdToSPIFlash((uint8_t *)pSaveCmd,SaveStartAddr+FLASH_ADDR_BACKUP_OFFSET,SaveBytes);
    WatchDogFeed();
    Ret = ReadLearningCmdAndCheck(pSaveCmd,SaveStartAddr,SaveBytes);
    WatchDogFeed();
    RetBak = ReadLearningCmdAndCheck(pSaveCmd,SaveStartAddr+FLASH_ADDR_BACKUP_OFFSET,SaveBytes);
    WatchDogFeed();
    if((Ret==0)&&(RetBak==0)){
        return 0;
    }else if((Ret==2)&&(RetBak==0)){
        return 2;
    }else if((Ret==0)&&(RetBak==2)){
        return 3;
    }else{
        return 4;
    }
}

//从Falsh中获取指定的命令号对应红外波形，保存到pCmd指定的地址
//返回0代表成功 1代表参数无效 2代表读取FLASH失败
extern uint32_t GetInfraLearningCmdFromFlash(const uint32_t ManuID,const uint32_t CmdID,SaveRecordFormatDef *pCmd,const uint32_t ReadBytes)
{
    uint8_t Ret;
    uint32_t ReadStartAddr;
    
    if(pCmd==NULL)
        return FUNC_RET_PARAM_ERROR;
    if(!ManuIDIsValid(ManuID))
        return FUNC_RET_PARAM_ERROR;
    if(!CMDIDIsValid(CmdID))
        return FUNC_RET_PARAM_ERROR;
 
    //根据传入的厂家编号ManuID(0开始)和命令号CmdID(0开始)计算读取FLASH起始地址
    ReadStartAddr = FLASH_ADDR_INFRA_LEARNING_RECORD_START;
    //ReadStartAddr += 0x400*ManuID*CMDID_MAX;
    ReadStartAddr += 0x400*CmdID;
    
    //debug("MeterData_SetValidDataToAT24CXX StartAddr=%x WriteBytes=%x\n",StartAddr,WriteBytes);
    Ret = ReadLearningCmdAndCheck(pCmd,ReadStartAddr,ReadBytes);
    
    if(Ret!=0){
        WatchDogFeed();
        Ret = ReadLearningCmdAndCheck(pCmd,ReadStartAddr+FLASH_ADDR_BACKUP_OFFSET,ReadBytes);
    }
    
    return Ret;

}

extern uint32_t SendInfraCmd(SaveRecordFormatDef *pCmd)
{
    uint32_t i,Us;
    
    if(pCmd==NULL)
        return FUNC_RET_PARAM_ERROR;
    //IRTX = IRTX_HIGH;
    //DelayUs(100);
    for(i=0;i<pCmd->RecordCnt;i++){
        if(i>=INFRA_RECORD_MAX){
            break;
        }
        Us = pCmd->Record[i].DurationUS;
        IRTX = (pCmd->Record[i].PinLevel==0?IRTX_LOW:IRTX_HIGH);
        DelayUs(Us);
    }
    IRTX = IRTX_HIGH; 
    return FUNC_RET_TRUE;
}

extern uint32_t CleanInfraCmd(void)
{
    uint32_t i,Ret;
    SaveRecordFormatDef *pTemp=NULL;

   if(FUNC_RET_TRUE != InfraGetRecordTemp(&pTemp))
		return FUNC_RET_FALSE;
    WatchDogFeed();
	memset(pTemp,0,sizeof(SaveRecordFormatDef));
	for(i=0;i<CMDID_MAX;i++){
		if(FUNC_RET_TRUE == SaveInfraLearningCmdToFlash(InfraGetManuID(),i,pTemp,sizeof(SaveRecordFormatDef))){
			Ret += FUNC_RET_TRUE;
		}
	}
	InfraFreeRecordTemp(&pTemp);
	return Ret;
}


