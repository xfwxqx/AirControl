#ifndef __USER_COMMON_H_
#define __USER_COMMON_H_

#include "NUC029xAN.h"
#include "Ydt1363.h"
#include "RN8209C.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "InfraRecord.h"

#define FUNC_RET_TRUE           0
#define FUNC_RET_PARAM_ERROR    1
#define FUNC_RET_FALSE          2

#define SWAP_DWORD(x)													((((x)<<24)&0xFF000000)|(((x)<<8)&0xFF0000)|(((x)>>8)&0xFF00)|(((x)>>24)&0xFF))
#define SWAP_WORD(x)													((((x)>>8)&0xFF)|(((x)<<8)&0xFF00))
#define SWAP_LONG(x)													((signed long int)((((x)<<24)&0xFF000000)|(((x)<<8)&0xFF0000)|(((x)>>8)&0xFF00)|(((x)>>24)&0xFF)))
#define SWAP_SHORT(x)													((signed short int)((((x)>>8)&0xFF)|(((x)<<8)&0xFF00)))

#define GetDWord(addr) 			(*(__packed uint32_t *)(addr))
#define GetWord(addr) 			(*(__packed uint16_t *)(addr))

/*************************************************************************************/

#define DEBUG

#ifdef DEBUG
	#define _DEBUG 1
#else 
	#define _DEBUG 0
#endif

#define debug_conf(conf,fmt,args...) \
	do{	\
		if(conf)	\
			printf(fmt,##args);\
	}while(0)
		
#define debug(fmt,args...) debug_conf(_DEBUG,fmt,##args)


typedef struct
{
    float       	Freq;       //频率
    float       	Voltage;    //电压
    float       	Current;    //电流
    float       	PowerP;     //有功功率
    float       	PowerQ;     //无功功率
    double       	EnergyP;    //有功电能
    double       	EnergyQ;    //无功电能    
}RN8209_DATA,*PRN8209_DATA; 

typedef struct
{
    RN8209_DATA     RN8209Data[3];  
    RN8209C         Rn8209c;
    PYDT1363_DATA   YdtData;
    float           LastDayEnergyP[3];
    float           LastDayAllEnergyP;
    float           NowAllEnergyP;
	uint32_t			AirCondLastCtrlCmdId;
}COMM_DATA,*PCOMM_DATA; 

typedef struct _PM5KT_COMM_TM
{
	uint16_t			Year;				/*70--*/
	uint8_t			Month;			/*1-12*/
	uint8_t			Day;				/*1-31*/
	uint8_t			Hour;				/*0-23*/
	uint8_t			Minute;			/*0-59*/
	uint8_t			Second;			/*0-59*/
}PM5KT_COMM_TM, *PPM5KT_COMM_TM;
 
/*********************************
关于FLASH数据分区(
PageSize		= 0x200,
BlockSize		= 0x1000,
SectorSize	= 0x20000,
Capacity		= 0x200000,
TotalPages	= 0x1080,)

将2M空间分成三部分：
    1)数据区:0x0-0xfffff,共0x100000Bytes
    2)备份区:0x100000-0x1fffff,共0x100000Bytes
    
   
数据区分块：
    0x0     -   0x1ff      首次初始化数据标记(4 按页对齐为:0x200)
    0x200   -   0x3ff      表参数(4 按页对齐为:0x200)
    0x400   -   0x5ff      历史天数据计数(4 按页对齐为:0x200)
    0x600   -   0x7ff      上一天电能(4 按页对齐为:0x200)
    0x5800  -   0x11fff     历史数据
    0x20000 -               红外学习命令编码
    0xd0000 -   0xd77ff     动态数据区0x7800Bytes,60Pages           

***********************************/

#define FLASH_ADDR_BACKUP_OFFSET 						0x100000  	

#define FLASH_ADDR_PAGE1  									0x0
#define FLASH_ADDR_PAGE2  									0x200
#define FLASH_ADDR_PAGE3  									0x400
#define FLASH_ADDR_PAGE4  									0x600

//Page1
//首次初始化数据标志
#define FIRST_INIT_FLAG        							0x5aa5a55a
#define FLASH_ADDR_FIRST_INIT     					(FLASH_ADDR_PAGE1) //占4个字节
#define FLASH_ADDR_FIRST_INIT_SIZE     			4 
#define FLASH_ADDR_FIRST_INIT_CHECKSUM     	(FLASH_ADDR_FIRST_INIT+FLASH_ADDR_FIRST_INIT_SIZE)

//Page2
//表参数
#define PARAM_POWER_ON_TEMP_DEFAULT     		28.0
#define PARAM_POWER_OFF_TEMP_DEFAULT    		25.0
#define PARAM_RETURN_TEMP_UPPER_DEFAULT 		30.0
#define PARAM_RETURN_TEMP_LOWER_DEFAULT 		10.0

#define PARAM_RUN_MODE_DEFAULT          		COLD
#define PARAM_WIND_MODE_DEFAULT        			LOW

#define FLASH_ADDR_PARAM     								(FLASH_ADDR_PAGE2) 
#define FLASH_ADDR_PARAM_SIZE     					(sizeof(YDT1363_PARAM)%0x200) 
#define FLASH_ADDR_PARAM_CHECKSUM     			(FLASH_ADDR_PARAM+FLASH_ADDR_PARAM_SIZE)

#define FLASH_ADDR_R_TEMP_RATIO     				(FLASH_ADDR_PARAM_CHECKSUM+1) 
#define FLASH_ADDR_R_TEMP_RATIO_SIZE     		(sizeof(TEMP_RATIO)%0x200) 
#define FLASH_ADDR_R_TEMP_RATIO_CHECKSUM    (FLASH_ADDR_R_TEMP_RATIO+FLASH_ADDR_R_TEMP_RATIO_SIZE)

#define FLASH_ADDR_S_TEMP_RATIO     				(FLASH_ADDR_R_TEMP_RATIO_CHECKSUM+1) 
#define FLASH_ADDR_S_TEMP_RATIO_SIZE     		(sizeof(TEMP_RATIO)%0x200) 
#define FLASH_ADDR_S_TEMP_RATIO_CHECKSUM    (FLASH_ADDR_S_TEMP_RATIO+FLASH_ADDR_S_TEMP_RATIO_SIZE)


#define FLASH_ADDR_PARAM_INFRARED     				(FLASH_ADDR_S_TEMP_RATIO_CHECKSUM+1) 
#define FLASH_ADDR_PARAM_INFRARED_SIZE     		(sizeof(YDT1363_PARAM_INFRARED)%0x200)
#define FLASH_ADDR_PARAM_INFRARED_CHECKSUM    (FLASH_ADDR_PARAM_INFRARED+FLASH_ADDR_PARAM_INFRARED_SIZE)

#define FLASH_ADDR_AIR_CON_MANUFACTURER     	(FLASH_ADDR_PARAM_INFRARED_CHECKSUM+1) 
#define FLASH_ADDR_AIR_CON_MANUFACTURER_SIZE    4
#define FLASH_ADDR_AIR_CON_MANUFACTURER_CHECKSUM    (FLASH_ADDR_AIR_CON_MANUFACTURER+FLASH_ADDR_AIR_CON_MANUFACTURER_SIZE)


//Page3
#define FLASH_ADDR_HISTORY_DAY_CNT     			(FLASH_ADDR_PAGE3) 
#define FLASH_ADDR_HISTORY_DAY_CNT_SIZE     4
#define FLASH_ADDR_HISTORY_DAY_CNT_CHECKSUM (FLASH_ADDR_HISTORY_DAY_CNT+FLASH_ADDR_HISTORY_DAY_CNT_SIZE)
//Page4
#define FLASH_ADDR_LAST_DAY_CONSUME     		(FLASH_ADDR_PAGE4) 
#define FLASH_ADDR_LAST_DAY_CONSUME_SIZE     4
#define FLASH_ADDR_LAST_DAY_CONSUME_CHECKSUM (FLASH_ADDR_LAST_DAY_CONSUME+FLASH_ADDR_LAST_DAY_CONSUME_SIZE)

//动态更新数据结构体，占用99字节,未超过100,每次读写一页，根据实时适中的分钟保存到相应叶
#define FLASH_ADDR_FREQUENT_UPDATE  					0xd0000
#define SAVE_FREQUENT_UPDATE_SIZE  						(sizeof(COMM_FREQUENT_UPDATE)%0x200) //39 44

//历史数据从0x5800处起始，每条记录占一页（0x200）,共0xc800Bytes,到0x12000-1为止
#define SAVE_HISTORY_DATA_CNT_MAX  						100
#define SAVE_HISTORY_DATA_START_ADDR  				0x5800
#define SAVE_HISTORY_DATA_BYTES  							11
#define SAVE_FLAG_HISTORY_DAY_DATA_MASK 			0x01

//厂家红外脉冲编码存储地址:起始地址+厂家编号*命令编号*0x400,每条命令最大占2页
#define FLASH_ADDR_INFRA_LEARNING_RECORD_START     		(0x20000) 
#define FLASH_ADDR_INFRA_LEARNING_RECORD_SIZE        (sizeof(SaveRecordFormatDef)%0x400)
#define FLASH_ADDR_INFRA_LEARNING_RECORD_CHECKSUM    (FLASH_ADDR_INFRA_LEARNING_RECORD_START+FLASH_ADDR_INFRA_LEARNING_RECORD_SIZE) 

typedef struct __attribute__((packed)) _HistoryDayData
{
    REAL_DATE 			Date;
    float 					EnergyP;
}HistoryDayData,*PHistoryDayData;

typedef struct __attribute__((packed)) _COMM_FREQUENT_UPDATE{
     
		REAL_DATE       Date;
    float           LastDayEnergyP[3];
    float           LastDayAllEnergyP;
    float           NowEnergyP[3];
    float           NowAllEnergyP;
	uint32_t				AirCondLastCtrlCmdId;
}COMM_FREQUENT_UPDATE,*PCOMM_FREQUENT_UPDATE;



extern void Rank(uint32_t average[],int n);
extern void DelayUs(uint32_t time);
extern uint32_t WriteDataFlash(uint32_t StartAddr,__packed uint32_t *pData,uint8_t WriteCnt);
extern uint32_t ReadDataFlash(uint32_t StartAddr,__packed uint32_t *pData,uint8_t ReadCnt);
extern uint32_t EraseDataFlash_Page(uint32_t StartAddr);


extern uint32_t RN8209C_Calibration_GN_ALL(PRN8209C_DEF pRn8209c);
//将Rn8209校准后数据写入DataFlash(起始地址：0x1f000-0x1fffff)
extern uint32_t WriteRn8209CalidataToFlash(PRN8209C pRn8209c);
extern uint32_t ReadRn8209CalidataFromFlash(PRN8209C pRn8209c);

    
extern uint32_t GetRealTimeData(PCOMM_DATA pData);
//获取回风温度
extern uint32_t GetReturnTemperature(PTEMP_RATIO pRatio,__packed float *pT);
//获取送风温度
extern uint32_t GetSupplyTemperature(PTEMP_RATIO pRatio,__packed float *pT);


//0 ：成功 1：数据未初始化 2:数据错误 3:输入参数错误
extern uint32_t MeterData_WaitReadSucc(uint8_t IsReadBackup,uint8_t *pData,uint32_t StartAddr,uint16_t ReadBytes);
//Input: IsReadBackup为1时,读取备份区，0时不读取备份区
//0:成功 1：参数错误 2：写数据失败 3：写备份数据失败
extern uint32_t MeterData_WaitWriteSucc(uint8_t IsReadBackup,uint8_t *pData,uint32_t StartAddr,uint16_t WriteBytes);

extern uint32_t GetFrequentUpdateData(PCOMM_DATA pData);
//每分钟调用一次
extern uint32_t SaveFrequentUpdateData(PCOMM_DATA pData);
extern void MeterDataInit(uint8_t IsInit,PCOMM_DATA pData);

extern void Sys_Soft_Reset(void);    
extern void MeterFactoryReset(void); 
//复位从LDROM开始
extern void Sys_Hard_Reset(void);

//获取指定保存计数的历史日电能记录
extern uint32_t HistoryDayData_GetRecord(uint32_t AssignCnt,PHistoryDayData pDayData);
//根据标志位判断哪些记录是否需要保存
extern void SaveHistoryData(uint32_t *pSaveFlag,PCOMM_DATA pData);

extern void WatchDogFeed(void);

extern uint32_t ACGainCalibration_ALL(PRN8209C pRn8209c);
extern uint32_t ACPhaseCalibration_ALL(PRN8209C pRn8209c);
extern uint32_t ACOffsetCalibration_ALL(PRN8209C pRn8209c);
extern uint32_t ACReactiveCalibration_ALL(PRN8209C pRn8209c);
extern uint32_t ACCurrentOffsetCalibration_ALL(PRN8209C pRn8209c);

#define		LED_ERROR					1
#define		LED_COMMUNICATION			2
#define		LED_LEARNING				3
extern void Comm_CtrlLightOnOff(uint8_t LightNum,uint8_t IsOn);

//从Falsh中获取指定厂家的指定的命令号对应红外波形，保存到pCmd指定的地址;若IsReadBackup为假，则不获取
//返回0代表成功 1代表参数无效 2代表读取FLASH失败 3代表数据校验失败
extern uint32_t GetInfraLearningCmdFromFlash(const uint32_t ManuID,const uint32_t CmdID,SaveRecordFormatDef *pCmd,const uint32_t ReadBytes);

//保存指定厂家的指定的命令号对应红外波形到Falsh中
//返回0代表成功 1代表参数无效 2代表写入FLASH失败 3代表写入备份数据失败 4代表写入数据和备份数据都失败
extern uint32_t SaveInfraLearningCmdToFlash(const uint32_t ManuID,const uint32_t CmdID,SaveRecordFormatDef *pSaveCmd,const uint32_t SaveBytes);

#define IRTX P26
#define IRTX_LOW 0
#define IRTX_HIGH 1
extern uint32_t SendInfraCmd(SaveRecordFormatDef *pCmd);

extern uint32_t CleanInfraCmd(void);


#endif

