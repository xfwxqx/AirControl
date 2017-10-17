#include "NUC029xAN.h"

#ifndef __INFRA_RECORD_H
#define __INFRA_RECORD_H

#define INFRA_RECORD_MAX	300

#define INFRAROD_LEARNING_CMD_CNT_MAX                       15      //红外学习支持的最大命令数



typedef enum{
	MANUFACTURER_WHIRLPOOL = 0x0001,   	//惠而浦
    MANUFACTURER_GREE = 0x0002,   			//格力
    MANUFACTURER_MEIDI = 0x0003,      		//美的
    MANUFACTURER_HAIER = 0x0004,      		//海尔
    MANUFACTURER_AUX = 0x0005,        		//奥克斯
    MANUFACTURER_TCL = 0x0006,        		//TCL
    MANUFACTURER_CHIGO = 0x0007,      		//志高
    MANUFACTURER_CORON = 0x0008,      		//科龙
    
    MANUFACTURER_ID_MAX = 0xffff,
}AIR_CONDITION_MANUFACTURER_ID_ENUM;

typedef enum{
    CMDID1_CTRLORDER_OFF = 0,   	//关机
    CMDID2_ON_COOL_TEMP20_STRONG,  	//开机、制冷模式、20度、高风
    CMDID3_ON_COOL_TEMP21_STRONG,  	//开机、制冷模式、21度、高风
    CMDID4_ON_COOL_TEMP22_STRONG,  	//开机、制冷模式、22度、高风
    CMDID5_ON_COOL_TEMP23_STRONG,  	//开机、制冷模式、23度、高风
    CMDID6_ON_COOL_TEMP24_STRONG,  	//开机、制冷模式、24度、高风
    CMDID7_ON_COOL_TEMP25_STRONG,   //开机、制冷模式、25度、高风
    CMDID8_ON_COOL_TEMP26_STRONG,  	//开机、制冷模式、26度、高风
    CMDID9_ON_COOL_TEMP27_STRONG,  	//开机、制冷模式、27度、高风
    CMDID10_ON_COOL_TEMP28_STRONG,  //开机、制冷模式、28度、高风
    CMDID11_ON_COOL_TEMP29_STRONG,  //开机、制冷模式、29度、高风
    CMDID12_ON_COOL_TEMP30_STRONG,  //开机、制冷模式、30度、高风
    CMDID13_AIRSUPPLY_STRONG,       //开机、送风模式、高风
    CMDID14_ON_HEAT_TEMP16_STRONG,  //开机、制热模式、16度、高风
    CMDID15_CTRLORDER_ON,          	//开机指令
    CMDID16_RESERVE,
    CMDID17_RESERVE,
    CMDID18_RESERVE,
    CMDID19_RESERVE,
    CMDID20_RESERVE,
    CMDID21_RESERVE,
    CMDID22_RESERVE,
    CMDID23_RESERVE,
    CMDID24_RESERVE,
    CMDID25_RESERVE,
    CMDID26_RESERVE,
    CMDID27_RESERVE,
    CMDID28_RESERVE,
    CMDID29_RESERVE,
    CMDID30_RESERVE,
    CMDID31_RESERVE,
    CMDID32_RESERVE,
    CMDID_MAX,
}INFRAROD_LEARNING_CMDID_ENUM;

typedef struct __attribute__((packed)) {
	uint8_t 	PinLevel;	//引脚电平
	uint16_t 	DurationUS;	//持续时间,最大持续时间为65536us
}RecordDef;

typedef struct __attribute__((packed)) {
	uint32_t    ManufacturerID;	//厂商编码
	uint32_t    LearningCmdID;		//自学习命令号
    uint32_t    RecordCnt;
    RecordDef   Record[INFRA_RECORD_MAX];	//红外脉冲
    uint8_t     CheckSum;
}SaveRecordFormatDef;


typedef struct __attribute__((packed)) {
	uint8_t     IsLastFailingEdge:1;	
	uint8_t     IsLastRisingEdge:1;
	uint8_t     IsFirstRecv:1;	
	uint8_t     IsSaveRcdMemoryUsing:1;	
	uint8_t     IsStartRecv:1;	
	uint8_t     Reser:3;
	uint32_t    LastRisingEdgeTVal;
	uint32_t    LastFailingEdgeTVal;
    SaveRecordFormatDef SaveRcd;
}InfraRecordManagerDef;

extern InfraRecordManagerDef gInfraMag;

extern void InfraInit(void);
extern void InfraDealPulse(uint32_t PinLevel,uint32_t TDRVal);
extern uint32_t InfraGetRecordTemp(SaveRecordFormatDef **pSave);
extern void InfraFreeRecordTemp(SaveRecordFormatDef **pSave);

extern uint32_t InfraGetRecvCode(SaveRecordFormatDef **pSave);

extern uint32_t InfraGetManuID(void);
extern uint32_t InfraSetManuID(uint32_t SetID);
extern uint32_t InfraLearning(uint32_t ManuID,uint32_t CmdID);
//输出:*pSta的bit0-bit31分别对应命令1-32的自学习状态，1-已学习 0-未学习
extern uint32_t InfraGetLearningStatus(uint32_t *pSta);

//#define ManuIDIsValid(ID) ((((uint32_t)(ID)&0xffff)!=0)&&(((uint32_t)(ID)&0xffff0000)!=0))
#define ManuIDIsValid(ID) (1)
#define CMDIDIsValid(ID) ((uint8_t)(ID)<CMDID_MAX)

#define USE_RECORD_MEMORY_MUTEX_LOCK  do{gInfraMag.IsSaveRcdMemoryUsing=1;}while(0)
#define USE_RECORD_MEMORY_MUTEX_UNLOCK  do{gInfraMag.IsSaveRcdMemoryUsing=0;}while(0)
#define USE_RECORD_MEMORY_MUTEX_IS_LOCK  ((gInfraMag.IsSaveRcdMemoryUsing == 1)||(gInfraMag.IsStartRecv == 1))


#endif

