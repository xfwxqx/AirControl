/**
  *****************************************************************************
  *                            电总YDT1363_3协议
  *
  *                       (C) Copyright 2017-2018, ***
  *                            All Rights Reserved
  *****************************************************************************
  *
  * @File    : Ydt1363.h
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
  				暂未支持获取历史数据和告警	
  *
  *
  *****************************************************************************
**/


#ifndef __YDT1363_h_ 
#define __YDT1363_h_


/**
  ****************************** Support C++ **********************************
**/
#ifdef __cplusplus
	extern "C"{
#endif		
/**
  *****************************************************************************
**/
		

/******************************************************************************
                             外部函数头文件
                  应用到不同的外设头文件请在这里修改即可                        
******************************************************************************/     
#include <stdint.h>
#include "RN8209C.h"

#define USER_DEFINE_SOFT_VERSION    "20171017"
		
#define YDT1363_FUNC_RET_FALSE 								-1
#define YDT1363_FUNC_RET_SUCCESS 							0
/**/

#define UNIT_TYPE																						"IACC-5000"
#define SOFTWARE_VERSION																		"0100"							//厂家软件版本
#define VENDER																							"JSYAAO"						//厂家名称

#define YDT1363_3_DATA_RATIO_VOLTAGE												1				//电压倍比
#define YDT1363_3_DATA_RATIO_CURRENT												100			//电流倍比
#define YDT1363_3_DATA_RATIO_TEMPERATUE											1				//温度倍比
#define YDT1363_3_DATA_RATIO_ENERGY 												100			//电能倍比
#define YDT1363_3_DATA_RATIO_TEMPERATUE_EX									10			//温度倍比
#define YDT1363_3_PROTOCAL_VER															0x10
#define YDT1363_3_PROTOCAL_ADDR															0x01
#define YDT1363_3_PROTOCAL_CID1_DCMETER											0x66

#define 	YDT1363_3_DATABUF_MAX							90  	

#define YDT1363_3_PROTOCAL_CID2_GET_DATE										0x4d    //获取时间信息（4DH）
#define YDT1363_3_PROTOCAL_CID2_SET_DATE										0x4e    //设置日期时间（4EH）
#define YDT1363_3_PROTOCAL_CID2_GET_VERSION									0x4f    //获取讯协议版本号（4FH）
#define YDT1363_3_PROTOCAL_CID2_GET_DEVICE_ADDR							0x50    //获取设备地址（50H）
#define YDT1363_3_PROTOCAL_CID2_GET_VENDER									0x51    //获取厂家信息（51H）

#define YDT1363_3_PROTOCAL_CID2_GET_ANALOG									0x42    //获取模拟量量化数据
#define YDT1363_3_PROTOCAL_CID2_GET_SWITCH									0x43    //获取开关量
#define YDT1363_3_PROTOCAL_CID2_GET_ALARM										0x44    //获取告警状态
#define YDT1363_3_PROTOCAL_CID2_REMOTE_CTRL									0x45    //遥控
#define YDT1363_3_PROTOCAL_CID2_GET_PARAM           				0x47    //获取表参数
#define YDT1363_3_PROTOCAL_CID2_SET_PARAM	        					0x49    //设置表参数
#define YDT1363_3_PROTOCAL_CID2_SET_PARAM_INFRARED    			0x80    //设置空调控制器安装参数
#define YDT1363_3_PROTOCAL_CID2_GET_ENERGY      						0x81    //获取空调电量数据(81H)
#define YDT1363_3_PROTOCAL_CID2_GET_HISTORY_DATA						0x86    //获取模块历史数据(86H)


/************************** 生产用  ***********************************************************/
#define YDT1363_3_PROTOCAL_CID2_FACTORY_RESET				    		0xe0  	//恢复出厂默认设置
#define YDT1363_3_PROTOCAL_CID2_GET_R_TEMP_RATIO						0xe1 		//获取回风温度校正系数
#define YDT1363_3_PROTOCAL_CID2_SET_R_TEMP_RATIO						0xe2 		//设置回风温度校正系数
#define YDT1363_3_PROTOCAL_CID2_GET_S_TEMP_RATIO						0xe3 		//获取送风温度校正系数
#define YDT1363_3_PROTOCAL_CID2_SET_S_TEMP_RATIO						0xe4 		//设置送风温度校正系数
#define YDT1363_3_PROTOCAL_CID2_CLEAN_SELF_LEARN		    		0xe5		//自学习清除
#define YDT1363_3_PROTOCAL_CID2_RN8209C_AC_GAIN				    	0xe6    //功率增益校准
#define YDT1363_3_PROTOCAL_CID2_RN8209C_AC_PHASE						0xe7    //相位校准
#define YDT1363_3_PROTOCAL_CID2_RN8209C_AC_OFFSET						0xe8    //有功功率OFFSET校准
#define YDT1363_3_PROTOCAL_CID2_RN8209C_AC_REACTIVE					0xe9    //无功相位补偿
#define YDT1363_3_PROTOCAL_CID2_RN8209C_AC_CURR				    	0xea    //电流有效值OFFSET

#define YDT1363_3_PROTOCAL_CID2_GET_ANALOG_EX								0xeb    //获取模拟量量化数据
#define YDT1363_3_PROTOCAL_CID2_GET_RN8209C_INITPARAM				0xec    //获取RN8209C初始化校表数据
#define YDT1363_3_PROTOCAL_CID2_SET_RN8209C_INITPARAM				0xed    //设置RN8209C初始化校表数据

#define YDT1363_3_PROTOCAL_CID2_AI_CALI				                0xee    //两路AI校准

//和分路计量直流表保持一致
//获取软件版本号
#define YDT1363_3_PROTOCAL_CID2_GET_SOFT_VER     0xef
//系统复位
#define YDT1363_3_PROTOCAL_CID2_SET_TO_BOOT 	0xf0  
//获取自学习遥控器厂家名,取值参考<<InfraRecord.h>>中的AIR_CONDITION_MANUFACTURER_ID_ENUM
#define YDT1363_3_PROTOCAL_CID2_GET_MANUFACTURER 	0xf1  
#define YDT1363_3_PROTOCAL_CID2_SET_MANUFACTURER 	0xf2 
//上传下载命令
#define YDT1363_3_PROTOCAL_CID2_UPLOAD_CMD 		0xf3  
#define YDT1363_3_PROTOCAL_CID2_DOWNLOAD_CMD 	0xf4 
//获取自学习状态
#define YDT1363_3_PROTOCAL_CID2_GET_LEARNING_STATUS 	0xf5 

/*
#define YDT1363_3_PROTOCAL_CID2_CS5460A_Calibration_OFF			0xe1  	//偏置校准
#define YDT1363_3_PROTOCAL_CID2_CS5460A_Calibration_GN			0xe2 		//增益校准
#define YDT1363_3_PROTOCAL_CID2_CS5460A_Calibration_P				0xe3 		//功率校准
#define YDT1363_3_PROTOCAL_CID2_GET_R_TEMP_RATIO						0xe4 		//获取回风温度校正系数
*/

#define YDT1363_3_PROTOCAL_RTN_NORMAL												0x0   	//正常
#define YDT1363_3_PROTOCAL_RTN_VER_ERROR										0x01   	//协议版本错
#define YDT1363_3_PROTOCAL_RTN_CHKSUM_ERROR									0x02   	//CHKSUM错
#define YDT1363_3_PROTOCAL_RTN_LCHKSUM_ERROR								0x03   	//LCHKSUM错
#define YDT1363_3_PROTOCAL_RTN_CID2_INVALID									0x04   	//CID2无效
#define YDT1363_3_PROTOCAL_RTN_FORMAT_ERROR									0x05   	//命令格式错
#define YDT1363_3_PROTOCAL_RTN_DATA_INVALID									0x06  	//无效数据

#define YDT1363_3_PROTOCAL_RTN_LEARNING_SUCC								0x80  	//红外学习成功
#define YDT1363_3_PROTOCAL_RTN_LEARNING_FAILURE							0x81  	//红外学习失败
#define YDT1363_3_PROTOCAL_RTN_DATA_NONE										0x82  	//用户自定义：无历史数据

#define YDT1363_3_PROTOCAL_RTN_CMD_EXEC_FAIL		0xe2  	//命令执行失败
#define YDT1363_3_PROTOCAL_RTN_AI_CALI_FIRST		0xe3  	//AI校准第一个点执行成功



#define DATE_YEAR_IS_VALID(x) 								(((x)>=2000)&&((x)<=2099))
#define DATE_MONTH_IS_VALID(x) 								(((x)>=1)&&((x)<=12))
#define DATE_DAY_IS_VALID(x) 									(((x)>=1)&&((x)<=31))
#define DATE_HOUR_IS_VALID(x) 								(((x)>=0)&&((x)<=23))
#define DATE_MINUTE_OR_SECOND_IS_VALID(x) 		(((x)>=0)&&((x)<=59))


#define YDT1363_LOAD_RECORD_MAX 	40  //单条报文中上传和下载命令中红外波形记录最大数

#define NeedReset(Rtn) (((Rtn)==YDT1363_3_PROTOCAL_CID2_FACTORY_RESET)||((Rtn)==YDT1363_3_PROTOCAL_CID2_CLEAN_SELF_LEARN))

/* 电总协议数据结构定义 
*变量均为原始数据值(未ASCII转码), Length为数据内容实际长度
*/
typedef struct __attribute__((packed)) _PROTOCAL_YDT1363_3
{
		uint8_t		Ver;
		uint8_t		Addr;
		uint8_t		CID1;
		uint8_t		CID2;
		uint16_t	Length;			
		uint8_t		DataBuf[YDT1363_3_DATABUF_MAX];
}PROTOCAL_YDT1363_3,*PPROTOCAL_YDT1363_3;


/*************************   模拟量 ***************************************/

typedef struct __attribute__((packed)) YDT1363_ANALOG
{
		float    VoltagePhaseA;  				//A相电压
    float    VoltagePhaseB;  				//B相电压
    float    VoltagePhaseC;  				//C相电压
    float    CurrentPhaseA;  				//A相电流
    float    CurrentPhaseB;  				//B相电压
    float    CurrentPhaseC;  				//C相电压
    float    SupplyTemperature;			//送风温度
		float    ReturnTemperature;			//回风温度
}YDT1363_ANALOG,*PYDT1363_ANALOG;



/*************************   开关状态 ******************************************/
typedef enum _SWITCH_DEFINE
{
		PowerOn 	= 1,		//
		PowerOff 	= 0,
}SWITCH_DEFINE;

typedef struct __attribute__((packed)) _YDT1363_SWITCH
{
		SWITCH_DEFINE     PowerOnOff;
    uint8_t     			UserDefine;
}YDT1363_SWITCH,*PYDT1363_SWITCH;


/*************************   模块告警 ***************************************/
/*
告警字节描述：
——00H 			：正常；
——01H 			：有告警；
——80H～EFH 	：用户自定义；
——F0H 			：其他故障。
*/


typedef enum _ALARM_DEFINE
{
		DCMETER_ALARM_NORMAL 			= 0,		//
		DCMETER_ALARM_LOWER 			= 1,
		DCMETER_ALARM_HIGH 				= 2,
    DCMETER_ALARM_NO_COLLECT 	= 0x20,
    DCMETER_ALARM_FAILURE 		= 0xf0,
}ALARM_DEFINE;

typedef struct __attribute__((packed)) _YDT1363_ALARM
{
		ALARM_DEFINE		ColdFailure;					//制冷异常
		ALARM_DEFINE		HeatFailure;					//制热异常
    ALARM_DEFINE		Power;								//电源告警
    ALARM_DEFINE		CurrSensor;						//电流传感器
    ALARM_DEFINE		ReturnTempSensor;			//回风温度传感器
    ALARM_DEFINE		SupplyTempSensor;			//送风温度传感器
}YDT1363_ALARM,*PYDT1363_ALARM;



/*************************   表参数 ***************************************/
typedef enum _MODE_DEFINE
{
		AUTO 	= 0,		//自动
		COLD 	= 1,		//制冷
		REHUM = 2,		//除湿
    WIND 	= 3,		//送风
    HEAT 	= 4,		//制热
}MODE_DEFINE;

typedef enum _WIND_MODE_DEFINE
{
		LOW 		= 1,	//低风
		MEDIUM 	= 2,	//中风
		HIGH 		= 3,	//高风
}WIND_MODE_DEFINE;

typedef struct __attribute__((packed)) _YDT1363_PARAM
{					
    float   						PowerOnTemp;    				//空调开机温度(制冷运行开机温度)
    float   						PowerOffTemp;    				//空调关机温度(制冷运行关机温度)
    float   						ReturnTempUpper;    		//回风温度上限（室内高温报警温度设定值）
    float   						ReturnTempLower;    		//回风温度下限（室内低温报警温度设定值）
    float   						SetTemp;    						//温度设定值
    MODE_DEFINE   			RunMode;    						//运行模式设定
    WIND_MODE_DEFINE   	WindMode;    						//内风机风速设定
}YDT1363_PARAM,*PYDT1363_PARAM;

/*************************   红外模块参数 ***************************************/
typedef enum _COMMAND_ACTION_DEFINE
{
		NoAction = 0,		//无动作
		Learning = 1,		//学习红外命令
		Transmit = 2,		//发射红外命令
}COMMAND_ACTION_DEFINE;

typedef struct __attribute__((packed)) _YDT1363_PARAM_INFRARED
{
		float   									TransmitFreq;    	//红外发射频率
    COMMAND_ACTION_DEFINE    	CommandAction;		//红外命令设置动作
    uint16_t    							CommandNum;				//红外命令动作序号
}YDT1363_PARAM_INFRARED,*PYDT1363_PARAM_INFRARED;

/*****************************************************************************/
typedef struct __attribute__((packed)) _REAL_DATE
{
		uint16_t	Year;						
		uint8_t 	Month;
		uint8_t 	Day;
		uint8_t 	Hour;
		uint8_t 	Minute;
		uint8_t 	Second;
}REAL_DATE,*PREAL_DATE;

/***********************     电量    ******************************************/
typedef struct __attribute__((packed)) _YDT1363_ENERGY
{
		float   LastDayEgy;
    float   TodayEgy;
}YDT1363_ENERGY,*PYDT1363_ENERGY;

/*****************************     校准源电压和电流满量程   *************************************/
typedef struct __attribute__((packed)) _DCMETER_CS5460_DC_MAX
{
    float   Imax;
    float   Vmax;
}DCMETER_CS5460_DC_MAX,*PDCMETER_CS5460_DC_MAX;

typedef struct
{
    uint32_t DayEgyCnt;     
}SAVE_COUNT,*PSAVE_COUNT;

/******************************** 温度校准 ****************************************/
#define TEMP_RATIO_RATIO 100000000.0

typedef struct
{
    float   k;  //电压修正系数
    float   b;  //电压零点
    float   R0; //电阻固定修正值
}TEMP_RATIO,*PTEMP_RATIO;


typedef struct _YDT1363_DATA
{
    REAL_DATE   			Date;
		YDT1363_ANALOG  	Analog;
    YDT1363_SWITCH  		Switch;
    YDT1363_ALARM   		Alarm;
    YDT1363_PARAM   		Param;
    YDT1363_PARAM_INFRARED 	InfraredParam;
    SAVE_COUNT  			SaveCnt;
    YDT1363_ENERGY  		Egy;     
    TEMP_RATIO              STempRadio;     //送风温度校准系数
    TEMP_RATIO              RTempRadio;     //回风温度校准系数
	PRN8209C         		pRn8209c;
	//uint8_t 				IsDownLoadCmd;
}YDT1363_DATA,*PYDT1363_DATA;//__attribute__((packed)) 

/******************************************************************************
                                  外部功能函数                      
******************************************************************************/
extern uint8_t GetCID2(void);
extern void SetDataFlag(uint8_t IsSwitchChange,uint8_t IsAlarmChange);
/*
 ************************************************************************************************************************************************************************     
 *函数名称:	ProtocolProc_YDT1363_3
 *功能描述:	协议处理进程
 *输入描述:	协议帧pRecvBuf、pRecvLen
 *输出描述:	pSendBuf、pSendLen
 *返回描述:	失败YDT1363_FUNC_RET_FALSE/成功YDT1363_FUNC_RET_SUCCESS
 *作者日期:	
 *全局声明: 
 *特殊说明: 
 ************************************************************************************************************************************************************************       
 */
extern int32_t ProtocolProc_YDT1363_3(PPROTOCAL_YDT1363_3 pProcBuf, uint8_t *pSendBuf, uint32_t *pSendLen);
extern int32_t ProtocolProc_YDT1363_3_Make_NoneDataBufFrame(const uint8_t Rtn,uint8_t *pBuf,uint32_t *pLen);
/*
 ************************************************************************************************************************************************************************     
 *函数名称:	ProtocolProc_YDT1363_3_CallBack
 *功能描述:	获取协议处理部分所需数据的地址指针
 *输入描述:	无
 *输出描述:	
 *返回描述:	数据的地址指针
 *作者日期:	
 *全局声明: 
 *特殊说明: 
 ************************************************************************************************************************************************************************       
 */
extern PYDT1363_DATA ProtocolProc_YDT1363_3_CallBack(void);

extern uint8_t AirCondGetPowerAlarmStatus(void);
extern uint8_t AirCondGetHotAlarmStatus(void);
extern uint8_t AirCondGetCoolAlarmStatus(void);
extern uint8_t AirCondGetReturnTempAlarmStatus(void);
extern uint8_t AirCondGetOutletTempAlarmStatus(void);
extern uint8_t AirCondGetSwitchOnOffStatus(void);
extern uint8_t AirCondGetSettingTempStatus(void);
extern uint8_t AirCondGetRunModeStatus(void);
extern uint8_t AirCondGetWindSpeedStatus(void);
extern void AirCondCtrlOperation(uint8_t OpCmd);
extern void AirCondLearnOperation(uint8_t OpCmd);
extern void AirCondSelfLearnClean(void);

/**
  ****************************** Support C++ **********************************
**/
#ifdef __cplusplus
	}
#endif
/**
  *****************************************************************************
**/


#endif  /* end Ydt1363.h */
