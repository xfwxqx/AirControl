#include "NUC029xAN.h"

#ifndef __SOFT_TIMER_H
#define __SOFT_TIMER_H


typedef void (*PTimeOutCallBack)(void *pHandle);

typedef struct
{
	uint32_t TimerEnable;
	uint32_t TimerOut;
	uint32_t TimerCnt; 
    //0常规模式,需要自己判断定时器是否超时,此时后面参数无用 
    //1定时器自动超时处理模式,定时中断时判断超时后调用超时回调函数
	uint32_t TimerMode; 
	PTimeOutCallBack pFunc;
	void *pFuncParam; 
}SoftTimer;


typedef  enum 
{
    TIMER_TEST,                     //系统测试定时器
	TIMER_INFRA_RECV,				//红外接收定时器
	TIMER_DOWNLOAD_INFRA_CMD,		//下载红外命令超时定时器
	TIMER_INIT_PARAM,               //初始化参数定时器
	TIMER_GET_REAL_DATA,           	//获取实时数据超时定时器
    TIMER_NUM						//定时器个数
}ENUM_TIMER;

#define SOFT_Timer_Disable TIMER_DisableInt(TIMER3)
#define SOFT_Timer_Enable TIMER_EnableInt(TIMER3)
#define SOFT_Timer_Interval_MS 100		//定时器最小定时间隔为100ms
//extern SoftTimer SoftTimers[TIMER_NUM];

extern void SoftTimerInit(void);
extern void SoftTimerStart(uint32_t TimerID, uint32_t n100Ms,uint32_t Mode,PTimeOutCallBack pTimeOutCallBack,void *pHandle);
extern void SoftTimerStop(uint32_t TimerID);
extern uint32_t SoftTimerIsOver(uint32_t TimerID);
extern uint32_t SoftTimerIsStart(uint32_t TimerID);
extern void SoftTimerIntervalProcess(void);

#endif

