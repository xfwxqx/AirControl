#include "SoftTimer.h"

#define TRUE 1
#define FALSE 0
#define TimerModeIsValid(x) (((x)==0)||((x)==1))

static SoftTimer gSoftTimers[TIMER_NUM];

extern void SoftTimerInit(void)
{
	uint32_t i;
	
	for(i=0; i<TIMER_NUM; i++)
	{
		SoftTimerStop(i);
	}
}

extern void SoftTimerStart(uint32_t TimerID, uint32_t n100Ms,uint32_t Mode,PTimeOutCallBack pTimeOutCallBack,void *pHandle)
{
	if (TimerID >= TIMER_NUM)
	{
		return;
	}

	SOFT_Timer_Disable;
	gSoftTimers[TimerID].TimerCnt = 0;
	gSoftTimers[TimerID].TimerOut = n100Ms;
	if(TimerModeIsValid(Mode))
		gSoftTimers[TimerID].TimerMode = Mode;
	else
		gSoftTimers[TimerID].TimerMode = 0;
	gSoftTimers[TimerID].pFunc = pTimeOutCallBack;
	gSoftTimers[TimerID].pFuncParam = pHandle;
	gSoftTimers[TimerID].TimerEnable = TRUE;
	
	SOFT_Timer_Enable;
	
	return;
}

extern void SoftTimerStop(uint32_t TimerID)
{
	if (TimerID >= TIMER_NUM)
	{
		return;
	}
	
	SOFT_Timer_Disable;
	gSoftTimers[TimerID].TimerCnt = 0;
	gSoftTimers[TimerID].TimerOut = 0;
	gSoftTimers[TimerID].pFunc = NULL;
	gSoftTimers[TimerID].pFuncParam = NULL;
	gSoftTimers[TimerID].TimerEnable = FALSE;
	SOFT_Timer_Enable;
	
	return;
}

extern uint32_t SoftTimerIsOver(uint32_t TimerID)
{
	uint32_t bRet = FALSE;

	
	if (TimerID >= TIMER_NUM)
	{
		return FALSE;
	}
	
	SOFT_Timer_Disable;
	if (gSoftTimers[TimerID].TimerEnable)
	{
		if (gSoftTimers[TimerID].TimerCnt > gSoftTimers[TimerID].TimerOut)
		{

			gSoftTimers[TimerID].TimerEnable = FALSE;
			bRet = TRUE;
			
			
		}
	}
	
	SOFT_Timer_Enable;
	
	return bRet;
}

extern uint32_t SoftTimerIsStart(uint32_t TimerID)
{
	if (TimerID >= TIMER_NUM)
	{
		return FALSE;
	}
	
	return gSoftTimers[TimerID].TimerEnable;
}

extern void SoftTimerIntervalProcess(void)
{
	uint32_t i;
	PTimeOutCallBack pTimeOutCallBack=NULL;
	void *pHandle=NULL;	
	
	for(i=0; i<TIMER_NUM; i++)
	{
		if(gSoftTimers[i].TimerEnable == TRUE){
			gSoftTimers[i].TimerCnt++;
            if ((gSoftTimers[i].TimerCnt > gSoftTimers[i].TimerOut)&&(gSoftTimers[i].TimerMode==1))
			{
				gSoftTimers[i].TimerEnable = FALSE;
				pTimeOutCallBack = gSoftTimers[i].pFunc;
				pHandle = gSoftTimers[i].pFuncParam;
				if(pTimeOutCallBack!=NULL){
					(*pTimeOutCallBack)(pHandle);
				}
			}
		}	
	}
}
