#include "InfraRecord.h"
#include "SoftTimer.h"
#include "string.h"
#include "UserCommon.h"

#define TIMER_REG_TDR_MAX 0xc00000

InfraRecordManagerDef gInfraMag;
uint32_t gu8ManuFacturer;

extern void InfraInit(void)
{
    memset(&gInfraMag,0,sizeof(InfraRecordManagerDef));
	//gInfraMag.IsFirstRecv = 1;  
}

extern void InfraDealPulse(uint32_t PinLevel,uint32_t TDRVal)
{
	volatile uint32_t TDRDiff;
    
	if(!gInfraMag.IsStartRecv)
		return;

	//接收到下降沿
	if(PinLevel==0){
		if(gInfraMag.IsFirstRecv){
			gInfraMag.IsFirstRecv = 0;
			gInfraMag.LastFailingEdgeTVal = TDRVal;
            if(!SoftTimerIsStart(TIMER_INFRA_RECV)){
                SoftTimerStart(TIMER_INFRA_RECV,4,0,NULL,NULL);
            }
		}else{
			if(gInfraMag.IsLastRisingEdge){
                if(gInfraMag.SaveRcd.RecordCnt<INFRA_RECORD_MAX){
                    gInfraMag.SaveRcd.Record[gInfraMag.SaveRcd.RecordCnt].PinLevel = 1;
                    TDRDiff = (TDRVal+TIMER_REG_TDR_MAX-gInfraMag.LastRisingEdgeTVal)&(~TIMER_REG_TDR_MAX);
					if(TDRDiff>750000){
						debug("PinLevel=1,TDRVal=%x,LastRisingEdgeTVal=%x,TDRDiff=%x\n",TDRVal,gInfraMag.LastRisingEdgeTVal,TDRDiff);
					}
                    gInfraMag.SaveRcd.Record[gInfraMag.SaveRcd.RecordCnt].DurationUS = TDRDiff/50;
                    gInfraMag.SaveRcd.RecordCnt++;
                }
				
				gInfraMag.LastFailingEdgeTVal = TDRVal;
			}
		}
		
		gInfraMag.IsLastFailingEdge = 1;
		gInfraMag.IsLastRisingEdge = 0;
		
	}else{
		if(gInfraMag.IsLastFailingEdge){
            if(gInfraMag.SaveRcd.RecordCnt<INFRA_RECORD_MAX){
                gInfraMag.SaveRcd.Record[gInfraMag.SaveRcd.RecordCnt].PinLevel = 0;
                TDRDiff = (TDRVal+TIMER_REG_TDR_MAX-gInfraMag.LastFailingEdgeTVal)&(~TIMER_REG_TDR_MAX);
				if(TDRDiff>750000){
					debug("PinLevel=0,TDRVal=%x,LastFailingEdgeTVal=%x,TDRDiff=%x\n",TDRVal,gInfraMag.LastFailingEdgeTVal,TDRDiff);
				}
                gInfraMag.SaveRcd.Record[gInfraMag.SaveRcd.RecordCnt].DurationUS = TDRDiff/50;
                gInfraMag.SaveRcd.RecordCnt++;
            }
			
			gInfraMag.LastRisingEdgeTVal = TDRVal;
		}

		gInfraMag.IsLastFailingEdge = 0;
		gInfraMag.IsLastRisingEdge = 1;
	}
}

extern uint32_t InfraGetRecordTemp(SaveRecordFormatDef **pSave)
{
	if(USE_RECORD_MEMORY_MUTEX_IS_LOCK)
		return FUNC_RET_FALSE;
	USE_RECORD_MEMORY_MUTEX_LOCK;
	//gInfraMag.IsSaveRcdMemoryUsing = 0;
    *pSave = &gInfraMag.SaveRcd;

	return FUNC_RET_TRUE;
}
extern void InfraFreeRecordTemp(SaveRecordFormatDef **pSave)
{
	USE_RECORD_MEMORY_MUTEX_UNLOCK;
}

extern uint32_t InfraGetRecvCode(SaveRecordFormatDef **pSave)
{
	volatile uint32_t Ret=FUNC_RET_FALSE;

	if(!SoftTimerIsStart(TIMER_INFRA_RECV)){
        gInfraMag.SaveRcd.RecordCnt = 0;
		gInfraMag.IsStartRecv = 1;
		gInfraMag.IsFirstRecv = 1;
	}
	
	if(SoftTimerIsOver(TIMER_INFRA_RECV)){
        *pSave = &gInfraMag.SaveRcd;
		gInfraMag.IsStartRecv = 0;
		Ret = FUNC_RET_TRUE;
	}
	
	return Ret;
}

extern uint32_t InfraGetManuID(void)
{
    return (uint32_t)gu8ManuFacturer;
}
extern uint32_t InfraSetManuID(uint32_t SetID)
{
    if(!ManuIDIsValid(SetID))
        return FUNC_RET_PARAM_ERROR;
    
    gu8ManuFacturer = SetID;
    return FUNC_RET_TRUE;
}
extern uint32_t InfraLearning(uint32_t ManuID,uint32_t CmdID)
{
	uint32_t i,Ret=FUNC_RET_FALSE;
	SaveRecordFormatDef *pSaveRecord=NULL;
    
    if(!ManuIDIsValid(ManuID))
        return FUNC_RET_PARAM_ERROR;
    if(!CMDIDIsValid(CmdID))
        return FUNC_RET_PARAM_ERROR;
    WatchDogFeed();
	if((FUNC_RET_TRUE == InfraGetRecvCode(&pSaveRecord))&&(pSaveRecord->RecordCnt>2)){
		
        debug("######### InfraGetRecvCode RecordCnt=%u ########\n",pSaveRecord->RecordCnt);
        for(i=0;i<pSaveRecord->RecordCnt;i++){
            if(i>=INFRA_RECORD_MAX)
                break;
            debug("{%d,%d} ",pSaveRecord->Record[i].PinLevel,pSaveRecord->Record[i].DurationUS);
        }
        debug("\n");
        WatchDogFeed();
        pSaveRecord->ManufacturerID = ManuID;
        pSaveRecord->LearningCmdID = CmdID;
        if(FUNC_RET_TRUE == SaveInfraLearningCmdToFlash(ManuID,CmdID,pSaveRecord,sizeof(SaveRecordFormatDef)))
            Ret = FUNC_RET_TRUE;
		debug("SaveInfraLearningCmdToFlash %s ManuID=%d CmdID=%d\n",(Ret==FUNC_RET_TRUE?"TRUE":"FALSE"),ManuID,CmdID);
	
	}
	return Ret;
}

//输出:*pSta的bit0-bit31分别对应命令1-32的自学习状态，1-已学习 0-未学习
extern uint32_t InfraGetLearningStatus(uint32_t *pSta)
{
	SaveRecordFormatDef *pTemp=NULL;
	uint32_t i,StaTemp;

	StaTemp = 0;
	for(i=0;i<CMDID_MAX;i++){
		if(FUNC_RET_TRUE == InfraGetRecordTemp(&pTemp)){
			if(FUNC_RET_TRUE == GetInfraLearningCmdFromFlash(InfraGetManuID(),i,pTemp,sizeof(SaveRecordFormatDef))){
				if(pTemp->RecordCnt>2){
					StaTemp |= (0x01<<i);
				}
			}
			InfraFreeRecordTemp(&pTemp);
		}
		InfraFreeRecordTemp(&pTemp);
	}

	if(pSta!=NULL){
		*pSta = StaTemp;
	}
    return FUNC_RET_TRUE;
}






