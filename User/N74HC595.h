#ifndef _74HC959__H_
#define _74HC959__H_

#include "NUC029xAN.h"

#define 	SDIN_1 						P06
#define 	SCLK_1 						P07
#define 	STB_1 						P20


//学习灯状态控制:mode=0时，根据IsRun控制学习灯亮灭；mode=1时，学习灯闪烁；
extern void LearnLightCtrl(char mode,char IsRun);
extern void RunLightCtrl(char IsRun);
//extern void HC595_CtrlLight(char LightNum);
extern void HC595_LedDisp(unsigned int LedNumVal);
extern void HC595_Init(void);
extern void FailureLightCtrl(char IsRun);
extern void CommunicationLightCtrl(char IsRun);


#endif
