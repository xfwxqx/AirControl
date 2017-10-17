#ifndef _NRX8010__H_
#define _NRX8010__H_

#include "NUC029xAN.h"
#include "UserCommon.h"

#define RX8010_PIN_SDA P45 
#define RX8010_PIN_SCL P44  

extern unsigned char RX8010_Check(void);
extern unsigned char RX8010_SetTime(PPM5KT_COMM_TM rtc_tm);
extern unsigned char RX8010_GetTime(PPM5KT_COMM_TM rtc_tm);
extern void RX8010_Init(void);

#endif
