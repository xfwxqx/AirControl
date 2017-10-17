#ifndef __RN8209C_H_
#define __RN8209C_H_

//#include "UserCommon.h"
#include "NUC029xAN.h"

//函数返回值定义
#define FUNC_RET_SUCC       						0
#define FUNC_RET_PARAM_ERR  						1
#define FUNC_RET_FAIL       						2

#define RN8209C_BAUD    								4800
#define RN8209C_PARITY    							'e'


typedef struct __attribute__((packed))
{	
		union __attribute__((packed))
		{
				struct __attribute__((packed))
				{
						volatile uint16_t	SYSCON;
						volatile uint16_t	EMUCON;
						volatile uint16_t	HFConst;
						volatile uint16_t	PStart;
						volatile uint16_t	DStart;
						volatile uint16_t	GPQA;
						volatile uint16_t	GPQB;
						volatile uint16_t	PhsA;
						volatile uint16_t	PhsB;
						volatile uint16_t	QPhsCal;
						volatile uint16_t	APOSA;
						volatile uint16_t	APOSB;
						volatile uint16_t	RPOSA;
						volatile uint16_t	RPOSB;
						volatile uint16_t	IARMSOS;
						volatile uint16_t	IBRMSOS;
						volatile uint16_t	IBGain;
						volatile uint16_t	D2FPL;
						volatile uint16_t	D2FPH;
						volatile uint16_t	DCIAH;
						volatile uint16_t	DCIBH;
						volatile uint16_t	DCUH;
						volatile uint16_t	DCL;
						volatile uint16_t	EMUCON2;
						volatile uint16_t	CHKSUM;
						volatile uint16_t	RESERVE;//四字节对齐
				}PARAM_List;
				volatile uint16_t __packed PARAM_Union[26];
		}UNION;
}RN8209C_INIT_PARAM,*PRN8209C_INIT_PARAM;

typedef struct __attribute__((packed))
{	
    float   Kv;
    float   Ki;
    float   Kp;
    float   In;
    float   Un;
}RN8209C_CALI_PARAM,*PRN8209C_CALI_PARAM;

typedef struct __attribute__((packed))
{
    RN8209C_INIT_PARAM InitParam;
    RN8209C_CALI_PARAM CaliParam;
}RN8209C_DEF,*PRN8209C_DEF;

typedef struct __attribute__((packed))
{
    RN8209C_DEF Def[3];
}RN8209C,*PRN8209C;



/*************************************** 
	RN8209C 特殊命令
***************************************/
extern uint32_t RN8209C_SoftReset(void);
extern uint32_t RN8209C_WriteEnable(void);
extern uint32_t RN8209C_WriteProtect(void); 
/*************************************** 
	RN8209C 内部寄存器定义
***************************************/

//校表参数和计量控制寄存器
#define	RN8209C_REG_SYSCON 											0x00	//系统控制寄存器,写保护
#define	RN8209C_REG_SYSCON_LEN 									2 		//占两个字节
#define	RN8209C_REG_EMUCON 											0x01	//计量控制寄存器,写保护
#define	RN8209C_REG_EMUCON_LEN 									2 		//占两个字节
#define	RN8209C_REG_HFConst 										0x02	//脉冲频率寄存器,写保护
#define	RN8209C_REG_HFConst_LEN 								2 		//占两个字节
#define	RN8209C_REG_PStart 											0x03	//有功启动功率设置,写保护
#define	RN8209C_REG_PStart_LEN 									2 		//占两个字节
#define	RN8209C_REG_DStart 											0x04	//自定义电能启动功率设置,写保护
#define	RN8209C_REG_DStart_LEN 									2 		//占两个字节
#define	RN8209C_REG_GPQA 												0x05	//通道A功率增益校正寄存器,写保护
#define	RN8209C_REG_GPQA_LEN 										2 		//占两个字节
#define	RN8209C_REG_GPQB 												0x06	//通道B功率增益校正寄存器,写保护
#define	RN8209C_REG_GPQB_LEN 										2 		//占两个字节
#define	RN8209C_REG_PhsA 												0x07	//通道A相位校正寄存器,写保护
#define	RN8209C_REG_PhsA_LEN 										1 		//占一个字节
#define	RN8209C_REG_PhsB 												0x08	//通道B相位校正寄存器,写保护
#define	RN8209C_REG_PhsB_LEN 										1 		//占一个字节
#define	RN8209C_REG_QPHSCAL 										0x09	//无功相位补偿,写保护
#define	RN8209C_REG_QPHSCAL_LEN 								2 		//占两个字节
#define	RN8209C_REG_APOSA 											0x0a	//通道A有功功率Offset校正寄存器,写保护
#define	RN8209C_REG_APOSA_LEN 									2 		//占两个字节
#define	RN8209C_REG_APOSB 											0x0b	//通道B有功功率Offset校正寄存器,写保护
#define	RN8209C_REG_APOSB_LEN 									2 		//占两个字节
#define	RN8209C_REG_RPOSA 											0x0c	//通道A无功功率Offset校正寄存器,写保护
#define	RN8209C_REG_RPOSA_LEN 									2 		//占两个字节
#define	RN8209C_REG_RPOSB 											0x0d	//通道B无功功率Offset校正寄存器,写保护
#define	RN8209C_REG_RPOSB_LEN 									2 		//占两个字节
#define	RN8209C_REG_IARMSOS 										0x0e	//电流通道A有效值Offset补偿,写保护
#define	RN8209C_REG_IARMSOS_LEN 								2 		//占两个字节
#define	RN8209C_REG_IBRMSOS 										0x0f	//电流通道B有效值Offset补偿,写保护
#define	RN8209C_REG_IBRMSOS_LEN 								2 		//占两个字节
#define	RN8209C_REG_IBGain 											0x10	//电流通道B增益设置,写保护
#define	RN8209C_REG_IBGain_LEN 									2 		//占两个字节
#define	RN8209C_REG_D2FPL 											0x11	//自定义功率寄存器D2FP的低16bit,写保护
#define	RN8209C_REG_D2FPL_LEN 									2 		//占两个字节
#define	RN8209C_REG_D2FPH 											0x12	//自定义功率寄存器D2FP的高16bit,写保护(需要先写D2FPH,再写D2FPL)
#define	RN8209C_REG_D2FPH_LEN 									2 		//占两个字节
#define	RN8209C_REG_DCIAH 											0x13	//IA通道直流Offset校正寄存器的高16bit,写保护
#define	RN8209C_REG_DCIAH_LEN 									2 		//占两个字节
#define	RN8209C_REG_DCIBH 											0x14	//IB通道直流Offset校正寄存器的高16bit,写保护
#define	RN8209C_REG_DCIBH_LEN 									2 		//占两个字节
#define	RN8209C_REG_DCUH 												0x15	//U通道直流Offset校正寄存器的高16bit,写保护
#define	RN8209C_REG_DCUH_LEN 										2 		//占两个字节
#define	RN8209C_REG_DCL 												0x16	//三个直流Offset校正寄存器的低4bit,写保护
#define	RN8209C_REG_DCL_LEN 										2 		//占两个字节
#define	RN8209C_REG_EMUCON2 										0x17	//计量控制寄存器2,写保护
#define	RN8209C_REG_EMUCON2_LEN 								2 		//占两个字节

//计量参数和状态寄存器
#define	RN8209C_REG_PFCnt 											0x20	//快速有功脉冲计数,写保护
#define	RN8209C_REG_PFCnt_LEN 									2 		//占两个字节
#define	RN8209C_REG_DFcnt 											0x21	//自定义电能快速脉冲计数,写保护
#define	RN8209C_REG_DFcnt_LEN 									2 		//占两个字节
#define	RN8209C_REG_IARMS 											0x22	//通道A电流有效值
#define	RN8209C_REG_IARMS_LEN 									3 		//占三个字节
#define	RN8209C_REG_IBRMS 											0x23	//通道B电流有效值
#define	RN8209C_REG_IBRMS_LEN 									3 		//占三个字节
#define	RN8209C_REG_URMS 												0x24	//电压有效值
#define	RN8209C_REG_URMS_LEN 										3 		//占三字节
#define	RN8209C_REG_UFreq 											0x25	//电压频率
#define	RN8209C_REG_UFreq_LEN 									2 		//占两个字节
#define	RN8209C_REG_PowerPA 										0x26	//有功功率A 
#define	RN8209C_REG_PowerPA_LEN 								4 		//占四个字节
#define	RN8209C_REG_PowerPB 										0x27	//有功功率B 
#define	RN8209C_REG_PowerPB_LEN 								4 		//占四个字节
#define	RN8209C_REG_PowerQ 											0x28	//无功功率
#define	RN8209C_REG_PowerQ_LEN 									4 		//占四个字节
#define	RN8209C_REG_EnergyP 										0x29	//有功能量，默认读后不清零
#define	RN8209C_REG_EnergyP_LEN 								3 		//占三个字节
#define	RN8209C_REG_EnergyP2 										0x2a	//有功能量，默认读后清零
#define	RN8209C_REG_EnergyP2_LEN 								3 		//占三个字节
#define	RN8209C_REG_EnergyD 										0x2b	//无功能量，默认读后不清零
#define	RN8209C_REG_EnergyD_LEN 								3 		//占三个字节
#define	RN8209C_REG_EnergyD2 										0x2c	//无功能量，默认读后清零
#define	RN8209C_REG_EnergyD2_LEN 								3 		//占三个字节
#define	RN8209C_REG_EMUStatus 									0x2d	//计量状态和校验和寄存器
#define	RN8209C_REG_EMUStatus_LEN 							3 		//占三个字节
#define	RN8209C_REG_SPL_IA 											0x30	//IA通道ADC采样值
#define	RN8209C_REG_SPL_IA_LEN 									3 		//占三个字节
#define	RN8209C_REG_SPL_IB 											0x31	//IB通道ADC采样值
#define	RN8209C_REG_SPL_IB_LEN 									3 		//占三个字节
#define	RN8209C_REG_SPL_U 											0x32	//U通道ADC采样值
#define	RN8209C_REG_SPL_U_LEN 									3 		//占三个字节
#define	RN8209C_REG_UFreq2 											0x35	//电压频率寄存器2
#define	RN8209C_REG_UFreq2_LEN 									3 		//占三个字节

//中断寄存器
#define	RN8209C_REG_IE 													0x40	//中断允许寄存器,写保护
#define	RN8209C_REG_IE_LEN 											1 		//占一个字节
#define	RN8209C_REG_IF 													0x41	//中断标志寄存器,读后清零
#define	RN8209C_REG_IF_LEN 											1 		//占两个字节
#define	RN8209C_REG_RIF 												0x42	//复位中断状态寄存器,读后清零
#define	RN8209C_REG_RIF_LEN 										1 		//占两个字节

//系统状态寄存器
#define	RN8209C_REG_SysStatus 									0x43	//系统状态寄存器
#define	RN8209C_REG_SysStatus_LEN 							1 		//占一个字节
#define	RN8209C_REG_RDataS 											0x44	//上一次SPI/UART读出的数据
#define	RN8209C_REG_RData_LEN 									4 		//占四个字节
#define	RN8209C_REG_WData 											0x45	//上一次SPI/UART写入的数据
#define	RN8209C_REG_WData_LEN 									2 		//占两个字节
#define	RN8209C_REG_DeviceID 										0x7f	//DEVICE ID:820900H
#define	RN8209C_REG_DeviceID_LEN 								3 		//占三个字节

/***************************RN8209C详细寄存器说明*******************************************/
//SYSCON 
#define REG_SYSCON_PGAIA_POS										0
#define REG_SYSCON_PGAIA_MSK										((uint16_t)0x03<<REG_SYSCON_PGAIA_POS)

#define REG_SYSCON_PGAIA_1											(0<<REG_SYSCON_PGAIA_POS)
#define REG_SYSCON_PGAIA_2											(1<<REG_SYSCON_PGAIA_POS)
#define REG_SYSCON_PGAIA_8											(2<<REG_SYSCON_PGAIA_POS)
#define REG_SYSCON_PGAIA_16											(3<<REG_SYSCON_PGAIA_POS)

#define REG_SYSCON_PGAU_POS											2
#define REG_SYSCON_PGAU_MSK											((uint16_t)0x03<<REG_SYSCON_PGAIA_POS)

#define REG_SYSCON_PGAU_1												(0<<REG_SYSCON_PGAIA_POS)
#define REG_SYSCON_PGAU_2												(1<<REG_SYSCON_PGAIA_POS)
#define REG_SYSCON_PGAU_4												(2<<REG_SYSCON_PGAIA_POS)
//#define REG_SYSCON_PGAU_16										(3<<REG_SYSCON_PGAIA_POS)

#define REG_SYSCON_PGAIB_POS										4
#define REG_SYSCON_PGAIB_MSK										((uint16_t)0x03<<REG_SYSCON_PGAIB_POS)
#define REG_SYSCON_PGAIB_1											(0<<REG_SYSCON_PGAIB_POS)
#define REG_SYSCON_PGAIB_2											(1<<REG_SYSCON_PGAIB_POS)
#define REG_SYSCON_PGAIB_4											(2<<REG_SYSCON_PGAIB_POS)
//#define REG_SYSCON_PGAIB_1										(0<<REG_SYSCON_PGAIB_POS)

#define REG_SYSCON_ADC2ON_POS										6
#define REG_SYSCON_ADC2ON_MSK										((uint16_t)0x01<<REG_SYSCON_ADC2ON_POS)
#define REG_SYSCON_ADC2ON_ON										(1<<REG_SYSCON_ADC2ON_POS)
#define REG_SYSCON_ADC2ON_OFF										(0<<REG_SYSCON_ADC2ON_POS)

//

extern uint16_t RN8209C_GetCheckSum(void);    
extern void RN8209C_WaitDataUpdate(void);
extern uint8_t RN8209C_TestDataUpdate(void);
extern uint32_t RN8209C_Init(PRN8209C pRn8209);
extern uint32_t RN8209C_Init_ResetParam(PRN8209C_INIT_PARAM pInitParam);

extern uint32_t RN8209C_DCOffsetCalibration(PRN8209C_DEF pRnDef);
extern uint32_t RN8209C_DCGainCalibration(PRN8209C_DEF pRn8209c);

extern uint32_t RN8209C_GetCurrVolAverage(uint32_t *pIa,uint32_t *pIb,uint32_t *pU);
extern uint32_t GetSPLIB(uint32_t *pIb);
extern uint32_t GetSPLIBAverage(uint32_t *pIb);
extern uint32_t RN8209C_GetFreqHz(float *pHz);
extern uint32_t RN8209C_GetIa(PRN8209C_CALI_PARAM pCali,float *pI);
extern uint32_t RN8209C_GetIb(PRN8209C_CALI_PARAM pCali,float *pI);
extern uint32_t RN8209C_GetU(PRN8209C_CALI_PARAM pCali,float *pU);
extern uint32_t RN8209C_GetPa(PRN8209C_CALI_PARAM pCali,float *pP);
extern uint32_t RN8209C_GetPb(PRN8209C_CALI_PARAM pCali,float *pP);
extern uint32_t RN8209C_GetEa(PRN8209C_CALI_PARAM pCali,double *pE);
extern uint32_t RN8209C_GetEa_NoRet(PRN8209C_CALI_PARAM pCali,double *pE);
extern uint32_t RN8209C_GetEb(PRN8209C_CALI_PARAM pCali,double *pE);
extern uint32_t RN8209C_ACGainCalibration(PRN8209C_DEF pRn8209c);
extern uint32_t RN8209C_GetEb_NoRet(PRN8209C_CALI_PARAM pCali,double *pE);


//PGAIN=-ERR/( 1+ERR)
//PGAIN>0,GPQA=PGAIN*2^15;
//PGAIN<0,GPQA=PGAIN*2^15+2^16;
extern uint32_t RN8209C_ACGainCalibration(PRN8209C_DEF pRn8209c);
extern uint32_t RN8209C_ACPhaseCalibration(PRN8209C_DEF pRn8209c);
extern uint32_t RN8209C_ACOffsetCalibration(PRN8209C_DEF pRn8209c);
extern uint32_t RN8209C_ACReactiveCalibration(PRN8209C_DEF pRn8209c);
extern uint32_t RN8209C_ACCurrentOffsetCalibration(PRN8209C_DEF pRn8209c);
//定时检查下电能芯片的参数是否被篡改
extern uint32_t RN8209C_RegularCheckParam(PRN8209C pRn8209);

#endif
