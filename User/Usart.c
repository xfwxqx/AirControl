/******************************************************************************
 * @file     	Usart.c
 * @version  	V1.00
 * $Revision: 3 $
 * $Date: 		17/07/04 11:45a $
 * @brief    	Uart
 * @auther		ljj
 * @note
 * Copyright (C) 2017 Jsyaao Technology Corp. All rights reserved.
*****************************************************************************/
#include "Usart.h"
#include "UserCommon.h"

PM5KT_UART_API	gUart_Pm5ktApi;
extern PPM5KT_UART_API GetUartMagCallBack(void)
{
    return &gUart_Pm5ktApi;
}
extern char Uart_ComRecvToCycBuffer(uint8_t RecvData, uint8_t *pRecvBuffer, uint16_t *pPushOffset, uint16_t PopOffset, uint16_t BufferSize)
{
		uint16_t		DataLen;

		if ((pRecvBuffer!=NULL) && (pPushOffset!=NULL))
		{
				pRecvBuffer[*pPushOffset]	= RecvData;
				*pPushOffset				= *pPushOffset+1;

				if (*pPushOffset>=BufferSize)
				{
						*pPushOffset	= 0;
				}
				DataLen		= (*pPushOffset>PopOffset) ? (*pPushOffset-PopOffset):(*pPushOffset+BufferSize-PopOffset);

				return	(DataLen>=BufferSize/2) ? 1:0;
		}
		else
		{
				return	0;
		}
}

void PM5KT_UPCOM_IRQHandler(void)   //串口0中断服务程序
{
	uint8_t RecvData = 0xFF;
	uint32_t u32IntSts = PM5KT_UPCOM->ISR;

	if(u32IntSts & UART_ISR_RDA_INT_Msk)
	{
			while(UART_IS_RX_READY(PM5KT_UPCOM))
			{
					/* Get the character from UART Buffer */
					RecvData = UART_READ(PM5KT_UPCOM);
					WatchDogFeed();
					if(gUart_Pm5ktApi.UpComRecvBufferPushOffset+1 == gUart_Pm5ktApi.UpComRecvBufferPopOffset)
					{	
							
					}

					gUart_Pm5ktApi.UpComRecvBuffer[gUart_Pm5ktApi.UpComRecvBufferPushOffset++]	= RecvData;

					if (gUart_Pm5ktApi.UpComRecvBufferPushOffset>=PM5KT_UPCOM_RECV_BUFFER_SIZE)
					{
							gUart_Pm5ktApi.UpComRecvBufferPushOffset	= 0;
					}

					if((gUart_Pm5ktApi.UpComRecvBufferPushOffset>gUart_Pm5ktApi.UpComRecvBufferPopOffset) ? 
                        (gUart_Pm5ktApi.UpComRecvBufferPushOffset-gUart_Pm5ktApi.UpComRecvBufferPopOffset):
                    (gUart_Pm5ktApi.UpComRecvBufferPushOffset+PM5KT_UPCOM_RECV_BUFFER_SIZE-gUart_Pm5ktApi.UpComRecvBufferPopOffset)>=PM5KT_UPCOM_RECV_BUFFER_SIZE/10)
					{
							gUart_Pm5ktApi.IsUpComRecv	= 1;
					}
					if(UART_GET_RX_EMPTY(PM5KT_UPCOM))
							gUart_Pm5ktApi.IsUpComRecv	= 1;
					
			}
		
	}
	if(u32IntSts & UART_ISR_TOUT_INT_Msk)
	{
			RecvData = UART_READ(PM5KT_UPCOM);
			gUart_Pm5ktApi.IsUpComRecv	= 1;
	}		
}

void PM5KT_DOWNCOM_IRQHandler(void)   //串口1中断服务程序
{

	uint8_t RecvData = 0xFF;
	uint32_t u32IntSts = PM5KT_DOWNCOM->ISR;

	if(u32IntSts & UART_ISR_RDA_INT_Msk)
	{
			while(UART_IS_RX_READY(PM5KT_DOWNCOM))
			{
					/* Get the character from UART Buffer */
					RecvData = UART_READ(PM5KT_DOWNCOM);
					WatchDogFeed();
					if (Uart_ComRecvToCycBuffer(RecvData, gUart_Pm5ktApi.DownComRecvBuffer, &gUart_Pm5ktApi.DownComRecvBufferPushOffset, gUart_Pm5ktApi.DownComRecvBufferPopOffset, PM5KT_DOWNCOM_RECV_BUFFER_SIZE))
					{
							//Pm5ktLocalManager.IsCom1Recv	= TRUE;
					}
			}
	}
	if(u32IntSts & UART_ISR_TOUT_INT_Msk)
	{
		
	}

		
} 
//cs=4 红外

extern void UART_CS(uint8_t cs)
{
		switch(cs)
		{
			case 1:
			{
					DOWNCOM_UnInit();
					DOWNCOM_Init_DisenableIRQ(PM5KT_DEFAULT_DOWNCOM1_BAUDRATE,PM5KT_DEFAULT_DOWNCOM1_PARITY);
					GPIO_SetMode(P0, BIT3, GPIO_PMD_OUTPUT);
					GPIO_SetMode(P4, BIT1, GPIO_PMD_OUTPUT);
					
					P03 = 1;
					P41 = 0;
					
			}
			break;
			case 2:
			{
					DOWNCOM_UnInit();
					DOWNCOM_Init_DisenableIRQ(PM5KT_DEFAULT_DOWNCOM2_BAUDRATE,PM5KT_DEFAULT_DOWNCOM2_PARITY);
					GPIO_SetMode(P0, BIT3, GPIO_PMD_OUTPUT);
					GPIO_SetMode(P4, BIT1, GPIO_PMD_OUTPUT);
					
					P03 = 0;
					P41 = 0;
					
			}
			break;
			case 3:
			{
					DOWNCOM_UnInit();
					DOWNCOM_Init_DisenableIRQ(PM5KT_DEFAULT_DOWNCOM3_BAUDRATE,PM5KT_DEFAULT_DOWNCOM3_PARITY);
					GPIO_SetMode(P0, BIT3, GPIO_PMD_OUTPUT);
					GPIO_SetMode(P4, BIT1, GPIO_PMD_OUTPUT);
					
					P03 = 0;
					P41 = 1;
					
			}
			break;
			case 4:
			{
					DOWNCOM_UnInit();
					DOWNCOM_Init(PM5KT_DEFAULT_DOWNCOM4_BAUDRATE,PM5KT_DEFAULT_DOWNCOM4_PARITY);
					GPIO_SetMode(P0, BIT3, GPIO_PMD_OUTPUT);
					GPIO_SetMode(P4, BIT1, GPIO_PMD_OUTPUT);
					
					P03 = 1;
					P41 = 1;
					
			}
			break;
			default:
			{
                    DOWNCOM_UnInit();
					GPIO_SetMode(P0, BIT3, GPIO_PMD_OUTPUT);
					GPIO_SetMode(P4, BIT1, GPIO_PMD_OUTPUT);
					
					P03 = 1;
					P41 = 1;
				
			}
			break;
		}
}

extern uint32_t UPCOM_Write(uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
		return UART_Write(PM5KT_UPCOM, pu8TxBuf,u32WriteBytes);
}

extern uint32_t DOWNCOM_Write(uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
		return UART_Write(PM5KT_DOWNCOM, pu8TxBuf,u32WriteBytes);
}

extern void UPCOM_UnInit(void)
{
		UART_DISABLE_INT(PM5KT_UPCOM, (UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_RTO_IEN_Msk));
    NVIC_DisableIRQ(PM5KT_UPCOM_IRQn);
}

extern void UPCOM_Init(uint32_t BaudRate)
{
     SYS_ResetModule(PM5KT_UPCOM_RST);

    UART_Open(PM5KT_UPCOM, BaudRate);
    UART_ENABLE_INT(PM5KT_UPCOM, (UART_IER_RDA_IEN_Msk));
    NVIC_EnableIRQ(PM5KT_UPCOM_IRQn);
}

extern void UPCOM_InitEx(uint32_t BaudRate,char	Parity)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    uint8_t u8UartClkSrcSel;
    uint32_t u32ClkTbl[4] = {__HXT, 0, 0, __HIRC};
    uint32_t u32Baud_Div = 0;
    
    SYS_ResetModule(PM5KT_UPCOM_RST);

    /* Get UART clock source selection */
    u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART_S_Msk) >> CLK_CLKSEL1_UART_S_Pos;

    /* Select UART function */
    UART0->FUN_SEL = UART_FUNC_SEL_UART;

    /* Set UART line configuration */
		if ((Parity=='N') || (Parity=='n'))
		{
			PM5KT_UPCOM->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
		}
		else if ((Parity=='E') || (Parity=='e'))
		{
			PM5KT_UPCOM->LCR = UART_WORD_LEN_8 | UART_PARITY_EVEN | UART_STOP_BIT_1;
		}
		else if ((Parity=='O') || (Parity=='o'))
		{
			PM5KT_UPCOM->LCR = UART_WORD_LEN_8 | UART_PARITY_ODD | UART_STOP_BIT_1;
		}

    /* Set UART Rx and RTS trigger level */
    PM5KT_UPCOM->FCR &= ~(UART_FCR_RFITL_Msk | UART_FCR_RTS_TRI_LEV_Msk);

    /* Get PLL clock frequency if UART clock source selection is PLL */
    if(u8UartClkSrcSel == 1)
        u32ClkTbl[u8UartClkSrcSel] = CLK_GetPLLClockFreq();

    /* Set UART baud rate */

    u32Baud_Div = UART_BAUD_MODE2_DIVIDER(u32ClkTbl[u8UartClkSrcSel], BaudRate);

    if(u32Baud_Div > 0xFFFF)
        PM5KT_UPCOM->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(u32ClkTbl[u8UartClkSrcSel], BaudRate));
    else
        PM5KT_UPCOM->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);

	//UART_ENABLE_INT(UART0, (UART_IER_RDA_IEN_Msk));
    //NVIC_EnableIRQ(UART0_IRQn);
}
extern void DOWNCOM_UnInit(void)
{
		UART_DISABLE_INT(PM5KT_DOWNCOM, (UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_RTO_IEN_Msk));
    NVIC_DisableIRQ(PM5KT_DOWNCOM_IRQn);
}
extern void DOWNCOM_Init(uint32_t BaudRate,char	Parity)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    uint8_t u8UartClkSrcSel;
    uint32_t u32ClkTbl[4] = {__HXT, 0, 0, __HIRC};
    uint32_t u32Baud_Div = 0;
    
    SYS_ResetModule(PM5KT_DOWNCOM_RST);

    /* Get UART clock source selection */
    u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART_S_Msk) >> CLK_CLKSEL1_UART_S_Pos;

    /* Select UART function */
    PM5KT_DOWNCOM->FUN_SEL = UART_FUNC_SEL_UART;

    /* Set UART line configuration */
		if ((Parity=='N') || (Parity=='n'))
		{
			PM5KT_DOWNCOM->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
		}
		else if ((Parity=='E') || (Parity=='e'))
		{
			PM5KT_DOWNCOM->LCR = UART_WORD_LEN_8 | UART_PARITY_EVEN | UART_STOP_BIT_1;
		}
		else if ((Parity=='O') || (Parity=='o'))
		{
			PM5KT_DOWNCOM->LCR = UART_WORD_LEN_8 | UART_PARITY_ODD | UART_STOP_BIT_1;
		}

    /* Set UART Rx and RTS trigger level */
    PM5KT_DOWNCOM->FCR &= ~(UART_FCR_RFITL_Msk | UART_FCR_RTS_TRI_LEV_Msk);

    /* Get PLL clock frequency if UART clock source selection is PLL */
    if(u8UartClkSrcSel == 1)
        u32ClkTbl[u8UartClkSrcSel] = CLK_GetPLLClockFreq();

    /* Set UART baud rate */

    u32Baud_Div = UART_BAUD_MODE2_DIVIDER(u32ClkTbl[u8UartClkSrcSel], BaudRate);

    if(u32Baud_Div > 0xFFFF)
        PM5KT_DOWNCOM->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(u32ClkTbl[u8UartClkSrcSel], BaudRate));
    else
        PM5KT_DOWNCOM->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);

	UART_ENABLE_INT(PM5KT_DOWNCOM, (UART_IER_RDA_IEN_Msk));
    NVIC_EnableIRQ(PM5KT_DOWNCOM_IRQn);
}

extern void DOWNCOM_Init_DisenableIRQ(uint32_t BaudRate,char	Parity)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    uint8_t u8UartClkSrcSel;
    uint32_t u32ClkTbl[4] = {__HXT, 0, 0, __HIRC};
    uint32_t u32Baud_Div = 0;
    
    SYS_ResetModule(PM5KT_DOWNCOM_RST);

    /* Get UART clock source selection */
    u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART_S_Msk) >> CLK_CLKSEL1_UART_S_Pos;

    /* Select UART function */
    PM5KT_DOWNCOM->FUN_SEL = UART_FUNC_SEL_UART;

    /* Set UART line configuration */
		if ((Parity=='N') || (Parity=='n'))
		{
			PM5KT_DOWNCOM->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
		}
		else if ((Parity=='E') || (Parity=='e'))
		{
			PM5KT_DOWNCOM->LCR = UART_WORD_LEN_8 | UART_PARITY_EVEN | UART_STOP_BIT_1;
		}
		else if ((Parity=='O') || (Parity=='o'))
		{
			PM5KT_DOWNCOM->LCR = UART_WORD_LEN_8 | UART_PARITY_ODD | UART_STOP_BIT_1;
		}

    /* Set UART Rx and RTS trigger level */
    PM5KT_DOWNCOM->FCR &= ~(UART_FCR_RFITL_Msk | UART_FCR_RTS_TRI_LEV_Msk);

    /* Get PLL clock frequency if UART clock source selection is PLL */
    if(u8UartClkSrcSel == 1)
        u32ClkTbl[u8UartClkSrcSel] = CLK_GetPLLClockFreq();

    /* Set UART baud rate */

    u32Baud_Div = UART_BAUD_MODE2_DIVIDER(u32ClkTbl[u8UartClkSrcSel], BaudRate);

    if(u32Baud_Div > 0xFFFF)
        PM5KT_DOWNCOM->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(u32ClkTbl[u8UartClkSrcSel], BaudRate));
    else
        PM5KT_DOWNCOM->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);

	//UART_ENABLE_INT(UART0, (UART_IER_RDA_IEN_Msk));
    //NVIC_EnableIRQ(UART0_IRQn);
}




