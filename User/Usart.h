#ifndef __USART_H_
#define __USART_H_


#include "NUC029xAN.h"
#include "UserCommon.h"


#define  PM5KT_UPCOM 		UART0
#define  PM5KT_DOWNCOM 		UART1

#define  PM5KT_UPCOM_IRQn 	UART0_IRQn
#define  PM5KT_DOWNCOM_IRQn UART1_IRQn

#define  PM5KT_UPCOM_RST 	UART0_RST
#define  PM5KT_DOWNCOM_RST 	UART1_RST

#define  PM5KT_DOWNCOM_IRQHandler   UART1_IRQHandler   
#define  PM5KT_UPCOM_IRQHandler   UART0_IRQHandler   

#define		PM5KT_PROTOCOL_BUFFER_SIZE    						(128 + 128)
#define		PM5KT_UPCOM_MTU_SIZE											78

#define		PM5KT_UPCOM_RECV_BUFFER_SIZE							(72 + 128)
#define		PM5KT_UPCOM_SEND_BUFFER_SIZE							(64 + 128)

#define		PM5KT_DOWNCOM_RECV_BUFFER_SIZE						(64 + 128)
#define		PM5KT_DOWNCOM_SEND_BUFFER_SIZE						(64 + 128)

#define		PM5KT_COM_PROTOCOL_ADDR_LOCAL							0

#define		PM5KT_COM_PROTOCOL_ADDR_COM1							0xA7					
#define		PM5KT_COM_PROTOCOL_ADDR_COM2							0xA8
#define		PM5KT_COM_PROTOCOL_ADDR_COM3							0xA9
#define		PM5KT_COM_PROTOCOL_ADDR_COM4							0xAA

#ifdef DEBUG
	#define		PM5KT_DEFAULT_UPCOM_BAUDRATE							115200
#else
	#define		PM5KT_DEFAULT_UPCOM_BAUDRATE							9600
#endif

#define		PM5KT_DEFAULT_DOWNCOM1_BAUDRATE						4800
#define		PM5KT_DEFAULT_DOWNCOM1_PARITY							'E'	
#define		PM5KT_DEFAULT_DOWNCOM2_BAUDRATE						4800
#define		PM5KT_DEFAULT_DOWNCOM2_PARITY							'E'	
#define		PM5KT_DEFAULT_DOWNCOM3_BAUDRATE						4800
#define		PM5KT_DEFAULT_DOWNCOM3_PARITY							'E'	
#define		PM5KT_DEFAULT_DOWNCOM4_BAUDRATE						9600
#define		PM5KT_DEFAULT_DOWNCOM4_PARITY							'N'	

typedef struct _PM5KT_UART_API
{
		uint8_t				ProtBuffer[PM5KT_PROTOCOL_BUFFER_SIZE];				
		uint16_t			ProtDataLen;
		
		uint8_t				IsUpComRecv;
		uint8_t				UpComRecvBuffer[PM5KT_UPCOM_RECV_BUFFER_SIZE];		
		uint16_t			UpComRecvBufferPushOffset;
		uint16_t			UpComRecvBufferPopOffset;
		
		uint8_t				UpComSendBuffer[PM5KT_UPCOM_SEND_BUFFER_SIZE];		
		uint16_t			UpComSendBufferPushOffset;
		uint16_t			UpComSendBufferPopOffset;
		
		uint8_t				DownComRecvBuffer[PM5KT_DOWNCOM_RECV_BUFFER_SIZE];
		uint16_t			DownComRecvBufferPushOffset;
		uint16_t			DownComRecvBufferPopOffset;
		
		uint8_t				DownComSendBuffer[PM5KT_DOWNCOM_SEND_BUFFER_SIZE];
		uint16_t			DownComSendBufferPushOffset;
		uint16_t			DownComSendBufferPopOffset;
}PM5KT_UART_API, *PPM5KT_UART_API;


extern PPM5KT_UART_API GetUartMagCallBack(void);
extern void UART_CS(uint8_t cs);
extern uint32_t UPCOM_Write(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
extern uint32_t DOWNCOM_Write(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
extern void UPCOM_UnInit(void);
extern void UPCOM_Init(uint32_t BaudRate);
//extern void UPCOM_InitEx(uint32_t BaudRate,char	Parity);
extern void DOWNCOM_UnInit(void);
extern void DOWNCOM_Init(uint32_t BaudRate,char	Parity);
extern void DOWNCOM_Init_DisenableIRQ(uint32_t BaudRate,char	Parity);





extern PM5KT_UART_API	gUart_Pm5ktApi;

#endif


