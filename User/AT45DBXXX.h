#ifndef __AT45DBXXX_H
#define __AT45DBXXX_H


//#include "UserCommon.h"
#include "NUC029xAN.h"

#define AT45DBxxxD_PIN_MOSI 				P15
#define AT45DBxxxD_PIN_MISO 				P16
#define AT45DBxxxD_PIN_SCK 					P17
#define AT45DBxxxD_PIN_CS 					P14
#define AT45DBxxxD_PIN_RST 					P42

//*****************************************************************************
//
//! \addtogroup AT45DBxxxD_CHIP_TYPE
//! AT45DBxxxD SPI DataFlash Chip Type Define.
//! @{
//
//*****************************************************************************
#define AT45DB011D     							1  // 1 buffer,264B/page,2KB block,32KB sector,128KB chip
#define AT45DB021D     							2  // 2 buffer,264B/page,2KB block,32KB sector,256KB chip
#define AT45DB041D     							4  // 2 buffer,264B/page,2KB block,64KB sector,512KB chip
#define AT45DB081D     							8  // 2 buffer,264B/page,2KB block,64KB sector,1MB chip
#define AT45DB161D     							16 // 2 buffer,528B/page,4KB block,128KB sector,2MB chip
#define AT45DB321D     							32 // 2 buffer,528B/page,4KB block,128KB sector,4MB chip
#define AT45DB642D     							64 // 2 buffer,1056B/page,8KB block,256KB sector,8MB chip

//*****************************************************************************
//
//! \addtogroup AT45DBxxxD_Attribute
//! AT45DBxxxD SPI DataFlash attribute.
//! @{
//
//*****************************************************************************
#define AT45DBxxxD_DEFAULT_PAGE_SIZE_LARGE    				1056
#define AT45DBxxxD_BINARY_PAGE_SIZE_LARGE     				1024
#define AT45DBxxxD_DEFAULT_PAGE_SIZE_BIG      				528
#define AT45DBxxxD_BINARY_PAGE_SIZE_BIG       				512
#define AT45DBxxxD_DEFAULT_PAGE_SIZE_SMALL    				264
#define AT45DBxxxD_BINARY_PAGE_SIZE_SMALL     				256


#define AT45DBxxxD_BIG_SECTOR                 				32
#define AT45DBxxxD_SMALL_SECTOR               				16

#define AT45DBXXX_PAGES_PER_BLOCK            					8

#define AT45DBxxxD_BUF1                   	 					1
#define AT45DBxxxD_BUF2                    						2

#define AT45DBxxxD_OVERTIME                 					100000

#define AT45DBxxxD_OP_OK                    					0
#define AT45DBxxxD_OP_BUSY                  					1
#define AT45DBxxxD_OP_INVALID               					2
#define AT45DBxxxD_OP_WRITEPROTECT          					3

#define AT45DBXXX_CMD_WRITE      											0x82  /* Write to Memory instruction */
#define AT45DBXXX_CMD_READ       											0x52  /* Read from Memory instruction */
#define AT45DBXXX_CMD_READ_STATUS       							0xD7  /* Read Status Register instruction  */
#define AT45DBXXX_CMD_READ_ID       									0x9F  /* Read identification */
#define AT45DBXXX_CMD_PAGE_ERASE         							0x81  /* Page Erase instruction 批量擦除*/
#define AT45DBXXX_CMD_BULK_ERASE_1        						0xC7  /* Bulk Erase instruction */
#define AT45DBXXX_CMD_BULK_ERASE_2        						0x94  /* Bulk Erase instruction */
#define AT45DBXXX_CMD_BULK_ERASE_3        						0x80  /* Bulk Erase instruction */
#define AT45DBXXX_CMD_BULK_ERASE_4        						0x9A  /* Bulk Erase instruction */


#define AT45DBxxxD_CMD_CARG       										0xE8
#define AT45DBxxxD_CMD_CARH       										0x0B
#define AT45DBxxxD_CMD_CARL       										0x03
#define AT45DBxxxD_CMD_MMPR       										0xD2
#define AT45DBxxxD_CMD_BF1R       										0xD1 //0xD4 for high speed read, but may cause error
#define AT45DBxxxD_CMD_BF2R       										0xD3 //0xD6 for high speed read, but may cause error
#define AT45DBxxxD_CMD_BF1W       										0x84
#define AT45DBxxxD_CMD_BF2W       										0x87
#define AT45DBxxxD_CMD_B1TMW      										0x83
#define AT45DBxxxD_CMD_B2TMW      										0x86
#define AT45DBxxxD_CMD_B1TMO      										0x88
#define AT45DBxxxD_CMD_B2TMO      										0x89
#define AT45DBxxxD_CMD_ERPG       										0x81
#define AT45DBxxxD_CMD_ERBL       										0x50
#define AT45DBxxxD_CMD_ERSC       										0x7C
#define AT45DBxxxD_CMD_MTB1       										0x53
#define AT45DBxxxD_CMD_MTB2       										0x55
#define AT45DBxxxD_CMD_APR1       										0x58
#define AT45DBxxxD_CMD_APR2       										0x59
#define AT45DBxxxD_CMD_SRRD       										0xD7
#define AT45DBxxxD_CMD_RDID       										0x9F

#define AT45DBxxxD_INVALID_BYTE 											0xff
#define AT45DBXXX_DUMMY_BYTE 													0xA5

#define  AT45DBXXX_CS_LOW()   												do{AT45DBxxxD_PIN_CS = 0;}while(0)	//片选    
#define  AT45DBXXX_CS_HIGH()   												do{AT45DBxxxD_PIN_CS = 1;}while(0)  //释放片选 

#define AT45DBxxxD_MANUFACTURER_ID 										0x1F00
#define AT45DBxxxD_AT45DB161    											0x0026

#define AT45DBXXX_STATUS_RDY  												0x8000 /* Ready/busy status flag */
#define AT45DBXXX_STATUS_COMP  												0x4000 /* Main memory page data does not match buffer data */
#define AT45DBXXX_STATUS_DENSITY  										0x2c00 /* 16-Mbit */
#define AT45DBXXX_STATUS_PROTECT  										0x0200 /* Sector protection is enabled. */
#define AT45DBXXX_STATUS_PAGE_SIZE  									0x0100 /* ? Device is configured for standard DataFlash page size (528 bytes). */
#define AT45DBXXX_STATUS_EPE  												0x0020 /* Erase or Program error detected */
#define AT45DBXXX_STATUS_SLE  												0x0008 /* Sector Lockdown command is enabled */
#define AT45DBXXX_STATUS_PS2  												0x0004 /* A sector is program suspended while using Buffer 2 */
#define AT45DBXXX_STATUS_PS1  												0x0002 /* A sector is program suspended while using Buffer 1 */
#define AT45DBXXX_STATUS_ES  													0x0001 /* A sector is erase suspended */



typedef struct
{
		unsigned short 		usChipID;
		unsigned short 		usPageSize;
		unsigned short 		ulBlockSize;
		unsigned long 		ulSectorSize;
		unsigned long 		ulCapacity;
		unsigned long 		ulTatalPages;
}AT45DBXXX_InfoStruct;


extern uint8_t AT45DBXXX_Init(void);
extern uint8_t AT45DBXXX_Check(void);

extern uint8_t AT45DBXXX_SPI_WriteAndReadByte(uint8_t byte);
extern void AT45DBXXX_ReadID(uint16_t *tmp);
extern uint16_t AT45DBXXX_GetStatus(void);


/*******************************************************************************
* Function Name  : AT45DBXXX_WaitForWriteEnd
* Description    : Polls the status of the Write In Progress (WIP) flag in the
*                  FLASH's status  register  and  loop  until write  opertaion
*                  has completed.
* Input          : None
* Output         : 0成功，1超时
* Return         : None
*******************************************************************************/
extern uint16_t AT45DBXXX_WaitForWriteEnd(void);
extern void AT45DBxxxD_WritePageAddr(uint32_t ulPageAddr);

/*******************************************************************************
* Function Name  : AT45DBXXX_PageErase
* Description    : Erases the specified FLASH Page.
* Input          : PageAddr: address of the sector to erase.
* Output         : None
* Return         : None
*******************************************************************************/
extern uint8_t AT45DB161_PageErase(uint32_t PageAddr);

/*******************************************************************************
* Function Name  : AT45DBXXX_BulkErase
* Description    : Erases the entire FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
extern uint8_t AT45DB161_BulkErase(void);

/*******************************************************************************
* Function Name  : AT45DBXXX_WriteSector
* Description    : Writes one Sector to the FLASH with a single WRITE
*                  cycle(Page WRITE sequence). The number of byte can't exceed
*                  the FLASH page size, But for FAT always 512 butes
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - SectorAddr : FLASH's internal Page address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH,
*                    must be equal or less than "SPI_FLASH_PageSize" value.
* Output         : None
* Return         : None
*******************************************************************************/
extern uint8_t AT45DB161_WriteSector(uint8_t* pBuffer, uint32_t SectorAddr, uint16_t NumByteToWrite);
/*******************************************************************************
* Function Name  : AT45DBXXX_ReadSector
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal Pages address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
extern void AT45DB161_ReadSector(uint8_t* pBuffer, uint32_t SectorAddr, uint16_t NumByteToRead);

//*****************************************************************************
//
//! \brief Main Memory Page to Buffer Transfer
//!
//! \param ucBufferNum specify the buffer 1 or 2 as the destination.
//! \param ulPageAddr specify the page address which you want to read.
//!
//! This function is to transfer data from specified page address of main memory
//! to specified AT45DBxxxD internal buffer.
//!
//! \return operation result
//
//*****************************************************************************
extern uint8_t AT45DBxxxD_MmtoBuf(uint8_t ucBufferNum, uint32_t ulPageAddr);


//*****************************************************************************
//
//! \brief Buffer to Main Memory Page Program with Built-in Erase
//!
//! \param ucBufferNum specify the buffer 1 or 2 as the source.
//! \param ulPageAddr specify the page address which you want to write.
//!
//! This function is to write data from specified internal buffer to the
//! specified main memory page address with built-in erase.
//!
//! \return operation result
//
//*****************************************************************************
extern uint8_t AT45DBxxxD_BuftoMm(uint8_t ucBufferNum, uint32_t ulPageAddr);

//*****************************************************************************
//
//! \brief Read data in specified AT45DBxxxD internal buffer
//!
//! \param ucBufferNum specify the buffer 1 or 2 as the source.
//! \param pucBuffer to store the data read out from internal buffer
//! \param ulOffAddr specify the start address in the buffer which you want to read.
//! \param ulNumByteToRead specify how many bytes to read.
//!
//! This function is to read data from specified internal buffer. If you have
//! just a little data to be stored(less than AT45DBxxxDInfo.usPageSize), you can temporarily
//! store them in the AT45DBxxxD internal buffer.It can be more fast than read write from
//! main memory
//!
//! \return operation result
//
//*****************************************************************************
extern uint8_t AT45DBxxxD_BufferRead(uint8_t ucBufferNum, uint8_t *pucBuffer,
                            uint32_t ulOffSetAddr, uint32_t ulNumByteToRead);


//*****************************************************************************
//
//! \brief Write data to specified AT45DBxxxD internal buffer
//!
//! \param ucBufferNum specify the buffer 1 or 2 as the destination.
//! \param pucBuffer to store the data need to be written.
//! \param ulWriteAddr specify the start address in the buffer which you want to write.
//! \param ulNumByteToRead specify how many bytes to write.
//!
//! This function is to write data to specified internal buffer. If you have
//! just a little data to be stored(less than AT45DBxxxDInfo.usPageSize), you can temporarily
//! store them in the AT45DBxxxD internal buffer.It can be more fast than read write from
//! main memory.This write function doesn't affect the content in main memory.
//!
//! \return operation result
//
//*****************************************************************************
extern uint8_t AT45DBxxxD_BufferWrite(uint8_t ucBufferNum, uint8_t *pucBuffer,
                           uint32_t ulOffSetAddr, uint32_t ulNumByteToWrite);

//*****************************************************************************
//
//! \brief Read a quantity of bytes from start of specified main memory page address
//!
//! \param pucBuffer to store the data read out from page.
//! \param ulPageAddr specify the page address in the main memory of AT45DBxxxD.
//! \param ulNumByteToRead specify how many bytes to read.
//!
//! This function is to read data from specified page address.
//! The function use main memory page read command to read data directly form main memory
//!
//! \return operation result
//
//*****************************************************************************
extern uint8_t AT45DBxxxD_PageRead(uint32_t ulPageAddr, uint8_t *pucBuffer, uint16_t ulNumByteToRead);

//*****************************************************************************
//
//! \brief Write one page of bytes to specified main memory page address
//!
//! \param pucBuffer to store the data read out from page.
//! \param ulPageAddr specify the page address in the main memory of AT45DBxxxD.
//!
//! This function is to write data to specified page address.
//! The function first write data to buffer then using buffer
//! to main memory transfer to write data to main memory.
//!
//! \return operation result
//
//*****************************************************************************
extern uint8_t AT45DBxxxD_PageWrite(uint32_t ulPageAddr, uint8_t *pucBuffer);

//*****************************************************************************
//
//! \brief Read data from main memory
//!
//! \param pucBuffer specifies the destination buffer pointer to store data.
//! \param ulReadAddr specifies the byte address which data will be read (0 to AT45DBxxxDInfo.usPageSize*AT45DBxxxDInfo.ulTatalPages-1)
//! \param ulNumByteToRead specifies the length of data will be read.
//!
//! This function is to read a quantity of bytes from specified byte
//! address in main memory of AT45DBxxxD.
//!
//! \return operation result
//!
//*****************************************************************************
extern uint8_t AT45DBxxxD_Read(uint8_t *pucBuffer, uint32_t ulReadAddr, uint32_t ulNumByteToRead);

//*****************************************************************************
//
//! \brief Write data to main memory
//!
//! \param pucBuffer specifies the source data buffer pointer.
//! \param ulWriteAddr specifies the byte address which data will be written to. (0 to AT45DBxxxDInfo.usPageSize*AT45DBxxxDInfo.ulTatalPages)
//! \param usNumByteToWrite specifies the length of data will be write.
//!
//! This function is to write a quantity of byte to specified byte address in
//! the main memory of AT45DBxxxD.
//!
//! \return operation result
//!
//*****************************************************************************
extern uint8_t AT45DBxxxD_Write(uint8_t *pucBuffer, uint32_t ulWriteAddr, uint32_t ulNumByteToWrite);




#endif
















