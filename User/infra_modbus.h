

#ifndef _INFRA_MODBUS_H
#define _INFRA_MODBUS_H


typedef unsigned char uint8;
typedef unsigned int  uint16;
typedef unsigned long uint32;

#define ERR 0
#define EOK 1 

#define DATA_NULL           ERR
#define ADDR_UNREACH        ERR
#define PACKET_LEN_UNMATCH  ERR


#define PROTOCAL_ADDR                       		0x01 		//发送地址

#define PROTOCAL_CODE_CHECK_MODEL_NUM       		0x31
#define PROTOCAL_CODE_SET_MODEL_NUM         		0x32   
#define PROTOCAL_CODE_CTRL                  		0x05     //控制命令
#define PROTOCAL_CODE_LEARN                 		0x06     //自学习命令
#define PROTOCAL_CODE_CHECK_STA             		0x21     //读取学习状态
#define PROTOCAL_CODE_CLEAN_ALL_CMD         		0x22
#define PROTOCAL_CODE_UPLOAD                		0x23
#define PROTOCAL_CODE_DOWNLOAD_START        		0x24
#define PROTOCAL_CODE_DOWNLOAD              		0x25
#define PROTOCAL_CODE_CHECK_VERSION         		0x26
#define PROTOCAL_CODE_QUERY_START           		0x28     //读取运行状态


#define AIR_CTRLORDER_OFF                       0x01    //关机
#define AIR_CTRLORDER_ON_COOL_TEMP20_STRONG     0x02    //开机、制冷模式、20度、高风
#define AIR_CTRLORDER_ON_COOL_TEMP21_STRONG     0x03    //开机、制冷模式、21度、高风
#define AIR_CTRLORDER_ON_COOL_TEMP22_STRONG     0x04    //开机、制冷模式、22度、高风
#define AIR_CTRLORDER_ON_COOL_TEMP23_STRONG     0x05    //开机、制冷模式、23度、高风
#define AIR_CTRLORDER_ON_COOL_TEMP24_STRONG     0x06    //开机、制冷模式、24度、高风
#define AIR_CTRLORDER_ON_COOL_TEMP25_STRONG     0x07    //开机、制冷模式、25度、高风
#define AIR_CTRLORDER_ON_COOL_TEMP26_STRONG     0x08    //开机、制冷模式、26度、高风
#define AIR_CTRLORDER_ON_COOL_TEMP27_STRONG     0x09    //开机、制冷模式、27度、高风
#define AIR_CTRLORDER_ON_COOL_TEMP28_STRONG     0x0a    //开机、制冷模式、28度、高风
#define AIR_CTRLORDER_ON_COOL_TEMP29_STRONG     0x0b    //开机、制冷模式、29度、高风
#define AIR_CTRLORDER_ON_COOL_TEMP30_STRONG     0x0c    //开机、制冷模式、30度、高风
#define AIR_CTRLORDER_AIRSUPPLY_STRONG          0x0d    //开机、送风模式、高风
#define AIR_CTRLORDER_ON_HEAT_TEMP16_STRONG     0x0e    //开机、制热模式、16度、高风
#define AIR_CTRLORDER_ON                        0x0f    //开机指令

#define AIR_CTRLORDER_FAIL                      0xff    //返回失败


#define PROTOCAL_NORMAL             						0x00    //正常
#define PROTOCAL_VER_ERROR          						0x01    //协议版本错
#define PROTOCAL_CRC_ERROR          						0x02    //CRC错
#define PROTOCAL_LCHKSUM_ERROR      						0x03    //LCHKSUM错
#define PROTOCAL_FORMAT_ERROR       						0x04    //命令格式错
#define PROTOCAL_DATA_INVALID       						0x05    //无效数据


typedef struct Modbus_28H_rx
{
    uint8 		ctrl_orderno;
    uint8 		swith;
    uint8 		temp;
    uint8 		mode;
    uint8 		speed;
}Modbus_28H_RX;

uint16 CRC16_1(uint8* pchMsg, uint16 wDataLen);

/**********************************************************
*addr       :地址
*funcode    :功能代码
*inbuff     :输入数据存储区地址
*outbuff    :输出数据存储地址
*outlen			:输出存储区长度
***********************************************************/
uint8 modbus_tx_fun(uint8 addr, uint8 funcode, const uint8 *inbuff, uint16 inlen, uint8 *outbuff, uint16 outlen);
/**********************************************************
*inbuff     :输入数据存储区地址
*inlen      :输入数据长度
*outbuff    :输出数据存储地址
*outlen     :输出存储区长度长度 
***********************************************************/
uint8 modbus_rx_fun(const uint8 *inbuff, const uint16 inlen, uint8 *outbuff, uint16 *pOutlen);



#endif



