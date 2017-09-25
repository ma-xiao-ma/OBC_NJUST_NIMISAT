/*
 * bsp_ttc.c
 *
 *  Created on: 2017年4月28日
 *      Author: 84474
 */

#include "driver_debug.h"
#include "bsp_ttc.h"
#include "error.h"
#include "qb50_mem.h"
#include "bsp_pca9665.h"
#include "command.h"
#include "crc.h"


int TTC_Cmd(uint8_t CmdNumber, uint8_t* pResp,uint16_t RespLen)
{
	int CmdReturn = E_NO_ERR;

	ttc_cmd_t* pCmdToSend = (ttc_cmd_t*)qb50Malloc(sizeof(ttc_cmd_t));
	ttc_resp_t* pCmdResp = (ttc_resp_t*)qb50Malloc(sizeof(ttc_resp_t) + RespLen);

	pCmdToSend->SLH = 0x00;
	pCmdToSend->SLL = 0x01;
	pCmdToSend->CMD = (uint8_t)CmdNumber;

	CmdReturn = i2c_master_transaction(TTC_I2C_Handle, TTC_I2C_Addr, pCmdToSend, sizeof(ttc_cmd_t),
			pCmdResp, sizeof(ttc_resp_t) + RespLen, TTC_I2C_Timeout);
	if(CmdReturn != E_NO_ERR)
	{
		driver_debug(DEBUG_TTC,"CMD 0x%02x transmit error!\r\n", pCmdToSend->CMD);
	}
	else if(pCmdResp->STATUS != TTC_Data_Valid)
	{
		driver_debug(DEBUG_TTC,"CMD 0x%02x transmit error!\r\n", pCmdToSend->CMD);
		driver_debug(DEBUG_TTC,"Invalid Data!\r\n");
		CmdReturn = E_INVALID_BUF_SIZE;
	}
	else if(RespLen < (((pCmdResp->SLH)<<8) | pCmdResp->SLL))
	{
		driver_debug(DEBUG_TTC,"CMD 0x%02x transmit error!\r\n", pCmdToSend->CMD);
		driver_debug(DEBUG_TTC,"Invalid Parameter!\r\n");
		CmdReturn = E_INVALID_PARAM;
	}
	else
	{
		driver_debug(DEBUG_I2C,"CMD 0x%02x receive success!\r\n", pCmdToSend->CMD);
		driver_debug(DEBUG_I2C,"TTC respond valid data length :%u\r\n", RespLen);
		memcpy(pResp, &pCmdResp->DATA[0], RespLen);
		CmdReturn = E_NO_ERR;
	}
	qb50Free(pCmdToSend);
	qb50Free(pCmdResp);
	return CmdReturn;
}

int TTC_Cmd_No_Reply(uint8_t CmdNoRep)
{
	int CmdReturn = E_NO_ERR;
	ttc_cmd_t * ttc_cmd = (ttc_cmd_t *)qb50Malloc(sizeof(ttc_cmd_t));

	ttc_cmd->CMD = CmdNoRep;
	ttc_cmd->SLH = 0x00;
	ttc_cmd->SLL = 0x01;

	CmdReturn = i2c_master_transaction(TTC_I2C_Handle, TTC_I2C_Addr, ttc_cmd, sizeof(ttc_cmd_t), NULL, 0, TTC_I2C_Timeout);
	if(CmdReturn != E_NO_ERR)
	{
		driver_debug(DEBUG_TTC,"CMD 0x%02x transmit error!\r\n", ttc_cmd->CMD);
		qb50Free(ttc_cmd);
		return CmdReturn;
	}
	driver_debug(DEBUG_TTC,"CMD 0x%02x receive success!\r\n", ttc_cmd->CMD);
	qb50Free(ttc_cmd);
	return CmdReturn;
}

int TTC_Send_Date(uint8_t* pBuffer, uint8_t NumByteToSend)
{
	size_t txlen = NumByteToSend + 5;
	size_t framesize = NumByteToSend + 3;
	size_t datasize = NumByteToSend;
	ttc_send_t* ttc_send = (ttc_send_t*)qb50Malloc(txlen);
//	uint8_t* pdata = (uint8_t*)ttc_send;
	int8_t send_return = E_NO_ERR;

	ttc_send->SLH = (uint8_t)(framesize >> 8);
	ttc_send->SLL = (uint8_t)framesize;
	ttc_send->CMD = SendFrame_t;
	ttc_send->PARA.DSLH = (uint8_t)(datasize >> 8);
	ttc_send->PARA.DSLL = (uint8_t)datasize;
//	ttc_send->PARA.DATA = &pdata[5];
	memcpy(ttc_send->PARA.DATA, pBuffer, datasize);

	send_return = i2c_master_transaction(TTC_I2C_Handle, TTC_I2C_Addr, ttc_send, txlen, NULL, 0, TTC_I2C_Timeout);
	if(send_return != E_NO_ERR)
	{
		driver_debug(DEBUG_TTC,"TTC send data error!\r\n");
		qb50Free(ttc_send);
		return send_return;
	}
	driver_debug(DEBUG_TTC,"TTC send data success!\r\n");
	qb50Free(ttc_send);
	return send_return;

}


const uint8_t EncodeArrey[5] = {0x00, 0x03, 0x05, 0x01, 0x0b};
const uint8_t DncodeArrey[26] = {0x7e, 0x42, 0x55, 0x53, 0x41, 0x54, 0x42, 0x4a, 0x42, 0x55, 0x53, 0x41, 0x54,
                                 0x42, 0x4a, 0x03, 0xe8, 0x00, 0x01, 0x82, 0x80, 0x85, 0xaf, 0xbe, 0x2f, 0xc0};
const uint8_t DestinationAddress[7] = {0x42, 0x55, 0x53, 0x41, 0x54, 0x42, 0x4a};
const uint8_t SourceAddress[7] = {0x42, 0x55, 0x53, 0x41, 0x54, 0x42, 0x4a};

#define ResultBufferSize 400
#define FramingBufferSize 400 //帧头16byte + crc16（2byte）+ 信息域 0-256
#define AX_25_FLAG    (0x7E)
#define CONTROL_WORD  (0x03)
#define COM_FLAG      (0xF0)

static uint8_t EncodeResult[ResultBufferSize] = {0};
static uint8_t FramingMake[FramingBufferSize] = {0};
static uint16_t ConvertIndex    = 0;
static uint8_t  BitIndex        = 7;
static uint8_t  BitSetTimes     = 0;
static uint32_t MoreBit         = 0;
static uint8_t  IsFrameEnd      = 0;

static inline void ConvertSetBit(void);
static inline void ConvertResetBit(void);

static inline void ConvertSetBit(void)
{
    EncodeResult[ConvertIndex] |= (1<<BitIndex);
    if(--BitIndex > 7)
    {
        BitIndex = 7;
        ConvertIndex++;
    }
}

static inline void ConvertResetBit(void)
{
    EncodeResult[ConvertIndex] &= ~(1<<BitIndex);
    if(--BitIndex > 7)
    {
        BitIndex = 7;
        ConvertIndex++;
    }
}

void ConversionParaInit(void)
{
    memset(FramingMake, (int)0, (size_t)FramingBufferSize);
    memset(EncodeResult, (int)0, (size_t)ResultBufferSize);
    ConvertIndex = 0;
    BitIndex = 7;
    BitSetTimes = 0;
    BitSetTimes = 0;
    MoreBit = 0;
    IsFrameEnd = 0;
}

int AX25_Encode_test(void)
{
    uint16_t EncodedLenth = 0;
    uint8_t ResultData[400];
    EncodedLenth = AX25_Encode((uint8_t*)EncodeArrey, 5, ResultData);
    return CMD_ERROR_NONE;
}

int AX25_Decode_test(void)
{
    int Result = 0;
    uint16_t pResultLen;
    uint8_t ResultData[400];
    Result = AX25_Decode(DncodeArrey, 26, ResultData, &pResultLen);
    return CMD_ERROR_NONE;
}

void AX25_Encode_Byte(uint8_t ebyte)
{
    uint8_t BitMask = 0x80;
    for(uint8_t i=0; i<8; i++, BitMask>>=1)
    {
        if(ebyte & BitMask)
        {
            BitSetTimes++;
            ConvertSetBit();
        }
        else
        {
            BitSetTimes = 0;
            ConvertResetBit();
        }
        if(BitSetTimes == 5) //每逢5个1 插一个0
        {
            BitSetTimes = 0;
            MoreBit++;
            ConvertResetBit();
        }
    }
}

void AX25_Dncode_Byte(uint8_t ebyte)
{
    uint8_t BitMask = 0x80;
    for(uint8_t i=0; i<8; i++, BitMask>>=1)
    {
        if(BitSetTimes == 5)
        {
            if(ebyte & BitMask) //如果第六位是1 则
            {
                BitSetTimes = 0;
                ConvertSetBit();
                if(IsFrameEnd++) //如果是第二个AX.25标志域   则帧结束
                    break;
            }
            else//如果第六位是0 则清标志位 并跳过这一位
            {
                BitSetTimes = 0;
                MoreBit++;
            }
            continue;
        }
        if(ebyte & BitMask)
        {
            BitSetTimes++;
            ConvertSetBit();
        }
        else
        {
            BitSetTimes = 0;
            ConvertResetBit();
        }
    }
}

    /* 0111 1110 */
void TailInsert(void)
{
    ConvertResetBit();
    /* 插入连续6个1 */
    for(uint8_t i =0; i<6; i++)
    {
        ConvertSetBit();
    }
    ConvertResetBit();
}

/*******************************************************************************
函数说明:  组织AX.25协议的UI信息帧函数（不带帧前后两个标志域），结果存放在FramingMake全局静态数组中
入口参数:
            pData     : 待编码数据指针
            Datalen   : 待编码数据长度

返回值:   无
/******************************************************************************/
void AX25_Framing(uint8_t* pData, uint32_t DataLen)
{
    uint16_t CRCGene = 0;

    memcpy(&FramingMake[0], DestinationAddress, (size_t)7);
    memcpy(&FramingMake[7], DestinationAddress, (size_t)7);
    FramingMake[14] = CONTROL_WORD;
    FramingMake[15] = COM_FLAG;
    memcpy(&FramingMake[16], pData, (size_t)DataLen);
    CRCGene = crc_citt_value(FramingMake, DataLen + 16);
    FramingMake[DataLen + 16] = (uint8_t)(CRCGene>>8);
    FramingMake[DataLen + 17] = (uint8_t)CRCGene;
}

/*******************************************************************************
函数说明:  AX.25编码函数
入口参数:
            pData     : 待编码数据指针
            Datalen   : 待编码数据长度
            pResult   : 编码完成后数据指针

返回值:   编码完成后的数据长度
/******************************************************************************/
uint16_t AX25_Encode(void* pData, uint32_t DataLen, uint8_t* pResult)
{
    ConversionParaInit();

    EncodeResult[0] = AX_25_FLAG;
    ConvertIndex ++;
    AX25_Framing(pData, DataLen);
    /* 数据 + 帧头16字节 + CRC校验2字节*/
    for(uint16_t i=0; i<DataLen+16+2; i++)
    {
        AX25_Encode_Byte(FramingMake[i]);
    }
    TailInsert();
    memcpy(pResult, EncodeResult, (size_t)(ConvertIndex+1));
    return ConvertIndex+1;
}

/*******************************************************************************
函数说明:  AX.25解码函数
入口参数:
            pData      : 待解码数据指针
            Datalen    : 待解码数据长度
            pResult    : 解码完成后数据指针
            pResultLen : 解码完成后数据长度指针

返回值:   CRC校验结果
/******************************************************************************/
int AX25_Decode(void* pData, uint32_t DataLen, uint8_t* pResult, uint16_t* pResultLen)
{
    uint16_t CRCCheck = 0;
    ConversionParaInit();
    while(DataLen--)
        AX25_Dncode_Byte(*(uint8_t *)pData++);
    if(EncodeResult[ConvertIndex] != 0x7E)
        return 0;
    memcpy(pResult, EncodeResult, (size_t)(ConvertIndex+1));
    *pResultLen = ConvertIndex+1;
    CRCCheck = (uint16_t)(EncodeResult[ConvertIndex-2]<<8 | EncodeResult[ConvertIndex-1]);
    if(crc_citt_value(&EncodeResult[1],  ConvertIndex+1-4) != CRCCheck)
        return 0;
    return -1;
}






