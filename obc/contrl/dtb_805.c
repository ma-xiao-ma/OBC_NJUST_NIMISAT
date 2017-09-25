/*
 * dtb_805.c
 *
 *  Created on: 2017年9月18日
 *      Author: Ma Wenli
 */
#include "QB50_mem.h"
#include "crc.h"
#include "error.h"
#include "dtb_805.h"

#define DTB_I2C_ADDR    0x4C
#define DTB_I2C_HANDLE  1
#define DTB_TM_FLAG     0x29
#define DTB_TC_FLAG     0x09

/*遥测获取函数，返回遥测数据长度*/
int xDTBTelemetryGet(uint8_t *pRxData, uint16_t Timeout)
{
    uint8_t RxDataLen;

    uint8_t *pBuffer = (uint8_t *)qb50Malloc(24);
    if(pBuffer == NULL)
    {
        return E_MALLOC_FAIL;
    }

    pBuffer[0] = 0x04;
    pBuffer[1] = 0x00;
    pBuffer[2] = 0x10;
    pBuffer[3] = 0x01;
    pBuffer[4] = 0x11;

    /*调用I2C接口主发主收*/
    if(i2c_master_transaction(DTB_I2C_HANDLE, DTB_I2C_ADDR,
            pBuffer, 5, pBuffer, 20, Timeout) != E_NO_ERR)
    {
        qb50Free(pBuffer);
        return E_NO_DEVICE;
    }

    /*获取接收数据包中遥测数据长度*/
    RxDataLen = pBuffer[0];

    /*差错检查*/
    if(RxDataLen > 20 || pBuffer[1] != DTB_TM_FLAG)
    {
        qb50Free(pBuffer);
        return E_NO_SS;
    }

    /*和校验*/
    if(pBuffer[RxDataLen+2] != sum_check((uint8_t *)&pBuffer[1], RxDataLen+1))
    {
        qb50Free(pBuffer);
        return E_NO_SS;
    }

    memcpy(pRxData, (uint8_t *)&pBuffer[2], RxDataLen);

    qb50Free(pBuffer);
    return RxDataLen;
}

/*数传板遥控指令发送函数，函数执行成功返回0*/
int xDTBTeleControlSend(uint8_t Cmd, uint16_t Timeout)
{
    uint8_t RxDataLen;

    uint8_t *pBuffer = (uint8_t *)qb50Malloc(8);
    if(pBuffer == NULL)
    {
        return E_MALLOC_FAIL;
    }

    pBuffer[0] = 0x06;
    pBuffer[1] = 0x20;
    pBuffer[2] = 0x09;
    pBuffer[3] = Cmd;
    pBuffer[4] = (Cmd<0x10) ? 0xC0+Cmd : 0xD0+Cmd-0x10;
    pBuffer[5] = pBuffer[4];
    pBuffer[6] = sum_check((uint8_t *)&pBuffer[1], 5);

    /*调用I2C接口主发主收*/
    if(i2c_master_transaction(DTB_I2C_HANDLE, DTB_I2C_ADDR,
            pBuffer, 7, pBuffer, 5, Timeout) != E_NO_ERR)
    {
        qb50Free(pBuffer);
        return E_NO_DEVICE;
    }

    /*获取接收数据包中遥测数据长度*/
    RxDataLen = pBuffer[0];

    /*差错检查*/
    if(RxDataLen > 5 || pBuffer[1] != DTB_TC_FLAG)
    {
        qb50Free(pBuffer);
        return E_NO_SS;
    }

    /*和校验*/
    if(pBuffer[4] != 0x2C)
    {
        qb50Free(pBuffer);
        return E_NO_SS;
    }

    return 0;
}
