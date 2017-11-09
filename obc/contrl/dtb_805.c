/*
 * dtb_805.c
 *
 *  Created on: 2017年9月18日
 *      Author: Ma Wenli
 */
#include "obc_mem.h"
#include "crc.h"
#include "error.h"
#include "bsp_pca9665.h"
#include "math.h"

#include "dtb_805.h"

#define DTB_I2C_ADDR    0x4C
#define DTB_I2C_HANDLE  1
#define DTB_TM_FLAG     0x29
#define DTB_TC_FLAG     0x09

/*遥测获取函数，返回遥测数据长度*/
int xDTBTelemetryGet(uint8_t *pRxData, uint16_t Timeout)
{
    uint8_t RxDataLen;

    uint8_t *pBuffer = (uint8_t *)ObcMemMalloc(24);
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
        ObcMemFree(pBuffer);
        return E_NO_DEVICE;
    }

    /*获取接收数据包中遥测数据长度*/
    RxDataLen = pBuffer[0];

    /*差错检查*/
    if(RxDataLen > 20 || pBuffer[1] != DTB_TM_FLAG)
    {
        ObcMemFree(pBuffer);
        return E_NO_SS;
    }

    /*和校验*/
    if(pBuffer[RxDataLen+2] != sum_check((uint8_t *)&pBuffer[1], RxDataLen+1))
    {
        ObcMemFree(pBuffer);
        return E_NO_SS;
    }

    memcpy(pRxData, (uint8_t *)&pBuffer[2], RxDataLen);

    ObcMemFree(pBuffer);
    return RxDataLen;
}

/*数传板遥控指令发送函数，函数执行成功返回0*/
int xDTBTeleControlSend(uint8_t Cmd, uint16_t Timeout)
{
    uint8_t RxDataLen;

    uint8_t *pBuffer = (uint8_t *)ObcMemMalloc(8);
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
        ObcMemFree(pBuffer);
        return E_NO_DEVICE;
    }

    /*获取接收数据包中遥测数据长度*/
    RxDataLen = pBuffer[0];

    /*差错检查*/
    if (RxDataLen > 5 || pBuffer[1] != DTB_TC_FLAG)
    {
        ObcMemFree(pBuffer);
        return E_NO_SS;
    }

    /*和校验*/
    if (pBuffer[4] != 0x28)
    {
        ObcMemFree(pBuffer);
        return E_NO_SS;
    }

    ObcMemFree(pBuffer);
    return 0;
}

float dtb_temp_conversion(uint8_t temp_raw)
{
    float A = 298.15, B = 4100.0, C = 5013.9, R;

    R = 10000.0 * (0.125 + (float)temp_raw) / (255.875 - (float)temp_raw);

    return 1 / (1/A + log(R/C)/B) - 273.15;
}

void dtb_tm_print(dtb_tm_pack *tm)
{
    printf("Item\t\t\tValue\r\n");
    printf("*******************************\r\n");
    printf("TM_STA\t\t\t0x%02X\r\n", tm->TM_STA);
    printf("AF_PWR\t\t\t%u\r\n", tm->AF_PWR);
    printf("AF_TEMP_RAW\t\t%u\r\n", tm->AF_TEMP);
    printf("AF_TEMP\t\t\t%5f C\r\n", dtb_temp_conversion(tm->AF_TEMP));

    printf("IS_CAN\t\t\t%u\r\n", tm->IS_CAN);
    printf("WD_CNT\t\t\t%u\r\n", tm->WD_CNT);
    printf("RS_CNT\t\t\t%u\r\n", tm->RS_CNT);

    printf("CAN_RS_CNT\t\t%u\r\n", tm->CAN_RS_CNT);
    printf("IIC_RS_CNT\t\t%u\r\n", tm->IIC_RS_CNT);

    printf("RX_INS_CNT\t\t%u\r\n", tm->RX_INS_CNT);
    printf("TRANS_ON\t\t%u\r\n", tm->TRANS_ON);
    printf("DOWN_RATE\t\t%u\r\n", tm->DOWN_RATE);

    printf("PWR_3V3_ON\t\t%u\r\n", tm->PWR_3V3_ON);
    printf("PSD_CODE_ON\t\t%u\r\n", tm->PSD_CODE_ON);
    printf("BACK_CORRECT\t\t%u\r\n", tm->BACK_CORRECT);
    printf("RECORD_CORRECT\t\t%u\r\n", tm->RECORD_CORRECT);
    printf("WORK_MODE\t\t%u\r\n", tm->WORK_MODE);
    printf("PADDING\t\t\t%u\r\n", tm->PADDING);

    printf("MEM1_STA\t\t%u\r\n", tm->MEM1_STA);
    printf("MEM2_STA\t\t%u\r\n", tm->MEM2_STA);
    printf("MEM3_STA\t\t%u\r\n", tm->MEM3_STA);
    printf("MEM4_STA\t\t%u\r\n", tm->MEM4_STA);

    printf("MEM1_MARGIN\t\t%u\r\n", tm->MEM1_MARGIN);
    printf("MEM2_MARGIN\t\t%u\r\n", tm->MEM2_MARGIN);
    printf("MEM3_MARGIN\t\t%u\r\n", tm->MEM3_MARGIN);
    printf("MEM4_MARGIN\t\t%u\r\n", tm->MEM4_MARGIN);

    printf("MEM1_RECORD_CNT\t\t%u\r\n", tm->MEM1_RECORD_CNT);
    printf("MEM2_RECORD_CNT\t\t%u\r\n", tm->MEM2_RECORD_CNT);
    printf("MEM3_RECORD_CNT\t\t%u\r\n", tm->MEM3_RECORD_CNT);
    printf("MEM4_RECORD_CNT\t\t%u\r\n", tm->MEM4_RECORD_CNT);

    printf("MEM1_BACK_CNT\t\t%u\r\n", tm->MEM1_BACK_CNT);
    printf("MEM2_BACK_CNT\t\t%u\r\n", tm->MEM2_BACK_CNT);
    printf("MEM3_BACK_CNT\t\t%u\r\n", tm->MEM3_BACK_CNT);
    printf("MEM4_BACK_CNT\t\t%u\r\n", tm->MEM4_BACK_CNT);

}

