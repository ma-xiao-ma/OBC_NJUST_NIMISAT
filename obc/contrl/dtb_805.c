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
#include "bsp_switch.h"
#include "hk.h"

#include "dtb_805.h"

#define DTB_I2C_ADDR    0x4C
#define DTB_I2C_HANDLE  1
#define DTB_TM_FLAG     0x29
#define DTB_TC_FLAG     0x09

/**
 * IO控制数传机上电
 *
 * @return 返回E_NO_ERR（-1）为正常
 */
int dtb_power_on(void)
{
    if( EpsOutSwitch(OUT_DTB_5V, ENABLE) != EPS_ERROR )
    {
        if( EpsOutSwitch(OUT_DTB_12V, ENABLE) != EPS_ERROR )
            return E_NO_ERR;
        else
        {
            EpsOutSwitch(OUT_DTB_5V, DISABLE);
            return E_NO_DEVICE;
        }

    }
    else
        return E_NO_DEVICE;
}

/**
 * IO控制数传机断电
 *
 */
void dtb_power_off(void)
{
    EpsOutSwitch(OUT_DTB_5V, DISABLE);
    EpsOutSwitch(OUT_DTB_12V, DISABLE);
}

/*遥测获取函数，返回遥测数据长度*/
int xDTBTelemetryGet(uint8_t *pRxData, uint16_t Timeout)
{
    uint8_t RxDataLen;

    uint8_t *pBuffer = (uint8_t *)ObcMemMalloc(24);
    if(pBuffer == NULL)
        return E_MALLOC_FAIL;

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
        return E_CRC_CHECK_ERROR;
    }

    memcpy(pRxData, (uint8_t *)&pBuffer[2], RxDataLen);

    ObcMemFree(pBuffer);
    return RxDataLen;
}

/*数传板遥控指令发送函数，函数执行成功返回E_NO_ERR(-1)*/
int xDTBTeleControlSend(uint8_t Cmd, uint16_t Timeout)
{
    uint8_t RxDataLen;

    uint8_t *pBuffer = (uint8_t *)ObcMemMalloc(8);
    if(pBuffer == NULL)
        return E_MALLOC_FAIL;

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
    return E_NO_ERR;
}

/**
 * 数传机擦除加记录
 *
 * @param mem_num 即将向存储区几存储
 * @return 返回E_NO_ERR（-1）为正常
 */
int cam_dtb_lvds(uint8_t mem_num)
{
    int ret;

    /* 保证供电 */
    if( EpsOutSwitch(OUT_DTB_5V, ENABLE) != EPS_ERROR &&
            EpsOutSwitch(OUT_DTB_12V, ENABLE) != EPS_ERROR )
    {
        /*根据传入参数选择擦除和记录的存储区*/
        switch( mem_num )
        {
            case 1:
                if( xDTBTeleControlSend(Mem1Erase, 100) != E_NO_ERR )
                    return E_NO_SS;

                vTaskDelay(8000);

                if( xDTBTeleControlSend(Mem1Record, 100) != E_NO_ERR )
                    return E_NO_SS;

                break;
            case 2:
                if( xDTBTeleControlSend(Mem2Erase, 100) != E_NO_ERR )
                    return E_NO_SS;

                vTaskDelay(8000);

                if( xDTBTeleControlSend(Mem2Record, 100) != E_NO_ERR )
                    return E_NO_SS;

                break;
            case 3:
                if( xDTBTeleControlSend(Mem3Erase, 100) != E_NO_ERR )
                    return E_NO_SS;

                vTaskDelay(8000);

                if( xDTBTeleControlSend(Mem3Record, 100) != E_NO_ERR )
                    return E_NO_SS;

                break;
            case 4:
                if( xDTBTeleControlSend(Mem4Erase, 100) != E_NO_ERR )
                    return E_NO_SS;

                vTaskDelay(8000);

                if( xDTBTeleControlSend(Mem4Record, 100) != E_NO_ERR )
                    return E_NO_SS;

                break;
            default:
                return E_THREAD_FAIL;
        }
    }

    return E_NO_ERR;
}


/**
 * 数传机回放下行流程
 *
 * @param mem_num 存储区号 1、2、3、4
 * @param data_rate 下行码速率1、2、4、8
 * @return 返回E_NO_ERR（-1）为正确
 */
int dtb_back(uint8_t mem_num, uint8_t data_rate)
{
    int ret;

    /* 保证电力供应 */
    if( dtb_power_on() != E_NO_ERR )
        return E_NO_DEVICE;

    if( xDTBTeleControlSend(Boot, 100) != E_NO_ERR )
    {
        dtb_power_off();
        return E_NO_SS;
    }

    switch( data_rate )
    {
        case 1:
            ret = xDTBTeleControlSend(Rate1Mbps, 100);
            break;
        case 2:
            ret = xDTBTeleControlSend(Rate2Mbps, 100);
            break;
        case 4:
            ret = xDTBTeleControlSend(Rate4Mbps, 100);
            break;
        case 8:
            ret = xDTBTeleControlSend(Rate8Mbps, 100);
            break;
        default:
            ret = E_THREAD_FAIL;
            break;
    }

    if( ret !=  E_NO_ERR)
    {
        dtb_power_off();
        return E_NO_SS;
    }

    switch( mem_num )
    {
        case 1:
            ret = xDTBTeleControlSend(Mem1Back, 100);
            break;
        case 2:
            ret = xDTBTeleControlSend(Mem2Back, 100);
            break;
        case 3:
            ret = xDTBTeleControlSend(Mem3Back, 100);
            break;
        case 4:
            ret = xDTBTeleControlSend(Mem4Back, 100);
            break;
        default:
            ret = E_THREAD_FAIL;
            break;
    }

    if( ret !=  E_NO_ERR)
    {
        dtb_power_off();
        return E_FLASH_ERROR;
    }


    dtb_805_hk_t *dtb = (dtb_805_hk_t *)ObcMemMalloc( sizeof(dtb_805_hk_t) );
    if( dtb == NULL )
    {
        dtb_power_off();
        return E_MALLOC_FAIL;
    }

    switch( mem_num )
    {
        case 1:
            ret = xDTBTeleControlSend(Mem1Back, 100);
            break;
        case 2:
            ret = xDTBTeleControlSend(Mem2Back, 100);
            break;
        case 3:
            ret = xDTBTeleControlSend(Mem3Back, 100);
            break;
        case 4:
            ret = xDTBTeleControlSend(Mem4Back, 100);
            break;
        default:
            ret = E_THREAD_FAIL;
            break;
    }

    int back_complete_flag = 0;

    while( ( ret = dtb_hk_get_peek(dtb) ) != pdFALSE )
    {
        switch( mem_num )
        {
            case 1:
                if( dtb->dtb_hk.MEM1_BACK_CNT == dtb->dtb_hk.MEM1_RECORD_CNT )
                    back_complete_flag = 1;
                break;
            case 2:
                if( dtb->dtb_hk.MEM2_BACK_CNT == dtb->dtb_hk.MEM2_RECORD_CNT )
                    back_complete_flag = 1;
                break;
            case 3:
                if( dtb->dtb_hk.MEM3_BACK_CNT == dtb->dtb_hk.MEM3_RECORD_CNT )
                    back_complete_flag = 1;
                break;
            case 4:
                if( dtb->dtb_hk.MEM4_BACK_CNT == dtb->dtb_hk.MEM4_RECORD_CNT )
                    back_complete_flag = 1;
                break;
            default:
                ret = E_THREAD_FAIL;
                break;
        }

        if( back_complete_flag )
            break;

        vTaskDelay(1000);
    }

    if( xDTBTeleControlSend(MemStop, 100) != E_NO_ERR )
    {
        dtb_power_off();
        return E_NO_SS;
    }

    if( xDTBTeleControlSend(ShutDown, 100) != E_NO_ERR )
    {
        dtb_power_off();
        return E_NO_SS;
    }

    dtb_power_off();

    return E_NO_ERR;
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

