/*
 * dtb_805.c
 *
 *  Created on: 2017年9月18日
 *      Author: Ma Wenli
 */

#include "stdbool.h"

#include "obc_mem.h"
#include "crc.h"
#include "error.h"
#include "bsp_pca9665.h"
#include "math.h"
#include "bsp_switch.h"
#include "hk.h"
#include "string.h"
#include "ctrl_cmd_types.h"


#include "dtb_805.h"

#define DTB_I2C_ADDR    0x4C
#define DTB_I2C_HANDLE  1
#define DTB_TM_FLAG     0x29
#define DTB_TC_FLAG     0x09

#define DTB_CMD_TIMEOUT 1000

bool DTB_Busy = false;

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
 *@return 返回E_NO_ERR（-1）为正常
 */
int dtb_power_off(void)
{
    if( EpsOutSwitch(OUT_DTB_5V, DISABLE) != EPS_ERROR &&
            EpsOutSwitch(OUT_DTB_12V, DISABLE) != EPS_ERROR )
        return E_NO_ERR;
    else
        return E_NO_DEVICE;
}

/**
 * 数传板供电控制函数
 *
 * @return 返回E_NO_ERR（-1）为正常
 */
int dtb_power_switch( tr_sw_status power_sw )
{
    if( power_sw == tr_sw_on )
        return dtb_power_on();
    else
        return dtb_power_off();
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
 * 数传固存擦除
 *
 * @param mem_num 固存编号
 * @return 返回E_NO_ERR（-1）为正常
 */
int mem_erase( mem_region mem_num )
{
    int ret = E_OUT_OF_MEM;

    /* 保证数传控制板供电 */
    if( OUT_SW_DTB_5V_PIN() != SET )
        return E_NO_SS;

    switch( mem_num )
    {
        case MemOne:
            ret = xDTBTeleControlSend(Mem1Erase, DTB_CMD_TIMEOUT);
            break;
        case MemTwo:
            ret = xDTBTeleControlSend(Mem2Erase, DTB_CMD_TIMEOUT);
            break;
        case MemThree:
            ret = xDTBTeleControlSend(Mem3Erase, DTB_CMD_TIMEOUT);
            break;
        case MemFour:
            ret = xDTBTeleControlSend(Mem4Erase, DTB_CMD_TIMEOUT);
            break;
        default:
            break;
    }

    return ret;
}

/**
 * 数传固存记录
 *
 * @param mem_num 固存编号
 * @return 返回E_NO_ERR（-1）为正常
 */
int mem_record( mem_region mem_num )
{
    int ret = E_OUT_OF_MEM;

     /* 保证数传控制板供电 */
     if( OUT_SW_DTB_5V_PIN() != SET )
         return E_NO_SS;

     switch( mem_num )
     {
         case MemOne:
             ret = xDTBTeleControlSend(Mem1Record, DTB_CMD_TIMEOUT);
             break;
         case MemTwo:
             ret = xDTBTeleControlSend(Mem2Record, DTB_CMD_TIMEOUT);
             break;
         case MemThree:
             ret = xDTBTeleControlSend(Mem3Record, DTB_CMD_TIMEOUT);
             break;
         case MemFour:
             ret = xDTBTeleControlSend(Mem4Record, DTB_CMD_TIMEOUT);
             break;
         default:
             break;
     }

     return ret;
}

/**
 * 数传固存回放
 *
 * @param mem_num 固存编号
 * @return 返回E_NO_ERR（-1）为正常
 */
int mem_back( mem_region mem_num )
{
    int ret = E_OUT_OF_MEM;

     /* 保证数传控制板供电 */
     if( OUT_SW_DTB_5V_PIN() != SET )
         return E_NO_SS;

     switch( mem_num )
     {
         case MemOne:
             ret = xDTBTeleControlSend(Mem1Back, DTB_CMD_TIMEOUT);
             break;
         case MemTwo:
             ret = xDTBTeleControlSend(Mem2Back, DTB_CMD_TIMEOUT);
             break;
         case MemThree:
             ret = xDTBTeleControlSend(Mem3Back, DTB_CMD_TIMEOUT);
             break;
         case MemFour:
             ret = xDTBTeleControlSend(Mem4Back, DTB_CMD_TIMEOUT);
             break;
         default:
             break;
     }

     return ret;
}

/**
 * 下行码速率选择
 *
 * @param down_rate 下行码速率选择 1--1Mbps 2--2Mbps 3--4Mbps 4--8Mbps
 * @return 返回E_NO_ERR（-1）为正常
 */
int rf_rate_select( data_rate down_rate )
{
    int ret = E_OUT_OF_MEM;

     /* 保证数传控制板供电 */
     if( OUT_SW_DTB_5V_PIN() != SET )
         return E_NO_SS;

     switch( down_rate )
     {
         case MemOne:
             ret = xDTBTeleControlSend(Rate1Mbps, DTB_CMD_TIMEOUT);
             break;
         case MemTwo:
             ret = xDTBTeleControlSend(Rate2Mbps, DTB_CMD_TIMEOUT);
             break;
         case MemThree:
             ret = xDTBTeleControlSend(Rate4Mbps, DTB_CMD_TIMEOUT);
             break;
         case MemFour:
             ret = xDTBTeleControlSend(Rate8Mbps, DTB_CMD_TIMEOUT);
             break;
         default:
             break;
     }

     return ret;
}

/**
 * 数传机擦除加记录
 *
 * @param mem_num 即将向存储区几存储
 * @param need_erase 是否需要擦除固存（0为不擦除  非0为擦除）
 *
 * @return 返回E_NO_ERR（-1）为正常
 */
int dtb_mem_record(uint8_t mem_num, uint8_t need_erase)
{
    int ret = E_OUT_OF_MEM;
    /* 保证数传控制板供电 */
    if( dtb_power_on() != E_NO_ERR )
        return E_NO_SS;

    if( need_erase )
    {
        if( ( ret = mem_erase( mem_num ) ) != E_NO_ERR )
            return ret;

        vTaskDelay( 8000 / portTICK_RATE_MS ); //擦除等待
    }

    ret = mem_record( mem_num );

    return ret;
}


/**
 * 数传机回放下行流程
 *
 * @param mem_num 存储区号 1、2、3、4
 * @param data_rate 下行码速率1--1Mbps、2--2Mbps、3--4Mbps、4--8Mbps
 * @param back_last 回放持续时间，单位：秒
 * @return 返回E_NO_ERR（-1）为正确
 */
int dtb_mem_back(uint8_t mem_num, uint8_t data_rate, uint16_t back_last)
{
    /* 保证电力供应 */
    if( dtb_power_on() != E_NO_ERR )
        return E_NO_DEVICE;

    vTaskDelay(2000);

    /* 选择下行码速率  1--1Mbps、2--2Mbps、3--4Mbps、4--8Mbps */
    if( rf_rate_select( data_rate ) != E_NO_ERR)
    {
        dtb_power_off();
        return E_NO_SS;
    }

    vTaskDelay(500);

    /* 发射机开机 */
    if( xDTBTeleControlSend(Boot, DTB_CMD_TIMEOUT) != E_NO_ERR )
    {
        dtb_power_off();
        return E_NO_SS;
    }

    vTaskDelay(500);

    if( mem_back( mem_num ) != E_NO_ERR )
    {
        dtb_power_off();
        return E_NO_SS;
    }

    back_last = back_last > 600 ? 600 : back_last;

    vTaskDelay(back_last * 1000);

    /* 固存停止 */
    if( xDTBTeleControlSend(MemStop, DTB_CMD_TIMEOUT) != E_NO_ERR )
    {
        dtb_power_off();
        return E_NO_SS;
    }

    vTaskDelay(500);

    /* 发射机关机 */
    if( xDTBTeleControlSend(ShutDown, DTB_CMD_TIMEOUT) != E_NO_ERR )
    {
        dtb_power_off();
        return E_NO_SS;
    }

    vTaskDelay(2000);

    dtb_power_off();

    return E_NO_ERR;
}

/**
 * 回放处理任务
 * @param para
 */
void dtb_mem_back_task(void *para)
{
    DTB_Busy = true;

    mem_back_bash *pBash = (mem_back_bash *)para;

    dtb_mem_back( pBash->mem_num, pBash->data_rate, pBash->back_last );

    DTB_Busy = false;

    vTaskDelete(NULL);
}

/**
 * 数传机回放事务，创建任务
 *
 * @param mem_num 内存区选择
 * @param data_rate 下行速率选择
 * @return 返回E_NO_ERR（-1）为正确
 */
int DTB_Mem_Back_Work(uint8_t mem_num, uint8_t data_rate, uint16_t back_last)
{
    static mem_back_bash back_bash;

    if( DTB_Busy == true )
        return E_TIMEOUT;

    back_bash.mem_num = mem_num;
    back_bash.data_rate = data_rate;

    back_last = back_last ? back_last : 600; //若回放参数设置为0则回放最大时间10分钟即600秒

    back_bash.back_last = back_last;

    if( xTaskCreate( dtb_mem_back_task, "BACK", 256, &back_bash, 1, NULL ) != pdTRUE )
        return E_OUT_OF_MEM;
    else
        return E_NO_ERR;
}

/**
 * 数传机射频端开关函数
 *
 * @param select 开为1，关为0
 * @return 函数执行成功返回E_NO_ERR(-1)
 */
int dtb_rf_switch( tr_sw_status select )
{
    if( select == tr_sw_on )
        return xDTBTeleControlSend(Boot, DTB_CMD_TIMEOUT);
    else
        return xDTBTeleControlSend(ShutDown, DTB_CMD_TIMEOUT);
}

/**
 * 数传伪码开关函数
 *
 * @param select 开为1，关为0
 * @return 函数执行成功返回E_NO_ERR(-1)
 */
int dtb_pseudo_switch( tr_sw_status select )
{
    if( select == tr_sw_on )
        return xDTBTeleControlSend(PseudoOn, DTB_CMD_TIMEOUT);
    else
        return xDTBTeleControlSend(PseudoOff, DTB_CMD_TIMEOUT);
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

