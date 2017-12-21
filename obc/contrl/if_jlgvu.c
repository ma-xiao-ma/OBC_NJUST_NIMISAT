/*
 * if_jlgvu.c
 *
 *  Created on: 2017年10月12日
 *      Author: Ma Wenli
 */
#include <stdint.h>
#include <inttypes.h>
#include <stddef.h>
#include <string.h>

#include "bsp_pca9665.h"
#include "if_downlink_vu.h"
#include "semphr.h"
#include "error.h"
#include "obc_mem.h"
#include "crc.h"
#include "if_trxvu.h"

#include "if_jlgvu.h"

extern xSemaphoreHandle i2c_lock;
extern QueueHandle_t rx_tm_queue;

/**
 *  ISIS vu收发机 无参数 无响应 指令
 *
 * @param addr  接收机地址或者发射机地址
 * @param cmd   指令在if_trxvu.h中定义
 * @return      E_NO_ERR（-1）说明传输成功
 */
static int vu_cmd( uint8_t cmd )
{
    return i2c_master_transaction(JLG_VU_I2C_HANDLE, JLG_VU_I2C_ADDR, &cmd,  1, NULL, 0, 0);
}


/**
 *  ISIS vu收发机 无参数 有响应 指令
 *
 * @param addr  接收机地址或者发射机地址
 * @param cmd   指令在if_trxvu.h中定义
 * @param rsp   接收响应的缓冲区指针
 * @param rsplen 响应字节数
 * @return      E_NO_ERR（-1）说明传输成功
 */
static int vu_cmd_rsp( uint8_t cmd, void * rsp, size_t rsplen )
{
    return i2c_master_transaction(JLG_VU_I2C_HANDLE, JLG_VU_I2C_ADDR, &cmd,  1, rsp, rsplen, JLG_VU_TIMEOUT);
}

/**
 *  ISIS vu收发机 有参数 无响应 指令
 *
 * @param addr  接收机地址或者发射机地址
 * @param cmd   指令在if_trxvu.h中定义
 * @param para  参数指针
 * @param paralen 参数字节数
 * @return E_NO_ERR（-1）说明传输成功
 */
static int vu_cmd_par( uint8_t cmd, void * para, size_t paralen )
{
    cmd_with_para *dat = ObcMemMalloc(sizeof(cmd_with_para) + paralen);

    dat->command = cmd;
    if (paralen > 0)
        memcpy(dat->parameter, para, paralen);

    int ret = i2c_master_transaction(JLG_VU_I2C_HANDLE, JLG_VU_I2C_ADDR, dat, sizeof(cmd_with_para)+paralen, NULL, 0, 0);

    ObcMemFree(dat);
    return ret;
}

/**
 *  ISIS vu收发机 有参数 有响应 指令
 *
 * @param addr  接收机地址或者发射机地址
 * @param cmd   指令在if_trxvu.h中定义
 * @param para  参数指针
 * @param paralen 参数字节数
 * @param rsp   接收响应的缓冲区指针
 * @param rsplen 响应字节数
 * @return      E_NO_ERR（-1）说明传输成功
 */
static int vu_cmd_par_rsp( uint8_t cmd, void * para, size_t paralen, void * rsp, size_t rsplen )
{
    cmd_with_para *dat = ObcMemMalloc(sizeof(cmd_with_para) + paralen);

    dat->command = cmd;
    if (paralen > 0)
        memcpy(dat->parameter, para, paralen);

    int ret = i2c_master_transaction(JLG_VU_I2C_HANDLE, JLG_VU_I2C_ADDR, dat, sizeof(cmd_with_para)+paralen, rsp, rsplen, JLG_VU_TIMEOUT);

    ObcMemFree(dat);
    return ret;
}

/**************************接口函数****************************/

/**
 * 接收单元看门狗复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_watchdog_reset(void)
{
    return vu_cmd( WATCHDOG_RESET );
}

/**
 * 接收单元软件复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_software_reset(void)
{
    return vu_cmd( SOFTWARE_RESET);
}


/**
 * 获取通信机射频接收缓冲区，帧计数
 *
 * @param pnum 帧计数指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_get_frame_num(uint16_t *pnum)
{
    return vu_cmd_rsp( GET_FRAME_NUM, pnum, sizeof(uint16_t));
}

/**
 * 获取通信机 射频接收缓冲区最早收到的一帧
 *
 * @param frame 数据帧指针
 * @param content_size 带获取数据帧中帧内容字节数
 * @return  E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_get_frame(rsp_frame *frame, uint8_t content_size)
{
    return vu_cmd_rsp( GET_FRAME, frame, sizeof(rsp_frame)+content_size);
}

/**
 * 获取接收机缓冲区最早的一帧，送入路由队列，路由协议的星地接口
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_router_get_frame(void)
{
    rsp_frame * rsp;

    i2c_frame_t * frame = (i2c_frame_t *) ObcMemMalloc(sizeof(i2c_frame_t));
    if (frame == NULL)
        return E_NO_BUFFER;

    /* Take the I2C lock */
    xSemaphoreTake(i2c_lock, 10 * configTICK_RATE_HZ);

    frame->dest = JLG_VU_I2C_ADDR;
    frame->data[0] = GET_FRAME;
    frame->len = 1;
    frame->len_rx = MAX_UPLINK_CONTENT_SIZE;

    if (i2c_send(JLG_VU_I2C_HANDLE, frame, 0) != E_NO_ERR)
    {
        ObcMemFree(frame);
        xSemaphoreGive(i2c_lock);
        return E_TIMEOUT;
    }

    if (i2c_receive(JLG_VU_I2C_HANDLE, &frame, ISIS_TIMEOUT) != E_NO_ERR)
    {
        xSemaphoreGive(i2c_lock);
        return E_TIMEOUT;
    }

    if ((frame->len < 9) || (frame->len > I2C_MTU))
    {
        ObcMemFree(frame);
        return E_INVALID_PARAM;
    }

    rsp = (rsp_frame *)frame->data;

    /**
     * 给多普勒和信号强度遥测变量赋值
     */

    if(rx_tm_queue != NULL)
    {
        receiving_tm rx_tm =
        {
                rx_tm.DopplerOffset = rsp->DopplerOffset,
                rx_tm.RSSI = rsp->RSSI
        };

        xQueueOverwrite(rx_tm_queue, &rx_tm);
    }

    if (*(uint32_t *)(&rsp->Data[rsp->DateSize-4]) != crc32_memory(rsp->Data, rsp->DateSize-4))
    {
        ObcMemFree(frame);
        return E_CRC_CHECK_ERROR;
    }

    memcpy(rsp, rsp->Data, rsp->DateSize);
    frame->len -= 9;

    route_queue_wirte( (route_packet_t *)frame, NULL );

    xSemaphoreGive(i2c_lock);
    return E_NO_ERR;
}

/**
 * 从通信机接收缓冲中清除最早的一帧
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_remove_frame(void)
{
    return vu_cmd( REMOVE_FRAME );
}

/**
 * 获取vu所有遥测
 *
 * @param receiver_tm 遥测数据指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_measure_all_tm(rsp_vu_tm *receiver_tm)
{
    return vu_cmd_rsp( MEASURE_ALL_TELEMETRY, receiver_tm, sizeof(rsp_vu_tm));
}

/**
 * 获取vu自上次复位以来运行时间
 *
 * @param uptime 运行时间指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_get_uptime(uint32_t *uptime)
{
    return vu_cmd_rsp( REPORT_UPTIME, uptime, sizeof(uint32_t));
}


/**
 * 发射单元发送AX.25数据帧，使用默认源呼号和目的呼号
 *
 * @param frame 数据帧指针
 * @param framelen 数据帧字节数，最大235个字节
 * @param rsp 发射机缓冲区剩余空间
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_send_frame(void *frame, uint8_t framelen, uint16_t *rsp)
{
    if(framelen < 1 || framelen > ISIS_MTU)
        return E_INVALID_BUF_SIZE;

    return vu_cmd_par_rsp( SEND_FRAME_DEFAULT, frame, framelen, rsp, sizeof(uint16_t));
}

/**
 * 发射单元发送AX.25数据帧，使用设置 的源呼号和目的呼号覆盖默认呼号
 *
 * @param frame 带呼号数据帧指针
 * @param framelen 数据帧字节数
 * @param rsp 发射机缓冲区剩余空间
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_send_frame_new_callsigns(par_frame_new_call *frame, uint8_t content_len, uint8_t *rsp)
{
    if(content_len < 1 || content_len > ISIS_MTU)
        return E_INVALID_BUF_SIZE;

    return vu_cmd_par_rsp( SEND_FRAME_NEW_CALLSIGN, frame, sizeof(par_frame_new_call)+content_len,
            rsp, sizeof(uint8_t));
}

/**
 *设置信标
 *
 * @param beacon 信标设置结构体指针
 * @param content_len 信标内容字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_beacon_set(par_beacon_set *beacon, uint8_t content_len)
{
    if(content_len < 1 || content_len > ISIS_MTU)
        return E_INVALID_BUF_SIZE;

    return vu_cmd_par( TRANSMITTER_SET_BEACON, beacon, sizeof(par_beacon_set)+content_len );
}

/**
 *设置覆盖默认呼号的信标
 *
 * @param beacon 信标设置结构体指针
 * @param content_len 信标内容字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_beacon_new_call_set(par_beacon_new_call_set *beacon, uint8_t content_len)
{
    if(content_len < 1 || content_len > ISIS_MTU)
        return E_INVALID_BUF_SIZE;

    return vu_cmd_par( SET_BEACON_NEW_CALLSIGN, beacon, sizeof(par_beacon_new_call_set)+content_len);
}

/**
 * 清除信标
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_clear_beacon(void)
{
    return vu_cmd( CLEAR_BEACON );
}

/**
 * 设置发射机默认呼号
 *
 * @param callsigns 呼号设置结构体指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_set_callsigns(par_default_call_set *callsigns)
{
    int ret = vu_cmd_par( SET_DEFAULT_TO_CALL, callsigns, 7);

    vTaskDelay(10/portTICK_PERIOD_MS);
    ret = vu_cmd_par( SET_DEFAULT_FROM_CALL, (uint8_t *)SET_DEFAULT_FROM_CALLSIGN+7, 7);

    return ret;
}

/**
 * 设置发射机空闲模式（即发送缓冲区为空时）是停止发射机工作，还是任然继续发送空序列，以便让地面锁定
 *
 * @param state 空闲状态，枚举数据类型，应为TurnOff或RemainOn其中之一
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_set_idle_state(par_idle_state state)
{
    return vu_cmd_par( SET_DILE_STATE, &state, sizeof(uint8_t));
}


/**
 * 设置发射机波特率
 *
 * @param bitrate 发射波特率，枚举类型bps1200，bps2400，bps4800，bps9600其中之一
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_set_bitrate(par_transmission_bitrate bitrate)
{
    return vu_cmd_par( SET_BITRATE, &bitrate, sizeof(uint8_t));
}

/**
 * 获取发射机工作状态
 *
 * @param state 工作状态结构体指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_get_state(rsp_transmitter_state *state)
{
    return vu_cmd_rsp( TRANSMITTER_STATE, state, sizeof(rsp_transmitter_state));
}

/**
 * fm转发开
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_fm_on(void)
{
    return vu_cmd( FM_FORWARDING_ON );
}

/**
 * fm转发关
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_fm_off(void)
{
    return vu_cmd( FM_FORWARDING_OFF );
}

