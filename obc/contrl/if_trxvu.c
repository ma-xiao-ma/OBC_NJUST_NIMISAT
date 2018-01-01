/*
 * if_trxvu.C
 *
 *  Created on: 2017年10月02日
 *      Author: Ma Wenli
 */

#include <stdint.h>
#include <inttypes.h>
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_pca9665.h"
#include "if_downlink_vu.h"
#include "semphr.h"
#include "error.h"
#include "obc_mem.h"
#include "crc.h"
#include "queue.h"

#include "if_trxvu.h"

extern xSemaphoreHandle i2c_lock;
extern pca9665_device_object_t device[2];

extern QueueHandle_t rx_tm_queue;

/**
 * Context: Task only
 */
static int i2c_isis_transaction(uint8_t addr, void * txbuf, size_t txlen, void * rxbuf, size_t rxlen, uint16_t timeout)
{
    if (ISIS_I2C_HANDLE >= pca9665_device_count)
        return E_NO_DEVICE;

    if (!device[ISIS_I2C_HANDLE].is_initialised)
        return E_NO_DEVICE;

    if ((txlen > I2C_MTU) || (rxlen > I2C_MTU))
        return E_INVALID_BUF_SIZE;

    i2c_frame_t * t_frame = (i2c_frame_t *) ObcMemMalloc(sizeof(i2c_frame_t));
    if (t_frame == NULL)
        return E_NO_BUFFER;

    /* Take the I2C lock */
    xSemaphoreTake(i2c_lock, 10 * configTICK_RATE_HZ);

    /* Temporarily disable the RX callback, because we wish the received message to go into the I2C queue instead */
    void * tmp_callback = device[ISIS_I2C_HANDLE].callback;
    device[ISIS_I2C_HANDLE].callback = NULL;

    t_frame->dest = addr;
    memcpy(&t_frame->data[ISIS_I2C_HANDLE], txbuf, txlen);
    t_frame->len = txlen;
    t_frame->len_rx = 0;

    if (i2c_send(ISIS_I2C_HANDLE, t_frame, 0) != E_NO_ERR) {
        ObcMemFree(t_frame);
        device[ISIS_I2C_HANDLE].callback = tmp_callback;
        xSemaphoreGive(i2c_lock);
        return E_TIMEOUT;
    }

    vTaskDelay(11); /** 主发主收之间需要一个短暂延时给通信机准备数据的时间 */

    if (rxlen == 0) { /*若i2c_send()函数执行成功则内存应该在底层被正确释放，此处不再释放内存*/
        device[ISIS_I2C_HANDLE].callback = tmp_callback;
        xSemaphoreGive(i2c_lock);
        return E_NO_ERR;
    }

    /**
     * 若有I2C主收需求，则执行下面代码
     */
    t_frame = (i2c_frame_t *) ObcMemMalloc(sizeof(i2c_frame_t));
    if (t_frame == NULL)
        return E_NO_BUFFER;

    t_frame->dest = addr;
    t_frame->len = 0;
    t_frame->len_rx = rxlen;

    if (i2c_send(ISIS_I2C_HANDLE, t_frame, 0) != E_NO_ERR) {
        ObcMemFree(t_frame);
        device[ISIS_I2C_HANDLE].callback = tmp_callback;
        xSemaphoreGive(i2c_lock);
        return E_TIMEOUT;
    }

    i2c_frame_t * r_frame;

    if ( (i2c_receive(ISIS_I2C_HANDLE, &r_frame, timeout) != E_NO_ERR) ||
            r_frame != t_frame) {
        device[ISIS_I2C_HANDLE].callback = tmp_callback;
        xSemaphoreGive(i2c_lock);
        return E_TIMEOUT;
    }

    memcpy(rxbuf, &r_frame->data[0], rxlen);

    ObcMemFree(r_frame);
    device[ISIS_I2C_HANDLE].callback = tmp_callback;
    xSemaphoreGive(i2c_lock);
    return E_NO_ERR;
}

/**
 *  ISIS vu收发机 无参数 无响应 指令
 *
 * @param addr  接收机地址或者发射机地址
 * @param cmd   指令在if_trxvu.h中定义
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
static int vu_cmd( vu_i2c_addr addr, uint8_t cmd )
{
	return i2c_isis_transaction( addr, &cmd,  1, NULL, 0, 0 );
}


/**
 *  ISIS vu收发机 无参数 有响应 指令
 *
 * @param addr  接收机地址或者发射机地址
 * @param cmd   指令在if_trxvu.h中定义
 * @param rsp   接收响应的缓冲区指针
 * @param rsplen 响应字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
static int vu_cmd_rsp( vu_i2c_addr addr, uint8_t cmd, void * rsp, size_t rsplen )
{
    return i2c_isis_transaction( addr, &cmd,  1, rsp, rsplen, ISIS_TIMEOUT );
}

/**
 *  ISIS vu收发机 有参数 无响应 指令
 *
 * @param addr  接收机地址或者发射机地址
 * @param cmd   指令在if_trxvu.h中定义
 * @param para  参数指针
 * @param paralen 参数字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
static int vu_cmd_par(vu_i2c_addr addr, uint8_t cmd, void * para, size_t paralen)
{
    cmd_with_para *dat = ObcMemMalloc(sizeof(cmd_with_para) + paralen);
    if (dat == NULL)
        return E_NO_BUFFER;

    dat->command = cmd;
    if (paralen > 0)
        memcpy(dat->parameter, para, paralen);

    int ret = i2c_isis_transaction( addr, dat, sizeof(cmd_with_para)+paralen, NULL, 0, 0 );

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
 * @return  E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
static int vu_cmd_par_rsp(vu_i2c_addr addr, uint8_t cmd, void * para, size_t paralen, void * rsp, size_t rsplen)
{
    cmd_with_para *dat = ObcMemMalloc(sizeof(cmd_with_para) + paralen);
    if (dat == NULL)
        return E_NO_BUFFER;

    dat->command = cmd;
    if (paralen > 0)
        memcpy(dat->parameter, para, paralen);

    int ret = i2c_isis_transaction( addr, dat, sizeof(cmd_with_para)+paralen, rsp, rsplen, ISIS_TIMEOUT );

    ObcMemFree(dat);
    return ret;
}


/**************************接收单元指令函数****************************/

/**
 * 接收单元看门狗复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_watchdog_reset(void)
{
    return vu_cmd(Receiver, RECEIVER_WATCHDOG_RESET);
}

/**
 * 接收单元软件复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_software_reset(void)
{
    return vu_cmd(Receiver, RECEIVER_SOFTWARE_RESET);
}

/**
 * 接收单元硬件复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_hardware_reset(void)
{
    return vu_cmd(Receiver, RECEIVER_HARDWARE_RESET);
}

/**
 * 获取通信机射频接收缓冲区，帧计数
 *
 * @param pnum 帧计数指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_get_frame_num(uint16_t *pnum)
{
    return vu_cmd_rsp(Receiver, RECEIVER_GET_FRAME_NUM, pnum, sizeof(uint16_t));
}

/**
 * 获取通信机 射频接收缓冲区最早收到的一帧
 *
 * @param frame 数据帧指针
 * @param content_size 带获取数据帧中帧内容字节数
 * @return  E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_get_frame(rsp_frame *frame, uint8_t content_size)
{
    return vu_cmd_rsp(Receiver, RECEIVER_GET_FRAME, frame, sizeof(rsp_frame)+content_size);
}

/**
 * 获取接收机缓冲区最早的一帧，送入路由队列，路由协议的星地接口
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_router_get_frame(void)
{
    rsp_frame * rsp;

    i2c_frame_t * frame = (i2c_frame_t *) ObcMemMalloc(sizeof(i2c_frame_t));
    if (frame == NULL)
        return E_NO_BUFFER;

    /* Take the I2C lock */
    xSemaphoreTake(i2c_lock, 10 * configTICK_RATE_HZ);

    frame->dest = RECEIVER_I2C_ADDR;
    frame->data[0] = RECEIVER_GET_FRAME;
    frame->len = 1;
    frame->len_rx = MAX_UPLINK_CONTENT_SIZE;

    if (i2c_send(ISIS_I2C_HANDLE, frame, 0) != E_NO_ERR)
    {
        ObcMemFree(frame);
        xSemaphoreGive(i2c_lock);
        return E_TIMEOUT;
    }

    if (i2c_receive(ISIS_I2C_HANDLE, &frame, ISIS_TIMEOUT) != E_NO_ERR)
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

    /*如果接收到错误帧*/
    if (!rsp->DateSize)
    {
        ObcMemFree(frame);
        return E_INVALID_PARAM;
    }

    /* 给多普勒和信号强度遥测变量赋值 */
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

    route_queue_wirte((route_packet_t *)frame, NULL);

    xSemaphoreGive(i2c_lock);
    return E_NO_ERR;
}

/**
 * 从通信机接收缓冲中清除最早的一帧
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_remove_frame(void)
{
    return vu_cmd(Receiver, RECEIVER_REMOVE_FRAME);
}

/**
 * 获取接收单元所有遥测
 *
 * @param receiver_tm 遥测数据指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_measure_tm(rsp_rx_tm *receiver_tm)
{
    return vu_cmd_rsp(Receiver, MEASURE_RECEIVER_TELEMETRY, receiver_tm, sizeof(rsp_rx_tm));
}

/**
 * 获取接收单元自上次复位以来运行时间
 *
 * @param uptime 运行时间指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_receiver_get_uptime(uint32_t *uptime)
{
    return vu_cmd_rsp(Receiver, REPORT_RECEIVER_UPTIME, uptime, sizeof(uint32_t));
}


/**************************发射单元指令函数****************************/

/**
 * 发射单元看门狗复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_watchdog_reset(void)
{
    return vu_cmd(Transmitter, TRANSMITTER_WATCHDOG_RESET);
}

/**
 * 发射单元软件复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_software_reset(void)
{
    return vu_cmd(Transmitter, TRANSMITTER_SOFTWARE_RESET);
}

/**
 * 发射单元硬件复位
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_hardware_reset(void)
{
    return vu_cmd(Transmitter, TRANSMITTER_HARDWARE_RESET);
}


/**
 * 发射单元发送AX.25数据帧，使用默认源呼号和目的呼号
 *
 * @param frame 数据帧指针
 * @param framelen 数据帧字节数
 * @param rsp 发射机缓冲区剩余空间
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_send_frame(void *frame, uint8_t framelen, uint8_t *rsp)
{
    if(framelen < 1 || framelen > ISIS_MTU)
        return E_INVALID_BUF_SIZE;

    return vu_cmd_par_rsp(Transmitter, TRANSMITTER_SEND_FRAME_DEFAULT, frame, framelen, rsp, sizeof(uint8_t));
}

/**
 * 发射单元发送AX.25数据帧，使用设置 的源呼号和目的呼号覆盖默认呼号
 *
 * @param frame 带呼号数据帧指针
 * @param framelen 数据帧字节数
 * @param rsp 发射机缓冲区剩余空间
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_send_frame_new_callsigns(par_frame_new_call *frame, uint8_t content_len, uint8_t *rsp)
{
    if(content_len < 1 || content_len > ISIS_MTU)
        return E_INVALID_BUF_SIZE;

    return vu_cmd_par_rsp(Transmitter, SEND_FRAME_NEW_CALLSIGN, frame, sizeof(par_frame_new_call)+content_len,
            rsp, sizeof(uint8_t));
}

/**
 *设置信标
 *
 * @param beacon 信标设置结构体指针
 * @param content_len 信标内容字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_beacon_set(par_beacon_set *beacon, uint8_t content_len)
{
    if(content_len < 1 || content_len > ISIS_MTU)
        return E_INVALID_BUF_SIZE;

    return vu_cmd_par(Transmitter, TRANSMITTER_SET_BEACON, beacon, sizeof(par_beacon_set)+content_len);
}

/**
 *设置覆盖默认呼号的信标
 *
 * @param beacon 信标设置结构体指针
 * @param content_len 信标内容字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_beacon_new_call_set(par_beacon_new_call_set *beacon, uint8_t content_len)
{
    if(content_len < 1 || content_len > ISIS_MTU)
        return E_INVALID_BUF_SIZE;

    return vu_cmd_par(Transmitter, SET_BEACON_NEW_CALLSIGN, beacon, sizeof(par_beacon_new_call_set)+content_len);
}

/**
 * 清除信标
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_clear_beacon(void)
{
    return vu_cmd(Transmitter, TRANSMITTER_CLEAR_BEACON);
}

/**
 * 设置发射机默认呼号
 *
 * @param callsigns 呼号设置结构体指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_set_callsigns(par_default_call_set *callsigns)
{
    int ret = vu_cmd_par(Transmitter, SET_DEFAULT_TO_CALLSIGN, callsigns, 7);

    vTaskDelay(10/portTICK_PERIOD_MS);
    ret = vu_cmd_par(Transmitter, SET_DEFAULT_FROM_CALLSIGN, (uint8_t *)SET_DEFAULT_FROM_CALLSIGN+7, 7);

    return ret;
}

/**
 * 设置发射机空闲模式（即发送缓冲区为空时）是停止发射机工作，还是任然继续发送空序列，以便让地面锁定
 *
 * @param state 空闲状态，枚举数据类型，应为TurnOff或RemainOn其中之一
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_set_idle_state(par_idle_state state)
{
    return vu_cmd_par(Transmitter, SET_TRANSMITTER_DILE_STATE, &state, sizeof(uint8_t));
}

/**
 * 发射单元获取所有遥测
 *
 * @param transmitter_tm 发送单元遥测结构体指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_measure_tm(rsp_tx_tm *transmitter_tm)
{
    return vu_cmd_rsp(Transmitter, MEASURE_TRANSMITTER_TELEMETRY, transmitter_tm, sizeof(rsp_tx_tm));
}

/**
 * 发射单元获取上次发射时遥测
 *
 * @param last_tm 发送单元遥测结构体指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_get_last_tm(rsp_tx_tm *last_tm)
{
    return vu_cmd_rsp(Transmitter, GET_TRANSMITTER_TELEMETRY_LAST, last_tm, sizeof(rsp_tx_tm));
}

/**
 * 设置发射机波特率
 *
 * @param bitrate 发射波特率，枚举类型bps1200，bps2400，bps4800，bps9600其中之一
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_set_bitrate(par_transmission_bitrate bitrate)
{
    return vu_cmd_par(Transmitter, SET_TRANSMISSION_BITRATE, &bitrate, sizeof(uint8_t));
}

/**
 * 获取发射单元自上次复位以来运行时间
 *
 * @param uptime 运行时间指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_get_uptime(uint32_t *uptime)
{
    return vu_cmd_rsp(Transmitter, REPORT_TRANSMITTER_UPTIME, uptime, sizeof(uint32_t));
}

/**
 * 获取发射机工作状态
 *
 * @param state 工作状态结构体指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_transmitter_get_state(rsp_transmitter_state *state)
{
    return vu_cmd_rsp(Transmitter, REPORT_TRANSMITTER_STATE, state, sizeof(rsp_transmitter_state));
}
