/*
 * if_adcs.c 姿控分系统接口函数源代码文件
 *
 *  Created on: 2017年9月29日
 *      Author: Ma Wenli
 */
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "bsp_pca9665.h"
#include "error.h"
#include "cube_com.h"
#include "router_io.h"
#include "hk.h"
#include "contrl.h"
#include "obc_mem.h"

#include "if_adcs.h"

static xQueueHandle adcs_queue;

/**
 * 创建姿控分系统消息接收队列
 *
 * @return 返回E_NO_ERR（-1）表示创建正确
 */
int adcs_queue_init(void)
{
    if(adcs_queue == NULL)
    {
        adcs_queue = xQueueCreate(ADCS_QUEUE_LEN, sizeof(route_packet_t *));
        if(!adcs_queue)
            return E_OUT_OF_MEM;
    }
    return E_NO_ERR;
}

/**
 * 姿控系统消息队列读出函数
 *
 * @param packet 读出的路由数据包
 * @return 返回E_NO_ERR（-1）表示执行正确
 */
static int adcs_queue_read(route_packet_t ** packet, TickType_t timeout)
{
    if (adcs_queue == NULL)
        adcs_queue_init();

    if (xQueueReceive(adcs_queue, packet, timeout) == pdFALSE)
        return E_TIMEOUT;

    return E_NO_ERR;
}

/**
 * 姿控系统消息队列写入函数，server进程调用
 *
 * @param packet 路由数据包指针
 * @param pxTaskWoken 在任务中调用时次值应置为NULL空指针，在中断中应指向有效地址
 */
void adcs_queue_wirte(route_packet_t *packet, portBASE_TYPE *pxTaskWoken)
{
    int result;

    if (adcs_queue == NULL)
        adcs_queue_init();

    if(packet == NULL)
    {
        printf("adcs_queue_wirte called with NULL packet\r\n");
        return;
    }

    if(pxTaskWoken == NULL)
        result = xQueueSendToBack(adcs_queue, &packet, 0);
    else
        result = xQueueSendToBackFromISR(adcs_queue, &packet, pxTaskWoken);

    if(result != pdTRUE)
    {
        printf("ERROR: ADCS queue is FULL. Dropping packet.\r\n");
        ObcMemFree(packet);
        printf("Clean up ADCS queue.\r\n");
        route_queue_clean(adcs_queue, NULL);
    }
}

/**
 * 直接调用I2C接口函数进行发送，响应收到I2C RX queue
 *
 * @param type 消息类型（以前的命令字）
 * @param txbuf 发送内容指针
 * @param txlen 发送内容长度（字节）
 * @param rxbuf 接收缓冲区指针
 * @param rxlen 接收长度
 * @param timeout 读接收队列超时时间
 * @return E_NO_ERR为正常
 */
int adcs_transaction(uint8_t type, void * txbuf, size_t txlen, void * rxbuf, size_t rxlen, uint16_t timeout)
{

    extern xSemaphoreHandle i2c_lock;

    i2c_frame_t * frame = (i2c_frame_t *) ObcMemMalloc(sizeof(i2c_frame_t));
    if (frame == NULL)
        return E_NO_BUFFER;

    /* Take the I2C lock */
    xSemaphoreTake(i2c_lock, 10 * configTICK_RATE_HZ);

    frame->dest = ADCS_I2C_ADDR;
    frame->len = txlen + ROUTE_HEAD_SIZE;
    frame->len_rx = 0;

    route_frame_t * r_frame = (route_frame_t *)frame->data;

    r_frame->dst = ADCS_ROUTE_ADDR;
    r_frame->src = OBC_ROUTE_ADDR;
    r_frame->typ = type;


    memcpy(&r_frame->dat[0], txbuf, txlen);

    if (adcs_i2c_send(OBC_TO_ADCS_HANDLE, frame, 0) != E_NO_ERR)
    {
        ObcMemFree(frame);
        xSemaphoreGive(i2c_lock);
        return E_TIMEOUT;
    }

    if (rxlen == 0)
    {
        xSemaphoreGive(i2c_lock);
        return E_NO_ERR;
    }

    route_packet_t *packet;

    if (adcs_queue_read(&packet, timeout) != E_NO_ERR)
    {
        xSemaphoreGive(i2c_lock);
        return E_TIMEOUT;
    }

    /* 接收内容正确性检查 */
    if (packet->dst != OBC_ROUTE_ADDR || packet->src != ADCS_ROUTE_ADDR ||
            packet->typ != type || packet->len != rxlen)
    {
        ObcMemFree(packet);
        xSemaphoreGive(i2c_lock);
        return E_INVALID_PARAM;
    }

    memcpy(rxbuf, &packet->dat[0], rxlen);

    ObcMemFree(packet);
    xSemaphoreGive(i2c_lock);

    return E_NO_ERR;
}

/**
 * 获取姿控分系统遥测值
 *
 * @param hk 接收指针
 * @param timeout 超时值设置
 * @return E_NO_ERR（-1）为正常
 */
int adcs_get_hk(void *hk, uint16_t timeout)
{

    return adcs_transaction(INS_OBC_GET_ADCS_HK, NULL, 0, NULL/*hk*/, /*sizeof(adcs805_hk_t)*/0, 0/*timeout*/);
}

/**
 * 发送电源信息到姿控计算机
 *
 * @param eps_mode 正常模式或者休眠模式
 * @return E_NO_ERR为正常
 */
int adcs_send_mode(uint8_t eps_mode)
{

    uint8_t type = (eps_mode == SLEEP_MODE)?INS_LowPower_Mode_ON:INS_LowPower_Mode_OFF;

    return adcs_transaction(type, NULL, 0, NULL, 0, 0);
}

/**
 * 姿控时间同步
 *
 * @param secs utc时间
 * @return E_NO_ERR为正常
 */
int adcstimesync(uint32_t secs)
{
    return adcs_transaction(INS_ADCS_TIME_IN, &secs, sizeof(uint32_t), NULL, 0, 0);
}
