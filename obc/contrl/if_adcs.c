/*
 * if_adcs.c
 *
 *  Created on: 2017年9月29日
 *      Author: Ma Wenli
 */

#include "FreeRTOS.h"

#include "bsp_pca9665.h"
#include "error.h"
#include "cube_com.h"
#include "router_io.h"
#include "if_adcs.h"
#include "hk.h"
#include "contrl.h"

static xQueueHandle adcs_queue;

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
int adcs_queue_read(route_packet_t ** packet, TickType_t timeout)
{
    if (adcs_queue == NULL)
        return E_NO_DEVICE;

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
        printf("ADCS queue not initialized!\r\n");

    if(packet == NULL)
        printf("adcs_queue_wirte called with NULL packet\r\n");

    if(pxTaskWoken == NULL)
        result = xQueueSendToBack(adcs_queue, &packet, 0);
    else
        result = xQueueSendToBackFromISR(adcs_queue, &packet, pxTaskWoken);

    if(result != pdTRUE)
    {
        printf("ERROR: ADCS queue is FULL. Dropping packet.\r\n");
        qb50Free(packet);
        printf("Clean up ADCS queue.\r\n");
        route_queue_clean(adcs_queue, NULL);
    }
}

/**
 * 组路由包送到发送处理队列进行处理，响应收到ADCS队列进行处理
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
    route_packet_t * packet = (route_packet_t *)qb50Malloc(sizeof(route_packet_t) + txlen);

    if (packet == NULL)
        return E_NO_BUFFER;

    if((txlen > I2C_MTU - ROUTE_HEAD_SIZE) || (rxlen > I2C_MTU - ROUTE_HEAD_SIZE))
        return E_INVALID_BUF_SIZE;

    if(txlen)
        memcpy(&packet->dat[0], txbuf, txlen);

    packet->len = txlen;
    packet->dst = ADCS_ROUTE_ADDR;
    packet->src = OBC_ROUTE_ADDR;
    packet->typ = type;

    send_processing_queue_wirte(packet, NULL);

    packet = NULL;

    if(rxlen == 0)
        return E_NO_ERR;

    if(xQueueReceive(adcs_queue, &packet, timeout) != pdTRUE)
        return E_TIMEOUT;

    if(packet == NULL)
    {
        printf("ERROR: ADCS queue Receive with NULL packet.\r\n");
        return E_TIMEOUT;
    }

    memcpy(rxbuf, &packet->dat[0], rxlen);

    qb50Free(packet);
    return E_NO_ERR;
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
int adcs_transaction_direct(uint8_t type, void * txbuf, size_t txlen, void * rxbuf, size_t rxlen, uint16_t timeout)
{
    int ret, rlen;

    /*取txlen和rxlen之间较大值*/
    rlen = (txlen>rxlen) ? txlen: rxlen;

    route_frame_t * frame = (route_frame_t *)qb50Malloc(sizeof(route_frame_t) + rlen);

    if (frame == NULL)
        return E_NO_BUFFER;

    frame->dst = ADCS_ROUTE_ADDR;
    frame->src = OBC_ROUTE_ADDR;
    frame->typ = type;

    if ((txlen > I2C_MTU - ROUTE_HEAD_SIZE) || (rxlen > I2C_MTU - ROUTE_HEAD_SIZE))
        return E_INVALID_BUF_SIZE;

    if (txlen != 0)
        memcpy(&frame->dat[0], txbuf, txlen);

    if (rxlen)
        rlen = rxlen + 3;
    else
        rlen = 0;

    ret = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_I2C1_ADDR, frame,
            txlen + ROUTE_HEAD_SIZE, frame, rlen, timeout);

    if ((rxlen != 0) && (ret == E_NO_ERR))
        memcpy(rxbuf, &frame->dat[0], rxlen);

    qb50Free(frame);

    return ret;
}

int adcs_get_hk(const void *hk, uint16_t timeout)
{

    return adcs_transaction_direct(INS_OBC_GET_ADCS_HK, NULL, 0, hk, sizeof(adcs805_hk_t), timeout);
}

int adcs_send_mode(uint8_t eps_mode)
{

    uint8_t type = (eps_mode == SLEEP_MODE)?INS_LowPower_Mode_ON:INS_LowPower_Mode_OFF;

    return adcs_transaction_direct(type, NULL, 0, NULL, 0, 0);
}

