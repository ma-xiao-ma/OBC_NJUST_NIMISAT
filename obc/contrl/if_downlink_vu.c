/*
 * if_downlink_vu.c
 *
 *  Created on: 2017年10月5日
 *      Author: Ma Wenli
 */

#include "bsp_pca9665.h"
#include "router_io.h"
#include "error.h"
#include "crc.h"
#include "semphr.h"

#include "if_downlink_vu.h"


static uint32_t vu_rx_count; //ISISvu通信机接收上行消息计数

extern xSemaphoreHandle i2c_lock;

/**
 * ISIS通信机下行接口函数，由OBC本地调用
 *
 * @param type 下行消息类型
 * @param pdata 下行数据指针
 * @param len 下行数据字节数
 * @return
 */
int vu_isis_downlink(uint8_t type, void *pdata, uint32_t len)
{
    int ret;
    uint8_t Error, TxRemainBufSize, FrameDataSize, RemainSize = len;
    pdata = (uint8_t *)pdata;

    route_frame_t *downlink = qb50Malloc(I2C_MTU);
    if(downlink == NULL)
        return E_MALLOC_FAIL;

    downlink->dst = GND_ROUTE_ADDR;
    downlink->src = router_get_my_address();
    downlink->typ = type;

    do
    {
        FrameDataSize = (RemainSize < DOWNLINK_MTU) ? RemainSize : DOWNLINK_MTU;
        memcpy(downlink->dat, pdata, FrameDataSize);

        *(uint32_t *)(&downlink->dat[ROUTE_HEAD_SIZE+FrameDataSize]) =
                crc32_memory((uint8_t *)downlink, ROUTE_HEAD_SIZE+FrameDataSize);

        ret = vu_transmitter_send_frame(downlink, FrameDataSize+DOWNLINK_OVERHEAD, &TxRemainBufSize);

        /**如果传输成功，且通信机成功将消息加入发送缓冲区，发送指针后移 */
        if ((ret == E_NO_ERR) && (TxRemainBufSize != 0xFF))
        {
            RemainSize -= FrameDataSize;
            pdata += FrameDataSize;
        }
        else
        {
            /**若发射机缓冲区已满，则等待5秒钟*/
            if(TxRemainBufSize == 0)
                vTaskDelay(5000 / portTICK_PERIOD_MS);

            Error++;
        }

        if (RemainSize != 0)
            vTaskDelay(1 / portTICK_PERIOD_MS);

     /**若发送完成或者错误次数超过五次，则跳出循环 */
    }while ((RemainSize > 0) && (Error < 5));

    qb50Free(downlink);

    if (RemainSize == 0)
        return E_NO_ERR;
    else
    {   /**如果错误超过五次，则复位发射机*/
        vu_transmitter_software_reset();
        return E_TRANSMIT_ERROR;
    }
}

/**
 * ISIS通信机上行轮询任务
 *
 * @param para 任务参数，没有用到
 */
void vu_isis_uplink(void *para __attribute__((unused)))
{
    static uint16_t frame_num;

    while(1)
    {
        /**获取接收机缓冲区帧计数*/
        if (vu_receiver_get_frame_num(&frame_num) != E_NO_ERR)
            continue;

        if (frame_num == 0)
            continue;

        /**若缓冲区不为空，则接收帧到路由器*/
        if (vu_receiver_router_get_frame() != E_NO_ERR)
            continue;

        /**通信机接收上行消息计数加1*/
        vu_rx_count++;

        /**成功接收后移除此帧*/
        if (vu_receiver_remove_frame() != E_NO_ERR)
        {
            for (uint8_t repeat_time = 0; repeat_time < 5; repeat_time++)
            {
                if (vu_receiver_remove_frame() == E_NO_ERR)
                    break;

                if (repeat_time == 5)
                {
                    vu_receiver_software_reset();
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                }
            }
        }
    }
}

int vu_isis_router_downlink(route_packet_t *packet)
{
    if (packet == NULL)
        return E_NO_BUFFER;

    if (!device[ISIS_I2C_HANDLE].is_initialised)
    {
        qb50Free(packet);
        return E_NO_DEVICE;
    }

    if (packet->len + 4 > ISIS_MTU)
    {
        qb50Free(packet);
        return E_INVALID_BUF_SIZE;
    }

    i2c_frame_t * frame = (i2c_frame_t *) qb50Malloc(sizeof(i2c_frame_t));
    if (frame == NULL)
    {
        qb50Free(packet);
        return E_NO_BUFFER;
    }

    /* Take the I2C lock */
    xSemaphoreTake(i2c_lock, 10 * configTICK_RATE_HZ);

    frame->dest = TRANSMITTER_I2C_ADDR;
    frame->len = packet->len + 4;
    frame->len_rx = 0;
    frame->data[0] = TRANSMITTER_SEND_FRAME_DEFAULT;
    memcpy(&frame->data[1], &packet->dst, packet->len + 3);

    if (i2c_send(ISIS_I2C_HANDLE, frame, 0) != E_NO_ERR)
    {
        qb50Free(packet);
        qb50Free(frame);
        xSemaphoreGive(i2c_lock);
        return E_TIMEOUT;
    }

    qb50Free(packet);
    xSemaphoreGive(i2c_lock);
    return E_NO_ERR;
}


