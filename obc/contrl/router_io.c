/*
 * router_io.c
 *
 *  Created on: 2017年9月24日
 *      Author: Ma Wenli
 */

#include <if_trxvu.h>
#include "FreeRTOS.h"
#include "route.h"
#include "error.h"
#include "bsp_pca9665.h"
#include "if_adcs.h"
#include "driver_debug.h"
#include "cube_com.h"
#include "obc_mem.h"
#include "if_downlink_vu.h"

#include "router_io.h"


int route_user_init(void)
{
    int ret;

    /**
     * 用户自定义的路由初始化函数
     */

    ret = adcs_queue_init();
    if(ret != E_NO_ERR)
        return ret;

    return E_NO_ERR;
}

/**
 *  通过路由地址查找MAC地址
 *
 * @param route_addr 路由地址，也就是网络地址
 * @return 若有MAC地址，则返回MAC地址，若未查找到，则返回ROUTE_NODE_MAC(0xFF)
 */
uint8_t route_find_mac(uint8_t route_addr)
{
    uint8_t mac_addr;

    /**
     * 分系统需要根据自身情况作调整
     */
    switch(route_addr)
    {
        case TTC_ROUTE_ADDR:

            return TRANSMITTER_I2C_ADDR;
        case ADCS_ROUTE_ADDR:

            return ADCS_I2C_ADDR;
        default:

            return ROUTE_NODE_MAC;
    }
}

int route_i2c0_tx(route_packet_t * packet, uint32_t timeout)
{
    /*把分组信息转换成I2C帧信息*/
    i2c_frame_t * frame = (i2c_frame_t *) packet;

    /*通过路由地址查找MAC地址*/
    if (route_find_mac(packet->dst) == ROUTE_NODE_MAC)
    {
        frame->dest = packet->dst;
    }
    else
    {
        frame->dest = route_find_mac(packet->dst);
    }

    /*添加路由头到I2C帧的长度字段 */
    frame->len += 3;
    frame->len_rx = 0;

    /*添加I2C帧到发送队列*/
    if (i2c_send(0, frame, timeout) != E_NO_ERR)
    {
        ObcMemFree(frame);
        return E_NO_DEVICE;
    }

    return E_NO_ERR;
}

int route_i2c1_tx(route_packet_t * packet, uint32_t timeout)
{
    /*把分组信息转换成I2C帧信息*/
    i2c_frame_t * frame = (i2c_frame_t *) packet;

    /*通过路由地址查找MAC地址*/
    if (route_find_mac(packet->dst) == ROUTE_NODE_MAC)
    {
        frame->dest = packet->dst;
    }
    else
    {
        frame->dest = route_find_mac(packet->dst);
    }

    /*添加路由头到I2C帧的长度字段 */
    frame->len += 3;
    frame->len_rx = 0;

    /*添加I2C帧到发送队列*/
    if (i2c_send(1, frame, timeout) != E_NO_ERR)
    {
        ObcMemFree(frame);
        return E_NO_DEVICE;
    }

    return E_NO_ERR;
}


int router_send_to_other_node(route_packet_t *packet)
{

    switch(packet->dst)
    {
        case ADCS_ROUTE_ADDR:

            route_i2c1_tx(packet, 0);
            break;
        case GND_ROUTE_ADDR:

#if USE_SERIAL_PORT_DOWNLINK_INTERFACE
            SendDownCmd(&packet->dst, packet->len + 3);
            ObcMemFree(packet);
#else
            vu_isis_router_downlink(packet);
#endif
            break;
        default:
            ObcMemFree(packet);
            break;
    }

    return E_NO_ERR;
}


int router_unpacket(route_packet_t *packet)
{

    /* 处理地面给星务的信息 */
    if (packet->src == GND_ROUTE_ADDR)
    {
        CubeUnPacket(packet);
        ObcMemFree(packet);
    }
    /* 处理姿控给星务的信息 */
    else if (packet->src == ADCS_ROUTE_ADDR)
    {
        switch (packet->typ)
        {
            case INS_OBC_GET_ADCS_HK:
                adcs_queue_wirte(packet, NULL);
                break;

            default:
                ObcMemFree(packet);
                break;
        }
    }

    /**
     * 处理过境标志,时间同步信息
     */
    else
        ObcMemFree(packet);

    return E_NO_ERR;
}



