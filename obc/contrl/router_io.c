/*
 * router_io.c
 *
 *  Created on: 2017年9月24日
 *      Author: Ma Wenli
 */

#include "FreeRTOS.h"
#include "router_io.h"
#include "route.h"
#include "error.h"
#include "bsp_pca9665.h"
#include "switches.h"
#include "driver_debug.h"

/** 路由软件定义地址 */
static uint8_t router_my_address;

void router_set_address(uint8_t addr)
{
    router_my_address = addr;
}

uint8_t router_get_my_address(void)
{
    return router_my_address;
}

int router_init(uint8_t address, uint32_t router_queue_len)
{
    int ret;

    router_set_address(address);

    ret = route_queue_init(router_queue_len);
    if(ret != E_NO_ERR)
        return ret;

    ret = server_queue_init(SERVER_QUEUE_LEN);
    if(ret != E_NO_ERR)
        return ret;
    /**
     * 想想还需要那些初始化
     */

    return E_NO_ERR;
}

int router_send_to_other_node(routing_packet_t *packet)
{
    /**
     * 根据各分系统情况做相应的处理
     */
    driver_debug(DEBUG_ROUTER, "OTP: S %u, D %u, Tp 0x%02X, Sz %u\r\n",
            packet->src, packet->dst, packet->typ, packet->len);

    switch(packet->dst)
    {
        case ADCS_ROUTE_ADDR:

            i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &packet->dst,
                            packet->len + 3, NULL, 0, 0);
            break;
        case GND_ROUTE_ADDR:

            SendDownCmd(&packet->dst, packet->len + 3);
            break;
        default:
            break;
    }

    qb50Free(packet);

    return E_NO_ERR;
}

int router_unpacket(routing_packet_t *packet)
{
    /**
     * 根据各分系统情况做相应的处理
     */


    qb50Free(packet);
    return E_NO_ERR;
}




