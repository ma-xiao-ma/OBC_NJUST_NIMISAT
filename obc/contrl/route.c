/*
 * route.c
 *
 *  Created on: 2017年9月23日
 *      Author: Ma Wenli
 */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "error.h"
#include "driver_debug.h"

#include "route.h"
#include "router_io.h"

static xQueueHandle routing_queue;
static xQueueHandle server_queue;

int route_queue_init(uint32_t queue_len_router)
{
    if(routing_queue == NULL)
    {
        routing_queue = xQueueCreate(queue_len_router, sizeof(routing_packet_t *));
        if(!routing_queue)
            return E_OUT_OF_MEM;
    }
    return E_NO_ERR;
}

int server_queue_init(uint32_t queue_len_server)
{
    if(server_queue == NULL)
    {
        server_queue = xQueueCreate(queue_len_server, sizeof(routing_packet_t *));
        if(!server_queue)
            return E_OUT_OF_MEM;
    }
    return E_NO_ERR;
}



int route_queue_read(routing_packet_t ** packet)
{
    if (routing_queue == NULL)
        return E_NO_DEVICE;

    if (xQueueReceive(routing_queue, packet, ROUTE_READ_TIMEOUT) == pdFALSE)
        return E_TIMEOUT;

    return E_NO_ERR;
}

void route_queue_wirte(routing_packet_t *packet, portBASE_TYPE *pxTaskWoken)
{
    int result;

    if (routing_queue == NULL)
        printf("route queue not initialized!\r\n");

    if(packet == NULL)
        printf("route_queue_wirte called with NULL packet\r\n");

    if(pxTaskWoken == NULL)
        result = xQueueSendToBack(routing_queue, &packet, 0);
    else
        result = xQueueSendToBackFromISR(routing_queue, &packet, pxTaskWoken);

    if(result != pdTRUE)
    {
        printf("ERROR: Routing queue is FULL. Dropping packet.\r\n");
        qb50Free(packet);
    }
}

void router_task(void *param __attribute__((unused)))
{
    routing_packet_t *packet = NULL;
    uint16_t len    = 0;

    while(1)
    {
        if(route_queue_read(packet) != E_NO_ERR)
            continue;

        if(packet == NULL)
        {
            printf("router task detected with NULL packet\r\n");
            continue;
        }

        driver_debug(DEBUG_ROUTER, "INP: S %u, D %u, Sz %u, Tp 0x%02X\r\n",
                packet->src, packet->dst, packet->len, packet->typ);

        /*如果数据包是发给其他设备的，则调用对应发送函数*/
        if(packet->dst != router_get_my_address())
        {
            /*根据各分系统情况实现函数内容*/
            router_send_to_other_node(packet);
        }

        if(xQueueSendToBack(server_queue, &packet, 0) != pdTRUE)
        {
            printf("ERROR: Server queue is FULL. Dropping packet.\r\n");
            qb50Free(packet);
        }
    }
}

int router_start_task(uint32_t task_stack_size, uint32_t priority)
{
    int ret = xTaskCreate(router_task, "RTE", task_stack_size, NULL, priority, NULL);

    if(ret != pdPASS)
    {
        printf("Failed to start router task!\r\n");
        return E_OUT_OF_MEM;
    }

    return E_NO_ERR;
}

void server_task(void *param __attribute__((unused)))
{
    routing_packet_t *packet = NULL;
    uint16_t len    = 0;

    while(1)
    {
        if(xQueueReceive(server_queue, packet, portMAX_DELAY) != E_NO_ERR)
            continue;

        if(packet == NULL)
        {
            printf("Server task detected with NULL packet.\r\n");
            continue;
        }

        if(packet->dst != router_get_my_address())
        {
            printf("Server task packet error. Dropping packet.\r\n");
            qb50Free(packet);
        }

        router_unpacket(packet);
    }
}

int server_start_task(uint32_t task_stack_size, uint32_t priority)
{
    int ret = xTaskCreate(server_task, "server", task_stack_size, NULL, priority, NULL);

    if(ret != pdPASS)
    {
        printf("Failed to start server task!\r\n");
        return E_OUT_OF_MEM;
    }

    return E_NO_ERR;
}

