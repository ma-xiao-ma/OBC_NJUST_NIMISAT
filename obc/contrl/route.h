/*
 * route.h
 *
 *  Created on: 2017年9月23日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_ROUTE_H_
#define CONTRL_ROUTE_H_

#include "FreeRTOS.h"

#define ROUTE_READ_TIMEOUT portMAX_DELAY

#define PADDING_BYTES 8
/*路由结构体*/
typedef struct __attribute__((packed))
{
    uint8_t    padding[PADDING_BYTES];
    uint16_t   len;         //data字段长度
    uint8_t    dst;         //目的地址
    uint8_t    src;         //源地址
    uint8_t    typ;         //消息类型
    uint8_t    dat[0];      //数据
}routing_packet_t;

int route_queue_init(uint32_t queue_len_router);

int server_queue_init(uint32_t queue_len_server);

int route_queue_read(routing_packet_t ** packet);

void route_queue_wirte(routing_packet_t *packet, portBASE_TYPE *pxTaskWoken);

uint8_t router_get_my_address(void);

void router_set_address(uint8_t addr);

int router_init(uint8_t address, uint32_t router_queue_len);

int router_start_task(uint32_t task_stack_size, uint32_t priority);

int server_start_task(uint32_t task_stack_size, uint32_t priority);

#endif /* CONTRL_ROUTE_H_ */
