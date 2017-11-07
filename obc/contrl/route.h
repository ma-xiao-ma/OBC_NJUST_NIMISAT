/*
 * route.h
 *
 *  Created on: 2017年9月23日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_ROUTE_H_
#define CONTRL_ROUTE_H_

#include "FreeRTOS.h"
#include "queue.h"

#define USE_ROUTE_PROTOCOL  1

#define ROUTE_QUEUE_READ_TIMEOUT    3000
#define SERVER_QUEUE_READ_TIMEOUT   4000
#define SEND_QUEUE_READ_TIMEOUT     5000

#define PADDING_BYTES       8
#define ROUTE_HEAD_SIZE     3


/*路由结构体*/
typedef struct __attribute__((packed))
{
    uint8_t    padding[PADDING_BYTES];
    uint16_t   len;         //data字段长度
    uint8_t    dst;         //目的地址
    uint8_t    src;         //源地址
    uint8_t    typ;         //消息类型
    uint8_t    dat[0];      //数据
}route_packet_t;

typedef struct __attribute__((packed))
{
    uint8_t    dst;         //目的地址
    uint8_t    src;         //源地址
    uint8_t    typ;         //消息类型
    uint8_t    dat[0];      //数据
}route_frame_t;

/**
 *
 * @param queue_len_router
 * @return
 */
int route_queue_init(uint32_t queue_len_router);

int server_queue_init(uint32_t queue_len_server);

int send_processing_queue_init(uint32_t queue_len_server);

int router_init(uint8_t address, uint32_t router_queue_len);

/**
 *
 * @param packet
 * @return
 */
int route_queue_read(route_packet_t ** packet);

void route_queue_wirte(route_packet_t *packet, portBASE_TYPE *pxTaskWoken);

int server_queue_read(route_packet_t ** packet);

void server_queue_wirte(route_packet_t *packet, portBASE_TYPE *pxTaskWoken);

int send_processing_queue_read(route_packet_t ** packet);

void send_processing_queue_wirte(route_packet_t *packet, portBASE_TYPE *pxTaskWoken);


/**
 *
 * @return
 */
uint8_t router_get_my_address(void);

void router_set_address(uint8_t addr);


/**
 *
 * @param task_stack_size
 * @param priority
 * @return
 */
int router_start_task(uint32_t task_stack_size, uint32_t priority);

int server_start_task(uint32_t task_stack_size, uint32_t priority);

int send_processing_start_task(uint32_t task_stack_size, uint32_t priority);


/**
 *
 * @param queue
 * @param pxTaskWoken
 */
void route_queue_clean(QueueHandle_t queue, portBASE_TYPE *pxTaskWoken);

#endif /* CONTRL_ROUTE_H_ */
