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
 * 路由器初始化函数，设置路由地址，创建相关队列
 *
 * @param address 路由地址设置值
 * @param router_queue_len 路由队列胜读
 * @return
 */
int router_init(uint8_t address, uint32_t router_queue_len);

/**
 * 路由队列写入函数，接口链路层和网络层的媒介
 *
 * @param packet 路由数据包指针
 * @param pxTaskWoken 在任务中调用时次值应置为NULL空指针，在中断中应指向有效地址
 */
void route_queue_wirte(route_packet_t *packet, portBASE_TYPE *pxTaskWoken);

/**
 * 获取路由地址设置值
 *
 * @return 路由地址
 */
uint8_t router_get_my_address(void);

/**
 * 路由地址设置函数
 *
 * @param addr 路由地址设置值
 */
void router_set_address(uint8_t addr);

/**
 * 路由任务创建
 *
 * @param task_stack_size 任务堆栈大小
 * @param priority 任务优先级
 * @return 任务创建成功返回E_NO_ERR（-1）
 */
int router_start_task(uint16_t task_stack_size, uint32_t priority);

/**
 * 接收处理任务创建
 *
 * @param task_stack_size 任务堆栈大小
 * @param priority 任务优先级
 * @return 任务创建成功返回E_NO_ERR（-1）
 */
int server_start_task(uint16_t task_stack_size, uint32_t priority);

/**
 * 路由器发送处理任务创建
 *
 * @param task_stack_size 任务堆栈大小
 * @param priority 任务优先级
 * @return 返回E_NO_ERR（-1）表示创建成功
 */
int send_processing_start_task(uint16_t task_stack_size, uint32_t priority);

/**
 * 路由器队列清理函数，清除已满的路由器队列元素，释放内存
 *
 * @param queue 队列句柄
 * @param pxTaskWoken 在任务中调用此项为0，在中断中调用时必须为一个有效指针
 */
void route_queue_clean(QueueHandle_t queue, portBASE_TYPE *pxTaskWoken);

#endif /* CONTRL_ROUTE_H_ */
