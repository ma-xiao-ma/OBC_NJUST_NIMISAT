/*
 * router_io.h
 *
 *  Created on: 2017年9月24日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_ROUTER_IO_H_
#define CONTRL_ROUTER_IO_H_

#include "route.h"
/***********************************************/
#define TTC_ROUTE_ADDR  0       /*星内节点*/
#define OBC_ROUTE_ADDR  1
#define ADCS_ROUTE_ADDR 2
#define EPS_ROUTE_ADDR  3
#define CAM_ROUTE_ADDR  5
/***********************************************/
#define GND_ROUTE_ADDR  64      /*地面节点*/
/***********************************************/

#define ROUTE_NODE_MAC 0xFF

#define MY_ROUTE_ADDR   1

#define SERVER_QUEUE_LEN            5
#define ADCS_QUEUE_LEN              5
#define SEND_PROCESSING_QUEUE_LEN   5

/**
 * 发送处理任务调用， 将待发送的分组信息通过相应接口发送出去
 *
 * @param packet 带发送分组
 * @return 返回E_NO_ERR（-1）表示函数执行成功
 */
int router_send_to_other_node(route_packet_t *packet);

/**
 * 路由器接收处理任务中调用，用于处理收到的送给本节点的分组
 *
 * @param packet 待处理分组
 * @return 返回E_NO_ERR（-1）表示函数执行成功
 */
int router_unpacket(route_packet_t *packet);

/**
 * 在路由器初始化函数中调用，用于初始化用户相关定义
 *
 * @return 返回E_NO_ERR（-1）表示函数执行成功
 */
int route_user_init(void);

#endif /* CONTRL_ROUTER_IO_H_ */
