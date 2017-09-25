/*
 * router_io.h
 *
 *  Created on: 2017年9月24日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_ROUTER_IO_H_
#define CONTRL_ROUTER_IO_H_

#include "route.h"

#define ADCS_ROUTE_ADDR 2
#define GND_ROUTE_ADDR  64
#define MY_ROUTE_ADDR   1

#define SERVER_QUEUE_LEN 5

int router_send_to_other_node(routing_packet_t *packet);

int router_unpacket(routing_packet_t *packet);

#endif /* CONTRL_ROUTER_IO_H_ */
