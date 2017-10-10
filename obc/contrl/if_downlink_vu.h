/*
 * if_downlink_vu.h
 *
 *  Created on: 2017年10月5日
 *      Author: Ma Wenli
 */
#ifndef CONTRL_IF_DOWNLINK_VU_H_
#define CONTRL_IF_DOWNLINK_VU_H_

#include "if_trxvu.h"
#include "route.h"
#include "camera_805.h"

#define DOWNLINK_CRC_SIZE   4
#define DOWNLINK_OVERHEAD (ROUTE_HEAD_SIZE + DOWNLINK_CRC_SIZE)
#define DOWNLINK_MTU (ISIS_MTU - DOWNLINK_OVERHEAD)

#define IMAGE_DOWNLINK_OVERHEAD (DOWNLINK_OVERHEAD + IMAGE_PACK_HEAD_SIZE)

#define MAX_UPLINK_CONTENT_SIZE  64


#endif /* CONTRL_IF_DOWNLINK_VU_H_ */
