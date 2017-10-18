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
#include "FreeRTOS.h"

#define DOWNLINK_CRC_SIZE   4
#define DOWNLINK_OVERHEAD (ROUTE_HEAD_SIZE + DOWNLINK_CRC_SIZE)
#define DOWNLINK_MTU (ISIS_MTU - DOWNLINK_OVERHEAD)

#define IMAGE_DOWNLINK_OVERHEAD (DOWNLINK_OVERHEAD + IMAGE_PACK_HEAD_SIZE)

#define MAX_UPLINK_CONTENT_SIZE  64
/*obc给trxvu数据包的时间间隔*/
#define PACK_DOWN_INTERVAL (1 / portTICK_PERIOD_MS)

/*当发射机缓冲区满时等待的时间*/
#define MS_WAIT_TRANS_FREE_BUFF (5000 / portTICK_PERIOD_MS)

typedef struct
{
    uint8_t tpye;
    void * pdata;
    uint32_t data_len;
} downlink_request;

#endif /* CONTRL_IF_DOWNLINK_VU_H_ */
