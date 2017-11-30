/*
 * if_adcs.h
 *
 *  Created on: 2017年9月29日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_IF_ADCS_H_
#define CONTRL_IF_ADCS_H_

#include "FreeRTOS.h"
#include "route.h"

#define OBC_TO_ADCS_HANDLE      0x01
#define ADCS_DELAY              1000U

#define ADCS_I2C_ADDR           ADCS_I2C1_ADDR

#define ADCS_I2C0_ADDR          0x06
#define ADCS_I2C1_ADDR          0x05

int adcs_get_hk(void *hk, uint16_t timeout);

int adcs_send_mode(uint8_t eps_mode);

/**
 * 姿控时间同步
 *
 * @param secs utc时间
 * @return E_NO_ERR为正常
 */
int adcstimesync(uint32_t secs);

int adcs_queue_init(void);

int adcs_queue_read(route_packet_t ** packet, TickType_t timeout);

void adcs_queue_wirte(route_packet_t *packet, portBASE_TYPE *pxTaskWoken);

int adcs_transaction(uint8_t type, void * txbuf, size_t txlen, void * rxbuf, size_t rxlen, uint16_t timeout);

#endif /* CONTRL_IF_ADCS_H_ */
