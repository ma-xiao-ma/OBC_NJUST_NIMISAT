/*
 * dtb_805.h
 *
 *  Created on: 2017年9月18日
 *      Author: Ma Wenli
 *  描述：Data
 */

#ifndef CONTRL_DTB_805_H_
#define CONTRL_DTB_805_H_

int xDTBTelemetryGet(uint8_t *pRxData, uint16_t Timeout);

int xDTBTeleControlSend(uint8_t Cmd, uint16_t Timeout);

#endif /* CONTRL_DTB_805_H_ */
