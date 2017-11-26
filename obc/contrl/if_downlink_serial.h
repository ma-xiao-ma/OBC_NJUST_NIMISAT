/*
 * if_downlink_serial.h
 *
 *  Created on: 2017年8月23日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_IF_DOWNLINK_SERIAL_H_
#define CONTRL_IF_DOWNLINK_SERIAL_H_

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "error.h"

/* 是否打开串口上下行 */
#define USE_SERIAL_PORT_DOWNLINK_INTERFACE  0

#define USART2_MTU 235

typedef struct __attribute__((packed))
{
    uint8_t  padding[8];
    uint16_t len_rx;
    uint8_t data[USART2_MTU];

} usart2_frame_t;

typedef struct
{
    xQueueHandle queue;
    usart2_frame_t * frame;

} usart2_transmission_object_t;

int vSerialSend(void *pdata, uint16_t length);
void vSerialACK(void *pdata, uint16_t length);
void vSerialInterfaceInit(void);
void USART2_Receive_Task(void *pvPara);

/**
 * 串口协议发送
 *
 * @param dst 目的地址
 * @param src 源地址
 * @param type 消息类型
 * @param pdata 待发送数据指针
 * @param len 待发送数据长度
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int ProtocolSerialSend( uint8_t dst, uint8_t src, uint8_t type, void *pdata, uint32_t len );

#endif /* CONTRL_IF_DOWNLINK_SERIAL_H_ */
