/*
 * downlink_interface.h
 *
 *  Created on: 2017年8月23日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_DOWNLINK_INTERFACE_H_
#define CONTRL_DOWNLINK_INTERFACE_H_

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "error.h"

/* 是否打开串口上下行 */
#define USE_SERIAL_PORT_DOWNLINK_INTERFACE  1

#define USART2_MTU 235

typedef struct __attribute__((packed))
{
    uint8_t len_rx;
    uint8_t data[235];

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

#endif /* CONTRL_DOWNLINK_INTERFACE_H_ */
