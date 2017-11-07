/*
 * task_monitor.c
 *
 *  Created on: 2017年11月7日
 *      Author: Ma Wenli
 */

#include <stdio.h>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "bsp_reset.h"
#include "bsp_iwdg.h"

#include "task_monitor.h"

#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"

#define BYTETOBINARY(byte)  \
  (byte & 0x80 ? 1 : 0), \
  (byte & 0x40 ? 1 : 0), \
  (byte & 0x20 ? 1 : 0), \
  (byte & 0x10 ? 1 : 0), \
  (byte & 0x08 ? 1 : 0), \
  (byte & 0x04 ? 1 : 0), \
  (byte & 0x02 ? 1 : 0), \
  (byte & 0x01 ? 1 : 0)

#define COLLECT_TASK_BIT    ( 1 << 0 )
#define DOWN_SAVE_TASK_BIT  ( 1 << 1 )
#define ROUTE_TASK_BIT      ( 1 << 2 )
#define SERVER_TASK_BIT     ( 1 << 3 )
#define SEND_TASK_BIT       ( 1 << 4 )
#define MONITOR_ALL_BIT     ( COLLECT_TASK_BIT | DOWN_SAVE_TASK_BIT | ROUTE_TASK_BIT | SERVER_TASK_BIT | SEND_TASK_BIT )


EventGroupHandle_t task_status;
TickType_t  monitor_window = 20000; // 超时时间默认20秒

/**
 *任务监视器初始化
 *
 * @param timeout 任务报告超时时间
 */
void supervisor_init(uint32_t timeout)
{
    /* 创建时间标志组 */
    task_status = xEventGroupCreate();

    /* 任务监视器超时时间 */
    monitor_window = timeout;
}

/**
 *线程监视任务，任务优先级需设为最高
 *
 * @param para
 */
void supervisor_task(void *para)
{
    EventBits_t EventValue;

    if ( task_status == NULL )
        supervisor_init( monitor_window );

    /* 硬件看门狗超时间20s */
    IWDG_Init(20000);

    while(1)
    {
        EventValue = xEventGroupWaitBits( task_status, (EventBits_t)MONITOR_ALL_BIT, pdTRUE, pdTRUE, monitor_window );

        IWDG_Feed();

//        printf("EventValue :"BYTETOBINARYPATTERN"\r\n", BYTETOBINARY((uint8_t)EventValue));

        if ( (EventValue & MONITOR_ALL_BIT) != MONITOR_ALL_BIT )
        {
            /**
             * 随后可以添加线程爆炸处理，暂时用计算机复位代替
             */
            if ( (EventValue & (EventBits_t)COLLECT_TASK_BIT) == 0)
                printf("Collect Task Already Exploded!!!\n");

            if ( (EventValue & (EventBits_t)DOWN_SAVE_TASK_BIT) == 0)
                printf("Down_Save Task Already Exploded!!!\n");

            if ( (EventValue & (EventBits_t)ROUTE_TASK_BIT) == 0)
                printf("Router Task Already Exploded!!!\n");

            if ( (EventValue & (EventBits_t)SERVER_TASK_BIT) == 0)
                printf("Server Task Already Exploded!!!\n");

            if ( (EventValue & (EventBits_t)SEND_TASK_BIT) == 0)
                printf("Send Task Already Exploded!!!\n");

            /* 计算机复位 */
            cpu_reset();
        }
    }
}

/**
 * 任务需要上报任务状态给监视器
 *
 * @param monitor_task_bit
 */
void task_report_alive(monitor_bit monitor_task_bit)
{
    if ( task_status == NULL )
        return;

    xEventGroupSetBits( task_status,  (EventBits_t)monitor_task_bit);
}
