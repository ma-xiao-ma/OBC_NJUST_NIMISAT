/*
 * task_monitor.h
 *
 *  Created on: 2017年11月7日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_TASK_MONITOR_H_
#define CONTRL_TASK_MONITOR_H_

typedef enum
{
    Collect = 1,
    DownSave = 2,
    Router = 4,
    Server = 8,
    Send = 16
} monitor_bit;

/**
 *任务监视器初始化
 *
 * @param timeout 任务报告超时时间
 */
void supervisor_init(uint32_t timeout);

/**
 *线程监视任务，任务优先级需设为最高
 *
 * @param para
 */
void supervisor_task(void *para);

/**
 * 任务需要上报任务状态给监视器
 *
 * @param monitor_task_bit
 */
void task_report_alive(monitor_bit monitor_task_bit);

#endif /* CONTRL_TASK_MONITOR_H_ */
