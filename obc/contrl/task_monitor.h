/*
 * task_monitor.h
 *
 *  Created on: 2017年11月7日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_TASK_MONITOR_H_
#define CONTRL_TASK_MONITOR_H_

#include <stdint.h>
#include "command.h"

typedef enum
{
    Collect = 1<<0,
    DownSave = 1<<1,
    Router = 1<<2,
    Server = 1<<3,
    Send = 1<<4
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


/**
 * Initialise and create supervisor task
 * This task will wake up to reset the hardware watchdog at a rate of initial_timeout/2.
 * A task can register a software heartbeat using the sv_add function, and decide its own timeout.
 * If a software heartbeat fails, the supervisor will not reset the hardware watchdog, and the system will reboot.
 * @param initial_timeout [ms] must be set to a value lower than the hardware watchdog timeout
 * @return 0 if OK, -1 if ERR
 */
int sv_init(uint32_t initial_timeout);

/**
 * Add a software heartbeat to the watchdog
 * @param name string containing the name of the task or the heartbeat
 * @param timeout [ms] must be set to a value min. 2 times higher than the expected heartbeat rate
 * @return id of software heartbeat (must be used with sv_reset(id) to clear), or -1 if ERR
 */
int sv_add(char * name, uint32_t timeout);

/**
 * Clear the software heartbeat timeout counter
 * @param id of software heartbeat
 * @return 0 if OK, -1 if ERR
 */
int sv_reset(unsigned int id);

/**
 * Print a list of all supervised tasks
 */
int sv_print(struct command_context *ctx);

#endif /* CONTRL_TASK_MONITOR_H_ */
