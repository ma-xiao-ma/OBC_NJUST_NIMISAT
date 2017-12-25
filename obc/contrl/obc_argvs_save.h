/*
 * obc_argvs_save.h
 *
 *  Created on: 2016年6月23日
 *      Author: Administrator
 */

#ifndef CONTRL_OBC_ARGVS_SAVE_H_
#define CONTRL_OBC_ARGVS_SAVE_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"

typedef void (*delay_task) (void *para);

typedef struct __attribute__((packed))
{
    delay_task      task_function;
    uint8_t         task_name[4];
    uint8_t         task_para[60];
} task_save;


typedef struct __attribute__((packed))
{
	uint32_t    obc_boot_count;
	uint32_t    obc_reset_time;
	uint32_t 	antenna_status;
	uint32_t    hk_down_cnt;
	uint32_t    hk_store_cnt;
	uint32_t    vu_rec_cnt;         /*vu通信机接收指令计数*/
	uint32_t    backup_vu_rec_cnt;  /*备份vu通信机接收指令计数*/
	task_save   delay_task_recover[5];
} obc_save_t;



uint8_t obc_argvs_store(void);
uint8_t obc_argvs_recover(void);

#endif /* CONTRL_OBC_ARGVS_SAVE_H_ */
