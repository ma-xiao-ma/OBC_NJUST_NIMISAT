
#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "bsp_adc.h"

#define OBC_I2C_RECEIVEMASK_1		0x0F
#define OBC_I2C_RECEIVEMASK_2		0xF0

#define FlanshBlockSize				(32 * 1024)

#define SLEEP_MODE  0
#define NORMAL_MODE 1

extern uint8_t mode;
extern uint8_t IsRealTelemetry;

typedef struct __attribute__((packed))
{
    uint32_t execution_utc;
    uint8_t typ;
    uint8_t dat[0];
} delay_task_t;

void ControlTask(void * pvParameters);
void SendDownCmd(void *pData, uint32_t Length);
void down_save_task(void * pvParameters);
void taskdelaytime(uint32_t *time);
void downdelaytimes(uint32_t *times);
void chcontinuetimes(uint32_t *times);

void OpenPanel_Task(void* param);
void OpenAntenna_Task(void* param);


int Battery_Task(const EpsAdcValue_t *eps_hk);
void SleepWorkMode(void);
void NormalWorkMode(void);

/* 停止下行遥测函数 */
void vContrlStopDownload(void);

void hk_data_save_task(void);
void hk_down_proc_task(void);
void hk_down_store_task(void);

/**
 * 延时处理任务创建
 *
 * @param para 延时任务参数
 * @return
 */
int Delay_Task_Mon_Start(void *para);


void adcs_pwr_task(void *pvParameters __attribute__((unused)));
void hk_collect_task(void *pvParameters __attribute__((unused)));

#endif
