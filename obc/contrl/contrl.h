
#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define OBC_I2C_HANDLE				1
#define OBC_I2C_RECEIVEMASK_1		0x0F
#define OBC_I2C_RECEIVEMASK_2		0xF0
#define ADCS_I2C_ADDR				0x06
#define FlanshBlockSize				(32 * 1024)


extern uint8_t mode;
extern uint8_t IsRealTelemetry;


void ControlTask(void * pvParameters);
void SendDownCmd(void *pData, uint32_t Length);
void down_save_task(void * pvParameters);
void taskdelaytime(uint32_t *time);
void downdelaytimes(uint32_t *times);
void chcontinuetimes(uint32_t *times);

void OpenPanel_Task(void* param);
void OpenAntenna_Task(void* param);

void ControlTask(void * pvParameters __attribute__((unused)));

int Battery_Task(void);

/* 停止下行遥测函数 */
void vContrlStopDownload(void);

void hk_data_save_task(void);
void hk_down_proc_task(void);
void hk_down_store_task(void);
void DownloadSavedAudioFiles (void *para);
void SaveNewAudioFiles (void *para);
void DownloadNewAudioFiles (void *para);
void SavePermanentAudioFiles (void *para);

void adcs_pwr_task(void *pvParameters __attribute__((unused)));
void i2c_server_task(void *param __attribute__((unused)));
void isis_read_task(void *para __attribute__((unused)));

#endif
