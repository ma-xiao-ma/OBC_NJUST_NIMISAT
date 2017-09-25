/*
 * flash_sd.h
 *
 *  Created on: 11 Jun 2015
 *      Author: yuyongjun
 */

#ifndef CONTRL_SENSOR_FLASH_SD_H_
#define CONTRL_SENSOR_FLASH_SD_H_

#include <stdint.h>

typedef struct __attribute__((packed)){
	uint32_t rst_time;
	uint32_t rst_cnt;
}flash_store_t;

typedef struct __attribute__((packed)){
	unsigned int Down_Cmd_Delay ;
	unsigned int Stop_Down_Time ;
	unsigned int downtimeset ;
}flash_param_t;

typedef struct __attribute__((packed)){
	uint8_t req ;
	unsigned int time ;
}flash_uptime_t;

uint8_t write_flash(void);
uint8_t read_flash(void);

#endif /* CONTRL_SENSOR_FLASH_SD_H_ */
