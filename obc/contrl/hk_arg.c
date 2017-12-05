/*
 * hk.c
 *
 *  Created on: 2016骞�5鏈�13鏃�
 *      Author: Administrator
 */

#include "hk_arg.h"

double mag_get[3] = {0.0,0.0,0.0};
float magtemparature;

uint16_t ad7490_data[16];

unsigned int obc_boot_count = 0;
uint32_t     obc_reset_time = 0;

uint8_t up_hk_down_cmd = 0;  //开始下行状态标识变量

uint32_t StorageIntervalCount = 5;
uint32_t Stop_Down_Time = 400;		//
uint32_t xDownFrequency = 3000;
uint32_t downtimeset = 3000;		//
//uint32_t saveinterval = 29;

uint8_t openpanel_times = 0;
uint32_t antenna_status = 0;
uint32_t panel_status = 0;

flash_store_t store_info = {0,0};

//uint8_t updateTimeFlag = 0;
//uint8_t AdcsGpsUse = 0;


