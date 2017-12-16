/*
 * obc_argvs_save.c
 *
 *  Created on: 2016年6月23日
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx.h"

#include "bsp_ds1302.h"
#include "bsp_nor_flash.h"
#include "bsp_cpu_flash.h"

#include "contrl.h"
#include "hk_arg.h"
#include "obc_argvs_save.h"

extern uint32_t hk_down_cnt;
extern uint32_t hk_store_cnt;

obc_save_t obc_save = {0};

uint8_t obc_argvs_store(void)
{
	uint8_t res = 1;

//	NOR_ReadBuffer(0, (uint8_t*)&obc_save, sizeof(obc_save));
//	obc_save.obc_reset_time = clock_get_time_nopara();
//	obc_save.antenna_status = antenna_status;
//	res = flash_program(0, (uint8_t*)&obc_save, sizeof(obc_save), 1);

//	res = bsp_ReadCpuFlash(OBC_STORE_ADDR, (uint8_t*)&obc_save, sizeof(obc_save));

	obc_save.obc_reset_time = clock_get_time_nopara();
	obc_save.antenna_status = antenna_status;
	obc_save.hk_down_cnt = hk_down_cnt;
	obc_save.hk_store_cnt = hk_store_cnt;
    for (int i = 0; i < 5; i++)
    {
        delay_task_t *task_para = (delay_task_t *)obc_save.delay_task_recover[i].task_para;

        if (clock_get_time_nopara() > task_para->execution_utc && task_para->execution_utc != 0)
            task_para->execution_utc = 0;
    }

	res = bsp_WriteCpuFlash(OBC_STORE_ADDR, (uint8_t*)&obc_save, sizeof(obc_save));

	return res;
}

uint8_t obc_argvs_recover(void) {
	uint8_t res = 1;

//	NOR_ReadBuffer(0, (uint8_t*)&obc_save, sizeof(obc_save));
//	if(obc_save.obc_boot_count == 0xFFFFFFFF || obc_save.obc_boot_count == 0) {
//		obc_save.obc_boot_count = 1;
//	}else{
//		obc_save.obc_boot_count = obc_save.obc_boot_count + 1;
//	}
//	res = flash_program(0, (uint8_t*)&obc_save, sizeof(obc_save), 1);

	if(bsp_ReadCpuFlash(OBC_STORE_ADDR, (uint8_t*)&obc_save, sizeof(obc_save)) == 1)
	{
		obc_boot_count = 0;
		obc_reset_time = 0;
		antenna_status = 0;

		return 1;
	}
	else
	{

		if(obc_save.obc_boot_count == 0xFFFFFFFF || obc_save.obc_boot_count == 0)
		{
			obc_save.obc_boot_count = 1;
		}
		else
		{
			obc_save.obc_boot_count = obc_save.obc_boot_count + 1;
		}

		res = bsp_WriteCpuFlash(OBC_STORE_ADDR, (uint8_t*)&obc_save, sizeof(obc_save));

		if(obc_save.obc_reset_time == 0xFFFFFFFF)
		{
			obc_save.obc_reset_time = 0;
		}

		if(obc_save.antenna_status != 0 && obc_save.antenna_status != 1 && obc_save.antenna_status != 2)
		{
			obc_save.antenna_status = 0;
		}

        if(obc_save.hk_down_cnt == 0xFFFFFFFF)
        {
            obc_save.hk_down_cnt = 0;
        }

        if(obc_save.hk_store_cnt == 0xFFFFFFFF)
        {
            obc_save.hk_store_cnt = 0;
        }
	}
	/* 延时任务恢复 */
	for (int i = 0; i < 5; i++)
	{
	    delay_task_t *task_para = (delay_task_t *)obc_save.delay_task_recover[i].task_para;

	    if (clock_get_time_nopara() < task_para->execution_utc &&
	            task_para->execution_utc != 0xFFFFFFFF && task_para->execution_utc != 0)
	    {
	        xTaskCreate(obc_save.delay_task_recover[i].task_function,
	                obc_save.delay_task_recover[i].task_name, 256, task_para, 4, NULL);
	    }
	}

	obc_boot_count = obc_save.obc_boot_count;
	obc_reset_time = obc_save.obc_reset_time;
	antenna_status = obc_save.antenna_status;
	hk_down_cnt = obc_save.hk_down_cnt;
	hk_store_cnt = obc_save.hk_store_cnt;

	return res;
}
