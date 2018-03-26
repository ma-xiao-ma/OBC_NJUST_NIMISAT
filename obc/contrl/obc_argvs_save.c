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
extern uint16_t vu_isis_rx_count; //ISISvu 通信机接收上行遥控帧计数
extern uint16_t vu_jlg_rx_count;  //JLGvu 通信机接收上行遥控帧计数

obc_save_t obc_save = {0};

uint8_t obc_argvs_store(void)
{
	uint8_t res = 1;

    /*若读内部flash失败*/
    /*if( bsp_ReadCpuFlash(OBC_STORE_FLASH_ADDR, (uint8_t*)&obc_save, sizeof(obc_save)) != 0 )
        return 1;*/

	FSMC_NOR_ReadBuffer( (uint16_t *)&obc_save, OBC_STORE_NOR_ADDR, sizeof(obc_save_t) / 2 );

	obc_save.obc_reset_time = clock_get_time_nopara();
	obc_save.antenna_status = antenna_status;
	obc_save.hk_down_cnt = hk_down_cnt;
	obc_save.hk_store_cnt = hk_store_cnt;
	obc_save.vu_rec_cnt = vu_isis_rx_count;
	obc_save.backup_vu_rec_cnt = vu_jlg_rx_count;

    for (int i = 0; i < DELAY_TASK_NUM; i++)
    {
        delay_task_t *task_para = (delay_task_t *)obc_save.delay_task_recover[i].task_para;

        /*如果执行时间已经早于当前时间*/
        if (task_para->execution_utc < clock_get_time_nopara() && task_para->execution_utc != 0)
            task_para->execution_utc = 0;
    }

    if (USER_NOR_SectorErase(0) != NOR_SUCCESS)
    {
        printf("ERROR: NorFalsh erase sector 0 fail!\r\n");
        return 1;
    }

    if (FSMC_NOR_WriteBuffer((uint16_t *)&obc_save, OBC_STORE_NOR_ADDR, sizeof(obc_save_t) / 2) != NOR_SUCCESS)
    {
        printf("ERROR: NorFalsh write sector 0 fail!\r\n");
        return 1;
    }
//	res = bsp_WriteCpuFlash(OBC_STORE_FLASH_ADDR, (uint8_t*)&obc_save, sizeof(obc_save));
	return 0;
}

uint8_t obc_argvs_recover(void)
{
	uint8_t res = 1;

	FSMC_NOR_ReadBuffer( (uint16_t *)&obc_save, OBC_STORE_NOR_ADDR, sizeof(obc_save_t) / 2 );

    if(obc_save.obc_boot_count == 0xFFFFFFFF || obc_save.obc_boot_count == 0)
        obc_save.obc_boot_count = 1;
    else
        obc_save.obc_boot_count = obc_save.obc_boot_count + 1;

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

    if(obc_save.vu_rec_cnt == 0xFFFFFFFF)
    {
        obc_save.vu_rec_cnt = 0;
    }

    if(obc_save.backup_vu_rec_cnt == 0xFFFFFFFF)
    {
        obc_save.backup_vu_rec_cnt = 0;
    }

	/* 延时任务恢复 */
	for (int i = 0; i < DELAY_TASK_NUM; i++)
	{
	    delay_task_t *task_para = (delay_task_t *)obc_save.delay_task_recover[i].task_para;

	    /* 延时任务执行时间参数是否有效 */
	    if (task_para->execution_utc > clock_get_time_nopara() &&
	            task_para->execution_utc != 0xFFFFFFFF && task_para->execution_utc != 0)
	    {
	        /* 判断是否为有效的函数指针 */
	        if( (int)obc_save.delay_task_recover[i].task_function > 0x20000000 &&
	                (int)obc_save.delay_task_recover[i].task_function < 0x20000000 + 128 * 1024 )
	        {
                xTaskCreate(obc_save.delay_task_recover[i].task_function,
                        obc_save.delay_task_recover[i].task_name, 256, task_para, 4, NULL);
	        }
	    }
	}

    if (USER_NOR_SectorErase(0) == NOR_SUCCESS)
        res = FSMC_NOR_WriteBuffer( (uint16_t *)&obc_save, OBC_STORE_NOR_ADDR, sizeof(obc_save_t) / 2 );

	obc_boot_count = obc_save.obc_boot_count;
	obc_reset_time = obc_save.obc_reset_time;
	antenna_status = obc_save.antenna_status;
	hk_down_cnt = obc_save.hk_down_cnt;
	hk_store_cnt = obc_save.hk_store_cnt;
    vu_isis_rx_count = obc_save.vu_rec_cnt;
    vu_jlg_rx_count = obc_save.backup_vu_rec_cnt;

	return res;
}
