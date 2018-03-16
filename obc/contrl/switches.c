/*
 * switches.c
 *
 *  Created on: 2016年5月10日
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bsp_delay.h"
#include "bsp_pca9665.h"
#include "bsp_switch.h"
#include "bsp_ants.h"
#include "bsp_ds1302.h"

#include "task.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "crc.h"
#include "error.h"
#include "driver_debug.h"
#include "stm32f4xx.h"
#include "hk_arg.h"

#include "ctrl_cmd_types.h"
#include "cubesat.h"
#include "switches.h"

#define ANTS_POWER_ENABLE		OUT_EPS_S3



int get_switch_status(uint8_t * pstatus)
{
	int result = 0;

//	memset(pstatus, 0, sizeof(obc_switch_t));

	/*W0B0--W0B4*/
	if(antenna_status == 2)
	{
		pstatus[0] |= ARM;
		pstatus[0] |= ANTS1;
		pstatus[0] |= ANTS2;
		pstatus[0] |= ANTS3;
		pstatus[0] |= ANTS4;
		result = 1;
	}
	else
	{   /*若天线加电，则获取遥测*/
	    if (OUT_ANTS_3V_PIN())
	        result = get_antenna_status(&pstatus[0]);   //ants[0-4] panel[5-6]
	}


	if(IN_SW_PAL_STATUS_1_PIN())
		pstatus[0] |= PANELA;
	else
		pstatus[0] &= ~PANELA;

	if(IN_SW_PAL_STATUS_2_PIN())
		pstatus[0] |= PANELB;
	else
		pstatus[0] &= ~PANELB;

    /*电池加热开关*/
    if(OUT_HEAT_5V_PIN())
        pstatus[0] |= BATTERY_HEAT_EN;
    else
        pstatus[0] &= ~BATTERY_HEAT_EN;



	/*W1*/

	/*姿控供电开关*/
	if(OUT_ADCS_7V_PIN())
		pstatus[1] |= ADCS_EN;
	else
	    pstatus[1] &= ~ADCS_EN;

	/*天线电源开关*/
	if(OUT_ANTS_3V_PIN())
		pstatus[1] |= ANTS_EN;
	else
	    pstatus[1] &= ~ANTS_EN;

	/*数传5V电开关*/
    if(OUT_SW_DTB_5V_PIN())
        pstatus[1] |= DTB_5V_EN;
    else
        pstatus[1] &= ~DTB_5V_EN;

    /*数传12V电开关*/
    if(OUT_SW_DTB_12V_PIN())
        pstatus[1] |= DTB_12V_EN;
    else
        pstatus[1] &= ~DTB_12V_EN;

    /*相机10W电开关*/
    if(OUT_SW_CAMERA_10W_PIN())
        pstatus[1] |= CAMERA_10W_5V_EN;
    else
        pstatus[1] &= ~CAMERA_10W_5V_EN;

    /*相机5W电开关*/
    if(OUT_SW_CAMERA_5W_PIN())
        pstatus[1] |= CAMERA_5W_5V_EN;
    else
        pstatus[1] &= ~CAMERA_5W_5V_EN;

    /*相机加热1开关*/
    if(OUT_SW_CAMERA_HEAT_1_PIN())
        pstatus[1] |= CAMERA_HEAT1_EN;
    else
        pstatus[1] &= ~CAMERA_HEAT1_EN;

    /*相机加热2开关*/
    if(OUT_SW_CAMERA_HEAT_2_PIN())
        pstatus[1] |= CAMERA_HEAT2_EN;
    else
        pstatus[1] &= ~CAMERA_HEAT2_EN;

    /*W2*/

    /*备份通信机供电开关*/
    if(OUT_BACKUP_VU_PIN())
        pstatus[2] |= JLG_VU_BUS_EN;
    else
        pstatus[2] &= ~JLG_VU_BUS_EN;

    /*通信机信道切换板*/
    if(OUT_SW_VU_5V_PIN())
        pstatus[2] |= SW_VU_5V_EN;
    else
        pstatus[2] &= ~SW_VU_5V_EN;

//    /*可展开帆展开状态*/
//    if (IN_SW_SAIL_STATUS_PIN())
//        pstatus[2] |= EXPANDABLE_SAIL;
//    else
//        pstatus[2] &= ~EXPANDABLE_SAIL;

	return result;
}



int open_antenna(void){

	isis_ants_status_t status;

	int result = ants_status(&status);

	if(result == 0){
		disable_antspwr(0,0);
		vTaskDelay(800 / portTICK_RATE_MS);
		enable_antspwr(0,0);
		vTaskDelay(800 / portTICK_RATE_MS);

		return 0;
	}

	if(ants_arm() == 0) return 0;

	if(ants_deploy_auto(20) == 0) return 0;

	return -1;
}

int enable_antspwr(uint32_t delay, uint32_t data __attribute__((unused))){
	int result = -1;

	if(delay > 0)
	{
		vTaskDelay(delay);
	}

	EpsOutSwitch(ANTS_POWER_ENABLE, ENABLE);
	result = SW_EPS_S3_PIN();

	return result;
}

int disable_antspwr(uint32_t delay, uint32_t data __attribute__((unused))){
	int result = -1;

	if(delay > 0) {
		vTaskDelay(delay);
	}

	EpsOutSwitch(ANTS_POWER_ENABLE, DISABLE);
	result = !SW_EPS_S3_PIN();

	return result;
}

uint8_t get_antenna_status_nopara(void){

	isis_ants_status_t  status;

	int result = ants_status(&status);

	if(result == 0){
		disable_antspwr(0,0);
		vTaskDelay(800 / portTICK_RATE_MS);
		enable_antspwr(0,0);
		vTaskDelay(800 / portTICK_RATE_MS);

		return 0;
	}

	if((!status.ant[0].not_deployed) && (!status.ant[1].not_deployed) && (!status.ant[2].not_deployed) && (!status.ant[3].not_deployed))
		return 2;

	if((!status.ant[1].not_deployed) || (!status.ant[3].not_deployed))
		return 1;

	return 0;
}

uint8_t get_antenna_status(uint8_t * sta){

	isis_ants_status_t  status;

	int result = ants_status(&status);

	if(result == 0)
	{
		return 0;
	}

	if(status.armed){
		*sta |= ARM;
	}else{
		*sta &= ~ARM;
	}
	if(!status.ant[0].not_deployed){
		*sta |= ANTS1;
	}else{
		*sta &= ~ANTS1;
	}
	if(!status.ant[1].not_deployed){
		*sta |= ANTS2;
	}else{
		*sta &= ~ANTS2;
	}
	if(!status.ant[2].not_deployed){
		*sta |= ANTS3;
	}else{
		*sta &= ~ANTS3;
	}
	if(!status.ant[3].not_deployed){
		*sta |= ANTS4;
	}else{
		*sta &= ~ANTS4;
	}

	return 1;
}

/**
 * 可展开电池阵展开系统使能
 *
 * @param delay 展开系统烧线使能时间（单位：毫秒）
 * @return 返回E_NO_ERR(-1)为成功
 */
int enable_panel(uint32_t delay)
{
	int result = E_TIMEOUT;

	if (delay > 10000)
	{
	    driver_debug(DEBUG_WARN, "WARN: Solar Burn Invalid Parameter.\r\n");
	    return E_INVALID_PARAM;
	}

	if ( EpsOutSwitch(OUT_SOLAR_EN, ENABLE) != EPS_ERROR)
	{
	    driver_debug(DEBUG_INFO, "INFO: Solar Burn Enable.\r\n");

	    vTaskDelay( delay / portTICK_RATE_MS );

        do
        {
            EpsOutSwitch(OUT_SOLAR_EN, DISABLE);
            driver_debug(DEBUG_INFO, "INFO: Solar Burn Disable.\r\n");
            vTaskDelay( 50 / portTICK_RATE_MS );

        } while( SW_SOLAR_EN_PIN() );

        result = E_NO_ERR;
	}
	else
	{
	    driver_debug(DEBUG_ERROR, "ERROR: Solar Burn Enable fail!!\r\n");
	    result = E_NO_SS;
	}

	return result;
}

/**
 * 配有检测开关的电池阵展开函数
 *
 * @param SafeTime_s 烧线的最长时间 （单位：秒）
 * @return 返回E_NO_ERR（-1）为正常
 */
int Solar_Array_Unfold(uint8_t SafeTime_s)
{
    /*如果检测开关显示帆板已经展开，则返回正常*/
    if( IN_SW_PAL_STATUS_1_PIN() && IN_SW_PAL_STATUS_2_PIN() )
        return E_NO_ERR;

    if( EpsOutSwitch(OUT_SOLAR_EN, ENABLE) != EPS_OK )
        return E_NO_SS;

    driver_debug(DEBUG_INFO, "INFO: Solar Burn Enable.\r\n");

    do
    {
        if(!SafeTime_s)
            break;
        vTaskDelay(1000/portTICK_RATE_MS);
        SafeTime_s--;
    }   /*帆板两个中只要有其中一个没有展开都会继续烧线*/
    while( IN_SW_PAL_STATUS_1_PIN() != SET || IN_SW_PAL_STATUS_2_PIN() != SET );

    do
    {
        EpsOutSwitch(OUT_SOLAR_EN, DISABLE);
        driver_debug(DEBUG_INFO, "INFO: Solar Burn Disable.\r\n");
        vTaskDelay( 50 / portTICK_RATE_MS );

    } while( SW_SOLAR_EN_PIN() );

    return E_NO_ERR;
}


/**
 * 离轨帆展开任务
 */
static void Sail_Unfold_task(void *para)
{

    uint8_t times = 20;

    EpsOutSwitch(OUT_SAIL_7V, ENABLE);

    vTaskDelay(6000);

    EpsOutSwitch(OUT_SAIL_7V, DISABLE);

    /*如果检测开关显示帆已经展开, 则删除任务*/
    if( IN_SW_SAIL_STATUS_PIN() )
        vTaskDelete(NULL);

    while( EpsOutSwitch(OUT_SAIL_7V, ENABLE) == EPS_ERROR )
        vTaskDelay(300);


    driver_debug(DEBUG_INFO, "INFO: Sail Burn Enable.\r\n");

    do
    {
        vTaskDelay(1000/portTICK_RATE_MS);
        times--;
    }
    while( IN_SW_SAIL_STATUS_PIN() != SET || times);

    do
    {
        EpsOutSwitch(OUT_SAIL_7V, DISABLE);
        driver_debug(DEBUG_INFO, "INFO: Sail Burn Disable.\r\n");
        vTaskDelay( 50 / portTICK_RATE_MS );

    } while( OUT_SAIL_7V_PIN() );

    vTaskDelete(NULL);
}

/**
 * 离轨帆展开
 *
 * @return 返回E_NO_ERR（-1）为正常
 */
int Sail_Unfold(void)
{
    if( xTaskCreate(Sail_Unfold_task, "SAIL", 128, NULL, 4, NULL) != pdTRUE)
        return E_NO_DEVICE;
    else
        return E_NO_ERR;
}




int disable_panel(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;

	if(delay > 0) {
		vTaskDelay(delay);
	}

	EpsOutSwitch(OUT_SOLAR_EN, DISABLE);
	result = !SW_SOLAR_EN_PIN();

	return result;
}

int enable_unfold_panel(uint32_t delay,uint16_t time)
{
    int result = 0;

    if(delay > 0) {
        vTaskDelay(delay);
    }

    EpsOutSwitch(OUT_7_4V_2, ENABLE);
    result = SW_7_4V_2_PIN();

    vTaskDelay(time);

    EpsOutSwitch(OUT_7_4V_2, DISABLE);
    EpsOutSwitch(OUT_7_4V_2, DISABLE);

    return result;
}


int disable_unfold_panel(uint32_t delay)
{
    int result = 0;

    if(delay > 0) {
        vTaskDelay(delay);
    }

    EpsOutSwitch(OUT_7_4V_2, DISABLE);
    result = !SW_7_4V_2_PIN();

    return result;
}



int obc_closeall(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;

	if(delay > 0) {
		vTaskDelay(delay);
	}

	DISABLE_ADCS_7V;
	DISABLE_ANTS_3V;
	DISABLE_PANEL_7V;
    SW_DTB_5V_DISABLE;
    SW_DTB_12V_DISABLE;
    SW_CAMERA_10W_DISABLE;
    SW_CAMERA_5W_DISABLE;
    SW_CAMERA_HEAT_1_DISABLE;
//    SW_CAMERA_HEAT_2_DISABLE;
	return result;
}
