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
#include "driver_debug.h"
#include "stm32f4xx.h"
#include "hk_arg.h"

#include "ctrl_cmd_types.h"
#include "cubesat.h"
#include "switches.h"

#define ANTS_POWER_ENABLE		OUT_EPS_S3

extern uint8_t mode;


//int get_adcs_hk(adcs_hk_t * phk) {
//	int result = 0;
//	ctrl_nopara_t 	cmd;
//
//	cmd.id 		= 2;
//	cmd.delay 	= 0;
//	cmd.cmd 	= 0xFF;
//
//	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &cmd, sizeof(ctrl_nopara_t), phk, sizeof(adcs_hk_t), ADCS_DELAY);
//
//	return result;
//}


int get_status_from_adcs(uint16_t * pstatus) {
	int result = 0;
	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= 0;
	cmd.cmd 	= 0xFE;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), pstatus, 2, ADCS_DELAY);

	return result;
}

int get_switch_status(uint8_t * pstatus) {
	int result = 0;

	*pstatus = 0;
	if(antenna_status == 2)
	{
		pstatus[0] |= ARM;
		pstatus[0] |= ANTS1;
		pstatus[0] |= ANTS2;
		pstatus[0] |= ANTS3;
		pstatus[0] |= ANTS4;
	}
	else
	{
		get_antenna_status(&pstatus[0]);   //ants[0-4] panel[5-6]
	}

	pstatus[0] = pstatus[0] & ANTSMSK;
	if(IN_SW_PAL_STATUS_1_PIN())
		pstatus[0] |= PANELA;
	else
		pstatus[0] &= ~PANELA;

	if(IN_SW_PAL_STATUS_2_PIN())
		pstatus[0] |= PANELB;
	else
		pstatus[0] &= ~PANELB;

	pstatus[1] = 0;

	/*姿控电源开关*/
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
    if(OUT_SW_DIGI_TRAN_5V_PIN())
        pstatus[1] |= DIGI_TRAN_5V_EN;
    else
        pstatus[1] &= ~DIGI_TRAN_5V_EN;

    /*数传12V电开关*/
    if(OUT_SW_DIGI_TRAN_12V_PIN())
        pstatus[1] |= DIGI_TRAN_12V_EN;
    else
        pstatus[1] &= ~DIGI_TRAN_12V_EN;

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


//	result = get_status_from_adcs((uint16_t *)&pstatus[2]);     //不用了
//	pstatus[3] |= ((uint8_t)antenna_status)<<6;

	return result;
}

int switch_off_all(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x3F;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);

	return result;
}

int adcs_reset(uint32_t delay, uint32_t data __attribute__((unused))) {

	int result = 0;
	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x08;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);

	return result;
}

int enable_gr(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x24;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);

	return result;
}

int disable_gr(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x25;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);

	return result;
}

int enable_adcs(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x24;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);

	return result;
}

int disable_adcs(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x25;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);

	return result;
}

//int enable_gpsa(uint32_t delay, uint32_t data __attribute__((unused)))
//{
//	int result = 0;
//	ctrl_nopara_t 	cmd;
//	crc_nopara_t	crc;
//
//	cmd.id 		= 2;
//	cmd.delay 	= delay;
//	cmd.cmd 	= 0x12;
//
//	crc.cmd = cmd;
//	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));
//
//	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);
//
//	return result;
//}
//
//int enable_gpsb(uint32_t delay, uint32_t data __attribute__((unused)))
//{
//	int result = 0;
//
//	ctrl_nopara_t 	cmd;
//	crc_nopara_t	crc;
//
//	cmd.id 		= 2;
//	cmd.delay 	= delay;
//	cmd.cmd 	= 0x14;
//
//	crc.cmd = cmd;
//	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));
//
//	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);
//
//	return result;
//}
//
//int enable_gps(uint32_t delay, uint32_t data __attribute__((unused)))
//{
//	int result = 0;
//
//	if(delay > 0) {
//		vTaskDelay(delay);
//	}
//	gps_status = 0;
//	EpsOutSwitch(OUT_5_3_3V_1, ENABLE);
//	result = SW_5_3_3V_1_PIN();
//
//	return result;
//}
//
//int disable_gps(uint32_t delay, uint32_t data __attribute__((unused)))
//{
//	int result = 0;
//
//	if(delay > 0) {
//		vTaskDelay(delay);
//	}
//	gps_status = 1;
//	EpsOutSwitch(OUT_5_3_3V_1, DISABLE);
//	result = !SW_5_3_3V_1_PIN();
//
//	return result;
//}

int disable_gpsa(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x13;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);

	return result;
}

int disable_gpsb(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;

	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x14;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);

	return result;
}

int enable_hmra(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x34;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);

	return result;
}

int enable_hmrb(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x35;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);

	return result;
}

int disable_hmra(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x38;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);

	return result;
}

int disable_hmrb(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_nopara_t 	cmd;
	crc_nopara_t	crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x39;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_nopara_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_nopara_t), NULL, 0, ADCS_DELAY);

	return result;
}

int enable_momentum1(uint32_t delay, uint32_t data)
{
	int result = 0;
	ctrl_motor_t 	cmd;
	crc_motor_t		crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.para	= data;
	cmd.cmd 	= 0x18;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_motor_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_motor_t), NULL, 0, ADCS_DELAY);

	return result;
}

int enable_momentum2(uint32_t delay, uint32_t data)
{
	int result = 0;
	ctrl_motor_t 	cmd;
	crc_motor_t		crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.para	= data;
	cmd.cmd 	= 0x1A;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_motor_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_motor_t), NULL, 0, ADCS_DELAY);

	return result;
}

int enable_momentum3(uint32_t delay, uint32_t data)
{
	int result = 0;
	ctrl_motor_t 	cmd;
	crc_motor_t		crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.para	= data;
	cmd.cmd 	= 0x20;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_motor_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_motor_t), NULL, 0, ADCS_DELAY);

	return result;
}

int enable_momentum4(uint32_t delay, uint32_t data)
{
	int result = 0;
	ctrl_motor_t 	cmd;
	crc_motor_t		crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.para	= data;
	cmd.cmd 	= 0x26;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_motor_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_motor_t), NULL, 0, ADCS_DELAY);

	return result;
}

int disable_momentum1(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_motor_t 	cmd;
	crc_motor_t		crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x19;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_motor_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_motor_t), NULL, 0, ADCS_DELAY);

	return result;
}

int disable_momentum2(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_motor_t 	cmd;
	crc_motor_t		crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x1B;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_motor_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_motor_t), NULL, 0, ADCS_DELAY);

	return result;
}

int disable_momentum3(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_motor_t 	cmd;
	crc_motor_t		crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x21;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_motor_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_motor_t), NULL, 0, ADCS_DELAY);

	return result;
}

int disable_momentum4(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;
	ctrl_motor_t 	cmd;
	crc_motor_t		crc;

	cmd.id 		= 2;
	cmd.delay 	= delay;
	cmd.cmd 	= 0x27;

	crc.cmd = cmd;
	crc.crc = crc32_memory((uint8_t *)&cmd, sizeof(ctrl_motor_t));

	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &crc, sizeof(crc_motor_t), NULL, 0, ADCS_DELAY);

	return result;
}

int send_mode(void)
{
    routing_packet_t *ModeSend = (routing_packet_t *)qb50Malloc((size_t)(sizeof(routing_packet_t) + 1));

    ModeSend->len = 1;
    ModeSend->dst = 2;
    ModeSend->src = 1;
    ModeSend->typ = 0x01;
    ModeSend->dat[0] = mode;


    route_queue_wirte(ModeSend, NULL);

    return 0;
}

//int send_gps(uint32_t delay __attribute__((unused)), uint32_t data __attribute__((unused)))
//{
//	int 			result = 0;
//	ctrl_gps_t 		cmd;
//
//	cmd.id 		= 2;
//	cmd.delay 	= 0;
//	cmd.cmd 	= 0x1F;
//
//	memcpy((uint8_t *)&(cmd.gps), (uint8_t *)(((uint8_t *)&gps_info) + 2), sizeof(gps_t));
//
//	result = i2c_master_transaction(OBC_TO_ADCS_HANDLE, ADCS_ADDR, &cmd, sizeof(ctrl_gps_t), NULL, 0, 200);
//
//	return result;
//}

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

	if(ants_deploy_auto(10) == 0) return 0;

	return -1;
}

int enable_antspwr(uint32_t delay, uint32_t data __attribute__((unused))){
	int result = -1;

	if(delay > 0) {
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

	if(result == 0){
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

uint8_t get_panel_status(void){
	uint8_t result = 0;

	if(PANELA_PIN_STATUS())
		result++;
	if(PANELB_PIN_STATUS())
		result++;

	return result;
}

int enable_panel(uint32_t delay, uint32_t data __attribute__((unused))){
	int result = 0;

	if(delay > 0) {
		vTaskDelay(delay);
	}

	EpsOutSwitch(OUT_SOLAR_EN, ENABLE);
	result = SW_SOLAR_EN_PIN();

	vTaskDelay(2500);

	EpsOutSwitch(OUT_SOLAR_EN, DISABLE);
	EpsOutSwitch(OUT_SOLAR_EN, DISABLE);

	return result;
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

int obc_closeall(uint32_t delay, uint32_t data __attribute__((unused)))
{
	int result = 0;

	if(delay > 0) {
		vTaskDelay(delay);
	}

	DISABLE_ADCS_7V;
	DISABLE_ANTS_3V;
	DISABLE_PANEL_7V;
    SW_DIGITAL_TRAN_5V_DISABLE;
    SW_DIGITAL_TRAN_12V_DISABLE;
    SW_CAMERA_10W_DISABLE;
    SW_CAMERA_5W_DISABLE;
    SW_CAMERA_HEAT_1_DISABLE;
    SW_CAMERA_HEAT_2_DISABLE;
	return result;
}
