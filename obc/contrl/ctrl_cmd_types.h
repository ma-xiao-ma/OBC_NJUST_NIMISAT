/*
 * ctrl_cmd_types.h
 *
 *  Created on: 2016年6月4日
 *      Author: Administrator
 */

#ifndef CONTRL_CTRL_TYPES_H_
#define CONTRL_CTRL_TYPES_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "camera_805.h"
#include "dtb_805.h"
#include "contrl.h"
#include "switches.h"


enum BOOL {INVALID=0,VALID=1};




typedef struct __attribute__((packed)) {
	uint8_t		head[2];
	uint8_t		id;
	uint8_t		cmd;
	uint8_t		result;  //0:error 1:ture
}cmd_ack_t;

typedef struct __attribute__((packed)) {

	uint32_t	    delay;
}ctrl_nopara_t;

typedef struct __attribute__((packed)) {
	uint8_t 	    id;
	uint8_t		cmd;
	uint32_t	    delay;
	uint8_t     	pass;
}ctrl_pass_t;

typedef struct __attribute__((packed)) {
	ctrl_nopara_t	cmd;
	uint32_t		crc;
}crc_nopara_t;

typedef struct __attribute__((packed)) {
//	uint8_t 	    id;
//	uint8_t		    cmd;
//	uint32_t	    delay;
	uint32_t    	msecs;
}ctrl_syntime_t;

typedef struct __attribute__((packed)) {
	uint8_t 	id;
	uint8_t		cmd;
	uint32_t	delay;
	uint32_t	msecs;
}ctrl_downtime_t;

typedef struct __attribute__((packed)) {

	uint8_t	select;		//0:SDCARD 1:RAM
	uint8_t	index;		//used for RAM
	uint32_t	secs;		//used for SDCARD
}ctrl_delayhk_t;

typedef struct __attribute__((packed)) {
	ctrl_syntime_t 	cmd;
	uint32_t 		crc;
}crc_syntime_t;

typedef struct __attribute__((packed)) {
	uint8_t 	id;
	uint8_t		cmd;
	uint32_t	delay;
	uint32_t	para;
}ctrl_motor_t;

typedef struct __attribute__((packed)) {
	ctrl_motor_t	cmd;
	uint32_t		crc;
}crc_motor_t;

typedef struct __attribute__((packed)) {

	char 			command[0];								/**< Zero-terminated string, max size = RSH_MAX_COMMAND_LENGTH */
} rsh_command_t;

typedef struct __attribute__((packed)) {
	uint8_t 	id;
	uint8_t		cmd;
	uint32_t	delay;
	uint32_t 	upAdcsConPDZ;
}ctrl_pdz_t;


typedef struct __attribute__((packed)) {
    uint32_t    img_id;
    uint16_t    packet_id;
} img_down_para;

typedef union __attribute__((packed))
{
    uint32_t        exp_time;
    uint8_t         gain;
    cam_ctl_t       cam_ctl_mode;
    uint32_t        img_id;
    img_down_para   img_down;
} cam_cmd_t;


typedef struct __attribute__((packed)) {
    uint8_t     store_period;
    uint8_t     img_id;
    uint8_t     source;
} enlaiimgdata_down_para;

typedef struct __attribute__((packed)) {
    uint8_t     store_period;
    uint8_t     img_id;
    uint16_t    packet_id;
    uint8_t     source;
} enlaiimgpack_down_para;

typedef union __attribute__((packed))
{
    uint8_t                 img_size;
    uint16_t                img_store_period;
    enlaiimgdata_down_para  info_data_down;
    enlaiimgpack_down_para  pack_down;
} enlaicam_cmd_t;



typedef struct __attribute__((packed)) {
	double upXwAdcsTLEBstar;
	double upXwAdcsTLEEcco;
	double upXwAdcsTLEInclo;
	double upXwAdcsTLEArgpo;
	double upXwAdcsTLEJdsatepoch;
	double upXwAdcsTLEMo;
	double upXwAdcsTLENo;
	double upXwAdcsTLENodeo;
}ctrl_adcs_t;

typedef struct __attribute__((packed)) {
	uint32_t 	adcstime;
//	gps_t 		gps;
	uint8_t 	mode;
}ctrl_adcstime_t;

typedef struct __attribute__((packed)) {
	double upXwAdcsTLEBstar;
	double upXwAdcsTLEEcco;
	double upXwAdcsTLEInclo;
	double upXwAdcsTLEArgpo;
	double upXwAdcsTLEJdsatepoch;
	double upXwAdcsTLEMo;
	double upXwAdcsTLENo;
	double upXwAdcsTLENodeo;

	int cntDmpFlag; /* 阻尼次数 */
	int cntPitcomFlag; /* 俯仰滤波次数 */
	int cntAttStaFlag; /* 三轴稳定控制次数 */

	enum BOOL updateTimeFlag; /* 授时标志位 */

	enum BOOL upXwAdcsReDmp; /* 重新阻尼标志，姿控清零 */
	enum BOOL upXwAdcsDmpForever; /* 永久阻尼标志 */

	enum BOOL AdcsOrbFlg;    /* 轨道有效标志位 */
	enum BOOL upAdcsTLEFlag; /* TLE轨道有效标志位 */

	enum BOOL magDotDmpFlg; /* 阻尼标志位 */
	enum BOOL pitFltComFlg; /* 俯仰滤波标志位 */
	enum BOOL attStaFlg; /* 三轴稳定控制标志位 */

	uint8_t pinstat3; /* 执行部件开关状态 */

	uint8_t  rst_mode;
	uint32_t rst_cnt;
	uint32_t rst_time;

	uint32_t lost_pwr; /* 判断是否是掉电复位 */
}adcs_argvs_t;


typedef struct __attribute__((packed)) {
	uint8_t 	id;
	uint8_t		cmd;
	uint32_t	delay;
}adcs_hkcmd_t;

typedef struct __attribute__((packed))
{
    uint8_t mem_num;
    uint8_t data_rate;
}mem_back_bash;

typedef struct __attribute__((packed))
{
    uint32_t    exp_time;
    uint8_t     gain;
    uint8_t     need_erase;
}cam_mode_bash;

typedef struct __attribute__((packed))
{
    uint32_t    id;
    uint8_t     mem_region;
    uint8_t     down_cnt;
}img_down_bash_1;


typedef struct __attribute__((packed))
{
    uint32_t    id;
    uint8_t     mem_region;
}img_down_bash_2;

typedef struct __attribute__((packed))
{
    uint32_t    id;
    uint16_t    start_packet;
    uint8_t     mem_region;
}img_down_bash_3;

typedef struct __attribute__((packed))
{
    uint32_t    id;
    uint16_t    packet;
    uint8_t     mem_region;
    uint8_t     down_cnt;
}img_down_bash_4;

typedef union __attribute__((packed))
{
    ctrl_delayhk_t  earlier_hk;
    uint32_t        time_sysn_para;
    delay_task_t    delay_task;
    char            command[0];

    switch_state    sw_status;
    mem_region      tr_mem_select;
    data_rate       tr_rate_select;
    mem_back_bash   tr_mem_back_bash;

    uint8_t         cam_gain;
    uint32_t        cam_exp_time;
    cam_mode_bash   cam_mode_set;
    img_down_bash_1 img_info_down;
    img_down_bash_2 img_data_down;
    img_down_bash_3 img_data_part;
    img_down_bash_4 img_pack_down;
} unpacket_t;

#endif /* CONTRL_CTRL_TYPES_H_ */
