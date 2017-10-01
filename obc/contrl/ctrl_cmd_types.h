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
	uint8_t 	    id;
	uint8_t		cmd;
	uint32_t	    delay;
	uint32_t    	msecs;
}ctrl_syntime_t;

typedef struct __attribute__((packed)) {
	uint8_t 	id;
	uint8_t		cmd;
	uint32_t	delay;
	uint32_t	msecs;
}ctrl_downtime_t;

typedef struct __attribute__((packed)) {

	uint16_t	select;		//0:SDCARD 1:RAM
	uint16_t	index;		//used for RAM
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

typedef struct __attribute__((packed))
{
    ctrl_motor_t    cmd;
    union
    {
        uint32_t    CamPara;
        struct __attribute__((__packed__))
        {
            uint8_t /*reserve*/;
            uint8_t TransMode;
            uint8_t WorkMode;
            uint8_t AutoExpo;
        };
    };
    uint16_t    ImageId;
    uint16_t    PacketId;
}cam_cmd_t;


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

//typedef struct __attribute__((packed)) {
//
//	uint8_t 	error;
//}adcs_hk_t;

typedef struct __attribute__((packed)) {
	uint8_t 	id;
	uint8_t		cmd;
	uint32_t	delay;
}adcs_hkcmd_t;

typedef struct __attribute__((packed)) {
	uint8_t header[2];
	union {
		uint16_t info;
		struct __attribute__((__packed__)) {
			uint16_t id    : 12; //must be 0~345
			uint16_t index : 4;  //must be 0~345
			};
		  };
	uint8_t 	len; //must be 0~345
}cup_flash_cmd_t;

typedef struct __attribute__((packed)) {
	union {
		uint16_t info;
		struct __attribute__((__packed__)) {
			uint16_t id    : 12;  //must be 0~345
			uint16_t index : 4;	  //must be 0~345
			};
		  };
	uint8_t 	len;  //must be 0~345
	uint8_t		data[91];
}cup_flash_content_t;



#endif /* CONTRL_CTRL_TYPES_H_ */
