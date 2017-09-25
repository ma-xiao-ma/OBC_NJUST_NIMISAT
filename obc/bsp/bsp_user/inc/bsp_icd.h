/*
 * bsp_icd.c
 *
 *  Created on: 2016年4月28日
 *      Author: Administrator
 */
#ifndef __I2C_ICD_H
#define	__I2C_ICD_H

#include <stdint.h>
#include <inttypes.h>
#include <stddef.h>

#include "stm32f4xx.h"

#define ISIS_HANDLE		0
#define ISIS_TIMEOUT	1000

#define SCALL			"BI4ST-0"
#define DCALL			"NJUST-1"
#define ISIS_STAT_SET	0x08

#define ISIS_DELAY_TO_READ      1234

/*地址定义*/
#define	Receiver_Address		0x60
#define	Transmitter_Address		0x61

/* 命令字定义 */
#define Watchingdog_reset			0xCC
#define Software_reset				0xAA
#define Hardware_system_reset		0xAB

/* Receiver */
#define Get_frame_number			0x21
#define Get_frames					0x22
#define Remove_frames				0x24
#define Measure_states				0x1A
#define Report_Receiver_uptime		0x40

/* transmitter */
#define Send_frame					0x10
#define Send_frame_with_callsigns	0x11
#define Set_beacon					0x14
#define Set_beacon_with_callsigns	0x15
#define Clear_beacon				0x1F
#define Set_des_callsigns			0x22
#define Set_orig_callsigns			0x23
#define Set_transmitter_idle_state	0x24
#define Measure_all_states			0x25
#define Store_last_states			0x26
#define Set_transmission_bitrate	0x28
#define Report_transmitter_uptime	0x40
#define Report_transmitter_state	0x41

/*2016.5.10*/

/*            带呼号结构体定义                     */
typedef  struct __attribute__((packed)) {
	uint8_t	command;
	uint8_t	origcall[7];
	uint8_t	descall[7];
	uint8_t	date[0];

}Date_with_call;

/*            信标设置结构体定义                     */

typedef struct __attribute__((packed))  {
	uint8_t	command;
	uint8_t	beacon[2];
	uint8_t	date[0];

}Beacon_set;


/*            AX 2.5信标设置结构体定义                     */

typedef struct  __attribute__((packed)) {
	uint8_t	command;
	uint8_t	beacon[2];
	uint8_t	origcall[7];
	uint8_t	descall[7];
	uint8_t	date[0];

}AX_Beacon_set;

//end

/* 函数声明*/
int I2C_ICD_send_command( uint8_t command, uint8_t Address );

int I2C_ICD_read_frame(uint8_t* pBuffer);

int I2C_ICD_get_frame(uint8_t* pBuffer);
int I2C_ICD_get_frame_stable(uint8_t* pBuffer);

int I2C_ICD_send_para(uint8_t* pBuffer, uint8_t command , uint8_t NumByteToWrite, uint8_t Address );

int I2C_ICD_send_date(uint8_t* pBuffer, uint8_t NumByteToWrite,uint8_t* rest_frames);

int I2C_ICD_read_para(uint8_t* pBuffer, uint8_t command , uint8_t Address ,  uint16_t NumByteToRead );

int I2C_ICD_Hardware_system_reset();

int I2C_ICD_software_reset();

int I2C_ICD_Watchingdog_reset();

int I2C_ICD_Set_transmission_bitrate(uint8_t* pBuffer);
int set_transmission_bitrate_1200(void);
int set_transmission_bitrate_4800(void);

int I2C_ICD_beacon_Set(uint8_t* pBuffer,uint8_t NumByteToWrite,uint8_t command);

int I2C_ICD_Set_transmitter_idle_state(uint8_t* pBuffer);

int I2C_ICD_Set_orig_callsigns(uint8_t* pBuffer);

int I2C_ICD_Set_des_callsigns(uint8_t* pBuffer);

int  I2C_ICD_beacon_clear();

int  I2C_ICD_sweep_butter();

int  I2C_ICD_read_countofframe(uint8_t* pBuffer);

int  I2C_ICD_read_Receiver_all_states(uint8_t* pBuffer);

int  I2C_ICD_read_Receiver_systime(uint8_t* pBuffer);

int  I2C_ICD_read_Transmitter_all_states(uint8_t* pBuffer);

int  I2C_ICD_read_laststate(uint8_t* pBuffer);

int  I2C_ICD_read_Transmitter_systime(uint8_t* pBuffer);

int  I2C_ICD_read_workstate(uint8_t* pBuffer);

int I2C_ICD_send_Axdate(uint8_t * ogc, uint8_t * dec, uint8_t* pBuffer, uint8_t NumByteToWrite, uint8_t* rest_frames);
int ICD_send(uint8_t* pBuffer, uint8_t NumByteToWrite, uint8_t* rest_frames);

int I2C_ICD_beacon_set(uint8_t* beon,uint8_t* pBuffer, uint8_t NumByteToWrite);

int I2C_ICD_Axbeacon_set(uint8_t *ben, uint8_t * ogc, uint8_t * dec,uint8_t* pBuffer, uint8_t NumByteToWrite);

#endif
