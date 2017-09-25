/*
 * bsp_cis.h
 *
 *  Created on: 2016年10月25日
 *      Author: Administrator
 */

#ifndef BSP_BSP_USER_INC_BSP_CIS_H_
#define BSP_BSP_USER_INC_BSP_CIS_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "stm32f4xx.h"

#define CSP_HANDLE		0
#define CIS_ADDR		28
#define CIS_LENGTH		96
#define CIS_DELAY		1000

typedef union {
	uint32_t head;
	struct __attribute__((__packed__)) {
		unsigned int flags : 8;
		unsigned int sport : 6;
		unsigned int dport : 6;
		unsigned int dst : 5;
		unsigned int src : 5;
		unsigned int pri : 2;
	};
} csp_head_t;

uint16_t hton16(uint16_t h16);

uint16_t ntoh16(uint16_t n16);

uint32_t hton32(uint32_t h32);

uint32_t ntoh32(uint32_t n32) ;

typedef struct __attribute__((__packed__)) {
	csp_head_t id;
	uint8_t buf[96];
}cis_frame_t;

typedef struct __attribute__((__packed__)) {
	csp_head_t id;
	char cmd[];
}cis_cmd_t;

int sendtocis(void *str, int length, uint32_t delay, uint8_t *cis_ret);
int FlanshAudioFilesToCis(void *data, uint8_t length, uint32_t delay);

uint8_t leop_status(uint8_t on_off);
uint8_t tran_status(uint8_t on_off);
uint8_t cis_rst(void);
uint8_t def_para(void);
uint8_t ccsds_on(uint16_t seconds);
uint8_t get_jpg (void);
uint8_t download_jpg (void);
uint8_t download_para (void);
uint8_t tx_gain (int16_t gain);
uint8_t dpacket_delay (uint32_t ticks);
uint8_t read_fram (uint32_t addr, uint8_t len);
uint8_t ccsds_off (void);



#endif /* BSP_BSP_USER_INC_BSP_CIS_H_ */
