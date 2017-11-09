/*
 * bsp_cis.c
 *
 *  Created on: 2016年10月25日
 *      Author: Administrator
 */

#include <string.h>

#include "bsp_cis.h"
#include "bsp_pca9665.h"

#include "obc_mem.h"


//uint16_t hton16(uint16_t h16) {
//	return (((h16 & 0xff00) >> 8) |
//			((h16 & 0x00ff) << 8));
//}

//uint16_t ntoh16(uint16_t n16) {
//	return hton16(n16);
//}

uint32_t hton32(uint32_t h32) {

	return (((h32 & 0xff000000) >> 24) |
			((h32 & 0x000000ff) << 24) |
			((h32 & 0x0000ff00) <<  8) |
			((h32 & 0x00ff0000) >>  8));
}

uint32_t ntoh32(uint32_t n32) {
	return hton32(n32);
}


int sendtocis(void *data, int length, uint32_t delay, uint8_t *cis_ret) {

	cis_frame_t *pbuf = NULL;

	pbuf = (cis_frame_t *)ObcMemMalloc(sizeof(cis_frame_t));

	if(pbuf == NULL) {
		ObcMemFree(pbuf);
		return *cis_ret;
	}

	pbuf->id.flags 	= 0;
	pbuf->id.sport 	= 0;
	pbuf->id.dport 	= 0;
	pbuf->id.dst	= 5;
	pbuf->id.src	= 26;
	pbuf->id.pri	= 0;

	memcpy(&(pbuf->buf[0]), data, length);
	pbuf->id.head = hton32(pbuf->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pbuf, sizeof(cis_frame_t), NULL, 0, delay);

	*cis_ret = 0;
	ObcMemFree(pbuf);

	return *cis_ret;
}

int FlanshAudioFilesToCis(void *data, uint8_t length, uint32_t delay) {

	cis_frame_t *pbuf = NULL;

	pbuf = (cis_frame_t *)ObcMemMalloc(sizeof(cis_frame_t));

	if(pbuf == NULL) {
		ObcMemFree(pbuf);
		return -1;
	}

	pbuf->id.flags 	= 0;
	pbuf->id.sport 	= 0;
	pbuf->id.dport 	= 0;
	pbuf->id.dst	= 5;
	pbuf->id.src	= 26;
	pbuf->id.pri	= 0;
	pbuf->buf[0]	= 0x1a;
	pbuf->buf[1]	= 0x56;

	memcpy(&(pbuf->buf[2]), data, length);
	pbuf->id.head = hton32(pbuf->id.head);
	int res = i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pbuf, length + 6, NULL, 0, delay);

	ObcMemFree(pbuf);

	return res;
}


   /*LEOP status on/off*/
uint8_t leop_status(uint8_t on_off) {
	cis_cmd_t *pcmd = malloc(sizeof(cis_cmd_t)+ 6 * sizeof(char));
	uint8_t cis_ret = 1;

	if(pcmd == NULL)
		return 1;

	pcmd->id.flags 	= 0;
	pcmd->id.sport 	= 1;
	pcmd->id.dport 	= 18;
	pcmd->id.dst	= 28;
	pcmd->id.src	= 26;
	pcmd->id.pri	= 0;

	pcmd->cmd[0] 	= 0x1c;
	pcmd->cmd[1] 	= 0xcc;
	pcmd->cmd[2] 	= 0x04;
	pcmd->cmd[3] 	= 0x0a;
	pcmd->cmd[4] 	= 0x01;
	if(on_off == 1)
		pcmd->cmd[5] 	= 0x5a;
	else
		pcmd->cmd[5] 	= 0x00;

	pcmd->id.head = hton32(pcmd->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pcmd, sizeof(cis_cmd_t)+ 6 * sizeof(char), NULL, 0, CIS_DELAY);

	free(pcmd);

	return cis_ret;
}


  /*communications transponder on/off*/
uint8_t tran_status(uint8_t on_off) {
	cis_cmd_t *pcmd = malloc(sizeof(cis_cmd_t)+ 6 * sizeof(char));
	uint8_t cis_ret = 1;

	if(pcmd == NULL)
		return 1;

	pcmd->id.flags 	= 0;
	pcmd->id.sport 	= 1;
	pcmd->id.dport 	= 18;
	pcmd->id.dst	= 28;
	pcmd->id.src	= 26;
	pcmd->id.pri	= 0;

	pcmd->cmd[0] 	= 0x1c;
	pcmd->cmd[1] 	= 0xcc;
	pcmd->cmd[2] 	= 0x50;
	pcmd->cmd[3] 	= 0x0a;
	pcmd->cmd[4] 	= 0x01;
	if(on_off == 1)
		pcmd->cmd[5] 	= 0x5a;
	else
		pcmd->cmd[5] 	= 0x00;

	pcmd->id.head = hton32(pcmd->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pcmd, sizeof(cis_cmd_t)+ 6 * sizeof(char), NULL, 0, CIS_DELAY);

	free(pcmd);

	return cis_ret;
}

    /*reset command*/
uint8_t cis_rst(void) {
	cis_cmd_t *pcmd = malloc(sizeof(cis_cmd_t)+ 5 * sizeof(char));
	uint8_t cis_ret = 1;

	if(pcmd == NULL)
		return 1;

	pcmd->id.flags 	= 0;
	pcmd->id.sport 	= 1;
	pcmd->id.dport 	= 18;
	pcmd->id.dst	= 28;
	pcmd->id.src	= 26;
	pcmd->id.pri	= 2;

	pcmd->cmd[0] 	= 0x1c;
	pcmd->cmd[1] 	= 0xcc;
	pcmd->cmd[2] 	= 0x00;
	pcmd->cmd[3] 	= 0x0a;
	pcmd->cmd[4] 	= 0x00;

	pcmd->id.head = hton32(pcmd->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pcmd, sizeof(cis_cmd_t)+ 5 * sizeof(char), NULL, 0, CIS_DELAY);

	free(pcmd);

	return cis_ret;
}

  /*restore the default parameters*/
uint8_t def_para(void) {
	cis_cmd_t *pcmd = malloc(sizeof(cis_cmd_t)+ 5 * sizeof(char));
	uint8_t cis_ret[5] = {1};

	if(pcmd == NULL)
		return 1;

	pcmd->id.flags 	= 0;
	pcmd->id.sport 	= 1;
	pcmd->id.dport 	= 18;
	pcmd->id.dst	= 28;
	pcmd->id.src	= 26;
	pcmd->id.pri	= 0;

	pcmd->cmd[0] 	= 0x1c;
	pcmd->cmd[1] 	= 0xcc;
	pcmd->cmd[2] 	= 0x03;
	pcmd->cmd[3] 	= 0x0a;
	pcmd->cmd[4] 	= 0x00;

	pcmd->id.head = hton32(pcmd->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pcmd, sizeof(cis_cmd_t)+ 5 * sizeof(char), NULL, 0, CIS_DELAY);

	free(pcmd);

	return cis_ret[0];
}


/*create JPG task // take a picture*/
uint8_t get_jpg (void) {
	cis_cmd_t *pcmd = malloc(sizeof(cis_cmd_t)+ 5 * sizeof(char));
	uint8_t cis_ret = 1;

	if(pcmd == NULL)
		return 1;

	pcmd->id.flags 	= 0;
	pcmd->id.sport 	= 1;
	pcmd->id.dport 	= 18;
	pcmd->id.dst	= 28;
	pcmd->id.src	= 26;
	pcmd->id.pri	= 0;

	pcmd->cmd[0] 	= 0x1c;
	pcmd->cmd[1] 	= 0xcc;
	pcmd->cmd[2] 	= 0x26;
	pcmd->cmd[3] 	= 0x0a;
	pcmd->cmd[4] 	= 0x00;

	pcmd->id.head = hton32(pcmd->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pcmd, sizeof(cis_cmd_t)+ 5 * sizeof(char), NULL, 0, CIS_DELAY);

	free(pcmd);

	return cis_ret;
}


/*Download JPG picture*/
uint8_t download_jpg (void) {
	cis_cmd_t *pcmd = malloc(sizeof(cis_cmd_t)+ 5 * sizeof(char));
	uint8_t cis_ret = 1;

	if(pcmd == NULL)
		return 1;

	pcmd->id.flags 	= 0;
	pcmd->id.sport 	= 1;
	pcmd->id.dport 	= 18;
	pcmd->id.dst	= 28;
	pcmd->id.src	= 26;
	pcmd->id.pri	= 0;

	pcmd->cmd[0] 	= 0x1c;
	pcmd->cmd[1] 	= 0xcc;
	pcmd->cmd[2] 	= 0x27;
	pcmd->cmd[3] 	= 0x0a;
	pcmd->cmd[4] 	= 0x00;

	pcmd->id.head = hton32(pcmd->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pcmd, sizeof(cis_cmd_t)+ 5 * sizeof(char), NULL, 0, CIS_DELAY);

	free(pcmd);

	return cis_ret;
}

/*Download the cis parameters*/
uint8_t download_para (void) {
	cis_cmd_t *pcmd = malloc(sizeof(cis_cmd_t)+ 5 * sizeof(char));
	uint8_t cis_ret = 1;

	if(pcmd == NULL)
		return 1;

	pcmd->id.flags 	= 0;
	pcmd->id.sport 	= 1;
	pcmd->id.dport 	= 18;
	pcmd->id.dst	= 28;
	pcmd->id.src	= 26;
	pcmd->id.pri	= 0;

	pcmd->cmd[0] 	= 0x1c;
	pcmd->cmd[1] 	= 0xcc;
	pcmd->cmd[2] 	= 0xfe;
	pcmd->cmd[3] 	= 0x0a;
	pcmd->cmd[4] 	= 0x00;

	pcmd->id.head = hton32(pcmd->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pcmd, sizeof(cis_cmd_t)+ 5 * sizeof(char), NULL, 0, CIS_DELAY);

	free(pcmd);

	return cis_ret;
}


/*CCSDS continuous emission [on] (interval [seconds])*/
uint8_t ccsds_on (uint16_t seconds) {
	cis_cmd_t *pcmd = malloc(sizeof(cis_cmd_t)+ 7 * sizeof(char));
	uint8_t cis_ret = 1;

	if(pcmd == NULL)
		return 1;

	pcmd->id.flags 	= 0;
	pcmd->id.sport 	= 1;
	pcmd->id.dport 	= 18;
	pcmd->id.dst	= 28;
	pcmd->id.src	= 26;
	pcmd->id.pri	= 0;

	pcmd->cmd[0] 	= 0x1c;
	pcmd->cmd[1] 	= 0xcc;
	pcmd->cmd[2] 	= 0x12;
	pcmd->cmd[3] 	= 0x0a;
	pcmd->cmd[4] 	= 0x02;
	pcmd->cmd[5] 	= (uint8_t)(seconds);
	pcmd->cmd[6] 	= (uint8_t)(seconds >> 8);

	pcmd->id.head = hton32(pcmd->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pcmd, sizeof(cis_cmd_t)+ 7 * sizeof(char), NULL, 0, CIS_DELAY);

	free(pcmd);

	return cis_ret;
}


 /*Set the tx gain*/
uint8_t tx_gain (int16_t gain) {
	cis_cmd_t *pcmd = malloc(sizeof(cis_cmd_t)+ 7 * sizeof(char));
	uint8_t cis_ret = 1;

	if(pcmd == NULL)
		return 1;

	pcmd->id.flags 	= 0;
	pcmd->id.sport 	= 1;
	pcmd->id.dport 	= 18;
	pcmd->id.dst	= 28;
	pcmd->id.src	= 26;
	pcmd->id.pri	= 0;

	pcmd->cmd[0] 	= 0x1c;
	pcmd->cmd[1] 	= 0xcc;
	pcmd->cmd[2] 	= 0x30;
	pcmd->cmd[3] 	= 0x0a;
	pcmd->cmd[4] 	= 0x02;
	pcmd->cmd[5] 	= (char)(gain);
	pcmd->cmd[6] 	= (char)(gain >> 8);

	pcmd->id.head = hton32(pcmd->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pcmd, sizeof(cis_cmd_t)+ 7 * sizeof(char), NULL, 0, CIS_DELAY);

	free(pcmd);

	return cis_ret;
}


/*downlink data packet delay */
uint8_t dpacket_delay (uint32_t ticks) {
	cis_cmd_t *pcmd = malloc(sizeof(cis_cmd_t)+ 9 * sizeof(char));
	uint8_t cis_ret = 1;

	if(pcmd == NULL)
		return 1;

	pcmd->id.flags 	= 0;
	pcmd->id.sport 	= 1;
	pcmd->id.dport 	= 18;
	pcmd->id.dst	= 28;
	pcmd->id.src	= 26;
	pcmd->id.pri	= 0;

	pcmd->cmd[0] 	= 0x1c;
	pcmd->cmd[1] 	= 0xcc;
	pcmd->cmd[2] 	= 0x28;
	pcmd->cmd[3] 	= 0x0a;
	pcmd->cmd[4] 	= 0x04;
	pcmd->cmd[5] 	= (uint8_t)(ticks);
	pcmd->cmd[6] 	= (uint8_t)(ticks >> 8);
	pcmd->cmd[7] 	= (uint8_t)(ticks >> 16);
	pcmd->cmd[8] 	= (uint8_t)(ticks >> 24);

	pcmd->id.head = hton32(pcmd->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pcmd, sizeof(cis_cmd_t)+ 9 * sizeof(char), NULL, 0, CIS_DELAY);

	free(pcmd);

	return cis_ret;
}


	/*read FRAM (para@[FRAM address],[The number of return bytes])
	 * The first byte is the instruction return value,the other LEN-1
	 * bytes are valid data.*/
uint8_t read_fram (uint32_t addr, uint8_t len) {
	cis_cmd_t *pcmd = malloc(sizeof(cis_cmd_t)+ 9 * sizeof(char));
	uint8_t cis_ret = 1;

	if(pcmd == NULL)
		return 1;

	pcmd->id.flags 	= 0;
	pcmd->id.sport 	= 1;
	pcmd->id.dport 	= 18;
	pcmd->id.dst	= 28;
	pcmd->id.src	= 26;
	pcmd->id.pri	= 0;

	pcmd->cmd[0] 	= 0x1c;
	pcmd->cmd[1] 	= 0xcc;
	pcmd->cmd[2] 	= 0x81;
	pcmd->cmd[3] 	= 0x0a;
	pcmd->cmd[4] 	= 0x04;
	pcmd->cmd[5] 	= (uint8_t)(addr);
	pcmd->cmd[6] 	= (uint8_t)(addr >> 8);
	pcmd->cmd[7] 	= (uint8_t)(addr >> 16);
	pcmd->cmd[8] 	= (uint8_t)(len);

	pcmd->id.head = hton32(pcmd->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pcmd, sizeof(cis_cmd_t)+ 9 * sizeof(char), NULL, 0, CIS_DELAY);

	free(pcmd);

	return cis_ret;
}


uint8_t ccsds_off (void) {
	cis_cmd_t *pcmd = malloc(sizeof(cis_cmd_t)+ 5 * sizeof(char));
	uint8_t cis_ret = 1;

	if(pcmd == NULL)
		return 1;

	pcmd->id.flags 	= 0;
	pcmd->id.sport 	= 1;
	pcmd->id.dport 	= 18;
	pcmd->id.dst	= 28;
	pcmd->id.src	= 26;
	pcmd->id.pri	= 0;

	pcmd->cmd[0] 	= 0x1c;
	pcmd->cmd[1] 	= 0xcc;
	pcmd->cmd[2] 	= 0x13;
	pcmd->cmd[3] 	= 0x0a;
	pcmd->cmd[4] 	= 0x00;

	pcmd->id.head = hton32(pcmd->id.head);
	i2c_master_transaction(CSP_HANDLE, CIS_ADDR, pcmd, sizeof(cis_cmd_t)+ 5 * sizeof(char), NULL, 0, CIS_DELAY);

	free(pcmd);



	return cis_ret;
}
