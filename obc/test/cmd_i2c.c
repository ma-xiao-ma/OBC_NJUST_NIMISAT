/*
 * cmd_i2c.c
 *
 *  Created on: 2016骞�3鏈�25鏃�
 *      Author: Administrator
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "driver_debug.h"
#include "command.h"
#include "console.h"
#include "qb50_mem.h"

#include "bsp_pca9665.h"
#include "bsp_cis.h"

#include "hk.h"

int i2c_csp(struct command_context * context) {

	char * args = command_args(context);
	int addr, flags, sport, dport, dst, src, pri;

	cis_frame_t *pbuf = NULL;

	pbuf = (cis_frame_t *)qb50Malloc(sizeof(cis_frame_t));

	if(pbuf == NULL) {
		return CMD_ERROR_NOMEM;
	}

	if (sscanf(args, "%d %d %d %d %d %d %d", &addr, &flags, &sport, &dport, &dst, &src, &pri) != 7) {
		free(pbuf);
		return CMD_ERROR_SYNTAX;
	}

	pbuf->id.flags 	= flags;
	pbuf->id.sport 	= sport;
	pbuf->id.dport 	= dport;
	pbuf->id.dst	= dst;
	pbuf->id.src	= src;
	pbuf->id.pri	= pri;

//	int i = 1;
//
//	pbuf->buf[0] = 0x1a;
//	for(;i<96;i++){
//		pbuf->buf[i] = i;
//	}

//	hk_collect_test();

//	memcpy(&(pbuf->buf[0]),(uint8_t *)&(hk_frame.main_frame.header[0]),sizeof(HK_Main_t));


	printf("addr:%d flags:%d sport:%d dport:%d dst:%d src:%d pri:%d\n",
			addr, pbuf->id.flags, pbuf->id.sport, pbuf->id.dport,
			pbuf->id.dst, pbuf->id.src, pbuf->id.pri);

	//BIG
	pbuf->id.head = hton32(pbuf->id.head);
	i2c_master_transaction(CSP_HANDLE, addr, pbuf, sizeof(cis_frame_t), NULL, 0, 5000);

	qb50Free(pbuf);

	return CMD_ERROR_NONE;
}

int i2c_csp_head(struct command_context * context) {

	char * args = command_args(context);
	int addr;

	cis_frame_t *pbuf = NULL;

	pbuf = (cis_frame_t *)malloc(sizeof(cis_frame_t));

	if(pbuf == NULL) {
		return CMD_ERROR_NOMEM;
	}

	if (sscanf(args, "%d %u", &addr, &(pbuf->id)) != 2) {
		free(pbuf);
		return CMD_ERROR_SYNTAX;
	}

	printf("addr:%d flags:%d sport:%d dport:%d dst:%d src:%d pri:%d\n",
			addr, pbuf->id.flags, pbuf->id.sport, pbuf->id.dport,
			pbuf->id.dst, pbuf->id.src, pbuf->id.pri);

	int i = 0;

	for(;i<96;i++){
		pbuf->buf[i] = i;
	}

	//BIG

	i2c_master_transaction(CSP_HANDLE, addr, pbuf, sizeof(cis_frame_t), NULL, 0, 5000);

	free(pbuf);

	return CMD_ERROR_NONE;
}

int i2c_send_handler(struct command_context * context) {

	char * args = command_args(context);
	int handle, addr;
	char *sbuf, *rbuf;
	int slen, rlen;

	sbuf = (char *)malloc(200);
	if(sbuf == NULL) return CMD_ERROR_NOMEM;
	rbuf = (char *)malloc(200);
	if(rbuf == NULL) return CMD_ERROR_NOMEM;

	printf("Pleas input <handle> <addr> <sbuf> <rlen>");

	if(sscanf(args, "%d %d %s %d", &handle, &addr, sbuf, &rlen) != 4) {
		free(sbuf);
		free(rbuf);
		return CMD_ERROR_SYNTAX;
	}

	rlen = (abs(rlen)>200?200:(abs(rlen)));

	if(i2c_master_transaction(handle, addr, sbuf, strlen(sbuf), rbuf, rlen, 5000) != -1) {
		printf("i2c master transaction error\n");
		free(sbuf);
		free(rbuf);
		return CMD_ERROR_NONE;
	}

	while(rlen--)
		printf("%c ", *(rbuf++));
	printf("\n");

	free(sbuf);
	free(rbuf);

	return CMD_ERROR_NONE;
}

int i2c_conn_handler(struct command_context * context) {

	char * args = command_args(context);
	int handle, addr;

	if (sscanf(args, "%d %d", &handle, &addr) != 2)
		return CMD_ERROR_SYNTAX;

	i2c_master_transaction(handle, addr, "buffer", 6, NULL, 0, 1000);

	return CMD_ERROR_NONE;
}

int pca9665_dump_regs_cmd(struct command_context * ctx) {
	int handler;
	if (ctx->argc != 2)
		return CMD_ERROR_SYNTAX;
	handler = atoi(ctx->argv[1]);
	pca9665_dump_regs(handler);
	return CMD_ERROR_NONE;
}

int pca9665_heap_size_cmd(struct command_context * ctx __attribute__((unused))) {

	size_t heap_size = qb50GetFreeHeapSize();

	printf("I2C free heap size: %d\r\n", heap_size);
	return CMD_ERROR_NONE;
}

struct command cmd_i2c_sub[] = {
	{
		.name = "conn",
		.help = "Connect the other node",
		.usage = "<handle><addr>",
		.handler = i2c_conn_handler,
	},{
		.name = "csp",
		.help = "Connect the U/V",
		.usage = "<addr><flags><sport><dport><dst><src><pri>",
		.handler = i2c_csp,
	},{
		.name = "head",
		.help = "Connect the U/V",
		.usage = "<addr><head>",
		.handler = i2c_csp_head,
	},{
		.name = "send",
		.help = "i2c transaction",
		.usage = "<handle><addr>",
		.handler = i2c_send_handler,
	},{
		.name = "i2cdump",
		.handler = pca9665_dump_regs_cmd,
		.help = "Dump PCA9665 registers",
		.usage = "<handle-id>"
	},{
		.name = "heap",
		.handler = pca9665_heap_size_cmd,
		.help = "Free heap size",
	}
};

command_t __root_command cmd_i2c_master[] = {
	{
		.name = "i2c",
		.help = "i2c test",
		.chain = INIT_CHAIN(cmd_i2c_sub),
	}
};

void cmd_i2c_setup(void) {
	command_register(cmd_i2c_master);
}
