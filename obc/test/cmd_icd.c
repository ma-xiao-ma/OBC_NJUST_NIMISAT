/*
 * cmd_isis.c
 *
 *  Created on: 2016年4月29日
 *      Author: Administrator
 */

/*
 * cmd_i2c.c
 *
 *  Created on: 2016年3月25日
 *      Author: Administrator
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"

#include "driver_debug.h"
#include "command.h"
#include "console.h"

#include "bsp_icd.h"
#include "bsp_pca9665.h"
#include "bsp_icd.h"

#include "QB50_mem.h"

typedef struct __attribute__((packed)){
	uint8_t cmd;
	uint8_t data[0];
} isis_test;

typedef struct __attribute__((packed)){
	uint16_t backpower;
    uint16_t temp;
    uint16_t forwardpower;
    uint16_t elec;
} measurestatus;

typedef struct __attribute__((packed)){
	uint16_t tranelec;
    uint16_t dop;
    uint16_t recelec;
    uint16_t VCC;
    uint16_t localtemp;
	uint16_t powertemp;
	uint16_t RSSI;
} measureofrecever;


int isis_send_handler(struct command_context * context) {

	int cmd=0, length=0;
	int date=0;
	isis_test * txbuf = qb50Malloc(I2C_MTU);


	char * args = command_args(context);

	if (sscanf(args, "%d %s %d", &cmd, &txbuf->data[0], &length) != 3) {
		qb50Free(txbuf);
		return CMD_ERROR_SYNTAX;
	}

	if(length >= I2C_MTU) {
		length = I2C_MTU-1;
	}

	txbuf->cmd = cmd;
	txbuf->data[length] = '\0';

	printf("cmd: %d data: %s length: %d\n", txbuf->cmd, txbuf->data, length);

	if(i2c_master_transaction(0, Transmitter_Address, txbuf, length+1, &date, 1, ISIS_TIMEOUT) != -1) {
		printf("isis send transaction error\n");
		qb50Free(txbuf);

		return CMD_ERROR_NONE;
	}

	//length = *((uint8_t *)(&txbuf));
	printf("rest fifo length: %d\n", date);

	qb50Free(txbuf);

	return CMD_ERROR_NONE;
}

int isis_status_handler(struct command_context * context __attribute__((unused))) {

	uint8_t rest_num = 0;

	I2C_ICD_read_workstate(&rest_num);
	if(I2C_ICD_read_workstate(&rest_num)!= -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("the status of machine: %d\n", rest_num);

	return CMD_ERROR_NONE;
}

int isis_read_count_handler(struct command_context * context __attribute__((unused))) {

	uint16_t rest_num = 0;

	I2C_ICD_read_countofframe((uint8_t*)&rest_num);
	if(I2C_ICD_read_countofframe((uint8_t*)&rest_num) != -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("number: %d\n", rest_num);

	return CMD_ERROR_NONE;
}

int isis_transtime_handler(struct command_context * context __attribute__((unused))) {


	time_t  time1 = 0;

	if(I2C_ICD_read_Transmitter_systime((uint8_t*)&time1) != -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("UTC time: %s\r\n", ctime(&time1));

	return CMD_ERROR_NONE;
}

int isis_recetime_handler(struct command_context * context __attribute__((unused))) {


	time_t  time1 = 0;

	if(I2C_ICD_read_Receiver_systime((uint8_t*)&time1) != -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("UTC time: %s\r\n", ctime(&time1));

	return CMD_ERROR_NONE;
}


int isis_rate_handler(struct command_context * context __attribute__((unused))) {

	uint32_t rate = 0;

	char * args = command_args(context);

	if (sscanf(args, "%u" , &rate) != 1)
		return CMD_ERROR_SYNTAX;

	printf("rate: %u\r\n", rate);

	if(I2C_ICD_Set_transmission_bitrate((uint8_t*)&rate)!= -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("set ok \r\n");

	return CMD_ERROR_NONE;
}

int isis_sleep_status_handler(struct command_context * context __attribute__((unused))) {

	uint32_t sleep_status = 0;

	char * args = command_args(context);

	if (sscanf(args, "%u" , &sleep_status) != 1)
		return CMD_ERROR_SYNTAX;

	printf("sleep_status: %u\r\n", sleep_status);

	if(I2C_ICD_Set_transmitter_idle_state((uint8_t*)&sleep_status)!= -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("set ok\r\n");

	return CMD_ERROR_NONE;
}

int isis_measure_status_handler(struct command_context * context __attribute__((unused))) {

	measurestatus  status = {0};

	if(I2C_ICD_read_Transmitter_all_states((uint8_t*)&status) != -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("backpower: %d,temp: %d,forwardpower: %d,elec: %d \r\n", status.backpower,status.temp,status.forwardpower,status.elec);

	return CMD_ERROR_NONE;
}

int isis_last_status_handler(struct command_context * context __attribute__((unused))) {

	measurestatus  status = {0};

	if(I2C_ICD_read_laststate((uint8_t*)&status) != -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("backpower: %d,temp: %d,forwardpower: %d,elec: %d \r\n", status.backpower,status.temp,status.forwardpower,status.elec);

	return CMD_ERROR_NONE;
}

int isis_orgcall_handler(struct command_context * context __attribute__((unused))) {

	uint8_t orgcall[8] = {'\0'};

	char * args = command_args(context);

	if (sscanf(args, "%s" , orgcall) != 1)
		return CMD_ERROR_SYNTAX;

	printf("orgcall: %s\r\n", orgcall);

	if(I2C_ICD_Set_orig_callsigns(orgcall)!= -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("set ok \r\n");

	return CMD_ERROR_NONE;
}
int isis_descall_handler(struct command_context * context __attribute__((unused))) {

	uint8_t descall[8] = {'\0'};

	char * args = command_args(context);

	if (sscanf(args, "%s" , descall) != 1)
		return CMD_ERROR_SYNTAX;

	printf("descall: %s\r\n", descall);

	if(I2C_ICD_Set_orig_callsigns(descall)!= -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("set ok \r\n");

	return CMD_ERROR_NONE;
}

int isis_clearben_handler(struct command_context * context __attribute__((unused))) {

	if(I2C_ICD_beacon_clear() != -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("set ok \r\n");

	return CMD_ERROR_NONE;
}

int isis_setAX_handler(struct command_context * context __attribute__((unused))) {

	uint32_t	beacon=0;
	uint8_t		origcall[8] = {'\0'};
	uint8_t		descall[8] = {'\0'};
	uint8_t 	length = 0;
	uint8_t		*date = qb50Malloc(I2C_MTU);

	char * args = command_args(context);

	if (sscanf(args, "%u %s %s %s %d", &beacon, origcall, descall, date, &length) != 5) {
		qb50Free(date);

		return CMD_ERROR_SYNTAX;
	}


	printf("beacon: %u, origcall: %s, descall: %s, date: %s, length: %d\r\n", beacon, origcall, descall, date, length);
	//printf("descall: %s\r\n", orgcall);

	if(I2C_ICD_Axbeacon_set((uint8_t*)&beacon, origcall, descall, date, length)!= -1) {
		printf("isis send transaction error\n");
		qb50Free(date);
		return CMD_ERROR_NONE;
	}

	printf("set ok \r\n");
	qb50Free(date);
	return CMD_ERROR_NONE;
}

int isis_setbeacon_handler(struct command_context * context __attribute__((unused))) {

	uint32_t	beacon=0;
	uint32_t 	length = 0;
	uint8_t		*date = qb50Malloc(I2C_MTU);

	char * args = command_args(context);

	if (sscanf(args, "%u %s %u", &beacon, date, &length) != 3) {
		qb50Free(date);
		return CMD_ERROR_SYNTAX;
	}

	printf("beacon: %u ,date: %s , length: %u ", beacon, date, length);
	//printf("descall: %s\r\n", orgcall);

	if (I2C_ICD_beacon_set((uint8_t*)&beacon, date, length)!= -1) {
		printf("isis send transaction error\n");
		qb50Free(date);
		return CMD_ERROR_NONE;
	}

	printf("set ok \r\n");
	qb50Free(date);
	return CMD_ERROR_NONE;
}

int isis_sendAXdate_handler(struct command_context * context __attribute__((unused))) {

	uint8_t	origcall[8] = {'\0'};
	uint8_t	descall[8] = {'\0'};
	uint32_t length = 0;
	uint8_t* date = qb50Malloc(I2C_MTU);
	uint32_t frame = 0;
	char * args = command_args(context);

	if (sscanf(args, "%s %s %s %u", descall, origcall, date, &length) != 4) {
		qb50Free(date);
		return CMD_ERROR_SYNTAX;
	}

	date[length] = '\0';

	printf("descall: %s, origcall :%s ,date: %s, length: %u\r\n", descall, origcall, date, length);
	//printf("descall: %s\r\n", orgcall);

	if (I2C_ICD_send_Axdate(origcall, descall, date, length, (uint8_t*)&frame)!= -1) {
		printf("isis send transaction error\n");
		qb50Free(date);
		return CMD_ERROR_NONE;
	}

	printf("return one byte: %u\r\n", frame);
	qb50Free(date);

	return CMD_ERROR_NONE;
}

int isis_senddate_handler(struct command_context * context __attribute__((unused))) {

	//u8 rate =0x08;
	uint32_t length = 0;
	uint8_t* date = qb50Malloc(I2C_MTU);
	uint32_t frame=0;
	char * args = command_args(context);

	if (sscanf(args, "%s %d", date, &length) != 2) {
		qb50Free(date);
		return CMD_ERROR_SYNTAX;
	}

	date[length] = '\0';

	printf("date: %s, length: %u \r\n ", date, length);
	//printf("descall: %s\r\n", orgcall);
	//2016.6.25

/*	if(I2C_ICD_Set_transmission_bitrate((uint8_t*)&rate)!= -1) {
			printf("isis send transaction error\n");

			return CMD_ERROR_NONE;
		}
*/

	if (I2C_ICD_send_date(date, length, (uint8_t*)&frame)!= -1) {
		printf("isis send transaction error\n");
		qb50Free(date);
		return CMD_ERROR_NONE;
	}

	printf("return one byte: %u\n", frame);
	qb50Free(date);

	return CMD_ERROR_NONE;
}

int isis_clearbutter_handler(struct command_context * context __attribute__((unused))) {

	if(I2C_ICD_sweep_butter() != -1) {

		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("set ok \r\n");

	return CMD_ERROR_NONE;
}

int isis_read_frame_handler(struct command_context * context __attribute__((unused))) {

	uint8_t *data = qb50Malloc(I2C_MTU);

	memset(data, 0, I2C_MTU);

	if( I2C_ICD_read_frame(data)  != -1) {

		qb50Free(data);
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	uint16_t length = (*((uint16_t*)&data[0]));
	int i = 0;

	for(;i<length+6;i++) {
		printf("0x%.2x  ", data[i]);
	}
	printf("\r\n");

//	data[(*((uint16_t*)&data[0]))] = '\0';
//	printf("data: %s\r\n", data);

	qb50Free(data);

	return CMD_ERROR_NONE;
}

int isis_read_frame200_handler(struct command_context * context __attribute__((unused))) {

	uint8_t *data = qb50Malloc(I2C_MTU);

	memset(data, 0, I2C_MTU);

	uint16_t count = 206;
	size_t txlen = 1;
	uint8_t * rxbuf = data;
	uint8_t txbuf = Get_frames;

	int result = i2c_master_transaction(ISIS_HANDLE, Receiver_Address, &txbuf, txlen, rxbuf, count, 1000);

	if( result  != -1) {

		qb50Free(data);
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	uint16_t length = (*((uint16_t*)&data[0]));
	int i = 0;

	printf("length: %u\r\n", length);

	for(;i<count;i++) {
		printf("0x%.2x  ", data[i]);
	}
	printf("\r\n");

	qb50Free(data);

	return CMD_ERROR_NONE;
}

int isis_resetdog_handler(struct command_context * context __attribute__((unused))) {

	if(I2C_ICD_Watchingdog_reset() != -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("set ok \r\n");

	return CMD_ERROR_NONE;
}

int isis_softwarereset_handler(struct command_context * context __attribute__((unused))) {

	if(I2C_ICD_software_reset() != -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("set ok \r\n");

	return CMD_ERROR_NONE;
}

int isis_hardwarereset_handler(struct command_context * context __attribute__((unused))) {

	if(I2C_ICD_Hardware_system_reset() != -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("set ok \r\n");

	return CMD_ERROR_NONE;
}

int isis_measureofreciever_handler(struct command_context * context __attribute__((unused))) {

	measureofrecever date={0};

	if( I2C_ICD_read_Receiver_all_states((uint8_t*)&date)  != -1) {
		printf("isis send transaction error\n");

		return CMD_ERROR_NONE;
	}

	printf("tranelec: %d,dop: %d,recelec: %d,VCC: %d,localtemp: %d, powertemp: %d,RSSI: %d\r\n", date.tranelec,date.dop,date.recelec,date.VCC,date.localtemp,date.powertemp,date.RSSI);

	return CMD_ERROR_NONE;
}

struct command cmd_isis_sub[] = {
	{
		.name = "send",
		.help = "i2c_send date without AX2.5",
		.usage = "<cmd><data><length>",
		.handler = isis_send_handler,
	},{
		.name = "status",
		.help = "isis_read the status of tansmmiter",
		.handler = isis_status_handler,
	},{
		.name = "tramstime",
		.help = "isis_ show the time of tansmmiter ",
		.handler = isis_transtime_handler,
	},{
		.name = "rate",
		.help = "isis_set rate of tansmmiter",
		.usage = "<rate>",
		.handler = isis_rate_handler,
	},{
		.name = "measure_status",
		.help = "isis_show measure_status of tansmmiter",
		.handler = isis_measure_status_handler,
	},{
		.name = "last_status",
		.help = "isis_show last measure_status of tansmmiter",
		.handler = isis_last_status_handler,
	 },{
		.name = "sleep_status",
		.help = "isis_set sleep_status of tansmmiter",
		.usage = "<date>",
		.handler = isis_sleep_status_handler,
	},{
		.name = "orgcall",
		.help = "isis_set_orgcall of tansmmiter",
		.usage = "<date>",
		.handler = isis_orgcall_handler,
	},{
		.name = "descall",
		.help = "isis_set_descall of tansmmiter",
		.usage = "<date>",
		.handler = isis_descall_handler,
	},{
		.name = "clearben",
		.help = "isis_clearben of tansmmiter",
		.handler = isis_clearben_handler,
	},{
		.name = "setAX2.5",
		.help = "i2c_set_AX2.5 of tansmmiter",
		.usage = "<time><des><org><date><length>",
		.handler = isis_setAX_handler,
	},{
		.name = "setbeacon",
		.help = "i2c_set_beacon of tansmmiter",
		.usage = "<time><date><length>",
		.handler = isis_setbeacon_handler,
	},{
		.name = "sendAX",
		.help = "i2c_send date with AX",
		.usage = "<des><org><date><length>",
		.handler = isis_sendAXdate_handler,
	},{
		.name = "recetime",
		.help = "isis_show recetime",
		.handler = isis_recetime_handler,
	},{
		.name = "cbutter",
		.help = "isis_clear receiver butter",
		.handler = isis_clearbutter_handler,
	},{
		.name = "rcount",
		.help = "isis_read_countof rece frame",
		.handler = isis_read_count_handler,
	},{
		.name = "rframe",
		.help = "isis_read_frame of receiver",
		.handler = isis_read_frame_handler,
	},{
		.name = "resetdog",
		.help = "isis_resetdog",
		.handler = isis_resetdog_handler,
	},{
		.name = "softwarereset",
		.help = "isis_softwarereset",
		.handler = isis_softwarereset_handler,
	},{
		.name = "hwrst",
		.help = "isis_hardwarereset",
		.handler = isis_hardwarereset_handler,
	},{
		.name = "meaofrec",
		.help = "isis_measureofreciever",
		.handler = isis_measureofreciever_handler,
	},{
		.name = "senddate",
		.help = "i2c_send date without AX",
		.usage = "<date><length>",
		.handler = isis_senddate_handler,
	},{
		.name = "r200",
		.help = "isis_read_frame of receiver",
		.handler = isis_read_frame200_handler,
	}
};

command_t __root_command cmd_isis_master[] = {
	{
		.name = "isis",
		.help = "isis test",
		.chain = INIT_CHAIN(cmd_isis_sub),
	}
};

void cmd_isis_setup(void) {
	command_register(cmd_isis_master);
}

