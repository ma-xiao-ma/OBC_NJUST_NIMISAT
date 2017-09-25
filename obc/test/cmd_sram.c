/*
 * cmd_i2c.c
 *
 *  Created on: 2016年3月25日
 *      Author: Administrator
 */

#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "driver_debug.h"
#include "command.h"
#include "console.h"

#include "bsp_fsmc_sram.h"

int sram_write_handler(struct command_context * context) {

	char * args = command_args(context);
	int addr, data;

	if (sscanf(args, "%d %d", &addr, &data) != 2)
		return CMD_ERROR_SYNTAX;

	printf("addr: %d data: %d\n", addr, data);

	SRAM_WRITE(addr,data);

	data = 0;

	data = SRAM_READ(addr);

	printf("data: %d\n", data);

	return CMD_ERROR_NONE;
}

int sram_read_handler(struct command_context * context) {

	char * args = command_args(context);
	int addr, data;

	if (sscanf(args, "%d", &addr) != 1)
		return CMD_ERROR_SYNTAX;

	printf("addr: %d\n", addr);

	data = 0;

	data = SRAM_READ(addr);

	printf("data: %d\n", data);

	return CMD_ERROR_NONE;
}

struct command cmd_sram_sub[] = {
	{
		.name = "write",
		.help = "write to sram",
		.usage = "<addr><data>",
		.handler = sram_write_handler,
	},{
		.name = "read",
		.help = "read sram",
		.usage = "<addr>",
		.handler = sram_read_handler,
	}
};

command_t __root_command cmd_sram_master[] = {
	{
		.name = "sram",
		.help = "flash test",
		.chain = INIT_CHAIN(cmd_sram_sub),
	}
};

void cmd_sram_setup(void) {
	command_register(cmd_sram_master);
}
