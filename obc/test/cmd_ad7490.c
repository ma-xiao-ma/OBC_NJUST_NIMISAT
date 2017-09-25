/*
 * cmd_ad7490.c
 *
 *  Created on: 2015年7月3日
 *      Author: lee
 */

#include <stdint.h>
#include <stdio.h>

#include "command.h"

#include "bsp_ad7490.h"

int cmd_ad7490_read(struct command_context *ctx __attribute__((unused))){
//	AD7490_Init();
	AD7490_Read_NoIntSeq_own();

	return CMD_ERROR_NONE;
}

struct command ad7490_subcommands[] = {
	{
		.name = "read",
		.help = "adc read",
		.handler = cmd_ad7490_read,
	},
};

struct command __root_command ad7490_commands_master[] =
{
	{
		.name = "ad7490",
		.help = "ad7490 master commands",
		.chain = INIT_CHAIN(ad7490_subcommands),
	},
};

void cmd_ad7490_setup(void)
{
	command_register(ad7490_commands_master);
}
