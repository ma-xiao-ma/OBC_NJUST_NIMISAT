/*
 * cmd_intadc.c
 *
 *  Created on: 2016年5月15日
 *      Author: Administrator
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "driver_debug.h"
#include "command.h"

#include "bsp_intadc.h"

int intadc_read_cmd(struct command_context * ctx __attribute__((unused)))
{
	short temp = 0;
	temp = get_mcu_temp();

	printf("Temperature: %.2f C\r\n", temp / 100.0);
	return CMD_ERROR_NONE;
}

struct command cmd_intadc_sub[] = {
	{
		.name = "read",
		.handler = intadc_read_cmd,
		.help = "CPU adc",
	}
};

command_t __root_command cmd_intadc_master[] = {
	{
		.name = "intadc",
		.help = "intadc test",
		.chain = INIT_CHAIN(cmd_intadc_sub),
	}
};

void cmd_intadc_setup(void) {
	command_register(cmd_intadc_master);
}
