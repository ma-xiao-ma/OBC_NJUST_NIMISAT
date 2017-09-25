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

int intadc_read_cmd(struct command_context * ctx) {

	double data = 0;

	char * args = command_args(ctx);

//	Get_Adc((uint8_t)16, &data, ADC_DELAY);

//	printf("temperature: %f\n", (data/4096.0*2.05-0.76)/0.0025+25);
	data = Get_Temprate();
	printf("Temperature: %f\n", data);
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
