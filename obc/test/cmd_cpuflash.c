/*
 * cmd_i2c.c
 *
 *  Created on: 2016年3月25日
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
#include "hk_arg.h"
#include "obc_argvs_save.h"

#include "bsp_cpu_flash.h"

int flash_write_handler(struct command_context * context) {

	char * args = command_args(context);
	obc_save_t obc_save_test = {0};

	if (sscanf(args, "%u %u %u", &obc_save_test.obc_boot_count, &obc_save_test.obc_reset_time, &obc_save_test.antenna_status) != 3)
		return CMD_ERROR_SYNTAX;

	printf("boot: %u rsttime: %u ants: %u\n", obc_save_test.obc_boot_count, obc_save_test.obc_reset_time, obc_save_test.antenna_status);

	bsp_WriteCpuFlash(OBC_STORE_FLASH_ADDR, (uint8_t*)&obc_save_test, sizeof(obc_save_test));

	obc_save_test.antenna_status = 0;
	obc_save_test.obc_boot_count = 0;
	obc_save_test.antenna_status = 0;

	bsp_ReadCpuFlash(OBC_STORE_FLASH_ADDR, (uint8_t*)&obc_save_test, sizeof(obc_save_test));

	printf("read from flash:\n");
	printf("boot: %u rsttime: %u ants: %u\n", obc_save_test.obc_boot_count, obc_save_test.obc_reset_time, obc_save_test.antenna_status);

	return CMD_ERROR_NONE;
}

int flash_read_handler(struct command_context * context __attribute__((unused))) {

	obc_save_t obc_save_test = {0};

	bsp_ReadCpuFlash(OBC_STORE_FLASH_ADDR, (uint8_t*)&obc_save_test, sizeof(obc_save_test));

	printf("read from flash:\n");
	printf("boot: %u rsttime: %u ants: %u\n", obc_save_test.obc_boot_count, obc_save_test.obc_reset_time, obc_save_test.antenna_status);

	return CMD_ERROR_NONE;
}

int flash_erase_handler( struct command_context * context __attribute__((unused)) )
{
    if (bsp_EraseCpuFlash(OBC_STORE_FLASH_ADDR) != 0)
        printf("ERROR: Erase system data sector fail!\r\n");
    else
        printf("INFO: Success!!!\r\n");

    return CMD_ERROR_NONE;
}

struct command cmd_nor_sub[] = {
	{
		.name = "read",
		.help = "read flash",
		.handler = flash_read_handler,
	},
	{
		.name = "write",
		.help = "write to flash",
		.usage = "<boot><rst><ants>",
		.handler = flash_write_handler,
	},
    {
        .name = "erase",
        .help = "erase system data sector",
        .handler = flash_erase_handler,
    }
};

command_t __root_command cmd_nor_master[] = {
	{
		.name = "flash",
		.help = "flash test",
		.chain = INIT_CHAIN(cmd_nor_sub),
	}
};

void cmd_nor_setup(void) {
	command_register(cmd_nor_master);
}
