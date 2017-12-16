/*
 * cmd_switches.c
 *
 *  Created on: 2016年5月10日
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bsp_delay.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "driver_debug.h"
#include "command.h"

#include "stm32f4xx.h"

#include "cubesat.h"
#include "switches.h"


int cmd_panel(struct command_context *ctx) {

	char * args = command_args(ctx);
	uint32_t opt;
	uint32_t delay;

	if (sscanf(args," %u %u", &opt, &delay) != 2)
		return CMD_ERROR_SYNTAX;

	if(opt == 0)	{
		int res = disable_panel(delay, 0);
		if(res != -1)
			printf("SEND CMD FAILED\r\nRes: %d, \r\n", res);
		else
			printf("panel close\r\n");
	}

	if(opt == 1)	{
		int res = enable_panel(delay, 0);
		if(res != 1)
			printf("SEND CMD FAILED\r\nRes: %d, \r\n", res);
		else
			printf("panel open\r\n");
	}

	if(opt != 1 && opt != 0)
		printf("No such option\r\n");

	return CMD_ERROR_NONE;
}

int cmd_unfold_panel(struct command_context *ctx) {

    char * args = command_args(ctx);
    uint32_t opt;
    uint32_t delay;
    uint16_t time;

    if (sscanf(args," %u %u %u", &opt, &delay, &time) != 3)
        return CMD_ERROR_SYNTAX;

    if(opt == 0)    {
        int res = disable_unfold_panel(delay);
        if(res != -1)
            printf("SEND CMD FAILED\r\nRes: %d, \r\n", res);
        else
            printf("unfold panel close\r\n");
    }

    if(opt == 1)    {
        int res = enable_unfold_panel(delay, time);
        if(res != 1)
            printf("SEND CMD FAILED\r\nRes: %d, \r\n", res);
        else
            printf("unfold panel open\r\n");
    }

    if(opt != 1 && opt != 0)
        printf("No such option\r\n");

    return CMD_ERROR_NONE;
}



int cmd_switch_status(struct command_context *ctx __attribute__((unused))) {

	int result = 0;

	uint8_t status[4] = {0};


	result = get_switch_status((uint8_t *)status);

    printf("Item\t\tStatus\r\n");
    printf("**********************\r\n");

    /*第一个字节*/

	if(result == 1)
	{
	    if(status[0] & ANTS1) {
	        printf("ants1\t\topen\r\n");
	    }else{
	        printf("ants1\t\tclose\r\n");
	    }

	    if(status[0] & ANTS2) {
	        printf("ants2\t\topen\r\n");
	    }else{
	        printf("ants2\t\tclose\r\n");
	    }

	    if(status[0] & ANTS3) {
	        printf("ants3\t\topen\r\n");
	    }else{
	        printf("ants3\t\tclose\r\n");
	    }

	    if(status[0] & ANTS4) {
	        printf("ants4\t\topen\r\n");
	    }else{
	        printf("ants4\t\tclose\r\n");
	    }

	    if(status[0] & ARM) {
	        printf("arm\t\topen\r\n");
	    }else{
	        printf("arm\t\tclose\r\n");
	    }
	}

	if(status[0] & PANELA) {
		printf("panela\t\topen\r\n");
	}else{
		printf("panela\t\tclose\r\n");
	}

	if(status[0] & PANELB) {
		printf("panelb\t\topen\r\n");
	}else{
		printf("panelb\t\tclose\r\n");
	}

	if(status[0] & EXPANDABLE_SAIL) {
        printf("sail\t\topen\r\n");
    }else{
        printf("sail\t\tclose\r\n");
    }

	/*第二个字节*/

	if(status[1] & ADCS_EN) {
		printf("adcs\t\topen\r\n");
	}else{
		printf("adcs\t\tclose\r\n");
	}

	if(status[1] & ANTS_EN) {
		printf("ants\t\topen\r\n");
	}else{
		printf("ants\t\tclose\r\n");
	}

	if(status[1] & DTB_5V_EN) {
		printf("dtb_5v\t\topen\r\n");
	}else{
		printf("dtb_5v\t\tclose\r\n");
	}

	if(status[1] & DTB_12V_EN) {
		printf("dtb_12v\t\topen\r\n");
	}else{
		printf("dtb_12v\t\tclose\r\n");
	}

	if(status[1] & CAMERA_10W_5V_EN) {
		printf("cam_10w\t\topen\r\n");
	}else{
		printf("cam_10w\t\tclose\r\n");
	}

	if(status[1] & CAMERA_5W_5V_EN) {
		printf("cam_5w\t\topen\r\n");
	}else{
		printf("cam_5w\t\tclose\r\n");
	}

	if(status[1] & CAMERA_HEAT1_EN) {
		printf("cam_heat1\topen\r\n");
	}else{
		printf("cam_heat1\tclose\r\n");
	}

    if(status[1] & CAMERA_HEAT2_EN) {
        printf("cam_heat2\topen\r\n");
    }else{
        printf("cam_heat2\tclose\r\n");
    }

	/*第三个字节*/

    if(status[2] & JLG_VU_BUS_EN) {
        printf("vu_backup\topen\r\n");
    }else{
        printf("vu_backup\tclose\r\n");
    }

    if(status[2] & BATTERY_HEAT_EN) {
        printf("bat_heat\topen\r\n");
    }else{
        printf("bat_heat\tclose\r\n");
    }
//	if(status[2] & M1_POWER_MASK) {
//		printf("m1_power_mask open\r\n");
//	}else{
//		printf("m1_power_mask close\r\n");
//	}
//
//
//	if(status[2] & M2_POWER_MASK) {
//		printf("m2_power_mask open\r\n");
//	}else{
//		printf("m2_power_mask close\r\n");
//	}
//
//	if(status[2] & M3_POWER_MASK) {
//		printf("m3_power_mask open\r\n");
//	}else{
//		printf("m3_power_mask close\r\n");
//	}
//
//	if(status[2] & M4_POWER_MASK) {
//		printf("m4_power_mask open\r\n");
//	}else{
//		printf("m4_power_mask close\r\n");
//	}
//
//	if(status[2] & OUT_EN_5V) {
//		printf("out_en_5v open\r\n");
//	}else{
//		printf("out_en_5v close\r\n");
//	}
//
//	if(status[2] & GR_POWER_MASK) {
//		printf("gr_power_mask open\r\n");
//	}else{
//		printf("gr_power_mask close\r\n");
//	}
//
//	if(status[2] & MAG_POWER_MASK) {
//		printf("mag_power_mask open\r\n");
//	}else{
//		printf("mag_power_mask close\r\n");
//	}
//
//	if(status[2] & GPS_POWER_MASK) {
//		printf("gps_power_mask open\r\n");
//	}else{
//		printf("gps_power_mask close\r\n");
//	}

	/*第四个字节*/
//	#define MAGA_EN_MASK
//	#define MAGB_EN_MASK
//	#define GPSA_EN_MASK
//	#define GPSB_EN_MASK

//	if(status[3] & MAGA_EN_MASK) {
//		printf("maga_en_mask open\r\n");
//	}else{
//		printf("maga_en_mask close\r\n");
//	}
//
//	if(status[3] & MAGB_EN_MASK) {
//		printf("magb_en_mask open\r\n");
//	}else{
//		printf("magb_en_mask close\r\n");
//	}
//
//	if(status[3] & GPSA_EN_MASK) {
//		printf("gpsa_en_mask open\r\n");
//	}else{
//		printf("gpsa_en_mask close\r\n");
//	}
//
//	if(status[3] & GPSB_EN_MASK) {
//		printf("gpsb_en_mask open\r\n");
//	}else{
//		printf("gpsb_en_mask close\r\n");
//	}
//
//	if(status[3] & MAGBAR_EN_MASK) {
//		printf("magbar_en_mask open\r\n");
//	}else{
//		printf("magbar_en_mask close\r\n");
//	}

	return CMD_ERROR_NONE;
}

struct command switches_subcommands[] = {

    {
		.name = "hk",
		.help = "read switches status",
		.handler = cmd_switch_status,
	},
	{
		.name = "panel",
		.help = "panel switches",
		.handler = cmd_panel,
		.usage = "<opts> 0:close 1:open <delay>",
	},{
        .name = "unfold_panel",
        .help = "unfold panel switches",
        .handler = cmd_unfold_panel,
        .usage = "<opts> 0:close 1:open <delay>",
    }
};

struct command __root_command switches_commands_master[] =
{
	{
		.name = "sw",
		.help = "switches operation",
		.chain = INIT_CHAIN(switches_subcommands),
	},
};

void cmd_switches_setup(void)
{
	command_register(switches_commands_master);
}
