/*
 * cmd_dfl.c
 *
 *  Created on: Sep 17, 2012
 *      Author: johan
 */

#include <cubesat.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "bsp_reset.h"

#include "cubesat.h"
#include "ctrl_cmd_types.h"
#include "switches.h"
#include "QB50_mem.h"
#include "cube_com.h"
#include "hk_arg.h"

#include "bsp_pca9665.h"

#include "FreeRTOS.h"
#include "task.h"
#include "command.h"

static uint8_t (*console_getc)(void) = usart_getc;
static void (*console_putc)(uint8_t) = usart_putc;

#include "driver_debug.h"
#include "command.h"

int help_handler(struct command_context * context) {
	command_help(command_args(context));
	return CMD_ERROR_NONE;
}

int sleep_handler(struct command_context * context) {
	unsigned long sleep_ms;

	if (context->argc != 2)
		return CMD_ERROR_SYNTAX;

	sleep_ms = atoi(context->argv[1]);

	if (sleep_ms < 1)
		return CMD_ERROR_SYNTAX;

	vTaskDelay(sleep_ms * (configTICK_RATE_HZ / 1000.0));

	return CMD_ERROR_NONE;
}


int watch_handler(struct command_context * context) {

	int sleep_ms = atoi(context->argv[1]);

	if (sleep_ms < 1)
		return CMD_ERROR_SYNTAX;

	printf("Execution delay: %d\r\n", sleep_ms);

	char * new_command = strstr(command_args(context), " ");

	if (new_command == NULL)
		return CMD_ERROR_SYNTAX;
	else
		new_command = new_command + 1;

	printf("Command: %s\r\n", new_command);

	while(1) {

		if (usart_messages_waiting())
			break;

		command_run(new_command);

		vTaskDelay(sleep_ms * (configTICK_RATE_HZ / 1000.0));

	}

	return CMD_ERROR_NONE;

}

#define CONTROL(X)  ((X) - '@')

int batch_handler(struct command_context * ctx __attribute__((unused))) {

	char c;
	int quit = 0, execute = 0;
	unsigned int batch_size = 100;
	unsigned int batch_input = 0;
	unsigned int batch_count = 0;
	char * batch[20] = {};
	printf("Type each command followed by enter, hit ctrl+e to end typing, ctrl+x to cancel:\r\n");

	/* Wait for ^q to quit. */
	while (quit == 0) {

		/* Get character */
		c = console_getc();

		switch (c) {

		/* CTRL + X */
		case 0x18:
			quit = 1;
			break;

		/* CTRL + E */
		case 0x05:
			execute = 1;
			quit = 1;
			break;

		/* Backspace */
		case CONTROL('H'):
		case 0x7f:
			if (batch_input > 0) {
				console_putc( '\b');
				console_putc(' ');
				console_putc('\b');
				batch_input--;
			}
			break;

		case '\r':
			console_putc( '\r');
			console_putc( '\n');
			if ((batch[batch_count] != NULL) && (batch_input < batch_size))
				batch[batch_count][batch_input++] = '\r';
			if ((batch[batch_count] != NULL) && (batch_input < batch_size))
				batch[batch_count][batch_input++] = '\0';
			batch_count++;
			batch_input = 0;
			if (batch_count == 20)
				quit = 1;
			break;

		default:
			console_putc( c);
			if (batch[batch_count] == NULL) {
				batch[batch_count] = calloc(1, 1);
			}

			if ((batch[batch_count] != NULL) && (batch_input < batch_size))
				batch[batch_count][batch_input++] = c;
			break;
		}
	}

	if (execute) {
		printf("\r\n");
		for (unsigned int i = 0; i <= batch_count; i++) {
			if (batch[i])
				printf("[%02u] %s\r\n", i, batch[i]);
		}
		printf("Press ctrl+e to execute, or any key to abort\r\n");
		c = console_getc();
		if (c != 0x05)
			execute = 0;
	}

	/* Run/Free batch job */
	for (unsigned int i = 0; i <= batch_count; i++) {
		if (execute && batch[i]) {
			printf("EXEC [%02u] %s\r\n", i, batch[i]);
			command_run(batch[i]);
		}
		free(batch[i]);
	}

	return CMD_ERROR_NONE;

}

int cpu_reset_handler(struct command_context * context __attribute__((unused))) {

	cpu_reset();

	return CMD_ERROR_NONE;
}

static inline char *cpGetTaskRunState(uint8_t state)
{
    switch(state)
    {
        case 0:
            return "Running";
        case 1:
            return "Ready";
        case 2:
            return "Blocked";
        case 3:
            return "Suspended";
        case 4:
            return "Deleted";
        case 5:
            return "Invalid";
        default:
            return  "\0";
    }
}

int ps_handler(struct command_context * context __attribute__((unused)))
{

    uint32_t TotalRunTime,x;
    TaskStatus_t *StatusArray;
    UBaseType_t ArraySize;
    ArraySize = uxTaskGetNumberOfTasks();
    StatusArray = qb50Malloc(ArraySize * sizeof(TaskStatus_t));

    if(uxTaskGetSystemState((TaskStatus_t *)StatusArray, (UBaseType_t)ArraySize, (uint32_t *)&TotalRunTime) == ArraySize)
    {
        if(StatusArray != NULL)
        {
            printf("TaskName\tTaskNumber\tCurrentState\tCurrentPriority\tBasePriority\tStackHighWaterMark\r\n\r\n");
            for(x=0; x<ArraySize; x++)
            {
                printf("%-10s\t%lu\t\t%-10s\t%lu\t\t%lu\t\t%hu\r\n", StatusArray[x].pcTaskName
                                                                     , StatusArray[x].xTaskNumber
                                                                     , cpGetTaskRunState(StatusArray[x].eCurrentState)
                                                                     , StatusArray[x].uxCurrentPriority
                                                                     , StatusArray[x].uxBasePriority
                                                                     , StatusArray[x].usStackHighWaterMark);
            }
        }
    }

    qb50Free(StatusArray);

	return CMD_ERROR_NONE;
}

int get_boot_count(struct command_context * context __attribute__((unused)))
{
	printf("boot count: %u\n", obc_boot_count);

	return CMD_ERROR_NONE;
}

int get_rst_time(struct command_context * context __attribute__((unused)))
{
	printf("boot time: %s\n", ctime(&obc_reset_time));

	return CMD_ERROR_NONE;
}


int TaskGetRunTimeStats(struct command_context * context __attribute__((unused)))
{
    char *RunTimeInfo;

    RunTimeInfo = (char *)qb50Malloc(400);

    if(RunTimeInfo != NULL)
    {
        memset(RunTimeInfo,0,400);              //信息缓冲区清零
        /*获取任务运行时间信息*/
        vTaskGetRunTimeStats(RunTimeInfo);
        /* 打印表格 */
        printf("TaskName\tRuningTime\tPercentage\r\n");
        printf("%s\r\n",RunTimeInfo);

        qb50Free(RunTimeInfo);
    }

    return CMD_ERROR_NONE;
}

command_t __root_command cmd_dfl[] = {
//	{
//		.name = "rsh",
//		.help = "send rsh cmd",
//		.usage = "<command>",
//		.handler = rsh_cmd_send_command,
//	},
	{
		.name = "time",
		.help = "get reset time",
		.handler = get_rst_time,
	},{
		.name = "boot",
		.help = "get boot count",
		.handler = get_boot_count,
	},{
		.name = "help",
		.help = "Show help",
		.usage = "<command>",
		.handler = help_handler,
	},{
		.name = "sleep",
		.help = "Sleep X ms",
		.usage = "<time>",
		.handler = sleep_handler,
	},{
		.name = "watch",
		.help = "Run cmd at intervals, abort with key",
		.usage = "<n(ms)> <command>",
		.handler = watch_handler,
	},{
		.name = "batch",
		.help = "Run multiple commands",
		.handler = batch_handler,
	},
	{
		.name = "reset",
		.help = "Reset now",
		.handler = cpu_reset_handler,
	},{
		.name = "ps",
		.help = "List tasks",
		.handler = ps_handler,
	},{
#if configGENERATE_RUN_TIME_STATS
        .name = "rts",
        .help = "Task Runing Time State",
        .handler = TaskGetRunTimeStats,
#endif
    },{
#if CONFIG_DRIVER_DEBUG
	.name = "tdebug",
	.help = "Toggle driver debug",
	.usage = "<level>",
	.handler = cmd_driver_debug_toggle,
	}
#endif
};

void cmd_dfl_setup(void) {
	command_register(cmd_dfl);
}
