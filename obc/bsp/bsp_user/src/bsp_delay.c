/*
 * bsp_delay.c
 *
 *  Created on: 2016年3月25日
 *      Author: Administrator
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <inttypes.h>
#include <string.h>
#include <stdint.h>

#include "driver_debug.h"
#include "command.h"

/* Init to 8 MHz to at least get some delay
 * if delay_init is not called */
static uint32_t fcpu = 168000000UL;
static const uint32_t us_per_s = 1000000UL;

/* Measured value. This will change if you update the wait states */
static const uint32_t cycles_per_loop = 6;
static inline void __attribute__ ((always_inline)) delay_cycles(uint32_t cycles) {
	asm volatile (
		"L_%=:					\n\t"
		"	nop					\n\t"
		"	subs %[c], %[c], #1	\n\t"
		"	bne L_%=			\n\t"
		: /* No output */
		: [c] "r" (cycles)
		: /* No clobbered registers */
	);
}

void delay_us(uint32_t us) {
	/* Round up so we loop at least once */
	uint32_t cycles = (((uint64_t)us * (uint64_t)fcpu) / us_per_s) / cycles_per_loop + 1;
	delay_cycles(cycles);
}

void delay_ms(uint32_t ms) {
	/* Round up so we loop at least once */
	uint32_t cycles = (((uint64_t)ms * 1000 * 168)) / cycles_per_loop + 1;
	delay_cycles(cycles);
}

void delay_init(uint32_t freq) {
	fcpu = freq;
}

int cmd_delay(struct command_context *ctx) {

	if (ctx->argc != 2)
		return CMD_ERROR_SYNTAX;

	unsigned long time = atoi(ctx->argv[1]);
	delay_us(time);

	return CMD_ERROR_NONE;
}

command_t __root_command delay_commands[] = {
	{
		.name = "delay",
		.help = "Test busy wait delay loop",
		.usage = "<microseconds>",
		.handler = cmd_delay,
	},
};

void cmd_setup_delay(void) {
	command_register(delay_commands);
}
