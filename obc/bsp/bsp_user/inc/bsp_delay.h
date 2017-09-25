/*
 * bsp_delay.h
 *
 *  Created on: 2016年3月25日
 *      Author: Administrator
 */

#ifndef BSP_BSP_USER_INC_BSP_DELAY_H_
#define BSP_BSP_USER_INC_BSP_DELAY_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <inttypes.h>
#include <string.h>
#include <stdint.h>

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void delay_init(uint32_t freq);
void cmd_setup_delay(void);

#endif /* BSP_BSP_USER_INC_BSP_DELAY_H_ */
