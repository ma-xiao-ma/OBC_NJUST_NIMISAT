#ifndef _BSP_H_
#define _BSP_H_


#include <stdio.h>			/* 因为用到了printf函数，所以必须包含这个文件 */
#include "string.h"
#include "stdint.h"
#include "stm32f4xx.h"





void bsp_Init(void);
void bsp_DelayUS(uint32_t _ulDelayTime);
void BSP_Tick_Init (void);
uint32_t  BSP_CPU_ClkFreq (void);
static void NVIC_Configuration(void);
void SoftReset(void);
#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */

#define SD_FILESYSTEM_ENABLE   0

void DEBUG_LOG(const char *format, ...);
#endif
