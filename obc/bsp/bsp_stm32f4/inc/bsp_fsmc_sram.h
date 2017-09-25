/*
*********************************************************************************************************
*
*	ģ������ : �ⲿSRAM����ģ��
*	�ļ����� : bsp_fsmc_sram.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v1.0    2012-10-12 armfly  ST�̼���汾 V2.1.0
*
*	
*
*********************************************************************************************************
*/

#ifndef _BSP_FSMC_SRAM_H
#define _BSP_FSMC_SRAM_H

#include "stm32f4xx.h"
#include <stdint.h>

#define EXT_SRAM_ADDR  	((uint32_t)0x68000000)
#define EXT_SRAM_SIZE	(2 * 1024 * 1024)

#define SRAM_WRITE(addr,data) 	     (*(__IO uint32_t *)(EXT_SRAM_ADDR+addr) = data)
#define SRAM_READ(addr)				 (*(__IO uint32_t *)(EXT_SRAM_ADDR+addr))

void bsp_InitExtSRAM(void);
uint8_t bsp_TestExtSRAM(void);

#endif
