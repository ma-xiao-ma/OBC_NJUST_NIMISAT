#ifndef _BSP_IWDG_H
#define _BSP_IWDG_H


#include "stdint.h"

/** ����    : �������Ź���ʼ��*/
void bsp_InitIwdg(uint32_t _ulIWDGTime);

/** ����    : ι�������Ź�*/
void IWDG_Feed(void);

#endif
