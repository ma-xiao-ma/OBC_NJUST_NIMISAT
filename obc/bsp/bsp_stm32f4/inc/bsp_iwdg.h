/*
 * bsp_iwdg.h
 *
 *  Created on: 2017年11月7日
 *      Author: Ma Wenli
 */

#ifndef BSP_BSP_STM32F4_INC_BSP_IWDG_H_
#define BSP_BSP_STM32F4_INC_BSP_IWDG_H_

#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"

/**
 * 初始化独立看门狗
 *
 * @param timeout 看门狗超时值设置，单位ms。 8ms ~ 32760ms
 */
void IWDG_Init(u16 timeout);

/**
 * 喂狗
 *
 */
void IWDG_Feed(void);

#endif /* BSP_BSP_STM32F4_INC_BSP_IWDG_H_ */
