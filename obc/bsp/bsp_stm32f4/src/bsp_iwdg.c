/*
 * bsp_iwdg.c
 *
 *  Created on: 2017年11月7日
 *      Author: Ma Wenli
 */
#include "bsp_iwdg.h"


/**
 * 初始化独立看门狗
 *
 * @param timeout 看门狗超时值设置，单位ms。 8ms ~ 32760ms
 */
void IWDG_Init(u16 timeout)
{
    IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable ); //使能对IWDG->PR IWDG->RLR的写

    IWDG_SetPrescaler( IWDG_Prescaler_256 ); //设置IWDG分频系数

    IWDG_SetReload( timeout / 8 );   //设置IWDG装载值

    IWDG_ReloadCounter(); //reload

    IWDG_Enable();       //使能看门狗
}

/**
 * 喂狗
 *
 */
void IWDG_Feed(void)
{
    IWDG_ReloadCounter();//reload
}
