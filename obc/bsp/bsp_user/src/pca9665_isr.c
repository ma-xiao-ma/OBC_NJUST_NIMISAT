/*
 * pca9665_isr.c
 *
 *  Created on: 2017年9月21日
 *      Author: Ma Wenli
 */

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"
#include "bsp_pca9665.h"


/* Number of devices present on board */
const int pca9665_device_count = 2;

/* Setup of each device */
pca9665_device_object_t device[2] =
{
    {
        .base = (uint8_t *)FSMC_Bank1_SRAM1_ADDR,
        .is_initialised = 0,
        .is_busy = 0,
        .mode = DEVICE_MODE_M_T,
        .callback = NULL,
    },
    {
        .base = (uint8_t *)FSMC_Bank1_SRAM4_ADDR,
        .is_initialised = 0,
        .is_busy = 0,
        .mode = DEVICE_MODE_M_T,
        .callback = NULL,
    },
};

void EXTI0_IRQHandler(void) {

    uint32_t status_value;
    static portBASE_TYPE TaskWokenExit0;

    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        TaskWokenExit0 = pdFALSE;

        status_value = taskENTER_CRITICAL_FROM_ISR();
        pca9665_dsr(&TaskWokenExit0);
        taskEXIT_CRITICAL_FROM_ISR(status_value);

        portYIELD_FROM_ISR(TaskWokenExit0);
    }

    EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI1_IRQHandler(void) {

    uint32_t status_value;
    static portBASE_TYPE TaskWokenExit1;

    if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line1);

        TaskWokenExit1 = pdFALSE;

        status_value = taskENTER_CRITICAL_FROM_ISR();
        pca9665_dsr(&TaskWokenExit1);
        taskEXIT_CRITICAL_FROM_ISR(status_value);

        portYIELD_FROM_ISR(TaskWokenExit1);
    }
}

void pca9665_isr_init(void) {

    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //I2C0--PC0, I2C1--PC1
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    EXTI_ClearITPendingBit(EXTI_Line0);
    EXTI_ClearITPendingBit(EXTI_Line1);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);

    EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

