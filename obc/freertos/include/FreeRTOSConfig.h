/*
    FreeRTOS V8.0.0 - Copyright (C) 2014 Real Time Engineers Ltd. 
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>! NOTE: The modification to the GPL is included to allow you to distribute
    >>! a combined work that includes FreeRTOS without being obliged to provide
    >>! the source code for proprietary components outside of the FreeRTOS
    >>! kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

/* Ensure stdint is only used by the compiler, and not the assembler. */
/*#ifdef __ICCARM__
	#include <stdint.h>
#endif*/

#include "bsp_reset.h"
#include "bsp_timer.h"

#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
    #include <stdint.h>
    extern uint32_t SystemCoreClock;
#endif

#define MEMORY_MANAGE_MODE_2

/* 操作系统断言定义 */
#define vAssertCalled(char,int)                         \
{                                                       \
    printf("FreeRTOS Error:%s,%d\r\n",char,int);        \
    taskDISABLE_INTERRUPTS();                           \
    cpu_reset();                                        \
}

#define configASSERT( x ) if( ( x ) == 0 ) vAssertCalled(__FILE__,__LINE__)

/******************************************************************************/
/*                            FreeRTOS基础配置选项                                                                              */
/******************************************************************************/
#define configUSE_PREEMPTION			            1 //使用抢占式内核，0使用协程
#define configUSE_TIME_SLICING                      1 //1使能时间片调度（默认是使能的）
 /* 使用特殊方法选择下一个要运行的任务，一般是硬件计算机前导零指令 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION     1  //2017-07-19 add by Mr.ROBOT
#define configUSE_TICKLESS_IDLE                     0  //1启用低功耗tickless模式
#define configUSE_QUEUE_SETS                        1  //为1时启用队列
#define configCPU_CLOCK_HZ				            ( SystemCoreClock ) //CPU频率
/* 时钟节拍频率，这里设置为1000，周期就是1ms */
#define configTICK_RATE_HZ				            ( ( TickType_t ) 1000 )
/* 可使用的最大优先级 */
#define configMAX_PRIORITIES			            ( 6 )
/* 空闲任务使用的堆栈大小 */
#define configMINIMAL_STACK_SIZE		            ( ( unsigned short ) 256 )
/* 任务名字字符串长度 */
#define configMAX_TASK_NAME_LEN			            ( 10 )
/* 系统节拍计数器变量数据类型，1表示16位无符号整形，0表示为32位无符号整形 */
#define configUSE_16_BIT_TICKS			            0
/* 为1时空闲任务放弃CPU使用权给其他同优先级的用户任务 */
#define configIDLE_SHOULD_YIELD			            1
/* 为1时开启任务通知功能，默认开启 */
#define configUSE_TASK_NOTIFICATIONS                1
/* 为1时使用互斥信号量 */
#define configUSE_MUTEXES				            1
/* 不为0时表示启用队列记录，具体的值是可以记录的队列和信号量最大数目 */
#define configQUEUE_REGISTRY_SIZE		            8
/* 大于0时启用堆栈溢出检测功能，如果使用此功能，用户必须提供一个栈溢出钩子函数，如果使用的话此值可以为1
 * 或者2，因为有两种栈溢出检测方法 */
#define configCHECK_FOR_STACK_OVERFLOW	            1

#define configUSE_RECURSIVE_MUTEXES		            1 //为1时使用递归互斥信号量
#define configUSE_MALLOC_FAILED_HOOK	            0 //为1时使用内存失败钩子函数
#define configUSE_APPLICATION_TASK_TAG	            0
#define configUSE_COUNTING_SEMAPHORES	            1 //为1时使用计数信号量
/******************************************************************************/
/*                            FreeRTOS内存申请有关配置选项                                                               */
/******************************************************************************/
#define configSUPPORT_DYNAMIC_ALLOCATION            1 //支持动态申请内存
    /* 系统支持总堆大小 */
#define configTOTAL_HEAP_SIZE                       ((size_t)(32 * 1024))

/******************************************************************************/
/*                             FreeRTOS与钩子函数有关配置选项                                                        */
/******************************************************************************/
#define configUSE_IDLE_HOOK                         0
#define configUSE_TICK_HOOK                         0

/******************************************************************************/
/*                   FreeRTOS与运行时间和任务状态收集有关的配置选项                                                   */
/******************************************************************************/
#define configGENERATE_RUN_TIME_STATS	            1   //为1时启用时间统计功能
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()    ConfigureTimeForRunTimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE()            FreeRTOSRunTimeTicks

#define configUSE_TRACE_FACILITY                    1   //为1启用可视化跟踪调试
#define configUSE_STATS_FORMATTING_FUNCTIONS        1   //与宏configUSE_TRACE_FACILITY
                                                        //同时为1时会编译下面三个函数
                                                        //prvWriteNameToBuffer(),
                                                        //vTaskList(),
                                                        //vTaskGetRunTimeStats()
/******************************************************************************/
/*                          FreeRTOS与协程有关的配置选项                                                                    */
/******************************************************************************/
#define configUSE_CO_ROUTINES       0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/******************************************************************************/
/*                          FreeRTOS与软件定时器有关的配置选项                                                        */
/******************************************************************************/
#define configUSE_TIMERS                0
#define configTIMER_TASK_PRIORITY       ( 2 )
#define configTIMER_QUEUE_LENGTH        10
#define configTIMER_TASK_STACK_DEPTH    ( configMINIMAL_STACK_SIZE * 2 )

/******************************************************************************/
/*                              FreeRTOS可选函数配置选项                                                                 */
/******************************************************************************/
#define INCLUDE_xTaskGetSchedulerState          0
#define INCLUDE_vTaskPrioritySet		        1
#define INCLUDE_uxTaskPriorityGet		        1
#define INCLUDE_vTaskDelete				        1
#define INCLUDE_vTaskCleanUpResources	        0
#define INCLUDE_vTaskSuspend			        1
#define INCLUDE_vTaskDelayUntil			        1
#define INCLUDE_vTaskDelay				        1
#define INCLUDE_eTaskGetState                   0
#define INCLUDE_xTimerPendFunctionCall          0

/******************************************************************************/
/*                           FreeRTOS与中断有关的配置选项                                                                 */
/******************************************************************************/
#ifdef __NVIC_PRIO_BITS
	/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
	#define configPRIO_BITS       		__NVIC_PRIO_BITS
#else
	#define configPRIO_BITS       		4        /* 15 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			0xf //中断最低优先级

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	5 //系统可管理的最高中断优先级

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
	
/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
/******************************************************************************/
/*                           FreeRTOS与中断有关的配置选项                                                                 */
/******************************************************************************/
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler


#endif /* FREERTOS_CONFIG_H */

