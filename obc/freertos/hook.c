////////////////////////////////////////////////////////////////////////////////
//	功能： FreeRTOS操作系统回调函数
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2016.01.06
////////////////////////////////////////////////////////////////////////////////

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_reset.h"

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName ) {

	printf("STACK OVERFLOW!\r\n");
	printf("In task %p name: %s\r\n", pxTask, pcTaskName);
	lr = get_banked_lr();
	printf("NMI INTERRUPT: LR=%#0x\r\n", lr);

	volatile unsigned int i = 0xFFFF;
	while(i--);
	if (cpu_set_reset_cause)
		cpu_set_reset_cause(CPU_RESET_STACK_OVERFLOW);
	cpu_reset();

}
