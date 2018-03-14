////////////////////////////////////////////////////////////////////////////////
//	���ܣ� ʵ�ֱ���CPU��λԭ�򡢸�λǰPCֵ�븴λǰ���е�����
//
//	�汾��V1.0
//  ������
//												�Ͼ�����ѧ΢����������
//												   2016.01.06
////////////////////////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "core_cm4.h"

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"

#include "driver_debug.h"
#include "obc_argvs_save.h"

#include "bsp_reset.h"

#define STR_RESET_NONE				"CPU_RESET_NONE"
#define STR_RESET_USER				"CPU_RESET_USER"
#define STR_RESET_STACK_OVERFLOW	"CPU_RESET_STACK_OVERFLOW"
#define STR_RESET_HardFault			"CPU_RESET_HardFault"
#define STR_RESET_MemManage			"CPU_RESET_MemManage"
#define STR_RESET_BusFault			"CPU_RESET_BusFault"
#define STR_RESET_UsageFault		"CPU_RESET_UsageFault"
#define STR_RESET_DebugMon			"CPU_RESET_DebugMon"
#define STR_RESET_NMI				"CPU_RESET_NMI"

unsigned int __attribute__((section(".ram_persist"))) cpu_reset_cause;
int __attribute__((section(".ram_persist"))) lr;
char __attribute__((section(".ram_persist"))) TaskName[configMAX_TASK_NAME_LEN];

void reset_cause_print(int cause){
	switch(cause){
		case CPU_RESET_NONE:printf(/*driver_debug(DEBUG_RESET ,*/"RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_NONE, TaskName, lr);break;
		case CPU_RESET_USER:printf(/*driver_debug(DEBUG_RESET ,*/"RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_USER, TaskName, lr);break;
		case CPU_RESET_STACK_OVERFLOW:printf(/*driver_debug(DEBUG_RESET ,*/"RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_STACK_OVERFLOW, TaskName, lr);break;
		case CPU_RESET_HardFault:printf(/*driver_debug(DEBUG_RESET ,*/"RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_HardFault, TaskName, lr);break;
		case CPU_RESET_MemManage:printf(/*driver_debug(DEBUG_RESET ,*/"RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_MemManage, TaskName, lr);break;
		case CPU_RESET_BusFault:printf(/*driver_debug(DEBUG_RESET ,*/"RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_BusFault, TaskName, lr);break;
		case CPU_RESET_UsageFault:printf(/*driver_debug(DEBUG_RESET ,*/"RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_UsageFault, TaskName, lr);break;
		case CPU_RESET_DebugMon:printf(/*driver_debug(DEBUG_RESET ,*/"RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_DebugMon, TaskName, lr);break;
		case CPU_RESET_NMI:printf(/*driver_debug(DEBUG_RESET ,*/"RESET CAUSET: %s Task name: %s LR=%#0x\r\n", STR_RESET_NMI, TaskName, lr);break;
		default:break;
	}
}

void cpu_reset(void)
{

	if(xTaskGetCurrentTaskName(TaskName))
		xTaskGetCurrentTaskName(TaskName);

	reset_cause_print(cpu_reset_cause);

	obc_argvs_store();

	NVIC_SystemReset();

//	__asm volatile(
//		 "MOV R0, #1    	   \n"  //
//		 "MSR FAULTMASK, R0    \n"
//		 "LDR R0, =0xE000ED0C  \n"  //
//		 "LDR R1, =0x05FA0304  \n"  //
//		 "STR R1, [R0]         \n"  //绯荤粺杞欢澶嶄綅
//		 "LOOP:   B LOOP       \n" //姝诲惊鐜瓑寰匔PU澶嶄綅
//	);
}

void reset(void) {
	NVIC_SystemReset();
}

void user_cpu_reset(void) {

	asm volatile ("mov %[c], lr" : [c]"=r"(lr));

	if (cpu_set_reset_cause)
		cpu_set_reset_cause(CPU_RESET_USER);

	cpu_reset();
}

unsigned int get_banked_lr(void) {
	unsigned int ip;

	asm volatile ("mov %[c], lr" : [c]"=r"(lr));

	return ip - 4;
}

void cpu_set_reset_cause(cpu_reset_cause_t cause) {
	cpu_reset_cause = cause;
}

void NMI_Handler(void) __attribute__((naked));
void NMI_Handler(void)
{
	asm volatile
	(
		"	mrs r0, psp							\n"
		"	isb									\n"
		"	add r0, #20							\n"
		"	ldr r1, [r0]						\n"
		"	mov %[c], r1						\n"
		:[c]"=r"(lr)
	);

	portDISABLE_INTERRUPTS();

	if (cpu_set_reset_cause)
			cpu_set_reset_cause(CPU_RESET_NMI);

	cpu_reset();
}

void HardFault_Handler(void) __attribute__((naked));
void HardFault_Handler(void)
{
//	asm volatile ("mov %[c], lr" : [c]"=r"(lr));
	asm volatile
	(
		"	mrs r0, psp							\n"
		"	isb									\n"
		"	add r0, #20							\n"
		"	ldr r1, [r0]						\n"
		"	mov %[c], r1						\n"
		:[c]"=r"(lr)
	);

	portDISABLE_INTERRUPTS();

	if (cpu_set_reset_cause)
			cpu_set_reset_cause(CPU_RESET_HardFault);

	cpu_reset();
}

void MemManage_Handler(void) __attribute__((naked));
void MemManage_Handler(void)
{
//	asm volatile ("mov %[c], lr" : [c]"=r"(
	asm volatile
	(
		"	mrs r0, psp							\n"
		"	isb									\n"
		"	add r0, #20							\n"
		"	ldr r1, [r0]						\n"
		"	mov %[c], r1						\n"
		:[c]"=r"(lr)
	);

	portDISABLE_INTERRUPTS();

	if (cpu_set_reset_cause)
			cpu_set_reset_cause(CPU_RESET_MemManage);

	cpu_reset();
}

void BusFault_Handler(void) __attribute__((naked));
void BusFault_Handler(void)
{
//	asm volatile ("mov %[c], lr" : [c]"=r"(l
	asm volatile
	(
		"	mrs r0, psp							\n"
		"	isb									\n"
		"	add r0, #20							\n"
		"	ldr r1, [r0]						\n"
		"	mov %[c], r1						\n"
		:[c]"=r"(lr)
	);

	portDISABLE_INTERRUPTS();

	if (cpu_set_reset_cause)
			cpu_set_reset_cause(CPU_RESET_BusFault);

	cpu_reset();
}

void UsageFault_Handler(void) __attribute__((naked));
void UsageFault_Handler(void)
{
//	asm volatile ("mov %[c], lr" : [c]"=r"
	asm volatile
	(
		"	mrs r0, psp							\n"
		"	isb									\n"
		"	add r0, #20							\n"
		"	ldr r1, [r0]						\n"
		"	mov %[c], r1						\n"
		:[c]"=r"(lr)
	);

	portDISABLE_INTERRUPTS();

	if (cpu_set_reset_cause)
			cpu_set_reset_cause(CPU_RESET_UsageFault);

	cpu_reset();
}

void DebugMon_Handler(void) __attribute__((naked));
void DebugMon_Handler(void)
{
//	asm volatile ("mov %[c], lr" : [c]"=r"(lr));
	asm volatile
	(
		"	mrs r0, psp							\n"
		"	isb									\n"
		"	add r0, #20							\n"
		"	ldr r1, [r0]						\n"
		"	mov %[c], r1						\n"
		:[c]"=r"(lr)
	);

	portDISABLE_INTERRUPTS();

	if (cpu_set_reset_cause)
			cpu_set_reset_cause(CPU_RESET_DebugMon);

	cpu_reset();
}
