////////////////////////////////////////////////////////////////////////////////
//	���ܣ� ��λ����ͷ�ļ�
//
//	�汾��V1.0
//  ������
//												�Ͼ�����ѧ΢����������
//												   2016.01.06
////////////////////////////////////////////////////////////////////////////////

#ifndef __BSP_RESET_H
#define __BSP_RESET_H

#include "FreeRTOSConfig.h"

//////////////////////////////////////////
///��λԭ��ö��
//////////////////////////////////////////
typedef enum {
	CPU_RESET_NONE = 0,
	CPU_RESET_USER = 1,
	CPU_RESET_STACK_OVERFLOW,
	CPU_RESET_HardFault,
	CPU_RESET_MemManage,
	CPU_RESET_BusFault,
	CPU_RESET_UsageFault,
	CPU_RESET_DebugMon,
	CPU_RESET_NMI,
} cpu_reset_cause_t;

//////////////////////////////////////////
///��λԭ�򱣴����
//////////////////////////////////////////
extern unsigned int __attribute__((section(".ram_persist"))) cpu_reset_cause;
extern int __attribute__((section(".ram_persist"))) lr;
extern char __attribute__((section(".ram_persist"))) TaskName[10];

void cpu_reset(void);
void reset(void);
void user_cpu_reset(void);
unsigned int get_banked_lr(void);
void cpu_set_reset_cause(cpu_reset_cause_t cause);

#endif /* __BSP_RESET_H */
