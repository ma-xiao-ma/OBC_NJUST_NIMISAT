////////////////////////////////////////////////////////////////////////////////
//	鍔熻兘锛� 鍗槦骞冲彴閫氱敤閰嶇疆瀹氫箟澶存枃浠�
//
//	鐗堟湰锛歏1.0
//  杩唬锛�
//												鍗椾含鐞嗗伐澶у寰撼鍗槦涓績
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_CUBESAT_H_
#define INCLUDE_CUBESAT_H_


#define CONSOLE COM3//4//2 //1//2		//


#include <stdint.h>
#include <stddef.h>

#define CONSOLE_HISTORY_ENABLE 1
#define CONSOLE_HISTORY_ELEMENTS 10
#define CONFIG_DRIVER_DEBUG 1

#define configTOTAL_QB50_HEAP_SIZE			( ( size_t ) ( 32768 ) )
#define portBYTE_ALIGNMENT_QB50				8
#define portBYTE_ALIGNMENT_QB50_MASK		0x07
#define configADJUSTED_QB50_HEAP_SIZE		( configTOTAL_QB50_HEAP_SIZE - portBYTE_ALIGNMENT_QB50 )
#define portPOINTER_SIZE_TYPE_QB50			uint32_t

#endif
