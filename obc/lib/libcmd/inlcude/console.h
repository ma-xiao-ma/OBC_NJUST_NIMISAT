////////////////////////////////////////////////////////////////////////////////
//	功能： 调试终端功能模块头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////

#ifndef LIB_LIBCMD_INLCUDE_CONSOLE_H_
#define LIB_LIBCMD_INLCUDE_CONSOLE_H_

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>


#define CONSOLE_LENGTH 1000		//队列块数

#define CONTROL(X)  ((X) - '@')

#define CONSOLE_NORMAL		0
#define CONSOLE_ESCAPE		1
#define CONSOLE_PRE_ESCAPE	2


void Console_Usart_init(uint32_t baudrate);
void debug_console(void *pvParameters __attribute__((unused)));

#endif

