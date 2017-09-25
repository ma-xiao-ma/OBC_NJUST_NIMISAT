/*
 * QB50_mem.h
 *
 *  Created on: 2016年4月29日
 *      Author: Administrator
 */

#ifndef CONTRL_QB50_MEM_H_
#define CONTRL_QB50_MEM_H_

#include "cubesat.h"

void *qb50Malloc( size_t xWantedSize );
void qb50Free( void *pv );
size_t qb50GetFreeHeapSize( void );

#endif /* CONTRL_QB50_MEM_H_ */
