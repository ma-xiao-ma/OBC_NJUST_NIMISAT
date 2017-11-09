/*
 * obc_mem.h
 *
 *  Created on: 2017年11月9日
 *      Author: Ma Wenli
 */

#ifndef CONTRL_OBC_MEM_H_
#define CONTRL_OBC_MEM_H_

#include "stddef.h"
/**
 * 内存申请函数
 *
 * @param xWantedSize 申请内存大小       返回空指针则说明申请内存失败
 */
void *ObcMemMalloc( size_t xWantedSize );

/**
 * 内存释放函数
 *
 * @param pv 待释放内存指针
 */
void ObcMemFree( void *pv );

/**
 * 查询内存堆剩余大小值
 *
 * @return 内存堆剩余大小值
 */
size_t ObcMemGetFreeHeapSize( void );

/**
 * 查询历史最小空闲内存块大小
 *
 * @return 历史最小空闲内存块大小
 */
size_t ObcMemGetMinimumEverFreeHeapSize( void );

#endif /* CONTRL_OBC_MEM_H_ */
