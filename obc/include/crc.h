/*
 * crc.h
 *
 *  Created on: 2016年5月14日
 *      Author: Administrator
 */

#ifndef SRC_CRC_H_
#define SRC_CRC_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "integer.h"

uint32_t crc32_memory(const uint8_t * data, uint32_t length);
uint16_t crc_citt_value(uint8_t *message,  int len);
uint8_t sum_check(uint8_t *pdata, uint32_t length);

#endif /* SRC_CRC_H_ */
