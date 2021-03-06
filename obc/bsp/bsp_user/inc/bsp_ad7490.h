////////////////////////////////////////////////////////////////////////////////
//	功能： 电源板AD模块头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.18
////////////////////////////////////////////////////////////////////////////////


#ifndef _AD7490_H_
#define _AD7490_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#define WRITE               0x8000
#define NO_WRITE            0x0000
/*********************************/
#define SEQ_NORMAL          0x0000
#define SEQ_CFG             0x0080
#define SEQ_UPDATE          0x4000

#define SHADOW				0x0008
/*********************************/
#define CHANNEL_0           0x0000
#define CHANNEL_1           0x0400
#define CHANNEL_2           0x0800
#define CHANNEL_3           0x0C00
#define CHANNEL_4           0x1000
#define CHANNEL_5           0x1400
#define CHANNEL_6           0x1800
#define CHANNEL_7           0x1C00
#define CHANNEL_8           0x2000
#define CHANNEL_9           0x2400
#define CHANNEL_10          0x2800
#define CHANNEL_11          0x2C00
#define CHANNEL_12          0x3000
#define CHANNEL_13          0x3400
#define CHANNEL_14          0x3800
#define CHANNEL_15          0x3C00
/*********************************/
#define POWER_NORMAL        0x0300
#define POWER_FULLDOWN      0x0200
#define POWER_AUTODOWN      0x0100
#define POWER_AUTOSTAN      0x0000
/*********************************/
#define RANGE_NORMAL        0x0020
#define RANGE_DOUBLE        0x0000
/*********************************/
#define DATA_BIN            0x0010
#define DATA_TWOS           0x0000
/*********************************/
#define ALL_CHANNEL         0xFFFF
#define DATA_UPDATE         0x0000
/*********************************/
#define AD_MODE_SEQ         0x54
#define AD_MODE_SINGLE      0x45

#define ADC_REF        	2500   //ADC�ο���ѹ
#define ADC_FULL_SCALE  4095  //ADC����

void AD7490_Init(void);

uint16_t *AD7490_Read_NoIntSeq_own(void);

void AD7490_Read(void);

uint16_t AD_SendByte_own(uint16_t _ucValue);

#endif
