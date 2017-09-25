/*
 * bsp_intadc.h
 *
 *  Created on: 2016年5月15日
 *      Author: Administrator
 */

#ifndef BSP_BSP_STM32F4_INC_BSP_INTADC_H_
#define BSP_BSP_STM32F4_INC_BSP_INTADC_H_

#include "stm32f4xx.h"

#define ADC1_HANDLE					1
#define ADC1_DR_ADDRESS    			((uint32_t)0x4001204C)
//#define DMA_ADC1_BUFFER_SIZE		7

#define ADC3_HANDLE					3
#define ADC3_DR_ADDRESS    			((uint32_t)0x4001224C)
//#define DMA_ADC3_BUFFER_SIZE		5

#define ADC_LENGTH				7
#define ADC_DELAY				1000

#define ADC3_OUT_D2_CHANNEL			ADC_Channel_4
#define ADC3_OUT3_CHANNEL			ADC_Channel_5
#define ADC3_OUT7_CHANNEL			ADC_Channel_6
#define ADC3_OUT4_CHANNEL			ADC_Channel_7
#define ADC3_OUT5_CHANNEL			ADC_Channel_8
#define IS_ADC3_CHANNEL(CHANNEL) 	(((CHANNEL) == ADC3_OUT_D2_CHANNEL) || \
                                 	 ((CHANNEL) == ADC3_OUT3_CHANNEL) || \
									 ((CHANNEL) == ADC3_OUT7_CHANNEL) || \
									 ((CHANNEL) == ADC3_OUT4_CHANNEL) || \
									 ((CHANNEL) == ADC3_OUT5_CHANNEL))

#define ADC1_OUT6_CHANNEL			ADC_Channel_10
#define ADC1_OUT2_CHANNEL			ADC_Channel_11
#define ADC1_OUT1_CHANNEL			ADC_Channel_12
#define ADC1_OUT_D1_CHANNEL			ADC_Channel_13
#define ADC1_OUT9_CHANNEL			ADC_Channel_14
#define ADC1_FB_M_CHANNEL			ADC_Channel_15
#define ADC1_CPU_CHANNEL			ADC_Channel_16
#define IS_ADC1_CHANNEL(CHANNEL) 	(((CHANNEL) == ADC1_OUT6_CHANNEL) || \
                                 	((CHANNEL) == ADC1_OUT2_CHANNEL) || \
									((CHANNEL) == ADC1_OUT1_CHANNEL) || \
									((CHANNEL) == ADC1_OUT_D1_CHANNEL) || \
									((CHANNEL) == ADC1_OUT9_CHANNEL) ||	\
									((CHANNEL) == ADC1_FB_M_CHANNEL) ||	\
									((CHANNEL) == ADC1_CPU_CHANNEL))

void int_adc_init(void);

void  ADC1_Init(void);

void  ADC3_Init(void);

int Get_Adc(uint8_t channel, uint16_t * data, uint32_t delay);

double Get_Temprate(void);

#endif /* BSP_BSP_STM32F4_INC_BSP_INTADC_H_ */
