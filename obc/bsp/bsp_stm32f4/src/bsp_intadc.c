/*
 * bsp_intadc.c
 *
 *  Created on: 2016年5月15日
 *      Author: Administrator
 */

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "bsp_intadc.h"

static __IO uint16_t ADC1ConvertedValue = 0;
static __IO uint16_t ADC3ConvertedValue = 0;
static xQueueHandle  adc1_queue;
static xQueueHandle  adc3_queue;

void int_adc_init(void) {
//	adc_isr_init();
	ADC1_Init();
//	ADC3_Init();
}

//void ADC1_Init(void)
//{
////	GPIO_InitTypeDef  		GPIO_InitStructure;
//	ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
//	ADC_InitTypeDef       	ADC_InitStructure;
//	DMA_InitTypeDef       	DMA_InitStructure;
//
//	adc1_queue = xQueueCreate(ADC_LENGTH,2);
//
//	/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 /*| RCC_AHB1Periph_GPIOC*/, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
//
//	/* DMA2 Stream0 channel2 configuration **************************************/
//	DMA_DeInit(DMA2_Stream0);
//	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
//	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1ConvertedValue;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	DMA_InitStructure.DMA_BufferSize = 1;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
//	DMA_Cmd(DMA2_Stream0, ENABLE);
//
////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
////	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
////	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
////	GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);	  //reset
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);	//complete reset
//
//	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
//	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles;
//	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
//	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
//	ADC_CommonInit(&ADC_CommonInitStructure);
//
//	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
//	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
//	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
//	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//	ADC_InitStructure.ADC_NbrOfConversion = 1;
//	ADC_Init(ADC1, &ADC_InitStructure);
//
//	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
//
////	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
//
//	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
//	ADC_DMACmd(ADC1, ENABLE);
//
//	ADC_Cmd(ADC1, ENABLE);
//
//	ADC_RegularChannelConfig(ADC1, ADC1_CPU_CHANNEL, 1, ADC_SampleTime_15Cycles );
//
//	ADC_TempSensorVrefintCmd(ENABLE);
//
//	ADC_SoftwareStartConv(ADC1);
//}



void  ADC3_Init(void)
{
	GPIO_InitTypeDef  		GPIO_InitStructure;
	ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
	ADC_InitTypeDef       	ADC_InitStructure;

	adc3_queue = xQueueCreate(ADC_LENGTH,2);

	/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,ENABLE);	  //reset
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,DISABLE);	//complete reset

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;

	ADC_Init(ADC3, &ADC_InitStructure);

	ADC_ClearFlag(ADC3, ADC_FLAG_EOC);

	ADC_ITConfig(ADC3,ADC_IT_EOC,ENABLE);

//	ADC_Cmd(ADC3, ENABLE);
}

void adc_isr_init(void) {
	NVIC_InitTypeDef 		NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;                                                               //ADC1,ADC2全局中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;//先占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   //从优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}

void ADC_IRQHandler(void) {

	portBASE_TYPE xTaskWoken = pdFALSE;

	vPortEnterCritical();

	if(ADC_GetITStatus(ADC1, ADC_IT_EOC)) {
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

		ADC_Cmd(ADC1, DISABLE);

//		ADC1ConvertedValue = ADC_GetConversionValue(ADC1);

		xQueueSendToBackFromISR(adc1_queue, &ADC1ConvertedValue, &xTaskWoken);
	}
	if(ADC_GetITStatus(ADC3, ADC_IT_EOC)) {
		ADC_ClearITPendingBit(ADC3, ADC_IT_EOC);

		ADC_Cmd(ADC3, DISABLE);

		ADC3ConvertedValue = ADC_GetConversionValue(ADC3);

		xQueueSendToBackFromISR(adc3_queue, &ADC3ConvertedValue, &xTaskWoken);
	}

	vPortExitCritical();

	portYIELD_FROM_ISR(xTaskWoken);
}

int Get_Adc(uint8_t channel, uint16_t * data, uint32_t delay)
{
//	int cnt = 500000;
//	uint16_t ADCConvertedValue = 0;
//
//	if(IS_ADC1_CHANNEL(channel)) {
//
//		ADC_Cmd(ADC1, ENABLE);
//
//		ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_15Cycles );
//
//		if(channel == ADC1_CPU_CHANNEL) {
//			ADC_TempSensorVrefintCmd(ENABLE);
//		}
//
//		ADC_SoftwareStartConv(ADC1);
//
//		if(xQueueReceive(adc1_queue, data, delay) != pdPASS) {
//			if(channel == ADC1_CPU_CHANNEL) {
//				ADC_TempSensorVrefintCmd(DISABLE);
//			}
//
//			return -1;
//		}
//
//		if(channel == ADC1_CPU_CHANNEL) {
//			ADC_TempSensorVrefintCmd(DISABLE);
//		}
//
//	} else if(IS_ADC3_CHANNEL(channel)) {
//
//		ADC_Cmd(ADC3, ENABLE);
//
//		ADC_RegularChannelConfig(ADC3, channel, 1, ADC_SampleTime_15Cycles );
//
//		ADC_SoftwareStartConv(ADC3);
//
//		if(xQueueReceive(adc3_queue,data,delay) != pdPASS) {
//			return -1;
//		}
//	}
//
//	return ADCConvertedValue;

	return *data = ADC1ConvertedValue;
}

//int Get_Adc(uint8_t channel)
//{
//	int cnt = 500000;
//	uint16_t ADCConvertedValue = 0;
//
//	if(IS_ADC1_CHANNEL(channel)) {
//
//		ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_15Cycles );
//
//		if(channel == ADC1_CPU_CHANNEL) {
//			ADC_TempSensorVrefintCmd(ENABLE);
//		}
//
//		ADC_SoftwareStartConv(ADC1);
//
//		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)&&cnt--);
//
//		ADCConvertedValue = ADC1ConvertedValue;
//
//		ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
//
//	} else if(IS_ADC3_CHANNEL(channel)) {
//
//		ADC_RegularChannelConfig(ADC3, channel, 1, ADC_SampleTime_15Cycles );
//
//		ADC_SoftwareStartConv(ADC3);
//
//		while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC )&&cnt--);
//
//		ADC_ClearFlag(ADC3, ADC_FLAG_EOC);
//
//		ADCConvertedValue = ADC_GetConversionValue(ADC3);
//	}
//
//	if(cnt <= 0) {
//		return -1;
//	}
//
//	return ADCConvertedValue;
//}

void ADC1_Init(void)
{
//  GPIO_InitTypeDef        GPIO_InitStructure;
    ADC_CommonInitTypeDef   ADC_CommonInitStructure;
    ADC_InitTypeDef         ADC_InitStructure;

//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

//  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);      //reset
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);   //complete reset

    ADC_TempSensorVrefintCmd(ENABLE);

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_480Cycles );

    ADC_Cmd(ADC1, ENABLE);
}

u16 Get_Adc1(u8 ch)
{

    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );
    ADC_SoftwareStartConv(ADC1);

    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));

    return ADC_GetConversionValue(ADC1);
}

u16 Get_Adc_Average(u8 ch,u8 times)
{
    u32 temp_val=0;
    u8 t;
    for(t=0;t<times;t++)
    {
        temp_val+=Get_Adc1(ch);
        delay_ms(5);
    }
    return temp_val/times;
}

double Get_Temprate(void)
{
    u32 adcx;

    double temperate;
    adcx=Get_Adc_Average(ADC_Channel_16,10);
    temperate=(float)adcx*(2.05/4096);
    temperate=(temperate-0.76)/0.0025 + 25;

    return temperate;
}
