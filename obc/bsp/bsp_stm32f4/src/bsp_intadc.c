/*
 * bsp_intadc.c
 *
 *  Created on: 2016年5月15日
 *      Author: Administrator
 */

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_intadc.h"

void ADC1_Init(void)
{
    ADC_CommonInitTypeDef   ADC_CommonInitStructure;
    ADC_InitTypeDef         ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

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

void int_adc_init(void)
{
    ADC1_Init();
}

static u16 Get_Adc1(u8 ch)
{
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );

    ADC_SoftwareStartConv(ADC1);

    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

    return ADC_GetConversionValue(ADC1);
}

static u16 Get_Adc_Average(u8 ch,u8 times)
{
    u32 temp_val=0;
    u8 t;
    for(t=0; t < times; t++)
    {
        temp_val += Get_Adc1(ch);
        vTaskDelay(5);
    }
    return temp_val/times;
}

short get_mcu_temp(void)
{
    short result;
    double temperate;

    result = Get_Adc_Average(ADC_Channel_16, 10);

    temperate = (float)result*(2.498/4096);

    temperate = (temperate-0.76) / 0.0025 + 25;

    result = temperate *= 100;

    return result;
}
