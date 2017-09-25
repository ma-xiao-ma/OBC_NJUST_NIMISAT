#include "bsp_iwdg.h"

#include "stdio.h"
#include "stdlib.h"
#include "stm32f4xx.h"
//#include "globalavr.h"

__IO uint32_t LsiFreq = 0;
__IO uint32_t CaptureNumber = 0, PeriodValue = 0;

uint32_t GetLSIFrequency(void);
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitIwdg
*	����˵��: �������Ź�ʱ�����ú���
*	��    �Σ�IWDGTime: 0 ---- 0x0FFF
*			  �������Ź�ʱ������,��λΪms,IWDGTime = 1000 ��Լ����һ���
*             ʱ�䣬����û�н��TIM5���ʵ��LSIƵ�ʣ�ֻ��Ϊ�˲�������ȡ��
*             һ������ֵ����IWDGTime������ι���Ļ�ϵͳ���Ḵλ��
*			  LSI = 34000����
*	�� �� ֵ: ��		        
*********************************************************************************************************
*/
void bsp_InitIwdg(uint32_t _ulIWDGTime)
{
		
	/* ���ϵͳ�Ƿ�Ӷ������Ź���λ�лָ� */
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
	{		
//		 OBCBootInfo.BootIWDGRSTCnt++;
		/* �����λ��־ */
		RCC_ClearFlag();
	}
	else
	{
		/* ��־û������ */

	}
	
#if 0
	/* ͨ��TIM5���벶��õ�LSIƵ�� */
	LsiFreq = GetLSIFrequency();
#else
	/* ʹ��LSI */
	RCC_LSICmd(ENABLE);
	
	/* �ȴ�ֱ��LSI���� */
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{}
	
    /* */		
	LsiFreq = 32000;
#endif
	
	/* д��0x5555��ʾ�������IWDG_PR ��IWDG_RLR�Ĵ��� */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	
	/*  LSI/32 ��Ƶ*/
	IWDG_SetPrescaler(IWDG_Prescaler_32);
	
	/*�ر�ע�⣬��������_ulIWDGTime����С��λ��ms, ����������װ������
	  ����ʱ ��Ҫ����1000
	 Counter Reload Value = (_ulIWDGTime / 1000) /(1 / IWDG counter clock period)
	                      = (_ulIWDGTime / 1000) / (32/LSI)
	                      = (_ulIWDGTime / 1000) / (32/LsiFreq)
	                      = LsiFreq * _ulIWDGTime / 32000
	 ʵ�ʲ���LsiFreq = 34000����������ȡ1��ʱ�� ��ž���1ms 
	*/
	IWDG_SetReload(_ulIWDGTime);
	
	/* ����IWDG���� */
	IWDG_ReloadCounter();
	
	/* ʹ�� IWDG (LSI oscillator ��Ӳ��ʹ��) */
	IWDG_Enable();		
}

/*
*********************************************************************************************************
*	�� �� ��: IWDG_Feed
*	����˵��: ι������
*	��    �Σ���
*	�� �� ֵ: ��		        
*********************************************************************************************************
*/
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();
}

/*
*********************************************************************************************************
*	�� �� ��: GetLSIFrequency
*	����˵��: ����TIM5��ͨ��4����LSIƵ��
*	��    �Σ���
*	�� �� ֵ: LSI Frequency	 ʵ�ʲ���LSI = 34000����,�ֲ�������ķ�Χ��	        
*********************************************************************************************************
*/
uint32_t GetLSIFrequency(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	RCC_ClocksTypeDef  RCC_ClockFreq;
	
	/* ʹ���ⲿ���پ��� */
	RCC_LSICmd(ENABLE);
	
	/* �ȴ�ֱ��LSI���� */
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{}
	
	/* TIM5 ���� */ 
	/* ʹ��TIM5ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
	/* ��TIM5_CH4���벶��ͨ�����ӵ�LSIʱ����� */
	TIM_RemapConfig(TIM5, TIM5_LSI);
	
	/* ����TIM5��Ƶ */
	TIM_PrescalerConfig(TIM5, 0, TIM_PSCReloadMode_Immediate);
	
	/* 
	 TIM5 ����: ���벶��ģʽ 
	 1. LSI �������ӵ� TIM5 CH4
	 2. ��������Ϊ����ı���
	 3. TIM5 CCR4���ڼ���Ƶ��ֵ 
	 4. ������Ҫ��Ƶ����TIM_ICPSC_DIV8��������˼����TIM��8��Ƶ
		������˼��ÿ8��ʱ�䲶��һ�Σ�����ͱ�ʾ8���������Ժ���
		һ�β�������жϣ�Ȼ���ٹ�8�������ز���һ�ν����жϣ�����
		���ж��Ժ�õ�ʱ������Ҳ����8�����ڣ��������õ��Ľ��
		�ǷŴ���8����
    */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	
	/* ʹ��TIM5�ж�ͨ�� */
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* ʹ��TIM5���� */
	TIM_Cmd(TIM5, ENABLE);
	
	/* �����־ */
	TIM5->SR = 0;
	
	/* ʹ�� CC4 �ж����� */  
	TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);
	
	
	/* �ȴ�ֱ�� TIM5 �õ� 2 ��LSI ���� */
	while(CaptureNumber != 2)
	{
		
	}
	
	/* �ض���TIM5����λֵ */
	TIM_DeInit(TIM5);
		
	/* ���� LSI Ƶ��, ����TIM5����ʱ��Ƶ��������(PCLK1)*/
	/* �õ� SYSCLK, HCLK �� PCLKx  */
	RCC_GetClocksFreq(&RCC_ClockFreq);
	
	/*
	  HCLK = SYSCLK / 1     (AHB1Periph)     = 168MHz
      PCLK2 = HCLK / 2      (APB2Periph)      = 84MHz
      PCLK1 = HCLK / 4      (APB1Periph)      = 42MHz
    
      ��ΪAPB1 prescaler != 1, ���� APB1�ϵ�TIMxCLK = PCLK1 x 2 = SystemCoreClock / 2;
      ��ΪAPB2 prescaler != 1, ���� APB2�ϵ�TIMxCLK = PCLK2 x 2 = SystemCoreClock;

      APB1 ��ʱ���� TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM6, TIM12, TIM13,TIM14
      APB2 ��ʱ���� TIM1, TIM8 ,TIM9, TIM10, TIM11
	*/
	if ((RCC->CFGR & RCC_CFGR_PPRE1) == 0)
	{   
		/* ���PCLK1 prescaler == 1 ��ô TIMCLK = PCLK1 */
		return ((RCC_ClockFreq.PCLK1_Frequency / PeriodValue)*8);
		
	}
	else
	{   
		/* 
		  ���PCLK1 prescaler ������ 1 ��ô TIMCLK = 2 * PCLK1
		  �������ó���8����Ϊǰ������ÿ8���¼�����һ�β���
		*/
		return (((2 * RCC_ClockFreq.PCLK1_Frequency) / PeriodValue)*8) ;
		
	}
}




