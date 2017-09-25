////////////////////////////////////////////////////////////////////////////////
//	���ܣ� ʵ��LED��ʼ����������ز���ͷ�ļ�
//
//	�汾��V1.0
//  ������
//												�Ͼ�����ѧ΢����������
//												   2015.11.05
////////////////////////////////////////////////////////////////////////////////

#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "stdint.h"
#include "stm32f4xx.h"


#define GPIO_WatchDog 			(RCC_AHB1Periph_GPIOB)	// LED��Ӧ��RCCʱ��
#define GPIO_PORT_WatchDog  	GPIOB 				//LED Port
#define GPIO_PIN_WatchDog		GPIO_Pin_5			//LED Pin



void bsp_InitWatchDog(void); 		//LED GPIO��ʼ��
void bsp_WatchDogOn(void);			//LED ��
void bsp_WatchDogOff(void);			//LED ��
void bsp_WatchDogToggle(void);		//LED ״̬��ת
uint8_t bsp_IsWatchDogOn(void);		//�ж�LED�Ƿ��

#endif
