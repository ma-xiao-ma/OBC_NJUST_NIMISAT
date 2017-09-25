/*
*********************************************************************************************************
*
*	ģ������ : RTC
*	�ļ����� : bsp_rtc.c
*	��    �� : V1.0
*	˵    �� : RTC�ײ�����
*	
*
*********************************************************************************************************
*/

#ifndef __BSP_RTC_H
#define __BSP_RTC_H

#include <time.h>
#include "stm32f4xx_rtc.h"
#include "stdint.h"

 RTC_TimeTypeDef   CurTime;                 /* ��ǰ����ʱ�� */
 RTC_DateTypeDef   CurDate;                 /* ��ǰ�������� */
 uint32_t          CurUTCTime;                 /* ��ǰ����ʱ��UTC��ʽ */

 struct OBCBootInfoStr
 {
 	char BootCnt;             /* ���������������� */
 	char BootPINRSTCnt;       /* �����������Ÿ�λ���� */
 	char BootPORRSTCnt;       //
 	char BootSFTRSTCnt;       /* �������������λ���� */
 	char BootIWDGRSTCnt;      /* ���������������Ź���λ���� */
 	char BootWWDGRSTCnt;      /* �����������ڿ��Ź���λ���� */
 	char BootLPWRRSTCnt;
 	/*******************/
 	char BootRTC_Source;      /* RTCʱ��Դ��0�ⲿ����1�ڲ�����2û������*/
 	char BootLSE_Error;       /* �ⲿʱ�Ӵ������ */
 	char BootLSI_Error;       /* �ڲ�ʱ�Ӵ������ */
 };

 struct OBCBootInfoStr OBCBootInfo;         /* OBC����״̬��Ϣ */


void bsp_InitRTC(void);
void RTC_Config(void);
void bsp_RTCSet(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
void bsp_RTCTimeGet(time_t * time);

#endif

