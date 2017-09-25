////////////////////////////////////////////////////////////////////////////////
//	功能： 串口功能头文件
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////
#ifndef __USARTCOMM_H
#define __USARTCOMM_H


#include "stdint.h"
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"

#define USART_PORT       6
#define MAX_USART_BUFFSIZE 1024


/// 定义端口号
typedef enum
{
	COM1 = 0,	/* USART1  PA9, PA10 或  PB6, PB7*/
	COM2 = 1,	/* USART2, PD5,PD6 或 PA2, PA3 */
	COM3 = 2,	/* USART3, PB10, PB11 */
	COM4 = 3,	/* UART4, PC10, PC11 */
	COM5 = 4,	/* UART5, PC12, PD2 */
	COM6 = 5	/* USART6, PC6, PC7 */
}COM_PORT_TypeDef;

//////////////////////////////////////////
///USART1 引脚及中断配置
//////////////////////////////////////////
#define USART1_PORT_NAME		USART1
#define USART1_TX_PORT          GPIOB//GPIOA//GPIOB
#define USART1_RX_PORT          GPIOB
#define USART1_TX_PORTCLK       RCC_AHB1Periph_GPIOB//RCC_AHB1Periph_GPIOA//RCC_AHB1Periph_GPIOB
#define USART1_RX_PORTCLK       RCC_AHB1Periph_GPIOB
#define USART1_USCLK			RCC_APB2Periph_USART1
#define USART1_RX_PIN          	GPIO_Pin_7
#define USART1_TX_PIN          	GPIO_Pin_6//GPIO_Pin_9//GPIO_Pin_6
#define USART1_EXTI_IRQn     	USART1_IRQn 
#define USART1_TX_AF         	GPIO_AF_USART1
#define USART1_TX_SOURCE     	GPIO_PinSource6//GPIO_PinSource9//GPIO_PinSource6
#define USART1_RX_AF         	GPIO_AF_USART1
#define USART1_RX_SOURCE     	GPIO_PinSource7

//////////////////////////////////////////
///USART2 引脚及中断配置
//////////////////////////////////////////
#define USART2_PORT_NAME		USART2
#define USART2_TX_PORT          GPIOA
#define USART2_RX_PORT         	GPIOA
#define USART2_TX_PORTCLK       RCC_AHB1Periph_GPIOA
#define USART2_RX_PORTCLK       RCC_AHB1Periph_GPIOA
#define USART2_USCLK			RCC_APB1Periph_USART2
#define USART2_RX_PIN          	GPIO_Pin_3
#define USART2_TX_PIN          	GPIO_Pin_2
#define USART2_EXTI_IRQn     	USART2_IRQn 
#define USART2_TX_AF         	GPIO_AF_USART2
#define USART2_TX_SOURCE     	GPIO_PinSource2
#define USART2_RX_AF         	GPIO_AF_USART2
#define USART2_RX_SOURCE     	GPIO_PinSource3

//////////////////////////////////////////
///USART3 引脚及中断配置
//////////////////////////////////////////
#define USART3_PORT_NAME		USART3
#define USART3_TX_PORT          GPIOB
#define USART3_RX_PORT          GPIOB
#define USART3_TX_PORTCLK       RCC_AHB1Periph_GPIOB
#define USART3_RX_PORTCLK       RCC_AHB1Periph_GPIOB
#define USART3_USCLK			RCC_APB1Periph_USART3
#define USART3_RX_PIN          	GPIO_Pin_11
#define USART3_TX_PIN          	GPIO_Pin_10
#define USART3_EXTI_IRQn     	USART3_IRQn 
#define USART3_TX_AF         	GPIO_AF_USART3
#define USART3_TX_SOURCE     	GPIO_PinSource10
#define USART3_RX_AF         	GPIO_AF_USART3
#define USART3_RX_SOURCE     	GPIO_PinSource11

//////////////////////////////////////////
///USART4 引脚及中断配置
//////////////////////////////////////////
#define USART4_PORT_NAME		UART4
#define USART4_TX_PORT          GPIOA
#define USART4_RX_PORT          GPIOA
#define USART4_TX_PORTCLK       RCC_AHB1Periph_GPIOA
#define USART4_RX_PORTCLK       RCC_AHB1Periph_GPIOA
#define USART4_USCLK			RCC_APB1Periph_UART4
#define USART4_RX_PIN          	GPIO_Pin_1
#define USART4_TX_PIN          	GPIO_Pin_0
#define USART4_EXTI_IRQn     	UART4_IRQn 
#define USART4_TX_AF         	GPIO_AF_UART4
#define USART4_TX_SOURCE     	GPIO_PinSource0
#define USART4_RX_AF         	GPIO_AF_UART4
#define USART4_RX_SOURCE     	GPIO_PinSource1

//////////////////////////////////////////
///USART5 引脚及中断配置
//////////////////////////////////////////
#define USART5_PORT_NAME		UART5
#define USART5_TX_PORT          GPIOC
#define USART5_RX_PORT          GPIOD
#define USART5_TX_PORTCLK       RCC_AHB1Periph_GPIOC
#define USART5_RX_PORTCLK       RCC_AHB1Periph_GPIOD
#define USART5_USCLK			RCC_APB1Periph_UART5
#define USART5_RX_PIN          	GPIO_Pin_2
#define USART5_TX_PIN          	GPIO_Pin_12
#define USART5_EXTI_IRQn     	UART5_IRQn 
#define USART5_TX_AF         	GPIO_AF_UART5
#define USART5_TX_SOURCE     	GPIO_PinSource12
#define USART5_RX_AF         	GPIO_AF_UART5
#define USART5_RX_SOURCE     	GPIO_PinSource2

//////////////////////////////////////////
///USART6 引脚及中断配置
//////////////////////////////////////////
#define USART6_PORT_NAME		USART6
#define USART6_TX_PORT          GPIOG
#define USART6_RX_PORT          GPIOG
#define USART6_TX_PORTCLK       RCC_AHB1Periph_GPIOG
#define USART6_RX_PORTCLK       RCC_AHB1Periph_GPIOG
#define USART6_USCLK			RCC_APB2Periph_USART6
#define USART6_RX_PIN          	GPIO_Pin_9
#define USART6_TX_PIN          	GPIO_Pin_14
#define USART6_EXTI_IRQn     	USART6_IRQn 
#define USART6_TX_AF         	GPIO_AF_USART6
#define USART6_TX_SOURCE     	GPIO_PinSource14
#define USART6_RX_AF         	GPIO_AF_USART6
#define USART6_RX_SOURCE     	GPIO_PinSource9



////////////////////////////////////////////////////////////////////////////////
 //串口结构体定义
 ////////////////////////////////////////////////////////////////////////////////
struct USART_TypeDefStruct {
	__IO COM_PORT_TypeDef ComPort; 		// 串口号 COM1,COM2,COM3,COM4,COM5
	__IO uint32_t BaudRate;			  	//波特率

	uint8_t USART_Read_buf[MAX_USART_BUFFSIZE];
	uint16_t USART_Read_Head;
	uint16_t USART_Read_Tail;

	uint8_t USART_Write_buf[MAX_USART_BUFFSIZE];
	uint16_t USART_Write_Head;
	uint16_t USART_Write_Tail;

	//////////////接收数据处理函数/////////////////
	void (*pFuncRxDataIrq)(uint8_t rxdata, struct USART_TypeDefStruct *pUSART);

	void (*pUsartIrq)();		//串口中断服务函数指针
};



void USART_CreatComPort(struct USART_TypeDefStruct *pUSART); //创建串口


uint8_t Usart_read(struct USART_TypeDefStruct *pUSART);		//从串口读取一个字节


void Usart_write(struct USART_TypeDefStruct *pUSART, uint8_t senddata);
															//从串口输出一个字节

void Usart_writes(struct USART_TypeDefStruct *pUSART, uint8_t *sendbuff);
															//从串口输出字符串

void Usart_writesb(struct USART_TypeDefStruct *pUSART, uint8_t *sendbuff,
		uint16_t length);
															//从串口输出给定长度的字符串


uint16_t Usart_intflag(struct USART_TypeDefStruct *pUSART);
															//查询读取缓冲区长度

uint16_t Usart_outflag(struct USART_TypeDefStruct *pUSART);
															//查询发送缓冲区长度

void Usart_intreset(struct USART_TypeDefStruct *pUSART);
															//清空读取缓冲区

void Usart_outreset(struct USART_TypeDefStruct *pUSART);
															//清空发送缓冲区

void USART_IRQ(struct USART_TypeDefStruct *pUSART);
															//串口中断函数

#endif
