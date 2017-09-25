////////////////////////////////////////////////////////////////////////////////
//	功能： 实现串口初始化、收发数据、串口中断功能
//
//	版本：V1.0
//  迭代：
//												南京理工大学微纳卫星中心
//												   2015.11.08
////////////////////////////////////////////////////////////////////////////////


#include <bsp_usart.h>

//////////////////////////////////本地函数定义/////////////////////////////////////
void USART_Config(COM_PORT_TypeDef PortName, uint32_t Baudrate);
void Usart_irq(COM_PORT_TypeDef COM_Name) ;
void (*pUsart_Irq[USART_PORT])();
////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////本地参数定义/////////////////////////////////////
USART_TypeDef * COMPORTNAME[USART_PORT] = {						//串口号
USART1_PORT_NAME,
USART2_PORT_NAME,
USART3_PORT_NAME,
USART4_PORT_NAME,
USART5_PORT_NAME,
USART6_PORT_NAME };

GPIO_TypeDef* COMPORT_TX_PORT[USART_PORT] = {					//发送引脚PORT
USART1_TX_PORT,
USART2_TX_PORT,
USART3_TX_PORT,
USART4_TX_PORT,
USART5_TX_PORT,
USART6_TX_PORT };

GPIO_TypeDef* COMPORT_RX_PORT[USART_PORT] = {					//接收引脚PORT
USART1_RX_PORT,
USART2_RX_PORT,
USART3_RX_PORT,
USART4_RX_PORT,
USART5_RX_PORT,
USART6_RX_PORT };



const uint32_t COMPORT_TX_PORTCLK[USART_PORT] = {				//发送引脚时钟
USART1_TX_PORTCLK,
USART2_TX_PORTCLK,
USART3_TX_PORTCLK,
USART4_TX_PORTCLK,
USART5_TX_PORTCLK,
USART6_TX_PORTCLK };

const uint32_t COMPORT_RX_PORTCLK[USART_PORT] = {				//接收引脚时钟
USART1_RX_PORTCLK,
USART2_RX_PORTCLK,
USART3_RX_PORTCLK,
USART4_RX_PORTCLK,
USART5_RX_PORTCLK,
USART6_RX_PORTCLK };

const uint32_t COMPORT_USCLK[USART_PORT] = {					//时钟
USART1_USCLK,
USART2_USCLK,
USART3_USCLK,
USART4_USCLK,
USART5_USCLK,
USART6_USCLK };

const uint16_t COMPORT_TX_PIN[USART_PORT] = {					//发送引脚
USART1_TX_PIN,
USART2_TX_PIN,
USART3_TX_PIN,
USART4_TX_PIN,
USART5_TX_PIN,
USART6_TX_PIN };

const uint16_t COMPORT_RX_PIN[USART_PORT] = {					//接收引脚
USART1_RX_PIN,
USART2_RX_PIN,
USART3_RX_PIN,
USART4_RX_PIN,
USART5_RX_PIN,
USART6_RX_PIN };

const uint16_t COMPORT_IRQn[USART_PORT] = {						//中断
USART1_EXTI_IRQn,
USART2_EXTI_IRQn,
USART3_EXTI_IRQn,
USART4_EXTI_IRQn,
USART5_EXTI_IRQn,
USART6_EXTI_IRQn };
const uint16_t COMPORT_TX_PIN_SOURCE[USART_PORT] = {			//发送引脚Source
USART1_TX_SOURCE,
USART2_TX_SOURCE,
USART3_TX_SOURCE,
USART4_TX_SOURCE,
USART5_TX_SOURCE,
USART6_TX_SOURCE };
const uint16_t COMPORT_RX_PIN_SOURCE[USART_PORT] = {			//接收引脚Source
USART1_RX_SOURCE,
USART2_RX_SOURCE,
USART3_RX_SOURCE,
USART4_RX_SOURCE,
USART5_RX_SOURCE,
USART6_RX_SOURCE };
const uint16_t COMPORT_TX_AF[USART_PORT] = {					//发送AF
USART1_TX_AF,
USART2_TX_AF,
USART3_TX_AF,
USART4_TX_AF,
USART5_TX_AF,
USART6_TX_AF };
const uint16_t COMPORT_RX_AF[USART_PORT] = {					//接收AF
USART1_RX_AF,
USART2_RX_AF,
USART3_RX_AF,
USART4_RX_AF,
USART5_RX_AF,
USART6_RX_AF };
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//	功能说明:创建串口
//	形    参:  串口结构体指针
//	返 回 值: 无
////////////////////////////////////////////////////////////////////////////////
void USART_CreatComPort(struct USART_TypeDefStruct *pUSART) {

	uint16_t kc;

	pUSART->USART_Read_Head = 0;  				//接收缓冲区清空
	pUSART->USART_Read_Tail = 0;

	pUSART->USART_Write_Head = 0;				//发送缓冲区清空
	pUSART->USART_Write_Tail = 0;

	for (kc = 0; kc < MAX_USART_BUFFSIZE; kc++) {	//缓冲区初始化
		pUSART->USART_Read_buf[kc] = 0;
		pUSART->USART_Write_buf[kc] = 0;
	}

	pUsart_Irq[pUSART->ComPort] = pUSART->pUsartIrq; // 设置接收中断服务函数的函数指针
	USART_Config(pUSART->ComPort, pUSART->BaudRate);

}

////////////////////////////////////////////////////////////////////////////////
//	功能说明:串口配置函数 功能：串口时钟开启、IO设置、波特率设置、打开接受、发送中断
//	形    参:
//			PortName:串口号
//			Baudrate:波特率
//	返 回 值: 无
////////////////////////////////////////////////////////////////////////////////
void USART_Config(COM_PORT_TypeDef PortName, uint32_t Baudrate) {

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(
			COMPORT_TX_PORTCLK[PortName] | COMPORT_RX_PORTCLK[PortName],
			ENABLE);

	if ((PortName == COM1) || (PortName == COM6)) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	} else {
		RCC_APB1PeriphClockCmd(COMPORT_USCLK[PortName], ENABLE);
	}

	// Connect PXx to USARTx_Tx
	GPIO_PinAFConfig(COMPORT_TX_PORT[PortName], COMPORT_TX_PIN_SOURCE[PortName],
			COMPORT_TX_AF[PortName]);

	// Connect PXx to USARTx_Rx
	GPIO_PinAFConfig(COMPORT_RX_PORT[PortName], COMPORT_RX_PIN_SOURCE[PortName],
			COMPORT_RX_AF[PortName]);

	GPIO_InitStructure.GPIO_Pin = COMPORT_TX_PIN[PortName];
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(COMPORT_TX_PORT[PortName], &GPIO_InitStructure);

	// 将USART Rx的GPIO配置为浮空输入模式
	// 由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
	// 但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
	GPIO_InitStructure.GPIO_Pin = COMPORT_RX_PIN[PortName];
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(COMPORT_RX_PORT[PortName], &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = Baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(COMPORTNAME[PortName], &USART_InitStructure); //配置串口参数函数

	// Enable USART1 Receive and Transmit interrupts
	USART_ITConfig(COMPORTNAME[PortName], USART_IT_RXNE, ENABLE); //使能接收中断


//	USART_ITConfig(COMPORTNAME[PortName], USART_IT_PE, ENABLE);	//开启PE错误接收中断Bit
																//8PEIE: PE interrupt enable

//	USART_ITConfig(COMPORTNAME[PortName], USART_IT_ERR, ENABLE);  //CR2 开启ERR中断

	// Enable the USART
	USART_Cmd(COMPORTNAME[PortName], ENABLE);

	// CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
	//	如下语句解决第1个字节无法正确发送出去的问题
	USART_ClearFlag(COMPORTNAME[PortName], USART_FLAG_TC);  //清发送完成标志
															//Transmission Complete flag


	// Configure the NVIC Preemption Priority Bits
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// Enable the USARTX Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = COMPORT_IRQn[PortName];
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}


////////////////////////////////////////////////////////////////////////////////
////功能：从串口缓冲区读取一个字节
////输入：串口结构体
////输出: 从缓冲区读取的字节
////////////////////////////////////////////////////////////////////////////////
uint8_t Usart_read(struct USART_TypeDefStruct *pUSART) {

	uint8_t readdata;

	readdata = pUSART->USART_Read_buf[pUSART->USART_Read_Head];
	if (++(pUSART->USART_Read_Head) == MAX_USART_BUFFSIZE)
		pUSART->USART_Read_Head = 0;

	return (readdata);
}

////////////////////////////////////////////////////////////////////////////////
////功能：从串口输出一个字节
////输入：串口结构体
///		senddata:待输出的字节
////输出: 无
////////////////////////////////////////////////////////////////////////////////
void Usart_write(struct USART_TypeDefStruct *pUSART, uint8_t senddata) {

	if (pUSART->USART_Write_Head == pUSART->USART_Write_Tail) {

		USART_SendData(COMPORTNAME[pUSART->ComPort], senddata);
		//while(USART_GetITStatus(COMPORTNAME[pUSART->ComPort], USART_IT_TXE) == RESET);
		while(USART_GetFlagStatus(COMPORTNAME[pUSART->ComPort],USART_FLAG_TC)==RESET);
		return;
	}

	pUSART->USART_Write_buf[pUSART->USART_Write_Tail] = senddata;
	if (++(pUSART->USART_Write_Tail) == MAX_USART_BUFFSIZE)
		pUSART->USART_Write_Tail = 0;

	//USART_ITConfig(COMPORTNAME[pUSART->ComPort], USART_IT_TXE, ENABLE);	//
}

////////////////////////////////////////////////////////////////////////////////
////功能：从串口输出任意长度的字符串
////输入：串口结构体
///		sendbuff:待输出的字符串
////输出: 无
////////////////////////////////////////////////////////////////////////////////
void Usart_writes(struct USART_TypeDefStruct *pUSART, uint8_t *sendbuff) {

	uint8_t kc = 0;
	for (;;) {
		if ((pUSART->USART_Write_buf[pUSART->USART_Write_Tail] = sendbuff[kc++])
				== '\0')
			break;
		if (++(pUSART->USART_Write_Tail) == MAX_USART_BUFFSIZE)
			pUSART->USART_Write_Tail = 0;
	}
	USART_ITConfig(COMPORTNAME[pUSART->ComPort], USART_IT_TXE, ENABLE);	//
}

////////////////////////////////////////////////////////////////////////////////
////功能：从串口输出固定长度的字符串
////输入：串口结构体
///		sendbuff:待输出的字符串
///		length  :length
////输出: 无
////////////////////////////////////////////////////////////////////////////////
void Usart_writesb(struct USART_TypeDefStruct *pUSART, uint8_t *sendbuff,
		uint16_t length) {

	uint8_t kc = 0;
	for (kc = 0; kc < length; kc++) {
		pUSART->USART_Write_buf[pUSART->USART_Write_Tail] = sendbuff[kc];
		if (++(pUSART->USART_Write_Tail) == MAX_USART_BUFFSIZE)
			pUSART->USART_Write_Tail = 0;
	}
	USART_ITConfig(COMPORTNAME[pUSART->ComPort], USART_IT_TXE, ENABLE);
}

////////////////////////////////////////////////////////////////////////////////
////功能：查询接收缓冲区长度
////输入：串口结构体
////输出: 接收缓冲区长度
////////////////////////////////////////////////////////////////////////////////
uint16_t Usart_intflag(struct USART_TypeDefStruct *pUSART) {

	uint16_t length = 0;
	if (pUSART->USART_Read_Tail >= pUSART->USART_Read_Head) {
		length = pUSART->USART_Read_Tail - pUSART->USART_Read_Head;
	} else {
		length = pUSART->USART_Read_Tail + MAX_USART_BUFFSIZE
				- pUSART->USART_Read_Head;
	}
	return length;
}

////////////////////////////////////////////////////////////////////////////////
////功能：查询发送缓冲区长度
////输入：串口结构体
////输出: 发送缓冲区长度
////////////////////////////////////////////////////////////////////////////////
uint16_t Usart_outflag(struct USART_TypeDefStruct *pUSART) {
	if (pUSART->USART_Write_Tail >= pUSART->USART_Write_Head)
		return (pUSART->USART_Write_Tail - pUSART->USART_Write_Head);
	else
		return (pUSART->USART_Write_Tail + MAX_USART_BUFFSIZE
				- pUSART->USART_Write_Head);
}

////////////////////////////////////////////////////////////////////////////////
////功能：清空接收缓冲区
////输入：串口结构体
////输出: 无
////////////////////////////////////////////////////////////////////////////////
void Usart_intreset(struct USART_TypeDefStruct *pUSART) {
	pUSART->USART_Write_Head = pUSART->USART_Read_Head = 0;
}

////////////////////////////////////////////////////////////////////////////////
////功能：清空发送缓冲区
////输入：串口结构体
////输出: 无
////////////////////////////////////////////////////////////////////////////////
void Usart_outreset(struct USART_TypeDefStruct *pUSART) {
	pUSART->USART_Read_Tail = pUSART->USART_Write_Tail = 0;
}

////////////////////////////////////////////////////////////////////////////////
////功能：串口中断响应
////输入：串口结构体
////输出: 无
////////////////////////////////////////////////////////////////////////////////
void USART_IRQ(struct USART_TypeDefStruct *pUSART) {
	uint8_t _rxData;
	uint8_t ComPort = pUSART->ComPort;

	if (USART_GetITStatus(COMPORTNAME[ComPort], USART_IT_ORE) != RESET) {
		USART_ClearFlag(COMPORTNAME[ComPort], USART_FLAG_ORE);
	}

	if (USART_GetITStatus(COMPORTNAME[ComPort], USART_IT_RXNE) != RESET) {
		//判断读寄存器是否非空
		_rxData = (uint8_t) (USART_ReceiveData(COMPORTNAME[ComPort]) & 0xff);

		pUSART->USART_Read_buf[pUSART->USART_Read_Tail] = _rxData;

		pUSART->pFuncRxDataIrq(_rxData & 0xff, pUSART);  // 接收数据送到处理函数中

		if (++(pUSART->USART_Read_Tail) == MAX_USART_BUFFSIZE)
			pUSART->USART_Read_Tail = 0;
		USART_ClearITPendingBit(COMPORTNAME[ComPort], USART_IT_RXNE);
	}

	if (USART_GetITStatus(COMPORTNAME[ComPort], USART_IT_TXE) != RESET) {
		//发送数据
		if (pUSART->USART_Write_Head != pUSART->USART_Write_Tail) {
			USART_SendData(COMPORTNAME[ComPort],
					pUSART->USART_Write_buf[pUSART->USART_Write_Head]);
			if (++(pUSART->USART_Write_Head) == MAX_USART_BUFFSIZE)
				pUSART->USART_Write_Head = 0;
		}
		else {
			USART_ITConfig(COMPORTNAME[ComPort], USART_IT_TXE, DISABLE);
		}

		USART_ClearITPendingBit(COMPORTNAME[ComPort], USART_IT_TXE);

	}

	if (USART_GetITStatus(COMPORTNAME[ComPort], USART_IT_NE) != RESET) {
		USART_ClearFlag(COMPORTNAME[ComPort], USART_FLAG_NE);

	}
	if (USART_GetITStatus(COMPORTNAME[ComPort], USART_IT_FE) != RESET) {
		USART_ClearFlag(COMPORTNAME[ComPort], USART_FLAG_FE);

	}
	if (USART_GetITStatus(COMPORTNAME[ComPort], USART_IT_PE) != RESET) {
		USART_ClearFlag(COMPORTNAME[ComPort], USART_FLAG_PE);

	}
}

////////////////////////////////////////////////////////////////////////////////
////功能：将中断捕获与处理联系起来
////输入：串口号
////输出: 无
////////////////////////////////////////////////////////////////////////////////
void Usart_irq(COM_PORT_TypeDef COM_Name) {
	pUsart_Irq[COM_Name]();
}


//void USART1_IRQHandler(void) {
//	Usart_irq(COM1);
//}

//void USART2_IRQHandler(void) {
//	Usart_irq(COM2);
//}

void USART3_IRQHandler(void) {
	Usart_irq(COM3);
}

//void UART4_IRQHandler(void) {
//	Usart_irq(COM4);
//}

void UART5_IRQHandler(void) {
	Usart_irq(COM5);
}

void USART6_IRQHandler(void) {
	Usart_irq(COM6);
}
