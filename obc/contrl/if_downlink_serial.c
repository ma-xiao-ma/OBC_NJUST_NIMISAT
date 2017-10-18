/*
 * if_downlink_serial.c
 *
 *  Created on: 2017年8月22日
 *      Author: Ma Wenli
 *
 *  描述：联调串行接口函数，实现桌面联调 串口发送星上遥测，接收地面指令
 *      H1A19--->USART2_TX H1A20--->USART2_RX
 */

#include <if_downlink_serial.h>
#include "ctrl_cmd_types.h"
#include "contrl.h"
#include "task_user.h"
#include "route.h"

static uint8_t SendBuffer[USART2_MTU] __attribute__((section(".bss.hk")));
static uint8_t ReceiveBuffer[USART2_MTU] __attribute__((section(".bss.hk")));

/* 接收结构体 */
static usart2_transmission_object_t Rx_Trans_Obj;


void USART2_DMA_Config(void)
{
    DMA_InitTypeDef     DMA_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//开启DMA时钟

    /* DMA1_Stream6--->USART2_TX */
    DMA_DeInit(DMA1_Stream6);
    DMA_InitStructure.DMA_Channel               = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)(&USART2->DR);      //外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr       = (uint32_t)SendBuffer;         //内存地址
    DMA_InitStructure.DMA_DIR                   = DMA_DIR_MemoryToPeripheral;   //内存到外设
    DMA_InitStructure.DMA_BufferSize            = USART2_MTU;                          //缓冲大小
    DMA_InitStructure.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;    //外设地址不增
    DMA_InitStructure.DMA_MemoryInc             = DMA_MemoryInc_Enable;         //内存地址增
    DMA_InitStructure.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_Byte;  //外设数据大小1byte
    DMA_InitStructure.DMA_MemoryDataSize        = DMA_MemoryDataSize_Byte;      //内存数据大小1byte
    DMA_InitStructure.DMA_Mode                  = DMA_Mode_Normal;              //循环模式或者正常模式
    DMA_InitStructure.DMA_Priority              = DMA_Priority_VeryHigh;        //优先级非常高
    DMA_InitStructure.DMA_FIFOMode              = DMA_FIFOMode_Disable;         //不开fifo
    DMA_InitStructure.DMA_FIFOThreshold         = DMA_FIFOThreshold_HalfFull;   //
    DMA_InitStructure.DMA_MemoryBurst           = DMA_MemoryBurst_Single;       //
    DMA_InitStructure.DMA_PeripheralBurst       = DMA_PeripheralBurst_Single;   //
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);                                 //初始化 dma1 stream6
    /* 开启DMA传输完成中断 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);

    /* DMA1_Stream5--->USART2_RX */
    DMA_DeInit(DMA1_Stream5);
    DMA_InitStructure.DMA_Channel               = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)(&USART2->DR);      //外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr       = (uint32_t)ReceiveBuffer;      //内存地址
    DMA_InitStructure.DMA_DIR                   = DMA_DIR_PeripheralToMemory;   //外设到内存
    DMA_InitStructure.DMA_BufferSize            = 65535;                        //缓冲大小
    DMA_InitStructure.DMA_PeripheralInc         = DMA_PeripheralInc_Disable;    //外设地址不增
    DMA_InitStructure.DMA_MemoryInc             = DMA_MemoryInc_Enable;         //内存地址增
    DMA_InitStructure.DMA_PeripheralDataSize    = DMA_PeripheralDataSize_Byte;  //外设数据大小1byte
    DMA_InitStructure.DMA_MemoryDataSize        = DMA_MemoryDataSize_Byte;      //内存数据大小1byte
    DMA_InitStructure.DMA_Mode                  = DMA_Mode_Normal;              //循环模式或者正常模式
    DMA_InitStructure.DMA_Priority              = DMA_Priority_High;            //优先级高
    DMA_InitStructure.DMA_FIFOMode              = DMA_FIFOMode_Disable;         //不开fifo
    DMA_InitStructure.DMA_FIFOThreshold         = DMA_FIFOThreshold_HalfFull;   //
    DMA_InitStructure.DMA_MemoryBurst           = DMA_MemoryBurst_Single;       //
    DMA_InitStructure.DMA_PeripheralBurst       = DMA_PeripheralBurst_Single;   //
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);                                 //初始化 dma1 stream5
    /* 开启DMA传输完成中断 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);

    /* 使能DMA接收 */
    DMA_Cmd(DMA1_Stream5, ENABLE);
}

void USART2_Init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;
    USART_InitTypeDef   USART_InitStructure;

    /* 使能GPIO时钟   PA2--->USART2_TX  PA3--->USART2_RX */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    /* PA2--->USART2_TX */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    /* PA3--->USART2_RX */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    /* Configure USART Tx and Rx as alternate function  */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* USART configuration */
    USART_Init(USART2, &USART_InitStructure);
    /* NVIC configuration */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    /* USART interrupt configuration */
    USART_ITConfig(USART2, USART_IT_TC,   DISABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART2, USART_IT_TXE,  DISABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

    /* 串口发送DMA使能 */
    USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);

    /* Enable USART */
    USART_Cmd(USART2, ENABLE);
}

void vSerialInterfaceInit(void)
{
    /* DMA配置 */
    USART2_DMA_Config();
    /* 串口配置 */
    USART2_Init();
    /* 创建接收队列 */
    Rx_Trans_Obj.queue = xQueueCreate(5, sizeof(void *));
}


/* USART2_TX DMA传输完成中断 */
void DMA1_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
    }
}

/* USART2_RX 串口接收 空闲中断 */
void USART2_IRQHandler(void)
{
    uint32_t ReceivedData;

    static BaseType_t xHigherPriorityTaskWoken;

    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        ReceivedData = USART2->SR;//必须读状态寄存器和数据寄存器 否则会一直进中断
        ReceivedData = USART2->DR;
        ReceivedData = 0;

        USART_ClearITPendingBit(USART2, USART_IT_IDLE);
        USART_ClearFlag(USART2, USART_FLAG_IDLE);

        /* 除能USART2_RX,进入DMA传输完成中断 */
        DMA_Cmd(DMA1_Stream5, DISABLE);
    }

}

/* USART2_RX DMA传输完成中断 */
void DMA1_Stream5_IRQHandler(void)
{
    static BaseType_t xHigherPriorityTaskWoken;
    static route_packet_t *packet;

    if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
    {
        /* 清中断标志 */
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
        /* 确保DMA除能，能被设置 */
        if(DMA_GetCmdStatus(DMA1_Stream5) == DISABLE)
        {
            /* 为接收到的每一包数据申请内存 */
            Rx_Trans_Obj.frame = (usart2_frame_t *)qb50Malloc(sizeof(usart2_frame_t));

            if(Rx_Trans_Obj.frame != NULL)
            {
                /* 组包 */
                Rx_Trans_Obj.frame->len_rx = 65535 - DMA1_Stream5->NDTR;
                memcpy(Rx_Trans_Obj.frame->data, ReceiveBuffer, Rx_Trans_Obj.frame->len_rx);
                xHigherPriorityTaskWoken = 0;
                /* 送入接收队列 */
#if USE_ROUTE_PROTOCOL
                Rx_Trans_Obj.frame->len_rx -= 3;
                packet = (route_packet_t *)Rx_Trans_Obj.frame;
                route_queue_wirte(packet, &xHigherPriorityTaskWoken);
#else
                xQueueSendToBackFromISR(Rx_Trans_Obj.queue, &Rx_Trans_Obj.frame, &xHigherPriorityTaskWoken);
#endif
            }
            /* 使能DMA接收 */
            DMA_Cmd(DMA1_Stream5, ENABLE);
        }
    }
    /* 若有高优先级任务唤醒，执行任务切换 */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx, u16 ndtr)
{
    DMA_Cmd(DMA_Streamx, DISABLE);

    while(DMA_GetCmdStatus(DMA_Streamx) != DISABLE)
    {

    }

    DMA_SetCurrDataCounter(DMA_Streamx, ndtr);

    DMA_Cmd(DMA_Streamx, ENABLE);
}

int vSerialSend(void *pdata, uint16_t length)
{
    uint8_t* DataToSend  = (uint8_t*)pdata;
    uint8_t DataLen = 0;
    do
    {
        DataLen = (length > USART2_MTU) ? USART2_MTU : length;

        /* 拷贝数据到发送缓冲区 */
        memcpy(SendBuffer, (uint8_t *)DataToSend, (size_t)DataLen);
        /* 使能串口DMA发送 */
        MYDMA_Enable(DMA1_Stream6, DataLen);

        DataToSend += DataLen;
        length -= DataLen;

        if(length != 0)
            vTaskDelay(300);
    } while(length);

    return E_NO_ERR;
}

void vSerialACK(void *pdata, uint16_t length)
{
    vSerialSend(pdata, length);
}

int xSerialReceive(usart2_frame_t ** frame, uint32_t timeout)
{
    if (Rx_Trans_Obj.queue == NULL)
        return E_NO_DEVICE;

    if (xQueueReceive(Rx_Trans_Obj.queue, frame, timeout) == pdFALSE)
        return E_TIMEOUT;

    return E_NO_ERR;
}

void USART2_UnpacketTask(void *pvPara)
{
    usart2_frame_t *pdata = (usart2_frame_t *)pvPara;

    CubeUnPacket(pdata->data);
    /* 释放传入任务的内存块 */
    qb50Free(pdata);
    /* 删除自身任务 */
    vTaskDelete(NULL);
}


