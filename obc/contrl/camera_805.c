/*
 * camera_805.c
 *
 *  Created on: 2017年6月17日
 *      Author: Ma Wenli
 */

#include <string.h>

#include "driver_debug.h"
#include "bsp_nor_flash.h"
#include "error.h"
#include "contrl.h"
#include "bsp_switch.h"
#include "crc.h"
#include "obc_mem.h"
#include "error.h"
#include "csp_endian.h"
#include "bsp_ds1302.h"
#include "hk.h"
#include "file_opt.h"
#include "hexdump.h"
#include "if_downlink_vu.h"
#include "cube_com.h"
#include "router_io.h"

#include "command.h"
#include "console.h"

#include "camera_805.h"

#define CAM_REP_TIMEOUT 100
#define FLASH_IMG_NUM   15
static int ImagStoreInFlash(void);
static int ImagStoreInSD(void);

/*相机数据传输信息*/
static CamTrans_t Cam __attribute__((section(".bss.hk"), aligned(4)));

static uint8_t image_test_arry[256 * 1024] __attribute__((section(".bss.hk"), aligned(4)));

/*当前图像信息*/
static ImageInfo_t CurrentImage __attribute__((section(".bss.hk"), aligned(4)));


/*图像和flash扇区对应表*/ /*每4个flash sector为一个图像的存储块 大小为 4*32768*2 byte(4*32768 halfword)*/
static cam_flash image_store_falsh[FLASH_IMG_NUM] __attribute__((section(".data.hk")))=
        {
            { 8, 0}, {12, 0}, {16, 0}, {20, 0}, {24, 0},
            {28, 0}, {32, 0}, {36, 0}, {40, 0}, {44, 0},
            {48, 0}, {52, 0}, {56, 0}, {60, 0}, {64, 0}
        };

/**
 * 测试缓冲区4字节对齐
 */
void aligned_addr_test(void)
{
    printf("Picture data address: 0x%08x\r\n", &Cam.ReceiveBuffer[6]);
    printf("Picture Info address: 0x%08x\r\n", &CurrentImage);
}


#define CamerarReceiveBufferSize  65535   //64 * 1024 -1

void Camera_805_USART_DMA_Config(void)
{
    DMA_InitTypeDef     DMA_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(CAMERA_DMA_CLK, ENABLE);//开启DMA时钟

    DMA_DeInit(CAMERA_DMA_RX_STREAM);
    DMA_InitStructure.DMA_Channel               = DMA_Channel_4;
    /* 外设地址 */
    DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)(&USART1->DR);
    /* 内存地址 */
    DMA_InitStructure.DMA_Memory0BaseAddr       = (uint32_t)Cam.ReceiveBuffer;
    /* 外设到内存 */
    DMA_InitStructure.DMA_DIR                   = DMA_DIR_PeripheralToMemory;
    /* 缓冲大小 */
    DMA_InitStructure.DMA_BufferSize            = CamerarReceiveBufferSize;
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
    DMA_Init(CAMERA_DMA_RX_STREAM, &DMA_InitStructure);                         //初始化dma2 stream2

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    DMA_ITConfig(CAMERA_DMA_RX_STREAM, DMA_IT_TC, ENABLE);

    DMA_DeInit(CAMERA_DMA_TX_STREAM);
    DMA_InitStructure.DMA_Channel               = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)(&USART1->DR);      //外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr       = (uint32_t)Cam.SendBuffer;     //内存地址
    DMA_InitStructure.DMA_DIR                   = DMA_DIR_MemoryToPeripheral;   //内存到外设
    DMA_InitStructure.DMA_BufferSize            = TX_BUFFER_SIZE;               //缓冲大小
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
    DMA_Init(CAMERA_DMA_TX_STREAM, &DMA_InitStructure);                         //初始化dma2 stream7

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel = CAMERA_TX_ISR_CHANEL;
    NVIC_Init(&NVIC_InitStructure);
    DMA_ITConfig(CAMERA_DMA_TX_STREAM, DMA_IT_TC, ENABLE);

    DMA_Cmd(CAMERA_DMA_RX_STREAM, ENABLE);
}

void Camera_805_USART_Init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;
    USART_InitTypeDef   USART_InitStructure;

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(CAMERA_TX_PORTCLK | CAMERA_RX_PORTCLK, ENABLE);
    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(CAMERA_SCLK, ENABLE);
    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(CAMERA_TX_PORT, CAMERA_TX_SOURCE, CAMERA_TX_AF);
    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(CAMERA_RX_PORT, CAMERA_RX_SOURCE, CAMERA_RX_AF);
    /* Configure USART Tx and Rx as alternate function  */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = CAMERA_TX_PIN | CAMERA_RX_PIN;
    GPIO_Init(CAMERA_TX_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* USART configuration */
    USART_Init(CAMERA_PORT_NAME, &USART_InitStructure);
    /* NVIC configuration */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel = CAMERA_RX_ISR_CHANEL;
    NVIC_Init(&NVIC_InitStructure);
    /* USART interrupt configuration */
    USART_ITConfig(CAMERA_PORT_NAME, USART_IT_TC,   DISABLE);
    USART_ITConfig(CAMERA_PORT_NAME, USART_IT_RXNE, DISABLE);
    USART_ITConfig(CAMERA_PORT_NAME, USART_IT_TXE,  DISABLE);
    USART_ITConfig(CAMERA_PORT_NAME, USART_IT_IDLE, ENABLE);

    /* 串口发送DMA使能 */
    USART_DMACmd(CAMERA_PORT_NAME, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
}

/**
 *图像存储映射表恢复，函数声明
 */
void img_store_block_recover(void);


void Camera_805_Init(void)
{
    Camera_805_USART_DMA_Config();
    Camera_805_USART_Init();
    Cam.AccessMutexSem = xSemaphoreCreateMutex();
    Cam.SynchBinSem = xSemaphoreCreateBinary();

    /*图像flash存储映射表恢复*/
    img_store_block_recover();
    /*开启相机加热常开的一路*/
    EpsOutSwitch(OUT_CAMERA_HEAT_1, ENABLE);
}

void CAMERA_TX_ISR_HANDLER(void)
{
    if(DMA_GetFlagStatus(CAMERA_DMA_TX_STREAM, DMA_FLAG_TCIF7) != RESET)
    {
        DMA_ClearFlag(CAMERA_DMA_TX_STREAM, DMA_FLAG_TCIF7);
    }
}

#define RESERVED_MASK   (uint32_t)0x0F7D0F7D
static uint32_t DataOffset = 1; //用来表征发起了几次DMA传输，还有用来计算
                                //需要设置的存储器地址，初始值为1

/* DMA传输完成中断设置，因为DMA每次最大传输65535个字
 * 节数据，为了实现连续串口接收超过65535字节数据 */
void DMA2_Stream2_IRQHandler(void)
{
    if ((DMA2_Stream2->CR & (uint32_t)DMA_SxCR_EN) != SET)
    {
        DMA2->LIFCR = (uint32_t)(DMA_FLAG_TCIF2 & RESERVED_MASK);  //清传输完成标志

        if(!DataOffset)
            DMA2_Stream2->NDTR = 0;

        DMA2_Stream2->M0AR = (uint32_t)&Cam.ReceiveBuffer[CamerarReceiveBufferSize*DataOffset -
                                        DMA2_Stream2->NDTR];  //设置存储器地址
        DMA2_Stream2->NDTR = (uint16_t)CamerarReceiveBufferSize;//编程DMA接收字节数
        DMA2_Stream2->CR |= DMA_SxCR_EN;    //使能 串口1 DMA接收
        DataOffset ++;
    }
}

    /* 串口空闲中断 */
void CAMERA_RX_ISR_HANDLER(void)
{
    uint8_t ReceivedData;
    static BaseType_t TaskWoken = pdFALSE;
    uint16_t CleanUpRegist;

    if(USART_GetFlagStatus(CAMERA_PORT_NAME, USART_FLAG_ORE) != RESET)
    {
        USART_ClearFlag(CAMERA_PORT_NAME, USART_FLAG_ORE);
        ReceivedData = USART_ReceiveData(CAMERA_PORT_NAME);
        printf("Receive overflow error!\n\r");
    }

    if( USART_GetITStatus( CAMERA_PORT_NAME, USART_IT_IDLE ) != RESET )
    {
        CleanUpRegist = USART1->SR;//必须读状态寄存器和数据寄存器 否则会一直进中断
        CleanUpRegist = USART1->DR;

        USART_ClearFlag( CAMERA_PORT_NAME, USART_FLAG_IDLE );
        USART_ClearITPendingBit( CAMERA_PORT_NAME, USART_IT_IDLE );

        Cam.Rxlen = CamerarReceiveBufferSize * DataOffset -
        		DMA_GetCurrDataCounter(CAMERA_DMA_RX_STREAM);
        driver_debug(DEBUG_CAMERA,"Data_Len = %u\n\r", Cam.Rxlen);
        driver_debug(DEBUG_CAMERA,"Address = %p\n\r", Cam.ReceiveBuffer);

        DataOffset = 0;
        DMA_Cmd(CAMERA_DMA_RX_STREAM, DISABLE);//关闭DMA会产生DMA传输完成中断
        xSemaphoreGiveFromISR(Cam.SynchBinSem, &TaskWoken);
    }

    portYIELD_FROM_ISR(TaskWoken);
}

/**
 * 相机复位指令
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_805_reset(void)
{
    /* 获取相机访问锁，申请访问相机 */
    if( xSemaphoreTake(Cam.AccessMutexSem, 10) != pdTRUE )
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM take mutex sem fail!\r\n");
        return E_NO_QUEUE;
    }

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xAA;
    Cam.SendBuffer[1] = 0x7E;

    /* 使能 USART */
    USART_Cmd(CAMERA_PORT_NAME, ENABLE);
    /* 使能 DMA，发送指令码 */
    DMA_Cmd(CAMERA_DMA_TX_STREAM, ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, CAM_REP_TIMEOUT) != pdTRUE)
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM rsp receive timeout!!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_TIMEOUT;
    }

    if (Cam.ReceiveBuffer[2] != 0x7E)
    {
        driver_debug(DEBUG_CAMERA,"ERROR: CAM rsp error!!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_INVALID_PARAM;
    }

    /* 除能 USART */
    USART_Cmd(CAMERA_PORT_NAME, DISABLE);
    /* 给出互斥锁 */
    xSemaphoreGive(Cam.AccessMutexSem);

    return E_NO_ERR;
}

/**
 * 获取相机三个采温点温度，不能在中断中调用
 *
 * @param Point 采温点1为0x01,采温点2为0x02,采温点3为0x03
 * @param temp Point点的温度输出值指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Temp_Get(uint8_t Point, uint16_t *temp)
{
    /* 获取相机访问锁，申请访问相机 */
    if( xSemaphoreTake(Cam.AccessMutexSem, 10) != pdTRUE )
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM take mutex sem fail!\r\n");
        return E_NO_QUEUE;
    }

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xAA;
    Cam.SendBuffer[1] = 0x05;
    Cam.SendBuffer[2] = Point;

    /* 使能 USART */
    USART_Cmd(CAMERA_PORT_NAME, ENABLE);
    /* 使能DMA传输 */
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, CAM_REP_TIMEOUT) != pdTRUE)
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM rsp receive timeout!!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_TIMEOUT;
    }

    /* 安全检查，回复的温度是否为需采的温度点 */
    if( Cam.ReceiveBuffer[2] != 0x05 || Cam.ReceiveBuffer[3] != Point )
    {
        driver_debug(DEBUG_CAMERA,"ERROR: CAM RSP error!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_INVALID_PARAM;
    }

    /* 大端转换成主机序 */
    *temp = csp_betoh16(*(uint16_t *)&Cam.ReceiveBuffer[4]);

    /* 除能 USART */
    USART_Cmd(CAMERA_PORT_NAME, DISABLE);
    /* 给出互斥锁 */
    xSemaphoreGive(Cam.AccessMutexSem);

    return E_NO_ERR;
}

/**
 * 读取相机的曝光时间设置值
 *
 * @param exp_time 接收缓冲区指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Exposure_Time_Read(uint32_t *exp_time)
{
    /* 获取相机访问锁，申请访问相机 */
    if( xSemaphoreTake(Cam.AccessMutexSem, 10) != pdTRUE )
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM take mutex sem fail!\r\n");
        return E_NO_QUEUE;
    }

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xAA;
    Cam.SendBuffer[1] = 0x06;

    /* 使能 USART */
    USART_Cmd(CAMERA_PORT_NAME, ENABLE);
    /* 使能DMA传输 */
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, CAM_REP_TIMEOUT) != pdTRUE)
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM rsp receive timeout!!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_TIMEOUT;
    }

    /* 安全检查 */
    if(Cam.ReceiveBuffer[2] != 0x06)
    {
        driver_debug(DEBUG_CAMERA,"ERROR: CAM RSP error!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_INVALID_PARAM;
    }

    /* 大字端转换成主机序 */
    Cam.ReceiveBuffer[2] = 0;
    *exp_time = csp_betoh32(*(uint32_t *)&Cam.ReceiveBuffer[2]);

    /* 除能 USART */
    USART_Cmd(CAMERA_PORT_NAME, DISABLE);
    /* 给出互斥锁 */
    xSemaphoreGive(Cam.AccessMutexSem);

    return E_NO_ERR;
}

/**
 * 曝光时间设置
 *
 * @param exp_time 曝光时间设置值
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Exposure_Time_Set(uint32_t exp_time)
{
    /* 获取相机访问锁，申请访问相机 */
    if( xSemaphoreTake(Cam.AccessMutexSem, 10) != pdTRUE )
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM take mutex sem fail!\r\n");
        return E_NO_QUEUE;
    }

    exp_time &= 0x00FFFFFF;

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xaa;
    Cam.SendBuffer[1] = 0x06;
    Cam.SendBuffer[2] = 0x01;
    Cam.SendBuffer[3] = (uint8_t)(exp_time>>16);
    Cam.SendBuffer[4] = (uint8_t)(exp_time>>8);
    Cam.SendBuffer[5] = (uint8_t)exp_time;

    /* 使能 USART */
    USART_Cmd(CAMERA_PORT_NAME, ENABLE);
    /* 使能DMA传输 */
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, CAM_REP_TIMEOUT) != pdTRUE)
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM rsp receive timeout!!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_TIMEOUT;
    }

    /* 安全检查 */
    if(Cam.ReceiveBuffer[2] != 0x06)
    {
        driver_debug(DEBUG_CAMERA,"ERROR: CAM RSP error!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_INVALID_PARAM;
    }

    /* 若返回值不等于设置值 */
    Cam.ReceiveBuffer[2] = 0;
    if (csp_betoh32(*(uint32_t *)&Cam.ReceiveBuffer[2]) != exp_time)
    {
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_INVALID_PARAM;
    }

    /* 除能 USART */
    USART_Cmd(CAMERA_PORT_NAME, DISABLE);
    /* 给出互斥锁 */
    xSemaphoreGive(Cam.AccessMutexSem);

    return E_NO_ERR;
}

/**
 * 读取相机已设置增益值
 *
 * @param gain 接收指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Gain_Get(uint8_t *gain)
{
    /* 获取相机访问锁，申请访问相机 */
    if( xSemaphoreTake(Cam.AccessMutexSem, 10) != pdTRUE )
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM take mutex sem fail!\r\n");
        return E_NO_QUEUE;
    }

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xAA;
    Cam.SendBuffer[1] = 0x07;

    /* 使能 USART */
    USART_Cmd(CAMERA_PORT_NAME, ENABLE);
    /* 使能DMA传输 */
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, CAM_REP_TIMEOUT) != pdTRUE)
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM rsp receive timeout!!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_TIMEOUT;
    }

    /* 安全检查 */
    if(Cam.ReceiveBuffer[2] != 0x07)
    {
        driver_debug(DEBUG_CAMERA,"ERROR: CAM RSP error!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_INVALID_PARAM;
    }

    *gain = Cam.ReceiveBuffer[5];

    /* 除能 USART */
    USART_Cmd(CAMERA_PORT_NAME, DISABLE);
    /* 给出互斥锁 */
    xSemaphoreGive(Cam.AccessMutexSem);

    return E_NO_ERR;
}

/**
 * 相机增益设置
 *
 * @param gain 相机增益系数设置值
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Gain_Set(uint8_t gain)
{
    /* 获取相机访问锁，申请访问相机 */
    if( xSemaphoreTake(Cam.AccessMutexSem, 10) != pdTRUE )
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM take mutex sem fail!\r\n");
        return E_NO_QUEUE;
    }

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xAA;
    Cam.SendBuffer[1] = 0x07;
    Cam.SendBuffer[2] = 0x01;
    Cam.SendBuffer[5] = gain;

    /* 使能 USART */
    USART_Cmd(CAMERA_PORT_NAME, ENABLE);
    /* 使能DMA传输 */
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, CAM_REP_TIMEOUT) != pdTRUE)
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM rsp receive timeout!!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_TIMEOUT;
    }

    /* 安全检查 */
    if(Cam.ReceiveBuffer[2] != 0x07 || Cam.ReceiveBuffer[5] != gain)
    {
        driver_debug(DEBUG_CAMERA,"ERROR: CAM RSP error!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_INVALID_PARAM;
    }

    /* 除能 USART */
    USART_Cmd(CAMERA_PORT_NAME, DISABLE);

    /* 给出互斥锁 */
    xSemaphoreGive(Cam.AccessMutexSem);

    return E_NO_ERR;
}

/**
 * 获取相机当前的控制模式
 *
 * @param cam_ctl_mode 接收缓冲区指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Work_Mode_Get(cam_ctl_t *cam_ctl_mode)
{
    /* 获取相机访问锁，申请访问相机 */
    if( xSemaphoreTake(Cam.AccessMutexSem, 10) != pdTRUE )
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM take mutex sem fail!\r\n");
        return E_NO_QUEUE;
    }

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xAA;
    Cam.SendBuffer[1] = 0x08;

    /* 使能 USART */
    USART_Cmd(CAMERA_PORT_NAME, ENABLE);
    /* 使能DMA传输 */
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);


    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, CAM_REP_TIMEOUT) != pdTRUE)
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM rsp receive timeout!!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_TIMEOUT;
    }

    /* 安全检查 */
    if(Cam.ReceiveBuffer[2] != 0x08)
    {
        driver_debug(DEBUG_CAMERA,"ERROR: CAM RSP error!\r\n");
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);
        return E_INVALID_PARAM;
    }

    memcpy(cam_ctl_mode, &Cam.ReceiveBuffer[3], sizeof(cam_ctl_t));

    /* 除能 USART */
    USART_Cmd(CAMERA_PORT_NAME, DISABLE);
    /* 给出互斥锁 */
    xSemaphoreGive(Cam.AccessMutexSem);

    return E_NO_ERR;
}

/*******************************************************************************
函数说明:   设置相机工作模式
入口参数:   传输方式字节：    0x00 关闭
                    0x01 LVDS传输
                    0x02 TTL传输

                            工作模式：            0x00 图像模式RAW
                    0x01 图像模式1fps
                    0x02 视频模式
                    0x03 备份模式

                            自动曝光：            0x00 自动曝光开
                    0x01 自动曝光关
返回值:   正常返回相机当前的控制模式，次高字节为传输方式，次低字节为工作模式，最低字节
        为是否自动曝光。
/******************************************************************************/

/**
 * 相机工作模式设定
 *
 * @param cam_ctl_mode 控制模式结构体
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Work_Mode_Set(cam_ctl_t cam_ctl_mode)
{
    /* 获取相机访问锁，申请访问相机 */
    if( xSemaphoreTake(Cam.AccessMutexSem, 10) != pdTRUE )
    {
        driver_debug(DEBUG_CAMERA,"WARNING: CAM take mutex sem fail!\r\n");
        return E_NO_QUEUE;
    }

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xaa;
    Cam.SendBuffer[1] = 0x08;
    Cam.SendBuffer[2] = 0x01;
    Cam.SendBuffer[3] = cam_ctl_mode.tran;
    Cam.SendBuffer[4] = cam_ctl_mode.mode;
    Cam.SendBuffer[5] = cam_ctl_mode.expo;

    /* 使能 USART */
    USART_Cmd(CAMERA_PORT_NAME, ENABLE);
    /* 使能DMA传输 */
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if (xSemaphoreTake(Cam.SynchBinSem, CAM_REP_TIMEOUT) != pdTRUE)
    {
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);

        driver_debug(DEBUG_CAMERA,"WARNING: CAM RSP receive timeout!\r\n");
        return E_TIMEOUT;
    }

    /* 安全检查 */
    if (Cam.ReceiveBuffer[2] != 0x08)
    {
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);

        driver_debug(DEBUG_CAMERA,"ERROR: CAM RSP error!\r\n");
        return E_INVALID_PARAM;
    }

    for( uint32_t i = 0; i < Cam.Rxlen; i++ )
    {
        driver_debug(DEBUG_CAMERA,"%02x ", Cam.ReceiveBuffer[i]);
    }
    driver_debug(DEBUG_CAMERA,"\r\n");

    /*若返回的的数值与设置的值不同*/
    if ( (*(cam_ctl_t *)&Cam.ReceiveBuffer[3]).tran != cam_ctl_mode.tran ||
         (*(cam_ctl_t *)&Cam.ReceiveBuffer[3]).mode != cam_ctl_mode.mode ||
         (*(cam_ctl_t *)&Cam.ReceiveBuffer[3]).expo != cam_ctl_mode.expo   )
    {
        /* 除能 USART */
        USART_Cmd(CAMERA_PORT_NAME, DISABLE);
        /* 给出互斥锁 */
        xSemaphoreGive(Cam.AccessMutexSem);

        driver_debug(DEBUG_CAMERA,"ERROR: CAM RSP error!\r\n");
        return E_INVALID_PARAM;
    }

    /* 如果是TTL传输+备份模式，即相机拍照并通过串口传到OBC */
    if(cam_ctl_mode.tran == TTL && cam_ctl_mode.mode == Backup)
    {
        /* 附拍照时间 */
        CurrentImage.ImageTime = clock_get_time_nopara();

        /* 附位置信息 */
        memcpy(CurrentImage.ImageLocation,
                hk_frame.append_frame.adcs_hk.adcs805_hk_orbit.downAdcsOrbPos, sizeof(CurrentImage.ImageLocation));

        /* 等待相机图像传输完成，超时时间40s */
        if(xSemaphoreTake(Cam.SynchBinSem, 40000) != pdTRUE)
        {
            /* 除能 USART */
            USART_Cmd(CAMERA_PORT_NAME, DISABLE);
            /* 给出互斥锁 */
            xSemaphoreGive(Cam.AccessMutexSem);

            driver_debug(DEBUG_CAMERA,"WARNING: CAM data receive timeout!!\r\n");
            return E_TIMEOUT;
        }

        driver_debug( DEBUG_CAMERA,"Check sum: %02x\r\n", sum_check(&Cam.ReceiveBuffer[0], Cam.Rxlen - 5) );

        memmove( &Cam.ReceiveBuffer[6], &Cam.ReceiveBuffer[0], Cam.Rxlen );

        driver_debug( DEBUG_CAMERA,"Check sum: %02x\r\n", sum_check(&Cam.ReceiveBuffer[6], Cam.Rxlen - 5) );

        for( uint32_t i = 0; i < 12; i++ )
        {
            driver_debug(DEBUG_CAMERA,"%02x ", Cam.ReceiveBuffer[i]);
        }
        driver_debug(DEBUG_CAMERA,"\r\n");

//        /* 和校验 */
//        if( sum_check(&Cam.ReceiveBuffer[6], Cam.Rxlen - 5) != Cam.ReceiveBuffer[Cam.Rxlen + 1] )
//        {
//            /* 除能 USART */
//            USART_Cmd(CAMERA_PORT_NAME, DISABLE);
//            /* 给出互斥锁 */
//            xSemaphoreGive(Cam.AccessMutexSem);
//
//            driver_debug(DEBUG_CAMERA,"WARNING: CAM data sum check fail!!\r\n");
//            return E_CRC_CHECK_ERROR;
//        }

        /* 给图像附ID号 */
        CurrentImage.ImageID ++;

        /* 常规图像包中图像数据的大小 */
        CurrentImage.PacketSize = IMAGE_PACK_MAX_SIZE;

        /* 图像大小 */
        CurrentImage.ImageSize = Cam.Rxlen - 5;
        CurrentImage.ImageSize = (CurrentImage.ImageSize % 2) ? CurrentImage.ImageSize + 1 : CurrentImage.ImageSize;

        /* 下行图像需要的总图像包数量 */
        CurrentImage.TotalPacket = (CurrentImage.ImageSize % IMAGE_PACK_MAX_SIZE) ?
                CurrentImage.ImageSize / IMAGE_PACK_MAX_SIZE + 1 : CurrentImage.ImageSize / IMAGE_PACK_MAX_SIZE;

        /* 最后一个图像包中图像数据的大小 */
        CurrentImage.LastPacketSize = (CurrentImage.ImageSize % IMAGE_PACK_MAX_SIZE) ?
                CurrentImage.ImageSize % IMAGE_PACK_MAX_SIZE : IMAGE_PACK_MAX_SIZE;

        /* 图像数据片外NorFLASH存储，Flash中总是存最近的一包       */
        ImagStoreInFlash();

        /* 图像数据TF卡存储 */
        ImagStoreInSD();
    }

    /* 除能 USART */
    USART_Cmd(CAMERA_PORT_NAME, DISABLE);
    /* 给出互斥锁 */
    xSemaphoreGive(Cam.AccessMutexSem);

    return E_NO_ERR;
}


/**
 * 通过照片ID去查找图像存储块的首扇区号
 *
 * @param id 待查找图像ID
 * @return 返回图像存储块的首扇区号， 若没有查到，则返回0
 */
uint32_t image_find_flash_store_sector(uint32_t id)
{
    for (int i = 0; i < FLASH_IMG_NUM; i++)
    {
        if (id == image_store_falsh[i].image_id)
            return image_store_falsh[i].falsh_sector;
    }

    return 0;
}

/**
 * 每4个flash sector为一个图像的存储块 大小为 4*32768*2 byte(4*32768 halfword)
 * 通过扇区号查找扇区在nor flash中的16位读写首地址
 *
 * @param sector 扇区号
 * @return 返回扇区16位读写首地址
 */
uint32_t get_addr_via_sector_num(uint32_t sector)
{
    if(sector < 8)
        return (sector * 4096);
    else
        return ((sector-7) * 32768);
}

/**
 * 拍照完成，在flash中存储图像时，寻找存储的扇区
 *
 * @return 返回可以进行存储的存储块首扇区，返回0为错误状态
 */
static uint32_t cam_find_flash_free_sector(void)
{
    int i;

    cam_flash *store_loc = &image_store_falsh[0];

    /* 遍历存储表 */
    for (i = 0; i < FLASH_IMG_NUM; i++)
    {
        /* 若有未使用的存储块，则返回未使用存储块的扇区号*/
        if ( image_store_falsh[i].image_id == 0 )
        {
            store_loc = &image_store_falsh[i];
            break;
        } /* 图像id越小，图像越老  */
        else if ( image_store_falsh[i].image_id < store_loc->image_id )
        {
            store_loc = &image_store_falsh[i];
        }
    }
    /* 若没有未使用的存储块，则擦除最老的一幅图像的存储块 ，
     * 并返回该存储块扇区号。
     */
    if (i == FLASH_IMG_NUM)
    {
        for ( int j = store_loc->falsh_sector; j < store_loc->falsh_sector + 4; j++ )
        {
            if ( USER_NOR_SectorErase(j) != NOR_SUCCESS )
                return 0;
        }

        store_loc->image_id = 0;
    }

    /* 确保整个存储块是擦除过的状态 */
    uint32_t sector_start_addr = get_addr_via_sector_num(store_loc->falsh_sector);
    for ( uint32_t flash_addr = sector_start_addr; flash_addr < sector_start_addr + 4 * 32768; flash_addr++)
    {
        if (0xFFFF != FSMC_NOR_ReadHalfWord(flash_addr))
        {
            if (FSMC_NOR_EraseBlock(flash_addr) != NOR_SUCCESS)
                return 0;
        }
    }

    return store_loc->falsh_sector;
}

/**
 * 用于寻找flash存储块中最新的一张照片的扇区号
 *
 * @return 返回最新照片的扇区号
 */
static uint32_t cam_find_newest_img(void)
{
    int i;

    cam_flash store_loc = image_store_falsh[0];

    /* 遍历存储表 */
    for (i = 0; i < FLASH_IMG_NUM; i++)
    {
        /* 跳过未使用的存储块 */
        if ( image_store_falsh[i].image_id != 0 )
        {   /*找到照片ID最大的一项*/
            if ( image_store_falsh[i].image_id > store_loc.image_id )
                store_loc = image_store_falsh[i];
        }
    }

    /*如果存储块还未使用*/
    if ( store_loc.image_id == 0 )
        return 0;

    return store_loc.falsh_sector;
}

/**
 * 将图像ID和存储块首扇区号绑定
 *
 * @param sector 扇区号
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
static int sector_bind_id(uint32_t sector, uint32_t id)
{
    int i;

    for (i = 0; i < FLASH_IMG_NUM; i++)
    {
        if (sector == image_store_falsh[i].falsh_sector)
        {
            if (image_store_falsh[i].image_id != 0)
                return E_INVALID_PARAM;
            else
            {
                image_store_falsh[i].image_id = id;
                break;
            }
        }
    }

    if (i == FLASH_IMG_NUM)
        return E_NO_BUFFER;

    return E_NO_ERR;
}

/**
 * 图像存储映射表恢复，照片ID恢复， 初始化时调用
 *
 */
void img_store_block_recover(void)
{
    uint32_t max_image_id = 0;
    /* 遍历存储表 */
    for (int i = 0; i < FLASH_IMG_NUM; i++)
    {
        FSMC_NOR_ReadBuffer((uint16_t *)&image_store_falsh[i].image_id,
                get_addr_via_sector_num(image_store_falsh[i].falsh_sector), sizeof(uint32_t) / 2);

        if (image_store_falsh[i].image_id == 0xFFFFFFFF)
            image_store_falsh[i].image_id = 0;

        /* 获取当前照片ID值 */
        max_image_id = (image_store_falsh[i].image_id > max_image_id) ?
                image_store_falsh[i].image_id : max_image_id;
    }

    /* 恢复当前照片ID值 */
    CurrentImage.ImageID = max_image_id;
}

/**
 * 向flash中存储拍好的照片， 内部调用
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
static int ImagStoreInFlash(void)
{
    uint32_t free_block_sector = cam_find_flash_free_sector();

    if(free_block_sector == 0)
    {
        driver_debug(DEBUG_CAMERA, "ERROR: Store in flash find free sector fail!!\r\n");
        return E_FLASH_ERROR;
    }

    uint32_t write_addr = get_addr_via_sector_num(free_block_sector);

    if (FSMC_NOR_WriteBuffer((uint16_t *)&CurrentImage, write_addr, sizeof(ImageInfo_t) / 2) != NOR_SUCCESS)
    {
        driver_debug(DEBUG_CAMERA, "ERROR: IMG FLASH STORE write image info fail!!\r\n");
        return E_FLASH_ERROR;
    }

    if (FSMC_NOR_WriteBuffer( (uint16_t *)&Cam.ReceiveBuffer[6], write_addr + sizeof(ImageInfo_t) / 2,
            CurrentImage.ImageSize / 2) != NOR_SUCCESS )
    {
        driver_debug(DEBUG_CAMERA, "ERROR: IMG FLASH STORE write image data fail!!\r\n");
        return E_FLASH_ERROR;
    }

    ImageInfo_t image_info_test = {0};
    FSMC_NOR_ReadBuffer( (uint16_t *)&image_info_test, write_addr, sizeof(ImageInfo_t) / 2 );

    /* 测试FLASH存储正确性 */
    memset(image_test_arry, 0, 256*1024);
    FSMC_NOR_ReadBuffer( (uint16_t *)image_test_arry, write_addr + sizeof(ImageInfo_t) / 2, CurrentImage.ImageSize / 2 );

    if( memcmp(image_test_arry, &Cam.ReceiveBuffer[6], CurrentImage.ImageSize ) == 0 )
    {
        driver_debug(DEBUG_CAMERA, "INFO: IMG flash store OK.\r\n");
    }
    else
    {
        driver_debug(DEBUG_CAMERA, "INFO: IMG flash store OK.\r\n");
    }

    if(sector_bind_id(free_block_sector, CurrentImage.ImageID) != E_NO_ERR)
    {
        driver_debug(DEBUG_CAMERA, "ERROR: IMG FLASH STORE id bind sector fail!!\r\n");
        return E_FLASH_ERROR;
    }

    return E_NO_ERR;
}

/**
 * 向SD卡中存储拍好的照片， 内部调用
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
static int ImagStoreInSD(void)
{
    static uint32_t img_dir_init = 0;

    char path[30] = {0};

    /*若初始化标志位0，则创建img文件夹*/
    if(img_dir_init == 0)
    {
        f_mkdir("0:img");
        img_dir_init++;
    }

    /**
     * 创建图像信息文件ImageInfo.dat
     */
    memset(path, 0, 30);
    sprintf(path, "0:img/ImageInfo-%u.dat", CurrentImage.ImageID);

    if (file_write(path, &CurrentImage, (UINT)sizeof(ImageInfo_t)) != FR_OK)
    {
        driver_debug(DEBUG_CAMERA, "Camera Create ImageInfo.dat Failure!!\r\n");
        return E_NO_DEVICE;
    }

    /* 创建图像原始数据文件 */
    memset(path, 0, 30);
    sprintf(path, "0:img/ImageData-%u.dat", CurrentImage.ImageID);

    if( file_write( path, &Cam.ReceiveBuffer[6], (UINT)CurrentImage.ImageSize ) != FR_OK )
    {
        driver_debug(DEBUG_CAMERA, "Camera Write ImageData.dat Failure!!\r\n");
        return E_NO_DEVICE;
    }

    return E_NO_ERR;
}

/**
 * 下行外部SRAM中图像信息缓冲区中的数据
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_sram_img_info_down(void)
{
    return ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE_INFO, &CurrentImage, sizeof(ImageInfo_t));
}

/**
 * 下行外部SRAM中图像数据缓冲区中的图像数据
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_sram_img_data_down(void)
{
    return image_whole_download(&Cam.ReceiveBuffer[6], CurrentImage.ImageSize, 0);
}

/**
 * 下行外部SRAM中从start_packet起始的图像
 *
 * @param start_packet 起始包号
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_sram_img_packet_data_down(uint16_t start_packet)
{
    if (start_packet >= CurrentImage.TotalPacket)
        return E_INVALID_PARAM;

    return image_whole_download(&Cam.ReceiveBuffer[6 + IMAGE_PACK_MAX_SIZE * start_packet],
            CurrentImage.ImageSize - IMAGE_PACK_MAX_SIZE * start_packet, start_packet);
}

/**
 * 下行外部SRAM中图像数据缓冲区中某包图像的数据
 *
 * @param packet_id 下行图像包的ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_sram_img_packet_down(uint16_t packet_id)
{
    if (packet_id >= CurrentImage.TotalPacket)
        return E_INVALID_PARAM;

    ImagePacket_t * img_packet = ObcMemMalloc(sizeof(ImagePacket_t));
    if (img_packet == NULL)
        return E_NO_BUFFER;

    img_packet->PacketID = packet_id;
    img_packet->PacketSize = (packet_id == CurrentImage.TotalPacket - 1) ?
            CurrentImage.LastPacketSize : IMAGE_PACK_MAX_SIZE;

    memcpy(img_packet->ImageData, &Cam.ReceiveBuffer[6 + packet_id*IMAGE_PACK_MAX_SIZE], img_packet->PacketSize);
    int ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE, &img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);

    ObcMemFree(img_packet);
    return ret;
}

/**
 * 通过图像ID参数下行SD卡中图像信息
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_sd_img_info_down(uint32_t id)
{
    char path[40] = {0};

    ImageInfo_t *img_info = (ImageInfo_t *)ObcMemMalloc(sizeof(ImageInfo_t));
    if (img_info == NULL)
        return E_NO_BUFFER;

    sprintf(path, "0:img/ImageInfo-%u.dat", id);

    if (file_read(path, img_info, (UINT)sizeof(ImageInfo_t), 0) != FR_OK)
    {
        driver_debug(DEBUG_CAMERA, "Camera read ImageInfo.dat Failure!!\r\n");
        ObcMemFree(img_info);
        return E_NO_DEVICE;
    }

    /* 调用传输层接口函数下行  */
    int ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE_INFO, img_info, sizeof(ImageInfo_t));

    ObcMemFree(img_info);
    return ret;
}



/**
 * 从SD卡中取出ID编号的图像以及信息到SRAM缓冲区
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_sd_img_to_sram(uint32_t id)
{
    char path[40] = {0};

    sprintf(path, "0:img/ImageInfo-%u.dat", id);

    if (file_read(path, &CurrentImage, (UINT)sizeof(ImageInfo_t), 0) != FR_OK)
    {
        driver_debug(DEBUG_CAMERA, "Camera read ImageInfo.dat Failure!!\r\n");
        return E_NO_DEVICE;
    }

    sprintf(path, "0:img/ImageData-%u.dat", id);

    if (file_read(path, &Cam.ReceiveBuffer[6], (UINT)CurrentImage.ImageSize, 0) != FR_OK)
    {
        driver_debug(DEBUG_CAMERA, "Camera read ImageData.dat Failure!!\r\n");
        return E_NO_DEVICE;
    }

    return E_NO_ERR;
}

/**
 * 下行SD卡中单包图像
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_sd_img_packet_down(uint32_t id, uint16_t packet)
{
    char path[40] = {0};

    ImageInfo_t *img_info = (ImageInfo_t *)ObcMemMalloc(sizeof(ImageInfo_t));
    if (img_info == NULL)
        return E_NO_BUFFER;

    sprintf(path, "0:img/ImageInfo-%u.dat", id);

    if (file_read(path, img_info, (UINT)sizeof(ImageInfo_t), 0) != FR_OK)
    {
        driver_debug(DEBUG_CAMERA, "Camera read ImageInfo.dat Failure!!\r\n");
        ObcMemFree(img_info);
        return E_NO_DEVICE;
    }

    if (packet >= img_info->TotalPacket)
    {
        ObcMemFree(img_info);
        return E_INVALID_PARAM;
    }

    ImagePacket_t * img_packet = (ImagePacket_t *)ObcMemMalloc(sizeof(ImagePacket_t));
    if (img_packet == NULL)
    {
        ObcMemFree(img_info);
        return E_NO_BUFFER;
    }

    img_packet->PacketID = packet;
    img_packet->PacketSize = (packet == img_info->TotalPacket-1) ?
            img_info->LastPacketSize : IMAGE_PACK_MAX_SIZE;

    sprintf(path, "0:img/ImageData-%u.dat", id);

    if (file_read(path, img_packet->ImageData, img_packet->PacketSize, packet * IMAGE_PACK_MAX_SIZE) != FR_OK)
    {
        driver_debug(DEBUG_CAMERA, "Camera read ImageData.dat Failure!!\r\n");
        ObcMemFree(img_info);
        ObcMemFree(img_packet);
        return E_NO_DEVICE;
    }

    /* 调用传输层接口函数下行 */
    int ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE, img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);

    ObcMemFree(img_info);
    ObcMemFree(img_packet);
    return E_NO_ERR;
}


/**
 * 通过图像ID参数下行flash中图像信息
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_flash_img_info_down(uint32_t id)
{
    uint32_t sector_num = image_find_flash_store_sector(id);

    /*若查找返回值为0，则说明flash中不存在这张图片*/
    if(sector_num == 0)
        return E_INVALID_PARAM;

    ImageInfo_t *img_info = (ImageInfo_t *)ObcMemMalloc(sizeof(ImageInfo_t));
    if (img_info == NULL)
        return E_NO_BUFFER;

    FSMC_NOR_ReadBuffer((uint16_t *)img_info, get_addr_via_sector_num(sector_num),
            sizeof(ImageInfo_t));

    /* 调用传输层接口函数下行，创建下行图像任务  */
    int ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE_INFO, img_info, sizeof(ImageInfo_t));

    ObcMemFree(img_info);
    return ret;
}

/**
 * 下行FLASH中单包图像
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_flash_img_packet_down(uint32_t id, uint16_t packet)
{
    uint32_t sector_num = image_find_flash_store_sector(id);

    /*若查找返回值为0，则说明flash中不存在这张图片*/
    if(sector_num == 0)
        return E_INVALID_PARAM;

    ImageInfo_t *img_info = (ImageInfo_t *)ObcMemMalloc(sizeof(ImageInfo_t));
    if (img_info == NULL)
        return E_NO_BUFFER;

    FSMC_NOR_ReadBuffer((uint16_t *)img_info, get_addr_via_sector_num(sector_num),
            sizeof(ImageInfo_t));

    if (packet >= img_info->TotalPacket)
    {
        ObcMemFree(img_info);
        return E_INVALID_PARAM;
    }

    ImagePacket_t * img_packet = (ImagePacket_t * )ObcMemMalloc(sizeof(ImagePacket_t));
    if (img_packet == NULL)
    {
        ObcMemFree(img_info);
        return E_NO_BUFFER;
    }

    img_packet->PacketID = packet;
    img_packet->PacketSize = (packet == img_info->TotalPacket-1) ?
            img_info->LastPacketSize : IMAGE_PACK_MAX_SIZE;

    FSMC_NOR_ReadBuffer((uint16_t *)img_packet->ImageData, get_addr_via_sector_num(sector_num) + sizeof(ImageInfo_t) / 2
            + packet * IMAGE_PACK_MAX_SIZE, img_packet->PacketSize);

    /* 调用传输层接口函数下行 */
    int ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE, img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);

    ObcMemFree(img_info);
    ObcMemFree(img_packet);
    return ret;
}


/**
 * 从FLASH中取出ID编号的图像以及信息到SRAM缓冲区
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_flash_img_to_sram(uint32_t id)
{
    uint32_t sector_num = image_find_flash_store_sector(id);

    /*若查找返回值为0，则说明flash中不存在这张图片*/
    if(sector_num == 0)
        return E_INVALID_PARAM;

    FSMC_NOR_ReadBuffer((uint16_t *)&CurrentImage, get_addr_via_sector_num(sector_num),
            sizeof(ImageInfo_t));

    FSMC_NOR_ReadBuffer((uint16_t *)&Cam.ReceiveBuffer[6],
            get_addr_via_sector_num(sector_num) + sizeof(ImageInfo_t) / 2, CurrentImage.ImageSize);

    return E_NO_ERR;
}

/**
 * 下行最新的照片信息
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_newest_img_info_down(void)
{
    int ret;
    uint32_t sector_num = cam_find_newest_img();

    /*如果存储区还未使用*/
    if(sector_num == 0 && CurrentImage.ImageID == 0)
        return E_INVALID_PARAM;

    ImageInfo_t *newest_img = (ImageInfo_t *)ObcMemMalloc(sizeof(ImageInfo_t));
    if (newest_img == NULL)
        return E_NO_BUFFER;


    FSMC_NOR_ReadBuffer((uint16_t *)newest_img, get_addr_via_sector_num(sector_num),
                sizeof(ImageInfo_t));

    if (newest_img->ImageID == CurrentImage.ImageID)
    {
        ret = cam_sram_img_info_down();
    }
    else
    {
        ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE_INFO, newest_img, sizeof(ImageInfo_t));

        if (ret != E_NO_ERR)
        {
            ret = cam_sd_img_info_down(newest_img->ImageID);
        }
    }

    ObcMemFree(newest_img);
    return ret;
}

/**
 * 下行特定ID图像信息
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_img_info_down(uint32_t id)
{
    int ret;

    if (id == CurrentImage.ImageID)
    {
        ret = cam_sram_img_info_down();
    }
    else
    {
        if (cam_flash_img_info_down(id) != E_NO_ERR)
        {
            ret = cam_sd_img_info_down(id);
        }

    }

    return ret;
}

/**
 * 相机下行指定ID照片
 *
 * @param id 照片ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_img_data_down(uint32_t id)
{
    int ret;

    if (id == CurrentImage.ImageID)
    {
        ret = cam_sram_img_data_down();

    }
    else
    {
        ret = cam_flash_img_to_sram(id);

        if (ret != E_NO_ERR)
            ret = cam_sd_img_to_sram(id);

        if (ret == E_NO_ERR)
            ret = cam_sram_img_data_down();
    }

    return ret;
}

/**
 * 下行ID编号图像的第packet包图像数据
 *
 * @param id 图像ID
 * @param packet 图像包号
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_img_packet_down(uint32_t id, uint16_t packet)
{
    int ret;

    if (id == CurrentImage.ImageID)
    {
        ret = cam_sram_img_packet_down(packet);

    }
    else
    {
        ret = cam_flash_img_packet_down(id, packet);

        if (ret != E_NO_ERR)
            ret = cam_sd_img_packet_down(id, packet);
    }

    return ret;
}

/**
 * 从起始包号start_packet开始下行图像数据
 *
 * @param id 图像ID号
 * @param start_packet 起始包号
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_img_data_packet_down(uint32_t id, uint16_t start_packet)
{
    int ret;

    if (id == CurrentImage.ImageID)
    {
        ret = cam_sram_img_packet_data_down(start_packet);

    }
    else
    {
        ret = cam_flash_img_to_sram(id);

        if (ret != E_NO_ERR)
            ret = cam_sd_img_to_sram(id);

        if (ret == E_NO_ERR)
            ret = cam_sram_img_packet_data_down(start_packet);
    }

    return ret;
}


int cmd_reset_camera(struct command_context * ctx __attribute__((unused)))
{
    int flag=0;
    flag=Camera_805_reset();
    if(flag==E_NO_ERR)
        printf("Reset successful");
    else
        printf("Reset failed");
    return CMD_ERROR_NONE;
}

int cmd_Camera_Temp_Get(struct command_context *ctx)
{
    int flag=0;
    char * args = command_args(ctx);
    uint16_t Temp;
    uint8_t point;

    if (sscanf(args, "%x", &point)!= 1)
        return CMD_ERROR_SYNTAX;

    flag=Camera_Temp_Get(point,&Temp);

    if(flag==E_NO_ERR)
        printf("Get temperature successful!!\r\nPoint:%x Temp:%x\r\n",point , Temp);
    else
        printf("Get temperature failed\r\n");

    return CMD_ERROR_NONE;
}


int cmd_Camera_Exposure_Time_Read(struct command_context * ctx __attribute__((unused)))
{
    int flag=0;
    uint32_t Exp_time = 0;

    flag = Camera_Exposure_Time_Read(&Exp_time);

    if(flag == E_NO_ERR)
        printf("Get exposure time successful!!\r\nResult: %u\r\n", Exp_time);
    else
        printf("Get exposure time failed.\r\n");

    return CMD_ERROR_NONE;
}

int cmd_Camera_Exposure_Time_Set(struct command_context *ctx)
{
    char * args = command_args(ctx);
    uint32_t Exp_time;
    int flag=0;

    if (sscanf(args, "%x", &Exp_time)!= 1)
        return CMD_ERROR_SYNTAX;

    flag=Camera_Exposure_Time_Set(Exp_time);

    if(flag==E_NO_ERR)
        printf("Set exposure time successful the value is:%x\r\n",Exp_time);
    else
        printf("Set exposure time failed\r\n");

    return CMD_ERROR_NONE;
}


int cmd_Camera_Gain_Get(struct command_context * ctx __attribute__((unused)))
{
    int flag=0;
    uint8_t Gain;

    flag=Camera_Gain_Get(&Gain);

    if(flag==E_NO_ERR)
        printf("Get gain  successful the result is:%x\r\n",Gain);
    else
        printf("Get gain failed\r\n");

    return CMD_ERROR_NONE;
}

int cmd_Camera_Gain_Set(struct command_context *ctx)
{
    char * args = command_args(ctx);
    uint8_t Gain;
    int flag=0;

    if (sscanf(args, "%x", &Gain)!= 1)
        return CMD_ERROR_SYNTAX;

    flag=Camera_Gain_Set(Gain);

    if(flag==E_NO_ERR)
        printf("Set gain successful the value is:%x\r\n",Gain);
    else
        printf("Set gain failed\r\n");

    return CMD_ERROR_NONE;
}

int cmd_Camera_Work_Mode_Get(struct command_context * ctx __attribute__((unused)))
{
    cam_ctl_t Cam_mode;
    int flag=0;

    flag=Camera_Work_Mode_Get(&Cam_mode);

    if(flag==E_NO_ERR)
        printf("trans_mode:%x work_mode:%x expo_mode:%x\r\n" , Cam_mode.tran, Cam_mode.mode, Cam_mode.expo);
    else
        printf("Get control mode failed\r\n");

    return CMD_ERROR_NONE;
}

int cmd_Camera_Work_Mode_Set(struct command_context *ctx)
{
    char * args = command_args(ctx);
    static uint8_t Tran, Mode, Expo;
    cam_ctl_t Cam_mode;

    int flag=0;

    if (sscanf(args, "%x %x %x", &Tran, &Mode, &Expo) != 3)
        return CMD_ERROR_SYNTAX;

    Cam_mode.tran=Tran;
    Cam_mode.mode=Mode;
    Cam_mode.expo=Expo;

    flag=Camera_Work_Mode_Set(Cam_mode);

    if(flag==E_NO_ERR)
        printf("Set successful trans_mode:%x work_mode:%x  expo_mode:%x\r\n" ,Cam_mode.tran,Cam_mode.mode,Cam_mode.expo);
    else
        printf("Set control mode failed.\r\n");

    return CMD_ERROR_NONE;
}

command_t __sub_command camera805_subcommands[] = {
    {
        .name = "reset",
        .help = "reset the camera",
        .handler = cmd_reset_camera,
    },{
        .name = "get_temp",
        .help = "Get three temperature at the mining temperature(0x01,0x02,0x03)",
        .usage = "<temperature point> ",
        .handler = cmd_Camera_Temp_Get,
    },{
        .name = "get_exptime",
        .help = "Get camera exposure time",
        .handler = cmd_Camera_Exposure_Time_Read,
    },{
        .name = "set_exptime",
        .help = "Set camera exposure time",
        .usage = "<Exposure time> ",
        .handler = cmd_Camera_Exposure_Time_Set,
    },{
        .name = "get_gain",
        .help = "Get camera gain value",
        .handler = cmd_Camera_Gain_Get,
    },{
        .name = "set_gain",
        .help = "Set camera gain value",
        .usage = "<Gain value> ",
        .handler = cmd_Camera_Gain_Set,
    },{
        .name = "get_mode",
        .help = "Camera control mode",
        .handler = cmd_Camera_Work_Mode_Get,
    },{
        .name = "set_mode",
        .help = "Camera control mode",
        .usage = "<tran> <mode> <expo>",
        .handler = cmd_Camera_Work_Mode_Set,
    },

};
command_t __root_command cam805_commands[] = {
    {
        .name = "camera805",
        .help = "camera command system",
        .chain = INIT_CHAIN(camera805_subcommands),
    },
};

void cmd_cam_setup(void) {
    command_register(cam805_commands);
}


