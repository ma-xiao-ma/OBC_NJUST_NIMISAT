/*
 * camera_805.c
 *
 *  Created on: 2017年6月17日
 *      Author: Ma Wenli
 */

#include "driver_debug.h"
#include "bsp_nor_flash.h"
#include "error.h"
#include "contrl.h"

#include "camera_805.h"


static int ImagStoreInFlash(void);
static int ImagStoreInSD(uint16_t ImagId);

/*相机数据传输信息*/
static CamTrans_t Cam __attribute__((section(".bss.hk")));

/*当前图像信息*/
static ImageInfo_t CurrentImage __attribute__((section(".bss.hk")));

static FIL FileHandle;      //文件句柄
static UINT nByteWritten;   //f_write()函数写入检测值
static UINT nByteRead;      //f_read()函数读取检测值
static char Path[50] = {0}; //文件路径字符串数组

/* 下行图片创建任务相关参数 */
static CamDownloadObj_t tPara;
static char pname[] = "cam0";

static BaseType_t TaskWoken;


#define CamerarReceiveBufferSize  65535

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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    DMA_ITConfig(CAMERA_DMA_RX_STREAM, DMA_IT_TC, ENABLE);

    DMA_DeInit(CAMERA_DMA_TX_STREAM);
    DMA_InitStructure.DMA_Channel               = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr    = (uint32_t)(&USART1->DR);      //外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr       = (uint32_t)Cam.SendBuffer;   //内存地址
    DMA_InitStructure.DMA_DIR                   = DMA_DIR_MemoryToPeripheral;   //内存到外设
    DMA_InitStructure.DMA_BufferSize            = TX_BUFFER_SIZE;         //缓冲大小
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

    /* Enable USART */
    USART_Cmd(CAMERA_PORT_NAME, ENABLE);
}


void Camera_805_Init(void)
{
    Camera_805_USART_DMA_Config();
    Camera_805_USART_Init();
    Cam.AccessMutexSem = xSemaphoreCreateMutex();
    Cam.SynchBinSem = xSemaphoreCreateBinary();
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

        DMA2_Stream2->M0AR = &Cam.ReceiveBuffer[CamerarReceiveBufferSize*DataOffset -
                                        DMA2_Stream2->NDTR];  //设置存储器地址
        DMA2_Stream2->NDTR = (uint16_t)CamerarReceiveBufferSize;//编程DMA接收字节数
        DMA2_Stream2->CR |= DMA_SxCR_EN;    //使能 串口1 DMA接收
        DataOffset ++;
    }
}

/* NorFlash相关定义 */
//uint16_t DataToNor;
//u32 CamReceiveCount = 0; //单位：字节
//u8 HalfWordFlag = 0;
//#define PhotoStorageBase 32768
//#define PhotoOne(x)  ((uint32_t)(PhotoStorageBase + x))

    /* 串口空闲中断 */
void CAMERA_RX_ISR_HANDLER(void)
{
    uint8_t ReceivedData;
    TaskWoken = pdFALSE;
    uint16_t CleanUpRegist;

    if(USART_GetFlagStatus(CAMERA_PORT_NAME, USART_FLAG_ORE) != RESET)
    {
        USART_ClearFlag(CAMERA_PORT_NAME, USART_FLAG_ORE);
        ReceivedData = USART_ReceiveData(CAMERA_PORT_NAME);
        printf("Receive overflow error!\n\r");
    }
    if(USART_GetITStatus(CAMERA_PORT_NAME, USART_IT_IDLE) != RESET)
    {
        CleanUpRegist = USART1->SR;//必须读状态寄存器和数据寄存器 否则会一直进中断
        CleanUpRegist = USART1->DR;

        USART_ClearFlag(CAMERA_PORT_NAME, USART_FLAG_IDLE);
        USART_ClearITPendingBit(CAMERA_PORT_NAME, USART_IT_IDLE);

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

/*******************************************************************************
函数说明:  相机复位 不能在中断中调用
入口参数:    无
返回值:     复位指令码0x7e
/******************************************************************************/
uint8_t Camera_805_reset(void)
{
    /* 获取相机访问锁，申请访问相机 */
    xSemaphoreTake(Cam.AccessMutexSem, 1000);

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xaa;
    Cam.SendBuffer[1] = 0x7e;
    /* 是能DMA，发送指令码 */
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, 1000) != pdTRUE)
    {
       xSemaphoreGive(Cam.AccessMutexSem);
       return 0;
    }

    uint8_t data_receive = Cam.ReceiveBuffer[2];
    xSemaphoreGive(Cam.AccessMutexSem);

    return data_receive;
}

/*******************************************************************************
函数说明:  获取相机三个采温点温度，不能在中断中调用
入口参数:
    Point  : 采温点1为0x01,采温点2为0x02,采温点3为0x03
返回值:   对应采温点两字节的温度转换值
/******************************************************************************/
uint16_t Camera_Temp_Get(uint8_t Point)
{
    /* 获取相机访问锁，申请访问相机 */
    xSemaphoreTake(Cam.AccessMutexSem, 1000);

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xaa;
    Cam.SendBuffer[1] = 0x05;
    Cam.SendBuffer[2] = Point;
    /* 使能DMA传输 */
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, 1000) != pdTRUE)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }
    /* 安全检查，回复的温度是否为需采的温度点 */
    if(Cam.ReceiveBuffer[3] != Point)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }

    uint16_t data_receive = *(uint16_t *)&Cam.ReceiveBuffer[4];
    xSemaphoreGive(Cam.AccessMutexSem);

    return data_receive;
}

/*******************************************************************************
函数说明:  读取曝光时间
入口参数:
         无
返回值:   4字节曝光时间转换值
/******************************************************************************/
uint32_t Camera_Exposure_Time_Read(void)
{
    /* 获取相机访问锁，申请访问相机 */
    xSemaphoreTake(Cam.AccessMutexSem, 1000);

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xaa;
    Cam.SendBuffer[1] = 0x06;
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, 1000) != pdTRUE)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }
    /* 安全检查 */
    if(Cam.ReceiveBuffer[2] != 0x06)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }

    uint32_t data_receive = *(uint32_t *)&Cam.ReceiveBuffer[2];
    data_receive &= (uint32_t)0x0fffffff;
    xSemaphoreGive(Cam.AccessMutexSem);

    return data_receive;
}

/*******************************************************************************
函数说明:  设置曝光时间
入口参数:
    Point  : 待设置曝光时间 单位：微秒
返回值:   4字节设置后的曝光时间
/******************************************************************************/
uint32_t Camera_Exposure_Time_Set(uint32_t data)
{
    /* 获取相机访问锁，申请访问相机 */
    xSemaphoreTake(Cam.AccessMutexSem, 1000);

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xaa;
    Cam.SendBuffer[1] = 0x06;
    Cam.SendBuffer[2] = 0x01;
    Cam.SendBuffer[3] = (uint8_t)(data>>16);
    Cam.SendBuffer[4] = (uint8_t)(data>>8);
    Cam.SendBuffer[5] = (uint8_t)data;
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, 1000) != pdTRUE)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }

    /* 安全检查 */
    if(Cam.ReceiveBuffer[2] != 0x06)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }

    uint32_t data_receive = *(uint32_t *)&Cam.ReceiveBuffer[2];
    data_receive &= (uint32_t)0x0fffffff;
    xSemaphoreGive(Cam.AccessMutexSem);

    return data_receive;
}

/*******************************************************************************
函数说明:  获取相机增益
入口参数:
            无
返回值:      一字节增益转换值
/******************************************************************************/
uint8_t Camera_Gain_Get(void)
{
    /* 获取相机访问锁，申请访问相机 */
    xSemaphoreTake(Cam.AccessMutexSem, 1000);

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xaa;
    Cam.SendBuffer[1] = 0x07;
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, 1000) != pdTRUE)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }

    /* 安全检查 */
    if(Cam.ReceiveBuffer[2] != 0x07)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }

    char data_receive = Cam.ReceiveBuffer[5];
    xSemaphoreGive(Cam.AccessMutexSem);

    return data_receive;
}

/*******************************************************************************
函数说明:  设置相机增益
入口参数:
            Coefficient：增益系数
返回值:      设置后的增益转换值
/******************************************************************************/
uint8_t Camera_Gain_Set(uint8_t Coefficient)
{
    /* 获取相机访问锁，申请访问相机 */
    xSemaphoreTake(Cam.AccessMutexSem, 1000);

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xaa;
    Cam.SendBuffer[1] = 0x07;
    Cam.SendBuffer[2] = 0x01;
    Cam.SendBuffer[5] = Coefficient;
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, 1000) != pdTRUE)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }

    /* 安全检查 */
    if(Cam.ReceiveBuffer[2] != 0x07)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }

    char data_receive = Cam.ReceiveBuffer[5];
    xSemaphoreGive(Cam.AccessMutexSem);

    return data_receive;
}

/*******************************************************************************
函数说明:  获取相机工作模式
入口参数:
         无
返回值:   正常返回相机当前的控制模式，次高字节为传输方式，次低字节为工作模式，最低字节
        为是否自动曝光。
/******************************************************************************/
uint32_t Camera_Work_Mode_Get(void)
{
    /* 获取相机访问锁，申请访问相机 */
    xSemaphoreTake(Cam.AccessMutexSem, 1000);

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xaa;
    Cam.SendBuffer[1] = 0x08;
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, 1000) != pdTRUE)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }

    /* 安全检查 */
    if(Cam.ReceiveBuffer[2] != 0x08)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }

    uint32_t data_receive = *(uint32_t *)&Cam.ReceiveBuffer[2];
    data_receive &= (uint32_t)0x00ffffff;
    xSemaphoreGive(Cam.AccessMutexSem);

    return data_receive;
}

/*******************************************************************************
函数说明:   设置相机工作模式
入口参数:   传输方式字节：    0x00 关闭
                      0x01 LVDS传输
                      0x02 TTL传输

                        工作模式：          0x00 图像模式RAW
                     0x01 图像模式1fps
                     0x02 视频模式
                     0x03 备份模式

                       自动曝光：           0x00 自动曝光开
                     0x01 自动曝光关
返回值:   正常返回相机当前的控制模式，次高字节为传输方式，次低字节为工作模式，最低字节
        为是否自动曝光。
/******************************************************************************/
uint32_t Camera_Work_Mode_Set(u8 TransMode, u8 WorkMode, u8 AutoExpo, uint16_t id)
{
    /* 获取相机访问锁，申请访问相机 */
    xSemaphoreTake(Cam.AccessMutexSem, 1000);

    memset(Cam.SendBuffer, 0, TX_BUFFER_SIZE);
    Cam.SendBuffer[0] = 0xaa;
    Cam.SendBuffer[1] = 0x08;
    Cam.SendBuffer[2] = 0x01;
    Cam.SendBuffer[3] = TransMode;
    Cam.SendBuffer[4] = WorkMode;
    Cam.SendBuffer[5] = AutoExpo;
    DMA_Cmd(CAMERA_DMA_TX_STREAM,ENABLE);

    /* 如果相机没有收到回复 */
    if(xSemaphoreTake(Cam.SynchBinSem, 1000) != pdTRUE)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }

    /* 安全检查 */
    if(Cam.ReceiveBuffer[2] != 0x08)
    {
        xSemaphoreGive(Cam.AccessMutexSem);
        return 0;
    }

    uint32_t data_receive = *(uint32_t *)&Cam.ReceiveBuffer[2];

    /* 如果是TTL传输+备份模式，即相机拍照并通过串口传到OBC */
    if(data_receive & (uint32_t)0x00020300)
    {
        /* 给图像附ID号 */
        CurrentImage.ImageID = id;
        /* 等待相机图像传输完成，超时时间10s */
        if(xSemaphoreTake(Cam.SynchBinSem, 10000) != pdTRUE)
        {
            xSemaphoreGive(Cam.AccessMutexSem);
            return 0;
        }
        /* 和校验 */
        if(sum_check(Cam.ReceiveBuffer, Cam.Rxlen-1) != Cam.ReceiveBuffer[Cam.Rxlen-1])
        {
            xSemaphoreGive(Cam.AccessMutexSem);
            return 0;
        }
        /* 图像数据TF卡存储 */
        if(ImagStoreInSD(CurrentImage.ImageID) != E_NO_ERR)
        {
            xSemaphoreGive(Cam.AccessMutexSem);
            return 0;
        }
        /* 图像数据片外NorFLASH存储，Flash中总是存最近的一包       */
        if(ImagStoreInFlash() != E_NO_ERR)
        {
            xSemaphoreGive(Cam.AccessMutexSem);
            return 0;
        }
    }
    else //如果是其他控制模式
    {

    }

    data_receive &= (uint32_t)0x00ffffff;
    xSemaphoreGive(Cam.AccessMutexSem);

    return data_receive;
}

static int ImagStoreInFlash(void)
{
    int erase_rult = USER_NOR_SectorErase(8);
    erase_rult = USER_NOR_SectorErase(9);
    erase_rult = USER_NOR_SectorErase(10);

    if(erase_rult != NOR_SUCCESS)
        return E_FLASH_ERROR;

    /* 写入长度信息 */
    uint32_t store_len = Cam.Rxlen - 5;
    store_len = (store_len % 2) ? store_len/2+1 : store_len/2; //2字节对齐的长度

    if(FSMC_NOR_WriteBuffer((uint16_t *)&Cam.ReceiveBuffer, IMAGE_FLASH_STORE_BASE, store_len) != NOR_SUCCESS)
        return E_FLASH_ERROR;

    return E_NO_ERR;
}

static int ImagStoreInSD(uint16_t ImagId)
{
    /* 图像数据大小 */
    uint16_t DataRemaining = Cam.Rxlen - 11; //只取备份图像的数据部分

    ImageHead_t *ptHead = (ImageHead_t *)Cam.ReceiveBuffer;

    ptHead->ImageId = ImagId;

    if(DataRemaining % CAM_PACK_SIZE == 0)//总大小如果能整除包长
    {
        ptHead->LastPacketId = DataRemaining/CAM_PACK_SIZE - 1;//包ID是从0开始的
        ptHead->LastPacketSize = CAM_PACK_SIZE;
    }
    else
    {
        ptHead->LastPacketId = DataRemaining/CAM_PACK_SIZE;
        ptHead->LastPacketSize = DataRemaining % CAM_PACK_SIZE;
    }

    /*
        创建文件夹 picture-“图片id号”
     */
    sprintf(Path, "picture-%u", ImagId);
    f_mkdir(Path);

    /*
        创建图像信息文件ImageInfo.dat
     */
    sprintf(Path, "0:picture-%u/ImageInfo.bin", ImagId);
    if(f_open(&FileHandle, Path, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
    {
        driver_debug(DEBUG_CAMERA, "Camera Create ImageInfo.bin Failure!!\r\n");
        return E_NO_SS;
    }
    /* 写入图像长度数据和包数,文件名：ImageInfo.txt */
    f_write(&FileHandle, Cam.ReceiveBuffer, (UINT)sizeof(ImageHead_t), &nByteWritten);

    if(nByteWritten != 6)
    {
        driver_debug(DEBUG_CAMERA, "Camera Write ImageSize Error!!\r\n");
        return E_FLASH_ERROR;
    }

    f_close(&FileHandle);

    /*
        创建图像数据包文件
     */
    for (uint32_t PacketID=0; PacketID<=ptHead->LastPacketId; PacketID++, DataRemaining-=CAM_PACK_SIZE)
    {
        if(DataRemaining > CAM_PACK_SIZE)    //如果不是最后一包数据
        {
            sprintf(Path, "0:picture-%u/%u.bin", ImagId, PacketID);
            if(f_open(&FileHandle, Path, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
            {
                driver_debug(DEBUG_CAMERA, "Camera Open %s Failure!!\r\n",Path);
                return E_NO_SS;
            }
            /* 指针每次偏移CAM_PACK_SIZE */
            f_write(&FileHandle, &Cam.ReceiveBuffer[CAM_PACK_HEAD_SIZE+PacketID*CAM_PACK_SIZE]
                                            , (UINT)CAM_PACK_SIZE, &nByteWritten);
            if(nByteWritten != CAM_PACK_SIZE)
            {
                driver_debug(DEBUG_CAMERA, "Camera Write ImageSize Failure!!\r\n");
                return E_FLASH_ERROR;
            }
            f_close(&FileHandle);
        }
        else    //如果是最后一包数据
        {
            sprintf(Path, "0:picture-%u/%u.bin", ImagId, PacketID);
            if(f_open(&FileHandle, Path, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
            {
                driver_debug(DEBUG_CAMERA, "Camera Open %s Failure!!\r\n",Path);
                return E_NO_SS;
            }
            /* 写入剩余的数据，大多数情况最后一包数据不是CAM_PACK_SIZE字节 */
            f_write(&FileHandle, &Cam.ReceiveBuffer[CAM_PACK_HEAD_SIZE+PacketID*CAM_PACK_SIZE]
                                            , (UINT)DataRemaining, &nByteWritten);
            if(nByteWritten != CAM_PACK_SIZE)
            {
                driver_debug(DEBUG_CAMERA, "Camera Write ImageSize Failure!!\r\n");
                return E_FLASH_ERROR;
            }
            f_close(&FileHandle);
        }
    }

    return E_NO_ERR;
}

/*******************************************************************************
函数说明: 照片照片下行任务
入口参数:
        CamDownloadObj_t * 结构体指针
返回值:  无
*******************************************************************************/
void ImageDownloadTask(void *pvParameters)
{
    CamDownloadObj_t *p = (CamDownloadObj_t *)pvParameters;
    ImageHead_t head;

    uint8_t *pbuffer = (uint8_t *)qb50Malloc(CAM_PACK_SIZE + CAM_PACK_HEAD_SIZE);
    if(pbuffer == NULL)
        goto error_state1;

    if(p->Opt.is_sd)
    {
        sprintf(Path, "0:picture-%u/ImageInfo.bin", p->ImageId);
        if(f_open(&FileHandle, Path, FA_READ) != FR_OK)
        {
            driver_debug(DEBUG_CAMERA, "Camera Open %s Failure!!\r\n",Path);
            goto error_state2;
        }
        /* 读出数据 */
        f_read(&FileHandle, pbuffer, 6, &nByteRead);
        if(nByteRead != 6)
        {
            driver_debug(DEBUG_CAMERA, "Camera Read %s Failure!!\r\n",Path);
            goto error_state3;
        }
        f_close(&FileHandle);

        //下行文件
        SendDownCmd(pbuffer, 6);

        /* 获取图像信息，为后续下传做准备 */
        head.ImageId = ((ImageHead_t *)pbuffer)->ImageId;
        head.LastPacketId = ((ImageHead_t *)pbuffer)->LastPacketId;
        head.LastPacketSize = ((ImageHead_t *)pbuffer)->LastPacketSize;

        if(p->Opt.is_single)//如果为单包传输
        {
            sprintf(Path, "0:picture-%u/%u.bin", p->ImageId, p->PacketId);
            if(f_open(&FileHandle, Path, FA_READ) != FR_OK)
            {
                driver_debug(DEBUG_CAMERA, "Camera Open %s Failure!!\r\n",Path);
                goto error_state2;
            }

            /* 如果不是最后一包数据，则每包400字节 */
            if(p->PacketId != head.LastPacketId)
            {
                f_read(&FileHandle, &pbuffer[4], CAM_PACK_SIZE, &nByteRead);
                if(nByteRead != CAM_PACK_SIZE)
                {
                    driver_debug(DEBUG_CAMERA, "Camera Read %s Failure!!\r\n",Path);
                    goto error_state3;
                }
                f_close(&FileHandle);

                /* 附加包ID，包数据长和累加和校验 */
                *(uint16_t *)pbuffer = p->PacketId;
                *(uint16_t *)&pbuffer[2] = (uint16_t)CAM_PACK_SIZE;
                pbuffer[404] = sum_check(pbuffer, 404);
                /* 下行数据包 */
                SendDownCmd(pbuffer, 406);
            }
            /* 如果是最后一包数据 */
            else
            {
                /* 读出最后一包数据 */
                f_read(&FileHandle, &pbuffer[4], head.LastPacketSize, &nByteRead);
                if(nByteRead != head.LastPacketSize)
                {
                    driver_debug(DEBUG_CAMERA, "Camera Read %s Failure!!\r\n",Path);
                    goto error_state3;
                }
                f_close(&FileHandle);

                /* 附加包ID，包数据长和累加和校验 */
                *(uint16_t *)pbuffer = head.LastPacketId;
                *(uint16_t *)&pbuffer[2] = head.LastPacketSize;
                pbuffer[head.LastPacketSize+4] = sum_check(pbuffer, head.LastPacketSize+4);

                /* 下行数据包 */
                SendDownCmd(pbuffer, head.LastPacketSize+6);
            }
        }
        /* 如果为整幅图像下传 */
        else
        {
            for(uint16_t i=0; i<head.LastPacketId; i++)
            {
                sprintf(Path, "0:picture-%u/%u.bin", p->ImageId, i);
                /* 打开文件 */
                if(f_open(&FileHandle, Path, FA_READ) != FR_OK)
                {
                    driver_debug(DEBUG_CAMERA, "Camera Open %s Failure!!\r\n",Path);
                    goto error_state2;
                }
                /* 读取文件 */
                f_read(&FileHandle, &pbuffer[4], CAM_PACK_SIZE, &nByteRead);
                if(nByteRead != CAM_PACK_SIZE)
                {
                    driver_debug(DEBUG_CAMERA, "Camera Read %s Failure!!\r\n",Path);
                    goto error_state3;
                }
                /* 关闭文件 */
                f_close(&FileHandle);

                /* 附加包ID，包数据长和累加和校验 */
                *(uint16_t *)pbuffer = i;
                *(uint16_t *)&pbuffer[2] = (uint16_t)CAM_PACK_SIZE;
                pbuffer[404] = sum_check(pbuffer, 404);

                /* 下行数据包 */
                SendDownCmd(pbuffer, 406);
            }

            /* 最后一包数据传输 */
            sprintf(Path, "0:picture-%u/%u.bin", p->ImageId, head.LastPacketId);
            /* 打开文件 */
            if(f_open(&FileHandle, Path, FA_READ) != FR_OK)
            {
                driver_debug(DEBUG_CAMERA, "Camera Open %s Failure!!\r\n",Path);
                goto error_state2;
            }
            /* 读取文件 */
            f_read(&FileHandle, &pbuffer[4], head.LastPacketSize, &nByteRead);
            if(nByteRead != head.LastPacketSize)
            {
                driver_debug(DEBUG_CAMERA, "Camera Read %s Failure!!\r\n",Path);
                goto error_state3;
            }
            /* 关闭文件 */
            f_close(&FileHandle);

            /* 附加包ID，包数据长和累加和校验 */
            *(uint16_t *)pbuffer = head.LastPacketId;
            *(uint16_t *)&pbuffer[2] = head.LastPacketSize;
            pbuffer[head.LastPacketSize+4] = sum_check(pbuffer, head.LastPacketSize+4);

            /* 下行数据包 */
            SendDownCmd(pbuffer, head.LastPacketSize+6);
        }
    }
    /* 如果需要下行NorFlash的内容 */
    else
    {
        /* 读出图像信息 */
        FSMC_NOR_ReadBuffer((uint16_t *)pbuffer, IMAGE_FLASH_STORE_BASE, 3);

        /* 暂存图像信息 */
        head.ImageId = ((ImageHead_t *)pbuffer)->ImageId;
        head.LastPacketId = ((ImageHead_t *)pbuffer)->LastPacketId;
        head.LastPacketSize = ((ImageHead_t *)pbuffer)->LastPacketSize;

        /* 下行图像信息 */
        SendDownCmd(pbuffer, 6);
        /* 如果为单包传输 */
        if(p->Opt.is_single)
        {
            /* 如果需要的数据包不是最后一包数据 */
            if(p->PacketId != head.LastPacketId)
            {
                /* 从Flash中读此ID的数据包，共400字节 */
                FSMC_NOR_ReadBuffer((uint16_t *)&pbuffer[4], IMAGE_FLASH_STORE_BASE+3+p->PacketId*200, 200);

                /* 附加包ID，包数据长和累加和校验 */
                *(uint16_t *)pbuffer = p->PacketId;
                *(uint16_t *)&pbuffer[2] = (uint16_t)CAM_PACK_SIZE;
                pbuffer[404] = sum_check(pbuffer, 404);

                /* 下行数据包 */
                SendDownCmd(pbuffer, 406);
            }
            /* 如果需要的数据包是最后一包数据 */
            else
            {
                /* 从Flash中读此ID的数据包，共400字节 */
                FSMC_NOR_ReadBuffer((uint16_t *)&pbuffer[4], IMAGE_FLASH_STORE_BASE+3+head.LastPacketId*200,
                        head.LastPacketSize / 2 + head.LastPacketSize % 2);

                /* 附加包ID，包数据长和累加和校验 */
                *(uint16_t *)pbuffer = head.LastPacketId;
                *(uint16_t *)&pbuffer[2] = head.LastPacketSize;
                pbuffer[head.LastPacketSize+4] = sum_check(pbuffer, head.LastPacketSize+4);

                /* 下行数据包 */
                SendDownCmd(pbuffer, head.LastPacketSize+6);
            }
        }
        /* 如果是整包传送 */
        else
        {
            for(uint16_t i=0; i<head.LastPacketId; i++)
            {
                /* 读取数据包 */
                FSMC_NOR_ReadBuffer((uint16_t *)&pbuffer[4], IMAGE_FLASH_STORE_BASE+3+i*200, 200);

                /* 附加包ID，包数据长和累加和校验 */
                *(uint16_t *)pbuffer = i;
                *(uint16_t *)&pbuffer[2] = (uint16_t)CAM_PACK_SIZE;
                pbuffer[404] = sum_check(pbuffer, 404);

                /* 下行数据包 */
                SendDownCmd(pbuffer, 406);
            }

            /* 读取最后一包数据 */
            FSMC_NOR_ReadBuffer((uint16_t *)&pbuffer[4], IMAGE_FLASH_STORE_BASE+3+head.LastPacketId*200,
                    head.LastPacketSize / 2 + head.LastPacketSize % 2);

            /* 附加包ID，包数据长和累加和校验 */
            *(uint16_t *)pbuffer = head.LastPacketId;
            *(uint16_t *)&pbuffer[2] = head.LastPacketSize;
            pbuffer[head.LastPacketSize+4] = sum_check(pbuffer, head.LastPacketSize+4);

            /* 下行数据包 */
            SendDownCmd(pbuffer, head.LastPacketSize+6);
        }
    }

    goto error_state2;

    error_state3: f_close(&FileHandle);
    error_state2: qb50Free(pbuffer);
    error_state1: vTaskDelete(NULL);
}

void ImageDownloadStart(CamDownloadObj_t para)
{
    tPara = para;
    pname[3]++;
    taskENTER_CRITICAL();
    xTaskCreate(ImageDownloadTask, pname, IMAGE_DOWN_TASK_STK, &tPara,
            IMAGE_DOWN_TASK_PRIO, ( TaskHandle_t * ) NULL);
    taskEXIT_CRITICAL();
}

/*******************************************************************************
函数说明: 照片信息下行
入口参数:
        IsFlash: 1为选择NorFlash 0为TF卡
        ImageId: 图像id
返回值:  函数执行结果：非0为错误，0为无误
/******************************************************************************/
int xImageInfoDownload(uint8_t IsFlash, uint16_t ImageId)
{
    uint8_t *pbuffer = (uint8_t *)qb50Malloc(6);
    /*下行Flash图像信息*/
    if(IsFlash)
    {
        /* 读出图像信息 */
        FSMC_NOR_ReadBuffer((uint16_t *)pbuffer, IMAGE_FLASH_STORE_BASE, 3);
        /* 下行图像信息 */
        SendDownCmd(pbuffer, 6);
    }
    /*否则下行TF中图像信息*/
    else
    {
        /* 打开文件 */
         sprintf(Path, "0:picture-%u/ImageInfo.bin", ImageId);
         if(f_open(&FileHandle, Path, FA_READ) != FR_OK)
         {
             driver_debug(DEBUG_CAMERA, "Camera Open %s Failure!!\r\n",Path);
             qb50Free(pbuffer);
             return E_NO_SS;
         }

         /* 读出数据 */
         f_read(&FileHandle, pbuffer, 6, &nByteRead);
         if(nByteRead != 6)
         {
             driver_debug(DEBUG_CAMERA, "Camera Read %s Failure!!\r\n",Path);
             f_close(&FileHandle);
             qb50Free(pbuffer);
             return E_NO_SS;
         }
         /* 关闭文件 */
         f_close(&FileHandle);

         //下行照片信息数据
         SendDownCmd(pbuffer, 6);
    }

    qb50Free(pbuffer);

    return 0;
}
/*******************************************************************************
函数说明: 照片下载
入口参数:
        opt：最低位0为NorFlash 1为TF卡 次低位0
        ImageId:照片id
        PacketId:包id
返回值:  函数执行结果
/******************************************************************************/
int xImageDownload(cam_opt opt, uint16_t ImageId, uint16_t PacketId)
{
    ImageHead_t head;

    uint8_t *pbuffer = (uint8_t *)qb50Malloc(CAM_PACK_SIZE + CAM_PACK_HEAD_SIZE);
    if(pbuffer == NULL)
        return E_MALLOC_FAIL;

    if(opt.is_sd) //如果为TF卡
    {
        /* 打开文件 */
        sprintf(Path, "0:picture-%u/ImageInfo.bin", ImageId);
        if(f_open(&FileHandle, Path, FA_READ) != FR_OK)
        {
            driver_debug(DEBUG_CAMERA, "Camera Open %s Failure!!\r\n",Path);
            qb50Free(pbuffer);
            return E_NO_SS;
        }

        /* 读出数据 */
        f_read(&FileHandle, pbuffer, 6, &nByteRead);
        if(nByteRead != 6)
        {
            driver_debug(DEBUG_CAMERA, "Camera Read %s Failure!!\r\n",Path);
            f_close(&FileHandle);
            qb50Free(pbuffer);
            return E_NO_SS;
        }
        /* 关闭文件 */
        f_close(&FileHandle);

        /* 保存图像信息 */
        head.ImageId = ((ImageHead_t *)pbuffer)->ImageId;
        head.LastPacketId = ((ImageHead_t *)pbuffer)->LastPacketId;
        head.LastPacketSize = ((ImageHead_t *)pbuffer)->LastPacketSize;

        if(opt.is_single)//如果为单包传输
        {
            sprintf(Path, "0:picture-%u/%u.bin", ImageId, PacketId);
            if(f_open(&FileHandle, Path, FA_READ) != FR_OK)
            {
                driver_debug(DEBUG_CAMERA, "Camera Open %s Failure!!\r\n",Path);
                qb50Free(pbuffer);
                return E_NO_SS;
            }

            /* 如果不是最后一包数据，则每包400字节 */
            if(PacketId != head.LastPacketId)
            {
                /*读出此包数据*/
                f_read(&FileHandle, &pbuffer[4], CAM_PACK_SIZE, &nByteRead);

                if(nByteRead != CAM_PACK_SIZE)
                {
                    driver_debug(DEBUG_CAMERA, "Camera Read %s Failure!!\r\n",Path);
                    f_close(&FileHandle);
                    qb50Free(pbuffer);
                    return E_NO_SS;
                }
                /* 关闭文件 */
                f_close(&FileHandle);

                /* 附加包ID，包数据长和累加和校验 */
                *(uint16_t *)pbuffer = PacketId;
                *(uint16_t *)&pbuffer[2] = (uint16_t)CAM_PACK_SIZE;
                pbuffer[404] = sum_check(pbuffer, 404);
                /* 下行数据包 */
                SendDownCmd(pbuffer, 406);
            }
            /* 如果是最后一包数据 */
            else
            {

                f_read(&FileHandle, &pbuffer[4], head.LastPacketSize, &nByteRead);
                if(nByteRead != head.LastPacketSize)
                {
                    driver_debug(DEBUG_CAMERA, "Camera Read %s Failure!!\r\n",Path);
                    f_close(&FileHandle);
                    qb50Free(pbuffer);
                    return E_NO_SS;
                }
                f_close(&FileHandle);

                /* 附加包ID，包数据长和累加和校验 */
                *(uint16_t *)pbuffer = head.LastPacketId;
                *(uint16_t *)&pbuffer[2] = head.LastPacketSize;
                pbuffer[head.LastPacketSize+4] = sum_check(pbuffer, head.LastPacketSize+4);

                /* 下行数据包 */
                SendDownCmd(pbuffer, head.LastPacketSize+6);
            }
        }
        /* 如果为整幅图像下传 */
        else
        {
            for(uint16_t i=0; i<head.LastPacketId; i++)
            {
                sprintf(Path, "0:picture-%u/%u.bin", ImageId, i);
                /* 打开文件 */
                if(f_open(&FileHandle, Path, FA_READ) != FR_OK)
                {
                    driver_debug(DEBUG_CAMERA, "Camera Open %s Failure!!\r\n",Path);
                    qb50Free(pbuffer);
                    return E_NO_SS;
                }

                /* 读取文件 */
                f_read(&FileHandle, &pbuffer[4], CAM_PACK_SIZE, &nByteRead);
                if(nByteRead != CAM_PACK_SIZE)
                {
                    driver_debug(DEBUG_CAMERA, "Camera Read %s Failure!!\r\n",Path);
                    f_close(&FileHandle);
                    qb50Free(pbuffer);
                    return E_NO_SS;
                }

                /* 关闭文件 */
                f_close(&FileHandle);

                /* 附加包ID，包数据长和累加和校验 */
                *(uint16_t *)pbuffer = i;
                *(uint16_t *)&pbuffer[2] = (uint16_t)CAM_PACK_SIZE;
                pbuffer[404] = sum_check(pbuffer, 404);

                /* 下行数据包 */
                SendDownCmd(pbuffer, 406);
            }

            /* 最后一包数据传输 */
            sprintf(Path, "0:picture-%u/%u.bin", ImageId, head.LastPacketId);
            /* 打开文件 */
            if(f_open(&FileHandle, Path, FA_READ) != FR_OK)
            {
                driver_debug(DEBUG_CAMERA, "Camera Open %s Failure!!\r\n",Path);
                qb50Free(pbuffer);
                return E_NO_SS;
            }
            /* 读取文件 */
            f_read(&FileHandle, &pbuffer[4], head.LastPacketSize, &nByteRead);
            if(nByteRead != head.LastPacketSize)
            {
                driver_debug(DEBUG_CAMERA, "Camera Read %s Failure!!\r\n",Path);
                f_close(&FileHandle);
                qb50Free(pbuffer);
                return E_NO_SS;
            }
            /* 关闭最后一包数据文件 */
            f_close(&FileHandle);

            /* 附加包ID，包数据长和累加和校验 */
            *(uint16_t *)pbuffer = head.LastPacketId;
            *(uint16_t *)&pbuffer[2] = head.LastPacketSize;
            pbuffer[head.LastPacketSize+4] = sum_check(pbuffer, head.LastPacketSize+4);

            /* 下行数据包 */
            SendDownCmd(pbuffer, head.LastPacketSize+6);
        }
    }
    else //如果为NorFlash
    {
        /* 读出图像信息 */
        FSMC_NOR_ReadBuffer((uint16_t *)pbuffer, IMAGE_FLASH_STORE_BASE, 3);

        /* 暂存图像信息 */
        head.ImageId = ((ImageHead_t *)pbuffer)->ImageId;
        head.LastPacketId = ((ImageHead_t *)pbuffer)->LastPacketId;
        head.LastPacketSize = ((ImageHead_t *)pbuffer)->LastPacketSize;

        /* 如果为单包传输 */
        if(opt.is_single)
        {
            /* 如果需要的数据包不是最后一包数据 */
            if(PacketId != head.LastPacketId)
            {
                /* 从Flash中读此ID的数据包，共400字节 */
                FSMC_NOR_ReadBuffer((uint16_t *)&pbuffer[4], IMAGE_FLASH_STORE_BASE+3+PacketId*200, 200);

                /* 附加包ID，包数据长和累加和校验 */
                *(uint16_t *)pbuffer = PacketId;
                *(uint16_t *)&pbuffer[2] = (uint16_t)CAM_PACK_SIZE;
                pbuffer[404] = sum_check(pbuffer, 404);
                /* 下行数据包 */
                SendDownCmd(pbuffer, 406);
            }
            /* 如果需要的数据包是最后一包数据 */
            else
            {
                /* 从Flash中读此ID的数据包，共400字节 */
                FSMC_NOR_ReadBuffer((uint16_t *)&pbuffer[4], IMAGE_FLASH_STORE_BASE+3+head.LastPacketId*200,
                        head.LastPacketSize / 2 + head.LastPacketSize % 2);

                /* 附加包ID，包数据长和累加和校验 */
                *(uint16_t *)pbuffer = head.LastPacketId;
                *(uint16_t *)&pbuffer[2] = head.LastPacketSize;
                pbuffer[head.LastPacketSize+4] = sum_check(pbuffer, head.LastPacketSize+4);

                /* 下行数据包 */
                SendDownCmd(pbuffer, head.LastPacketSize+6);
            }
        }
        /* 如果是整包传送 */
        else
        {
            for(uint16_t i=0; i<head.LastPacketId; i++)
            {
                /* 读取数据包 */
                FSMC_NOR_ReadBuffer((uint16_t *)&pbuffer[4], IMAGE_FLASH_STORE_BASE+3+i*200, 200);

                /* 附加包ID，包数据长和累加和校验 */
                *(uint16_t *)pbuffer = i;
                *(uint16_t *)&pbuffer[2] = (uint16_t)CAM_PACK_SIZE;
                pbuffer[404] = sum_check(pbuffer, 404);

                /* 下行数据包 */
                SendDownCmd(pbuffer, 406);
            }

            /* 读取最后一包数据 */
            FSMC_NOR_ReadBuffer((uint16_t *)&pbuffer[4], IMAGE_FLASH_STORE_BASE+3+head.LastPacketId*200,
                    head.LastPacketSize / 2 + head.LastPacketSize % 2);

            /* 附加包ID，包数据长和累加和校验 */
            *(uint16_t *)pbuffer = head.LastPacketId;
            *(uint16_t *)&pbuffer[2] = head.LastPacketSize;
            pbuffer[head.LastPacketSize+4] = sum_check(pbuffer, head.LastPacketSize+4);

            /* 下行数据包 */
            SendDownCmd(pbuffer, head.LastPacketSize+6);
        }
    }
    qb50Free(pbuffer);
    return 0;
}




