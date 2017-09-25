/*
 * camera_805.h
 *
 *  Created on: 2017年6月17日
 *      Author: Ma Wenli
 */
#ifndef CONTRL_CAMERA_805_H_
#define CONTRL_CAMERA_805_H_

#include <stdint.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ff.h"

#define camDATA_SRORE_IN_FLASH      1
#define camDATA_SRORE_IN_SD         1

#define TX_BUFFER_SIZE              6
#define RX_BUFFER_SIZE              (3*64*1024)

#define CAM_PACK_SIZE               400
#define CAM_PACK_HEAD_SIZE          6
#define IMAGE_FLASH_STORE_BASE      32768

#define CAMERA_TX_ISR_HANDLER           DMA2_Stream7_IRQHandler
#define CAMERA_RX_ISR_HANDLER           USART1_IRQHandler
#define CAMERA_TX_ISR_CHANEL            DMA2_Stream7_IRQn
#define CAMERA_RX_ISR_CHANEL            USART1_IRQn
#define CAMERA_PORT_NAME                USART1
#define CAMERA_TX_PORT                  GPIOB
#define CAMERA_RX_PORT                  GPIOB
#define CAMERA_TX_PORTCLK               RCC_AHB1Periph_GPIOB
#define CAMERA_RX_PORTCLK               RCC_AHB1Periph_GPIOB
#define CAMERA_SCLK                     RCC_APB2Periph_USART1
#define CAMERA_RX_PIN                   GPIO_Pin_7
#define CAMERA_TX_PIN                   GPIO_Pin_6
#define CAMERA_EXTI_IRQn                USART1_IRQn
#define CAMERA_TX_AF                    GPIO_AF_USART1
#define CAMERA_TX_SOURCE                GPIO_PinSource6
#define CAMERA_RX_AF                    GPIO_AF_USART1
#define CAMERA_RX_SOURCE                GPIO_PinSource7
#define CAMERA_DMA_CLK                  RCC_AHB1Periph_DMA2
#define CAMERA_DMA_RX_STREAM            DMA2_Stream2
#define CAMERA_DMA_TX_STREAM            DMA2_Stream7
#define CAMERA_DMA_CHANEL               DMA_Channel_4
#define CAMERA_ACCESS_TIMEOUT           (portTickType)1000
#define CAMERA_CreateImage_TIMEOUT      (portTickType)1000
#define CAMERA_ImagePacket_TIMEOUT      (portTickType)1000

#define IMAGE_DOWN_TASK_PRIO            4
#define IMAGE_DOWN_TASK_STK             256

typedef struct __attribute__((__packed__))
{
        uint32_t ImageTime;     //拍照时间
        float ImageLocation[3]; //拍照位置
        uint32_t ImageSize;     //图像大小
        uint16_t ImageID;       //图像ID
        uint16_t TotalPacket;   //图像包总数
        uint8_t PacketSize;     //数据包大小
        uint8_t LastPacketSize; //尾包大小
} ImageInfo_t;

typedef struct __attribute__((__packed__))
{
        uint8_t SendBuffer[TX_BUFFER_SIZE];/* 相机指令发送缓冲区 */
        uint8_t ReceiveBuffer[RX_BUFFER_SIZE];/* 照片数据流缓冲区 3 * 64KB */
        SemaphoreHandle_t AccessMutexSem;/* 相机访问互斥信号量 */
        SemaphoreHandle_t SynchBinSem;/* 数据接收完成中断同步信号量 */
        uint32_t Rxlen;/* 串口DMA接收数据长度 */
} CamTrans_t;

typedef union {
    unsigned char ext;
    struct __attribute__((__packed__)) {
        unsigned char is_sd : 1;        //1为下载TF卡中的图像数据，0为下载norflash中的图像数据
        unsigned char is_single : 1;    //1为下载单包数据，0为下载整幅图像
        unsigned char reserved : 6;     //保留
    };
} cam_opt;

typedef struct __attribute__((__packed__))
{
    cam_opt Opt;
    unsigned short ImageId;
    unsigned short PacketId;
} CamDownloadObj_t;

typedef struct __attribute__((__packed__))
{
    unsigned short ImageId;
    unsigned short LastPacketId;
    unsigned short LastPacketSize;
} ImageHead_t;




void Camera_805_Init(void);
uint8_t Camera_805_reset(void);
uint16_t Camera_Temp_Get(uint8_t Point);
uint32_t Camera_Exposure_Time_Read(void);
uint32_t Camera_Exposure_Time_Set(uint32_t data);
uint8_t Camera_Gain_Get(void);
uint8_t Camera_Gain_Set(uint8_t Coefficient);
uint32_t Camera_Work_Mode_Get(void);
uint32_t Camera_Work_Mode_Set(uint8_t TransMode,
                              uint8_t PicMode,
                              uint8_t AutoExpo,
                              uint16_t id);
int xImageInfoDownload(uint8_t IsFlash, uint16_t ImageId);
int xImageDownload(cam_opt opt, uint16_t ImageId, uint16_t PacketId);

#endif /* CONTRL_CAMERA_805_H_ */
