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

#define TX_BUFFER_SIZE              6
#define RX_BUFFER_SIZE              (4*64*1024)

#define IMAGE_PACK_MAX_SIZE         220
#define IMAGE_PACK_HEAD_SIZE        3

#define CAMERA_TX_ISR_HANDLER       DMA2_Stream7_IRQHandler
#define CAMERA_RX_ISR_HANDLER       USART1_IRQHandler
#define CAMERA_TX_ISR_CHANEL        DMA2_Stream7_IRQn
#define CAMERA_RX_ISR_CHANEL        USART1_IRQn
#define CAMERA_PORT_NAME            USART1
#define CAMERA_TX_PORT              GPIOB
#define CAMERA_RX_PORT              GPIOB
#define CAMERA_TX_PORTCLK           RCC_AHB1Periph_GPIOB
#define CAMERA_RX_PORTCLK           RCC_AHB1Periph_GPIOB
#define CAMERA_SCLK                 RCC_APB2Periph_USART1
#define CAMERA_RX_PIN               GPIO_Pin_7
#define CAMERA_TX_PIN               GPIO_Pin_6
#define CAMERA_EXTI_IRQn            USART1_IRQn
#define CAMERA_TX_AF                GPIO_AF_USART1
#define CAMERA_TX_SOURCE            GPIO_PinSource6
#define CAMERA_RX_AF                GPIO_AF_USART1
#define CAMERA_RX_SOURCE            GPIO_PinSource7
#define CAMERA_DMA_CLK              RCC_AHB1Periph_DMA2
#define CAMERA_DMA_RX_STREAM        DMA2_Stream2
#define CAMERA_DMA_TX_STREAM        DMA2_Stream7
#define CAMERA_DMA_CHANEL           DMA_Channel_4
#define CAMERA_ACCESS_TIMEOUT       (portTickType)1000
#define CAMERA_CreateImage_TIMEOUT  (portTickType)1000
#define CAMERA_ImagePacket_TIMEOUT  (portTickType)1000

#define IMAGE_DOWN_TASK_PRIO            4
#define IMAGE_DOWN_TASK_STK             256


typedef struct __attribute__((__packed__))
{
    uint32_t ImageID;       //图像ID
    uint32_t ImageSize;     //图像大小
    uint16_t TotalPacket;   //图像包总数
    uint8_t PacketSize;     //数据包大小
    uint8_t LastPacketSize; //尾包大小
    uint32_t ImageTime;     //拍照时间
    float ImageLocation[3]; //拍照位置
} ImageInfo_t;

typedef struct __attribute__((__packed__))
{
        uint8_t SendBuffer[TX_BUFFER_SIZE];/* 相机指令发送缓冲区 */
        uint8_t ReceiveBuffer[RX_BUFFER_SIZE];/* 照片数据流缓冲区 4 * 64KB */
        SemaphoreHandle_t AccessMutexSem;/* 相机访问互斥信号量 */
        SemaphoreHandle_t SynchBinSem;/* 数据接收完成中断同步信号量 */
        uint32_t Rxlen;/* 串口DMA接收数据长度 */
} CamTrans_t;

/**传输方式*/
typedef enum __attribute__((__packed__))
{
    TransOff = 0,
    LVDS,
    TTL
} trans_mode;

/**工作模式*/
typedef enum __attribute__((__packed__))
{
    ImageRaw = 0,
    Image1fps,
    Video,
    Backup
} work_mode;

/**曝光模式*/
typedef enum __attribute__((__packed__))
{
    AutoExpoOn = 0,
    AutoExpoOff
} expo_mode;

/** 相机控制模式 */
typedef struct __attribute__((__packed__))
{
    trans_mode tran;
    work_mode mode;
    expo_mode expo;
} cam_ctl_t;

/**相机模式*/
typedef struct __attribute__((__packed__))
{
    uint8_t expo_mode :1; //bit0
    uint8_t work_mode :2; //bit1~bit2
    uint8_t trans_mode :2; //bit3~bit4
    uint8_t padding :3; //bit5~bit7
} cam_mode_t;


typedef struct __attribute__((__packed__))
{
    uint32_t falsh_sector;
    uint32_t image_id;
} cam_flash;

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
    uint16_t PacketID;
    uint8_t PacketSize;
    uint8_t ImageData[IMAGE_PACK_MAX_SIZE];
} ImagePacket_t;

/***********************相机控制相关函数*************************/

/**
 * 相机两路5V加电
 * @return E_NO_ERR（-1）为正常
 */
int Camera_Power_On(void);

/**
 * 相机两路5V断电
 *
 */
void Camera_Power_Off(void);

/**
 * 相机初始化
 */
void Camera_805_Init(void);

/**
 * 相机复位指令
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_805_reset(void);

/**
 * 获取相机三个采温点温度，不能在中断中调用
 *
 * @param Point 采温点1为0x01,采温点2为0x02,采温点3为0x03
 * @param temp Point点的温度输出值指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Temp_Get(uint8_t Point, uint16_t *temp);

/**
 * 读取相机的曝光时间设置值
 *
 * @param exp_time 接收缓冲区指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Exposure_Time_Read(uint32_t *exp_time);

/**
 * 曝光时间设置
 *
 * @param exp_time 曝光时间设置值
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Exposure_Time_Set(uint32_t exp_time);

/**
 * 读取相机已设置增益值
 *
 * @param gain 接收指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Gain_Get(uint8_t *gain);

/**
 * 相机增益设置
 *
 * @param gain 相机增益系数设置值
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Gain_Set(uint8_t gain);

/**
 * 获取相机当前的控制模式
 *
 * @param cam_ctl_mode 接收缓冲区指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Work_Mode_Get(cam_ctl_t *cam_ctl_mode);

/**
 * 相机工作模式设定
 *
 * @param cam_ctl_mode 控制模式结构体
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Camera_Work_Mode_Set(cam_ctl_t cam_ctl_mode);


/***********************图像下行相关函数*************************/

/**
 * 下行最新的照片信息
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_newest_img_info_down(void);

/**
 * 下行特定ID图像信息
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_img_info_down(uint32_t id);

/**
 * 相机下行指定ID照片
 *
 * @param id 照片ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_img_data_down(uint32_t id);

/**
 * 下行ID编号图像的第packet包图像数据
 *
 * @param id 图像ID
 * @param packet 图像包号
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_img_packet_down(uint32_t id, uint16_t packet);

/**
 * 从起始包号start_packet开始下行图像数据
 *
 * @param id 图像ID号
 * @param start_packet 起始包号
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_img_data_packet_down(uint32_t id, uint16_t start_packet);

#endif /* CONTRL_CAMERA_805_H_ */
