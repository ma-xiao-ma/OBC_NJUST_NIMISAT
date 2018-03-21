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

/** 相机开关控制*/
typedef enum
{
    cam_sw_off = 0,
    cam_sw_on
} cam_sw_status;

/**存储区域选择*/
typedef enum
{
    mem_sd = 0xAA,
    mem_flash = 0xCC
} cam_mem_region;

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


typedef struct __attribute__((__packed__))
{
    uint32_t    exp_time;
    uint8_t     gain;
    uint8_t     need_erase;
    work_mode   mode;
    uint16_t    record_last;
} CamModeSet_t;

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
int Camera_Power_Off(void);

/**
 * 数传板供电控制函数
 *
 * @return 返回E_NO_ERR（-1）为正常
 */
int cam_power_switch( cam_sw_status power_sw );

/**
 * 相机加热控制开关
 *
 * @param heat_sw 0为关， 非0为开
 * @return EPS_OK（0）为执行成功
 */
int cam_heat2_switch( cam_sw_status heat_sw );

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

/**
 * 图像模式1fps相机 数传机工作流程
 *
 * @param exp_time 相机曝光时间设置
 * @param gain 相机增益设置
 * @param need_erase 数传机固存是否需要擦除 （非0为需要擦除，0为不需要擦除）
 * @param record_last 数传记录模式持续时间， 单位：秒
 *
 * @return 返回E_NO_ERR（-1）为正确
 */
int Image_1fps_Mode_Process(uint32_t exp_time, uint8_t gain, uint8_t need_erase, uint16_t record_last);

/**
 * 视频模式  相机数传机工作流程
 *
 * @param exp_time 相机曝光时间设置
 * @param gain 相机增益设置
 * @param need_erase 数传机固存是否需要擦除 （非0为需要擦除，0为不需要擦除）
 * @param record_last 数传记录模式持续时间， 单位：秒
 *
 * @return 返回E_NO_ERR（-1）为正确
 */
int Video_Mode_Process(uint32_t exp_time, uint8_t gain, uint8_t need_erase, uint16_t record_last);

/**
 * 图像模式RAW 相机数传机工作流程
 *
 * @param exp_time 相机曝光时间设置
 * @param gain 相机增益设置
 * @param need_erase 数传机固存是否需要擦除 （非0为需要擦除，0为不需要擦除）
 * @param record_last 数传记录模式持续时间， 单位：秒
 *
 * @return 返回E_NO_ERR（-1）为正确
 */
int Image_Raw_Mode_Process(uint32_t exp_time, uint8_t gain, uint8_t need_erase, uint16_t record_last);

/**
 * 相机数传工作模式设置，创建任务模式
 *
 * @param exp_time 相机曝光时间设置
 * @param gain 相机增益设置
 * @param need_erase 数传固存擦除选项
 * @param mode work_mode 枚举类型
 * @param record_last 数传机回放持续时间
 *
 * @return 返回E_NO_ERR（-1）任务创建成功
 */
int Cam_DTB_Work(uint32_t exp_time, uint8_t gain, uint8_t need_erase, work_mode mode, uint16_t record_last);


/***********************图像下行相关函数*************************/

/**
 * 下行特定ID图像信息
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_img_info_down( uint8_t mem_region, uint32_t id, uint8_t down_cnt );

/**
 * 图像信息下行
 *
 * @param mem_region 存储区域选择 0xAA--SD卡  0xCC--FLASH
 * @param id 图像编号 若图像ID参数为0xFFFF则下行最新照片的照片信息，否则下行指定ID照片信息
 * @param down_cnt 单包下行次数，不超过10次
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int Image_Info_Down( uint32_t id, uint8_t mem_region, uint8_t down_cnt );

/**
 * 相机下行指定ID照片
 *
 * @param id 照片ID
 * @param mem_region 存储区域选择 0xAA--SD卡  0xCC--FLASH
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_img_data_down( uint32_t id, uint8_t mem_region );

/**
 * 从起始包号start_packet开始下行图像数据
 *
 * @param id 图像ID号
 * @param start_packet 起始包号
 * @param mem_region 存储区域选择 0xAA--SD卡  0xCC--FLASH
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_img_data_packet_down( uint32_t id, uint16_t start_packet, uint8_t mem_region );

/**
 * 下行ID编号图像的第packet包图像数据
 *
 * @param id 图像ID
 * @param packet 图像包号
 * @param mem_region 存储区域选择 0xAA--SD卡  0xCC--FLASH
 * @param down_cnt 单包下行次数     不超过10次
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_img_packet_down( uint32_t id, uint16_t packet, uint8_t mem_region, uint8_t down_cnt );

#endif /* CONTRL_CAMERA_805_H_ */
