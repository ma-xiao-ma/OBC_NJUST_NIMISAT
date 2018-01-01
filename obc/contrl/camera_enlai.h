/*
 * camera_enlai.h
 *
 *  Created on: 2017年6月17日
 *      Author: Liu Yadong
 */
#ifndef CONTRL_CAMERA_ENLAI_H_
#define CONTRL_CAMERA_ENLAI_H_

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define IMAGE_PACK_MAX_SIZE         220


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


//#define CAMERA_RECORD_ADDR          ((uint32_t)(0x080E0000))
//#define CAMERA_TIME_ADDRESS         ((uint32_t)(0x080A0000))     //存放记录拍照次数的地址，存放于norflash中，片满会被清零







#define TP_STORE_OK              0

/*error flags*/

#define ERR_SENDTP               1
#define ERR_REC_TAKEPICTURE      2
#define ERR_REC_TAKEPACK         3
#define ERR_CRC                  4
#define ERR_POWERUP              5
#define ERR_ERASE_NORFLASH       6


typedef struct __attribute__((packed)) {
	uint32_t     ImageSize;         //图片的大小
	uint32_t     ImageDddress;      //相片存储地址
	uint32_t     ImageTime;         //相片的拍摄时间
    uint16_t     TotalPacket;       //相片的分包数量,220每包大小
	uint8_t      ImageID;           //图片的编号（所拍的第几张相片）,一直递增
	uint8_t		 erase_period;      //norflash写满一次，递增1
	uint8_t      PacketSize;        //数据包大小
	uint8_t      LastPacketSize;    //尾包大小
}ImageInfo_el_t;                    //图片信息存储

typedef struct __attribute__((__packed__))
{
    uint16_t PacketID;
    uint8_t PacketSize;
    uint8_t ImageData[IMAGE_PACK_MAX_SIZE];
} ImagePacket_enlai_t;


int take_store_picture(uint8_t picture_size);
void Camera_Enlai_Usart_Init(uint32_t baudrate);
int init_camera_address();
int init_enlai_camera(uint16_t erase_period);


int cam_flash_enlaiimg_lll_down(uint8_t id, uint16_t packet);
int cam_flash_enlaiimg_all_down(uint8_t id);
int cam_flash_enlaiimg_packet_down(uint8_t id, uint16_t packet);
int cam_flash_enlaiimg_info_down(uint8_t id);
int cam_sd_enlaiimg_lll_down(uint8_t erase_period, uint8_t id, uint16_t packet);
int cam_sd_enlaiimg_all_down(uint8_t erase_period, uint8_t id);
int cam_sd_enlaiimg_packet_down(uint8_t erase_period, uint8_t id, uint16_t packet);
int cam_sd_enlaiimg_info_down(uint8_t erase_period, uint8_t id);

int cam_enlaiimg_info_down(uint8_t erase_period, uint8_t id, uint8_t source);
int cam_enlaiimg_data_down(uint8_t erase_period, uint8_t id, uint8_t source);
int cam_enlaiimg_packet_down(uint8_t erase_period, uint8_t id, uint16_t packet, uint8_t source);
int cam_enlaiimg_data_packet_down(uint8_t erase_period, uint8_t id, uint16_t start_packet, uint8_t source);

//void Camera_Tx_DMA_Config(uint32_t mar,uint16_t ndtr);
//void Camera_Rx_DMA_Config(uint32_t mar,uint16_t ndtr);
//void Camera_Tx_DMA_Enable(uint16_t ndtr);
//void Camera_Rx_DMA_Enable(uint16_t ndtr);

/**
 * 恩来拍照任务
 *
 * @param pic_size 照片大小
 * @return E_NO_ERR(-1)为任务创建成功
 */
int enlai_take_pic_task(uint8_t pic_size);

void close_camera(void);

#endif /* CONTRL_CAMERA_ENLAI_H_ */
