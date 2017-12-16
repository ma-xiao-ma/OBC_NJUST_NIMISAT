/*
 * camera_enlai.c
 *
 *  Created on: 2017年10月18日
 *      Author: Liu Ydong
 */

#include "camera_enlai.h"
#include "bsp_pca9665.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "bsp_usart.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "command.h"
#include "bsp_cpu_flash.h"
#include "if_downlink_vu.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"
#include "hk_arg.h"
#include "obc_argvs_save.h"
#include "ff.h"
#include "bsp_switch.h"
#include "driver_debug.h"
#include "error.h"
#include "cube_com.h"
#include "router_io.h"
#include "file_opt.h"


/*全局变量*/
uint8_t image_times[1];         //存储拍照次数变量，flash满时会清零（全局变量，在程序运行时一直有效）
uint32_t image_address;          //相片存储地址

uint8_t  cmd_take_picture[7]={0x55,0x48,0x00,0x32,0x00,0x02,0x23};   /*拍照指令*/
uint8_t  ack_take_picture[14]={0};                               /*存储拍照指令发送后接收到的14个字节的应答数据*/
uint8_t  cmd_get_pack[6]={0x55,0x45,0x00,0x03,0x00,0x23};
uint8_t  ack_get_pack[525]={0};
uint16_t pack_store_norflash[256];                                    //存储到norflash中数组
uint8_t  pack_store_sd[512];

u16 camera_pack_send[110];
u8 camera_pack_send_1[220];



/*标志量*/
uint8_t sendcom_flag=0;        //指令已经发送标志
uint8_t getack_flag=0;         //指定数目的指令已经接收到标志

struct USART_TypeDefStruct Camera_Usart;
xQueueHandle  camera_queue;


void Camera2_Tx_DMA_Config(u32 mar,u16 ndtr)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitU1Struct;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	DMA_DeInit(DMA2_Stream7);
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}
		  /* 閰嶇疆DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = mar;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = ndtr;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
	NVIC_InitU1Struct.NVIC_IRQChannel=DMA2_Stream7_IRQn;
	NVIC_InitU1Struct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitU1Struct.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitU1Struct.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitU1Struct);
}

void Camera2_Rx_DMA_Config(u32 mar,u16 ndtr)
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitU1Struct;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
    DMA_DeInit(DMA2_Stream5);
	while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE){}
	  /* 閰嶇疆DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = mar;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ndtr;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
	NVIC_InitU1Struct.NVIC_IRQChannel=DMA2_Stream5_IRQn;
	NVIC_InitU1Struct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitU1Struct.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitU1Struct.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitU1Struct);
}

void Camera_Enlai_Usart_Init(uint32_t baudrate)
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

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* USART configuration */
    USART_Init(CAMERA_PORT_NAME, &USART_InitStructure);
//    /* NVIC configuration */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_InitStructure.NVIC_IRQChannel = CAMERA_RX_ISR_CHANEL;
//    NVIC_Init(&NVIC_InitStructure);
//    /* USART interrupt configuration */
//    USART_ITConfig(CAMERA_PORT_NAME, USART_IT_TC,   DISABLE);
//    USART_ITConfig(CAMERA_PORT_NAME, USART_IT_RXNE, DISABLE);

	// Enable the USART
	USART_Cmd(USART1, ENABLE);

	// CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
	//	如下语句解决第1个字节无法正确发送出去的问题
	USART_ClearFlag(USART1, USART_FLAG_TC);  //清发送完成标志
											//Transmission Complete flag

}

//void CameraUsartIrq(void)
//{
//	USART_IRQ(&Camera_Usart);
//}
//
//void Camera_ReceivedByte(u8 R_data, struct USART_TypeDefStruct *pUSART)
//{
//
////	ack_number++;
//}


void Camera2_Tx_DMA_Enable(u16 ndtr)
{
	DMA_Cmd(DMA2_Stream7, DISABLE);                     //close DMA
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){} //enable DMA can work
	DMA_SetCurrDataCounter(DMA2_Stream7,ndtr);          //set the number of the datum
	DMA_Cmd(DMA2_Stream7, ENABLE);                      //open DMA
}

void Camera2_Rx_DMA_Enable(u16 ndtr)
{
	DMA_Cmd(DMA2_Stream5, DISABLE);
	while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE){}
	DMA_SetCurrDataCounter(DMA2_Stream5,ndtr);
	DMA_Cmd(DMA2_Stream5, ENABLE);
}

void close_camera(void)
{
	USART_DMACmd(USART1,USART_DMAReq_Rx,DISABLE);
	DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5);

	USART_DMACmd(USART1,USART_DMAReq_Tx,DISABLE);
	DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);

	DMA_Cmd(DMA2_Stream5, DISABLE);
	DMA_Cmd(DMA2_Stream7, DISABLE);

	EpsOutSwitch(OUT_EPS_S2, DISABLE);

	getack_flag=0;
	sendcom_flag=0;
}

/*接收完成中断*/
void DMA2_Stream5_IRQHandler()
{
	if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5) != RESET)
	{
		USART_DMACmd(USART1,USART_DMAReq_Rx,DISABLE);
		DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5);
		DMA_Cmd(DMA2_Stream5, DISABLE);                     //close DMA
	}
	getack_flag=1;
}

/*发送完成中断*/
void DMA2_Stream7_IRQHandler()
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET){};
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
	{
		USART_DMACmd(USART1,USART_DMAReq_Tx,DISABLE);
		DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);
		DMA_Cmd(DMA2_Stream7, DISABLE);                     //close DMA
	}
	sendcom_flag=1;
}

int take_store_picture(uint8_t picture_size)
{
	uint8_t t=0;                   //信息暂存，不在乎其中具体的数值
	uint16_t CRC_pack;             //CRC校验和存储变量
	uint8_t norflash_status;       //norflash写入情况变量

	/*相片信息变量*/
	uint16_t num_of_pack;             //相片分包数量，这个是取包指令次数，并不是相片信息中的数据
	uint16_t last_size_of_pack;       //最后一包的大小

	ImageInfo_el_t image_info_last = {
			.PacketSize = 220,
	};        //结构体，保存上一张相片的信息
	ImageInfo_el_t image_info_curr = {
			.PacketSize = 220,
	};        //结构体，保存当前相片的信息


	static uint32_t elimg_dir_init = 0;
    /*若初始化标志位0，则创建img文件夹*/
    if(elimg_dir_init == 0)
    {
        f_mkdir("0:enlai_img");
        elimg_dir_init++;
    }

	/*设置文件名，组成sd卡存储文件*/
	FIL fil;
	UINT bww;
	DIR dir;
	char cur_path[40]={0};
	char picturename[4];

    /*根据拍照大小指令确定拍照相片指令*/
	cmd_take_picture[3]=picture_size;

	/*相机上电，如果上电失败，则程序退出*/
	EpsOutSwitch(OUT_EPS_S2, ENABLE);

	vTaskDelay(5000);              /*延时5s，等待相机启动*/
	if(SW_EPS_S2_PIN()!=1)
		  return ERR_POWERUP;

	bsp_ReadCpuFlash(CAMERA_TIME_ADDRESS, (uint8_t*)image_times, 1);
	sprintf(cur_path, "0:enlai_img/ImageInfo-%d.dat", image_times[0] + 1);

//	itoa(image_times[0], picturename, 10);
//	strcpy(pre_path,"enlai_camera/");
//	strcat(pre_path,picturename);
//	strcat(pre_path,".jpg");

	/*防止在DMA启动之前串口上产生数据，所以要清除上溢标志并且读取串口线上的数据寄存器一次，清除线上的数据*/
	while(SET == USART_GetFlagStatus(USART1,USART_FLAG_ORE))
	{
		t=USART1->SR;
		t=USART1->DR;
	}
	t=USART_ReceiveData(USART1);
	/***************************************************************************/
	/*启动相机的接收，随时接收数据，一旦接收到指定长度的数据之后就进入接收中断*/
	Camera2_Rx_DMA_Config((uint32_t)ack_take_picture,(uint16_t)sizeof(ack_take_picture));
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	Camera2_Rx_DMA_Enable((uint16_t)sizeof(ack_take_picture));
	/*启动相机发送，一旦发送指定长度的数据之后就进入发送中断处理函数*/
	Camera2_Tx_DMA_Config((uint32_t)cmd_take_picture,(uint16_t)sizeof(cmd_take_picture));
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	Camera2_Tx_DMA_Enable((uint16_t)sizeof(cmd_take_picture));
    /*等待相机回复应答信息，一般在1s钟会有回复，收到指定长度的回复后就会进入中断操作，如果超过2分钟，则拍照失败*/
    vTaskDelay(2000);

	/*已经发送了指定长度数据和接收到指定长度数据了*/
	if((sendcom_flag==1)&(getack_flag==1))
	{
		sendcom_flag=0;//清除已拍照片标志和已接收数据标志
		getack_flag=0;
	}
	else/*如果拍照指令已经发送但是在超时时间范围内一直没有接收到指定数量的数据*/
	{
		printf("take a picture failed!\r\n");
		if(sendcom_flag==1)
		{
			driver_debug(DEBUG_CAMERA, "haven't accept enough pack\r\n");
			driver_debug(DEBUG_CAMERA, "the remaining data :%d\r\n",DMA_GetCurrDataCounter(DMA2_Stream5));
		}else{
			driver_debug(DEBUG_CAMERA, "haven't send the command of taking picture!\r\n");
		}
		close_camera();
		return ERR_RECEIVE_TP;
	}

	USART_DMACmd(USART1,USART_DMAReq_Rx,DISABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,DISABLE);
    /*打印接收到的应答指令，调试时候用*/
	for(int i=0;i<14;i++)
	{
		driver_debug(DEBUG_CAMERA, "ack_take2picture[%d] : %d\r\n", i, ack_take_picture[i]);
	}
	/*获取所拍相片的大小以及分包数量，同时获得相片的拍摄时间*/
	image_info_curr.ImageSize = ((u32)ack_take_picture[10]<<24)|((u32)ack_take_picture[9]<<16)|((u32)ack_take_picture[8]<<8)|((u32)ack_take_picture[7]);
	image_info_curr.TotalPacket = image_info_curr.ImageSize / image_info_curr.PacketSize + 1;
	image_info_curr.LastPacketSize = image_info_curr.ImageSize % image_info_curr.PacketSize;
	num_of_pack = ((u16)ack_take_picture[12]<<8)|((u16)ack_take_picture[11]);
	image_info_curr.ImageTime = clock_get_time_nopara();                      //保存拍照时间
	driver_debug(DEBUG_CAMERA, "size_of_picture=%X\r\nnum_of_pack=%X\r\n",image_info_curr.ImageSize,num_of_pack);
	driver_debug(DEBUG_CAMERA, "SYS Time is: %s\r\n", ctime((time_t *) &image_info_curr.ImageTime));
	driver_debug(DEBUG_CAMERA, "the remaining data :%d\r\n",DMA_GetCurrDataCounter(DMA2_Stream5));
	/*如果是拍的第一张相片，就把相片的存储位置设置为0，同时把相片的记录信息存储到norflash中，递增拍照次数，存储所拍相片次数，否则根据上次存储位置设置存储位置
	 */
	if(image_times[0] == 0)
	{
		image_times[0] = 1 ;
		driver_debug(DEBUG_CAMERA, "takep_time: %d\r\n",image_times[0]);
		image_info_curr.ImageDddress = 65536;    //待确认
		image_info_curr.ImageID = image_times[0] ;
		bsp_WriteCpuFlash(CAMERA_RECORD_ADDR, (uint8_t*)&image_info_curr, sizeof(ImageInfo_el_t));
		bsp_WriteCpuFlash(CAMERA_TIME_ADDRESS, (uint8_t*)image_times, 1);
	}else{
		image_times[0] ++ ;
		bsp_ReadCpuFlash(CAMERA_RECORD_ADDR+(image_times[0]-2)*sizeof(ImageInfo_el_t), (uint8_t*)&image_info_last, sizeof(ImageInfo_el_t));
		image_info_curr.ImageDddress=image_info_last.ImageDddress + num_of_pack * 512;
		image_info_curr.ImageID = image_times[0];
		if((image_info_curr.ImageDddress + image_info_curr.TotalPacket * image_info_last.PacketSize>4194302) || (image_times[0] == 254))    //如果norflash的存储空间已经满，就全片擦除norflash，然后把记录地址给擦除掉
		{
			printf("********************************full*******************************!\r\n");
			image_times[0] = 1;
			image_info_curr.ImageDddress = 65536;    //待确认
			if(FSMC_NOR_EraseChip() != 0)
			{
				driver_debug(DEBUG_CAMERA, "Chip Erase Failed!\n");
			}
			else
			{
				driver_debug(DEBUG_CAMERA, "Chip Erase Success!\n");
			}
		}
		driver_debug(DEBUG_CAMERA, "image_times: %d\r\n",image_times[0]);
		bsp_WriteCpuFlash(CAMERA_RECORD_ADDR+(image_times[0]-1)*sizeof(ImageInfo_el_t), (uint8_t*)&image_info_curr, sizeof(ImageInfo_el_t));
		bsp_WriteCpuFlash(CAMERA_TIME_ADDRESS, (uint8_t*)image_times, 1);
	}
	image_address = image_info_curr.ImageDddress ;
	/*拍照完成，接下来进行相片存储*/
	/*考虑接收到的数据，如果不是正常反馈的14个字节的信息，就断电后退出*/
	if(ack_take_picture[13]!=0x23)
	{
		driver_debug(DEBUG_CAMERA, "haven't take a picyure!\r\n");
		close_camera();
		return ERR_RECEIVE_TP;
	}
	/*如果前期检测正常，就获取最后一包数据的大小*/
	last_size_of_pack = image_info_curr.ImageSize % 512;
	f_open(&fil, cur_path,FA_CREATE_ALWAYS|FA_WRITE);
	f_write (&fil, (uint8_t*)&image_info_curr, sizeof(image_info_curr), &bww);
	f_close (&fil);

	/* 创建图像原始数据文件 */
	memcpy(cur_path,0,sizeof(cur_path));
    sprintf(cur_path, "0:enlai_img/ImageData-%d.dat", image_times[0]);
    f_open(&fil, cur_path,FA_CREATE_ALWAYS|FA_WRITE);

	for(int i=1;i<num_of_pack;i++)
	{
		CRC_pack = 0;                                            //清零CRC校验和数据
		cmd_get_pack[3]=(u8)(i&0xFF);                             //组帧取包指令
		cmd_get_pack[4]=(u8)((i>>8)&0xFF);
		driver_debug(DEBUG_CAMERA, "cmd_get_pack[3]=%X\r\n cmd_get_pack[4]=%X\r\n",cmd_get_pack[3],cmd_get_pack[4]);
		/*防止在DMA启动之前串口上产生数据，所以要清除上溢标志并且读取串口线上的数据寄存器一次，清除线上的数据*/
		while(SET == USART_GetFlagStatus(USART1,USART_FLAG_ORE))
		{
			t=USART1->SR;
			t=USART1->DR;
		}
		t=USART_ReceiveData(USART1);
		getack_flag=0;
		/***************************************************************************/
		Camera2_Rx_DMA_Config((u32)ack_get_pack,sizeof(ack_get_pack));
		USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
		Camera2_Rx_DMA_Enable(sizeof(ack_get_pack));

		Camera2_Tx_DMA_Config((u32)cmd_get_pack,sizeof(cmd_get_pack));
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
		Camera2_Tx_DMA_Enable(sizeof(cmd_get_pack));

		vTaskDelay(100);
		if((sendcom_flag==1)&(getack_flag==1))
		{
			/*CRC校验*/
			for(int i=4;i<sizeof(ack_get_pack)-2;i++)
			{
				CRC_pack=CRC_pack+ack_get_pack[i];
			}
			driver_debug(DEBUG_CAMERA, "CRC_pack=%d\r\n",CRC_pack);
			driver_debug(DEBUG_CAMERA, "((u16)ack_pack[524]<<8)|((u16)ack_pack[523])=%d\r\n",((u16)ack_get_pack[524]<<8)|((u16)ack_get_pack[523]));
			if((((u16)ack_get_pack[524]<<8)|((u16)ack_get_pack[523]))!=CRC_pack)
			{
				/*CRC校验失败*/
				driver_debug(DEBUG_CAMERA, "CRC fail!\r\n");
				close_camera();
				memset(ack_get_pack,0,sizeof(ack_get_pack));
				getack_flag=0;
				sendcom_flag=0;
				return ERR_CRC;
			}
			getack_flag=0;
			sendcom_flag=0;
			norflash_status=FSMC_NOR_WriteBuffer((u16 *)(&ack_get_pack[11]),(uint32_t)image_address,(uint32_t)(sizeof(ack_get_pack)-13)/2);
//			for(int i=12;i<sizeof(ack_get_pack)-2;i=i+2)
//			{
//				pack_store_norflash[(i-12)/2]=((u16)ack_get_pack[i])|((u16)ack_get_pack[i-1]<<8);
//			}
//			norflash_status=FSMC_NOR_WriteBuffer(pack_store_norflash,(uint32_t)image_address,(uint32_t)sizeof(pack_store_norflash)/2);
			image_address += (0x100);
			driver_debug(DEBUG_CAMERA, "norflash_status=%d\r\n",norflash_status);
		}
		else/*如果取包指令已经发送但是在超时时间范围内一直没有接收到指定数量的数据*/
		{
			driver_debug(DEBUG_CAMERA, "get a pack failed!\r\n");
			if(sendcom_flag==1)
			{
				driver_debug(DEBUG_CAMERA, "haven't accept enough pack\r\n");
				driver_debug(DEBUG_CAMERA, "the remaining data :%d\r\n",DMA_GetCurrDataCounter(DMA2_Stream5));
			}else{
				driver_debug(DEBUG_CAMERA, "haven't send the command of getting a pack!\r\n");
			}
			close_camera();
			memset(ack_get_pack,0,sizeof(ack_get_pack));
			return ERR_RECEIVE_TP;
		}

		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);
		USART_DMACmd(USART1,USART_DMAReq_Rx,DISABLE);
		 /*write to cd*/
		for(int i=11;i<sizeof(ack_get_pack)-2;i++)
		{
			pack_store_sd[i-11]=ack_get_pack[i];
		}
		f_write (&fil, pack_store_sd, sizeof(pack_store_sd), &bww);
		//f_write (&fil, (u8 *)(&ack_get_pack[11]), sizeof(ack_get_pack)-13, &bww);
	}
	/*receive the last pack*/
	CRC_pack=0;
	memset(ack_get_pack,0,sizeof(ack_get_pack));
	cmd_get_pack[3]=(u8)(num_of_pack&0xFF);
	cmd_get_pack[4]=(u8)((num_of_pack>>8)&0xFF);
	driver_debug(DEBUG_CAMERA, "cmd_get_pack[3]=%X\r\n cmd_get_pack[4]=%X\r\n",cmd_get_pack[3],cmd_get_pack[4]);
	/*防止在DMA启动之前串口上产生数据，所以要清除上溢标志并且读取串口线上的数据寄存器一次，清除线上的数据*/
	while(SET == USART_GetFlagStatus(USART1,USART_FLAG_ORE))
	{
		t=USART1->SR;
		t=USART1->DR;
	}
	t=USART_ReceiveData(USART1);
	getack_flag=0;
	/***************************************************************************/
	Camera2_Rx_DMA_Config((u32)ack_get_pack,last_size_of_pack+13);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	Camera2_Rx_DMA_Enable(last_size_of_pack+13);

	Camera2_Tx_DMA_Config((u32)cmd_get_pack,sizeof(cmd_get_pack));
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	Camera2_Tx_DMA_Enable(sizeof(cmd_get_pack));
	vTaskDelay(100);
	if((sendcom_flag==1)&(getack_flag==1))
	{
		for(int i=4;i<(last_size_of_pack+11);i++)
		{
			CRC_pack=CRC_pack+ack_get_pack[i];
		}
		driver_debug(DEBUG_CAMERA, "CRC_pack=%d\r\n",CRC_pack);
		driver_debug(DEBUG_CAMERA, "((u16)ack_pack[%d]<<8)|((u16)ack_pack[%d])=%d\r\n",last_size_of_pack+12,last_size_of_pack+11,(((u16)ack_get_pack[last_size_of_pack+12]<<8)|((u16)ack_get_pack[last_size_of_pack+11])));
		if((((u16)ack_get_pack[last_size_of_pack+12]<<8)|((u16)ack_get_pack[last_size_of_pack+11]))!=CRC_pack)
		{
			driver_debug(DEBUG_CAMERA, "CRC fail!\r\n");
			close_camera();
			memset(ack_get_pack,0,sizeof(ack_get_pack));
			getack_flag=0;
			sendcom_flag=0;
			return ERR_CRC;
		}
		sendcom_flag=0;
		getack_flag=0;
		norflash_status=FSMC_NOR_WriteBuffer((u16 *)(&ack_get_pack[11]),(uint32_t)image_address,(uint32_t)(sizeof(ack_get_pack)-13)/2);
//		for(int i=12;i<sizeof(ack_get_pack)-2;i=i+2)
//		{
//			pack_store_norflash[(i-12)/2]=((u16)ack_get_pack[i])|((u16)ack_get_pack[i-1]<<8);
//		}
//		norflash_status=FSMC_NOR_WriteBuffer(pack_store_norflash,image_address,sizeof(pack_store_norflash)/2);
		image_address += (0x100);
		driver_debug(DEBUG_CAMERA, "norflash_status=%d\r\n",norflash_status);
	}
	else /*如果取包指令已经发送但是在超时时间范围内一直没有接收到指定数量的数据*/
	{
		driver_debug(DEBUG_CAMERA, "get a pack failed!\r\n");
		if(sendcom_flag==1)
		{
			driver_debug(DEBUG_CAMERA, "haven't accept enough pack\r\n");
			driver_debug(DEBUG_CAMERA, "the remaining data :%d\r\n",DMA_GetCurrDataCounter(DMA2_Stream5));
		}else{
			driver_debug(DEBUG_CAMERA, "haven't send the command of getting a pack!\r\n");
		}
		close_camera();
		memset(ack_get_pack,0,sizeof(ack_get_pack));
		getack_flag=0;
		sendcom_flag=0;
		return ERR_RECEIVE_TP;
	}

	for(int i=11;i<last_size_of_pack+11;i++)
	{
		f_write (&fil, &ack_get_pack[i], 1, &bww);
	}
	f_close (&fil);
	close_camera();
	memset(ack_get_pack,0,sizeof(ack_get_pack));
	return 0;
}

/**
 * 通过图像ID参数下行flash中图像信息
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_flash_enlaiimg_info_down(uint8_t id)
{
	ImageInfo_el_t *img_info = (ImageInfo_el_t *)ObcMemMalloc(sizeof(ImageInfo_el_t));
	if (img_info == NULL)
	        return E_NO_BUFFER;

	bsp_ReadCpuFlash(CAMERA_RECORD_ADDR + (id - 1)*sizeof(ImageInfo_el_t), (uint8_t*)img_info, sizeof(ImageInfo_el_t));

    /* 调用传输层接口函数下行，创建下行图像任务  */
	int ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE_INFO, img_info, sizeof(ImageInfo_el_t));
    //int ret = vu_isis_downlink(CAM_IMAGE_INFO, img_info, (uint32_t)sizeof(ImageInfo_el_t));
    ObcMemFree(img_info);

    return ret;
}




/**
 * 通过图像ID参数下行SD卡中图像信息
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_sd_enlaiimg_info_down(uint8_t id)
{
    char path[40] = {0};

    ImageInfo_el_t *img_info = (ImageInfo_el_t *)ObcMemMalloc(sizeof(ImageInfo_el_t));
    if (img_info == NULL)
        return E_NO_BUFFER;

    sprintf(path, "0:enlai_img/ImageInfo-%d.dat", id);

    if (file_read(path, img_info, (UINT)sizeof(ImageInfo_el_t), 0) != FR_OK)
    {
        driver_debug(DEBUG_CAMERA, "Camera read ImageInfo.dat Failure!!\r\n");
        ObcMemFree(img_info);
        return E_NO_DEVICE;
    }

    /* 调用传输层接口函数下行  */
    int ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE_INFO, img_info, sizeof(ImageInfo_el_t));

    ObcMemFree(img_info);
    return ret;
}

/**
 * 下行SD卡中单包图像
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_sd_enlaiimg_packet_down(uint8_t id, uint16_t packet)
{
    char path[40] = {0};

    ImageInfo_el_t *img_info = (ImageInfo_el_t *)ObcMemMalloc(sizeof(ImageInfo_el_t));
    if (img_info == NULL)
        return E_NO_BUFFER;

    sprintf(path, "0:enlaiimg/ImageInfo-%d.dat", id);

    if (file_read(path, img_info, (UINT)sizeof(ImageInfo_el_t), 0) != FR_OK)
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

    ImagePacket_enlai_t * img_packet = (ImagePacket_enlai_t *)ObcMemMalloc(sizeof(ImagePacket_enlai_t));
    if (img_packet == NULL)
    {
        ObcMemFree(img_info);
        return E_NO_BUFFER;
    }

    img_packet->PacketID = packet;
    img_packet->PacketSize = (packet == img_info->TotalPacket - 1) ?
            img_info->LastPacketSize : IMAGE_PACK_MAX_SIZE;

    sprintf(path, "0:enlai_img/ImageData-%d.dat", id);

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
 * 下行SD卡中指定ID编号的图像
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_sd_enlaiimg_all_down(uint8_t id)
{
    char path[40] = {0};
    int ret;

    ImageInfo_el_t *img_info = (ImageInfo_el_t *)ObcMemMalloc(sizeof(ImageInfo_el_t));
    if (img_info == NULL)
        return E_NO_BUFFER;

    sprintf(path, "0:enlaiimg/ImageInfo-%d.dat", id);

    if (file_read(path, img_info, (UINT)sizeof(ImageInfo_el_t), 0) != FR_OK)
    {
        driver_debug(DEBUG_CAMERA, "Camera read ImageInfo.dat Failure!!\r\n");
        ObcMemFree(img_info);
        return E_NO_DEVICE;
    }

    for(int i=0; i < img_info->TotalPacket; i++)
    {
        ImagePacket_enlai_t * img_packet = (ImagePacket_enlai_t *)ObcMemMalloc(sizeof(ImagePacket_enlai_t));
        if (img_packet == NULL)
        {
            ObcMemFree(img_info);
            return E_NO_BUFFER;
        }

        img_packet->PacketID = i;
        img_packet->PacketSize = (i == img_info->TotalPacket-1) ?
                img_info->LastPacketSize : IMAGE_PACK_MAX_SIZE;
        sprintf(path, "0:enlai_img/ImageData-%d.dat", id);

        if (file_read(path, img_packet->ImageData, img_packet->PacketSize, i * IMAGE_PACK_MAX_SIZE) != FR_OK)
        {
            driver_debug(DEBUG_CAMERA, "Camera read ImageData.dat Failure!!\r\n");
            ObcMemFree(img_info);
            ObcMemFree(img_packet);
            return E_NO_DEVICE;
        }
        /* 调用传输层接口函数下行 */
       ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE_INFO, img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);
       //ret = vu_isis_downlink(CAM_IMAGE, img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);

       ObcMemFree(img_packet);
    }

    ObcMemFree(img_info);

    return ret;
}

/**
 * 下行SD卡中指定ID编号某包开始剩余包的图像
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_sd_enlaiimg_lll_down(uint8_t id,uint16_t packet)
{
    char path[40] = {0};
    int ret;

    ImageInfo_el_t *img_info = (ImageInfo_el_t *)ObcMemMalloc(sizeof(ImageInfo_el_t));
    if (img_info == NULL)
        return E_NO_BUFFER;

    sprintf(path, "0:enlaiimg/ImageInfo-%d.dat", id);

    if (file_read(path, img_info, (UINT)sizeof(ImageInfo_el_t), 0) != FR_OK)
    {
        driver_debug(DEBUG_CAMERA, "Camera read ImageInfo.dat Failure!!\r\n");
        ObcMemFree(img_info);
        return E_NO_DEVICE;
    }

    for(int i=packet; i < img_info->TotalPacket; i++)
    {
        ImagePacket_enlai_t * img_packet = (ImagePacket_enlai_t *)ObcMemMalloc(sizeof(ImagePacket_enlai_t));
        if (img_packet == NULL)
        {
            ObcMemFree(img_info);
            return E_NO_BUFFER;
        }

        img_packet->PacketID = i;
        img_packet->PacketSize = (i == img_info->TotalPacket-1) ?
                img_info->LastPacketSize : IMAGE_PACK_MAX_SIZE;
        sprintf(path, "0:enlai_img/ImageData-%d.dat", id);

        if (file_read(path, img_packet->ImageData, img_packet->PacketSize, i * IMAGE_PACK_MAX_SIZE) != FR_OK)
        {
            driver_debug(DEBUG_CAMERA, "Camera read ImageData.dat Failure!!\r\n");
            ObcMemFree(img_info);
            ObcMemFree(img_packet);
            return E_NO_DEVICE;
        }
        /* 调用传输层接口函数下行 */
       ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE_INFO, img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);
       //ret = vu_isis_downlink(CAM_IMAGE, img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);

       ObcMemFree(img_packet);
    }

    ObcMemFree(img_info);

    return ret;
}


/**
 * 下行FLASH中单包图像
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_flash_enlaiimg_packet_down(uint8_t id, uint16_t packet)
{
    int ret;
	ImageInfo_el_t *img_info = (ImageInfo_el_t *)ObcMemMalloc(sizeof(ImageInfo_el_t));
    if (img_info == NULL)
        return E_NO_BUFFER;

    bsp_ReadCpuFlash(CAMERA_RECORD_ADDR + (id - 1)*sizeof(ImageInfo_el_t), (uint8_t*)img_info, sizeof(ImageInfo_el_t));

    if (packet >= img_info->TotalPacket)
    {
    	ObcMemFree(img_info);
        return E_INVALID_PARAM;
    }

    ImagePacket_enlai_t * img_packet = (ImagePacket_enlai_t * )ObcMemMalloc(sizeof(ImagePacket_enlai_t));

    if (img_packet == NULL)
    {
    	ObcMemFree(img_info);
        return E_NO_BUFFER;
    }

    img_packet->PacketID = packet;
    img_packet->PacketSize = (packet == img_info->TotalPacket - 1) ?
            img_info->LastPacketSize : IMAGE_PACK_MAX_SIZE;

    //FSMC_NOR_ReadBuffer((uint16_t *)img_packet->ImageData, (uint32_t)(img_info->ImageDddress + (packet - 1) * IMAGE_PACK_MAX_SIZE), (u32)img_packet->PacketSize);

    FSMC_NOR_ReadBuffer((uint16_t *)img_packet->ImageData, img_info->ImageDddress + packet * IMAGE_PACK_MAX_SIZE / 2, 110);
    /* 调用传输层接口函数下行 */
    ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE_INFO, img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);
//    int ret = vu_isis_downlink(CAM_IMAGE, img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);

    ObcMemFree(img_info);
    ObcMemFree(img_packet);
    return ret;
}

/**
 * 下行FLASH中一整张图片图像
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_flash_enlaiimg_all_down(uint32_t id)
{
	int ret;
	ImageInfo_el_t *img_info = (ImageInfo_el_t *)ObcMemMalloc(sizeof(ImageInfo_el_t));
	if (img_info == NULL)
		return E_NO_BUFFER;

	bsp_ReadCpuFlash(CAMERA_RECORD_ADDR + (id - 1)*sizeof(ImageInfo_el_t), (uint8_t*)img_info, sizeof(ImageInfo_el_t));

	for(int i=0; i < img_info->TotalPacket; i++)
	{
	    ImagePacket_enlai_t * img_packet = (ImagePacket_enlai_t * )ObcMemMalloc(sizeof(ImagePacket_enlai_t));

	    if (img_packet == NULL)
	    {
	    	ObcMemFree(img_info);
	        return E_NO_BUFFER;
	    }

	    img_packet->PacketID = i;
	    img_packet->PacketSize = (i == img_info->TotalPacket -1) ?
	            img_info->LastPacketSize : IMAGE_PACK_MAX_SIZE;

	    FSMC_NOR_ReadBuffer((uint16_t *)img_packet->ImageData, img_info->ImageDddress + i * IMAGE_PACK_MAX_SIZE / 2, 110);

//	    for(int kc = 0;kc<220;kc=kc+2)  //把norflash对应地址上的值取出来组成每帧220字节的数据
//        {
//            camera_pack_send[kc/2] = FSMC_NOR_ReadHalfWord(img_info->ImageDddress);
//            camera_pack_send_1[kc]=(camera_pack_send[kc/2]>>8);
//            camera_pack_send_1[kc+1]=(camera_pack_send[kc/2])&(0xFF);
//            img_info->ImageDddress += 1;
//        }
//	    memcpy(img_packet->ImageData,camera_pack_send_1,220);

	    /* 调用传输层接口函数下行 */
	    ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE_INFO, img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);
	    //ret = vu_isis_downlink(CAM_IMAGE, img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);

	    ObcMemFree(img_packet);

	}
	ObcMemFree(img_info);
	return ret;

}


/**
 * 下行FLASH中一整张图片某张包开始剩余的包
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_flash_enlaiimg_lll_down(uint8_t id, uint16_t packet)
{
	int ret;
	ImageInfo_el_t *img_info = (ImageInfo_el_t *)ObcMemMalloc(sizeof(ImageInfo_el_t));
	if (img_info == NULL)
		return E_NO_BUFFER;
	bsp_ReadCpuFlash(CAMERA_RECORD_ADDR + (id - 1)*sizeof(ImageInfo_el_t), (uint8_t*)img_info, sizeof(ImageInfo_el_t));

	if (packet >= img_info->TotalPacket)                //为什么是等于号？
	{
		ObcMemFree(img_info);
		return E_INVALID_PARAM;
	}

	for(int i = packet; i < img_info->TotalPacket; i++)
	{

	    ImagePacket_enlai_t * img_packet = (ImagePacket_enlai_t * )ObcMemMalloc(sizeof(ImagePacket_enlai_t));

	    if (img_packet == NULL)
	    {
	    	ObcMemFree(img_info);
	        return E_NO_BUFFER;
	    }

	    img_packet->PacketID = i;
	    img_packet->PacketSize = (i == img_info->TotalPacket - 1) ?
	            img_info->LastPacketSize : IMAGE_PACK_MAX_SIZE;

	    //FSMC_NOR_ReadBuffer((uint16_t *)img_packet->ImageData, img_info->ImageDddress + (i-1) * IMAGE_PACK_MAX_SIZE, img_packet->PacketSize / 2);
	    FSMC_NOR_ReadBuffer((uint16_t *)img_packet->ImageData, img_info->ImageDddress + i * IMAGE_PACK_MAX_SIZE / 2, 110);
	    /* 调用传输层接口函数下行 */
	    //ret = vu_isis_downlink(CAM_IMAGE, img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);
	    ret = ProtocolSendDownCmd(GND_ROUTE_ADDR, CAM_ROUTE_ADDR, CAM_IMAGE_INFO, img_packet, img_packet->PacketSize + IMAGE_PACK_HEAD_SIZE);
	    ObcMemFree(img_packet);
	}
	ObcMemFree(img_info);
	return ret;
}

/***************************下行调用函数********************************/
/**
 * 下行特定ID图像信息
 * 先从cpuflash中读取数据，如果传输失败，则从sd卡中读取数据
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_enlaiimg_info_down(uint32_t id, uint8_t source)
{
    int ret;
    if(source == 0){
        ret = cam_flash_enlaiimg_info_down(id);
    }
    else{
        ret = cam_sd_enlaiimg_info_down(id);
    }
    return ret;
}

/**
 * 相机下行指定ID照片
 * 先从norflash中下行数据，如果出错，则从sd卡中下行数据
 *
 * @param id 照片ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_enlaiimg_data_down(uint32_t id, uint8_t source)
{
    int ret;
    if(source == 0){
        ret = cam_flash_enlaiimg_all_down(id);
    }
    else{
        ret = cam_sd_enlaiimg_all_down(id);
    }
    return ret;
}

/**
 * 下行ID编号图像的第packet图像数据
 * 先从flash中读取第packet包数据，如果出错，就从sd卡中读取
 *
 * @param id 图像ID
 * @param packet 图像包号
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int cam_enlaiimg_packet_down(uint32_t id, uint16_t packet, uint8_t source)
{
    int ret;
    if(source == 0){
        ret = cam_flash_enlaiimg_packet_down(id,packet);
    }
    else{
        ret = cam_sd_enlaiimg_packet_down(id,packet);
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
int cam_enlaiimg_data_packet_down(uint32_t id, uint16_t start_packet, uint8_t source)
{
    int ret;
    if(source == 0){
        ret = cam_flash_enlaiimg_lll_down(id,start_packet);
    }
    else{
        ret = cam_sd_enlaiimg_lll_down(id,start_packet);
    }
    return ret;
}

void init_camera_address()
{
    uint8_t take_time =0;
    bsp_WriteCpuFlash(CAMERA_TIME_ADDRESS,(uint8_t*)&take_time,sizeof(take_time));
    FSMC_NOR_EraseChip();
}

/*****************************测试函数********************************/

int take_a_picture(struct command_context * context)
{
	char * args = command_args(context);

	uint8_t picture_size;  //接收所拍照片大小的数值

	if (sscanf(args, "%X", &picture_size) != 1)
	     return CMD_ERROR_SYNTAX;
	if(take_store_picture(picture_size)!=0)
	{
		printf("take a picture failed!\r\n");
		return 1;
	}
	printf("take a picture successful!\r\n");
	return 0;
}



/**
 * 下行目前所拍相片的数量
 * 测试函数
 *
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int get_imagenumber_flash()
{

	bsp_ReadCpuFlash(CAMERA_TIME_ADDRESS, (uint8_t*)image_times, 1);

	/* 调用传输层接口函数下行，创建下行图像任务  */
	int ret = vu_obc_downlink(CAM_IMAGE_INFO, image_times, sizeof(image_times));
	return ret;
}


/**
 * 通过图像ID参数下行flash中图像信息
 * 测试函数
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int get_imageinfo_flash(struct command_context * context)
{
	char * args = command_args(context);

	uint8_t image_id;  //接收所拍照片大小的数值

	if (sscanf(args, "%X", &image_id) != 1)
	     return CMD_ERROR_SYNTAX;

	if(cam_flash_enlaiimg_info_down(image_id)!=0)
	{
		printf("get the %dth picture information from flash failed!\r\n",image_id);
		return 1;
	}
	printf("get the %dth picture information from flash successful!\r\n",image_id);
	return 0;
}

/**
 * 下行FLASH中某一包图像
 * 测试函数
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */

int get_a_pack_flash(struct command_context * context) {

	char * args = command_args(context);
	uint8_t id;
	uint16_t packet;

	if (sscanf(args, "%x %u", &id, &packet)!= 2)
		return CMD_ERROR_SYNTAX;
	if(cam_flash_enlaiimg_packet_down(id,packet)!=0)
	{
		printf("get the %dth pack of the %dth image from flash failed!\r\n",packet,id);
		return 1;
	}
	printf("get the %dth pack of the %dth image from flash successful!\r\n",packet,id);
	return 0;
}

/**
 * 下行FLASH中一整张图片图像
 * 测试函数
 *
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int get_wholeimage_flash(struct command_context * context)
{
	char * args = command_args(context);

	uint8_t image_id;  //接收所拍照片大小的数值

	if (sscanf(args, "%x", &image_id) != 1)
	     return CMD_ERROR_SYNTAX;

	if(cam_flash_enlaiimg_all_down(image_id)!= -1)
	{
		printf("get the %dth picture from flash failed!\r\n",image_id);
		return 1;
	}
	printf("get the %dth picture from flash successful!\r\n",image_id);
	return 0;
}

/**
 * 下行FLASH中一整张图片某张包开始剩余的包
 * 测试函数
 * @param id 图像ID
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int get_image_last_packs_flash(struct command_context * context) {

	char * args = command_args(context);
	uint8_t id;
	uint16_t packet;

	if (sscanf(args, "%x %u", &id, &packet)!= 2)
		return CMD_ERROR_SYNTAX;
	if(cam_flash_enlaiimg_lll_down(id, packet)!=0)
	{
		printf("get last %dth packs of the %dth image  from flash failed!\r\n",packet,id);
		return 1;
	}
	printf("get last %dth pack of the %dth image from flash successful!\r\n",packet,id);
	return 0;
}

/**********************串口测试接收函数***************************/

int send_pack(struct command_context * context)
{

	char * args = command_args(context);

	int picture_number_1;
	int pack_number_1;
	ImageInfo_el_t image_info_usart = {
				.PacketSize = 220,
		};        //结构体，保存串口传输时一张相片的信息

	if (sscanf(args, "%u %u", &picture_number_1, &pack_number_1) != 2)
		return CMD_ERROR_SYNTAX;

	vSerialInterfaceInit();

	bsp_ReadCpuFlash(CAMERA_RECORD_ADDR+(picture_number_1-1)*sizeof(image_info_usart), (uint8_t*)&image_info_usart, sizeof(image_info_usart));

	for(int kc = 0;kc<220;kc=kc+2)  //把norflash对应地址上的值取出来组成每帧128字节的数据
	{
		camera_pack_send[kc/2] = FSMC_NOR_ReadHalfWord(image_info_usart.ImageDddress+(uint32_t)(pack_number_1-1)*220);
		camera_pack_send_1[kc]=(camera_pack_send[kc/2]>>8);
		camera_pack_send_1[kc+1]=(camera_pack_send[kc/2])&(0xFF);
		image_info_usart.ImageDddress +=1;
	}
	vSerialSend(camera_pack_send_1,(uint16_t)sizeof(camera_pack_send_1));

    return 0;
}

int send_picture(struct command_context * context)
{

	char * args = command_args(context);
	ImageInfo_el_t image_info_usart = {
					.PacketSize = 220,
			};        //结构体，保存串口传输时一张相片的信息
	int picture_number_1;

	if (sscanf(args, "%u", &picture_number_1) != 1)
	    ObcMemFree(image_info_usart);
		return CMD_ERROR_SYNTAX;

	vSerialInterfaceInit();

	bsp_ReadCpuFlash(CAMERA_RECORD_ADDR+(picture_number_1-1)*sizeof(image_info_usart), (uint8_t*)&image_info_usart, sizeof(image_info_usart));
    for(int t=0;t<image_info_usart.TotalPacket;t++)
	{
        ImagePacket_enlai_t * img_packet = (ImagePacket_enlai_t * )ObcMemMalloc(sizeof(ImagePacket_enlai_t));

        if (img_packet == NULL)
        {
            ObcMemFree(image_info_usart);
            return E_NO_BUFFER;
        }
        img_packet->PacketID = t;
        img_packet->PacketSize = (t == image_info_usart.TotalPacket-1) ?
        		image_info_usart.LastPacketSize : IMAGE_PACK_MAX_SIZE;
//		for(int kc = 0;kc<220;kc=kc+2)  //把norflash对应地址上的值取出来组成每帧220字节的数据
//		{
//			camera_pack_send[kc/2] = FSMC_NOR_ReadHalfWord(image_info_usart.ImageDddress);
//			camera_pack_send_1[kc]=(camera_pack_send[kc/2]>>8);
//			camera_pack_send_1[kc+1]=(camera_pack_send[kc/2])&(0xFF);
//			image_info_usart.ImageDddress += 1;
//		}
        FSMC_NOR_ReadBuffer((uint16_t *)img_packet->ImageData, image_info_usart.ImageDddress + t * IMAGE_PACK_MAX_SIZE / 2, image_info_usart.PacketSize /2);

        //vSerialSend(camera_pack_send_1,sizeof(camera_pack_send_1));
		vSerialSend(img_packet->ImageData,img_packet->PacketSize);
		vTaskDelay(50);
		ObcMemFree(img_packet);
	 }
    ObcMemFree(image_info_usart);
    return 0;
}


struct command camera_subcommands[] = {
	{
		.name = "take",
		.help = "take a picture",
		.handler = take_a_picture,
	},{
		.name = "init",
		.help = "initialize the camera address",
		.handler = init_camera_address,
	},{
		.name = "info",
		.help = "get a image information from flash",
		.handler = get_imageinfo_flash,
	},{
		.name = "pack",
		.help = "get a pack of a image from flash",
		.handler = get_a_pack_flash,
	},{
		.name = "picture",
		.help = "get a whole image from flash",
		.handler = get_wholeimage_flash,
	},{
		.name = "lastpack",
		.help = "get the last packs of a image from flash",
		.handler = get_image_last_packs_flash,
	},{
		.name = "usartpack",
		.help = "get the last packs of a image from flash through usart",
		.handler = send_pack,
	},{
		.name = "usartpicture",
		.help = "get the last packs of a image from flash through usart",
		.handler = send_picture,
	},{
		.name = "numberpicture",
		.help = "get the numbers of the picture have taken",
		.handler = get_imagenumber_flash,
	}
};

struct command __root_command camera_commands_master[] = {
	{
		.name = "camera",
		.help = "take and store a picture",
		.chain = INIT_CHAIN(camera_subcommands),
	},
};

void cmd_camera_setup(void) {
	command_register(camera_commands_master);
}







