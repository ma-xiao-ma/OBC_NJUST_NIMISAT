/*
 * if_downlink_vu.c
 *
 *  Created on: 2017年10月5日
 *      Author: Ma Wenli
 */
#include <string.h>

#include "bsp_pca9665.h"
#include "router_io.h"
#include "error.h"
#include "crc.h"
#include "semphr.h"
#include "cube_com.h"
#include "obc_mem.h"
#include "if_jlgvu.h"
#include "hexdump.h"

#include "if_downlink_vu.h"

typedef struct __attribute__((__packed__))
{
	uint32_t file_size;		//文件总字节数
    uint16_t total_packet;   //文件包总数
    uint8_t packet_size;     //数据包大小
    uint32_t time;
	uint8_t filename[20];
} file_info_down_t;

/*接收单元上次接收到上行数据时遥测存储队列*/
QueueHandle_t rx_tm_queue;

static uint32_t vu_rx_count; //ISISvu通信机接收上行消息计数

/**
 * ISIS通信机下行接口函数，由OBC本地调用
 *
 * @param type 下行消息类型
 * @param pdata 下行数据指针
 * @param len 下行数据字节数
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_isis_downlink(uint8_t type, void *pdata, uint32_t len)
{
    int ret;
    uint8_t Error = 0, TxRemainBufSize, FrameDataSize;
    uint32_t RemainSize = len;

    pdata = (uint8_t *)pdata;

    route_frame_t *downlink = ObcMemMalloc(I2C_MTU);
    if(downlink == NULL)
        return E_MALLOC_FAIL;

    downlink->dst = GND_ROUTE_ADDR;
    downlink->src = router_get_my_address();
    downlink->typ = type;

    do
    {
        FrameDataSize = (RemainSize < DOWNLINK_MTU) ? RemainSize : DOWNLINK_MTU;
        memcpy(downlink->dat, pdata, FrameDataSize);

//        *(uint32_t *)(&downlink->dat[FrameDataSize]) =
//                crc32_memory((uint8_t *)downlink, ROUTE_HEAD_SIZE+FrameDataSize);

        ret = vu_transmitter_send_frame(downlink, FrameDataSize + DOWNLINK_OVERHEAD, &TxRemainBufSize);

        /**如果传输成功，且通信机成功将消息加入发送缓冲区，发送指针后移 */
        if ((ret == E_NO_ERR) && (TxRemainBufSize != 0xFF))
        {
            RemainSize -= FrameDataSize;
            pdata += FrameDataSize;

            /**若发射机缓冲区已满，则等待5秒钟*/
            if(TxRemainBufSize == 1)
                vTaskDelay(MS_WAIT_TRANS_FREE_BUFF);
        }
        else
            Error++;

        if (RemainSize != 0)
            vTaskDelay(PACK_DOWN_INTERVAL);

     /**若发送完成或者错误次数超过10次，则跳出循环 */
    }while ((RemainSize > 0) && (Error < 10));

    ObcMemFree(downlink);

    if (RemainSize == 0)
        return E_NO_ERR;
    else
    {   /**如果错误超过10次，则复位发射机*/
        vu_transmitter_software_reset();
        return E_TRANSMIT_ERROR;
    }
}

/**
 * 通过vu无线接口发送数据
 *
 * @param dst 目的地址
 * @param src 源地址
 * @param type 消息类型
 * @param pdata 待发送数据指针
 * @param len 待发送数据长度
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_send( uint8_t dst, uint8_t src, uint8_t type, void *pdata, uint32_t len )
{
    int ret;
    uint8_t Error = 0, TxRemainBufSize, FrameDataSize;
    uint32_t RemainSize = len;

    pdata = (uint8_t *)pdata;

    route_frame_t *downlink = ObcMemMalloc(I2C_MTU);
    if(downlink == NULL)
        return E_MALLOC_FAIL;

    downlink->dst = dst;
    downlink->src = src;
    downlink->typ = type;

    do
    {
        FrameDataSize = (RemainSize < DOWNLINK_MTU) ? RemainSize : DOWNLINK_MTU;
        memcpy(downlink->dat, pdata, FrameDataSize);

//        *(uint32_t *)(&downlink->dat[FrameDataSize]) =
//                crc32_memory((uint8_t *)downlink, ROUTE_HEAD_SIZE+FrameDataSize);

        ret = vu_transmitter_send_frame(downlink, FrameDataSize + DOWNLINK_OVERHEAD, &TxRemainBufSize);

        /**如果传输成功，且通信机成功将消息加入发送缓冲区，发送指针后移 */
        if ((ret == E_NO_ERR) && (TxRemainBufSize != 0xFF))
        {
            RemainSize -= FrameDataSize;
            pdata += FrameDataSize;

            /**若发射机缓冲区已满，则等待5秒钟*/
            if(TxRemainBufSize == 1)
                vTaskDelay(MS_WAIT_TRANS_FREE_BUFF);
        }
        else
            Error++;

        if (RemainSize != 0)
            vTaskDelay(PACK_DOWN_INTERVAL);

     /**若发送完成或者错误次数超过10次，则跳出循环 */
    }while ((RemainSize > 0) && (Error < 10));

    ObcMemFree(downlink);

    if (RemainSize == 0)
        return E_NO_ERR;
    else
    {   /**如果错误超过10次，则复位发射机*/
        vu_transmitter_software_reset();
        return E_TRANSMIT_ERROR;
    }
}

/**
 * 文件下行，由OBC本地调用
 *
 * @param file 文件句柄指针
 * @param file_name 文件名指针
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_isis_file_download(FIL *file, char *file_name)
{

	file_info_down_t *file_info = (file_info_down_t *)ObcMemMalloc(sizeof(file_info_down_t));

	if( file_info == NULL )
	{
		printf("ERROR: Malloc fail!!\r\n");
		return E_MALLOC_FAIL;
	}

	file_info->file_size = f_size( file );

	file_info->packet_size = IMAGE_PACK_MAX_SIZE;

	file_info->total_packet = (file_info->file_size % file_info->packet_size) ?
			file_info->file_size/file_info->packet_size + 1 : file_info->file_size/file_info->packet_size;

	file_info->time = clock_get_time_nopara();

	memcpy( file_info->filename, file_name, strlen(file_name));

	vu_send(GND_ROUTE_ADDR, OBC_ROUTE_ADDR, FILE_INFO, file_info, sizeof(file_info_down_t));

    int ret;
    uint8_t Error = 0, TxRemainBufSize;
    uint32_t RemainSize = file_info->file_size, byte_read;

    route_frame_t *downlink = ObcMemMalloc(I2C_MTU);
    if(downlink == NULL)
    {
		printf("ERROR: Malloc fail!!\r\n");
		return E_MALLOC_FAIL;
    }

    /**组路由协议包*/
    downlink->dst = GND_ROUTE_ADDR;
    downlink->src = OBC_ROUTE_ADDR;
    downlink->typ = FILE_DATA;

    ImagePacket_t * packet = (ImagePacket_t *)downlink->dat;

    packet->PacketID = 0;
    do
    {
        /**组图像数据包*/
        packet->PacketSize = (RemainSize < file_info->packet_size) ? RemainSize : file_info->packet_size;

        f_lseek( file, packet->PacketID * file_info->packet_size);

        f_read( file, packet->ImageData, packet->PacketSize,  &byte_read);

        if( byte_read != packet->PacketSize)
        {
        	printf("ERROR: File read error!!\r\n");
			return E_INVALID_PARAM;
        }

        ret = vu_transmitter_send_frame(downlink, packet->PacketSize + IMAGE_DOWNLINK_OVERHEAD, &TxRemainBufSize);

        /**如果传输成功，且通信机成功将消息加入发送缓冲区，发送指针后移 */
        if ((ret == E_NO_ERR) && (TxRemainBufSize != 0xFF))
        {
            RemainSize -= packet->PacketSize;

            packet->PacketID += 1;

            /**若发射机缓冲区已满，则等待5秒钟*/
            if(TxRemainBufSize == 1)
                vTaskDelay(MS_WAIT_TRANS_FREE_BUFF);
        }
        else
            Error++;

        if (RemainSize != 0)
            vTaskDelay(PACK_DOWN_INTERVAL);

     /**若发送完成或者错误次数超过五次，则跳出循环 */
    }while ((RemainSize > 0) && (Error < 5));

    ObcMemFree(file_info);
    ObcMemFree(downlink);

    if (RemainSize == 0)
        return E_NO_ERR;
    else
    {   /**如果错误超过五次，则复位发射机*/
        vu_transmitter_software_reset();
        return E_TRANSMIT_ERROR;
    }
}

/**
 * 图像下行接口，由OBC本地调用
 *
 * @param image_data 图像数据指针
 * @param image_len 图像长度
 * @param start_pack 图像起始包号
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_isis_image_downlink(const void * image_data, uint32_t image_len, uint16_t start_pack)
{
    int ret;
    uint8_t Error = 0, TxRemainBufSize;
    image_data = (uint8_t *)image_data;
    uint32_t RemainSize = image_len;

    route_frame_t *downlink = ObcMemMalloc(I2C_MTU);
    if(downlink == NULL)
        return E_MALLOC_FAIL;

    /**组路由协议包*/
    downlink->dst = GND_ROUTE_ADDR;
    downlink->src = CAM_ROUTE_ADDR;
    downlink->typ = CAM_IMAGE;

    ImagePacket_t * packet = (ImagePacket_t *)downlink->dat;

    packet->PacketID = start_pack;
    do
    {
        /**组图像数据包*/
        packet->PacketSize = (RemainSize < IMAGE_PACK_MAX_SIZE) ? RemainSize : IMAGE_PACK_MAX_SIZE;
        memcpy(packet->ImageData, image_data, IMAGE_PACK_MAX_SIZE);

//        *(uint32_t *)(&downlink->dat[IMAGE_PACK_HEAD_SIZE + packet->PacketSize]) =
//                crc32_memory((uint8_t *)downlink, ROUTE_HEAD_SIZE + IMAGE_PACK_HEAD_SIZE + packet->PacketSize);

        ret = vu_transmitter_send_frame(downlink, packet->PacketSize + IMAGE_DOWNLINK_OVERHEAD, &TxRemainBufSize);

        /**如果传输成功，且通信机成功将消息加入发送缓冲区，发送指针后移 */
        if ((ret == E_NO_ERR) && (TxRemainBufSize != 0xFF))
        {
            RemainSize -= packet->PacketSize;
            image_data += packet->PacketSize;

            packet->PacketID += 1;

            /**若发射机缓冲区已满，则等待5秒钟*/
            if(TxRemainBufSize == 1)
                vTaskDelay(MS_WAIT_TRANS_FREE_BUFF);
        }
        else
            Error++;

        if (RemainSize != 0)
            vTaskDelay(PACK_DOWN_INTERVAL);

     /**若发送完成或者错误次数超过五次，则跳出循环 */
    }while ((RemainSize > 0) && (Error < 5));

    ObcMemFree(downlink);

    if (RemainSize == 0)
        return E_NO_ERR;
    else
    {   /**如果错误超过五次，则复位发射机*/
        vu_transmitter_software_reset();
        return E_TRANSMIT_ERROR;
    }
}


/**
 * ISIS通信机上行轮询任务
 *
 * @param para 任务参数，没有用到
 */
void vu_isis_uplink_task(void *para __attribute__((unused)))
{
    static uint16_t frame_num;

    if(rx_tm_queue == NULL)
            rx_tm_queue = xQueueCreate(1, sizeof( receiving_tm ));

    while(1)
    {
        vTaskDelay(534);
        /**获取接收机缓冲区帧计数*/
        if (vu_receiver_get_frame_num(&frame_num) != E_NO_ERR ||
                vu_receiver_get_frame_num(&frame_num) != E_NO_ERR)
            continue;

        if (frame_num == 0)
            continue;

        rsp_frame *recv_frame = (rsp_frame *)ObcMemMalloc(sizeof(route_packet_t) + MAX_UPLINK_CONTENT_SIZE);
        if (recv_frame == NULL)
            continue;

        if (vu_receiver_get_frame(recv_frame, MAX_UPLINK_CONTENT_SIZE) != E_NO_ERR ||
                vu_receiver_get_frame(recv_frame, MAX_UPLINK_CONTENT_SIZE) != E_NO_ERR)
        {
            ObcMemFree(recv_frame);
            continue;
        }

        /* 若收到的帧数据长度字段不匹配，则视为错帧 */
        if (recv_frame->DateSize == 0 || recv_frame->DateSize > MAX_UPLINK_CONTENT_SIZE)
        {
            ObcMemFree(recv_frame);
            continue;
        }

        /* 给多普勒和信号强度遥测变量赋值 */
        if(rx_tm_queue != NULL)
        {
            receiving_tm rx_tm =
            {
                rx_tm.DopplerOffset = recv_frame->DopplerOffset,
                rx_tm.RSSI = recv_frame->RSSI
            };

            xQueueOverwrite(rx_tm_queue, &rx_tm);
        }

        frame_num = recv_frame->DateSize;

        memmove(&((route_packet_t *)recv_frame)->dst, &recv_frame->Data, frame_num);

        /* 去掉路由头长度 */
        ((route_packet_t *)recv_frame)->len = frame_num - ROUTE_HEAD_SIZE;

        /* 送入路由队列 */
        route_queue_wirte((route_packet_t *)recv_frame, NULL);

        /**通信机接收上行消息计数加1*/
        vu_rx_count++;

        /**成功接收后移除此帧*/
        if (vu_receiver_remove_frame() != E_NO_ERR)
        {
            for (uint8_t repeat_time = 0; repeat_time < 5; repeat_time++)
            {
                if (vu_receiver_remove_frame() == E_NO_ERR)
                    break;

                if (repeat_time == 4)
                {
                    vu_receiver_software_reset();
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                }
            }
        }
    }
}

/**
 * 大数据下行任务
 *
 * @param para 任务传入参数
 */
static void vu_isis_downlink_task(void *para)
{
    downlink_request * request = (downlink_request *)para;

    if(request->tpye == FILE_DATA)
    {
    	vu_isis_file_download( request->file, request->file_name );

    	f_close(request->file);
    	ObcMemFree(request->file);
    	ObcMemFree(request->file_name);
    }
    /**
     * 访问相机接收缓冲区，需要加锁
     */
    else if(request->tpye == CAM_IMAGE)
        vu_isis_image_downlink(request->pdata, request->data_len, request->start_pack);
    /**
     * 下行完毕，解锁
     */

    vTaskDelete(NULL);
}

/**
 * 整个文件下行接口函数
 *
 * @param file 文件句柄指针
 * @param file_name 文件名指针
 * @return E_NO_ERR 正常，任务创建成功
 */
int file_whole_download(FIL *file, char *file_name)
{
	static downlink_request request;

	request.tpye = FILE_DATA;
	request.file = file;
	request.file_name = file_name;

    int ret = xTaskCreate(vu_isis_downlink_task, "FILE", configMINIMAL_STACK_SIZE * 4, &request, tskIDLE_PRIORITY + 3, NULL);

    if (ret != pdPASS)
        return E_OUT_OF_MEM;

    return E_NO_ERR;

}

/**
 * 下行整幅图片接口函数
 *
 * @param pdata 图像数据指针
 * @param len 图像数据字节数
 * @param start_pack 图像起始包号
 * @return E_NO_ERR 正常
 */
int image_whole_download(void *pdata, uint32_t len, uint16_t start_pack)
{
    static downlink_request request;

    request.tpye = CAM_IMAGE;
    request.pdata = pdata;
    request.data_len = len;
    request.start_pack = start_pack;

    int ret = xTaskCreate(vu_isis_downlink_task, "IMG", configMINIMAL_STACK_SIZE * 4, &request, tskIDLE_PRIORITY + 3, NULL);

    if (ret != pdPASS)
        return E_OUT_OF_MEM;

    return E_NO_ERR;
}

/**
 * 获取接收单元接收上一帧时的遥测
 *
 * @param tm 接收缓冲区指针
 * @return pdTRUE为正常，pdFALSE不正常
 */
int vu_isis_get_receiving_tm(receiving_tm *tm)
{
    if (rx_tm_queue == NULL)
        return pdFALSE;
    return xQueuePeek(rx_tm_queue, tm, 0);
}

/**
 * 路由器中下行接口调用，接受一个路由包
 *
 * @param packet 送到路由器的待下行的数据包
 * @return E_NO_ERR（-1）说明传输成功，其他错误类型参见error.h
 */
int vu_isis_router_downlink(route_packet_t *packet)
{
    if (packet == NULL)
        return E_NO_BUFFER;

    if (!device[ISIS_I2C_HANDLE].is_initialised)
    {
        ObcMemFree(packet);
        return E_NO_DEVICE;
    }

    if (packet->len + ROUTE_HEAD_SIZE > ISIS_MTU)
    {
        ObcMemFree(packet);
        return E_INVALID_BUF_SIZE;
    }

    uint8_t rsp;

    int ret = vu_transmitter_send_frame(&packet->dst, packet->len + ROUTE_HEAD_SIZE, &rsp);

    ObcMemFree(packet);

    if (ret == E_NO_ERR && rsp != 0xFF)
        return E_NO_ERR;
    else
        return E_TRANSMIT_ERROR;
}


/**
 * 解理工通信机上行接收任务（测试）
 *
 * @param para 没用
 */
void vu_jlg_uplink_task(void *para __attribute__((unused)))
{
    static uint16_t frame_num;

    while(1)
    {
        vTaskDelay(500);

        /**获取接收机缓冲区帧计数*/
        if (vu_get_frame_num(&frame_num) != E_NO_ERR)
            continue;

        if (frame_num == 0)
            continue;

//        /**若缓冲区不为空，则接收帧到路由器*/
//        if (vu_router_get_frame() != E_NO_ERR)
//            continue;
        rsp_frame *frame = ObcMemMalloc(sizeof(rsp_frame)+/*ISIS_RX_MTU*/249);

        if( vu_get_frame(frame, /*ISIS_RX_MTU*/249) == E_NO_ERR)
        {
//            printf("TM Item\t\t\tRaw Value\tActual value:\r\n");
//            printf("********************************************************\r\n");
//            printf("DopplerOffset:\t\t%u\t\t%.4f Hz\n"
//                   "RSSI:\t\t\t%u\t\t%.4f dBm\n",
//                    frame->DopplerOffset, VU_SDO_Hz(frame->DopplerOffset),
//                    frame->RSSI, VU_RSS_dBm(frame->RSSI)
//                  );
//            printf("\n\n");
//
            printf("Len: %u bytes.\n", frame->DateSize);

            if (frame->DateSize /*!= ISIS_RX_MTU*/ > 249 || frame->DateSize == 0 )
            {
                printf("ERROR: Rx length!!!");
            }
            else
            {

                hex_dump( frame->Data, frame->DateSize );
//                for(uint32_t i=0; i<frame->DateSize; i++)
//                    printf("0x%02x ", frame->Data[i]);
//                printf("\r\n");
            }
            printf("\n\n");
        }

        ObcMemFree(frame);

        /**通信机接收上行消息计数加1*/
        vu_rx_count++;

        /**成功接收后移除此帧*/
        if (vu_remove_frame() != E_NO_ERR)
        {
            for (uint8_t repeat_time = 0; repeat_time < 5; repeat_time++)
            {
                if (vu_remove_frame() == E_NO_ERR)
                    break;

                if (repeat_time == 5)
                {
                    vu_software_reset();
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                }
            }
        }
    }
}

